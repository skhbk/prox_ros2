//  Copyright 2022 Sakai Hibiki
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "prox2f_contact_analysis/resample_cloud_node.hpp"
#include "prox2f_contact_analysis/robotiq_2f_85_fingertip.hpp"

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Surface_mesh.h>
#include <pcl/filters/filter_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <vector>

namespace prox
{
namespace contact
{
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;
using Mesh = CGAL::Surface_mesh<Kernel::Point_3>;

static std::vector<Kernel::Point_3> pcl_cloud_to_cgal(
  const PCLCloud & cloud, const pcl::Indices & indices)
{
  std::vector<Kernel::Point_3> points;
  points.reserve(indices.size());

  for (const auto & index : indices) {
    const auto & point = cloud.at(index);
    points.emplace_back(point.x, point.y, point.z);
  }

  return points;
}

static Mesh reconstruct_mesh(const PCLCloud & cloud)
{
  assert(cloud.isOrganized());

  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(cloud, indices);
  const auto points = pcl_cloud_to_cgal(cloud, indices);

  Mesh mesh;
  if (points.size() < 5) {
    // Return empty mesh
    return mesh;
  }

  // Surface reconstruction
  using Facet = std::array<std::size_t, 3>;
  std::vector<Facet> facets;
  CGAL::advancing_front_surface_reconstruction(
    points.begin(), points.end(), std::back_inserter(facets));

  // Convert to mesh
  CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, facets, mesh);
  // CGAL::draw(mesh);

  return mesh;
}

ResampleCloud::ResampleCloud(const rclcpp::NodeOptions & options)
: Node("resample_cloud", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  contact_surface_(std::make_unique<Robotiq2F85Fingertip>())
{
  this->declare_parameter<std::string>("surface_frame_id", "world");

  subscription_ = this->create_subscription<PointCloud2>(
    "input/points", rclcpp::SensorDataQoS(), std::bind(&ResampleCloud::topic_callback, this, _1));
  publisher_ = this->create_publisher<PointCloud2>("points", rclcpp::SensorDataQoS());
}

void ResampleCloud::topic_callback(const PointCloud2::SharedPtr input_msg)
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  std::string surface_frame_id;
  this->get_parameter("surface_frame_id", surface_frame_id);

  // Get transform
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform =
      tf_buffer_.lookupTransform(surface_frame_id, input_msg->header.frame_id, rclcpp::Time(0));
  } catch (tf2::LookupException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  } catch (tf2::ExtrapolationException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }

  PointCloud2 msg;
  pcl_ros::transformPointCloud(surface_frame_id, transform, *input_msg, msg);
  PCLCloud cloud;
  pcl::fromROSMsg(msg, cloud);

  const auto resampled_cloud = this->resample_cloud(cloud, 20);

  PointCloud2 output_msg;
  pcl::toROSMsg(resampled_cloud, output_msg);
  publisher_->publish(output_msg);
}

PCLCloud ResampleCloud::resample_cloud(const PCLCloud & cloud, uint16_t dpi) const
{
  std::string surface_frame_id;
  this->get_parameter("surface_frame_id", surface_frame_id);
  assert(surface_frame_id == cloud.header.frame_id);

  // Surface reconstruction
  const auto mesh = reconstruct_mesh(cloud);

  if (mesh.is_empty()) {
    // Return empty point cloud
    PCLCloud empty_cloud;
    empty_cloud.header = cloud.header;
    return empty_cloud;
  }

  // AABB tree
  using AABBPrimitive = CGAL::AABB_face_graph_triangle_primitive<Mesh>;
  using AABBTraits = CGAL::AABB_traits<Kernel, AABBPrimitive>;
  using AABBTree = CGAL::AABB_tree<AABBTraits>;
  AABBTree tree(CGAL::faces(mesh).first, CGAL::faces(mesh).second, mesh);

  // Ray intersection
  PCLCloud resampled_cloud;
  resampled_cloud.header = cloud.header;
  const auto rays = contact_surface_->get_organized_rays(dpi);
  const std::size_t height = rays.size();
  const std::size_t width = rays.front().size();
  resampled_cloud.reserve(height * width);
  for (const auto & row : rays) {
    for (const auto & ray : row) {
      const auto intersection = tree.first_intersection(ray);
      pcl::PointXYZ point;
      if (!intersection) {
        point = {NAN, NAN, NAN};
      } else {
        const auto point_cgal = boost::get<Kernel::Point_3>(&(intersection->first));
        assert(point_cgal);
        point = {
          static_cast<float>(CGAL::to_double(point_cgal->x())),
          static_cast<float>(CGAL::to_double(point_cgal->y())),
          static_cast<float>(CGAL::to_double(point_cgal->z()))};
      }
      resampled_cloud.transient_push_back(point);
    }
  }
  resampled_cloud.resize(width, height);
  assert(resampled_cloud.isOrganized());

  return resampled_cloud;
}

}  // namespace contact
}  // namespace prox