//  Copyright 2023 Sakai Hibiki
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

#include "prox_mesh/triangulation_node.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/surface/organized_fast_mesh.h"
#include "pcl_conversions/pcl_conversions.h"

namespace prox::mesh
{

using prox_msgs::msg::MeshStamped;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;

Triangulation::Triangulation(const rclcpp::NodeOptions & options) : Node("triangulation", options)
{
  subscription_ = this->create_subscription<PointCloud2>(
    "input/points", rclcpp::SensorDataQoS(), std::bind(&Triangulation::topic_callback, this, _1));
  publisher_ = this->create_publisher<MeshStamped>("~/mesh_stamped", rclcpp::SensorDataQoS());
}

void Triangulation::topic_callback(const PointCloud2::ConstSharedPtr & cloud_msg)
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  const auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*cloud_msg, *cloud);
  assert(cloud->isOrganized());

  // Mesh reconstruction
  std::vector<pcl::Vertices> polygons;
  pcl::OrganizedFastMesh<pcl::PointXYZ> ofm;
  ofm.setInputCloud(cloud);
  ofm.setTriangulationType(ofm.TRIANGLE_ADAPTIVE_CUT);
  ofm.reconstruct(polygons);

  // Convert to message
  MeshStamped mesh_msg;
  mesh_msg.header = cloud_msg->header;
  for (const auto & e : *cloud) {
    geometry_msgs::msg::Point point;
    point.x = e.x;
    point.y = e.y;
    point.z = e.z;
    mesh_msg.mesh.vertices.emplace_back(point);
  }
  for (const auto & e : polygons) {
    shape_msgs::msg::MeshTriangle triangle;
    for (size_t i = 0; i < 3; ++i) {
      triangle.vertex_indices[i] = e.vertices[i];
    }
    mesh_msg.mesh.triangles.emplace_back(triangle);
  }

  publisher_->publish(mesh_msg);
}

}  // namespace prox::mesh
