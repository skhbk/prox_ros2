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

#include "prox_mesh/resample_mesh_node.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "prox_mesh/raycaster_embree.hpp"

namespace prox::mesh
{

using prox_msgs::msg::MeshStamped;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;

ResampleMesh::ResampleMesh(const rclcpp::NodeOptions & options)
: Node("resample_mesh", options),
  param_listener_(this->get_node_parameters_interface()),
  params_(param_listener_.get_params()),
  raycaster_(std::make_unique<RaycasterEmbree>())
{
  rows_ = static_cast<size_t>(std::floor(params_.height / params_.pitch));
  cols_ = static_cast<size_t>(std::floor(params_.width / params_.pitch));

  subscription_ = this->create_subscription<MeshStamped>(
    "input/mesh_stamped", rclcpp::SensorDataQoS(),
    std::bind(&ResampleMesh::topic_callback, this, _1));
  publisher_ = this->create_publisher<PointCloud2>("resampled_points", rclcpp::SensorDataQoS());
}

void ResampleMesh::topic_callback(const MeshStamped::ConstSharedPtr & mesh_msg)
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (mesh_msg->mesh.triangles.size() > 0) {
    raycaster_->load_mesh(mesh_msg->mesh);
    const auto hits = raycaster_->raycast(this->get_rays());

    cloud.reserve(hits.size());
    cloud.width = cols_;
    cloud.height = rows_;
    for (const auto & hit : hits) {
      if (hit) {
        cloud.transient_emplace_back(hit->at(0), hit->at(1), hit->at(2));
      } else {
        cloud.transient_emplace_back(NAN, NAN, NAN);
      }
    }
  } else {
    // Set at least one element
    cloud.emplace_back(NAN, NAN, NAN);
  }

  PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header = mesh_msg->header;

  publisher_->publish(cloud_msg);
}

std::vector<Ray> ResampleMesh::get_rays() const
{
  std::vector<Ray> rays;

  const std::array<double, 2> center{params_.width / 2, params_.height / 2};

  for (size_t i = 0; i < rows_; ++i) {
    for (size_t j = 0; j < cols_; ++j) {
      Vertex origin{
        static_cast<float>(j * params_.pitch - center[0]),
        static_cast<float>(i * params_.pitch - center[1]), 0};
      Vertex direction{0, 0, 1};
      rays.push_back({std::move(origin), std::move(direction)});
    }
  }

  assert(rows_ * cols_ == rays.size());

  return rays;
}

}  // namespace prox::mesh
