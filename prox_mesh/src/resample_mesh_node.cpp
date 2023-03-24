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

using prox_msgs::msg::Grid;
using prox_msgs::msg::MeshStamped;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;

ResampleMesh::ResampleMesh(const rclcpp::NodeOptions & options)
: Node("resample_mesh", options), raycaster_(std::make_unique<RaycasterEmbree>())
{
  param_listener_ =
    std::make_shared<resample_mesh::ParamListener>(this->get_node_parameters_interface());
  this->configure_params();

  subscription_ = this->create_subscription<MeshStamped>(
    "input/mesh_stamped", rclcpp::SensorDataQoS(),
    std::bind(&ResampleMesh::topic_callback, this, _1));

  if (params_.publish_cloud) {
    cloud_publisher_ = this->create_publisher<PointCloud2>("~/points", rclcpp::SensorDataQoS());
  }
  if (params_.publish_grid) {
    grid_publisher_ = this->create_publisher<Grid>("~/grid", rclcpp::SensorDataQoS());
  }
}

void ResampleMesh::topic_callback(const MeshStamped::ConstSharedPtr & mesh_msg)
{
  if (param_listener_->is_old(params_)) {
    this->configure_params();
  }

  std::vector<Hit> hits;
  if (!mesh_msg->mesh.triangles.empty()) {
    raycaster_->load_mesh(mesh_msg->mesh);
    hits = raycaster_->raycast(this->get_rays());
    assert(rows_ * cols_ == hits.size());
  }

  if (cloud_publisher_) {
    auto cloud_msg = hits_to_cloud(hits);
    cloud_msg.header = mesh_msg->header;
    cloud_publisher_->publish(cloud_msg);
  }

  if (grid_publisher_) {
    auto grid_msg = hits_to_grid(hits);
    grid_msg.header = mesh_msg->header;
    grid_publisher_->publish(grid_msg);
  }
}

void ResampleMesh::configure_params()
{
  params_ = param_listener_->get_params();

  rows_ = static_cast<size_t>(std::floor(params_.height / params_.pitch));
  cols_ = static_cast<size_t>(std::floor(params_.width / params_.pitch));
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

PointCloud2 ResampleMesh::hits_to_cloud(const std::vector<Hit> & hits) const
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  if (hits.empty()) {
    // Set at least one element
    cloud.emplace_back(NAN, NAN, NAN);
  } else {
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
  }

  PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  return cloud_msg;
}

Grid ResampleMesh::hits_to_grid(const std::vector<Hit> & hits) const
{
  Grid grid_msg;

  grid_msg.width = cols_;
  grid_msg.height = rows_;
  grid_msg.pitch = params_.pitch;

  if (hits.empty()) {
    return grid_msg;
  }

  grid_msg.data.reserve(hits.size());
  for (const auto & hit : hits) {
    if (hit) {
      // Push z value
      grid_msg.data.emplace_back(hit->at(2));
    } else {
      grid_msg.data.emplace_back(NAN);
    }
  }

  return grid_msg;
}

}  // namespace prox::mesh
