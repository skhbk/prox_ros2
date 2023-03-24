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

#ifndef PROX_MESH__RESAMPLE_MESH_NODE_HPP_
#define PROX_MESH__RESAMPLE_MESH_NODE_HPP_

#include <memory>
#include <vector>

#include "prox_mesh/raycaster.hpp"
#include "resample_mesh_params.hpp"

#include "rclcpp/rclcpp.hpp"

#include "prox_msgs/msg/grid.hpp"
#include "prox_msgs/msg/mesh_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace prox::mesh
{

using Vertex = std::array<float, 3>;
using Polygon = std::array<uint32_t, 3>;

class ResampleMesh : public rclcpp::Node
{
  resample_mesh::ParamListener param_listener_;
  resample_mesh::Params params_;

  rclcpp::Subscription<prox_msgs::msg::MeshStamped>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<prox_msgs::msg::Grid>::SharedPtr grid_publisher_;

  std::unique_ptr<Raycaster> raycaster_;
  size_t rows_, cols_;

public:
  explicit ResampleMesh(const rclcpp::NodeOptions & options);

private:
  void topic_callback(const prox_msgs::msg::MeshStamped::ConstSharedPtr & mesh_msg);
  std::vector<Ray> get_rays() const;
  sensor_msgs::msg::PointCloud2 hits_to_cloud(const std::vector<Hit> & hits) const;
  prox_msgs::msg::Grid hits_to_grid(const std::vector<Hit> & hits) const;
};

}  // namespace prox::mesh

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::mesh::ResampleMesh)

#endif  // PROX_MESH__RESAMPLE_MESH_NODE_HPP_
