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

#ifndef PROX_MESH__TRIANGULATION_NODE_HPP_
#define PROX_MESH__TRIANGULATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "prox_msgs/msg/mesh_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace prox::mesh
{

class Triangulation : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<prox_msgs::msg::MeshStamped>::SharedPtr publisher_;

public:
  explicit Triangulation(const rclcpp::NodeOptions & options);

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points_msg);
};

}  // namespace prox::mesh

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::mesh::Triangulation)

#endif  // PROX_MESH__TRIANGULATION_NODE_HPP_
