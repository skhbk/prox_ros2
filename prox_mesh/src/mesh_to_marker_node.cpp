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

#include "prox_mesh/mesh_to_marker_node.hpp"

namespace prox::mesh
{

using prox_msgs::msg::MeshStamped;
using std::placeholders::_1;
using visualization_msgs::msg::Marker;

MeshToMarker::MeshToMarker(const rclcpp::NodeOptions & options) : Node("mesh_to_marker", options)
{
  subscription_ = this->create_subscription<MeshStamped>(
    "input/mesh_stamped", rclcpp::SensorDataQoS(),
    std::bind(&MeshToMarker::topic_callback, this, _1));
  publisher_ = this->create_publisher<Marker>("mesh_marker", rclcpp::SensorDataQoS());
}

void MeshToMarker::topic_callback(const MeshStamped::ConstSharedPtr & mesh_msg)
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  Marker marker;
  marker.header = mesh_msg->header;

  marker.ns = this->get_namespace();
  marker.id = 0;
  marker.type = Marker::TRIANGLE_LIST;
  marker.action = Marker::ADD;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 0.5;
  for (const auto & e : mesh_msg->mesh.triangles) {
    marker.points.emplace_back(mesh_msg->mesh.vertices.at(e.vertex_indices.at(0)));
    marker.points.emplace_back(mesh_msg->mesh.vertices.at(e.vertex_indices.at(1)));
    marker.points.emplace_back(mesh_msg->mesh.vertices.at(e.vertex_indices.at(2)));
  }

  publisher_->publish(marker);
}

}  // namespace prox::mesh
