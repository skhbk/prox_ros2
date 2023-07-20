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

#include "prox_mesh/normals_to_marker_node.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

namespace prox::mesh
{

using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;
using visualization_msgs::msg::Marker;

NormalsToMarker::NormalsToMarker(const rclcpp::NodeOptions & options)
: Node("normals_to_marker", options)
{
  subscription_ = this->create_subscription<PointCloud2>(
    "input/normals", rclcpp::SensorDataQoS(),
    std::bind(&NormalsToMarker::topic_callback, this, _1));
  publisher_ = this->create_publisher<Marker>("~/marker", rclcpp::SensorDataQoS());
}

void NormalsToMarker::topic_callback(const PointCloud2::ConstSharedPtr & normals_msg)
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  pcl::PointCloud<pcl::PointNormal> normals;
  pcl::fromROSMsg(*normals_msg, normals);

  Marker marker;
  marker.header = normals_msg->header;

  marker.ns = this->get_namespace();
  marker.id = 1;
  marker.type = Marker::LINE_LIST;

  if (normals.empty()) {
    marker.action = Marker::DELETE;
  } else {
    marker.action = Marker::ADD;
    marker.scale.x = 0.0004;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;
    marker.color.a = 1;

    for (const auto & e : normals) {
      if (!std::isfinite(e.x) || !std::isfinite(e.normal_x)) {
        continue;
      }

      constexpr float normal_scale = 0.007;

      geometry_msgs::msg::Point point1, point2;
      point1.x = e.x;
      point1.y = e.y;
      point1.z = e.z;
      point2.x = e.x + e.normal_x * normal_scale;
      point2.y = e.y + e.normal_y * normal_scale;
      point2.z = e.z + e.normal_z * normal_scale;
      marker.points.emplace_back(point1);
      marker.points.emplace_back(point2);
    }
  }

  publisher_->publish(marker);
}

}  // namespace prox::mesh
