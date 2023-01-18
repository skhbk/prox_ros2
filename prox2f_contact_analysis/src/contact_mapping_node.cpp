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

#include "prox2f_contact_analysis/contact_mapping_node.hpp"

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
using namespace std::chrono_literals;

ContactMapping::ContactMapping(const rclcpp::NodeOptions & options)
: Node("contact_mapping", options)
{
  this->declare_parameter<float>("penetration", .003);
  this->declare_parameter<float>("margin", .002);

  subscription_ = this->create_subscription<PointCloud2>(
    "input/points", rclcpp::SensorDataQoS(), std::bind(&ContactMapping::topic_callback, this, _1));
  publisher_ = this->create_publisher<PointCloud2>("points", rclcpp::SensorDataQoS());
}

void ContactMapping::topic_callback(const PointCloud2::SharedPtr input_msg)
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud, contact_cloud;
  pcl::fromROSMsg(*input_msg, cloud);
  contact_cloud.header = cloud.header;

  if (!cloud.empty()) {
    float penetration, margin;
    this->get_parameter("penetration", penetration);
    this->get_parameter("margin", margin);

    // If min(z) < margin, update the offset
    float offset = margin;
    for (const auto & e : cloud) {
      if (!std::isnan(e.z) && e.z < offset) {
        offset = e.z;
      }
    }

    for (const auto & e : cloud) {
      const float z = e.z - offset;
      if (!std::isnan(z) && z <= penetration) {
        contact_cloud.emplace_back(e.x, e.y, z - penetration);
      }
    }
  }

  PointCloud2 output_msg;
  pcl::toROSMsg(contact_cloud, output_msg);
  publisher_->publish(output_msg);
}

}  // namespace contact
}  // namespace prox
