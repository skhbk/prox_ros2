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

#pragma once

#include "prox2f_contact_analysis/contact_surface.hpp"

#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace prox
{
namespace contact
{
using PCLCloud = pcl::PointCloud<pcl::PointXYZ>;

class ContactMapping : public rclcpp::Node
{
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  std::unique_ptr<ContactSurface> contact_surface_;

public:
  explicit ContactMapping(const rclcpp::NodeOptions & options);

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg);
  PCLCloud resample_cloud(const PCLCloud & cloud, uint16_t dpi) const;
  std::vector<Kernel::Point_3> pcl_cloud_to_cgal(const PCLCloud & cloud) const;
};
}  // namespace contact
}  // namespace prox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::contact::ContactMapping)
