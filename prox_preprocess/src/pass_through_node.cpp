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

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

namespace prox
{
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;

class PassThrough : public rclcpp::Node
{
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;

public:
  explicit PassThrough(const rclcpp::NodeOptions & options) : Node("pass_through", options)
  {
    this->declare_parameter<std::string>("field_name", "z");
    this->declare_parameter<double>("limit_min", 0.);
    this->declare_parameter<double>("limit_max", 1.);

    subscription_ = this->create_subscription<PointCloud2>(
      "input/points", rclcpp::SensorDataQoS(), std::bind(&PassThrough::topic_callback, this, _1));
    publisher_ = this->create_publisher<PointCloud2>("points", rclcpp::SensorDataQoS());
  }

  void topic_callback(const PointCloud2::SharedPtr input_msg)
  {
    if (publisher_->get_subscription_count() == 0) {
      return;
    }

    std::string field_name;
    double limit_min, limit_max;
    this->get_parameter("field_name", field_name);
    this->get_parameter("limit_min", limit_min);
    this->get_parameter("limit_max", limit_max);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*input_msg, cloud);
    pcl::Indices index;
    pcl::removeNaNFromPointCloud(cloud, cloud, index);

    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(pcl::make_shared<decltype(cloud)>(cloud));
    filter.setFilterFieldName(field_name);
    filter.setFilterLimits(limit_min, limit_max);
    filter.filter(cloud);

    PointCloud2 output_msg;
    pcl::toROSMsg(cloud, output_msg);
    output_msg.header = input_msg->header;

    publisher_->publish(output_msg);
  }
};
}  // namespace prox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::PassThrough)
