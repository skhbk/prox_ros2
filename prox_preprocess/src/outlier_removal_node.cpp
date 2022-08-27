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

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace prox
{
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;

class OutlierRemoval : public rclcpp::Node
{
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;

public:
  explicit OutlierRemoval(const rclcpp::NodeOptions & options) : Node("outlier_removal", options)
  {
    this->declare_parameter<double>("radius", .01);
    this->declare_parameter<int64_t>("min_neighbors", 2);

    subscription_ = this->create_subscription<PointCloud2>(
      "input/points", rclcpp::SensorDataQoS(),
      std::bind(&OutlierRemoval::topic_callback, this, _1));
    publisher_ = this->create_publisher<PointCloud2>("points", rclcpp::SensorDataQoS());
  }

  void topic_callback(const PointCloud2::SharedPtr input_msg)
  {
    if (publisher_->get_subscription_count() == 0) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*input_msg, cloud);
    pcl::Indices index;
    pcl::removeNaNFromPointCloud(cloud, cloud, index);

    if (cloud.empty()) {
      return;
    }

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(pcl::make_shared<decltype(cloud)>(cloud));
    filter.setRadiusSearch(this->get_parameter("radius").as_double());
    filter.setMinNeighborsInRadius(this->get_parameter("min_neighbors").as_int());
    filter.filter(cloud);

    PointCloud2 output_msg;
    pcl::toROSMsg(cloud, output_msg);
    output_msg.header = input_msg->header;

    publisher_->publish(output_msg);
  }
};
}  // namespace prox

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(prox::OutlierRemoval)
