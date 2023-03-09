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

#include <string>

#include "pcl/filters/filter.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace prox::preprocess
{
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;

class CloudProcessor : public rclcpp::Node
{
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;

public:
  explicit CloudProcessor(const rclcpp::NodeOptions & options) : Node("cloud_processor", options)
  {
    this->declare_parameter<std::string>("pass_through/field_name", "z");
    this->declare_parameter<double>("pass_through/limit_min", 0.);
    this->declare_parameter<double>("pass_through/limit_max", 1.);
    this->declare_parameter<double>("outlier/radius", .01);
    this->declare_parameter<int64_t>("outlier/min_neighbors", 2);

    subscription_ = this->create_subscription<PointCloud2>(
      "input/points", rclcpp::SensorDataQoS(),
      std::bind(&CloudProcessor::topic_callback, this, _1));
    publisher_ = this->create_publisher<PointCloud2>("points", rclcpp::SensorDataQoS());
  }

  void topic_callback(const PointCloud2::ConstSharedPtr input_msg)
  {
    if (publisher_->get_subscription_count() == 0) {
      return;
    }

    PointCloud2 output_msg;
    output_msg.header = input_msg->header;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*input_msg, cloud);
    assert(cloud.isOrganized());

    const auto cloud_ptr = pcl::make_shared<decltype(cloud)>(cloud);

    // Pass through
    {
      std::string field_name;
      double limit_min, limit_max;
      this->get_parameter("pass_through/field_name", field_name);
      this->get_parameter("pass_through/limit_min", limit_min);
      this->get_parameter("pass_through/limit_max", limit_max);

      pcl::PassThrough<pcl::PointXYZ> filter(true);
      filter.setInputCloud(cloud_ptr);
      filter.setFilterFieldName(field_name);
      filter.setFilterLimits(limit_min, limit_max);
      filter.setKeepOrganized(true);
      decltype(cloud) cloud_out;
      filter.filter(cloud_out);
      cloud = cloud_out;
    }

    // Get indices without NaN points
    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*cloud_ptr, indices);

    // Remove outliers
    {
      double radius;
      int64_t min_neighbors;
      this->get_parameter("outlier/radius", radius);
      this->get_parameter("outlier/min_neighbors", min_neighbors);

      pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter(true);
      filter.setInputCloud(cloud_ptr);
      filter.setIndices(pcl::make_shared<decltype(indices)>(indices));
      filter.setRadiusSearch(radius);
      filter.setMinNeighborsInRadius(min_neighbors);
      filter.setKeepOrganized(true);
      decltype(cloud) cloud_out;
      filter.filter(cloud_out);
      cloud = cloud_out;
    }

    pcl::toROSMsg(cloud, output_msg);

    publisher_->publish(output_msg);
  }
};
}  // namespace prox::preprocess

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::preprocess::CloudProcessor)
