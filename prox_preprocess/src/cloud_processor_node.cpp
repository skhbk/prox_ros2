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

#include "cloud_processor_params.hpp"

namespace prox::preprocess
{
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;

class CloudProcessor : public rclcpp::Node
{
  std::shared_ptr<cloud_processor::ParamListener> param_listener_;
  cloud_processor::Params params_;

  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;

public:
  explicit CloudProcessor(const rclcpp::NodeOptions & options) : Node("cloud_processor", options)
  {
    param_listener_ =
      std::make_shared<cloud_processor::ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

    subscription_ = this->create_subscription<PointCloud2>(
      "input/points", rclcpp::SensorDataQoS(),
      std::bind(&CloudProcessor::topic_callback, this, _1));
    publisher_ = this->create_publisher<PointCloud2>("~/points", rclcpp::SensorDataQoS());
  }

  void topic_callback(const PointCloud2::ConstSharedPtr input_msg)
  {
    if (param_listener_->is_old(params_)) {
      params_ = param_listener_->get_params();
    }

    if (publisher_->get_subscription_count() == 0) {
      return;
    }

    PointCloud2 output_msg;
    output_msg.header = input_msg->header;

    const auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*input_msg, *cloud);
    assert(cloud->isOrganized());

    // Pass through
    {
      pcl::PassThrough<pcl::PointXYZ> filter(true);
      filter.setInputCloud(cloud);
      filter.setFilterFieldName(params_.pass_through.field_name);
      filter.setFilterLimits(params_.pass_through.bounds[0], params_.pass_through.bounds[1]);
      filter.setKeepOrganized(true);
      filter.filter(*cloud);
    }

    // Get indices without NaN points
    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*cloud, indices);

    // Remove outliers
    {
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter(true);
      filter.setInputCloud(cloud);
      filter.setIndices(pcl::make_shared<decltype(indices)>(indices));
      filter.setRadiusSearch(params_.outlier_removal.radius);
      filter.setMinNeighborsInRadius(params_.outlier_removal.min_neighbors);
      filter.setKeepOrganized(true);
      filter.filter(*cloud);
    }

    pcl::toROSMsg(*cloud, output_msg);

    publisher_->publish(output_msg);
  }
};
}  // namespace prox::preprocess

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::preprocess::CloudProcessor)
