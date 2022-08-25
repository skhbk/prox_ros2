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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

namespace prox
{
using message_filters::sync_policies::ApproximateTime;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;
using std::placeholders::_2;

class ConcatenateClouds : public rclcpp::Node
{
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<PointCloud2> subscriber1_, subscriber2_;
  message_filters::Synchronizer<ApproximateTime<PointCloud2, PointCloud2>> sync_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;

public:
  explicit ConcatenateClouds(const rclcpp::NodeOptions & options)
  : Node("concatenate_clouds", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    sync_(5)
  {
    this->declare_parameter<std::string>("target_frame_id", "world");

    subscriber1_.subscribe(this, "input1/points", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    subscriber2_.subscribe(this, "input2/points", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    sync_.connectInput(subscriber1_, subscriber2_);
    sync_.registerCallback(std::bind(&ConcatenateClouds::topic_callback, this, _1, _2));
    publisher_ = this->create_publisher<PointCloud2>("points", rclcpp::SensorDataQoS());
  }

  void topic_callback(
    const PointCloud2::ConstSharedPtr & input_msg1, const PointCloud2::ConstSharedPtr & input_msg2)
  {
    if (publisher_->get_subscription_count() == 0) {
      return;
    }

    std::string target_frame_id;
    this->get_parameter("target_frame_id", target_frame_id);

    std::array<const PointCloud2, 2> input_msgs{*input_msg1, *input_msg2};
    std::array<PointCloud2, 2> msgs;
    std::array<pcl::PointCloud<pcl::PointXYZ>, 2> clouds;

    for (std::size_t i = 0; i < msgs.size(); ++i) {
      const auto & input_msg = input_msgs.at(i);
      auto & msg = msgs.at(i);
      auto & cloud = clouds.at(i);

      // Get transform
      geometry_msgs::msg::TransformStamped transform;
      try {
        transform = tf_buffer_.lookupTransform(
          target_frame_id, input_msg.header.frame_id, input_msg.header.stamp,
          rclcpp::Duration::from_seconds(.1));
      } catch (tf2::LookupException & e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
      } catch (tf2::ExtrapolationException & e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
      }

      pcl_ros::transformPointCloud(target_frame_id, transform, input_msg, msg);

      pcl::fromROSMsg(msg, cloud);
      pcl::Indices index;
      pcl::removeNaNFromPointCloud(cloud, cloud, index);
    }

    // Concatenate
    const auto output_cloud = clouds.at(0) + clouds.at(1);

    PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    output_msg.header = input_msg1->header;
    output_msg.header.frame_id = target_frame_id;

    publisher_->publish(output_msg);
  }
};
}  // namespace prox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::ConcatenateClouds)
