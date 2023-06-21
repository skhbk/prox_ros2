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

#ifndef PROX2F_ATTRACTION__VIRTUAL_WRENCH_NODE_HPP_
#define PROX2F_ATTRACTION__VIRTUAL_WRENCH_NODE_HPP_

#include <memory>
#include <vector>

#include "virtual_wrench_params.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace prox::attraction
{

class VirtualWrench : public rclcpp::Node
{
  std::shared_ptr<virtual_wrench::ParamListener> param_listener_;
  virtual_wrench::Params params_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subscriber1_, subscriber2_;
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>
    sync_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;

public:
  explicit VirtualWrench(const rclcpp::NodeOptions & options);

private:
  void topic_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & normals_msg1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & normals_msg2);
};

}  // namespace prox::attraction

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::attraction::VirtualWrench)

#endif  // PROX2F_ATTRACTION__VIRTUAL_WRENCH_NODE_HPP_
