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

#pragma once

#include <memory>

#include "camera_transform_publisher_params.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace prox::camera
{

class CameraTransformPublisher : public rclcpp::Node
{
  std::shared_ptr<camera_transform_publisher::ParamListener> param_listener_;
  camera_transform_publisher::Params params_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  image_transport::CameraSubscriber camera_subscriber_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

public:
  explicit CameraTransformPublisher(const rclcpp::NodeOptions & options);

private:
  void topic_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);
};

}  // namespace prox::camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::camera::CameraTransformPublisher)
