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
#include <vector>

#include "servo_params.hpp"

#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace prox::pregrasp
{

class Servo
{
  std::shared_ptr<servo::ParamListener> param_listener_;
  servo::Params params_;

  const rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  geometry_msgs::msg::PoseStamped::SharedPtr pose_msg_;
  Eigen::Isometry3d target_pose_;

public:
  explicit Servo(const rclcpp::NodeOptions & options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

private:
  void timer_callback();
  bool check_collision(const std::vector<double> & joint_positions) const;
};

}  // namespace prox::pregrasp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::pregrasp::Servo)
