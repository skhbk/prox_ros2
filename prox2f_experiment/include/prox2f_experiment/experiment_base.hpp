//  Copyright 2024 Sakai Hibiki
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

#include "experiment_base_params.hpp"

#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include "moveit_msgs/action/hybrid_planner.hpp"

namespace prox::experiment
{
using HybridPlannerWrappedResult =
  rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::WrappedResult;
using GripperCommandWrappedResult =
  rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult;

class ExperimentBase
{
protected:
  std::shared_ptr<experiment_base::ParamListener> param_listener_;
  experiment_base::Params params_;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SharedPtr
    hybrid_planner_action_client_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

public:
  explicit ExperimentBase(rclcpp::Node::SharedPtr node);

  const moveit::core::RobotState & getCurrentState() const;

  moveit::core::RobotState getState(const std::string & state_name) const;

  const Eigen::Isometry3d & getTransform(const std::string & link_name) const;

  moveit_msgs::msg::MotionPlanRequest constructMotionPlanRequest() const;

  std::shared_future<HybridPlannerWrappedResult> executeMotionPlanRequest(
    const moveit_msgs::msg::MotionPlanRequest & request);

  std::shared_future<HybridPlannerWrappedResult> moveToState(
    const moveit::core::RobotState & state);

  std::shared_future<HybridPlannerWrappedResult> moveToState(const std::string & state_name);

  std::shared_future<HybridPlannerWrappedResult> moveToPose(
    const std::string & link_name, const geometry_msgs::msg::PoseStamped & pose);

  std::shared_future<GripperCommandWrappedResult> executeGripperCommand(
    const control_msgs::msg::GripperCommand & command);
};

}  // namespace prox::experiment
