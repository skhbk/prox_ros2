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

#include "moveit/global_planner/global_planner_interface.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

namespace prox::planning
{
class MoveGroupInterface : public moveit::hybrid_planning::GlobalPlannerInterface
{
  rclcpp::Node::SharedPtr node_;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_;

public:
  MoveGroupInterface() = default;
  ~MoveGroupInterface() override = default;
  bool initialize(const rclcpp::Node::SharedPtr & node) override;
  bool reset() noexcept override;
  moveit_msgs::msg::MotionPlanResponse plan(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>>
      global_goal_handle) override;

private:
  moveit_msgs::msg::RobotTrajectory computeCartesianPlan(
    const std::vector<moveit_msgs::msg::Constraints> & constraints,
    moveit_msgs::msg::MoveItErrorCodes & error_code);
  moveit_msgs::msg::RobotTrajectory computeJointSpacePlan(
    const std::vector<moveit_msgs::msg::JointConstraint> & joint_constraints,
    moveit_msgs::msg::MoveItErrorCodes & error_code);
};
}  // namespace prox::planning
