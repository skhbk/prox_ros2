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

#include "prox2f_planning/global_planner/move_group_interface.hpp"

#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_state/conversions.h"

namespace prox::planning
{
bool MoveGroupInterface::initialize(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;

  return true;
}

bool MoveGroupInterface::reset() noexcept { return true; }

moveit_msgs::msg::MotionPlanResponse MoveGroupInterface::plan(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>>
    global_goal_handle)
{
  moveit_msgs::msg::MotionPlanResponse response;

  if ((global_goal_handle->get_goal())->motion_sequence.items.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Global planner received motion sequence request with no items. At least one is needed.");
    response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return response;
  }

  const auto request = global_goal_handle->get_goal()->motion_sequence.items.front().req;

  move_group_ =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, request.group_name);
  move_group_->setStartState(request.start_state);

  const auto trajectory = [&] {
    const auto & goal_constraints = request.goal_constraints;

    if (!goal_constraints.empty()) {
      const auto & c = goal_constraints.front();

      if (!c.joint_constraints.empty()) {
        // Use joint space planning
        return this->computeJointSpacePlan(c.joint_constraints, response.error_code);
      }

      if (!c.position_constraints.empty() && !c.orientation_constraints.empty()) {
        // Use cartesian planning
        return this->computeCartesianPlan(goal_constraints, response.error_code);
      }
    }

    RCLCPP_WARN(node_->get_logger(), "Goal constraints is empty");
    response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return moveit_msgs::msg::RobotTrajectory();
  }();

  response.group_name = request.group_name;
  response.trajectory_start = request.start_state;
  response.trajectory = trajectory;

  return response;
}

moveit_msgs::msg::RobotTrajectory MoveGroupInterface::computeCartesianPlan(
  const std::vector<moveit_msgs::msg::Constraints> & constraints,
  moveit_msgs::msg::MoveItErrorCodes & error_code)
{
  std::optional<std::string> link_name, reference_frame;

  // Extract waypoints from goal constraints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  for (const auto & c : constraints) {
    geometry_msgs::msg::Pose pose;

    // Position
    const auto & pc = c.position_constraints.front();
    pose.position = pc.constraint_region.primitive_poses.front().position;

    // Orientation
    const auto & oc = c.orientation_constraints.front();
    pose.orientation = oc.orientation;

    if (!link_name) {
      link_name = pc.link_name;
    }

    if (!reference_frame) {
      reference_frame = pc.header.frame_id;
    }

    waypoints.emplace_back(pose);
  }

  move_group_->setEndEffectorLink(link_name.value());
  move_group_->setPoseReferenceFrame(reference_frame.value());

  moveit_msgs::msg::RobotTrajectory trajectory;
  move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory, true, &error_code);

  return trajectory;
}

moveit_msgs::msg::RobotTrajectory MoveGroupInterface::computeJointSpacePlan(
  const std::vector<moveit_msgs::msg::JointConstraint> & joint_constraints,
  moveit_msgs::msg::MoveItErrorCodes & error_code)
{
  for (const auto & e : joint_constraints) {
    move_group_->setJointValueTarget(e.joint_name, e.position);
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  error_code = move_group_->plan(plan);

  return plan.trajectory_;
}
}  // namespace prox::planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  prox::planning::MoveGroupInterface, moveit::hybrid_planning::GlobalPlannerInterface);
