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

#include <string>
#include <vector>

#include "local_planner_params.hpp"

#include "control_toolbox/pid.hpp"
#include "moveit/local_planner/local_constraint_solver_interface.h"
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/trigger.hpp"

namespace prox::planning
{
class Servo : public moveit::hybrid_planning::LocalConstraintSolverInterface
{
  std::shared_ptr<local_planner::ParamListener> param_listener_;
  local_planner::Params params_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_servo_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_servo_service_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  const moveit::core::JointModelGroup * joint_group_;
  std::string group_name_;

  double dt_;
  bool servo_enabled_;
  bool override_goal_;
  bool lost_target_;
  geometry_msgs::msg::PoseStamped::SharedPtr pose_msg_;
  std::vector<control_toolbox::Pid> pids_;
  Eigen::VectorXd command_;
  std::optional<rclcpp::Time> suspended_time_;

public:
  Servo() = default;
  ~Servo() override = default;
  bool initialize(
    const rclcpp::Node::SharedPtr & node,
    const planning_scene_monitor::PlanningSceneMonitorPtr & planning_scene_monitor,
    const std::string & group_name) override;
  bool reset() override;

  moveit_msgs::action::LocalPlanner::Feedback solve(
    const robot_trajectory::RobotTrajectory & local_trajectory,
    const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
    trajectory_msgs::msg::JointTrajectory & local_solution) override;

private:
  const moveit::core::RobotState & getCurrentState() const;
  Eigen::VectorXd computeCommand(const moveit::core::RobotState & goal_state);
  moveit::core::RobotState computeServoState(const Eigen::Isometry3d & target_pose) const;
  Eigen::Isometry3d processPose(const Eigen::Isometry3d & pose) const;
  bool isStateValid(const moveit::core::RobotState & state) const;
};
}  // namespace prox::planning
