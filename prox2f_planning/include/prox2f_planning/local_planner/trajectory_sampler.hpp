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

#include "moveit/local_planner/trajectory_operator_interface.h"

namespace prox::planning
{
class TrajectorySampler : public moveit::hybrid_planning::TrajectoryOperatorInterface
{
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;

  std::size_t target_waypoint_index_;
  moveit::core::RobotStatePtr previous_waypoint_;

public:
  TrajectorySampler() = default;
  ~TrajectorySampler() override = default;

  bool initialize(
    const rclcpp::Node::SharedPtr & node, const moveit::core::RobotModelConstPtr & robot_model,
    const std::string & group_name) override;

  moveit_msgs::action::LocalPlanner::Feedback addTrajectorySegment(
    const robot_trajectory::RobotTrajectory & new_trajectory) override;

  moveit_msgs::action::LocalPlanner::Feedback getLocalTrajectory(
    const moveit::core::RobotState & current_state,
    robot_trajectory::RobotTrajectory & local_trajectory) override;

  double getTrajectoryProgress(const moveit::core::RobotState & current_state) override;

  bool reset() override;

private:
  double computeSegmentProgress(
    const moveit::core::RobotState & start_state, const moveit::core::RobotState & current_state,
    const moveit::core::RobotState & end_state) const;
};
}  // namespace prox::planning
