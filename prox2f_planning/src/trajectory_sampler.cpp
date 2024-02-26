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

#include "prox2f_planning/local_planner/trajectory_sampler.hpp"

namespace prox::planning
{
bool TrajectorySampler::initialize(
  const rclcpp::Node::SharedPtr & node, const moveit::core::RobotModelConstPtr & robot_model,
  const std::string & group_name)
{
  node_ = node;

  robot_model_ = robot_model;
  reference_trajectory_ =
    std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group_name);
  group_ = group_name;

  this->reset();

  return true;
}

moveit_msgs::action::LocalPlanner::Feedback TrajectorySampler::addTrajectorySegment(
  const robot_trajectory::RobotTrajectory & new_trajectory)
{
  this->reset();

  if (new_trajectory.size() < 2) {
    return moveit_msgs::action::LocalPlanner::Feedback();
  }

  const auto joint_group = robot_model_->getJointModelGroup(group_);
  double sample_distance = 0.1;

  reference_trajectory_->addSuffixWayPoint(new_trajectory.getFirstWayPoint(), 0.0);

  for (size_t i = 1; i < new_trajectory.size(); ++i) {
    const auto & previous_waypoint = reference_trajectory_->getLastWayPoint();
    const auto & next_waypoint = new_trajectory.getWayPoint(i);
    const auto distance = previous_waypoint.distance(next_waypoint, joint_group);

    if (distance > sample_distance) {
      // Interpolate
      const auto n_segment = std::floor(distance / sample_distance);
      const auto fraction = 1 / n_segment;
      for (size_t j = 1; j < n_segment; ++j) {
        moveit::core::RobotState interpolation(robot_model_);
        previous_waypoint.interpolate(next_waypoint, j * fraction, interpolation, joint_group);
        reference_trajectory_->addSuffixWayPoint(interpolation, 0.0);
      }
    } else if (i != new_trajectory.size() - 1) {
      // Skip
      continue;
    }

    reference_trajectory_->addSuffixWayPoint(next_waypoint, 0.0);
  }

  return moveit_msgs::action::LocalPlanner::Feedback();
}

bool TrajectorySampler::reset()
{
  target_waypoint_index_ = 1;
  reference_trajectory_->clear();

  return true;
}

moveit_msgs::action::LocalPlanner::Feedback TrajectorySampler::getLocalTrajectory(
  const moveit::core::RobotState & current_state,
  robot_trajectory::RobotTrajectory & local_trajectory)
{
  local_trajectory.clear();

  if (reference_trajectory_->empty()) {
    return moveit_msgs::action::LocalPlanner::Feedback();
  }

  if (!previous_waypoint_) {
    previous_waypoint_ = std::make_shared<moveit::core::RobotState>(current_state);
  }

  const auto & target_waypoint = reference_trajectory_->getWayPoint(target_waypoint_index_);

  if (target_waypoint_index_ == reference_trajectory_->size() - 1) {
    // This is the last waypoint
    local_trajectory.addSuffixWayPoint(target_waypoint, 0.0);

    return moveit_msgs::action::LocalPlanner::Feedback();
  }

  moveit::core::RobotState target_state(robot_model_);

  // Blend with next waypoint
  const auto current_progress =
    this->computeSegmentProgress(*previous_waypoint_, current_state, target_waypoint);
  const auto & next_waypoint = reference_trajectory_->getWayPoint(target_waypoint_index_ + 1);
  target_waypoint.interpolate(next_waypoint, current_progress, target_state);

  local_trajectory.addSuffixWayPoint(target_state, 0.0);

  const auto next_progress =
    this->computeSegmentProgress(target_waypoint, current_state, next_waypoint);

  // Check if target state is passed
  if (next_progress > 0.0 || current_progress > 0.95) {
    // Update target waypoint
    ++target_waypoint_index_;
    *previous_waypoint_ = current_state;
  }

  return moveit_msgs::action::LocalPlanner::Feedback();
}

double TrajectorySampler::getTrajectoryProgress(
  const moveit::core::RobotState & /* current_state */)
{
  double progress = 0.0;

  // Get progress from the local constraint solver
  node_->get_parameter("progress", progress);

  return progress;
}

double TrajectorySampler::computeSegmentProgress(
  const moveit::core::RobotState & start_state, const moveit::core::RobotState & current_state,
  const moveit::core::RobotState & end_state) const
{
  const auto joint_group = robot_model_->getJointModelGroup(group_);

  const auto segment_length = start_state.distance(end_state, joint_group);
  const auto remaining_distance = current_state.distance(end_state, joint_group);
  const auto progress = std::clamp(1.0 - (remaining_distance / segment_length), 0.0, 1.0);

  return progress;
}
}  // namespace prox::planning

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  prox::planning::TrajectorySampler, moveit::hybrid_planning::TrajectoryOperatorInterface);
