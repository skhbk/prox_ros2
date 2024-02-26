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

#include "prox2f_planning/local_planner/servo.hpp"

#include "moveit/local_planner/feedback_types.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_state/conversions.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace prox::planning
{
using geometry_msgs::msg::PoseStamped;
using std_srvs::srv::Trigger;

bool Servo::initialize(
  const rclcpp::Node::SharedPtr & node,
  const planning_scene_monitor::PlanningSceneMonitorPtr & planning_scene_monitor,
  const std::string & group_name)
{
  node_ = node;

  param_listener_ =
    std::make_shared<local_planner::ParamListener>(node_->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  planning_scene_monitor_ = planning_scene_monitor;
  planning_scene_monitor_->getStateMonitor()->enableCopyDynamics(true);
  group_name_ = group_name;
  joint_group_ = planning_scene_monitor_->getRobotModel()->getJointModelGroup(group_name);

  pids_.resize(joint_group_->getActiveVariableCount());
  for (auto & pid : pids_) {
    pid.initPid(
      params_.pid.p, params_.pid.i, params_.pid.d, params_.pid.i_clamp, -params_.pid.i_clamp);
  }

  double frequency;
  node_->get_parameter("local_planning_frequency", frequency);
  dt_ = 1 / frequency;

  this->reset();

  pose_subscription_ = node_->create_subscription<PoseStamped>(
    "~/pose", rclcpp::SensorDataQoS(), [this](const PoseStamped::SharedPtr msg) {
      if (servo_enabled_) {
        override_goal_ = true;
      }
      pose_msg_ = msg;
    });

  enable_servo_service_ = node_->create_service<Trigger>(
    "~/enable_servo",
    [this](
      const Trigger::Request::ConstSharedPtr /* request */, Trigger::Response::SharedPtr response) {
      servo_enabled_ = true;
      response->success = true;
    });

  disable_servo_service_ = node_->create_service<Trigger>(
    "~/disable_servo",
    [this](
      const Trigger::Request::ConstSharedPtr /* request */, Trigger::Response::SharedPtr response) {
      servo_enabled_ = false;
      response->success = true;
    });

  return true;
}

bool Servo::reset()
{
  servo_enabled_ = false;
  override_goal_ = false;
  lost_target_ = false;
  pose_msg_.reset();
  suspended_time_.reset();

  for (auto & pid : pids_) {
    pid.reset();
  }

  command_.resize(joint_group_->getActiveVariableCount());
  command_.setZero();

  rclcpp::Parameter progress("progress", 0.0);
  node_->set_parameter(progress);

  return true;
};

moveit_msgs::action::LocalPlanner::Feedback Servo::solve(
  const robot_trajectory::RobotTrajectory & local_trajectory,
  const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> /* local_goal */,
  trajectory_msgs::msg::JointTrajectory & local_solution)
{
  moveit_msgs::action::LocalPlanner::Feedback feedback;

  const auto & current_state = this->getCurrentState();

  std::optional<moveit::core::RobotState> goal_state;

  if (!override_goal_ && !local_trajectory.empty()) {
    /* Follow trajectory */
    goal_state = local_trajectory.getFirstWayPoint();

  } else if (servo_enabled_ && pose_msg_ && !lost_target_) {
    /* Servo */
    // Check pose frame
    if (pose_msg_->header.frame_id != params_.servo_frame) {
      RCLCPP_ERROR(
        node_->get_logger(), "Pose command frame is '%s', expected '%s'",
        pose_msg_->header.frame_id.c_str(), params_.servo_frame.c_str());
      feedback.feedback = "unhandled_exception";
    }

    const bool pose_stale = node_->now() - pose_msg_->header.stamp >
                            rclcpp::Duration::from_seconds(params_.pose_command_timeout);

    if (pose_stale) {
      RCLCPP_WARN(node_->get_logger(), "Lost target pose to follow");
      lost_target_ = true;
      pose_msg_.reset();
      feedback.feedback = "unhandled_exception";
    } else {
      Eigen::Isometry3d new_pose;
      tf2::fromMsg(pose_msg_->pose, new_pose);
      const auto target_pose = this->processPose(new_pose);

      try {
        goal_state = this->computeServoState(target_pose);
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(node_->get_logger(), e.what());
        feedback.feedback = "unhandled_exception";
      }
    }
  }

  if (goal_state) {
    suspended_time_.reset();

    goal_state->updateCollisionBodyTransforms();
    if (!this->isStateValid(*goal_state)) {
      *goal_state = current_state;
      feedback.feedback = "collision_ahead";
    }

    // Compute joint velocities from goal state
    auto command = this->computeCommand(*goal_state);

    // Blend with previous command
    {
      assert(current_state.hasVelocities());
      Eigen::VectorXd current_velocities;
      current_state.copyJointGroupVelocities(joint_group_, current_velocities);
      command_ = params_.filter.command * command + (1 - params_.filter.command) * command_;
    }
  } else {
    if (!suspended_time_) {
      suspended_time_ = node_->now();
    } else if ((node_->now() - *suspended_time_).seconds() > params_.suspend_timeout) {
      suspended_time_.reset();
    }
    goal_state = current_state;
  }

  assert(goal_state);

  const bool goal_reached =
    current_state.distance(*goal_state, joint_group_) < params_.goal_position_tolerance;
  const bool robot_stopped = command_.norm() < params_.stopped_velocity_tolerance;

  if (goal_reached && robot_stopped && !suspended_time_) {
    // Stop the robot
    command_.setZero();

    // Report progress to the trajectory operator
    rclcpp::Parameter progress("progress", 1.0);
    node_->set_parameter(progress);
  }

  // Write command
  local_solution.points.clear();
  auto & point = local_solution.points.emplace_back();
  point.velocities = std::vector<double>(command_.data(), command_.data() + command_.size());

  return feedback;
}

const moveit::core::RobotState & Servo::getCurrentState() const
{
  planning_scene_monitor_->updateSceneWithCurrentState();
  const planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
  return locked_scene->getCurrentState();
}

Eigen::VectorXd Servo::computeCommand(const moveit::core::RobotState & goal_state)
{
  const auto & current_state = this->getCurrentState();
  assert(current_state.hasVelocities());

  const auto & active_joints = joint_group_->getActiveJointModels();

  Eigen::VectorXd command;
  command.resize(active_joints.size());

  double scaling_factor = 1.0;

  for (size_t i = 0; i < active_joints.size(); ++i) {
    const auto joint = active_joints.at(i);
    const auto & bounds = joint->getVariableBounds().front();

    if (!bounds.velocity_bounded_ || bounds.max_velocity_ == 0 || bounds.min_velocity_ == 0) {
      throw std::runtime_error(
        "Joint velocity limit is not defined or set to zero: " + joint->getName());
    }

    const auto index = joint->getFirstVariableIndex();
    const auto p_current = current_state.getVariablePosition(index);
    const auto p_goal = goal_state.getVariablePosition(index);
    const auto v_current = current_state.getVariableVelocity(index);

    const auto v = pids_.at(i).computeCommand(
      p_goal - p_current, rclcpp::Duration::from_seconds(dt_).nanoseconds());
    command[i] = v;

    const auto v_limit = v >= 0.0 ? bounds.max_velocity_ : bounds.min_velocity_;
    const auto v_scaling_factor = v == 0.0 ? 1.0 : std::clamp(v_limit / v, 0.0, 1.0);

    const auto a = (v * v_scaling_factor - v_current) / dt_;
    const auto a_limit = a >= 0.0 ? bounds.max_acceleration_ : bounds.min_acceleration_;
    const auto a_scaling_factor = a == 0.0 ? 1.0 : std::clamp(a_limit / a, 0.0, 1.0);

    scaling_factor = std::min(scaling_factor, v_scaling_factor * a_scaling_factor);
  }

  command *= scaling_factor;

  return command;
}

moveit::core::RobotState Servo::computeServoState(const Eigen::Isometry3d & target_pose) const
{
  const auto & current_state = this->getCurrentState();

  // Get IK solver
  const auto ik_solver = joint_group_->getSolverInstance();

  if (!ik_solver) {
    throw std::runtime_error("IK solver not found");
  }

  // Solve IK problem
  std::vector<double> target_positions;
  {
    // Get servo frame
    bool frame_found = true;
    const auto & current_pose = current_state.getFrameTransform(params_.servo_frame, &frame_found);
    if (!frame_found) {
      throw std::runtime_error("Servo frame " + params_.servo_frame + " not found");
    }

    // Transform pose from servo frame to base frame (e.g. 'base_link')
    const auto & base_pose = current_state.getGlobalLinkTransform(ik_solver->getBaseFrame());
    const auto base_to_servo_transform = base_pose.inverse() * current_pose;
    const auto ik_pose = base_to_servo_transform * target_pose;

    std::vector<double> current_positions;
    current_state.copyJointGroupPositions(joint_group_, current_positions);

    moveit_msgs::msg::MoveItErrorCodes error_code;
    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = true;
    if (!ik_solver->searchPositionIK(
          tf2::toMsg(ik_pose), current_positions, dt_, target_positions, error_code, options)) {
      throw std::runtime_error(
        "Could not find valid IK solution, got error code: " + error_code.val);
    }
  }

  moveit::core::RobotState goal_state(current_state.getRobotModel());
  goal_state.setJointGroupPositions(
    joint_group_, Eigen::Map<Eigen::VectorXd>(target_positions.data(), target_positions.size()));
  return goal_state;
}

Eigen::Isometry3d Servo::processPose(const Eigen::Isometry3d & pose) const
{
  Eigen::Vector3d t = pose.translation();
  const auto t_norm = t.norm();
  if (t_norm < params_.dead_zone.linear) {
    // Apply suppression
    t *= t_norm / params_.dead_zone.linear;
  }

  Eigen::Quaterniond r(pose.rotation());
  const auto r_angle = Eigen::AngleAxisd(r).angle();
  if (r_angle < params_.dead_zone.angular) {
    // Apply suppression
    r = Eigen::Quaterniond::Identity().slerp(r_angle / params_.dead_zone.angular, r);
  }

  auto target_pose = Eigen::Isometry3d::Identity();
  target_pose.translate(t);
  target_pose.rotate(r);

  return target_pose;
}

bool Servo::isStateValid(const moveit::core::RobotState & state) const
{
  const planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
  return locked_scene->isStateValid(state, group_name_);
}
}  // namespace prox::planning

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  prox::planning::Servo, moveit::hybrid_planning::LocalConstraintSolverInterface);
