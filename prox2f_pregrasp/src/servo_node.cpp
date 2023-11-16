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

#include "prox2f_pregrasp/servo_node.hpp"

#include "tf2_eigen/tf2_eigen.hpp"

using geometry_msgs::msg::PoseStamped;
using trajectory_msgs::msg::JointTrajectory;

namespace prox::pregrasp
{

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Servo::get_node_base_interface()
{
  return node_->get_node_base_interface();
}

Servo::Servo(const rclcpp::NodeOptions & options)
: node_{std::make_shared<rclcpp::Node>("servo", options)},
  target_pose_{Eigen::Isometry3d::Identity()}
{
  param_listener_ = std::make_shared<servo::ParamListener>(node_->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    node_, "robot_description", "planning_scene_monitor");
  planning_scene_monitor_->startStateMonitor(params_.joint_topic);
  planning_scene_monitor_->startSceneMonitor(params_.monitored_planning_scene_topic);
  planning_scene_monitor_->setPlanningScenePublishingFrequency(25);
  planning_scene_monitor_->getStateMonitor()->enableCopyDynamics(true);
  planning_scene_monitor_->startPublishingPlanningScene(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
    std::string(node_->get_fully_qualified_name()) + "/publish_planning_scene");

  if (!planning_scene_monitor_->getStateMonitor()->waitForCompleteState(
        params_.move_group_name, 5.0)) {
    RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for current state");
    std::exit(EXIT_FAILURE);
  }

  if (params_.is_primary_planning_scene_monitor) {
    planning_scene_monitor_->providePlanningSceneService();
  } else {
    planning_scene_monitor_->requestPlanningSceneState();
  }

  pose_subscription_ = node_->create_subscription<PoseStamped>(
    "input/pose", rclcpp::SensorDataQoS(),
    [this](const PoseStamped::SharedPtr msg) { pose_msg_ = msg; });

  trajectory_publisher_ =
    node_->create_publisher<JointTrajectory>("~/joint_trajectory", rclcpp::SystemDefaultsQoS());

  timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1 / params_.publish_frequency),
    std::bind(&Servo::timer_callback, this));
}

void Servo::timer_callback()
{
  const auto & robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  const auto joint_model_group = robot_state->getJointModelGroup(params_.move_group_name);

  // Get current joint positions
  std::vector<double> current_joint_positions, current_joint_velocities;
  robot_state->copyJointGroupPositions(joint_model_group, current_joint_positions);
  robot_state->copyJointGroupVelocities(joint_model_group, current_joint_velocities);

  const auto & joint_names = joint_model_group->getActiveJointModelNames();
  const auto n_joints = joint_names.size();

  // Command not yet received
  if (!pose_msg_) {
    return;
  }

  {
    // Get new pose in matrix form
    Eigen::Isometry3d new_pose;
    tf2::fromMsg(pose_msg_->pose, new_pose);

    Eigen::Vector3d new_translation = new_pose.translation();
    const auto new_translation_norm = new_translation.norm();
    if (new_translation_norm < params_.dead_zone.linear) {
      // Apply suppression
      new_translation *= new_translation_norm / params_.dead_zone.linear;
    }
    const Eigen::Vector3d last_translation = target_pose_.translation();
    // Apply exponential filter
    const auto target_translation = params_.filter_coefficient * new_translation +
                                    (1 - params_.filter_coefficient) * last_translation;

    auto new_rotation = Eigen::Quaterniond(new_pose.rotation());
    const auto new_rotation_angle = Eigen::AngleAxisd(new_rotation).angle();
    if (new_rotation_angle < params_.dead_zone.angular) {
      // Apply suppression
      new_rotation = Eigen::Quaterniond::Identity().slerp(
        new_rotation_angle / params_.dead_zone.angular, new_rotation);
    }
    const auto last_rotation = Eigen::Quaterniond(target_pose_.rotation());
    // Apply exponential filter
    const auto target_rotation = last_rotation.slerp(params_.filter_coefficient, new_rotation);

    target_pose_.setIdentity();
    target_pose_.translate(target_translation);
    target_pose_.rotate(target_rotation);
  }

  // Get IK solver
  const auto ik_solver = joint_model_group->getSolverInstance();

  if (!ik_solver) {
    RCLCPP_ERROR(node_->get_logger(), "IK solver not found");
  }

  const auto & tip_frame = ik_solver->getTipFrame();
  if (pose_msg_->header.frame_id != tip_frame) {
    RCLCPP_ERROR(
      node_->get_logger(), "Command frame is '%s', expected '%s'",
      pose_msg_->header.frame_id.c_str(), tip_frame.c_str());
    pose_msg_.reset();
    return;
  }

  // Solve IK problem
  std::vector<double> target_joint_positions;
  {
    // Transform pose from tip frame (e.g. 'tcp') to base frame (e.g. 'base_link')
    const auto & base_pose = robot_state->getGlobalLinkTransform(ik_solver->getBaseFrame());
    const auto & tip_pose = robot_state->getGlobalLinkTransform(ik_solver->getTipFrame());
    const auto base_to_tip_transform = base_pose.inverse() * tip_pose;
    const auto ik_pose = base_to_tip_transform * target_pose_;

    moveit_msgs::msg::MoveItErrorCodes error_code;
    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = true;
    if (!ik_solver->searchPositionIK(
          tf2::toMsg(ik_pose), current_joint_positions, 1 / params_.publish_frequency,
          target_joint_positions, error_code, options)) {
      RCLCPP_WARN(
        node_->get_logger(), "Could not find valid IK solution, got error code: %i",
        error_code.val);
    }
  }

  // Check Collision
  if (this->check_collision(target_joint_positions)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000 /* ms */, "Collision predicted");
    return;
  }

  // Compute trajectory goal time
  double goal_time;
  {
    std::vector<double> goal_times;
    goal_times.reserve(2 /* least times */ + n_joints);

    // Least times
    goal_times.emplace_back(1. / params_.publish_frequency);
    goal_times.emplace_back(params_.least_goal_time);

    // Based on joint displacement
    const auto & joint_bounds = joint_model_group->getActiveJointModelsBounds();

    for (size_t i = 0; i < n_joints; ++i) {
      const auto & joint_bound = joint_bounds.at(i)->front();

      if (
        !joint_bound.velocity_bounded_ || joint_bound.min_velocity_ == 0 ||
        joint_bound.max_velocity_ == 0) {
        RCLCPP_ERROR(
          node_->get_logger(), "Joint velocity limit is not defined or set to zero: %s",
          joint_names.at(i).c_str());
        timer_->cancel();
        return;
      }

      if (
        !joint_bound.acceleration_bounded_ || joint_bound.min_acceleration_ == 0 ||
        joint_bound.max_acceleration_ == 0) {
        RCLCPP_ERROR(
          node_->get_logger(), "Joint acceleration limit is not defined or set to zero: %s",
          joint_names.at(i).c_str());
        timer_->cancel();
        return;
      }

      const double pos_delta = target_joint_positions.at(i) - current_joint_positions.at(i);
      const double target_vel =
        pos_delta >= 0 ? joint_bound.max_velocity_ : joint_bound.min_velocity_;

      const double vel_delta = target_vel - current_joint_velocities.at(i);
      const double acc =
        vel_delta >= 0 ? joint_bound.max_acceleration_ : joint_bound.min_acceleration_;

      // Constant acceleration section
      const double acc_time = vel_delta / acc;

      const double displacement_on_acc =
        current_joint_velocities.at(i) * acc_time + acc * acc_time * acc_time / 2;

      // Constant velocity section
      const double linear_time = (pos_delta - displacement_on_acc) / target_vel;

      // Note: total_time can be negative
      const double total_time = acc_time + linear_time;
      assert(std::isfinite(total_time));

      goal_times.emplace_back(total_time);
    }

    // Use largest time
    goal_time = *std::max_element(goal_times.begin(), goal_times.end());
  }

  // Compose JointTrajectory message
  JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node_->now();
  trajectory_msg.header.frame_id = ik_solver->getBaseFrame();
  trajectory_msg.joint_names = joint_names;

  const bool is_command_stale = node_->now() - pose_msg_->header.stamp >
                                rclcpp::Duration::from_seconds(params_.incoming_command_timeout);

  if (!is_command_stale) {
    auto & point = trajectory_msg.points.emplace_back();
    point.time_from_start = rclcpp::Duration::from_seconds(goal_time);
    point.positions = target_joint_positions;
    trajectory_publisher_->publish(trajectory_msg);
  } else {
    pose_msg_.reset();
    target_pose_.setIdentity();

    auto & point = trajectory_msg.points.emplace_back();
    point.time_from_start = rclcpp::Duration::from_seconds(params_.stopping_time);

    // Compute future positions based on current velocities
    point.positions = current_joint_positions;
    for (size_t i = 0; i < n_joints; ++i) {
      point.positions.at(i) += current_joint_velocities.at(i) * params_.stopping_time;
    }

    point.velocities.resize(n_joints, 0.0);
    trajectory_publisher_->publish(trajectory_msg);
  }
}

// Self collision only for now
bool Servo::check_collision(const std::vector<double> & joint_positions) const
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  collision_request.group_name = params_.move_group_name;
  collision_request.distance = true;

  const auto robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  robot_state->setJointGroupActivePositions(params_.move_group_name, joint_positions);
  robot_state->updateCollisionBodyTransforms();

  planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);

  locked_scene->getCollisionEnvUnpadded()->checkSelfCollision(
    collision_request, collision_result, *robot_state, locked_scene->getAllowedCollisionMatrix());

  return collision_result.distance < params_.collision_threshold_distance;
}

}  // namespace prox::pregrasp
