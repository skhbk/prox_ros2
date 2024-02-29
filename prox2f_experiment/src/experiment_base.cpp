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

#include "prox2f_experiment/experiment_base.hpp"

#include "moveit/kinematic_constraints/utils.h"
#include "moveit/robot_state/conversions.h"
#include "rclcpp_action/create_client.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace prox::experiment
{
using control_msgs::action::GripperCommand;
using moveit_msgs::action::HybridPlanner;
using moveit_msgs::msg::MotionPlanRequest;

ExperimentBase::ExperimentBase(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  param_listener_ =
    std::make_shared<experiment_base::ParamListener>(node_->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    node_, "robot_description", "planning_scene_monitor");

  if (!planning_scene_monitor_->getPlanningScene()) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to configure planning scene monitor");
    std::exit(EXIT_FAILURE);
  }
  planning_scene_monitor_->startStateMonitor();
  planning_scene_monitor_->providePlanningSceneService();
  planning_scene_monitor_->setPlanningScenePublishingFrequency(60.0);
  planning_scene_monitor_->startPublishingPlanningScene(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  planning_scene_monitor_->startSceneMonitor();

  if (!planning_scene_monitor_->waitForCurrentRobotState(node_->now(), 5.0)) {
    std::exit(EXIT_FAILURE);
  }

  hybrid_planner_action_client_ =
    rclcpp_action::create_client<HybridPlanner>(node_, params_.hybrid_planning_action_name);

  gripper_action_client_ =
    rclcpp_action::create_client<GripperCommand>(node_, params_.gripper_command_action_name);

  hybrid_planner_action_client_->wait_for_action_server();
  gripper_action_client_->wait_for_action_server();
}

const moveit::core::RobotState & ExperimentBase::getCurrentState() const
{
  const planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
  return locked_scene->getCurrentState();
}

moveit::core::RobotState ExperimentBase::getState(const std::string & state_name) const
{
  const auto robot_model = planning_scene_monitor_->getRobotModel();
  const auto joint_group = robot_model->getJointModelGroup(params_.group_name);

  moveit::core::RobotState state(robot_model);
  if (!state.setToDefaultValues(joint_group, state_name)) {
    throw std::runtime_error("State " + state_name + " not found in " + params_.group_name);
  }
  state.update();

  return state;
}

const Eigen::Isometry3d & ExperimentBase::getTransform(const std::string & link_name) const
{
  const auto & current_state = this->getCurrentState();
  return current_state.getGlobalLinkTransform(link_name);
}

moveit_msgs::msg::MotionPlanRequest ExperimentBase::constructMotionPlanRequest() const
{
  MotionPlanRequest request;
  request.group_name = params_.group_name;
  moveit::core::robotStateToRobotStateMsg(this->getCurrentState(), request.start_state);

  return request;
}

std::shared_future<HybridPlannerWrappedResult> ExperimentBase::executeMotionPlanRequest(
  const moveit_msgs::msg::MotionPlanRequest & request)
{
  rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::Goal action_goal;
  action_goal.planning_group = request.group_name;
  action_goal.motion_sequence.items.emplace_back().req = request;

  const auto goal_handle = hybrid_planner_action_client_->async_send_goal(action_goal).get();
  const auto result_future = hybrid_planner_action_client_->async_get_result(goal_handle);

  return result_future;
}

std::shared_future<HybridPlannerWrappedResult> ExperimentBase::moveToState(
  const moveit::core::RobotState & state)
{
  const auto robot_model = planning_scene_monitor_->getRobotModel();
  const auto joint_group = robot_model->getJointModelGroup(params_.group_name);

  auto request = this->constructMotionPlanRequest();

  request.goal_constraints.emplace_back(
    kinematic_constraints::constructGoalConstraints(state, joint_group));

  return this->executeMotionPlanRequest(request);
}

std::shared_future<HybridPlannerWrappedResult> ExperimentBase::moveToState(
  const std::string & state_name)
{
  const auto target_state = this->getState(state_name);

  return this->moveToState(target_state);
}

std::shared_future<HybridPlannerWrappedResult> ExperimentBase::moveToPose(
  const std::string & link_name, const geometry_msgs::msg::PoseStamped & pose)
{
  auto request = this->constructMotionPlanRequest();
  request.goal_constraints.emplace_back(
    kinematic_constraints::constructGoalConstraints(link_name, pose));

  return this->executeMotionPlanRequest(request);
}

std::shared_future<GripperCommandWrappedResult> ExperimentBase::executeGripperCommand(
  const control_msgs::msg::GripperCommand & command)
{
  rclcpp_action::Client<GripperCommand>::Goal action_goal;
  action_goal.command = command;

  const auto goal_handle = gripper_action_client_->async_send_goal(action_goal).get();
  const auto result_future = gripper_action_client_->async_get_result(goal_handle);

  return result_future;
}
}  // namespace prox::experiment
