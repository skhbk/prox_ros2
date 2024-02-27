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

#include <fstream>

#include "prox2f_experiment/experiment_base.hpp"

#include "moveit/kinematic_constraints/utils.h"
#include "moveit/robot_state/conversions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "moveit_msgs/msg/motion_plan_response.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace prox::experiment
{
using namespace std::chrono_literals;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using rcl_interfaces::srv::SetParametersAtomically;

class Pickup : public ExperimentBase
{
  rclcpp::Client<SetParametersAtomically>::SharedPtr grasp_pose_publisher_set_params_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_servo_client_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_subscription_;

  PoseWithCovarianceStamped::SharedPtr pose_;
  std::deque<Eigen::Vector3d> release_positions_;

public:
  Pickup(rclcpp::Node::SharedPtr node) : ExperimentBase(node)
  {
    grasp_pose_publisher_set_params_client_ = node_->create_client<SetParametersAtomically>(
      "grasp_pose_publisher/set_parameters_atomically");

    enable_servo_client_ =
      node->create_client<std_srvs::srv::Trigger>("local_planner/enable_servo");

    pose_subscription_ = node_->create_subscription<PoseWithCovarianceStamped>(
      "object_detection/pose", rclcpp::SensorDataQoS(),
      [this](const PoseWithCovarianceStamped::SharedPtr msg) { pose_ = msg; });

    release_positions_.emplace_back(Eigen::Vector3d(0.3, -0.3, 0.1));
    release_positions_.emplace_back(Eigen::Vector3d(0.5, -0.3, 0.1));
    release_positions_.emplace_back(Eigen::Vector3d(0.4, -0.2, 0.1));
  }

  bool run()
  {
    const std::string ee_frame = "tcp";

    // Open gripper
    {
      control_msgs::msg::GripperCommand gripper_command;
      gripper_command.position = 0.1;
      gripper_command.max_effort = 2.0;

      const auto & result = this->executeGripperCommand(gripper_command).get();

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        default:
          return false;
      }
    }

    // Move to start state
    {
      const auto & result = this->moveToState("home").get();
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "Start state reached");
          break;
        default:
          RCLCPP_ERROR(node_->get_logger(), result.result->error_message.c_str());
          return false;
      }
    }

    const auto ee_default_orientation = Eigen::Quaterniond(this->getTransform(ee_frame).rotation());

    pose_.reset();
    rclcpp::WallRate wall_rate(1s);

    while (true) {
      if (pose_) {
        const Eigen::Map<Eigen::MatrixXd> covariance(pose_->pose.covariance.data(), 6, 6);
        const auto position_covariance = covariance.block(0, 0, 3, 3);
        const auto orientation_covariance = covariance.block(3, 3, 3, 3);

        if (position_covariance.sum() < 0.001) {
          std::vector<bool> enabled_axes(false, 6);

          if (orientation_covariance.sum() > 5.0) {
            enabled_axes = {true, true, true, false, false, false};
            RCLCPP_WARN(node_->get_logger(), "Disabled RZ rotation");
          } else {
            enabled_axes = {true, true, true, false, false, true};
            RCLCPP_WARN(node_->get_logger(), "Enabled RZ rotation");
          }

          rclcpp::Parameter parameter("enabled_axes", enabled_axes);
          auto set_params_request = std::make_shared<SetParametersAtomically::Request>();
          set_params_request->parameters.emplace_back(parameter.to_parameter_msg());
          grasp_pose_publisher_set_params_client_->async_send_request(set_params_request).get();

          break;
        }
      }

      wall_rate.sleep();
    }

    // Enable servo
    {
      auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
      enable_servo_client_->async_send_request(trigger).get();
      RCLCPP_INFO(node_->get_logger(), "Servo enabled");
    }

    // Move down
    {
      auto request = this->constructMotionPlanRequest();

      auto object_position_msg = pose_->pose.pose.position;
      const auto tf_buffer = planning_scene_monitor_->getTFClient();
      const auto tf_msg =
        tf_buffer->lookupTransform("world", pose_->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(object_position_msg, object_position_msg, tf_msg);

      // const Eigen::Vector3d translation{0, 0, -0.28};

      const auto & ee_transform = this->getTransform(ee_frame);

      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.frame_id = "world";
      goal_pose.pose.position = object_position_msg;
      goal_pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond(ee_transform.rotation()));

      auto above_goal_pose = goal_pose;
      above_goal_pose.pose.position.z += 0.1;

      request.goal_constraints.emplace_back(
        kinematic_constraints::constructGoalConstraints(ee_frame, above_goal_pose));
      request.goal_constraints.emplace_back(
        kinematic_constraints::constructGoalConstraints(ee_frame, goal_pose));

      const auto & result = this->executeMotionPlanRequest(request).get();

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        default:
          return false;
      }
    }

    // Close gripper
    {
      control_msgs::msg::GripperCommand gripper_command;
      gripper_command.position = 0.01;
      gripper_command.max_effort = 4.0;

      const auto & result = this->executeGripperCommand(gripper_command).get();
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        default:
          return false;
      }
    }

    const auto z_release = this->getTransform(ee_frame).translation().z();

    if (release_positions_.empty()) {
      std::cerr << "RELEASE POSITIONS EMPTY" << std::endl;
      return false;
    }

    // Place path
    {
      auto request = this->constructMotionPlanRequest();

      const auto & ee_transform = this->getTransform(ee_frame);
      const Eigen::Vector3d translation{0, 0, 0.1};

      geometry_msgs::msg::PoseStamped move_up, above_goal, goal;

      move_up.header.frame_id = "world";
      move_up.pose = tf2::toMsg(ee_transform);
      move_up.pose.position.z += 0.1;
      move_up.pose.orientation = tf2::toMsg(ee_default_orientation);

      goal.header.frame_id = "world";
      goal.pose.position = tf2::toMsg(release_positions_.front());
      release_positions_.pop_front();
      goal.pose.position.z = z_release;
      goal.pose.orientation = tf2::toMsg(ee_default_orientation);

      above_goal = goal;
      above_goal.pose.position.z += 0.1;

      request.goal_constraints.emplace_back(
        kinematic_constraints::constructGoalConstraints(ee_frame, move_up));
      request.goal_constraints.emplace_back(
        kinematic_constraints::constructGoalConstraints(ee_frame, above_goal));
      request.goal_constraints.emplace_back(
        kinematic_constraints::constructGoalConstraints(ee_frame, goal));

      const auto & result = this->executeMotionPlanRequest(request).get();

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        default:
          return false;
      }
    }

    // Open gripper
    {
      control_msgs::msg::GripperCommand gripper_command;
      gripper_command.position = 0.10;
      gripper_command.max_effort = 2.0;

      const auto & result = this->executeGripperCommand(gripper_command).get();

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        default:
          return false;
      }
    }

    RCLCPP_INFO(node_->get_logger(), "TASK COMPLETED");
    return true;
  }
};
}  // namespace prox::experiment

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  const auto node = std::make_shared<rclcpp::Node>("pickup", node_options);

  prox::experiment::Pickup experiment{node};
  std::thread run_experiment([&] {
    while (rclcpp::ok()) {
      const auto success = experiment.run();
      if (!success) {
        break;
      }
    }
  });

  rclcpp::spin(node);
  run_experiment.join();
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
