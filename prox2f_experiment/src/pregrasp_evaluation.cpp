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

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/motion_plan_response.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace prox::experiment
{
using namespace std::chrono_literals;
using geometry_msgs::msg::PoseArray;

class PreGraspEvaluation : public ExperimentBase
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_servo_client_;

public:
  PreGraspEvaluation(rclcpp::Node::SharedPtr node) : ExperimentBase(node)
  {
    enable_servo_client_ =
      node->create_client<std_srvs::srv::Trigger>("local_planner/enable_servo");
  }

  void run()
  {
    const std::string ee_frame = "tcp";
    const std::string reference_state_name = "pregrasp_reference";

    const auto reference_state = this->getState(reference_state_name);
    const auto & reference_pose = reference_state.getFrameTransform(ee_frame);

    // Translation
    // constexpr double delta = 0.005;
    // constexpr double max_error = 0.061;
    // constexpr double min_error = -max_error;

    // Rotation
    constexpr double delta = 5 * M_PI / 180;
    constexpr double max_error = 45.1 * M_PI / 180;
    constexpr double min_error = -max_error;

    std::vector<double> errors;
    for (int i = min_error / delta; i <= max_error / delta; ++i) {
      errors.emplace_back(i * delta);
    }

    // Record file
    std::stringstream ss;
    ss << node_->get_name() << "_rz.csv";
    std::ofstream ofs(ss.str());
    ofs << "error,position.x,position.y,position.z,orientation.w,orientation.x,"
           "orientation.y,orientation.z"
        << std::endl;

    for (const auto & error : errors) {
      // Move to start state
      {
        const auto & result = this->moveToState(reference_state).get();
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Start state reached");
            break;
          default:
            RCLCPP_ERROR(node_->get_logger(), result.result->error_message.c_str());
            return;
        }
      }

      // Apply error
      {
        auto request = this->constructMotionPlanRequest();

        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = ee_frame;
        // goal_pose.pose.position.y = error;
        // goal_pose.pose.orientation.w = 1.0;

        const Eigen::AngleAxisd angle_axis(error, Eigen::Vector3d{0, 0, 1});
        goal_pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond(angle_axis));

        request.goal_constraints.emplace_back(
          kinematic_constraints::constructGoalConstraints(ee_frame, goal_pose));

        const auto & result = this->executeMotionPlanRequest(request).get();

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          default:
            return;
        }
      }

      // Enable servo
      {
        auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
        enable_servo_client_->async_send_request(trigger).get();
        RCLCPP_INFO(node_->get_logger(), "Servo enabled");
      }

      {
        const auto request = this->constructMotionPlanRequest();
        const auto & result = this->executeMotionPlanRequest(request).get();
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "PREGRASP SUCCEEDED");
            break;
          default:
            return;
        }
      }

      // Record
      {
        ofs << error << ",";

        const auto & ee_pose = this->getCurrentState().getFrameTransform(ee_frame);
        const auto transform = reference_pose.inverse() * ee_pose;
        const auto & t = transform.translation();
        const Eigen::Quaterniond q(transform.rotation());
        ofs << t.x() << ",";
        ofs << t.y() << ",";
        ofs << t.z() << ",";
        ofs << q.w() << ",";
        ofs << q.x() << ",";
        ofs << q.y() << ",";
        ofs << q.z() << std::endl;
      }

      RCLCPP_WARN(node_->get_logger(), "COMPLETED. ERROR: %f", error);
    }
  }
};
}  // namespace prox::experiment

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  const auto node = std::make_shared<rclcpp::Node>("pregrasp_evaluation", node_options);

  prox::experiment::PreGraspEvaluation experiment{node};
  std::thread run_experiment([&] { experiment.run(); });

  rclcpp::spin(node);
  run_experiment.join();
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
