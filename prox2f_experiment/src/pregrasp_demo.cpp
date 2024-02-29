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

class PreGraspDemo : public ExperimentBase
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_servo_client_;

public:
  PreGraspDemo(rclcpp::Node::SharedPtr node) : ExperimentBase(node)
  {
    enable_servo_client_ =
      node->create_client<std_srvs::srv::Trigger>("local_planner/enable_servo");
  }

  void run()
  {
    // Enable servo
    {
      auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
      enable_servo_client_->async_send_request(trigger).get();
      RCLCPP_INFO(node_->get_logger(), "Servo enabled");
    }

    // Send empty request
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
  }
};
}  // namespace prox::experiment

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  const auto node = std::make_shared<rclcpp::Node>("pregrasp_demo", node_options);

  prox::experiment::PreGraspDemo experiment{node};
  std::thread run_experiment([&] { experiment.run(); });

  rclcpp::spin(node);
  run_experiment.join();
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
