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

#ifndef PROX2F_CONTROL__TWIST_CONTROLLER_HPP_
#define PROX2F_CONTROL__TWIST_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_toolbox/pid.hpp"
#include "controller_interface/controller_interface.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "realtime_tools/realtime_buffer.h"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "prox_msgs/msg/twist_controller_state.hpp"

#include "twist_controller_params.hpp"

namespace prox::control
{

using CmdType = geometry_msgs::msg::TwistStamped;
using ControllerStateMsg = prox_msgs::msg::TwistControllerState;

class TwistController : public controller_interface::ControllerInterface
{
  std::shared_ptr<twist_controller::ParamListener> param_listener_;
  twist_controller::Params params_;

  std::string control_frame_id_;
  Eigen::Vector<double, 6> twist_;

  std::vector<control_toolbox::Pid> pids_;
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> ik_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> ik_;

  realtime_tools::RealtimeBuffer<CmdType::SharedPtr> rt_buffer_;
  rclcpp::Subscription<CmdType>::SharedPtr command_subscription_;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr state_publisher_;

public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  Eigen::Vector<double, 6> limit_twist(Eigen::Vector<double, 6> twist) const;
};

}  // namespace prox::control

#endif  // PROX2F_CONTROL__TWIST_CONTROLLER_HPP_
