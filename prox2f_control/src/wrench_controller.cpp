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

#include "prox2f_control/wrench_controller.hpp"

#include "control_toolbox/filters.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace prox::control
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;

static Eigen::Block<Eigen::Vector<double, 6>> linear_part(Eigen::Vector<double, 6> & twist)
{
  return twist.block(0, 0, 3, 1);
}
static Eigen::Block<Eigen::Vector<double, 6>> angular_part(Eigen::Vector<double, 6> & twist)
{
  return twist.block(3, 0, 3, 1);
}

CallbackReturn WrenchController::on_init()
{
  param_listener_ = std::make_shared<wrench_controller::ParamListener>(this->get_node());
  params_ = param_listener_->get_params();

  wrench_.setZero();
  pids_.resize(6);

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration WrenchController::command_interface_configuration() const
{
  std::vector<std::string> command_interfaces_config_names;
  for (const auto & joint : params_.joints) {
    const auto full_name = joint + "/" + hardware_interface::HW_IF_VELOCITY;
    command_interfaces_config_names.push_back(full_name);
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names};
}

InterfaceConfiguration WrenchController::state_interface_configuration() const
{
  std::vector<std::string> state_interfaces_config_names;
  for (const auto & joint : params_.joints) {
    auto full_name = joint + "/" + hardware_interface::HW_IF_POSITION;
    state_interfaces_config_names.push_back(full_name);
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
}

CallbackReturn WrenchController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  command_subscription_ = this->get_node()->create_subscription<CmdType>(
    params_.command_topic, rclcpp::SensorDataQoS(),
    [this](const CmdType::SharedPtr msg) { rt_buffer_.writeFromNonRT(msg); });

  // Reset PID parameters
  for (size_t i = 0; i < 6; ++i) {
    pids_[i].initPid(
      params_.pid.p[i], params_.pid.i[i], params_.pid.d[i], params_.pid.i_clamp[i],
      -params_.pid.i_clamp[i]);
  }

  // Load IK plugin
  ik_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
    params_.kinematics.plugin_package, "kinematics_interface::KinematicsInterface");
  ik_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
    ik_loader_->createUnmanagedInstance(params_.kinematics.plugin_name));
  if (!ik_->initialize(
        this->get_node()->get_node_parameters_interface(), params_.kinematics.end_effector_link)) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn WrenchController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  rt_buffer_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn WrenchController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop all joints
  for (auto & interface : command_interfaces_) {
    interface.set_value(0.0);
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type WrenchController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const auto wrench_msg = *rt_buffer_.readFromRT();

  if (!wrench_msg) {
    RCLCPP_INFO_ONCE(this->get_node()->get_logger(), "No command message received yet.");
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_INFO_ONCE(this->get_node()->get_logger(), "Received command message.");
  }

  // Update control frame id
  const auto & new_frame_id = wrench_msg->header.frame_id;
  if (control_frame_id_ != new_frame_id) {
    control_frame_id_ = new_frame_id;
    RCLCPP_INFO_STREAM(
      this->get_node()->get_logger(),
      "Control-target frame-id is updated: [" << control_frame_id_ << "]");
  }

  // Get wrench
  const auto & force = wrench_msg->wrench.force;
  const auto & torque = wrench_msg->wrench.torque;
  Eigen::Vector<double, 6> wrench{force.x, force.y, force.z, torque.x, torque.y, torque.z};

  // If any element is NaN, assume all values are 0
  if (std::any_of(wrench.begin(), wrench.end(), [](double x) { return std::isnan(x); })) {
    wrench.setZero();
  }

  Eigen::Vector<double, 6> twist;
  for (size_t i = 0; i < 6; ++i) {
    // Apply exponential filter
    wrench_[i] = filters::exponentialSmoothing(wrench[i], wrench_[i], params_.filter_coefficient);
    // Compute command
    twist[i] = pids_[i].computeCommand(wrench_[i], period.nanoseconds());
  }

  auto actual_twist = twist;
  for (size_t i = 0; i < 6; ++i) {
    if (!params_.enabled_axes[i]) {
      actual_twist[i] = 0;
    }
  }

  // Get joint states
  std::vector<double> joint_states;
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const auto & interface = state_interfaces_[i];
    assert(interface.get_name() == params_.joints[i] + "/" + hardware_interface::HW_IF_POSITION);
    joint_states.push_back(interface.get_value());
  }

  // Rotate twist
  Eigen::Isometry3d transform;
  if (!ik_->calculate_link_transform(joint_states, control_frame_id_, transform)) {
    return controller_interface::return_type::ERROR;
  }
  Eigen::Vector<double, 6> base_twist;
  linear_part(base_twist) = transform.rotation() * linear_part(actual_twist);
  angular_part(base_twist) = transform.rotation() * angular_part(actual_twist);

  // Convert twist to joint commands
  std::vector<double> joint_commands(params_.joints.size(), 0.);
  const std::vector<double> cartesian_commands(
    base_twist.data(), base_twist.data() + base_twist.size());
  if (!ik_->convert_cartesian_deltas_to_joint_deltas(
        joint_states, cartesian_commands, control_frame_id_, joint_commands)) {
    return controller_interface::return_type::ERROR;
  }

  // Write commands to interface
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    command_interfaces_[i].set_value(joint_commands[i]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace prox::control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(prox::control::WrenchController, controller_interface::ControllerInterface)
