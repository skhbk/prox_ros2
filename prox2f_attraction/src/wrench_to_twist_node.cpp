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

#include "prox2f_attraction/wrench_to_twist_node.hpp"

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace prox::attraction
{

using geometry_msgs::msg::TwistStamped;
using geometry_msgs::msg::WrenchStamped;
using std::placeholders::_1;

WrenchToTwist::WrenchToTwist(const rclcpp::NodeOptions & options) : Node("wrench_to_twist", options)
{
  param_listener_ =
    std::make_shared<wrench_to_twist::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  subscription_ = this->create_subscription<WrenchStamped>(
    "input/wrench_stamped", rclcpp::SensorDataQoS(),
    std::bind(&WrenchToTwist::topic_callback, this, _1));
  publisher_ = this->create_publisher<TwistStamped>("~/twist_stamped", rclcpp::SensorDataQoS());
}

void WrenchToTwist::topic_callback(const WrenchStamped::ConstSharedPtr & wrench_msg)
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  TwistStamped twist_msg;
  twist_msg.header = wrench_msg->header;

  tf2::Vector3 force, torque;
  tf2::convert(wrench_msg->wrench.force, force);
  tf2::convert(wrench_msg->wrench.torque, torque);

  tf2::convert(force / params_.mass, twist_msg.twist.linear);
  tf2::convert(torque / params_.inertia, twist_msg.twist.angular);

  publisher_->publish(twist_msg);
}

}  // namespace prox::attraction
