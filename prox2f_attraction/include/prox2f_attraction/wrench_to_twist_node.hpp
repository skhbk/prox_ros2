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

#ifndef PROX2F_ATTRACTION__WRENCH_TO_TWIST_NODE_HPP_
#define PROX2F_ATTRACTION__WRENCH_TO_TWIST_NODE_HPP_

#include <memory>

#include "wrench_to_twist_params.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace prox::attraction
{

class WrenchToTwist : public rclcpp::Node
{
  std::shared_ptr<wrench_to_twist::ParamListener> param_listener_;
  wrench_to_twist::Params params_;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;

public:
  explicit WrenchToTwist(const rclcpp::NodeOptions & options);

private:
  void topic_callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr & wrench_msg);
};

}  // namespace prox::attraction

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::attraction::WrenchToTwist)

#endif  // PROX2F_ATTRACTION__WRENCH_TO_TWIST_NODE_HPP_
