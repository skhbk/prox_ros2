//  Copyright 2022 Sakai Hibiki
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

#ifndef VL53L5CX_CLIENT__AUTO_CONTROL_NODE_HPP_
#define VL53L5CX_CLIENT__AUTO_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <string>

namespace vl53l5cx
{
class AutoControl : public rclcpp::Node
{
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_ranging_cli_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_ranging_cli_;

public:
  AutoControl();
  ~AutoControl();
};

}  // namespace vl53l5cx

#endif  // VL53L5CX_CLIENT__AUTO_CONTROL_NODE_HPP_
