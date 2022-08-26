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

#include "vl53l5cx_client/auto_control_node.hpp"

#include <memory>
#include <string>
#include <vector>

using std_srvs::srv::Empty;
using namespace std::chrono_literals;

namespace vl53l5cx
{
AutoControl::AutoControl() : Node("vl53l5cx_auto_control")
{
  this->declare_parameter<std::string>("target_node", "/vl53l5cx");
  std::string targte_node;
  this->get_parameter("target_node", targte_node);

  start_ranging_cli_ = this->create_client<Empty>(targte_node + "/start_ranging");
  stop_ranging_cli_ = this->create_client<Empty>(targte_node + "/stop_ranging");

  // Call start_ranging service
  const std::string service_name(start_ranging_cli_->get_service_name());
  if (start_ranging_cli_->wait_for_service(1s)) {
    start_ranging_cli_->async_send_request(std::make_shared<Empty::Request>());
    RCLCPP_INFO(this->get_logger(), "Called " + service_name);
  } else {
    RCLCPP_ERROR(
      this->get_logger(), service_name + " is not ready. Please call the service manually.");
  }
}

AutoControl::~AutoControl()
{
  // Call stop_ranging service
  stop_ranging_cli_->async_send_request(std::make_shared<Empty::Request>());
}
}  // namespace vl53l5cx
