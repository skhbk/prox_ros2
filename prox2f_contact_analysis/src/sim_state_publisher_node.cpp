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

#include "prox2f_contact_analysis/robotiq_2f_85_fingertip.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>

namespace prox
{
namespace contact
{
using sensor_msgs::msg::JointState;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;

class SimStatePublisher : public rclcpp::Node
{
  float contact_width_;
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<JointState>::SharedPtr publisher_;

public:
  SimStatePublisher() : Node("sim_state_publisher"), contact_width_(Robotiq2F85Fingertip::stroke)
  {
    this->declare_parameter<std::string>("joint", "sim_robotiq_85_left_knuckle_joint");
    this->declare_parameter<std::string>("base_link", "robotiq_85_base_link");
    this->declare_parameter<double>("weight", 1);

    subscription_ = this->create_subscription<PointCloud2>(
      "input/points", rclcpp::SensorDataQoS(),
      std::bind(&SimStatePublisher::topic_callback, this, _1));
    publisher_ = this->create_publisher<JointState>("joint_states", 10);
  }

  void topic_callback(const PointCloud2::ConstSharedPtr input_msg)
  {
    std::string joint_name, base_link;
    this->get_parameter("joint", joint_name);
    this->get_parameter("base_link", base_link);

    const auto & input_frame_id = input_msg->header.frame_id;
    if (input_frame_id != base_link) {
      throw std::runtime_error(
        "The frame id of the received message [" + input_frame_id +
        "] is different from expected one [" + base_link + "]");
    }

    JointState msg;
    msg.header.stamp = this->now();
    msg.name.push_back(joint_name);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*input_msg, cloud);

    const auto contact_width = this->get_contact_width(cloud);

    // EMA filter
    double weight;
    this->get_parameter("weight", weight);
    contact_width_ = contact_width * weight + contact_width_ * (1 - weight);

    const auto position = this->width_to_position(contact_width_);
    msg.position.push_back(position);

    publisher_->publish(msg);
  }

  static float get_contact_width(const pcl::PointCloud<pcl::PointXYZ> cloud)
  {
    // Gripper width
    constexpr auto y_lim = Robotiq2F85Fingertip::width / 2;

    if (cloud.empty()) {
      return Robotiq2F85Fingertip::stroke;
    }

    float x_max = 0;
    for (const auto & e : cloud) {
      const auto x = std::abs(e.x);
      const auto y = std::abs(e.y);
      if (x > x_max && y < y_lim) {
        x_max = x;
      }
    }
    return 2 * x_max;
  }

  static float width_to_position(float width)
  {
    constexpr float stroke = .085;
    constexpr float alpha = 49.45 * M_PI / 180;
    constexpr float beta = 84.64 * M_PI / 180;

    if (width < 0 || width > stroke) {
      return 0;
    }

    const float cosa = std::cos(alpha);
    const float cosb = std::cos(beta);

    const float position = std::acos((cosa + cosb) / stroke * width - cosb) - alpha;
    if (std::isnan(position) || position < 0.) {
      return 0;
    }
    if (position > .8) {
      return .8;
    }
    return position;
  }
};
}  // namespace contact
}  // namespace prox

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<prox::contact::SimStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
