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

#include "prox2f_contact_analysis/virtual_wrench_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace prox::contact
{
using geometry_msgs::msg::Vector3;
using geometry_msgs::msg::Wrench;
using geometry_msgs::msg::WrenchStamped;
using message_filters::sync_policies::ApproximateTime;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;
using std::placeholders::_2;

static tf2::Vector3 sum(const std::vector<tf2::Vector3> & v)
{
  tf2::Vector3 sum = std::accumulate(v.begin(), v.end(), tf2::Vector3{0, 0, 0});
  return sum;
}

VirtualWrench::VirtualWrench(const rclcpp::NodeOptions & options)
: Node("virtual_wrench", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), sync_(5)
{
  this->declare_parameter<std::string>("reference_frame_id", "tool0");
  this->declare_parameter<double>("grasp_force", 1.);  // x-axis grasp force

  subscriber1_.subscribe(this, "input1/points", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  subscriber2_.subscribe(this, "input2/points", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_.connectInput(subscriber1_, subscriber2_);
  sync_.registerCallback(std::bind(&VirtualWrench::topic_callback, this, _1, _2));
  publisher_ = this->create_publisher<WrenchStamped>("wrench_stamped", rclcpp::SensorDataQoS());
}

void VirtualWrench::topic_callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg1,
  sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg2)
{
  WrenchStamped output_msg;
  this->get_parameter("reference_frame_id", output_msg.header.frame_id);
  output_msg.header.stamp = input_msg1->header.stamp;

  std::vector<tf2::Vector3> forces, torques;
  for (const auto & msg : {input_msg1, input_msg2}) {
    try {
      const auto wrench = this->compute_wrench(*msg);
      tf2::Vector3 force, torque;
      tf2::convert(wrench.force, force);
      tf2::convert(wrench.torque, torque);
      forces.push_back(std::move(force));
      torques.push_back(std::move(torque));
    } catch (const tf2::LookupException & e) {
      RCLCPP_ERROR(this->get_logger(), e.what());
      return;
    }
  }

  // Compute stiffness
  double grasp_force;
  this->get_parameter("grasp_force", grasp_force);
  double penetration_magnitude = 0;
  for (const auto & force : forces) {
    penetration_magnitude += std::abs(force.x());
  }
  const double stiffness = penetration_magnitude > 0 ? grasp_force / penetration_magnitude : 0;

  for (auto & force : forces) {
    force *= stiffness;
  }
  for (auto & torque : torques) {
    torque *= stiffness;
  }

  tf2::convert(sum(forces), output_msg.wrench.force);
  tf2::convert(sum(torques), output_msg.wrench.torque);

  publisher_->publish(output_msg);
}

Wrench VirtualWrench::compute_wrench(const PointCloud2 & cloud_msg) const
{
  std::string reference_frame_id;
  this->get_parameter("reference_frame_id", reference_frame_id);

  // Get transform
  const auto tf_msg =
    tf_buffer_.lookupTransform(reference_frame_id, cloud_msg.header.frame_id, tf2::TimePointZero);
  tf2::Transform tf;
  tf2::convert(tf_msg.transform, tf);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);

  // Compute force and torque in source frame
  tf2::Vector3 force_src{0, 0, 0}, torque_src{0, 0, 0};
  for (const auto & point : cloud) {
    if (!std::isfinite(point.z)) {
      continue;
    }
    const tf2::Vector3 position{point.x, point.y, 0};
    const tf2::Vector3 force{0, 0, std::abs(point.z)};
    const auto torque = position.cross(force);
    force_src += force;
    torque_src += torque;
  }

  // Compute force and torque in reference frame
  const auto & basis = tf.getBasis();
  const auto & origin = tf.getOrigin();
  const auto force_ref = basis * force_src;
  const auto torque_ref = basis * (torque_src - (basis.inverse() * origin).cross(force_src));

  Wrench wrench;
  tf2::convert(force_ref, wrench.force);
  tf2::convert(torque_ref, wrench.torque);
  return wrench;
}

}  // namespace prox::contact
