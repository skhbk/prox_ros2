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

#include "prox2f_attraction/virtual_wrench_node.hpp"

#include <algorithm>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace prox::attraction
{

using geometry_msgs::msg::WrenchStamped;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1, std::placeholders::_2;

VirtualWrench::VirtualWrench(const rclcpp::NodeOptions & options)
: Node("virtual_wrench", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), sync_(5)
{
  param_listener_ =
    std::make_shared<virtual_wrench::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  subscriber1_.subscribe(this, "input1/normals", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  subscriber2_.subscribe(this, "input2/normals", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_.connectInput(subscriber1_, subscriber2_);
  sync_.registerCallback(std::bind(&VirtualWrench::topic_callback, this, _1, _2));
  publisher_ = this->create_publisher<WrenchStamped>("~/wrench_stamped", rclcpp::SensorDataQoS());
}

void VirtualWrench::topic_callback(
  const PointCloud2::ConstSharedPtr & normals_msg1,
  const PointCloud2::ConstSharedPtr & normals_msg2)
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  std::vector<PointCloud2::ConstSharedPtr> msgs{normals_msg1, normals_msg2};

  tf2::Vector3 force{0, 0, 0}, torque{0, 0, 0};

  for (const auto & msg : msgs) {
    // Get transform to the wrench frame
    tf2::Transform tf_sensor;
    {
      const auto tf_sensor_msg = tf_buffer_.lookupTransform(
        params_.wrench_frame_id, msg->header.frame_id, tf2::TimePointZero);
      tf2::convert(tf_sensor_msg.transform, tf_sensor);
    }

    pcl::PointCloud<pcl::PointNormal> normals;
    pcl::fromROSMsg(*msg, normals);

    for (const auto & e : normals) {
      if (!std::isfinite(e.x) || !std::isfinite(e.normal_x)) {
        continue;
      }

      tf2::Vector3 point{e.x, e.y, e.z};
      tf2::Vector3 normal{e.normal_x, e.normal_y, 0};

      // Transform
      point = tf_sensor * point;
      normal = tf_sensor.getBasis() * normal;

      // Compute wrench
      force += params_.stiffness * point;
      torque += params_.stiffness * point.cross(params_.normal_scale * normal);
    }
  }

  tf2::Vector3 force_scale{
    params_.force_scales[0], params_.force_scales[1], params_.force_scales[2]};
  tf2::Vector3 torque_scale{
    params_.torque_scales[0], params_.torque_scales[1], params_.torque_scales[2]};

  WrenchStamped wrench_msg;
  wrench_msg.header.frame_id = params_.wrench_frame_id;
  wrench_msg.header.stamp = normals_msg1->header.stamp;
  tf2::convert(force * force_scale, wrench_msg.wrench.force);
  tf2::convert(torque * torque_scale, wrench_msg.wrench.torque);

  publisher_->publish(wrench_msg);
}

}  // namespace prox::attraction
