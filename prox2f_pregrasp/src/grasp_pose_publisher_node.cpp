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

#include "prox2f_pregrasp/grasp_pose_publisher_node.hpp"

#include <algorithm>

#include "pcl/filters/filter.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace prox::pregrasp
{

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1, std::placeholders::_2;

GraspPosePublisher::GraspPosePublisher(const rclcpp::NodeOptions & options)
: Node("grasp_pose_publisher", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  sync_(5)
{
  param_listener_ =
    std::make_shared<grasp_pose_publisher::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  subscriber1_.subscribe(this, "input1/normals", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  subscriber2_.subscribe(this, "input2/normals", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_.connectInput(subscriber1_, subscriber2_);
  sync_.registerCallback(std::bind(&GraspPosePublisher::topic_callback, this, _1, _2));
  publisher_ = this->create_publisher<PoseStamped>("~/pose", rclcpp::SensorDataQoS());
}

void GraspPosePublisher::topic_callback(
  const PointCloud2::ConstSharedPtr & normals_msg1,
  const PointCloud2::ConstSharedPtr & normals_msg2)
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  // Pose alignment vector
  tf2::Vector3 alignment;
  {
    const auto & v = params_.alignment_vector;
    alignment = tf2::Vector3{v[0], v[1], v[2]};
    if (alignment.length2() != 0) {
      alignment.normalize();
    }
  }

  // Output
  tf2::Vector3 position{0, 0, 0}, orientation{0, 0, 0};

  const auto msgs = {normals_msg1, normals_msg2};
  for (const auto & msg : msgs) {
    pcl::PointCloud<pcl::PointNormal> normals;
    pcl::fromROSMsg(*msg, normals);

    assert(normals.isOrganized());

    // Remove NaN
    {
      pcl::Indices index;
      pcl::removeNaNFromPointCloud(normals, normals, index);
      pcl::removeNaNNormalsFromPointCloud(normals, normals, index);
    }

    if (normals.empty()) {
      // Object not detected
      return;
    }

    // Transform from message frame to TCP frame
    tf2::Transform tf;
    const auto tf_msg =
      tf_buffer_.lookupTransform(params_.tcp_frame_id, msg->header.frame_id, tf2::TimePointZero);
    tf2::convert(tf_msg.transform, tf);

    tf2::Vector3 local_position{0, 0, 0}, local_orientation{0, 0, 0};

    for (const auto & e : normals) {
      tf2::Vector3 point{e.x, e.y, e.z};
      tf2::Vector3 normal{e.normal_x, e.normal_y, e.normal_z};

      // Position calculation
      local_position += tf * point;

      if (normal.length2() == 0) {
        // Isolated point
        continue;
      }
      normal.normalize();

      // Orientation calculation
      const auto cross = alignment.cross(normal);
      const auto cross_norm = cross.length();
      if (cross_norm == 0 || cross_norm >= 1) {
        // No orientation or right angle
        continue;
      }
      const auto angle = std::asin(cross_norm);
      const auto axis = -cross / cross_norm;
      local_orientation += tf.getBasis() * (angle * axis);
    }

    // Average
    local_position /= normals.size();
    local_orientation /= normals.size();

    position += local_position;
    orientation += local_orientation;
  }

  // Average
  position /= msgs.size();
  orientation /= msgs.size();

  assert(std::isfinite(position.x()) && std::isfinite(position.y()) && std::isfinite(position.z()));
  assert(
    std::isfinite(orientation.x()) && std::isfinite(orientation.y()) &&
    std::isfinite(orientation.z()));

  tf2::Quaternion quaternion;
  quaternion.setRPY(orientation.x(), orientation.y(), orientation.z());

  // Compose pose message
  Pose pose;
  tf2::toMsg(position, pose.position);
  pose.orientation = tf2::toMsg(quaternion);
  {
    // Transform from TCP frame to output frame
    const auto tf_msg =
      tf_buffer_.lookupTransform(params_.output_frame_id, params_.tcp_frame_id, tf2::TimePointZero);
    tf2::doTransform(pose, pose, tf_msg);
  }

  // Publish message
  PoseStamped pose_msg;
  pose_msg.header.frame_id = params_.output_frame_id;
  pose_msg.header.stamp = normals_msg1->header.stamp;
  pose_msg.pose = pose;
  publisher_->publish(pose_msg);
}

}  // namespace prox::pregrasp
