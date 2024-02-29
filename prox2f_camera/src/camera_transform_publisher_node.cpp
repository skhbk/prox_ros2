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

#include "prox2f_camera/cacmera_transform_publisher_node.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "opencv2/aruco.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace prox::camera
{
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

CameraTransformPublisher::CameraTransformPublisher(const rclcpp::NodeOptions & options)
: Node("camera_transform_publisher", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  param_listener_ = std::make_shared<camera_transform_publisher::ParamListener>(
    this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  camera_subscriber_ = image_transport::create_camera_subscription(
    this, "~/image_raw",
    std::bind(
      &CameraTransformPublisher::topic_callback, this, std::placeholders::_1,
      std::placeholders::_2),
    "raw");

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void CameraTransformPublisher::topic_callback(
  const Image::ConstSharedPtr & image_msg, const CameraInfo::ConstSharedPtr & info_msg)
{
  const cv::Mat image = cv_bridge::toCvShare(image_msg)->image.clone();

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;

  const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::aruco::detectMarkers(image, dictionary, corners, ids);

  if (corners.empty()) {
    return;
  }

  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(info_msg);

  std::vector<cv::Vec3d> rvecs, tvecs;

  cv::aruco::estimatePoseSingleMarkers(
    corners, params_.marker_length, model.intrinsicMatrix(), model.distortionCoeffs(), rvecs,
    tvecs);

  for (size_t i = 0; i < ids.size(); ++i) {
    if (ids.at(i) != params_.marker_id) {
      continue;
    }

    tf2::Transform tf_image_to_marker;
    {
      const auto & tvec = tvecs.at(i);
      const auto & rvec = rvecs.at(i);
      const tf2::Vector3 translation(tvec[0], tvec[1], tvec[2]);
      const tf2::Quaternion rotation({rvec[0], rvec[1], rvec[2]}, cv::norm(rvec));
      tf_image_to_marker.setOrigin(translation);
      tf_image_to_marker.setRotation(rotation);
    }

    tf2::Transform tf_camera_to_image;
    {
      const auto tf_msg = tf_buffer_.lookupTransform(
        image_msg->header.frame_id, params_.camera_frame_id, tf2::TimePointZero);
      tf2::convert(tf_msg.transform, tf_camera_to_image);
    }

    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = image_msg->header.stamp;
    transform_msg.header.frame_id = params_.marker_frame_id;
    transform_msg.child_frame_id = params_.camera_frame_id;

    transform_msg.transform = tf2::toMsg(tf_image_to_marker.inverse() * tf_camera_to_image);
    tf_broadcaster_->sendTransform(transform_msg);

    RCLCPP_INFO(
      this->get_logger(), "Published transform from %s to %s",
      transform_msg.header.frame_id.c_str(), transform_msg.child_frame_id.c_str());

    camera_subscriber_.shutdown();
  }
}
}  // namespace prox::camera
