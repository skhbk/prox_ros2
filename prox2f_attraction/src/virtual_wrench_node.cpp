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

#include "opencv2/imgproc.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace prox::attraction
{

using geometry_msgs::msg::WrenchStamped;
using prox_msgs::msg::Grid;
using std::placeholders::_1, std::placeholders::_2;
using visualization_msgs::msg::Marker;

VirtualWrench::VirtualWrench(const rclcpp::NodeOptions & options)
: Node("virtual_wrench", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), sync_(5)
{
  param_listener_ =
    std::make_shared<virtual_wrench::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  subscriber1_.subscribe(this, "input1/grid", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  subscriber2_.subscribe(this, "input2/grid", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_.connectInput(subscriber1_, subscriber2_);
  sync_.registerCallback(std::bind(&VirtualWrench::topic_callback, this, _1, _2));
  publisher_ = this->create_publisher<WrenchStamped>("~/wrench_stamped", rclcpp::SensorDataQoS());

  for (const auto & topic_name : {"~/marker1", "~/marker2"}) {
    marker_publishers_.emplace_back(
      this->create_publisher<Marker>(topic_name, rclcpp::SensorDataQoS()));
  }
}

void VirtualWrench::topic_callback(
  const Grid::ConstSharedPtr & grid_msg1, const Grid::ConstSharedPtr & grid_msg2)
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  if (
    grid_msg1->width != grid_msg2->width || grid_msg1->height != grid_msg2->height ||
    grid_msg1->pitch != grid_msg2->pitch) {
    throw std::runtime_error(
      "The geometry (width, height and pitch) of the two Grid messages is different.");
  }

  width_ = grid_msg1->width;
  height_ = grid_msg1->height;
  pitch_ = grid_msg1->pitch;

  std::vector<Grid::ConstSharedPtr> msgs{grid_msg1, grid_msg2};
  std::vector<cv::Mat1f> mats;
  std::vector<cv::Mat1b> masks;

  // Convert Grid to cv::Mat
  for (const auto & msg : msgs) {
    constexpr float invalid_value = 1e3;
    cv::Mat1f mat;
    if (msg->data.empty()) {
      mat = cv::Mat1f(cv::Size(width_, height_), invalid_value);
    } else {
      mat = cv::Mat1f(msg->data, true).reshape(1, height_);
      // Seems not working
      // cv::patchNaNs(mat, invalid_value);
    }
    mats.push_back(mat);
    // Filter NaN values
    masks.push_back(mat < invalid_value);
  }

  tf2::Vector3 force{0, 0, 0}, torque{0, 0, 0};
  for (size_t i = 0; i < msgs.size(); ++i) {
    const auto visual_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Compute wrench
    const auto wrench = this->compute_wrench(
      mats[i], masks[i], msgs[i]->header.frame_id, params_.attraction_frame_ids[i], visual_cloud);
    force += wrench[0];
    torque += wrench[1];

    // Visual stuff
    auto marker = this->visualize_attraction(*visual_cloud, i);
    marker.header.frame_id = params_.attraction_frame_ids[i];
    marker.header.stamp = msgs[i]->header.stamp;
    marker_publishers_[i]->publish(marker);
  }

  tf2::Vector3 force_scale{
    params_.force_scales[0], params_.force_scales[1], params_.force_scales[2]};
  tf2::Vector3 torque_scale{
    params_.torque_scales[0], params_.torque_scales[1], params_.torque_scales[2]};

  WrenchStamped wrench_msg;
  wrench_msg.header.frame_id = params_.wrench_frame_id;
  wrench_msg.header.stamp = grid_msg1->header.stamp;
  tf2::convert(force * force_scale, wrench_msg.wrench.force);
  tf2::convert(torque * torque_scale, wrench_msg.wrench.torque);

  publisher_->publish(wrench_msg);
}

std::array<tf2::Vector3, 2> VirtualWrench::compute_wrench(
  cv::Mat1f mat, cv::Mat1b mask, const std::string & sensor_frame_id,
  const std::string & attraction_frame_id,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> visual_cloud) const
{
  tf2::Vector3 force{0, 0, 0}, torque{0, 0, 0};

  const float center_x = width_ * pitch_ / 2;
  const float center_y = height_ * pitch_ / 2;
  const float force_gain = pitch_ * pitch_ * params_.stiffness;

  // Get transforms to the wrench frame
  tf2::Transform tf_sensor, tf_attraction;
  {
    const auto tf_sensor_msg =
      tf_buffer_.lookupTransform(params_.wrench_frame_id, sensor_frame_id, tf2::TimePointZero);
    tf2::convert(tf_sensor_msg.transform, tf_sensor);
    const auto tf_attraction_msg =
      tf_buffer_.lookupTransform(params_.wrench_frame_id, attraction_frame_id, tf2::TimePointZero);
    tf2::convert(tf_attraction_msg.transform, tf_attraction);
  }

  // Gradients
  cv::Mat1f dx, dy;
  cv::Scharr(mat, dx, CV_32F, 1, 0);
  cv::Scharr(mat, dy, CV_32F, 0, 1);
  cv::medianBlur(dx, dx, 3);
  cv::medianBlur(dy, dy, 3);

  // Erode mask
  cv::Mat1b eroded_mask;
  cv::erode(mask, eroded_mask, cv::Mat1b::ones(5, 5));

  // Avoid too many visual elements
  constexpr uint8_t skip_factor = 3;

  for (size_t i = 0; i < height_; ++i) {
    auto mat_row = mat[i];
    auto dx_row = dx[i];
    auto dy_row = dy[i];
    auto mask_row = eroded_mask[i];

    for (size_t j = 0; j < width_; ++j) {
      if (!mask_row[j]) {
        continue;
      }

      assert(std::isfinite(mat_row[j]));

      const auto origin = tf_attraction.getOrigin();
      const tf2::Vector3 r_sensor{j * pitch_ - center_x, i * pitch_ - center_y, mat_row[j]};

      // Force calculation
      const auto r = tf_sensor * r_sensor;
      const auto f = r - origin;
      force += f;

      // Torque calculation
      const tf2::Vector3 shift{dx_row[j], dy_row[j], 0};
      const auto r_shift = tf_sensor * (r_sensor + shift);
      const auto f_shift = r_shift - origin;
      torque += r_shift.cross(f_shift);

      // Visual stuff
      if (!(i % skip_factor || j % skip_factor)) {
        const auto r_attraction = tf_attraction.inverse() * r_shift;
        visual_cloud->emplace_back(r_attraction.x(), r_attraction.y(), r_attraction.z());
      }
    }
  }

  return {force * force_gain, torque * force_gain};
}

Marker VirtualWrench::visualize_attraction(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, int32_t marker_id) const
{
  Marker marker;

  marker.ns = this->get_namespace();
  marker.id = marker_id;
  marker.type = Marker::LINE_LIST;

  if (cloud.empty()) {
    marker.action = Marker::DELETE;
  } else {
    marker.action = Marker::ADD;
    marker.scale.x = 0.0002;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 0.7;

    geometry_msgs::msg::Point origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;

    marker.points.reserve(cloud.size());
    for (const auto & e : cloud) {
      marker.points.emplace_back(origin);
      geometry_msgs::msg::Point point;
      point.x = e.x;
      point.y = e.y;
      point.z = e.z;
      marker.points.emplace_back(point);
    }
  }

  return marker;
}

}  // namespace prox::attraction
