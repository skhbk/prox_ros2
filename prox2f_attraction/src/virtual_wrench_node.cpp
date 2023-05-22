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

  std::vector<Grid::ConstSharedPtr> msgs{grid_msg1, grid_msg2};

  tf2::Vector3 force{0, 0, 0}, torque{0, 0, 0};
  for (size_t i = 0; i < msgs.size(); ++i) {
    std::vector<tf2::Vector3> positions, gradients;

    this->get_vectors(*msgs[i], positions, gradients);

    // Compute wrench
    const size_t n_force_lines = positions.size();
    tf2::Vector3 local_force{0, 0, 0}, local_torque{0, 0, 0};
    for (size_t n = 0; n < n_force_lines; ++n) {
      const auto wrench = this->compute_wrench(positions[n], gradients[n]);
      local_force += wrench[0];
      local_torque += wrench[1];
    }
    if (n_force_lines > 0) {
      local_force /= n_force_lines;
      local_torque /= n_force_lines;
    }
    force += local_force;
    torque += local_torque;
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

void VirtualWrench::get_vectors(
  const Grid & grid_msg, std::vector<tf2::Vector3> & positions,
  std::vector<tf2::Vector3> & gradients) const
{
  const size_t width = grid_msg.width, height = grid_msg.height;
  const float pitch = grid_msg.pitch;

  // Convert Grid to cv::Mat
  cv::Mat1f mat;
  cv::Mat1b mask;
  {
    constexpr float invalid_value = 1e3;

    if (grid_msg.data.empty()) {
      mat = cv::Mat1f(cv::Size(width, height), invalid_value);
    } else {
      mat = cv::Mat1f(grid_msg.data, true).reshape(1, height);
      // Seems not working
      // cv::patchNaNs(mat, invalid_value);
    }

    // Get mask
    mask = mat < invalid_value;
  }

  // Compute gradients
  cv::Mat1f dx, dy;
  {
    const float scale = params_.gradient_scale / pitch;
    cv::Scharr(mat, dx, CV_32F, 1, 0, scale);
    cv::Scharr(mat, dy, CV_32F, 0, 1, scale);
    cv::medianBlur(dx, dx, 3);
    cv::medianBlur(dy, dy, 3);
  }

  // Erode mask
  cv::Mat1b eroded_mask;
  cv::erode(mask, eroded_mask, cv::Mat1b::ones(5, 5));

  // Get transform to the wrench frame
  tf2::Transform tf_sensor;
  {
    const auto tf_sensor_msg = tf_buffer_.lookupTransform(
      params_.wrench_frame_id, grid_msg.header.frame_id, tf2::TimePointZero);
    tf2::convert(tf_sensor_msg.transform, tf_sensor);
  }

  const float center_x = width * pitch / 2;
  const float center_y = height * pitch / 2;

  // Clear and reserve vectors
  {
    const auto n_valid_points = cv::countNonZero(eroded_mask);
    positions.clear();
    positions.reserve(n_valid_points);
    gradients.clear();
    gradients.reserve(n_valid_points);
  }

  for (size_t i = 0; i < height; ++i) {
    const auto mat_row = mat[i];
    const auto dx_row = dx[i];
    const auto dy_row = dy[i];
    const auto mask_row = eroded_mask[i];

    for (size_t j = 0; j < width; ++j) {
      if (!mask_row[j]) {
        continue;
      }

      assert(std::isfinite(mat_row[j]));

      // Compute position vector
      const tf2::Vector3 p_sensor{j * pitch - center_x, i * pitch - center_y, mat_row[j]};
      const auto p = tf_sensor * p_sensor;
      positions.push_back(p);

      // Compute gradient vector
      const tf2::Vector3 g_sensor{dx_row[j], dy_row[j], 0};
      auto g = tf_sensor.getBasis() * g_sensor;
      gradients.push_back(g);
    }
  }
}

std::array<tf2::Vector3, 2> VirtualWrench::compute_wrench(
  const tf2::Vector3 & position, const tf2::Vector3 & gradient) const
{
  const auto force = params_.stiffness * position;
  const auto torque = position.cross(params_.stiffness * gradient);

  return {force, torque};
}

}  // namespace prox::attraction
