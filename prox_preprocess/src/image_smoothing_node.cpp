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

#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "image_smoothing_params.hpp"

namespace prox::preprocess
{
using sensor_msgs::msg::Image;
using std::placeholders::_1;

class ImageSmoothing : public rclcpp::Node
{
  std::shared_ptr<image_smoothing::ParamListener> param_listener_;
  image_smoothing::Params params_;

  rclcpp::Subscription<Image>::SharedPtr subscription_;
  rclcpp::Publisher<Image>::SharedPtr publisher_;

  cv::Mat1f img_;

public:
  explicit ImageSmoothing(const rclcpp::NodeOptions & options) : Node("image_smoothing", options)
  {
    param_listener_ =
      std::make_shared<image_smoothing::ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

    subscription_ = create_subscription<Image>(
      "input/image", rclcpp::SensorDataQoS(), std::bind(&ImageSmoothing::topic_callback, this, _1));
    publisher_ = create_publisher<Image>("image", rclcpp::SensorDataQoS());
  }

private:
  void topic_callback(const Image::SharedPtr input_msg)
  {
    if (param_listener_->is_old(params_)) {
      params_ = param_listener_->get_params();
    }

    if (publisher_->get_subscription_count() == 0) {
      return;
    }

    cv::Mat1f input_img = cv_bridge::toCvShare(input_msg)->image;
    if (img_.size() != input_img.size()) {
      // Fill with NaN
      img_ = cv::Mat1f(input_img.size(), NAN);
    }
    assert(img_.size == input_img.size);

    cv::medianBlur(input_img, input_img, 3);

    img_.forEach([&](float & pixel, const int * position) {
      const auto & input_pixel = input_img.at<float>(position[0], position[1]);

      if (std::isnan(input_pixel) || input_pixel < .02) {
        pixel = NAN;
      } else if (std::isnan(pixel)) {
        pixel = input_pixel;
      } else {
        // EMA calculation
        pixel = input_pixel * params_.filter_coefficient + pixel * (1 - params_.filter_coefficient);
      }
    });

    cv::Mat1f output_img;
    cv::medianBlur(img_, output_img, 3);

    auto output_msg = *input_msg;
    output_msg.data =
      std::vector<uint8_t>(output_img.data, output_img.data + output_msg.step * output_msg.height);
    publisher_->publish(output_msg);
  }
};
}  // namespace prox::preprocess

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prox::preprocess::ImageSmoothing)
