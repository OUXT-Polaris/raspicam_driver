// Copyright (c) 2020 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <raspicam_driver/raspicam_driver_component.hpp>

#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

namespace raspicam_driver
{
RaspiCamDriverComponent::RaspiCamDriverComponent(const rclcpp::NodeOptions & options)
: Node("raspicam_driver", options)
{
  Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
  declare_parameter("enable_trigger", false);
  get_parameter("enable_trigger", enable_trigger_);
  declare_parameter("trigger_duration", 0.01);
  get_parameter("trigger_duration", trigger_duration_);
  declare_parameter("compress", true);
  get_parameter("compress", compress_);
  declare_parameter("frame_id", "front_camera");
  get_parameter("frame_id", frame_id_);
  declare_parameter("capture_frequency", 30);
  get_parameter("capture_duration", capture_duration_);
  std::vector<int> params_ = std::vector<int>(2);
  params_[0] = CV_IMWRITE_JPEG_QUALITY;
  declare_parameter("jpeg_quality", 75);
  get_parameter("jpeg_quality", params_[0]);
  if (compress_) {
    compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "~/image_raw/compressed", 1);
  } else {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/image_raw", 1);
  }
  timer_ =
    create_wall_timer(std::chrono::milliseconds(capture_duration_),
      std::bind(&RaspiCamDriverComponent::timerCallback, this));
}

RaspiCamDriverComponent::~RaspiCamDriverComponent()
{
  Camera.release();
}

bool RaspiCamDriverComponent::captureImage(rclcpp::Time stamp)
{
  Camera.grab();
  cv::Mat image;
  Camera.retrieve(image);
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  cv_ptr->header.stamp = stamp;
  cv_ptr->header.frame_id = frame_id_;
  if (compress_) {
    sensor_msgs::msg::CompressedImage compressed;
    try {
      if (cv::imencode(".jpg", image, compressed.data, params_)) {
        compressed_image_pub_->publish(compressed);
      }
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return false;
    } catch (cv::Exception & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return false;
    }
  } else {
    cv_ptr->image = image;
    image_pub_->publish(*cv_ptr->toImageMsg());
    return true;
  }
  return false;
}

void RaspiCamDriverComponent::timerCallback()
{
  captureImage(get_clock()->now());
}

void RaspiCamDriverComponent::triggerCallback(const builtin_interfaces::msg::Time::SharedPtr msg)
{
  rclcpp::Duration duration = get_clock()->now() - *msg;
  double seconds = duration.seconds();
  if (std::fabs(seconds) > trigger_duration_) {
    return;
  }
  captureImage(*msg);
}
}  // namespace raspicam_driver

RCLCPP_COMPONENTS_REGISTER_NODE(raspicam_driver::RaspiCamDriverComponent)
