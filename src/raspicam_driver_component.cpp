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

/**
 * @file raspicam_driver_component.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implementation of the RaspiCamDriverComponent class
 * @version 0.1
 * @date 2020-07-24
 * @copyright Copyright (c) 2020
 *
 */

#include <raspicam_driver/raspicam_driver_component.hpp>

#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>
#include <fstream>

namespace raspicam_driver
{
RaspiCamDriverComponent::RaspiCamDriverComponent(const rclcpp::NodeOptions & options)
: Node("raspicam_driver", options), diag_updater_(this)
{
  Camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);
  declare_parameter("enable_trigger", false);
  get_parameter("enable_trigger", enable_trigger_);
  declare_parameter("trigger_duration", 0.01);
  get_parameter("trigger_duration", trigger_duration_);
  declare_parameter("compress", true);
  get_parameter("compress", compress_);
  declare_parameter("capture_frequency", 30);
  get_parameter("capture_duration", capture_duration_);
  declare_parameter("camera_name", "front_camera");
  get_parameter("camera_name", camera_name_);
  declare_parameter("camera_info_yaml_path", "");
  get_parameter("camera_info_yaml_path", camera_info_yaml_path_);
  if (camera_info_yaml_path_ == "") {
    RCLCPP_ERROR(get_logger(), "parameter camera_info_yaml_path is empty");
    return;
  }
  try {
    std::ifstream ifs(camera_info_yaml_path_, std::ios::in);
    std::string yaml_str;
    ifs >> yaml_str;
    auto ret =
      camera_calibration_parsers::parseCalibrationYml(yaml_str, camera_name_, camera_info_);
    if (!ret) {
      RCLCPP_ERROR(get_logger(), "failed to parse yaml file");
      return;
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    RCLCPP_ERROR(get_logger(), "e.what()");
    return;
  }
  diag_updater_.setHardwareID(camera_name_);
  std::vector<int> params_ = std::vector<int>(2);
  params_[0] = cv::IMWRITE_JPEG_QUALITY;
  declare_parameter("jpeg_quality", 75);
  get_parameter("jpeg_quality", params_[0]);
  if (compress_) {
    compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "~/image_raw/compressed", 1);
  } else {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/image_raw", 1);
  }
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/image_raw", 1);
  if (enable_trigger_) {
    trigger_sub_ = this->create_subscription<builtin_interfaces::msg::Time>(
      "trigger", 1,
      std::bind(&RaspiCamDriverComponent::triggerCallback, this, std::placeholders::_1));
  } else {
    last_capture_time_ = get_clock()->now();
    diag_updater_.add("capture_rate", this, &RaspiCamDriverComponent::captureRateDiag);
    timer_ =
      create_wall_timer(
      std::chrono::milliseconds(capture_duration_),
      std::bind(&RaspiCamDriverComponent::timerCallback, this));
    using namespace std::literals::chrono_literals;
    diag_timer_ =
      create_wall_timer((100ms), std::bind(&RaspiCamDriverComponent::diagCallback, this));
  }
}

RaspiCamDriverComponent::~RaspiCamDriverComponent()
{
  Camera.release();
}

void RaspiCamDriverComponent::diagCallback()
{
  diag_updater_.force_update();
}

void RaspiCamDriverComponent::captureRateDiag(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  auto duration = (get_clock()->now() - last_capture_time_).seconds();
  if (duration > 1.0) {
    status.summaryf(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "cannot capture image in 1 sec.");
  } else if (duration > 0.3) {
    status.summaryf(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "cannot capture image in 0.3 sec.");
  } else {
    status.summaryf(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "raspicam_driver is running in timer mode.");
  }
}

bool RaspiCamDriverComponent::captureImage(rclcpp::Time stamp)
{
  Camera.grab();
  cv::Mat image;
  Camera.retrieve(image);
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  cv_ptr->header.stamp = stamp;
  cv_ptr->header.frame_id = camera_info_.header.frame_id;
  camera_info_.header.stamp = stamp;
  if (compress_) {
    sensor_msgs::msg::CompressedImage compressed;
    try {
      if (cv::imencode(".jpg", image, compressed.data, params_)) {
        compressed_image_pub_->publish(compressed);
        camera_info_pub_->publish(camera_info_);
        return true;
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
    camera_info_pub_->publish(camera_info_);
    return true;
  }
  return false;
}

void RaspiCamDriverComponent::timerCallback()
{
  auto now = get_clock()->now();
  last_capture_time_ = now;
  captureImage(now);
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
