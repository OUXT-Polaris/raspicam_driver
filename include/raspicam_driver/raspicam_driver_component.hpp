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
 * @file raspicam_driver_component.hpp
 */

/**
 * @mainpage view_all node for color_names
 * @image html images/logo.png
 * @author Masaya Kataoka
 * @date 2020-07-17
 */

#ifndef RASPICAM_DRIVER__RASPICAM_DRIVER_COMPONENT_HPP_
#define RASPICAM_DRIVER__RASPICAM_DRIVER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_EXPORT __attribute__((dllexport))
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_EXPORT __declspec(dllexport)
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_BUILDING_DLL
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_PUBLIC \
  RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_EXPORT
#else
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_PUBLIC \
  RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_IMPORT
#endif
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_PUBLIC_TYPE \
  RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_PUBLIC
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_LOCAL
#else
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_PUBLIC
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_LOCAL
#endif
#define RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <raspicam/raspicam_cv.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>

/**
 * @brief namespace for raspicam driver
 */
namespace raspicam_driver
{
class RaspiCamDriverComponent : public rclcpp::Node
{
public:
  RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_PUBLIC
  explicit RaspiCamDriverComponent(const rclcpp::NodeOptions & options);
  ~RaspiCamDriverComponent();

private:
  /**
   * @brief raspi camera capture class in raspicam library
   */
  raspicam::RaspiCam_Cv Camera;
  /**
   * @brief if true, this node works with trigger mode
   */
  bool enable_trigger_;
  double trigger_duration_;
  void triggerCallback(const builtin_interfaces::msg::Time::SharedPtr msg);
  void timerCallback();
  void diagCallback();
  bool captureImage(rclcpp::Time stamp);
  bool compress_;
  std::string frame_id_;
  int capture_duration_;
  std::vector<int> params_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr raspicam_timer_;
  diagnostic_updater::Updater diag_updater_;
  void captureRateDiag(diagnostic_updater::DiagnosticStatusWrapper & status);
  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr trigger_sub_;
  rclcpp::Time last_capture_time_;
};
}  // namespace raspicam_driver

#endif  // RASPICAM_DRIVER__RASPICAM_DRIVER_COMPONENT_HPP_
