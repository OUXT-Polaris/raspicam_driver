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
 * @mainpage view_all node for color_names
 * @image html images/logo.png
 * @author Masaya Kataoka
 * @date 2020-07-24
 * @brief ROS2 component for publishing images by using raspicam
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
#include <camera_calibration_parsers/parse_yml.h>
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
  /**
   * @brief if the durtion between trigger message timestamp
   *  and current ROS timestamp is over trigger duration, this component does not capture image
   */
  double trigger_duration_;
  /**
   * @brief callback function for capture images, if the durtion between message timestamp
   *  and current ROS timestamp is over trigger duration, this component does not capture image
   * @param msg
   */
  void triggerCallback(const builtin_interfaces::msg::Time::SharedPtr msg);
  /**
   * @brief callback function for captureimg images (it only works in timer mode)
   */
  void timerCallback();
  /**
   * @brief callback function for diagnostics (it only works in timer mode)
   */
  void diagCallback();
  bool captureImage(rclcpp::Time stamp);
  /**
   * @brief if true, publish image as sensor_msgs/msg/CompressedImage (jpeg format)
   */
  bool compress_;
  /**
   * @brief capture duration in timer mode (ms)
   */
  int capture_duration_;
  /**
   * @brief parameter for compressing images by jpeg method
   */
  std::vector<int> params_;
  /**
   * @brief ROS Publishr for publishing compressed image
   */
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
  /**
   * @brief ROS Publisher for raw image
   */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  /**
   * @brief timer for the captureimg images
   */
  rclcpp::TimerBase::SharedPtr timer_;
  /**
   * @brief timer for diagnostics
   */
  rclcpp::TimerBase::SharedPtr diag_timer_;
  /**
   * @brief utility class for updating diagnostics
   * @ref diag_updater_
   */
  diagnostic_updater::Updater diag_updater_;
  /**
   * @brief callback function for checking diagnostics in timer mode
   * @param status this argument passed to the diag_updater_
   * @sa diag_updater_
   */
  void captureRateDiag(diagnostic_updater::DiagnosticStatusWrapper & status);
  /**
   * @brief ROS subscriber for trigger
   */
  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr trigger_sub_;
  /**
   * @brief the time which we capture latest images
   */
  rclcpp::Time last_capture_time_;
  /**
   * @brief full path of the calibration yaml file
   */
  std::string camera_info_yaml_path_;
  /**
   * @brief name pf the camera
   */
  std::string camera_name_;
  /**
   * @brief camera info of the raspicam
   */
  sensor_msgs::msg::CameraInfo camera_info_;
  /**
   * @brief ROS publisher for camera info topic
   */
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};
}  // namespace raspicam_driver

#endif  // RASPICAM_DRIVER__RASPICAM_DRIVER_COMPONENT_HPP_
