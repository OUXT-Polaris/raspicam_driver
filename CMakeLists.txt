cmake_minimum_required(VERSION 3.5)
project(raspicam_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(diagnostic_updater REQUIRED)

find_package(raspicam REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
)

add_library(raspicam_driver_component SHARED
  src/raspicam_driver_component.cpp)
target_compile_definitions(raspicam_driver_component
  PRIVATE "RASPICAM_DRIVER_RASPICAM_DRIVER_COMPONENT_BUILDING_DLL")
ament_target_dependencies(raspicam_driver_component
  rclcpp
  rclcpp_components
  image_transport
  camera_calibration_parsers
  diagnostic_updater
  cv_bridge)
rclcpp_components_register_nodes(raspicam_driver_component
  "raspicam_drive::RaspiCamDriverComponent")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
