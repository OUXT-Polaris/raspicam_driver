# raspicam_driver ![ROS2-Eloquent](https://github.com/OUXT-Polaris/raspicam_driver/workflows/ROS2-Eloquent/badge.svg)

ROS2 wrapper for the raspicam. (https://github.com/cedricve/raspicam)

![Developed By OUXT Polaris](img/logo.png "Logo")

## components

### raspicam_driver_component

it captures image via raspicam C++ API and publish it as a sensor_msgs/msg/Image or sensor_msgs/msg/CompressedImage

## How to build

1. clone forked raspicam in colcon workspace. (https://github.com/OUXT-Polaris/raspicam) We modify install directory for .cmake file in this fork.  
1. git clone this package in a same workspace. 
1. just run "colcon build"