# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/usr/src/googletest/googlemock"
  "/home/embody/ros_ws/src/G1-ROS-Navigation/catkin_ws/src/livox_ros_driver/livox_ros_driver/build/gmock"
  "/home/embody/ros_ws/src/G1-ROS-Navigation/catkin_ws/src/livox_ros_driver/livox_ros_driver/build/GMock-prefix"
  "/home/embody/ros_ws/src/G1-ROS-Navigation/catkin_ws/src/livox_ros_driver/livox_ros_driver/build/GMock-prefix/tmp"
  "/home/embody/ros_ws/src/G1-ROS-Navigation/catkin_ws/src/livox_ros_driver/livox_ros_driver/build/GMock-prefix/src/GMock-stamp"
  "/home/embody/ros_ws/src/G1-ROS-Navigation/catkin_ws/src/livox_ros_driver/livox_ros_driver/build/GMock-prefix/src"
  "/home/embody/ros_ws/src/G1-ROS-Navigation/catkin_ws/src/livox_ros_driver/livox_ros_driver/build/GMock-prefix/src/GMock-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/embody/ros_ws/src/G1-ROS-Navigation/catkin_ws/src/livox_ros_driver/livox_ros_driver/build/GMock-prefix/src/GMock-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/embody/ros_ws/src/G1-ROS-Navigation/catkin_ws/src/livox_ros_driver/livox_ros_driver/build/GMock-prefix/src/GMock-stamp${cfgdir}") # cfgdir has leading slash
endif()
