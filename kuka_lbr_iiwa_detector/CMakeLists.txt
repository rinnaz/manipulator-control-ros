cmake_minimum_required(VERSION 3.0.2)
project(kuka_lbr_iiwa_detector)

find_package(catkin REQUIRED COMPONENTS
  rnrt_aruco_detector
)

## System dependencies are found with CMake's conventions

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_converter
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
)



