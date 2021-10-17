cmake_minimum_required(VERSION 3.0.2)
project(kuka_lbr_iiwa_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning_interface
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES 
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(iiwa_controller src/iiwa_controller.cpp)
target_link_libraries(iiwa_controller ${catkin_LIBRARIES})

add_executable(iiwa_controller_node src/main.cpp)

target_link_libraries(iiwa_controller_node
  iiwa_controller
  ${catkin_LIBRARIES}
)
