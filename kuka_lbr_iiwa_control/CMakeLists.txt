cmake_minimum_required(VERSION 3.0.2)
project(kuka_lbr_iiwa_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rm_planner
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
