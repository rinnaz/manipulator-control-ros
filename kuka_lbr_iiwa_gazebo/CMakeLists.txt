cmake_minimum_required(VERSION 3.0.2)
project(kuka_lbr_iiwa_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rm_gazebo
   CATKIN_DEPENDS xacro
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch sdf urdf worlds)
    install(DIRECTORY ${dir}/ 
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
    )
endforeach(dir)

