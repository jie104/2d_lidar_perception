cmake_minimum_required(VERSION 3.0.2)
project(laser_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
        INCLUDE_DIRS include
#        LIBRARIES percipio_wrapper
#        CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_runtime
#        DEPENDS libtycam
)

include_directories(include
        ${catkin_INCLUDE_DIRS}
#        ${Boost_INCLUDE_DIRS}
        )

add_library(laser_detect_pallet_lib
        src/laser_detect_pallet.cpp)


add_executable(${PROJECT_NAME}_node src/2d_lidar_perception_node.cpp  )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        laser_detect_pallet_lib
        glog
        )



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
## include
#  ${catkin_INCLUDE_DIRS}
#)

