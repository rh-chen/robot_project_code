cmake_minimum_required(VERSION 2.8.3)
project(ram_msgs)
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  genmsg
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  std_msgs
  uuid_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  AdditiveManufacturingParams.msg
  AdditiveManufacturingPose.msg
  AdditiveManufacturingTrajectory.msg
  AdditiveManufacturingTrajectoryInfo.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  uuid_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  genmsg
  geometry_msgs
  message_runtime
  std_msgs
  uuid_msgs
)

#############
## Install ##
#############

# Messages are installed automatically by catkin

#############
## Testing ##
#############
