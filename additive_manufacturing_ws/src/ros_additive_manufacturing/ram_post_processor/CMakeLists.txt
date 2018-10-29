cmake_minimum_required(VERSION 2.8.3)
project(ram_post_processor)
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  ram_msgs
  ram_utils
  roscpp
  tf2_ros
  tf_conversions
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  geometry_msgs
  ram_msgs
  LIBRARIES
  ${PROJECT_NAME}_post_processor
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}_post_processor
  src/post_processor.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_post_processor
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_post_processor
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}_post_processor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION
  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
