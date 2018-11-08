cmake_minimum_required(VERSION 2.8.3)
project(ram_path_planning)
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  roslib
  std_msgs
  tf_conversions
  unique_id
  uuid_msgs
  visualization_msgs
  nav_msgs
)
find_package(OpenCV REQUIRED)

find_package(VTK 7.1.0 QUIET)
if (NOT VTK_FOUND)
  find_package(VTK 8.0.0 REQUIRED)
endif()
message(STATUS "vtk version: ${VTK_DIR}")
message(STATUS "include path: ${VTK_INCLUDE_DIRS}")
include(${VTK_USE_FILE})

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
	FILES
	Cpp.srv
	MapRotate.srv
	ModifyMap.srv
)
add_message_files(
	FILES
	AdditiveManufacturingPose.msg
	AdditiveManufacturingTrajectory.msg
)

#add_action_files(
#  FILES
#Contours.action
#  DonghongDing.action
#  FollowPoses.action
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  uuid_msgs
  nav_msgs
)
###################################
## catkin specific configuration ##
###################################

#catkin_package(
#  INCLUDE_DIRS
#  include
#  LIBRARIES
#  ${PROJECT_NAME}_mesh_slicer
#  CATKIN_DEPENDS
#  actionlib
#  actionlib_msgs
#  geometry_msgs
#  message_runtime
#  ram_msgs
#  ram_utils
#  std_msgs
#  uuid_msgs
#  visualization_msgs
#  DEPENDS
#  VTK
#)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Library (mostly VTK) that slices a mesh into layers
#add_library(
#  ${PROJECT_NAME}_mesh_slicer
#  src/mesh_slicer.cpp
#)
#target_link_libraries(
#  ${PROJECT_NAME}_mesh_slicer
#  ${catkin_LIBRARIES}
#  ${VTK_LIBRARIES}
#)
#add_dependencies(
#  ${PROJECT_NAME}_mesh_slicer
#  ${catkin_EXPORTED_TARGETS}
#)

# Service node
add_executable(
  ${PROJECT_NAME}_node
  src/Cpp.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  ${OpenCV_LIBS}
  #${PROJECT_NAME}_mesh_slicer
)
add_dependencies(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(map_modify_node src/map_modify_node.cpp src/lsd.cpp)

target_link_libraries(map_modify_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

add_dependencies(map_modify_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(ram_cpp_node src/ram_cpp_node.cpp)
target_link_libraries(ram_cpp_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

add_dependencies(ram_cpp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(line_detection_and_rotation_node src/line_detection_and_rotation.cpp src/lsd.cpp)

target_link_libraries(line_detection_and_rotation_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

add_dependencies(line_detection_and_rotation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS
#  ${PROJECT_NAME}
 #${PROJECT_NAME}_mesh_slicer
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(
#  DIRECTORY
#  include/${PROJECT_NAME}/
#  DESTINATION
#  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(
#  DIRECTORY
#  launch
#  meshes # Make sure meshes are not too heavy
#  yaml
#  DESTINATION
#  ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(
#    ${PROJECT_NAME}_services_clients
#    test/services_clients.launch
#    test/services_clients.cpp
#  )
#  target_link_libraries(
#    ${PROJECT_NAME}_services_clients
#    ${catkin_LIBRARIES}
#    #${PROJECT_NAME}_mesh_slicer
#    ${VTK_LIBRARIES}
#  )
#  add_dependencies(
#    ${PROJECT_NAME}_services_clients
#    ${PROJECT_NAME}
#    ${PROJECT_NAME}_generate_messages_cpp
#    ${catkin_EXPORTED_TARGETS}
#  )
#endif()
