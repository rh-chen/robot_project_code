cmake_minimum_required(VERSION 2.8.3)
project(ram_qt_guis)
add_compile_options(-std=c++11 -Wall -Wextra)

SET(CMAKE_BUILD_TYPE Release)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  class_loader
  eigen_conversions
  geometry_msgs
  ram_display
  ram_modify_trajectory
  ram_msgs
  ram_path_planning
  ram_utils
  roscpp
  rqt_gui
  rviz
  tf2_ros
  tf_conversions
)

## Find Qt5 modules
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

set(
  PROJECT_QT_PANELS_TARGETS
  ${PROJECT_NAME}_display
  ${PROJECT_NAME}_entry_exit_strategies
  ${PROJECT_NAME}_fill_trajectory
  ${PROJECT_NAME}_frames
  ${PROJECT_NAME}_modify
  ${PROJECT_NAME}_path_planning
  ${PROJECT_NAME}_pose_info
  ${PROJECT_NAME}_traj_info
  ${PROJECT_NAME}_trajectory_utils
)

set(
  PROJECT_TARGETS
  ${PROJECT_NAME}_modify_panel_modify_poses
  ${PROJECT_NAME}_modify_panel_range_list_selection
  ${PROJECT_NAME}_path_planning_contours_widget
  ${PROJECT_NAME}_path_planning_donghong_ding_widget
  ${PROJECT_NAME}_path_planning_follow_poses_widget
  ${PROJECT_NAME}_path_planning_progress_dialog
  ${PROJECT_NAME}_pose
  ${PROJECT_QT_PANELS_TARGETS}
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
  LIBRARIES
  ${PROJECT_TARGETS}
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  ram_msgs
  roscpp
  rqt_gui
  rviz
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Pose widget
qt5_wrap_cpp(
  pose_MOCS
  include/${PROJECT_NAME}/pose.hpp
)
add_library(
  ${PROJECT_NAME}_pose
  ${pose_MOCS}
  src/pose.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_pose
  Qt5::Widgets
  Qt5::Gui
  ${catkin_LIBRARIES}
)

## Qt panel - Use frame
qt5_wrap_cpp(
  frames_MOCS
  include/${PROJECT_NAME}/frames.hpp
)
add_library(
  ${PROJECT_NAME}_frames
  ${frames_MOCS}
  src/frames.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_frames
  ${PROJECT_NAME}_pose
  Qt5::Widgets
  Qt5::Gui
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_frames
  ${PROJECT_NAME}_pose
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Display
qt5_wrap_cpp(
  display_MOCS
  include/${PROJECT_NAME}/display.hpp
)
add_library(
  ${PROJECT_NAME}_display
  ${display_MOCS}
  src/display.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_display
  Qt5::Widgets
  Qt5::Gui
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_display
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Path planning
qt5_wrap_cpp(
  path_planning_progress_dialog_MOCS
  include/${PROJECT_NAME}/path_planning_widgets/progress_dialog.hpp
)
add_library(
  ${PROJECT_NAME}_path_planning_progress_dialog
  ${path_planning_progress_dialog_MOCS}
  src/path_planning_widgets/progress_dialog.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning_progress_dialog
  Qt5::Widgets
  Qt5::Gui
)
add_dependencies(
  ${PROJECT_NAME}_path_planning_progress_dialog
  ${catkin_EXPORTED_TARGETS}
)

qt5_wrap_cpp(
  path_planning_donghong_ding_widget_MOCS
  include/${PROJECT_NAME}/path_planning_widgets/donghong_ding.hpp
)
add_library(
  ${PROJECT_NAME}_path_planning_donghong_ding_widget
  ${path_planning_donghong_ding_widget_MOCS}
  src/path_planning_widgets/donghong_ding.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning_donghong_ding_widget
  Qt5::Widgets
  Qt5::Gui
)
add_dependencies(
  ${PROJECT_NAME}_path_planning_donghong_ding_widget
  ${catkin_EXPORTED_TARGETS}
)

qt5_wrap_cpp(
  path_planning_contours_widget_MOCS
  include/${PROJECT_NAME}/path_planning_widgets/contours.hpp
)
add_library(
  ${PROJECT_NAME}_path_planning_contours_widget
  ${path_planning_contours_widget_MOCS}
  src/path_planning_widgets/contours.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning_contours_widget
  Qt5::Widgets
  Qt5::Gui
)
add_dependencies(
  ${PROJECT_NAME}_path_planning_contours_widget
  ${catkin_EXPORTED_TARGETS}
)

qt5_wrap_cpp(
  path_planning_follow_poses_widget_MOCS
  include/${PROJECT_NAME}/path_planning_widgets/follow_poses.hpp
)
add_library(
  ${PROJECT_NAME}_path_planning_follow_poses_widget
  ${path_planning_follow_poses_widget_MOCS}
  src/path_planning_widgets/follow_poses.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning_follow_poses_widget
  Qt5::Widgets
  Qt5::Gui
)
add_dependencies(
  ${PROJECT_NAME}_path_planning_follow_poses_widget
  ${catkin_EXPORTED_TARGETS}
)

qt5_wrap_cpp(
  path_planning_MOCS
  include/${PROJECT_NAME}/path_planning.hpp
)

add_library(
  ${PROJECT_NAME}_path_planning
  ${path_planning_MOCS}
  src/path_planning.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_path_planning
  ${PROJECT_NAME}_path_planning_progress_dialog
  ${PROJECT_NAME}_path_planning_contours_widget
  ${PROJECT_NAME}_path_planning_donghong_ding_widget
  ${PROJECT_NAME}_path_planning_follow_poses_widget
  Qt5::Widgets
  Qt5::Gui
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_path_planning
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Fill trajectory parameters
qt5_wrap_cpp(
  fill_trajectory_MOCS
  include/${PROJECT_NAME}/fill_trajectory.hpp
)

add_library(
  ${PROJECT_NAME}_fill_trajectory
  ${fill_trajectory_MOCS}
  src/fill_trajectory.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_fill_trajectory
  Qt5::Widgets
  Qt5::Gui
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_fill_trajectory
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Trajectory utils
qt5_wrap_cpp(
  fill_trajectory_MOCS
  include/${PROJECT_NAME}/trajectory_utils.hpp
)

add_library(
  ${PROJECT_NAME}_trajectory_utils
  ${fill_trajectory_MOCS}
  src/trajectory_utils.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_trajectory_utils
  Qt5::Widgets
  Qt5::Gui
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_trajectory_utils
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Pose info panel
qt5_wrap_cpp(
  pose_info_MOCS
  include/${PROJECT_NAME}/pose_info.hpp
)
add_library(
  ${PROJECT_NAME}_pose_info
  ${pose_info_MOCS}
  src/pose_info.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_pose_info
  ${PROJECT_NAME}_pose
  Qt5::Widgets
  Qt5::Gui
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_pose_info
  ${PROJECT_NAME}_pose
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Info panel
qt5_wrap_cpp(
  fill_trajectory_MOCS
  include/${PROJECT_NAME}/traj_info.hpp
)
add_library(
  ${PROJECT_NAME}_traj_info
  ${fill_trajectory_MOCS}
  src/traj_info.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_traj_info
  Qt5::Widgets
  Qt5::Gui
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_traj_info
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Modify trajectory
qt5_wrap_cpp(
  modify_panel_range_list_selection_MOCS
  include/${PROJECT_NAME}/modify_widgets/range_list_selection.hpp
)
add_library(
  ${PROJECT_NAME}_modify_panel_range_list_selection
  ${modify_panel_range_list_selection_MOCS}
  src/modify_widgets/range_list_selection.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_modify_panel_range_list_selection
  Qt5::Widgets
  Qt5::Gui
)

qt5_wrap_cpp(
  modify_panel_modify_poses_MOCS
  include/${PROJECT_NAME}/modify_widgets/modify_poses.hpp
)
add_library(
  ${PROJECT_NAME}_modify_panel_modify_poses
  ${modify_panel_modify_poses_MOCS}
  src/modify_widgets/modify_poses.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_modify_panel_modify_poses
  ${PROJECT_NAME}_pose
  Qt5::Widgets
  Qt5::Gui
)
add_dependencies(
  ${PROJECT_NAME}_modify_panel_modify_poses
  ${PROJECT_NAME}_pose
  ${catkin_EXPORTED_TARGETS}
)

qt5_wrap_cpp(
  modify_trajectory_MOCS
  include/${PROJECT_NAME}/modify.hpp
)
add_library(
  ${PROJECT_NAME}_modify
  ${modify_trajectory_MOCS}
  src/modify.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_modify
  ${PROJECT_NAME}_modify_panel_range_list_selection
  ${PROJECT_NAME}_modify_panel_modify_poses
  Qt5::Widgets
  Qt5::Gui
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_modify
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel - Entry exit strategies
qt5_wrap_cpp(
  entry_exit_strategies_MOCS
  include/${PROJECT_NAME}/entry_exit_strategies.hpp
)
add_library(
  ${PROJECT_NAME}_entry_exit_strategies
  ${entry_exit_strategies_MOCS}
  src/entry_exit_strategies.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_entry_exit_strategies
  Qt5::Widgets
  Qt5::Gui
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_entry_exit_strategies
  ${catkin_EXPORTED_TARGETS}
)

# Needed by Qt / RViz / ROS
class_loader_hide_library_symbols(
  ${PROJECT_QT_PANELS_TARGETS}
)

#############
## Install ##
#############

if("${CMAKE_INSTALL_PREFIX}" MATCHES "^/opt/")
  # Handle release case
  set(DESKTOP_APP_DIR "/usr/share/applications")
  set(DESKTOP_ICON_DIR "/usr/share/icons")
  set(DESKTOP_APP_SUFFIX "")
else()
  set(DESKTOP_APP_DIR "$ENV{HOME}/.local/share/applications")
  set(DESKTOP_ICON_DIR ${DESKTOP_APP_DIR})
  set(DESKTOP_APP_SUFFIX " (debug)")
endif()
set(ROS_DISTRO $ENV{ROS_DISTRO})

# Create directory for the desktop file/icon
install(DIRECTORY DESTINATION
  ${DESKTOP_APP_DIR}
)
install(DIRECTORY DESTINATION
  ${DESKTOP_ICON_DIR}
)

# Configure bash script
configure_file(
  scripts/ros_additive_manufacturing.bash.in
  scripts/ros_additive_manufacturing.bash
  @ONLY
)

# Install bash scripts
install(
  PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/scripts/ros_additive_manufacturing.bash
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Configure / install desktop launchers
configure_file(
  desktop/ros_additive_manufacturing.desktop.in
  desktop/ros_additive_manufacturing.desktop
)
install(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/desktop/ros_additive_manufacturing.desktop
  DESTINATION
  ${DESKTOP_APP_DIR}
)

# Install icon
install(
  FILES
  desktop/ros_additive_manufacturing.png
  DESTINATION
  ${DESKTOP_ICON_DIR}
)

## Mark executables and/or libraries for installation
# Qt panel
install(
  TARGETS
  ${PROJECT_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION
  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  config
  launch
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  FILES
  plugin.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
