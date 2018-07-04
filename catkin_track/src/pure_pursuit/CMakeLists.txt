cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  ackermann_msgs
  tf2
  tf2_ros
  tf
  rospy
  dynamic_reconfigure
  message_generation
)

find_package(orocos_kdl REQUIRED)

add_service_files(
  FILES
  myntMoveTrack.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    )

############################
## dynamic reconfiure api ##
############################
generate_dynamic_reconfigure_options(
  cfg/PurePursuit.cfg
  cfg/myntMoveTrackService.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs ackermann_msgs tf2 tf2_ros rospy dynamic_reconfigure
)


###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(pure_pursuit src/pure_pursuit.cpp)
add_executable(pure_pursuit_alter src/pure_pursuit_alter.cpp)
add_executable(mynt_move_track_service src/mynt_move_track_service.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pure_pursuit ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
target_link_libraries(pure_pursuit_alter ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
target_link_libraries(mynt_move_track_service ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

## Make sure configure headers are built before any node using them
add_dependencies(pure_pursuit ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pure_pursuit_alter ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(mynt_move_track_service ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS lattice_planner lattice_planner_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lattice_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
