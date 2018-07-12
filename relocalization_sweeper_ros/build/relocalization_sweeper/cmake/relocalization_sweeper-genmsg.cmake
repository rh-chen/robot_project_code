# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "relocalization_sweeper: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(relocalization_sweeper_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv" NAME_WE)
add_custom_target(_relocalization_sweeper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "relocalization_sweeper" "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv" "geometry_msgs/PoseStamped:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:sensor_msgs/Image:geometry_msgs/Pose"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(relocalization_sweeper
  "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/relocalization_sweeper
)

### Generating Module File
_generate_module_cpp(relocalization_sweeper
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/relocalization_sweeper
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(relocalization_sweeper_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(relocalization_sweeper_generate_messages relocalization_sweeper_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv" NAME_WE)
add_dependencies(relocalization_sweeper_generate_messages_cpp _relocalization_sweeper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(relocalization_sweeper_gencpp)
add_dependencies(relocalization_sweeper_gencpp relocalization_sweeper_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS relocalization_sweeper_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(relocalization_sweeper
  "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/relocalization_sweeper
)

### Generating Module File
_generate_module_eus(relocalization_sweeper
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/relocalization_sweeper
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(relocalization_sweeper_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(relocalization_sweeper_generate_messages relocalization_sweeper_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv" NAME_WE)
add_dependencies(relocalization_sweeper_generate_messages_eus _relocalization_sweeper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(relocalization_sweeper_geneus)
add_dependencies(relocalization_sweeper_geneus relocalization_sweeper_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS relocalization_sweeper_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(relocalization_sweeper
  "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/relocalization_sweeper
)

### Generating Module File
_generate_module_lisp(relocalization_sweeper
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/relocalization_sweeper
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(relocalization_sweeper_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(relocalization_sweeper_generate_messages relocalization_sweeper_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv" NAME_WE)
add_dependencies(relocalization_sweeper_generate_messages_lisp _relocalization_sweeper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(relocalization_sweeper_genlisp)
add_dependencies(relocalization_sweeper_genlisp relocalization_sweeper_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS relocalization_sweeper_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(relocalization_sweeper
  "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/relocalization_sweeper
)

### Generating Module File
_generate_module_nodejs(relocalization_sweeper
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/relocalization_sweeper
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(relocalization_sweeper_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(relocalization_sweeper_generate_messages relocalization_sweeper_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv" NAME_WE)
add_dependencies(relocalization_sweeper_generate_messages_nodejs _relocalization_sweeper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(relocalization_sweeper_gennodejs)
add_dependencies(relocalization_sweeper_gennodejs relocalization_sweeper_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS relocalization_sweeper_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(relocalization_sweeper
  "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/relocalization_sweeper
)

### Generating Module File
_generate_module_py(relocalization_sweeper
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/relocalization_sweeper
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(relocalization_sweeper_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(relocalization_sweeper_generate_messages relocalization_sweeper_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wzm/relocalization_sweeper_ros/src/relocalization_sweeper/srv/GetRobotPose.srv" NAME_WE)
add_dependencies(relocalization_sweeper_generate_messages_py _relocalization_sweeper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(relocalization_sweeper_genpy)
add_dependencies(relocalization_sweeper_genpy relocalization_sweeper_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS relocalization_sweeper_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/relocalization_sweeper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/relocalization_sweeper
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(relocalization_sweeper_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(relocalization_sweeper_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(relocalization_sweeper_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(relocalization_sweeper_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/relocalization_sweeper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/relocalization_sweeper
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(relocalization_sweeper_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(relocalization_sweeper_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(relocalization_sweeper_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(relocalization_sweeper_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/relocalization_sweeper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/relocalization_sweeper
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(relocalization_sweeper_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(relocalization_sweeper_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(relocalization_sweeper_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(relocalization_sweeper_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/relocalization_sweeper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/relocalization_sweeper
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(relocalization_sweeper_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(relocalization_sweeper_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(relocalization_sweeper_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(relocalization_sweeper_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/relocalization_sweeper)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/relocalization_sweeper\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/relocalization_sweeper
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(relocalization_sweeper_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(relocalization_sweeper_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(relocalization_sweeper_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(relocalization_sweeper_generate_messages_py std_msgs_generate_messages_py)
endif()
