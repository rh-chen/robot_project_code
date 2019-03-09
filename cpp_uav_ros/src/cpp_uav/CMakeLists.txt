cmake_minimum_required(VERSION 2.8.3)
project(cpp_uav)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  roslint
	nav_msgs
)

find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)

include( ${CGAL_USE_FILE} )
include( CGAL_CreateSingleSourceCGALProgram )

add_service_files(
  FILES
  Torres16.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
	nav_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs
)

catkin_package()
message(STATUS "OpenCV_INCLUDE_DIRS:" ${OpenCV_INCLUDE_DIRS})
message(STATUS "CGAL_INCLUDE_DIRS:" ${CGAL_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

# lint
roslint_cpp()
roslint_python()

# Executables service
add_executable(zig_zag_cpp_service src/zig_zag_cpp_service.cpp)

add_dependencies(zig_zag_cpp_service ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(zig_zag_cpp_service ${catkin_LIBRARIES} ${OpenCV_LIBS})
# Executables client
add_executable(zig_zag_cpp_client src/zig_zag_cpp_client.cpp)

add_dependencies(zig_zag_cpp_client ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(zig_zag_cpp_client ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(zig_zag src/zig_zag.cpp)

add_dependencies(zig_zag ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(zig_zag ${catkin_LIBRARIES} ${OpenCV_LIBS} CGAL)
# Tests
#catkin_add_gtest(test_zig_zag_cpp_service test/test_zig_zag_cpp_service.cpp)
#catkin_add_gtest(test_cgutil test/test_cgutil.cpp)

#target_link_libraries(test_zig_zag_cpp_service ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(test_cgutil ${catkin_LIBRARIES} ${OpenCV_LIBS})
