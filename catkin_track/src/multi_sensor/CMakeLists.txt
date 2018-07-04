cmake_minimum_required(VERSION 2.8.3)
project(multi_sensor)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  image_geometry
  tf
  camera_info_manager
  visualization_msgs
  nodelet
  geometry_msgs
  nav_msgs
  kobuki_msgs

)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sonar_publisher src/sonar_publisher.cpp)
target_link_libraries(sonar_publisher  ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(initialpose_publisher src/initialpose_publisher.cpp)
target_link_libraries(initialpose_publisher  ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(imu_publisher src/imu_publisher.cpp)
target_link_libraries(imu_publisher  ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(rviz_marker src/rviz_marker.cpp)
target_link_libraries(rviz_marker  ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(imu_pro_publisher src/imu_pro_publisher.cpp)
target_link_libraries(imu_pro_publisher  ${catkin_LIBRARIES} ${OpenCV_LIBS} )


add_executable(ir_state_publisher src/ir_state_publisher.cpp)
target_link_libraries(ir_state_publisher  ${catkin_LIBRARIES} ${OpenCV_LIBS} )


add_executable(odom_fusion src/odom_fusion.cpp)
target_link_libraries(odom_fusion  ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(odom_fusion_new src/odom_fusion_new.cpp)
target_link_libraries(odom_fusion_new  ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(odom_fake src/odom_fake.cpp)
target_link_libraries(odom_fake  ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(odom_sub src/odom_sub.cpp)
target_link_libraries(odom_sub  ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(path_pub src/path_pub.cpp)
target_link_libraries(path_pub  ${catkin_LIBRARIES} ${OpenCV_LIBS} )


