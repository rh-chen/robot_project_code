INCLUDE_DIRECTORIES(include/aruco_marker_detection)

LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES} )

ADD_EXECUTABLE(aruco_test_svm  aruco_test_svm.cpp)
target_link_libraries(aruco_test_svm    aruco_marker_detection opencv_calib3d)

INSTALL(TARGETS aruco_test_svm RUNTIME DESTINATION bin)

