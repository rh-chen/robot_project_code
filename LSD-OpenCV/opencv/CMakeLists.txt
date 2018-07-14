FIND_PACKAGE(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE(lsd_opencv_example lsd_opencv_example.cpp)
TARGET_LINK_LIBRARIES(lsd_opencv_example ${LSD_SOURCE_DIR}/build/lib/liblsd.so ${OpenCV_LIBS})
