#project name
PROJECT(image_marker_detection)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE Release)


message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV_DIR: ${OpenCV_DIR}")

include_directories(${PROJECT_SOURCE_DIR}/include)

#output
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")


SET(IMAGEPROC_SRC
	src/imageProc_release.cpp
	src/otsu.cpp 
	src/contour.cpp
	src/findmarker.cpp
	src/processunitmarker.cpp
	src/decodemarker.cpp
	src/hammingdistance.cpp
	src/bitmatrixrotate.cpp
	src/bitmatrixtoid.cpp
	src/markersubpixel.cpp
)

ADD_EXECUTABLE(markerProc ${IMAGEPROC_SRC})
TARGET_LINK_LIBRARIES(markerProc
					${OpenCV_LIBS}
	                -lpthread
)

