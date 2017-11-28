cmake_minimum_required (VERSION 2.8.2)
project (read_map_number)

add_definitions(-std=c++11)

include_directories (include)

aux_source_directory(./src/ SRC_LIST)

add_executable(${PROJECT_NAME} ${SRC_LIST})



