cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(lab_3)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_library(filter include/filter.h src/filter.cpp)

add_executable(${PROJECT_NAME} src/main.cpp src/filter.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} filter)
