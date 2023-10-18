cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(lab_5)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_library(PANORAMIC_UTILS 
	include/panoramic_utils.h 
	include/PanoramicImage.h src/PanoramicImage.cpp)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} PANORAMIC_UTILS)
