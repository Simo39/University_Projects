cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(BoWgenerator)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_library(BoWCreator 
	include/BoWCreator.h src/BoWCreator.cpp)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} BoWCreator)
