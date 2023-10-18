cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(BoatDetection)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_library(BoatDetection 
	include/BoatDetection.h src/BoatDetection.cpp)

add_library(ClassificationBoat 
	include/ClassificationBoat.h src/ClassificationBoat.cpp
	include/Dictionary.h	src/Dictionary.cpp)

add_library(Localization 
	include/Localization.h src/Localization.cpp
	include/Filter.h src/Filter.cpp)

add_library(utils 
	include/debug_utils.h src/debug_utils.cpp
	include/FilteringTest.h src/FilteringTest.cpp)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} BoatDetection)
