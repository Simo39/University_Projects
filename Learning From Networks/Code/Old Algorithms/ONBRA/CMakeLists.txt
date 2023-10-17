cmake_minimum_required (VERSION 2.8)
#set(CMAKE_CXX_COMPILER "/usr/bin/g++-9")
project (temporal-betweenness)
#include (cmake/libs.cmake)

#set(CMAKE_CXX_FLAGS " -std=c++17 -g -fsanitize=address -fsanitize=undefined -fsanitize=signed-integer-overflow -pedantic -Wall -Wextra ${CMAKE_CXX_FLAGS}")
message("Compiler Version: ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS " -std=c++17 -lpthread -pthread -pedantic -Wall -Wextra -O3 ${CMAKE_CXX_FLAGS}")

find_package(Boost)
if (Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIR}/boost)
#else ()
#  install_boost(gcc program_options)
endif ()
#############
# TGLib for Temporal Graph Management
# Set the TGLib library Path
cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH MY_PARENT_DIR)
SET(TGLib_INCLUDE_DIR "${MY_PARENT_DIR}/TGLib")
#SET(TGLib_INCLUDE_DIR "your_path_to/TGLib")
IF (NOT TGLib_INCLUDE_DIR)
MESSAGE(FATAL_ERROR "Please point the installation PATH of TGLib. \nCurrent TGLib Directory is = ${TGLib_INCLUDE_DIR}")
ENDIF ()
INCLUDE_DIRECTORIES("${TGLib_INCLUDE_DIR}")
##############
#add_executable(btwBenchmark 
add_executable(onbra 
	btwBenchmark.cpp
	algorithms.cpp
	adjList.cpp
	graph.cpp
)

target_link_libraries(onbra boost_program_options)
