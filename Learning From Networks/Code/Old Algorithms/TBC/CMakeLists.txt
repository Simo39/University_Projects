cmake_minimum_required (VERSION 3.0)
# Name of the Project
project(tbc)

# Manual Format

# Sobstitute "$ENV{EIGEN3_INCLUDE_DIR}" to a PATH FOR EIGEN3 LIBRARY
# FROM THE SYSTEM PACKAGE "/usr/include/eigen3" (Linux System) OR FROM YOUR SOURCE DOWNLOAD "/home/bprin/Scrivania/Eigen3/eigen-3.4.0"
#SET(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
#IF (NOT EIGEN3_INCLUDE_DIR)
    #MESSAGE(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
#ENDIF ()
#INCLUDE_DIRECTORIES("${EIGEN3_INCLUDE_DIR}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -DNDEBUG")

# Automatic Format

# Try with this Automatic format to load the Eigen3 Library
# For Linux Run , Otherwise comment this and uncomment the Manual format from line 15 to 20 

find_package (Eigen3)
if (Eigen3_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -DNDEBUG")
endif ()
###############
# Boost
find_package(Boost)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR}/boost)
endif ()
##############
# OpenMP 
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
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
# Set the Customized Libraries and Directories
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
add_library(Tools INTERFACE include/tbc.h  include/results.h include/static_graph.h)
target_include_directories(Tools INTERFACE)

add_executable(${PROJECT_NAME} src/main.cpp)
# Comment Eigen3::Eigen if you use the manual linking to Eigen3
target_link_libraries( ${PROJECT_NAME} PRIVATE Tools boost_program_options ${OpenMP_CXX_LIBRARIES} Eigen3::Eigen)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)