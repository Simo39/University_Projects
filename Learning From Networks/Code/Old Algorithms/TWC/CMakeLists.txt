cmake_minimum_required (VERSION 3.0)
# Name of the Project
project(twc)

include(CTest)
enable_testing()
###############
# Boost
find_package(Boost)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR}/boost)
endif ()
##############
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
#cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH MY_PARENT_DIR)
SET(TGLib_INCLUDE_DIR "/home/bprin/Learning_From_Network/LFN_04_01_2023/Code/Old Algorithms/TGLib")
#/home/bprin/Scrivania/Learning from Network Project/Learning_from_Network_Project/Code/Old Algorithms
#SET(TGLib_INCLUDE_DIR "${MY_PARENT_DIR}/TGLib")
IF (NOT TGLib_INCLUDE_DIR)
MESSAGE(FATAL_ERROR "Please point the installation PATH of TGLib. \nCurrent TGLib Directory is = ${TGLib_INCLUDE_DIR}")
ENDIF ()
INCLUDE_DIRECTORIES("${TGLib_INCLUDE_DIR}")
##############
##############
# Set the Customized Libraries and Directories
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
add_library(Tools INTERFACE include/results.h)
target_include_directories(Tools INTERFACE )
##############
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries( ${PROJECT_NAME} PRIVATE Tools boost_program_options ${OpenMP_CXX_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)