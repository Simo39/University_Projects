# Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
#
# This file is part of TGLib which is released under MIT license.
# See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
# for full license details.
cmake_minimum_required(VERSION 3.4...3.18)
project(tglib)

# Catch2 for unit tests
Include(FetchContent)

FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.0.1 # or a later release
)

FetchContent_MakeAvailable(Catch2)

set(TEST_SOURCES
        test-algorithms/test-burstiness.cpp
        test-main.cpp TemporalGraphsInstances.cpp)

add_executable(custom-main-tests ${TEST_SOURCES})

add_subdirectory(test-algorithms)
add_subdirectory(test-core)
add_subdirectory(test-util)

target_link_libraries(custom-main-tests PRIVATE Catch2::Catch2WithMain)

include(CTest)
include(Catch)
catch_discover_tests(custom-main-tests)
