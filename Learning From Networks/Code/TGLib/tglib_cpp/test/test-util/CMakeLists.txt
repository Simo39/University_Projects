# Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
#
# This file is part of TGLib which is released under MIT license.
# See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
# for full license details.

cmake_minimum_required(VERSION 3.4...3.18)
project(tglib)

set(TEST_SOURCES
        test-topkresult.cpp
        test-inputOutput.cpp
        )

target_sources(custom-main-tests PRIVATE ${TEST_SOURCES})