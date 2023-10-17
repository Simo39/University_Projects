# Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
#
# This file is part of TGLib which is released under MIT license.
# See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
# for full license details.

cmake_minimum_required(VERSION 3.4...3.18)
project(tglib)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# OpenMP for parallelization
find_package(OpenMP REQUIRED)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_subdirectory(extern/pybind11)

add_subdirectory(src)

add_subdirectory(test)


# Doxygen for documentation
find_package(Doxygen)
if (DOXYGEN_FOUND)
    set(DOXYGEN_GENERATE_TREEVIEW YES)
    set(DOXYGEN_PROJECT_NAME TGLib)
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE ../README.md)
    set(DOXYGEN_TOC_INCLUDE_HEADINGS 2)

    doxygen_add_docs(doxygen ${TO_DOCUMENT} src/namespaces.h ../README.md)
else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the documentation")
endif (DOXYGEN_FOUND)


# automatically generate amalgamated header
function(cat IN_FILE OUT_FILE)
    file(READ ${IN_FILE} CONTENTS)
    file(APPEND ${OUT_FILE} "${CONTENTS}")
endfunction()

# Prepare a temporary file to "cat" to:
file(WRITE "${PROJECT_SOURCE_DIR}/amalgamated/tglib_amalgamated.h"
        "// Amalgamated header of TGLib\n\n")

# Call the "cat" function for each input file
foreach(HEADER ${TO_DOCUMENT})
    cat(${HEADER} "${PROJECT_SOURCE_DIR}/amalgamated/tglib_amalgamated.h")
endforeach()

# remove includes
execute_process(
    COMMAND sh -c "/bin/sed -i -e '/#include \"/d' ${PROJECT_SOURCE_DIR}/amalgamated/tglib_amalgamated.h"
)
