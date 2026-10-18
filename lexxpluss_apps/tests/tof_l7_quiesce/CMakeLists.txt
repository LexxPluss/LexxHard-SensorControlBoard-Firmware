# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause
#
# The quiesce classification and its record, compiled pure: no Zephyr device, no bus, no vendor
# object. The rule is four cases and an ordering, so it is cheaper to test exhaustively than to
# reason about -- which is the point of extracting it out of the adapter in the first place.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_l7_quiesce_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# THE CALL-SITE GATE IS A TEST, in C++, run with the rest of the suite. It was Python invoked from
# this file, which the repository's language boundary forbids three ways over -- product code and
# executable tests are C and C++, and scripts/check_language_boundary.sh enforces it. Running it as
# a test rather than at configure time is also where it belongs: a configure-time gate is skipped by
# anyone who only runs the tests. It reads the sources, so the paths come in as definitions.
# The paths go through a GENERATED FILE rather than a macro. A semicolon-separated list embedded in
# a -D argument broke the compiler command line, and quoting it further would have made the build
# depend on how a shell splits it -- one path per line is read the same way everywhere.
file(GLOB APP_CPP ${APP_SRC_DIR}/*.cpp)
string(REPLACE ";" "\n" APP_CPP_LINES "${APP_CPP}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/app_sources.txt "${APP_CPP_LINES}\n")

target_sources(app PRIVATE
    src/test_main.cpp
    src/test_call_sites.cpp
    ${APP_SRC_DIR}/tof_l7_quiesce.cpp
    ${APP_SRC_DIR}/tof_l7_step_record.cpp)
target_compile_definitions(app PRIVATE
    DEVICE_SOURCE=\"${APP_SRC_DIR}/tof_package_device.cpp\"
    SHELL_SOURCE=\"${APP_SRC_DIR}/tof_package_bench.cpp\"
    SOURCE_LIST_FILE=\"${CMAKE_CURRENT_BINARY_DIR}/app_sources.txt\")
target_compile_definitions(app PRIVATE ENABLE_TOF_L7_ULD)
target_include_directories(app PRIVATE ${APP_SRC_DIR})

