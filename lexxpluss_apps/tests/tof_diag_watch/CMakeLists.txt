# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# The watchdog decision of the L7 hang-isolation images, on the host.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_diag_watch_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_diag_watch.cpp
)
target_compile_definitions(app PRIVATE TOF_DIAG_HANG=2)
target_include_directories(app PRIVATE ${APP_SRC_DIR})
