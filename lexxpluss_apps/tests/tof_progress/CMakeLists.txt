# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# The watchdog's six heartbeats. Needs a real kernel rather than a host stub, because the CAN slot
# discrimination is a comparison against the system work queue's thread.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_progress_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_progress.cpp)
target_include_directories(app PRIVATE ${APP_SRC_DIR})
