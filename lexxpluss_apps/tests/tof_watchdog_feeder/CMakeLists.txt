# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# The startup handshake, against a fake watchdog. Links the production feeder and everything it
# reads, so the order under test is the order the board runs.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_watchdog_feeder_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_watchdog_feeder.cpp
    ${APP_SRC_DIR}/tof_task_watchdog.cpp
    ${APP_SRC_DIR}/tof_watchdog_tombstone.cpp
    ${APP_SRC_DIR}/tof_progress.cpp)
target_include_directories(app PRIVATE ${APP_SRC_DIR})
