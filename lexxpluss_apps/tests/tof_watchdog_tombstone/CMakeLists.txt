# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# The watchdog tombstone. Pure formatting and validation over a caller-supplied region, so the host
# suite writes into an ordinary array and the production placement is the only thing that knows
# about DTCM.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_watchdog_tombstone_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_watchdog_tombstone.cpp)
target_include_directories(app PRIVATE ${APP_SRC_DIR})
