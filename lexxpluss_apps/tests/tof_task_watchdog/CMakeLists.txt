# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Feeding the hardware watchdog on task progress. Pure logic over counters and a clock, so it links
# the production source directly and needs no board.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_task_watchdog_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_task_watchdog.cpp)
target_include_directories(app PRIVATE ${APP_SRC_DIR})
