# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zcan_bounded_send)

target_sources(app PRIVATE
    src/test_main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/zcan_bounded_send.cpp)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
