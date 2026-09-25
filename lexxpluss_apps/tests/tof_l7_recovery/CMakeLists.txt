# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# The boot-time L7 recovery pass. Pure logic over an injected pair of ULD calls, so it links the
# production source directly and needs no bus, no Zephyr device and no ULD.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_l7_recovery_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_l7_recovery.cpp)
target_include_directories(app PRIVATE ${APP_SRC_DIR})
