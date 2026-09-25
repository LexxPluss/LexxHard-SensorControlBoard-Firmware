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

set(L7_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l7cx_uld)

# The adapter is built in alongside the pure pass, against fake ULD and port symbols, so the
# translation and the logic it serves are proven in one binary rather than in two that agree by
# assumption.
target_sources(app PRIVATE
    src/test_main.cpp
    src/test_ops.cpp
    ${APP_SRC_DIR}/tof_l7_recovery.cpp
    ${APP_SRC_DIR}/tof_l7_recovery_ops.cpp)
target_compile_definitions(app PRIVATE ENABLE_TOF_CHAIN ENABLE_TOF_L7_ULD)
target_include_directories(app PRIVATE
    ${APP_SRC_DIR}
    ${L7_DIR}/zephyr
    ${L7_DIR}/upstream/modules)
