# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Adapter tests use the real production API with fake ULD and runtime symbols. In particular there
# is no test-only `open_with_firmware()` path: the object assignment still calls
# tof_l7_runtime::firmware_data(), and the link substitute only controls that answer.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_l7_sensor_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
set(L7_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l7cx_uld)

target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_l7_sensor.cpp
)
target_compile_definitions(app PRIVATE ENABLE_TOF_CHAIN ENABLE_TOF_L7_ULD)
target_include_directories(app PRIVATE
    ${APP_SRC_DIR}
    ${L7_DIR}/zephyr
    ${L7_DIR}/upstream/modules
)
