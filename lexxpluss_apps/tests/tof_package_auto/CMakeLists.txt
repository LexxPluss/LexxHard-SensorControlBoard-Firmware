# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause

# What the unattended runtime does next, with no device anywhere near it. The policy names the
# lifecycle's states and steps, so tof_package_l7.cpp is linked in for that vocabulary rather than
# re-declared -- a second copy of those enums is exactly the drift this would be testing against.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_package_auto_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_package_auto.cpp
    ${APP_SRC_DIR}/tof_package_l7.cpp
    ${APP_SRC_DIR}/tof_l7_step_record.cpp
    ${APP_SRC_DIR}/tof_l7_quiesce.cpp
)
target_compile_definitions(app PRIVATE ENABLE_TOF_L7_ULD)
target_include_directories(app PRIVATE ${APP_SRC_DIR})
