# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause

# Ordering tests for the Package single-device lifecycle. Nothing here touches a bus: the hooks are
# injected, which is what makes the rules -- and the one rule about an unclassifiable probe -- the
# thing under test rather than the ULD's register sequence.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_package_l7_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# THE STEP RECORD IS LINKED IN, not faked. The rule under test -- which `up` may clear it -- is an
# interaction between the lifecycle's entry guard and that module's storage, and a stand-in for
# either side would be testing the stand-in.
target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_package_l7.cpp
    ${APP_SRC_DIR}/tof_l7_step_record.cpp
    ${APP_SRC_DIR}/tof_l7_quiesce.cpp
)
# ENABLE_TOF_L7_ULD alone: this lifecycle names no bus and no chain, which is the premise the
# Package configuration rests on.
target_compile_definitions(app PRIVATE ENABLE_TOF_L7_ULD)
target_include_directories(app PRIVATE ${APP_SRC_DIR})
