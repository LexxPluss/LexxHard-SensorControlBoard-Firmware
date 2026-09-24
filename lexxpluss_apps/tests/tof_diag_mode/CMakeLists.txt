# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Which of the two things a hang-isolation mode does: read the real L7, and put grid frames on CAN.
# Built once per mode, against the production header, because the point of the third mode is that
# it changed neither of the two that were already measured on hardware.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_diag_mode_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

if(NOT TOF_DIAG_TEST_MODE)
    set(TOF_DIAG_TEST_MODE 3)
endif()

target_sources(app PRIVATE src/test_main.cpp)
target_include_directories(app PRIVATE ${APP_SRC_DIR})

# The TX seam is read as text. It lives in a translation unit that needs the Zephyr CAN device API
# and a real can2 node, so linking it here would test the shim it was given rather than the branch
# the board takes. What must not silently change is which macro selects the branch.
target_compile_definitions(app PRIVATE
    TOF_DIAG_HANG=${TOF_DIAG_TEST_MODE}
    TOF_CLIFF_CAN_SOURCE="${APP_SRC_DIR}/tof_cliff_can.cpp")
