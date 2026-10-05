# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_commission_boot_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# THE ORDER AND THE LATCH, AND NOTHING ELSE. The module takes both steps as function pointers, so
# this suite needs neither the cliff runtime nor the CAN binding to exercise what it decides -- the
# same arrangement as tests/tof_l7_recovery, and for the same reason: an ordering rule that can only
# be checked by booting a board is an ordering rule nobody checks.
target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_commission_boot.cpp)
target_compile_definitions(app PRIVATE ENABLE_TOF_CHAIN ENABLE_TOF_CLIFF_ULD)
target_include_directories(app PRIVATE ${APP_SRC_DIR})
