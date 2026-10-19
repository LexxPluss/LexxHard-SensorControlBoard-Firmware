# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause

# The boot order, built as an image with NO grid driver would build it.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_l7_boot_order_no_grid_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# THE SAME CASES, IN AN IMAGE WITH NO GRID DRIVER. tof_l7_boot_order is deliberately unguarded --
# every image reaches its control lines through it -- so the one source and the one test file build
# both ways and only the flag differs.
#
# This is the half the other binary cannot reach: steps_for_image() drops the recovery steps when
# ENABLE_TOF_L7_ULD is absent, and a suite that only ever builds with the flag on compiles that
# branch in neither direction.
target_sources(app PRIVATE
    ../tof_l7_recovery/src/test_order.cpp
    ${APP_SRC_DIR}/tof_l7_boot_order.cpp
)
target_compile_definitions(app PRIVATE ENABLE_TOF_CHAIN)
target_include_directories(app PRIVATE ${APP_SRC_DIR})
