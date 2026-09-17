# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Wire-level tests for the L7 Zephyr port. The vendor ULD is deliberately absent: these tests pin
# our 16-bit index, repeated-start, segmentation and errno contract without freezing ST's internal
# register sequence into the repository.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_l7_port_test)

set(L7_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l7cx_uld)
set(I2C_FAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../tof_cliff_sensor/src)

target_sources(app PRIVATE
    src/test_port.c
    ${I2C_FAKE_DIR}/fake_i2c.c
    ${L7_DIR}/zephyr/platform.c
)
target_include_directories(app PRIVATE
    ${I2C_FAKE_DIR}
    ${L7_DIR}/zephyr
    ${L7_DIR}/upstream/modules
)
