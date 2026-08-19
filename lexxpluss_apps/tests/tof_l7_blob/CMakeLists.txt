# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# The stored blob record, with the real SHA-256 and the real CRC. The only thing faked is where the
# bytes come from: on a board it is memory-mapped flash, here it is a RAM buffer a case can damage.
# The record itself is the GENERATOR's output, committed as src/golden_record.h -- a layout written
# once at manufacture and read at every boot needs its two implementations pinned to each other.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_l7_blob_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_l7_blob_record.cpp
    ${APP_SRC_DIR}/tof_l7_runtime.cpp
)
target_compile_definitions(app PRIVATE ENABLE_TOF_CHAIN ENABLE_TOF_L7_ULD)
target_include_directories(app PRIVATE
    ${APP_SRC_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l7cx_uld/zephyr
)
