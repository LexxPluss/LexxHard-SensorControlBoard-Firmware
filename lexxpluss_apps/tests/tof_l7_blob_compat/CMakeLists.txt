# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause

# CAN THIS BRANCH'S READER ACCEPT THE RECORD THAT IS ACTUALLY ON THE BOARD?
#
# The record on dasher1 was written by an image built from another branch, whose copy of
# tof_l7_blob_record.cpp and of the accept-list header is not byte-identical to this one. The on-flash
# format constants agree -- 64-byte header, 4-byte marker, magic L7B1, marker L7OK, format 2 -- but
# constants agreeing is an argument, not a check. This suite is the check: the exact 131,072 bytes
# that should now be in storage_partition, read by THIS branch's reader against THIS branch's signed
# accept-list.
#
# The image is passed in rather than generated here, for the reason the provisioner's CMake gives: a
# firmware build never runs the packer. L7_REGION_IMG names a 131,072-byte file built from the
# generator's record.uncommitted.img plus its 4-byte marker plus 0xff padding.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_l7_blob_compat_test)

if(NOT L7_REGION_IMG OR NOT EXISTS "${L7_REGION_IMG}")
    message(FATAL_ERROR "tof_l7_blob_compat needs L7_REGION_IMG naming the 131,072-byte region image")
endif()

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/l7_region)
file(MAKE_DIRECTORY ${GEN_DIR})
generate_inc_file_for_target(app ${L7_REGION_IMG} ${GEN_DIR}/l7_region.inc)

target_sources(app PRIVATE src/test_main.cpp ${APP_SRC_DIR}/tof_l7_blob_record.cpp)
target_compile_definitions(app PRIVATE ENABLE_TOF_L7_ULD)
target_include_directories(app PRIVATE
    ${APP_SRC_DIR}
    ${GEN_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l7cx_uld/zephyr
)
