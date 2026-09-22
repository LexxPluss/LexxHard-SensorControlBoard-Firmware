# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# The L7 blob provisioner against a fake flash with NOR semantics, with power loss injected at every
# erase and write. The record is the generator's real output for the vendored blob, produced by the
# Makefile before this build and passed in L7_BLOB_RECORD_IMG -- the same bytes the DEV image embeds.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_l7_blob_provisioner_test)

if(NOT L7_BLOB_RECORD_IMG OR NOT EXISTS "${L7_BLOB_RECORD_IMG}")
    message(FATAL_ERROR "L7_BLOB_RECORD_IMG must name the generated uncommitted record "
                        "(see the test_tof_l7_blob_provisioner Makefile target)")
endif()

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
set(L7_BLOB_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/l7_blob_record)
file(MAKE_DIRECTORY ${L7_BLOB_GEN_DIR})
generate_inc_file_for_target(app ${L7_BLOB_RECORD_IMG} ${L7_BLOB_GEN_DIR}/l7_blob_record.inc)

target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_l7_blob_provisioner.cpp
    ${APP_SRC_DIR}/tof_l7_blob_record.cpp
)
# Only the provisioner's own definition: the record reader must compile for this image without the
# ToF chain, exactly as it does in the DEV image.
target_compile_definitions(app PRIVATE ENABLE_L7_BLOB_PROVISIONER)
target_include_directories(app PRIVATE
    ${APP_SRC_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l7cx_uld/zephyr
    ${L7_BLOB_GEN_DIR}
)
