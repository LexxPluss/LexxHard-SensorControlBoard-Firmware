# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause
#
# The second suite that compiles a vendor translation unit, and the same reasoning applies as to
# tests/tof_uld_status: it does not emulate the device, it pins a property of OUR PATCH.
#
# tests/tof_l7_sensor replaces the whole ULD with fakes, which is right for the adapter and is
# exactly why it cannot see this: vl53l7cx_stop_ranging()'s own timeout handling is in the vendor
# source, and a suite that substitutes that function tests its own substitute. So this one compiles
# the PATCHED copy and drives the real function over a register map the test controls.
#
# The copy-and-patch below is the same mechanism vl53l7cx_sources.cmake uses, repeated rather than
# shared because a test must be able to say which source it compiled without depending on the
# application's build having run first.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_l7_uld_stop_test)

set(L7_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l7cx_uld)
set(PATCHED_DIR ${CMAKE_CURRENT_BINARY_DIR}/vl53l7cx_uld_patched/modules)

file(MAKE_DIRECTORY ${PATCHED_DIR})
configure_file(${L7_DIR}/upstream/modules/vl53l7cx_api.c ${PATCHED_DIR}/vl53l7cx_api.c COPYONLY)
configure_file(${L7_DIR}/upstream/modules/vl53l7cx_buffers.h ${PATCHED_DIR}/vl53l7cx_buffers.h COPYONLY)

find_program(PATCH_EXECUTABLE patch REQUIRED)
foreach(l7_patch
    0001-use-verified-external-firmware.patch
    0002-stop-ranging-timeout-is-a-failure.patch)
  execute_process(
    COMMAND ${PATCH_EXECUTABLE} --silent -p1 -i ${L7_DIR}/zephyr/patches/${l7_patch}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/vl53l7cx_uld_patched
    RESULT_VARIABLE PATCH_RESULT
  )
  if(NOT PATCH_RESULT EQUAL 0)
    message(FATAL_ERROR
      "The VL53L7CX patch ${l7_patch} no longer applies -- the upstream snapshot moved. "
      "Re-base the patch; do not drop it.")
  endif()
endforeach()

target_sources(app PRIVATE
    src/test_main.c
    ${PATCHED_DIR}/vl53l7cx_api.c
)
target_include_directories(app PRIVATE
    ${L7_DIR}/zephyr
    ${PATCHED_DIR}
    ${L7_DIR}/upstream/modules
    ${L7_DIR}/upstream/porting
    ${L7_DIR}/upstream
)
# Vendor code, compiled as vendor code: its warnings are not ours to fix in place. The external
# firmware definition is the same one the production build uses, so the 86,016-byte array is out of
# this binary too.
set_source_files_properties(${PATCHED_DIR}/vl53l7cx_api.c PROPERTIES
    COMPILE_DEFINITIONS VL53L7CX_EXTERNAL_FIRMWARE=1
    COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-sign-compare;-Wno-unused-but-set-variable"
)
