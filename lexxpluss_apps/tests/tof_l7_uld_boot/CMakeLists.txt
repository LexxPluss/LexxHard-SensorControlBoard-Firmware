# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause
#
# The MCU boot wait, driven directly.
#
# tests/tof_l7_uld_stop drives vl53l7cx_stop_ranging(), which is a public API. The boot wait is
# `static`, reachable only through vl53l7cx_init() -- and getting init as far as the boot wait means
# satisfying two firmware-access polls and an 86 KB download first, which would make the fake larger
# than the thing under test and the failure harder to attribute. So this suite INCLUDES the patched
# vendor translation unit from its test source, which makes the statics visible, and calls the boot
# wait with the register map it controls. The .c is therefore NOT also compiled as a source here:
# one translation unit, no duplicate symbols.
#
# The copy-and-patch is the same mechanism vl53l7cx_sources.cmake uses, repeated rather than shared
# for the same reason the sibling suite repeats it: a test must be able to say which source it
# compiled without depending on the application's build having run first.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_l7_uld_boot_test)

set(L7_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l7cx_uld)
set(PATCHED_DIR ${CMAKE_CURRENT_BINARY_DIR}/vl53l7cx_uld_patched/modules)

file(MAKE_DIRECTORY ${PATCHED_DIR})
configure_file(${L7_DIR}/upstream/modules/vl53l7cx_api.c ${PATCHED_DIR}/vl53l7cx_api.c COPYONLY)
configure_file(${L7_DIR}/upstream/modules/vl53l7cx_buffers.h ${PATCHED_DIR}/vl53l7cx_buffers.h COPYONLY)

find_program(PATCH_EXECUTABLE patch REQUIRED)
foreach(l7_patch
    0001-use-verified-external-firmware.patch
    0004-mcu-boot-timeout-is-a-failure.patch)
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

target_sources(app PRIVATE src/test_main.c)
target_include_directories(app PRIVATE
    ${L7_DIR}/zephyr
    ${PATCHED_DIR}
    ${L7_DIR}/upstream/modules
    ${L7_DIR}/upstream/porting
    ${L7_DIR}/upstream)
# The vendor code arrives through an #include, so its warnings land on this file.
set_source_files_properties(src/test_main.c PROPERTIES
    COMPILE_DEFINITIONS VL53L7CX_EXTERNAL_FIRMWARE=1
    COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-sign-compare;-Wno-unused-but-set-variable;-Wno-unused-function"
)
