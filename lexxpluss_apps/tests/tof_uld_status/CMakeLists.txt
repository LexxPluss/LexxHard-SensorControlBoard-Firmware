# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# The one suite that compiles a vendor translation unit.
#
# tests/tof_cliff_sensor deliberately does not: emulating the VL53L4CX register map
# well enough to drive the ULD would freeze a vendor-internal register sequence into
# our tests. That reasoning still holds, and this suite does not violate it -- it does
# not drive the device at all. It compiles the PATCHED copy of vl53lx_api.c and
# replaces the one function the patch reasons about, VL53LX_get_device_results(), with
# a stub. What is pinned is the patch's own property: that a failed fetch is reported
# as a failure and does not move the device's stream-count history.
#
# The copy-and-patch below is the same mechanism vl53l4cx_sources.cmake uses, repeated
# rather than shared because a test must be able to say which source it compiled
# without depending on the application's build having run first.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_uld_status_test)

set(ULD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l4cx_uld)
set(PATCHED_DIR ${CMAKE_CURRENT_BINARY_DIR}/vl53l4cx_uld_patched/modules)

file(MAKE_DIRECTORY ${PATCHED_DIR})
configure_file(${ULD_DIR}/upstream/modules/vl53lx_api.c
               ${PATCHED_DIR}/vl53lx_api.c COPYONLY)

find_program(PATCH_EXECUTABLE patch REQUIRED)
execute_process(
  COMMAND ${PATCH_EXECUTABLE} --silent -p1
          -i ${ULD_DIR}/zephyr/patches/0001-propagate-get-device-results-status.patch
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/vl53l4cx_uld_patched
  RESULT_VARIABLE PATCH_RESULT
)
if(NOT PATCH_RESULT EQUAL 0)
  message(FATAL_ERROR
    "The VL53L4CX get_device_results status patch no longer applies -- the upstream "
    "snapshot moved. Re-base the patch; do not drop it.")
endif()

target_sources(app PRIVATE
    src/test_main.c
    ${PATCHED_DIR}/vl53lx_api.c
)
target_include_directories(app PRIVATE
    ${ULD_DIR}/zephyr
    ${ULD_DIR}/upstream
    ${ULD_DIR}/upstream/modules
    ${ULD_DIR}/upstream/porting
)
# Vendor code, compiled as vendor code: its warnings are not ours to fix in place.
set_source_files_properties(${PATCHED_DIR}/vl53lx_api.c PROPERTIES
    COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-unused-but-set-variable"
)
