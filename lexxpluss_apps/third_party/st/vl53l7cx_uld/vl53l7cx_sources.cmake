# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# Explicit source lists for the vendored VL53L7CX ULD. Never glob this directory: a new upstream
# plugin must not silently enter a safety-related image merely because the snapshot changed.

set(VL53L7CX_ULD_DIR ${CMAKE_CURRENT_LIST_DIR})
set(VL53L7CX_PATCHED_DIR ${CMAKE_CURRENT_BINARY_DIR}/vl53l7cx_uld_patched/modules)

file(MAKE_DIRECTORY ${VL53L7CX_PATCHED_DIR})
configure_file(${VL53L7CX_ULD_DIR}/upstream/modules/vl53l7cx_api.c
               ${VL53L7CX_PATCHED_DIR}/vl53l7cx_api.c COPYONLY)
configure_file(${VL53L7CX_ULD_DIR}/upstream/modules/vl53l7cx_buffers.h
               ${VL53L7CX_PATCHED_DIR}/vl53l7cx_buffers.h COPYONLY)

find_program(VL53L7CX_PATCH_EXECUTABLE patch REQUIRED)
execute_process(
  COMMAND ${VL53L7CX_PATCH_EXECUTABLE} --silent -p1
          -i ${VL53L7CX_ULD_DIR}/zephyr/patches/0001-use-verified-external-firmware.patch
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/vl53l7cx_uld_patched
  RESULT_VARIABLE VL53L7CX_PATCH_RESULT
)
if(NOT VL53L7CX_PATCH_RESULT EQUAL 0)
  message(FATAL_ERROR "The VL53L7CX external-firmware patch no longer applies")
endif()

set(VL53L7CX_ULD_PRODUCTION_SOURCES
  ${VL53L7CX_PATCHED_DIR}/vl53l7cx_api.c
  ${VL53L7CX_ULD_DIR}/zephyr/platform.c
)

set(VL53L7CX_ULD_OPTIONAL_SOURCES
  ${VL53L7CX_ULD_DIR}/upstream/modules/vl53l7cx_plugin_detection_thresholds.c
  ${VL53L7CX_ULD_DIR}/upstream/modules/vl53l7cx_plugin_motion_indicator.c
  ${VL53L7CX_ULD_DIR}/upstream/modules/vl53l7cx_plugin_xtalk.c
  ${VL53L7CX_ULD_DIR}/upstream/vl53l7cx.c
)

set(VL53L7CX_ULD_INCLUDE_DIRS
  ${VL53L7CX_ULD_DIR}/zephyr
  ${VL53L7CX_PATCHED_DIR}
  ${VL53L7CX_ULD_DIR}/upstream/modules
  ${VL53L7CX_ULD_DIR}/upstream/porting
  ${VL53L7CX_ULD_DIR}/upstream
)

set_source_files_properties(${VL53L7CX_PATCHED_DIR}/vl53l7cx_api.c PROPERTIES
  COMPILE_DEFINITIONS VL53L7CX_EXTERNAL_FIRMWARE=1
)
