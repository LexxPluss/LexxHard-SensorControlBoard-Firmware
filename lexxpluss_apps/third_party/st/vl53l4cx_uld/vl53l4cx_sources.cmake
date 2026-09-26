# Copyright (c) 2024-2026, LexxPluss Inc.
# All rights reserved.
#
# Explicit source lists for the vendored VL53L4CX ULD. Never use file globbing here:
# the point of the snapshot is that what is compiled is decided in review, not by
# whatever happens to be on disk.
#
# Two groups, deliberately:
#
#   VL53L4CX_ULD_PRODUCTION_SOURCES
#       everything the ranging path needs - init, configure, start, read, stop.
#       This is what the flash budget measures.
#
#   VL53L4CX_ULD_OPTIONAL_SOURCES
#       calibration and debug translation units. Present in the snapshot so the
#       provenance is complete and enabling them later is one line, but not
#       compiled today, so the budget figures mean what they say.
#
# Nothing is deleted from upstream/ to shrink the build. Linker garbage collection
# already drops unreachable code; the split exists to keep the budget honest and the
# review surface small, not to save flash.

set(VL53L4CX_ULD_DIR ${CMAKE_CURRENT_LIST_DIR})

# One upstream file is compiled from a PATCHED COPY in the build directory, never from
# the snapshot. VL53LX_GetMultiRangingData() reports a failed result fetch as a
# successful measurement, and also advances the device's stream-count history while
# doing it, which is the one thing a replay guard cannot survive. The patch header
# carries the full reasoning.
#
# Patching a copy rather than the file keeps SHA256SUMS meaningful: it still answers
# "has anyone edited upstream?" with one command, because nobody has. It also makes
# upstream drift loud -- a snapshot refresh that moves this function fails the patch,
# and therefore fails configuration, instead of silently dropping the fix.
set(VL53L4CX_PATCHED_DIR ${CMAKE_CURRENT_BINARY_DIR}/vl53l4cx_uld_patched/modules)

file(MAKE_DIRECTORY ${VL53L4CX_PATCHED_DIR})
configure_file(${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_api.c
               ${VL53L4CX_PATCHED_DIR}/vl53lx_api.c COPYONLY)

find_program(VL53L4CX_PATCH_EXECUTABLE patch REQUIRED)
execute_process(
  COMMAND ${VL53L4CX_PATCH_EXECUTABLE} --silent -p1
          -i ${VL53L4CX_ULD_DIR}/zephyr/patches/0001-propagate-get-device-results-status.patch
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/vl53l4cx_uld_patched
  RESULT_VARIABLE VL53L4CX_PATCH_RESULT
)
if(NOT VL53L4CX_PATCH_RESULT EQUAL 0)
  message(FATAL_ERROR
    "The VL53L4CX get_device_results status patch no longer applies -- the upstream "
    "snapshot moved. Re-base the patch; do not drop it.")
endif()

set(VL53L4CX_ULD_PRODUCTION_SOURCES
  ${VL53L4CX_ULD_DIR}/upstream/vl53l4cx.c
  ${VL53L4CX_PATCHED_DIR}/vl53lx_api.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_api_calibration.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_api_core.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_api_preset_modes.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_core.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_core_support.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_dmax.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_hist_algos_gen3.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_hist_algos_gen4.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_hist_char.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_hist_core.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_hist_funcs.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_nvm.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_register_funcs.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_sigma_estimate.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_silicon_core.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_wait.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_xtalk.c
  ${VL53L4CX_ULD_DIR}/upstream/porting/vl53lx_platform_ipp.c
  ${VL53L4CX_ULD_DIR}/zephyr/vl53lx_platform.c
  ${VL53L4CX_ULD_DIR}/zephyr/vl53l4cx_bus_io.c
)

# Not compiled today. Enabling one is a deliberate edit plus a fresh budget run.
#
# vl53lx_api_calibration.c is NOT here, and that is a link-report finding rather than
# a judgement: VL53LX_PerformRefSpadManagement lives in vl53lx_api.c, is reachable
# from the BSP init path, and calls VL53LX_run_ref_spad_char in the calibration unit.
# Excluding it by its name would have failed to link. This is why the split is
# decided by the linker and not by the file list.
set(VL53L4CX_ULD_OPTIONAL_SOURCES
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_api_debug.c
  ${VL53L4CX_ULD_DIR}/upstream/modules/vl53lx_nvm_debug.c
)

# upstream/porting/vl53lx_platform.c is kept in the snapshot as provenance and is
# deliberately NOT in either list. It is the upstream example layer: it carries a
# single file-scope _I2CBuffer[256] shared by every device and its own tracing, so
# it is neither reentrant across the four sensors on our one bus nor compatible with
# our locking and test boundaries. zephyr/vl53lx_platform.c replaces it.

set(VL53L4CX_ULD_INCLUDE_DIRS
  ${VL53L4CX_ULD_DIR}/zephyr
  ${VL53L4CX_ULD_DIR}/upstream
  ${VL53L4CX_ULD_DIR}/upstream/modules
  ${VL53L4CX_ULD_DIR}/upstream/porting
)
