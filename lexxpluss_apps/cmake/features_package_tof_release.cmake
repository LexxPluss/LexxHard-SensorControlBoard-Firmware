# Feature selection for the Package ToF image: the ToF chain plus the Package sensor.
#
# THIS IS WHAT THE BRANCH DEFAULT BUILDS. features_dasher.cmake is a single include of this file, so
# `make firmware` with no FEATURES_FILE produces this configuration and a release tag publishes it.
# Naming it explicitly still works and builds the same image:
#   make firmware_initial VERSION=<version> FEATURES_FILE=cmake/features_package_tof_release.cmake
#
# It includes features_tof_release.cmake so the chain's flags, overlays and Kconfig fragments are
# inherited rather than restated, and the Dasher machine flags arrive with them -- including
# BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST, which is what confines this image to a machine with no
# safety lidar fitted. Taking features_dasher.cmake here instead would be a cycle: that file is the
# default and includes this one.
include(${CMAKE_CURRENT_LIST_DIR}/features_tof_release.cmake)

# THIS CONFIGURATION CARRIES ITS OWN FLASH FIT, so it does not wait on the AMRSW-2887 tree changes.
# The chain and this runtime together overflow the 256 KiB slot at LINK time on the production base;
# overlays/tof_package_flashfit.conf is what makes it link, and that file states what it sets, what it
# measured and when it should be deleted. Appended after the inherited fragments so these values win.
#
# IT REACHES EVERY DEFAULT BUILD ON THIS BRANCH, not just this image, now that features_dasher.cmake
# includes this file -- the two-state key switch and interlock variants are built from the same
# selection and inherit the same trim and the same margin. That is the same reach the AMRSW-2887 tree
# changes have, which is why they replace this fragment rather than stacking with it.
list(APPEND EXTRA_CONF_FILE
    ${CMAKE_CURRENT_LIST_DIR}/../overlays/tof_package_flashfit.conf)

set(ENABLE_TOF_PACKAGE_AUTO ON)

# The rate the whole path was measured at, and the only one it has run at. CMakeLists.txt refuses to
# configure without a value here on purpose, and its reason still holds: no rate has been accepted as
# sustainable for this geometry.
set(TOF_PACKAGE_AUTO_HZ 5)

# RELEASE POLICY: A RESIDENT IMAGE CARRIES NO UNATTENDED TIMED REVERT. What is switched off is the
# bench deadman -- a ten-minute cold reboot of the whole machine, there to put an unsupervised
# candidate back, which is the opposite of what a release is for. It is a decision rather than an
# inference about an asset, and it is now the only thing deciding. main.cpp used to self-confirm
# before arm() could run whenever ENABLE_TOF_CHAIN was set, so the revert was absent by an ordering
# coincidence between two features that knew nothing about each other. That guard is explicit now --
# an image compiled with a deadman does not self-confirm -- so OFF here is what keeps a resident
# image confirmed, and ON genuinely arms the revert. features_package_tof_bench.cmake is the
# candidate configuration that turns it on.
#
# THIS IS NOT THE WATCHDOG, AND READING IT AS "NO RESET PROTECTION" WOULD BE WRONG. tof_watchdog_feeder
# is installed in every image, by board_controller and not by any ToF flag, and it judges the L7 slots
# and declared long operations, so a wedged ULD operation can still stop the feed and reset the board.
# It now tells the two L7 populations apart as well -- see below.
set(TOF_PACKAGE_AUTO_DEADMAN OFF)

# The only fragment of its own is the flash fit above: CONFIG_IMG_MANAGER and CONFIG_STREAM_FLASH
# arrive with overlays/tof_release.conf, and the stored blob is read through flash_area_*, which
# prj.conf configures with CONFIG_FLASH_MAP. The CAN identifiers stay in src/tof_can_ids.hpp; the
# TOF_PACKAGE_AUTO_DATA_ID / _HEALTH_ID overrides are bench-only and a release image carries neither.
#
# TWO L7 POPULATIONS IN ONE IMAGE WAS THE 2026-11-04 FAILURE, and f3fc71e8 is what answers it. This
# configuration was the first to put the chain and the Package device in one image, and it reset
# itself every 11.6 s: runtime_progress carried a SINGLE activity::l7 written by tof_acquisition and
# by tof_package_device both, so progress by a healthy population refreshed the judgement for a
# wedged one and the tombstone named "the L7" when only one of two had failed. There are now
# l7_chain and l7_package, judged from separate clocks, armed and reported separately.
#
# The exemption was the other half, and separate slots alone would not have reached it: one mask
# suspended the chain's acquisition judgement whoever had declared, so healthy Package work on
# another bus exempted a wedged chain acquisition for its whole duration. kChainLongOperationSuspends
# and kPackageLongOperationSuspends now scope each declaration to what its owner actually holds, and
# the bookkeeping is a depth count per population in tof_long_operation.hpp rather than a flat flag,
# so one site's end() can no longer strip the cover from another's operation.
#
# WHAT IS STILL ONLY ARGUED. hook_open declares a long operation now, as quiesce already did -- it
# moves the 86,016 B ULD blob over i2c4 at the 100 kHz Zephyr default, 7.74 s of bus time against a
# 5,000 ms bound -- and tof_l7_quiesce's call-site gate asserts that both declare and that no third
# hook does so without its own measurement. The host suite tests/tof_long_operation covers the
# bookkeeping. What no run has yet shown is the two populations declaring at once on hardware:
# main() still runs tof_chain_controller::init() to completion before it creates the Package thread,
# so the ordering that keeps them apart is structural rather than observed.
