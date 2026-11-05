# Feature selection for the Package ToF release image: the ToF release plus the Package sensor.
# Built through the ordinary release entry, with the asset names unchanged:
#   make firmware_initial VERSION=<version> FEATURES_FILE=cmake/features_package_tof_release.cmake
#
# It includes features_tof_release.cmake rather than features_dasher.cmake so there is one release
# line and one image: the chain's flags, overlays and Kconfig fragments are inherited rather than
# restated, and features_dasher.cmake arrives with them -- including
# BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST, which is what confines this image to a machine with no
# safety lidar fitted.
include(${CMAKE_CURRENT_LIST_DIR}/features_tof_release.cmake)

# NOT BUILDABLE ON A TREE WITHOUT THE FLASH REDUCTIONS. The chain and this runtime together overflow
# the 256 KiB slot at LINK time, so picolibc, the shell trim, the SPI DMA removal and the
# SD/filesystem removal have to be present. That is a build-order dependency rather than a margin to
# keep an eye on; the measurements are in the pull request, not here.
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
# is installed in every image, by board_controller and not by any ToF flag, and it judges the L7 slot
# and declared long operations, so a wedged ULD operation can still stop the feed and reset the board.
# What that coverage cannot yet do is tell the two L7 populations apart -- see below.
set(TOF_PACKAGE_AUTO_DEADMAN OFF)

# No Kconfig fragment and no overlay of its own: CONFIG_IMG_MANAGER and CONFIG_STREAM_FLASH arrive
# with overlays/tof_release.conf, and the stored blob is read through flash_area_*, which prj.conf
# configures with CONFIG_FLASH_MAP. The CAN identifiers stay in src/tof_can_ids.hpp; the
# TOF_PACKAGE_AUTO_DATA_ID / _HEALTH_ID overrides are bench-only and a release image carries neither.
#
# WHAT TWO L7 POPULATIONS IN ONE IMAGE DO NOT YET SETTLE, since the absence of a conflicting CMake
# flag does not cover it: runtime_progress::activity::l7 is a SINGLE slot, written by the chain in
# tof_acquisition.cpp and by this runtime in tof_package_device.cpp (watched_uld_operation, around
# every ULD call), behind ONE l7_expected boolean that tof_chain_controller.cpp and
# tof_package_runtime.cpp both set, and tof_task_watchdog.cpp judges L7 liveness from that slot
# alone. So progress by a healthy population refreshes the judgement for a wedged one -- the
# in-flight clock included, because note_progress re-stamps it every time `begun` moves rather than
# holding it from the first begin, so each new operation on the healthy side pushes the wedged side's
# deadline out again. That is the gate to close before this image is a release candidate.
#
# The declared-long-operation flag is a SEPARATE mechanism and a weaker claim, stated separately so
# the two are not confused. It is flat, with no depth count, and both sides declare one -- the chain
# around its blob bootstrap, this runtime around quiesce ALONE (the 5.43 s stop poll; probe, open and
# start keep the ordinary bound). Nothing here demonstrates an overlap: main() runs
# tof_chain_controller::init() to completion before it creates the Package thread, so the chain's
# declaration is over before this runtime can make one. The hazard is structural, and
# tof_package_device.cpp already names this configuration as the one that has to revisit it.
