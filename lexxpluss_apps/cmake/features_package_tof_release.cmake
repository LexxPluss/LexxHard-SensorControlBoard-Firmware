# Feature selection for the Package ToF release image: the ToF release plus the Package sensor.
# Built with the ordinary release entry, which is the one that also produces out/firmware.bin, and
# the asset names are unchanged:
#   make firmware_initial VERSION=<version> FEATURES_FILE=cmake/features_package_tof_release.cmake
#
# It includes features_tof_release.cmake rather than features_dasher.cmake so there is one release
# line and one image, with the chain's flags, overlays and Kconfig fragments inherited rather than
# restated. That file includes features_dasher.cmake in turn, so ENABLE_PUSH_MODE and
# BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST come along -- and the bypass is what confines this image to
# a machine with no safety lidar fitted.
include(${CMAKE_CURRENT_LIST_DIR}/features_tof_release.cmake)

# THIS IMAGE DOES NOT FIT ON A TREE WITHOUT THE FLASH REDUCTIONS, so this is a build-order
# dependency and not a margin to keep an eye on.
#
# On a tree carrying picolibc, the shell trim, the SPI DMA removal and the SD/filesystem removal,
# this configuration signs at 234,368 B with 27,344 B spare of the 261,712 B ceiling, and the chain
# alone on that same tree signs at 226,608 B -- so the Package runtime costs 7,760 B here, against
# the 19,856 B it costs a plain Dasher image on the same tree, because the chain already carries the
# L7 ULD that both sensor populations drive.
#
# WITHOUT those four, the same configuration does not reach the signing ceiling to be measured
# against it: it overflows the 256 KiB slot at LINK time, by 3,552 B. (That shortfall moves a few
# bytes with the length of the VERSION string, so reproduce it rather than quote it.) The four are
# unmerged, so none of the figures above is a result on the production base.
set(ENABLE_TOF_PACKAGE_AUTO ON)

# The rate the whole path was measured at -- sensor to CAN to ROS -- and the only rate it has run at.
# CMakeLists.txt refuses to configure without a value here on purpose, and its reason still holds: no
# rate has been accepted as sustainable for this geometry. 5 is what the evidence covers, not what a
# review has blessed.
set(TOF_PACKAGE_AUTO_HZ 5)

# RELEASE POLICY: A RESIDENT IMAGE CARRIES NO UNATTENDED RESET. The deadman exists to put a bench
# candidate back when nobody is watching it, which is the opposite of what a release is for, so it is
# off here by decision rather than by inference about any particular asset. Stating it also keeps the
# behaviour from depending on features that know nothing about each other: main.cpp happens to
# self-confirm before the subsystem threads start whenever ENABLE_TOF_CHAIN is set, and arm() happens
# to ask boot_is_img_confirmed() after that, so the default would decline to arm in this
# configuration today -- a coincidence of ordering is not a policy.
#
# What it gives up, stated here rather than discovered later: nothing in this image resets the board
# if the Package thread wedges, and the board watchdog is not a substitute, because everything else
# goes on feeding it.
set(TOF_PACKAGE_AUTO_DEADMAN OFF)

# No Kconfig fragment and no devicetree overlay of its own. CONFIG_IMG_MANAGER and
# CONFIG_STREAM_FLASH arrive with overlays/tof_release.conf, and the stored L7 blob is read through
# flash_area_*, which prj.conf already configures with CONFIG_FLASH_MAP.
#
# The CAN identifiers are deliberately absent too: 0x21A and 0x21B live in src/tof_can_ids.hpp, and
# TOF_PACKAGE_AUTO_DATA_ID / TOF_PACKAGE_AUTO_HEALTH_ID exist to override that registry on a bench.
# A release image must not carry an override.
#
# TWO POPULATIONS IN ONE IMAGE SHARE THE L7 WATCHDOG, AND THAT IS NOT SETTLED BY THE ABSENCE OF A
# CONFLICTING FLAG. runtime_progress::activity::l7 is a single slot, written by the chain in
# tof_acquisition.cpp and by the Package path in tof_package_device.cpp (watched_uld_operation); the
# l7_expected gate is one boolean, set true by tof_chain_controller.cpp and tof_package_runtime.cpp
# alike; and tof_task_watchdog.cpp judges L7 liveness from that one slot. So progress by a healthy
# population can satisfy the judgement while the other one is wedged inside a ULD operation, in
# either direction, and two operations in flight leave begun - ended at 2 with the in-flight bound
# timed from the first begin. tof_watchdog_feeder's long_operation_ flag is likewise flat, with no
# depth count -- latent rather than live, since only the Package path declares one today. None of
# this is fixed here, and none of it is a reason not to build the image; it is what has to be
# verified before the image is a release candidate.
