# Feature selection for the ToF release image.
# Built with the ordinary release entry, which is the one that also produces out/firmware.bin:
#   make firmware_initial VERSION=<version> FEATURES_FILE=cmake/features_tof_release.cmake
#
# This is the Dasher variant PLUS the ToF chain, so it includes features_dasher.cmake rather than
# restating its flags: a Dasher that gains ToF must not silently lose ROS-controlled braking
# (ENABLE_PUSH_MODE). features_dasher.cmake is also where BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST
# comes from, which is what confines this image to a machine with no safety lidar fitted (DS20002).
include(${CMAKE_CURRENT_LIST_DIR}/features_dasher.cmake)

# The whole ToF feature in one image: chain, L4 cliff, L7 grid and commissioning downlink.
# firmware_tof_full shares these four ToF flags, but lacks the release permissions and the
# inherited Dasher flags; its size is not a like-for-like comparison.
set(ENABLE_TOF_CHAIN 1)
set(ENABLE_TOF_CLIFF_ULD ON)
set(ENABLE_TOF_L7_ULD ON)
set(ENABLE_TOF_AUTO_COMMISSION 1)

# Overlays stack in this order, and tof_release.overlay is last because it is the only one that
# grants commissioning permission -- the two before it leave the downlink answering `disabled`.
# lexxpluss_apps/CMakeLists.txt includes this file before find_package(Zephyr), which is what lets
# these lists take effect at all; it appends forensics_dtcm.overlay and, for L7, tof_l7.conf after.
list(APPEND EXTRA_DTC_OVERLAY_FILE
    ${CMAKE_CURRENT_LIST_DIR}/../overlays/tof_chain.overlay
    ${CMAKE_CURRENT_LIST_DIR}/../overlays/auto_commission.overlay
    ${CMAKE_CURRENT_LIST_DIR}/../overlays/tof_release.overlay)

list(APPEND EXTRA_CONF_FILE
    ${CMAKE_CURRENT_LIST_DIR}/../overlays/tof_integration_slim.conf
    ${CMAKE_CURRENT_LIST_DIR}/../overlays/auto_commission.conf
    ${CMAKE_CURRENT_LIST_DIR}/../overlays/tof_release.conf)
