# THE DEFAULT BUILD FOR THIS BRANCH, which is why it is one include and nothing else. The Makefile
# has FEATURES_FILE ?= cmake/features_dasher.cmake, so this file is what `make firmware`,
# `make firmware_two_state_ksw` and `make firmware_interlock` build when nothing is passed, and what
# a release tag publishes.
#
# ToF IS PART OF THE DASHER BUILD HERE, NOT A VARIANT OF IT. Cliff, the hanging-object grid and the
# Package sensor all ship by default, so a machine gets the sensing it is fitted with instead of a
# separately named image somebody has to remember to build -- and the release entry needs no feature
# file of its own, which is what `make firmware` producing the shipped image means. A path whose
# sensor is absent reports a standing fault and nothing consumes it, because the ROS nodes are
# opt-in; that is what makes one image for every fitment workable.
#
# WHAT IT COSTS IS FLASH, and the margin is the thing to watch: the default signs at roughly
# 258,440 B against the 261,712 B signed ceiling, about 1.25% spare, and it only links at all
# because of the fragment features_package_tof_release.cmake brings in. Every variant built from
# this file pays the same, so a variant that adds code -- two-state key switch, interlock -- has to
# be measured rather than assumed. features_dasher_base.cmake is the Dasher without any of it.
include(${CMAKE_CURRENT_LIST_DIR}/features_package_tof_release.cmake)
