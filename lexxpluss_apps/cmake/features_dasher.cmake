# THE DEFAULT BUILD FOR THIS BRANCH, which is why it is one include and nothing else. The Makefile
# has FEATURES_FILE ?= cmake/features_dasher.cmake, so this file is what `make firmware`,
# `make firmware_two_state_ksw` and `make firmware_interlock` build when nothing is passed, and what
# a release tag publishes.
#
# ToF IS PART OF THE DASHER BUILD HERE, NOT A VARIANT OF IT. Cliff, the hanging-object grid and the
# Package sensor all ship by default, so the sensing does not sit behind a separately named image
# somebody has to remember to build, and the release entry needs no feature file of its own -- which
# is what `make firmware` producing the shipped image means. The rest of the Dasher default -- push
# mode, the lidar bypass, no LED strips, no PGV, the LCS host -- lives in
# features_dasher_base.cmake, which the ToF selection includes.
#
# THAT IS A DECISION FOR THIS BRANCH, NOT AN INFERENCE FROM A MEASUREMENT, and the absent-sensor
# argument covers one path only. A machine with no Package sensor fitted publishes a standing fault
# on its diagnostic frame and nothing consumes it, because that ROS node is opt-in and
# use_package_tof defaults to false; that was measured on dasher2. The cliff and L7 paths carry no
# such argument here: they were already in the ToF release selection, their consumers are not opt-in
# in the same way, and what a machine without those sensors fitted does on the consumer side is not
# established in this repository. Flashing this image to a machine is still a fitment question.
#
# WHAT IT COSTS IS FLASH, and the margin is the thing to watch. The LCS host costs more than the
# LED and PGV removals return, so this selection only fits after the flash-fit fragment grew (see
# overlays/tof_package_flashfit.conf for the measured trail). On this tree the default signs at
# 258,408 B of the 261,712 B ceiling, 3,304 B spare, and the two-state key switch target at
# 258,272 B with 3,440 B -- a 1.26% margin. The next feature in does not fit without picolibc
# (AMRSW-2887 PR #132) or a feature coming out; measure any variant that adds code before relying
# on any of this.
include(${CMAKE_CURRENT_LIST_DIR}/features_package_tof_release.cmake)
