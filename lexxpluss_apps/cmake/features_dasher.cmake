# THE DEFAULT BUILD FOR THIS BRANCH, which is why it is one include and nothing else. The Makefile
# has FEATURES_FILE ?= cmake/features_dasher.cmake, so this file is what `make firmware`,
# `make firmware_two_state_ksw` and `make firmware_interlock` build when nothing is passed, and what
# a release tag publishes.
#
# ToF IS PART OF THE DASHER BUILD HERE, NOT A VARIANT OF IT. Cliff, the hanging-object grid and the
# Package sensor all ship by default, so the sensing does not sit behind a separately named image
# somebody has to remember to build, and the release entry needs no feature file of its own -- which
# is what `make firmware` producing the shipped image means.
#
# THAT IS A DECISION FOR THIS BRANCH, NOT AN INFERENCE FROM A MEASUREMENT, and the absent-sensor
# argument covers one path only. A machine with no Package sensor fitted publishes a standing fault
# on its diagnostic frame and nothing consumes it, because that ROS node is opt-in and
# use_package_tof defaults to false; that was measured on dasher2. The cliff and L7 paths carry no
# such argument here: they were already in the ToF release selection, their consumers are not opt-in
# in the same way, and what a machine without those sensors fitted does on the consumer side is not
# established in this repository. Flashing this image to a machine is still a fitment question.
#
# WHAT IT COSTS IS FLASH, and the margin is the thing to watch. Against the 261,712 B signed ceiling,
# measured on this tree: the default and the interlock target both sign at 258,440 B with 3,272 B
# spare, and the two-state key switch target at 258,296 B with 3,416 B. It only links at all because
# of the fragment features_package_tof_release.cmake brings in. The interlock figure is equal rather
# than close: ENABLE_INTERLOCK is not referenced by the current source, so that target is the same
# application payload, which says the target still builds and nothing about interlock itself.
#
# Every variant built from this file inherits that 1.25%, so the next one that does add code has to
# be measured rather than assumed. features_dasher_base.cmake is the Dasher without any of it.
include(${CMAKE_CURRENT_LIST_DIR}/features_package_tof_release.cmake)
