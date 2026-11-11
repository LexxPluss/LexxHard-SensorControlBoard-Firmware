# The Package ToF release configuration, delivered as a bench candidate that puts itself back.
#
#   make firmware_initial VERSION=<version> FEATURES_FILE=cmake/features_package_tof_bench.cmake
#
# Identical to the release image in every feature that touches sensing. The only difference is who
# owns the test-boot decision, and that difference is the point: this image keeps its unconfirmed
# trailer and arms the deadman, so ANY reset puts the previous image back rather than leaving a
# candidate resident on a machine nobody is watching.
include(${CMAKE_CURRENT_LIST_DIR}/features_package_tof_release.cmake)

# WHAT THIS BUYS, AND WHAT IT COSTS, BOTH STATED. The release image self-confirms about eleven
# seconds into boot -- after thread creation, before any subsystem thread runs -- so its automatic
# rollback covers initialisation only and nothing a running thread can do. That is the window the
# 2026-11-04 incident fell outside of by roughly six tenths of a second, which is why a candidate
# that wedges after its threads start cannot be recovered over the CAN DFU: the reset cascade
# power-cycles the whole machine faster than a host boots, so no network window ever opens.
#
# With the deadman armed the image never confirms, so the FIRST reset from any cause -- watchdog,
# fault, or the deadman's own timer -- reverts. The cost is that it reverts on success too: the
# window is ten minutes from arming, arming happens once every subsystem thread is up, and an auto
# build has no `pkg_tof hold` command to extend it. Ten minutes is the whole test, and a candidate
# worth keeping has to be reflashed as a confirmed image afterwards.
#
# This is a bench target. A resident release image must keep carrying OFF, for the reason stated in
# features_package_tof_release.cmake: putting an unsupervised machine back is the opposite of what a
# release is for.
set(TOF_PACKAGE_AUTO_DEADMAN ON)
