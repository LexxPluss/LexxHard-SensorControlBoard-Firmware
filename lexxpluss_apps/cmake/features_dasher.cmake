# Feature flags for the PACO robot variant.
# Passed to west build via: -- -DFEATURES_FILE=cmake/features_dasher.cmake
#
# PACO uses a Roboteq SBLMG2360T motor driver with non-excitation electromagnetic brakes.
# ROS handles braking via CANopen (Gear Stop), so the firmware must not cut motor power
# on bumper/sensor/manual-switch events.

set(ENABLE_PUSH_MODE 1)

# Dasher has no safety LiDAR connected, so OSSD1/OSSD2 stay low and the robot would stay in emergency stop.
set(BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST 1)

# The Dasher family, and the ToF release image that includes this file, have no LED strips.
set(ENABLE_LED_STRIP 0)

# Dasher has no PGV; its RS485 port (uart4) and the pgv_en/pgv_en_n pins are free for other use.
set(ENABLE_PGV 0)

# Dasher carries the LCS host: the RS485 port (uart4) and the pgv_en/pgv_en_n pins belong to it.
set(ENABLE_LCS 1)

# Print the counter names (<name>=<value>) in the shell command `lcs stat`. Costs about 440 B of flash.
# set(ENABLE_LCS_DIAG_NAMES 1)

# ToF is part of the Dasher build on this branch: this one include brings the whole feature -- L4
# cliff, the L7 hanging-object grid, the Package sensor and its flash fit -- into the default image,
# so `make firmware`, `make firmware_two_state_ksw` and `make firmware_interlock` all carry it and a
# release tag publishes it. The values set above win over the included files' copies, which are
# guarded; the chain-only selection stays a named variant (cmake/features_tof_release.cmake).
include(${CMAKE_CURRENT_LIST_DIR}/features_package_tof_release.cmake)
