# The Dasher machine itself, and nothing it is fitted with: the two flags that describe the robot
# rather than a feature of it.
#
# THIS FILE EXISTS TO KEEP THE INCLUDES ACYCLIC. features_dasher.cmake is the Makefile's default
# FEATURES_FILE and now carries the ToF feature as well, so the ToF selection files cannot include
# it -- they would be including the file that includes them. Splitting the machine from the feature
# is what lets both sides have these two flags.
#
# A Dasher WITHOUT ToF is built from this file directly:
#   make firmware VERSION=<version> FEATURES_FILE=cmake/features_dasher_base.cmake
# which is also the configuration to fall back to when a flash overflow has to be bisected.

# PACO uses a Roboteq SBLMG2360T motor driver with non-excitation electromagnetic brakes.
# ROS handles braking via CANopen (Gear Stop), so the firmware must not cut motor power
# on bumper/sensor/manual-switch events.
set(ENABLE_PUSH_MODE 1)

# Dasher has no safety LiDAR connected, so OSSD1/OSSD2 stay low and the robot would stay in emergency stop.
set(BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST 1)
