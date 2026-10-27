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
