# Feature flags for the PACO robot variant.
# Passed to west build via: -- -DFEATURES_FILE=cmake/features_paco.cmake
#
# PACO uses a Roboteq SBLMG2360T motor driver with non-excitation electromagnetic brakes.
# ROS handles braking via CANopen (Gear Stop), so the firmware must not cut motor power
# on bumper/sensor/manual-switch events.

set(ENABLE_PUSH_MODE 1)
