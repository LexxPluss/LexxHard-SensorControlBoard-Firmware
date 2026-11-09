# The Dasher machine itself, and nothing it is fitted with: the flags that describe the robot
# rather than a feature of it. features_dasher.cmake states the same facts for the default build;
# this file is how a named ToF selection states them without including the default file. That
# matters because the change that makes the default include the Package configuration would turn
# such an include into a cycle -- and aliasing would silently grow the chain-only image.
#
# EVERY FLAG IS GUARDED so an includer that has already stated a value keeps it. The guard prevents
# a silent overwrite, nothing more: it does not detect this file and features_dasher.cmake drifting
# apart, and the chain-only path reads only this file, so a change to either file means re-checking
# the resolved flags of both the default and the chain-only selection (cmake -P over both, as the
# release checklist does).

# PACO uses a Roboteq SBLMG2360T motor driver with non-excitation electromagnetic brakes.
# ROS handles braking via CANopen (Gear Stop), so the firmware must not cut motor power
# on bumper/sensor/manual-switch events.
if(NOT DEFINED ENABLE_PUSH_MODE)
    set(ENABLE_PUSH_MODE 1)
endif()

# Dasher has no safety LiDAR connected, so OSSD1/OSSD2 stay low and the robot would stay in emergency stop.
if(NOT DEFINED BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST)
    set(BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST 1)
endif()

# The Dasher family has no LED strips, so every image built from these flags drops the strip driver
# and its three SPI buses; the IMU's SPI stays.
if(NOT DEFINED ENABLE_LED_STRIP)
    set(ENABLE_LED_STRIP 0)
endif()

# Dasher has no PGV. Off is not only a fitment fact: CMakeLists.txt refuses ENABLE_LCS and ENABLE_PGV
# together, because both claim uart4 and the pgv_en/pgv_en_n pins.
if(NOT DEFINED ENABLE_PGV)
    set(ENABLE_PGV 0)
endif()

# Dasher carries the LCS host: the RS485 port (uart4) and the pgv_en/pgv_en_n pins belong to it.
if(NOT DEFINED ENABLE_LCS)
    set(ENABLE_LCS 1)
endif()

# Print the counter names (<name>=<value>) in the shell command `lcs stat`. Costs about 440 B of flash.
# set(ENABLE_LCS_DIAG_NAMES 1)
