# Copyright (c) 2022, LexxPluss Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr)
project(lexxpluss_apps)

FILE(GLOB app_sources src/*.cpp)
FILE(GLOB ros_sources ../ros_msgs/*.cpp)
target_sources(app PRIVATE ${app_sources} ${ros_sources})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../ros_msgs)

if(ENABLE_INTERLOCK)
    add_definitions(-DENABLE_INTERLOCK)
endif()

if(USE_TWO_STATE_KEY_SWITCH)
    add_definitions(-DUSE_TWO_STATE_KEY_SWITCH)
endif()

# Diagnostic-only: bypass safety-lidar assertion for stationary AUTO_CHARGE testing.
# Must NEVER be enabled on a robot that is allowed to drive.
if(BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST)
    add_definitions(-DBYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST)
    message(WARNING "UNSAFE BUILD: BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST is enabled. "
                    "Safety lidar assertion is forced to false. "
                    "DO NOT flash to a robot allowed to drive.")
endif()

# Diagnostic-only: ToF differential-I2C bring-up shell (AMRSW-2322 Phase D).
# Enables the `tof_diag` command group and disables the tug encoder polling
# thread, the only other i2c2 user. E-stop handling is NOT affected.
# tof_diag_pinctrl.c is C (PINCTRL_DT_DEFINE does not compile as C++) and is
# outside the src/*.cpp glob, so it is added explicitly here.
if(TOF_I2C_DIAG)
    add_definitions(-DTOF_I2C_DIAG)
    target_sources(app PRIVATE src/tof_diag_pinctrl.c)
    message(WARNING "DIAGNOSTIC BUILD: TOF_I2C_DIAG is enabled. "
                    "Tug encoder polling is disabled and the tof_diag shell owns i2c2. "
                    "Not a production image.")
endif()

if(VERSION)
    add_definitions(-DVERSION=${VERSION})
endif()
