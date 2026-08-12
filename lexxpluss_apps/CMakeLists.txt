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

# ToF chain enumeration glue (AMRSW-2322 Phase 2). Owns i2c2: the tug
# encoder polling thread is not started and resolves to "disconnected".
# Mutually exclusive with the diagnostic build (enforced by #error).
if(ENABLE_TOF_CHAIN)
    add_definitions(-DENABLE_TOF_CHAIN)
    message(WARNING "TOF CHAIN BUILD: tug encoder polling is disabled; "
                    "the ToF chain owns i2c2. Driver return-code hardware "
                    "verification is still pending.")
endif()

# Cliff ToF: the vendored VL53L4CX ULD and its Zephyr platform layer. Off by
# default, so the production image is byte-identical until the acquisition path
# is real. TOF_CLIFF_BUDGET selects a build point of the flash/RAM budget series.
if(ENABLE_TOF_CLIFF_ULD)
    include(${CMAKE_CURRENT_SOURCE_DIR}/third_party/st/vl53l4cx_uld/vl53l4cx_sources.cmake)
    target_sources(app PRIVATE ${VL53L4CX_ULD_PRODUCTION_SOURCES})
    target_include_directories(app PRIVATE ${VL53L4CX_ULD_INCLUDE_DIRS})
    # Third-party code, exempted narrowly rather than by deleting files or by a
    # global -Wno-*. Only this target's compile options are affected.
    set_source_files_properties(${VL53L4CX_ULD_PRODUCTION_SOURCES} PROPERTIES
        COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-sign-compare;-Wno-unused-but-set-variable;-Wno-missing-field-initializers")
    # The sensor layer itself. Compiled whenever the ULD is, so the budget series
    # measures the real production file rather than a probe-local copy of it.
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/tof_cliff_sensor.c)
    # src/*.cpp is globbed at the top of this file, so tof_acquisition.cpp is already a
    # source of every build. It needs this definition to compile its body: without the
    # ULD's include directories a chain build without the cliff sensors failed on
    # vl53l4cx.h, which is exactly what the guard prevents.
    add_definitions(-DENABLE_TOF_CLIFF_ULD)
    if(TOF_CLIFF_BUDGET)
        target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/tof_cliff_budget_probe.c)
        add_definitions(-DTOF_CLIFF_BUDGET=${TOF_CLIFF_BUDGET})
        if(ENABLE_TOF_CHAIN AND TOF_CLIFF_BUDGET GREATER_EQUAL 6)
            target_sources(app PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/src/tof_cliff_budget_scheduler.cpp)
        endif()
    endif()
endif()

if(VERSION)
    add_definitions(-DVERSION=${VERSION})
endif()
