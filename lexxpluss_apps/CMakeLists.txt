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
# docs/can holds the generated production half of the CAN wire contracts --
# identifiers, field encodings and the cliff status classification table. Production
# code includes it directly, exactly as the tests do, so there is no vendored copy to
# drift from the contract it was generated from. Only tof_cliff_contract.h is meant to
# be reachable this way; the vector header and the commissioning timing profile are
# separate artefacts and are not on this path by accident.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../ros_msgs
                                       ${CMAKE_CURRENT_SOURCE_DIR}/../docs/can)

# ONE BOUNDED WAY TO TRANSMIT, enforced here because the unbounded way looks identical at the
# call site. can_send() with a null callback waits for acknowledgement with K_FOREVER whatever
# timeout is passed to it (see src/zcan_bounded_send.hpp), so a bus with no other node awake
# stops the calling thread for good. Every sender goes through zcan_bounded_send::send(); this
# refuses to configure if a null-callback call comes back, in a new module or a merge, rather
# than letting it ship and be found on a vehicle whose host was switched off.
file(GLOB_RECURSE _can_send_sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
                                    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp)
set(_blocking_can_send "")
foreach(_f ${_can_send_sources})
    if(_f STREQUAL ${CMAKE_CURRENT_SOURCE_DIR}/src/zcan_bounded_send.cpp)
        continue()  # the one place allowed to call can_send at all
    endif()
    file(STRINGS ${_f} _hits REGEX "can_send[ \t]*\\(.*(nullptr|NULL)[ \t]*,[ \t]*(nullptr|NULL)")
    if(_hits)
        get_filename_component(_name ${_f} NAME)
        list(APPEND _blocking_can_send ${_name})
    endif()
endforeach()
if(_blocking_can_send)
    message(FATAL_ERROR
        "can_send() with a null callback waits for acknowledgement forever, whatever timeout "
        "it is given: a bus with no other node awake stops the caller permanently. Use "
        "zcan_bounded_send::send() instead. Found in: ${_blocking_can_send}")
endif()

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
# default. With it on, the whole cliff data path is reachable from
# tof_chain_controller, so this option now costs what the feature costs; there is
# no longer a staged series to measure it in pieces. See the retirement note in
# the commit that removed TOF_CLIFF_BUDGET.
if(ENABLE_TOF_CLIFF_ULD)
    # The chain flag is a DEPENDENCY, not a neighbour. i2c2 is the tug encoder's
    # bus until ENABLE_TOF_CHAIN replaces its init with init_disconnected_for_tof()
    # and starts no polling thread for it (see main.cpp). The cliff ULD reaches the
    # same bus, so the combination without the chain flag gives one bus two owners,
    # no chain overlay and no ownership handover. Refuse it at configure time rather
    # than let it link.
    #
    # This used to add "and TOF_CLIFF_BUDGET >= 5 does so from a SYS_INIT at boot,
    # before anyone could intervene". That is no longer true and the sentence is
    # removed rather than left to mislead: the budget probes now register an anchor
    # that keeps the walk linked without executing it during init, because they
    # measure REACHABLE code and a boot-time run only ever probed an unpowered,
    # uncommissioned bus. The dependency below stands on its own without it.
    if(NOT ENABLE_TOF_CHAIN)
        message(FATAL_ERROR
            "ENABLE_TOF_CLIFF_ULD requires ENABLE_TOF_CHAIN: without it the tug "
            "encoder keeps i2c2 and its polling thread, while the cliff ULD "
            "drives the same bus.")
    endif()
    include(${CMAKE_CURRENT_SOURCE_DIR}/third_party/st/vl53l4cx_uld/vl53l4cx_sources.cmake)
    target_sources(app PRIVATE ${VL53L4CX_ULD_PRODUCTION_SOURCES})
    target_include_directories(app PRIVATE ${VL53L4CX_ULD_INCLUDE_DIRS})
    # Third-party code, exempted narrowly rather than by deleting files or by a
    # global -Wno-*. Only this target's compile options are affected.
    set_source_files_properties(${VL53L4CX_ULD_PRODUCTION_SOURCES} PROPERTIES
        COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-sign-compare;-Wno-unused-but-set-variable;-Wno-missing-field-initializers")
    # The sensor layer itself. Compiled whenever the ULD is.
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/tof_cliff_sensor.c)
    # src/*.cpp is globbed at the top of this file, so tof_acquisition.cpp is already a
    # source of every build. It needs this definition to compile its body: without the
    # ULD's include directories a chain build without the cliff sensors failed on
    # vl53l4cx.h, which is exactly what its guard prevents.
    add_definitions(-DENABLE_TOF_CLIFF_ULD)
endif()

if(VERSION)
    add_definitions(-DVERSION=${VERSION})
endif()
