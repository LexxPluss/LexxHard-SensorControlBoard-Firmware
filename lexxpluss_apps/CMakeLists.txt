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

# An L7 image verifies the stored blob on the main stack at boot, which does not fit the default
# main stack: see overlays/tof_l7.conf for the per-frame measurement. Kconfig is fixed by
# find_package(Zephyr) below, so the fragment is added here, and only for L7 builds -- every other
# image is configured exactly as before.
if(ENABLE_TOF_L7_ULD)
    list(APPEND EXTRA_CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/overlays/tof_l7.conf)
endif()

# The DTCM reservation that turns the watchdog tombstone into reserved memory rather than memory
# nobody happens to use yet. Added here because Kconfig and the devicetree are fixed by
# find_package(Zephyr) below, and applied to EVERY image rather than named per target: a build that
# carries the record must not be configurable without it. See overlays/forensics_dtcm.overlay, and
# src/tof_watchdog_tombstone.cpp for the compile-time check that this line is what satisfies.
list(APPEND EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_LIST_DIR}/overlays/forensics_dtcm.overlay)

# THE HARDWARE WATCHDOG HAS EXACTLY ONE FEEDER, AND THIS IS WHAT KEEPS IT THAT WAY.
#
# The property is not checkable in the binary: wdt_feed() is a Zephyr syscall that, without
# userspace, inlines into an indirect call through the driver's API table, so there is no symbol to
# count call sites of. What IS checkable is the source, and this is where a second call site would
# be introduced -- most plausibly by somebody restoring the old one-second timer feed while chasing
# an unrelated reset, which is exactly the change that would silently undo all of this.
file(GLOB_RECURSE wdt_feed_sources ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp ${CMAKE_CURRENT_LIST_DIR}/src/*.hpp)
set(wdt_feed_callers "")
foreach(f ${wdt_feed_sources})
    file(READ ${f} contents)
    # A real call, not the name in prose. Every actual call passes a device, so it opens with an
    # identifier or an address-of; the comments that discuss this rule write `wdt_feed()` with
    # nothing between the parentheses. That difference is what separates them.
    if(contents MATCHES "[^a-zA-Z_]wdt_feed[ \t]*\\([ \t]*[A-Za-z_&]")
        get_filename_component(base ${f} NAME)
        if(NOT base STREQUAL "tof_watchdog_feeder.cpp")
            list(APPEND wdt_feed_callers ${base})
        endif()
    endif()
endforeach()
if(wdt_feed_callers)
    message(FATAL_ERROR
        "wdt_feed() is called outside tof_watchdog_feeder.cpp: ${wdt_feed_callers}. "
        "The hardware watchdog has exactly one feeder, it is a thread, and it feeds only while the "
        "watched work keeps finishing. A second call site -- especially one reachable from an "
        "interrupt -- restores the failure this was built to end. See tof_watchdog_feeder.hpp.")
endif()

find_package(Zephyr)
project(lexxpluss_apps)

FILE(GLOB app_sources src/*.cpp)
FILE(GLOB ros_sources ../ros_msgs/*.cpp)
target_sources(app PRIVATE ${app_sources} ${ros_sources})

# LOAD-BEARING, NOT TIDY, AND -Werror=switch-enum ALONE DID NOT DO IT.
#
# tof_commission_map.cpp translates the firmware's internal enums into wire values. A switch that
# does not handle an enumerator is not an error in C++; it falls through, and the wire silently
# gains a meaning nobody wrote.
#
# The guard used to be -Werror=switch-enum on its own, with a comment claiming a new enumerator
# would fail the build. It does not. GCC reports a missing enumerator in a switch with no `default`
# under -Wswitch, and -Werror=switch-enum promotes only -Wswitch-enum -- so the diagnostic that
# actually fires stayed a warning. Three enumerators this baseline has were unmapped underneath it
# and fell through to internal_error: begin_refusal::nonce_exhausted, commit_refusal::wrong_issuer
# and proof refusal::challenge_foreign.
#
# Both are needed, and they catch different mistakes. -Werror=switch catches the missing enumerator.
# -Werror=switch-enum catches the next person silencing it with a `default:` label, which would make
# -Wswitch go quiet while the fall-through came back.
#
# IT HAS TO BE HERE AS WELL AS IN THE SUITES. A suite can only fail for a tree it compiles, and the
# obligation -- "a new enumerator must be mapped" -- belongs to the production image, where the
# mapper actually ships.
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/src/tof_commission_map.cpp
    PROPERTIES COMPILE_OPTIONS "-Werror=switch;-Werror=switch-enum")
# docs/can holds the generated production half of the CAN wire contracts --
# identifiers, field encodings and the cliff status classification table. Production
# code includes it directly, exactly as the tests do, so there is no vendored copy to
# drift from the contract it was generated from. Only tof_cliff_contract.h is meant to
# be reachable this way; the vector header and the commissioning timing profile are
# separate artefacts and are not on this path by accident.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../ros_msgs
                                       ${CMAKE_CURRENT_SOURCE_DIR}/../docs/can)

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

# The automatic-commissioning build: the cliff image plus the hardware entropy the session token
# needs. A separate flag, a separate overlay and a separate Kconfig fragment, so that every other
# image -- product, chain, cliff -- comes out byte for byte as it did before this existed.
#
# WHAT THIS FLAG DOES AND DOES NOT DO. It compiles the downlink runtime, the bus binding and the
# entropy module, and tof_chain_controller::init() now starts them through tof_commission_wiring: the
# receive filter goes in, a session token is drawn and announced, and every request is answered.
#
# WHAT IT STILL DOES NOT DO is commission anything. The two deployment decisions that govern that --
# entertain a request at all, and re-enumerate the chain when one arrives -- are the devicetree
# properties commission-profile-enabled and commission-enumeration-permitted, absent by default and
# absent from the auto-commission overlay. An image acquires neither by linking the code that could
# perform them.
if(ENABLE_TOF_AUTO_COMMISSION)
    if(NOT ENABLE_TOF_CLIFF_ULD)
        message(FATAL_ERROR "ENABLE_TOF_AUTO_COMMISSION requires ENABLE_TOF_CLIFF_ULD")
    endif()
    add_definitions(-DENABLE_TOF_AUTO_COMMISSION)
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

# Hanging-object ToF: the vendored VL53L7CX ULD, our Zephyr platform layer, and the blob machinery
# that lets the 86,016-byte device firmware live in storage_partition instead of in the signed
# image. Off by default, and nothing in this branch turns it on. The acquisition layer now has a
# grid lifecycle -- open, configure, start, read, stop, dispatched per model -- but no production
# descriptor binds it: build_descriptors() still hands every grid position the named -ENOSYS stub
# in tof_acquisition.cpp. So with the flag off the image is unchanged and with it on the driver is
# present and unreachable.
#
# The explicit source list compiles the ULD core and our platform only. The plugins and the STM32
# BSP wrapper stay vendored and do not enter any image; see third_party/st/vl53l7cx_uld/UPSTREAM.md.
if(ENABLE_TOF_L7_ULD)
    # Same dependency, and for the same reason, as the cliff ULD: i2c2 belongs to the tug encoder
    # until ENABLE_TOF_CHAIN hands it over. Refuse the combination at configure time.
    if(NOT ENABLE_TOF_CHAIN)
        message(FATAL_ERROR "ENABLE_TOF_L7_ULD requires ENABLE_TOF_CHAIN")
    endif()

    # THE FILESYSTEM IS NOT REMOVED HERE, and the reason is a measurement rather than inheritance.
    #
    # The development images that carried this driver also dropped the FAT stack, on the grounds
    # that an L7 image does not fit beside it. Measured on this branch against the clean Zephyr
    # tree, chain + cliff with the filesystem still configured, 261,712 B ceiling:
    #
    #   flag off  build-pub-prod-off  245,452 B signed, 16,260 B spare
    #   flag on   build-pub-prod-on   246,776 B signed, 14,936 B spare   (+1,324 B)
    #
    # READ THAT CAREFULLY RATHER THAN AS A PASS. The +1,324 B is not the driver's cost. No
    # production table binds the L7 adapter: build_descriptors() still gives every grid position
    # the named -ENOSYS stub and leaves grid_ops null, so the grid lifecycle is reachable only
    # through a function-pointer table nothing fills, and --gc-sections keeps the dispatch while
    # dropping most of what it would have called. Compiled, the ULD objects are 11,531 B of text --
    # a reference for what binding them will add, not a prediction, since inlining and
    # --gc-sections move it again once there are callers.
    #
    # The grid publisher and packer are NOT in this list and are in no product image; they are
    # built and tested by lexxpluss_apps/tests/tof_cliff_sensor only. Their in-image cost, and the
    # cost of an adapter that is actually bound, are both unmeasured and belong to the wiring
    # change that creates the callers. The runtime monitoring and the boot recovery are likewise
    # not measured here.
    # So the headroom after wiring is an estimate and no more: 14,936 spare less 11,531 of ULD text
    # is about 3.3 KB, before the publisher, the packer, a bound adapter or anything else that is
    # still to come. It is a reason to expect the decision to be forced later, and it is NOT an
    # account of why the development images overflowed, which was never measured contribution by
    # contribution.
    #
    # It is not forced by THIS change, and removing ~26 KB of product content is not something a
    # driver's build block should do on the way past. It is a change about what the image CONTAINS,
    # it needs its own justification and its own verification, and whoever makes it must re-measure
    # on the configuration they are actually shipping rather than quote this comment.

    include(${CMAKE_CURRENT_SOURCE_DIR}/third_party/st/vl53l7cx_uld/vl53l7cx_sources.cmake)
    target_sources(app PRIVATE ${VL53L7CX_ULD_PRODUCTION_SOURCES})
    # The generated expectation header names tof_l7_blob::expectation, a type owned by src/.
    # Quoted includes otherwise search from the vendor directory and cannot find it.
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
                                           ${VL53L7CX_ULD_INCLUDE_DIRS})
    # Third-party code, exempted narrowly rather than by deleting files or by a global -Wno-*.
    set_source_files_properties(${VL53L7CX_PATCHED_DIR}/vl53l7cx_api.c PROPERTIES
        COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-sign-compare;-Wno-unused-but-set-variable")
    add_definitions(-DENABLE_TOF_L7_ULD)
endif()

if(VERSION)
    add_definitions(-DVERSION=${VERSION})
endif()
