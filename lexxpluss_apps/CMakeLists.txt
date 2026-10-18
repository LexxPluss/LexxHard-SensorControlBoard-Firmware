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

# The bench driver runs the whole ULD path on the shell thread, which the default shell stack does
# not cover: see overlays/tof_package_bench.conf for the measurement. Appended here rather than in
# the ENABLE_TOF_PACKAGE_BENCH block further down, because Kconfig is fixed by find_package(Zephyr)
# below and that block runs after it.
if(ENABLE_TOF_PACKAGE_BENCH)
    list(APPEND EXTRA_CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/overlays/tof_package_bench.conf)
endif()

# The unattended runtime keeps the revert armed unless told otherwise, and the deadman needs the
# image-manager query to ask whether this image is confirmed. Appended here for the same reason as
# the bench's: Kconfig is fixed by find_package(Zephyr) below.
if(ENABLE_TOF_PACKAGE_AUTO AND NOT DEFINED TOF_PACKAGE_AUTO_DEADMAN)
    set(TOF_PACKAGE_AUTO_DEADMAN ON)
endif()
if(ENABLE_TOF_PACKAGE_AUTO AND TOF_PACKAGE_AUTO_DEADMAN)
    list(APPEND EXTRA_CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/overlays/tof_package_auto_deadman.conf)
endif()

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

# Hanging-object ToF: the vendored VL53L7CX ULD, our Zephyr platform layer, and the blob machinery
# that lets the 86,016-byte device firmware live in storage_partition instead of in the signed
# image. Off by default, and nothing in this branch turns it on: the grid operations that would
# call the adapter are still the named -ENOSYS stubs in tof_acquisition.cpp, so with the flag off
# the image is unchanged and with it on the driver is present and unreachable.
#
# The explicit source list compiles the ULD core and our platform only. The plugins and the STM32
# BSP wrapper stay vendored and do not enter any image; see third_party/st/vl53l7cx_uld/UPSTREAM.md.
if(ENABLE_TOF_L7_ULD)
    # NO CHAIN DEPENDENCY, and that is a change from the cliff ULD above rather than an oversight.
    #
    # This block used to refuse ENABLE_TOF_L7_ULD without ENABLE_TOF_CHAIN "for the same reason as
    # the cliff ULD": i2c2 belongs to the tug encoder until the chain hands it over, and a ULD that
    # reaches the same bus would give it two owners. That reason held while the L7 port resolved
    # DT_NODELABEL(i2c2) itself. It no longer does -- the port takes its controller from a
    # per-device vl53l7cx_port_context the caller supplies, and references no devicetree node at
    # all -- so enabling this flag no longer names any bus, and cannot contend for i2c2 by
    # construction.
    #
    # The cliff refusal above stays exactly as it was, because VL53L4CX_I2C_NODE in
    # third_party/st/vl53l4cx_uld/zephyr/vl53lx_platform.c really is DT_NODELABEL(i2c2). The
    # asymmetry is the point: one port has been decoupled and the other has not.
    #
    # WHAT REPLACES THE REFUSAL IS A RUNTIME REFUSAL, not nothing. tof_l7::open() rejects a null
    # context and one that names no controller, and the port refuses every transfer without one.
    # Handing an L7 the i2c2 device while the tug encoder still owns it is now a wiring decision in
    # whoever builds the context, which is where the knowledge lives; a configure-time flag could
    # only have guessed at it.

    # THE FILESYSTEM IS NOT REMOVED HERE, and the reason is a measurement rather than inheritance.
    #
    # The development images that carried this driver also dropped the FAT stack, on the grounds
    # that an L7 image does not fit beside it. Measured on this branch: chain + cliff + L7 with the
    # filesystem still configured signs at 245,504 B against the 261,712 B ceiling, 16,208 B spare,
    # and 72 B more than the same image without the flag.
    #
    # READ THAT CAREFULLY RATHER THAN AS A PASS. It is 72 B because nothing calls the driver: the
    # grid operations are still the -ENOSYS stubs in tof_acquisition.cpp, so --gc-sections drops the
    # ULD, the platform layer, the adapter and the blob machinery. Compiled, those objects are
    # 11,531 B of text -- a reference for what binding them will add, not a prediction, since
    # inlining and --gc-sections move it again once there are callers. The publisher, the runtime
    # monitoring and the boot recovery are each still to come and none of them is measured here.
    # So the headroom after wiring is an estimate of roughly 4.7 KB and no more than that: it is a
    # reason to expect the decision to be forced later, and it is NOT an account of why the
    # development images overflowed, which was never measured contribution by contribution.
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

# Package ToF bench driver. BENCH ONLY, and a separate flag rather than a corner of the L7 one:
# tof_package_l7 has no caller, so --gc-sections drops it and every size measured for the L7 flag
# says nothing about what the lifecycle costs. This is what calls the path, which is the only way
# the linked cost becomes a number -- and binding it into a product entry point instead would make
# a bring-up decision that belongs to the Package configuration, which does not exist yet.
#
# It registers a shell command group and nothing else: no thread, no timer, no SYS_INIT. It never
# provisions the blob, publishes nothing on CAN and moves no shutter; the file says why for each.
#
# No product configuration sets this. The guard inside the source also refuses the combination
# without ENABLE_TOF_L7_ULD, so a bench image cannot be built without the driver it drives.
if(ENABLE_TOF_PACKAGE_BENCH)
    if(NOT ENABLE_TOF_L7_ULD)
        message(FATAL_ERROR
            "ENABLE_TOF_PACKAGE_BENCH requires ENABLE_TOF_L7_ULD: the bench driver exists to call "
            "the L7 lifecycle, and without the driver there is nothing to call.")
    endif()
    if(ENABLE_TOF_PACKAGE_AUTO)
        message(FATAL_ERROR
            "ENABLE_TOF_PACKAGE_BENCH and ENABLE_TOF_PACKAGE_AUTO claim the same device: the shell "
            "commands and the runtime thread would drive one sensor through one set of unlocked "
            "objects. Pick one.")
    endif()
    add_definitions(-DENABLE_TOF_PACKAGE_BENCH)
endif()

# THE UNATTENDED RUNTIME. One thread that binds the Package sensor at boot, keeps it measuring and
# puts every acquisition on can2 with nobody typing anything -- the bench's bind/takeover/stream
# without the shell, over the same device layer and the same publication rule.
#
# IT NEEDS ITS IDENTIFIERS AND ITS RATE FROM THE BUILD, with no defaults anywhere in the source:
# none of the three is registered or accepted, and a number compiled into a header becomes the
# answer by being the only one anybody can find. The source refuses to compile without them, so a
# target that forgets one fails here rather than transmitting on something nobody chose.
if(ENABLE_TOF_PACKAGE_AUTO)
    if(NOT ENABLE_TOF_L7_ULD)
        message(FATAL_ERROR
            "ENABLE_TOF_PACKAGE_AUTO requires ENABLE_TOF_L7_ULD: the runtime exists to drive the "
            "L7 lifecycle, and without the driver there is nothing to drive.")
    endif()
    foreach(required TOF_PACKAGE_AUTO_DATA_ID TOF_PACKAGE_AUTO_HEALTH_ID TOF_PACKAGE_AUTO_HZ)
        if(NOT DEFINED ${required})
            message(FATAL_ERROR
                "ENABLE_TOF_PACKAGE_AUTO needs ${required}: nothing about this stream is "
                "registered or accepted, so there is no default to fall back to.")
        endif()
    endforeach()
    # The budget the policy runs on. Defaulted, because unlike the identifiers these are this
    # project's own decisions about how far an unattended board may go without a human, and they
    # are documented where the policy defines them.
    if(NOT DEFINED TOF_PACKAGE_AUTO_CLAIMS)
        set(TOF_PACKAGE_AUTO_CLAIMS 6)
    endif()
    if(NOT DEFINED TOF_PACKAGE_AUTO_TAKEOVERS)
        set(TOF_PACKAGE_AUTO_TAKEOVERS 1)
    endif()
    if(NOT DEFINED TOF_PACKAGE_AUTO_QUIESCE_FIRST)
        set(TOF_PACKAGE_AUTO_QUIESCE_FIRST 1)
    endif()
    # THE UNATTENDED REVERT, ON BY DEFAULT. Being unconfirmed only protects an image once
    # something resets the board, and the hazard this runtime adds is a Package thread that wedges
    # while the board controller keeps feeding the watchdog -- nothing would reset it. So the
    # first acceptance of a candidate keeps the timer and the residency configuration passes
    # -DTOF_PACKAGE_AUTO_DEADMAN=OFF once the image has been watched. Defaulting it ON means
    # forgetting the flag leaves a machine protected rather than stranded. The window is
    # kWindowMinutes and CANNOT be extended here: `pkg_tof hold` lives in the shell, which this
    # image does not have.
    if(TOF_PACKAGE_AUTO_DEADMAN)
        add_definitions(-DENABLE_TOF_PACKAGE_AUTO_DEADMAN)
        message(STATUS "package_tof runtime: unattended revert ARMED (one window per flash)")
    else()
        message(WARNING
            "package_tof runtime: unattended revert DISABLED. Nothing in this image will reset "
            "the board if the Package thread wedges, so its unconfirmed trailer protects it only "
            "against a reset that comes from somewhere else.")
    endif()
    add_definitions(-DENABLE_TOF_PACKAGE_AUTO
                    -DTOF_PACKAGE_AUTO_DATA_ID=${TOF_PACKAGE_AUTO_DATA_ID}
                    -DTOF_PACKAGE_AUTO_HEALTH_ID=${TOF_PACKAGE_AUTO_HEALTH_ID}
                    -DTOF_PACKAGE_AUTO_HZ=${TOF_PACKAGE_AUTO_HZ}
                    -DTOF_PACKAGE_AUTO_CLAIMS=${TOF_PACKAGE_AUTO_CLAIMS}
                    -DTOF_PACKAGE_AUTO_TAKEOVERS=${TOF_PACKAGE_AUTO_TAKEOVERS}
                    -DTOF_PACKAGE_AUTO_QUIESCE_FIRST=${TOF_PACKAGE_AUTO_QUIESCE_FIRST})
endif()

if(VERSION)
    add_definitions(-DVERSION=${VERSION})
endif()
