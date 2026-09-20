# Copyright (c) 2026, LexxPluss Inc.
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


# Two layers, deliberately separate binaries' worth of concern in one image:
#
#   test_port_traffic.c   the real Zephyr I2C path, through an emulated controller,
#                         so the 16-bit big-endian index, the two write segments with
#                         no intermediate STOP, the read's repeated start and final
#                         STOP, and the errno mapping are pinned against the API the
#                         production code actually calls.
#
#   test_read_once.c      the adapter, with the ULD's entry points replaced by fakes.
#                         The ULD's .c files are NOT compiled here: emulating the
#                         VL53L4CX register map well enough to drive it would freeze
#                         a vendor-internal register sequence into our tests, which is
#                         precisely the coupling to avoid. The fakes reproduce the
#                         ULD's observable defects instead, including a transport
#                         failure masked by a successful return code.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_cliff_sensor_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
set(ULD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l4cx_uld)
set(L7_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l7cx_uld)

#   test_publisher.cpp    acquisition -> packer -> a fake CAN sink. The sink is a
#                         function pointer, so the whole publish path is exercised with
#                         no Zephyr CAN driver linked; the real glue is a separate commit
#                         so its flash cost can be measured on its own. The publication
#                         gate is injected here so a case can drive both sides of it, and
#                         the real gate is pinned in both directions: PROVEN publishes,
#                         every other state suppresses. Re-inserting a clamp above it, or
#                         removing the gate, fails here rather than shipping.
target_sources(app PRIVATE
    src/fake_i2c.c
    src/test_port_traffic.c
    src/test_read_once.c
    src/test_acquisition.cpp
    src/test_publisher.cpp
    src/test_stream_loop.cpp
    src/test_grid_publisher.cpp
    ${APP_SRC_DIR}/tof_cliff_sensor.c
    ${APP_SRC_DIR}/tof_acquisition.cpp
    ${APP_SRC_DIR}/tof_cliff_packer.cpp
    ${APP_SRC_DIR}/tof_cliff_publisher.cpp
    ${APP_SRC_DIR}/tof_grid_packer.cpp
    ${APP_SRC_DIR}/tof_grid_publisher.cpp
    ${APP_SRC_DIR}/tof_l7_sensor.cpp
    src/l7_vendor_stubs.cpp
    ${ULD_DIR}/zephyr/vl53lx_platform.c
    ${ULD_DIR}/zephyr/vl53l4cx_bus_io.c
)

# The acquisition layer is compiled only under ENABLE_TOF_CHAIN in production, so the
# test defines it too. The chain controller itself is NOT linked here - the test fakes
# chain_lock() with a real k_mutex, which keeps the lock assertions honest without
# dragging in the bus and the shell.
#
# ENABLE_TOF_L7_ULD is here for the grid publisher, which is guarded on all three. It pulls in
# NO vendor code: the publisher consumes tof_l7::sample, which is vendor-free on purpose, so
# this suite still compiles without a line of ST's VL53L7CX driver. The adapter that does use
# it has its own suite.
target_compile_definitions(app PRIVATE ENABLE_TOF_CHAIN ENABLE_TOF_CLIFF_ULD ENABLE_TOF_L7_ULD)
set(CONTRACT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/can)
target_include_directories(app PRIVATE
    ${APP_SRC_DIR}
    ${CONTRACT_DIR}
    ${ULD_DIR}/zephyr
    ${ULD_DIR}/upstream
    ${ULD_DIR}/upstream/modules
    ${ULD_DIR}/upstream/porting
    ${L7_DIR}/zephyr
    ${L7_DIR}/upstream/modules
)
target_compile_options(app PRIVATE -Wno-unused-parameter)
