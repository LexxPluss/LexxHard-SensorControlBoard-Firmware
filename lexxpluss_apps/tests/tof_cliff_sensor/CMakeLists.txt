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
# The generated contract headers the publisher and the packer pin against.
set(CONTRACT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/can)
set(ULD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l4cx_uld)

target_sources(app PRIVATE
    src/fake_i2c.c
    src/test_port_traffic.c
    src/test_read_once.c
    src/test_acquisition.cpp
    src/test_publisher.cpp
    ${APP_SRC_DIR}/tof_cliff_sensor.c
    ${APP_SRC_DIR}/tof_acquisition.cpp
    ${APP_SRC_DIR}/tof_cliff_packer.cpp
    ${APP_SRC_DIR}/tof_cliff_publisher.cpp
    ${ULD_DIR}/zephyr/vl53lx_platform.c
    ${ULD_DIR}/zephyr/vl53l4cx_bus_io.c
)

# The acquisition layer is compiled only under ENABLE_TOF_CHAIN and ENABLE_TOF_CLIFF_ULD in
# production, so the test defines both. The chain controller itself is NOT linked here: the test
# fakes chain_lock() with a real k_mutex, which keeps the lock assertions honest without dragging
# the bus and the shell into a suite about lifecycle.
target_compile_definitions(app PRIVATE ENABLE_TOF_CHAIN ENABLE_TOF_CLIFF_ULD)
target_include_directories(app PRIVATE
    ${APP_SRC_DIR}
    ${CONTRACT_DIR}
    ${ULD_DIR}/zephyr
    ${ULD_DIR}/upstream
    ${ULD_DIR}/upstream/modules
    ${ULD_DIR}/upstream/porting
)
target_compile_options(app PRIVATE -Wno-unused-parameter)
