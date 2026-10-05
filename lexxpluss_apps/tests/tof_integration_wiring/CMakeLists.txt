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


cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_integration_wiring_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
set(ULD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l4cx_uld)
set(L7_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l7cx_uld)
set(CONTRACT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/can)

# THE WIRING ITSELF, WITH THE GRID FLAG ON. Every other suite that links tof_cliff_runtime.cpp
# builds it WITHOUT ENABLE_TOF_L7_ULD, so the grid publisher's init, the two-publisher fan-out and
# the grid sink are compiled in neither direction there: 20 green cases said nothing about them.
#
# This binary is the flag-on build of the same production sources. Both vendor ULDs are stubbed --
# the L7 one answering, the L4 one refusing -- because what is under test is which modules are
# connected to which, not what ST's drivers do. See the two stub files for why they differ.
target_sources(app PRIVATE
    src/test_main.cpp
    src/l4_silent_stubs.cpp
    src/l7_answering_stubs.cpp
    ${APP_SRC_DIR}/tof_acquisition.cpp
    ${APP_SRC_DIR}/tof_cliff_packer.cpp
    ${APP_SRC_DIR}/tof_cliff_publisher.cpp
    ${APP_SRC_DIR}/tof_cliff_runtime.cpp
    ${APP_SRC_DIR}/tof_cliff_can.cpp
    ${APP_SRC_DIR}/tof_commissioning.cpp
    ${APP_SRC_DIR}/tof_enumerator.cpp
    ${APP_SRC_DIR}/tof_grid_packer.cpp
    ${APP_SRC_DIR}/tof_grid_publisher.cpp
    ${APP_SRC_DIR}/tof_l7_sensor.cpp
    ${APP_SRC_DIR}/tof_mapping_authority.cpp
    ${APP_SRC_DIR}/tof_mapping_proof.cpp
    ${APP_SRC_DIR}/tof_tail_isolation.cpp
)
target_compile_definitions(app PRIVATE ENABLE_TOF_CHAIN ENABLE_TOF_CLIFF_ULD ENABLE_TOF_L7_ULD)
target_include_directories(app PRIVATE
    ${APP_SRC_DIR}
    ${CONTRACT_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tof_commissioning/src
    ${ULD_DIR}/zephyr
    ${ULD_DIR}/upstream
    ${ULD_DIR}/upstream/modules
    ${ULD_DIR}/upstream/porting
    ${L7_DIR}/zephyr
    ${L7_DIR}/upstream/modules
)
