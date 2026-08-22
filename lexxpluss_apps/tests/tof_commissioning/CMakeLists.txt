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
project(tof_commissioning_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# The whole transaction, with a REAL k_mutex for the chain and the real authority, proof, enumerator
# and isolation behind it. Only the bus and the quiesce hook are faked, because everything this
# commit is about -- lock ordering, recursion, which exit paths release, what stays open on failure --
# is a property of the real components' interaction and a fake of any of them would assume the
# answer.
# tof_acquisition.cpp is linked because the quiesce is now the real one: the step before the session
# takes the chain used to block on it, and only the real acquisition layer can show that it no longer
# does. Its dependency on the vendor sensor entry points is stubbed in the test rather than satisfied
# by linking the ULD, which belongs to the sensor suite.
# Two suites in one binary, sharing the chain fake: the commissioning transaction and the runtime
# bootstrap. They share it because a second copy of a fake that models the flip-flop chain, the L4/L7
# reset-class asymmetry and the tail-races-ahead defect would drift, and the verdicts depend on
# exactly those details. The runtime suite also links the publisher and packer, because the bootstrap
# it tests is the production one -- wiring the modules up in the test would be a third wiring.
target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_acquisition.cpp
    ${APP_SRC_DIR}/tof_commissioning.cpp
    ${APP_SRC_DIR}/tof_mapping_authority.cpp
    ${APP_SRC_DIR}/tof_mapping_proof.cpp
    ${APP_SRC_DIR}/tof_tail_isolation.cpp
    ${APP_SRC_DIR}/tof_enumerator.cpp
)
target_compile_definitions(app PRIVATE ENABLE_TOF_CHAIN ENABLE_TOF_CLIFF_ULD)
set(ULD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/st/vl53l4cx_uld)
set(CONTRACT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/can)
target_include_directories(app PRIVATE
    ${APP_SRC_DIR}
    ${CONTRACT_DIR}
    ${ULD_DIR}/zephyr
    ${ULD_DIR}/upstream
    ${ULD_DIR}/upstream/modules
    ${ULD_DIR}/upstream/porting
)
