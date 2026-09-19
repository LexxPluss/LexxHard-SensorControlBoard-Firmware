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
project(tof_commission_runtime_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
set(CONTRACT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/can)

# The runtime adapter over the REAL session machine, sequencer, codec and mapper. The transport and
# the hardware are injected; everything the adapter talks to is the real thing. The IDENTIFIERS here
# are test values and deliberately not the allocated ones: what this suite checks is that the runtime
# uses the identifiers it is given, which a suite using the real pair could not tell from a runtime
# that had them baked in.
target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_commission_runtime.cpp
    ${APP_SRC_DIR}/tof_commission_session.cpp
    ${APP_SRC_DIR}/tof_commission_wire.cpp
    ${APP_SRC_DIR}/tof_commission_map.cpp
    ${APP_SRC_DIR}/tof_auto_commission.cpp
)
set_source_files_properties(${APP_SRC_DIR}/tof_commission_map.cpp
    PROPERTIES COMPILE_OPTIONS "-Werror=switch-enum")
target_compile_definitions(app PRIVATE ENABLE_TOF_CHAIN ENABLE_TOF_CLIFF_ULD)
target_include_directories(app PRIVATE ${APP_SRC_DIR} ${CONTRACT_DIR})
