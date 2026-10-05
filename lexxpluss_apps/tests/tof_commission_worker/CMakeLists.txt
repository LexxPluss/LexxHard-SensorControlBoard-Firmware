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
project(tof_commission_worker_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
set(CONTRACT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/can)

# ITS OWN BINARY, AND THAT IS THE POINT. This suite creates the worker thread, and a thread cannot
# be unmade: it outlives the case that created it, the k_thread object it was created on cannot be
# reused, and init() reconfigures the session layer -- which draws a new token and resets the attempt
# budgets, a once-per-boot operation under #116.
#
# Keeping this in the runtime binary meant every later case ran against a live worker and a module
# that had been re-initialised underneath it, and the one case that noticed passed or failed on the
# order ztest happened to choose. Separating the binaries is the whole fix; a lock or a lease here
# would be a mechanism invented to make a mistake survivable that no caller in this firmware makes.
target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_commission_runtime.cpp
    ${APP_SRC_DIR}/tof_commission_session.cpp
    ${APP_SRC_DIR}/tof_commission_wire.cpp
    ${APP_SRC_DIR}/tof_commission_map.cpp
    ${APP_SRC_DIR}/tof_auto_commission.cpp
)
# BOTH FLAGS, and this copy was carried in with only one. -Werror=switch-enum promotes
# -Wswitch-enum, which is not the diagnostic that fires for an enumerator missing from a switch with
# no `default` -- that is -Wswitch. With switch-enum alone the guard never worked, and in #116 three
# enumerators were found unmapped underneath it, falling through to internal_error. switch catches
# the missing enumerator; switch-enum catches the next person silencing it with a `default:` label.
set_source_files_properties(${APP_SRC_DIR}/tof_commission_map.cpp
    PROPERTIES COMPILE_OPTIONS "-Werror=switch;-Werror=switch-enum")
target_compile_definitions(app PRIVATE ENABLE_TOF_CHAIN ENABLE_TOF_CLIFF_ULD)
target_include_directories(app PRIVATE ${APP_SRC_DIR} ${CONTRACT_DIR})
