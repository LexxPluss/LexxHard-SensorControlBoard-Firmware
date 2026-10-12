# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause
#
# The quiesce classification and its record, compiled pure: no Zephyr device, no bus, no vendor
# object. The rule is four cases and an ordering, so it is cheaper to test exhaustively than to
# reason about -- which is the point of extracting it out of the adapter in the first place.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_l7_quiesce_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/test_main.cpp
    ${APP_SRC_DIR}/tof_l7_quiesce.cpp)
target_compile_definitions(app PRIVATE ENABLE_TOF_L7_ULD)
target_include_directories(app PRIVATE ${APP_SRC_DIR})

# THE CALL-SITE GATE. Three of this round's properties are about WHERE something is called, which
# no unit test can see; the checker says which and why, and it runs standalone so its own failures
# can be demonstrated. The first version of this gate counted one call anywhere in the adapter and
# would have passed a write moved into the probe hook -- review found that, and the checker now
# locates the hook body.
find_program(GATE_PYTHON NAMES python3 REQUIRED)
execute_process(
    COMMAND ${GATE_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/check_call_sites.py ${APP_SRC_DIR}
    RESULT_VARIABLE GATE_RESULT
    ERROR_VARIABLE GATE_OUTPUT
)
if(NOT GATE_RESULT EQUAL 0)
    message(FATAL_ERROR "call-site gate failed:\n${GATE_OUTPUT}")
endif()
