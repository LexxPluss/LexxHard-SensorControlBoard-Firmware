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

# "NO LATER PROBE OR STOP REWRITES THE RECORD" IS CHECKED, NOT PROMISED.
#
# The property review asked for is that a quiesce's snapshot survives whatever runs next. The code
# achieves it by having exactly ONE writer, which is a claim about call sites rather than about a
# function -- so no unit test can see it. This counts them at configure time, the same way this
# directory's sibling suite refuses to build when a vendor patch stops applying.
file(GLOB_RECURSE APP_SOURCES ${APP_SRC_DIR}/*.cpp)
set(WRITERS 0)
foreach(src ${APP_SOURCES})
    file(STRINGS ${src} hits REGEX "tof_l7_quiesce::remember\\(")
    list(LENGTH hits n)
    math(EXPR WRITERS "${WRITERS} + ${n}")
    if(n GREATER 0 AND NOT src MATCHES "tof_package_bench\\.cpp$")
        message(FATAL_ERROR
            "tof_l7_quiesce::remember() is called from ${src}. The record is written by the quiesce "
            "hook and by nothing else -- a second writer is how a later probe or stop comes to "
            "overwrite the observation the record exists to keep.")
    endif()
endforeach()
if(NOT WRITERS EQUAL 1)
    message(FATAL_ERROR
        "expected exactly one tof_l7_quiesce::remember() call site, found ${WRITERS}. Zero means "
        "nothing records the snapshot; more than one means something other than the quiesce can "
        "write it.")
endif()
