# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause
#
# The declared-long-operation bookkeeping, compiled pure: no watchdog device, no feeder thread, no
# lock. The lock is the owner's (tof_watchdog_feeder holds a spinlock across every call), and what
# is left here is a state machine whose whole behaviour is reachable from a host test -- which is
# the point of extracting it out of the feeder, where the previous two-atomic version could not be
# driven through the orderings that broke it.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_long_operation_test)

target_sources(app PRIVATE src/test_main.cpp)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
