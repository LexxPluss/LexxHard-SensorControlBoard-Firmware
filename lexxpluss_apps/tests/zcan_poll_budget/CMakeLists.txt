# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause

# The per-pass budget, exercised through the production pollers themselves rather than a model of
# them: the headers are included as the firmware includes them and the message queues are real
# kernel queues. Only zcan_bounded_send::send is replaced, which is why zcan_bounded_send.cpp is
# not linked here -- its own behaviour is covered in tests/zcan_bounded_send.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zcan_poll_budget_test)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
    src/poll_probe.cpp
    src/test_imu.cpp
    src/test_board.cpp
    src/test_dfu.cpp
    src/test_actuator_service.cpp)
target_include_directories(app PRIVATE ${APP_SRC_DIR})
