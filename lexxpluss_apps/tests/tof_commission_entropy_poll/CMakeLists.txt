# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tof_commission_entropy_poll)

# Header-only, and that is the point: tof_commission_entropy.cpp is nailed to the st,stm32-rng node
# and cannot be compiled here, so the decision this suite drives had no test at all before it was
# separated out.
target_sources(app PRIVATE src/test_main.cpp)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_compile_definitions(app PRIVATE ENABLE_TOF_AUTO_COMMISSION)
