# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# The STM32 I2C v2 interrupt handler, on the host, driving the PRODUCTION driver source against a
# simulated register file. Nothing here reimplements the handler: shim/ provides registers and the
# LL side effects, and each test plays the part of the peripheral, one flag at a time.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i2c_stm32_v2_isr_test)

target_sources(app PRIVATE
    src/test_main.c
    $ENV{ZEPHYR_BASE}/drivers/i2c/i2c_ll_stm32_v2.c
)

# shim/ comes FIRST so that <soc.h>, <stm32_ll_i2c.h> and the clock control header resolve to the
# test's register file. Every other <zephyr/...> include still resolves to the real tree, and
# drivers/i2c is on the path for the driver's own private headers -- which are used as they are,
# so the data structures under test are the production ones.
target_include_directories(app BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/shim
    $ENV{ZEPHYR_BASE}/drivers/i2c
)

# Three definitions stand in for what the devicetree and Kconfig give the driver on the board.
# DT_COMPAT_HAS_OKAY_... is what the private header checks before it declares the v2 timing fields,
# so without it struct i2c_stm32_config would differ from the one under test on hardware. The other
# two select the same build of the driver the board runs -- v2, interrupt mode -- rather than the
# polling path, which is the thing the tests are about. None of them changes a decision the handler
# makes; they choose which code exists.
target_compile_definitions(app PRIVATE
    DT_COMPAT_HAS_OKAY_st_stm32_i2c_v2=1
    CONFIG_I2C_STM32_INTERRUPT=1
    CONFIG_I2C_STM32_V2=1
)
