/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Zephyr glue of the SCB as host of the RS-485 link to the load cell boards (uart4, async API with DMA).
 * msgq carries the CAN frames built from the received uplinks (lexxhard::lcs::CanFrame).
 */

#pragma once

#include <zephyr/kernel.h>

namespace lexxhard::lcs_controller {

void init();
void run(void *p1, void *p2, void *p3);
extern k_thread thread;
extern k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
