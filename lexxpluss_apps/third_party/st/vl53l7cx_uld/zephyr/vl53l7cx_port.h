/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LEXXPLUSS_VL53L7CX_PORT_H_
#define LEXXPLUSS_VL53L7CX_PORT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The ULD collapses every platform failure into an 8-bit status. The acquisition thread clears
 * this before one ULD operation and reads it afterwards, so the original Zephyr errno survives.
 * It is process-global because the ST platform callback carries no context; single-thread ULD
 * ownership is therefore a correctness requirement, not merely a performance choice. */
void vl53l7cx_port_clear_error(void);
int vl53l7cx_port_error(void);

/* Largest single payload transfer proven on the real differential I2C chain. Larger ULD requests
 * are segmented with a fresh 16-bit big-endian register index per transaction. */
enum { VL53L7CX_PORT_MAX_TRANSFER = 328 };

#ifdef __cplusplus
}
#endif

#endif
