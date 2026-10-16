/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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

/* HOW MANY CHUNK TRANSACTIONS COMPLETED since the last clear, which is a different question from
 * "was there an error" and cannot be answered from the error alone.
 *
 * The recovery pass needs it. vl53l7cx_is_alive() issues four transfers with no early return and
 * only the FIRST error is kept, so -ENXIO on its own cannot distinguish an address where nothing
 * answered -- the ordinary cold boot -- from a survivor that acknowledged some of the exchange and
 * NACKed the rest. The ULD's own alive flag cannot either: it is set only when BOTH identity bytes
 * match, so it reads zero for a device that answered one read and refused the other.
 *
 * Counts COMPLETED i2c_transfer() calls, so a transaction that was partly acknowledged before
 * failing does not count. That makes a non-zero value evidence that something answered, and zero
 * only weak evidence that nothing did -- which is the direction that matters here, because the
 * conclusion drawn from zero is the unremarkable one. */
uint32_t vl53l7cx_port_completed_transfers(void);

/* Largest single payload transfer proven on the real differential I2C chain. Larger ULD requests
 * are segmented with a fresh 16-bit big-endian register index per transaction. */
enum { VL53L7CX_PORT_MAX_TRANSFER = 328 };

#ifdef __cplusplus
}
#endif

#endif
