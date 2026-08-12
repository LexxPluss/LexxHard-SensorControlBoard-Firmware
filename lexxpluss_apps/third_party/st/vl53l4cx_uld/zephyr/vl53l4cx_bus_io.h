/*
 * Copyright (c) 2024-2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The VL53L4CX_IO_t block this port registers with the BSP wrapper.
 */

#ifndef LEXXPLUSS_VL53L4CX_BUS_IO_H_
#define LEXXPLUSS_VL53L4CX_BUS_IO_H_

#include "vl53l4cx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Fills pIO for a sensor at the given 7-bit address. Init and DeInit are checked
 * no-ops, GetTick is real because the BSP wrapper polls with it, and WriteReg and
 * ReadReg are placeholders that fail loudly - see the .c for why. */
void vl53l4cx_bus_io_fill(VL53L4CX_IO_t *pIO, uint16_t address_7bit);

/* Counts calls to the placeholder transport callbacks. Must stay zero: the tests
 * assert that the production path never routes traffic through them. */
uint32_t vl53l4cx_bus_io_placeholder_calls(void);

#ifdef __cplusplus
}
#endif

#endif /* LEXXPLUSS_VL53L4CX_BUS_IO_H_ */
