/*
 * Copyright (c) 2024-2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Port-level sticky transport error.
 *
 * The ULD loses transport failures. VL53LX_GetMultiRangingData runs
 * VL53LX_get_device_results and then overwrites Status with SetMeasurementData's
 * result unconditionally (vl53lx_api.c, the two consecutive assignments), so an I2C
 * failure during the results read can come back as success. The BSP wrapper is worse:
 * its poll helper casts VL53LX_GetMeasurementDataReady's return to void.
 *
 * So the port records the FIRST raw Zephyr errno it saw and the caller treats that as
 * authoritative, ahead of the ULD's return code. Reset it immediately before each ULD
 * operation, read it immediately after.
 *
 * Precondition: one acquisition thread. The sticky value is per bus, not per device,
 * because the platform callbacks cannot store anything in the upstream object. Four
 * sensors read sequentially by one thread is exactly the intended use; concurrent use
 * from two threads would interleave the records and must not be added without making
 * this per-device first.
 */

#ifndef LEXXPLUSS_VL53L4CX_PORT_H_
#define LEXXPLUSS_VL53L4CX_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Clear before each ULD operation. */
void vl53l4cx_port_sticky_reset(void);

/* 0 when every transfer since the reset succeeded, otherwise the first raw Zephyr
 * errno observed. Negative, as Zephyr reports it. */
int vl53l4cx_port_sticky_errno(void);

#ifdef __cplusplus
}
#endif

#endif /* LEXXPLUSS_VL53L4CX_PORT_H_ */
