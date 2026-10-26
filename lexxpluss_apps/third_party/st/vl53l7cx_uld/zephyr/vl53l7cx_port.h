/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef LEXXPLUSS_VL53L7CX_PORT_H_
#define LEXXPLUSS_VL53L7CX_PORT_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ONE CONTEXT PER DEVICE, NOT ONE PER IMAGE.
 *
 * The ULD collapses every platform failure into an 8-bit status, so the caller clears this before
 * one ULD operation and reads it afterwards to recover the original Zephyr errno. That state used
 * to be file-scope in platform.c, with the bus compiled in as DT_NODELABEL(i2c2) beside it. The
 * stated reason was that the ST callback carries no context; it does -- every transfer callback
 * takes VL53L7CX_Platform* -- and only VL53L7CX_SwapBuffer() does not.
 *
 * What the old shape cost was not tidiness. Two L7 on two controllers would have shared one errno
 * and one completion count, so a wedged bus could be read as a fault on the healthy one, and a
 * recovery pass that asks "did anything answer" would have been answered about the wrong device.
 *
 * SINGLE-THREAD OWNERSHIP IS STILL REQUIRED per context: the clear-call-read sequence is not
 * atomic as a whole, and two threads sharing one context would interleave into each other's
 * answers. Separate contexts on separate threads are now independent, which is the point. */
struct vl53l7cx_port_context {
	/* Must be ready before any transfer; the port checks on every call rather than caching, so
	 * a controller that goes away is reported rather than assumed. */
	const struct device *bus;
	/* First errno since the last clear. First, not last: a later failure is usually a
	 * consequence of the first and overwriting would bury the cause. */
	atomic_t first_error;
	/* COMPLETED chunk transactions since the last clear, which is a different question from
	 * "was there an error" and cannot be answered from the error alone.
	 *
	 * The recovery pass needs it. vl53l7cx_is_alive() issues four transfers with no early
	 * return and only the FIRST error is kept, so -ENXIO on its own cannot distinguish an
	 * address where nothing answered -- the ordinary cold boot -- from a survivor that
	 * acknowledged some of the exchange and NACKed the rest. The ULD's own alive flag cannot
	 * either: it is set only when BOTH identity bytes match, so it reads zero for a device that
	 * answered one read and refused the other.
	 *
	 * Counts COMPLETED i2c_transfer() calls, so a transaction that was partly acknowledged
	 * before failing does not count. That makes a non-zero value evidence that something
	 * answered, and zero only weak evidence that nothing did -- which is the direction that
	 * matters here, because the conclusion drawn from zero is the unremarkable one. */
	atomic_t completed_transfers;
};

/* Clears both counters. Null is accepted and does nothing, so a caller recovering from a failed
 * construction does not have to branch. */
void vl53l7cx_port_clear_error(struct vl53l7cx_port_context *port);
int vl53l7cx_port_error(const struct vl53l7cx_port_context *port);
uint32_t vl53l7cx_port_completed_transfers(const struct vl53l7cx_port_context *port);

/* THE ONE FAULT WITH NO DEVICE TO BLAME.
 *
 * VL53L7CX_SwapBuffer() is the single ST callback that receives no platform pointer, so a null or
 * misaligned buffer handed to it cannot be attributed to a bus or a sensor. It is a programming
 * error in the ULD or in this port -- never something a device did -- and it is latched here
 * rather than dropped so that a build which trips it can be caught by a test instead of silently
 * swapping nothing. Deliberately NOT folded into a context's first_error: doing that would let an
 * image-wide bug surface as one device's transport fault. */
int vl53l7cx_port_argument_fault(void);
void vl53l7cx_port_clear_argument_fault(void);

/* Largest single payload transfer proven on the real differential I2C chain. Larger ULD requests
 * are segmented with a fresh 16-bit big-endian register index per transaction.
 *
 * Proven on the J29 chain at its bitrate. It is applied to every context because it is the only
 * measured bound this project has; a second bus does not inherit the evidence, it inherits the
 * conservative limit. */
enum { VL53L7CX_PORT_MAX_TRANSFER = 328 };

#ifdef __cplusplus
}
#endif

#endif
