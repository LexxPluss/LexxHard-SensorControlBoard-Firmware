/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * A recording I2C target behind Zephyr's emulated controller. The point of going
 * through i2c_emul rather than replacing the port's calls is that the production code
 * keeps using i2c_transfer and i2c_write_read unchanged, so the recorded messages are
 * the ones the driver really builds - including the flags that i2c_write_read sets on
 * the caller's behalf.
 */

#ifndef FAKE_I2C_H_
#define FAKE_I2C_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FAKE_I2C_MAX_XFERS 16
#define FAKE_I2C_MAX_SEGS  4
#define FAKE_I2C_MAX_BYTES 8

struct fake_i2c_seg {
	uint16_t len;
	uint8_t flags;
	uint8_t data[FAKE_I2C_MAX_BYTES]; /* write payload as sent, truncated */
};

struct fake_i2c_xfer {
	uint16_t addr; /* the 7-bit address the driver passed down */
	uint8_t num_segs;
	struct fake_i2c_seg segs[FAKE_I2C_MAX_SEGS];
};

/* Recorded traffic since the last reset. */
extern struct fake_i2c_xfer fake_i2c_log[FAKE_I2C_MAX_XFERS];
extern int fake_i2c_count;

/* Bytes handed back for read segments, cycled if the read is longer. */
void fake_i2c_set_read_data(const uint8_t *data, size_t len);

/* Fail the nth transfer since the reset, counting from 1, with this errno. Zero
 * disables injection. Only one injection point at a time, which is enough: each test
 * places it at the first, a middle or the last transfer of an operation. */
void fake_i2c_fail_on(int nth_xfer, int err);

void fake_i2c_reset(void);

/* True when every recorded transfer used the given 7-bit address. */
bool fake_i2c_all_addressed(uint16_t addr7);

#endif /* FAKE_I2C_H_ */
