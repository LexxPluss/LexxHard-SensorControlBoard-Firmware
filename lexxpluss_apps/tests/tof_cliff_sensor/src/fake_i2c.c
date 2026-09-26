/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/init.h>

#include "fake_i2c.h"

#define FAKE_I2C_ADDR 0x29

struct fake_i2c_xfer fake_i2c_log[FAKE_I2C_MAX_XFERS];
int fake_i2c_count;

static uint8_t read_data[FAKE_I2C_MAX_BYTES];
static size_t read_len;
static int fail_nth;
static int fail_err;

void fake_i2c_set_read_data(const uint8_t *data, size_t len)
{
	read_len = (len > sizeof(read_data)) ? sizeof(read_data) : len;
	memcpy(read_data, data, read_len);
}

void fake_i2c_fail_on(int nth_xfer, int err)
{
	fail_nth = nth_xfer;
	fail_err = err;
}

void fake_i2c_reset(void)
{
	memset(fake_i2c_log, 0, sizeof(fake_i2c_log));
	fake_i2c_count = 0;
	fail_nth = 0;
	fail_err = 0;
	read_len = 0;
}

bool fake_i2c_all_addressed(uint16_t addr7)
{
	for (int i = 0; i < fake_i2c_count && i < FAKE_I2C_MAX_XFERS; i++) {
		if (fake_i2c_log[i].addr != addr7) {
			return false;
		}
	}
	return fake_i2c_count > 0;
}

static int fake_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr)
{
	int idx = fake_i2c_count++;

	ARG_UNUSED(target);

	if (idx < FAKE_I2C_MAX_XFERS) {
		struct fake_i2c_xfer *rec = &fake_i2c_log[idx];
		int segs = (num_msgs > FAKE_I2C_MAX_SEGS) ? FAKE_I2C_MAX_SEGS : num_msgs;

		rec->addr = (uint16_t)addr;
		rec->num_segs = (uint8_t)num_msgs;
		for (int i = 0; i < segs; i++) {
			rec->segs[i].len = (uint16_t)msgs[i].len;
			rec->segs[i].flags = msgs[i].flags;
			if ((msgs[i].flags & I2C_MSG_READ) == 0U && msgs[i].buf != NULL) {
				size_t n = msgs[i].len;

				if (n > FAKE_I2C_MAX_BYTES) {
					n = FAKE_I2C_MAX_BYTES;
				}
				memcpy(rec->segs[i].data, msgs[i].buf, n);
			}
		}
	}

	if (fail_nth != 0 && fake_i2c_count == fail_nth) {
		return fail_err;
	}

	for (int i = 0; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_READ) != 0U && msgs[i].buf != NULL) {
			for (size_t b = 0; b < msgs[i].len; b++) {
				msgs[i].buf[b] = (read_len > 0) ? read_data[b % read_len] : 0U;
			}
		}
	}
	return 0;
}

static const struct i2c_emul_api fake_api = {
	.transfer = fake_transfer,
};

/* i2c_emul_register reads emul->target->dev->name for its log line, so the target has
 * to be a real struct emul with a device. Pointing it at the bus itself is enough:
 * nothing else in the emulated path dereferences it. */
static struct i2c_emul fake_emul;

static const struct emul fake_emul_target = {
	.dev = DEVICE_DT_GET(DT_NODELABEL(i2c2)),
};

static int fake_i2c_init(void)
{
	fake_emul.target = &fake_emul_target;
	fake_emul.api = &fake_api;
	fake_emul.addr = FAKE_I2C_ADDR;
	return i2c_emul_register(DEVICE_DT_GET(DT_NODELABEL(i2c2)), &fake_emul);
}

SYS_INIT(fake_i2c_init, APPLICATION, 1);
