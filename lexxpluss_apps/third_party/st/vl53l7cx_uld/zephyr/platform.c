/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "platform.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "vl53l7cx_api.h"
#include "vl53l7cx_port.h"

#ifdef TOF_DIAG_HANG
/* DEV ONLY (see src/tof_diag_i2c.hpp). Records what this layer asked of the bus, so a transfer that
 * never returns can still be named afterwards: which sensor, which register, and which chunk of a
 * request the controller has to split again. */
void lexx_tof_port_begin(uint32_t addr8, uint16_t addr7, int is_read, uint16_t reg_index,
                         uint32_t total_len, uint32_t chunk_index, uint32_t chunk_off,
                         uint32_t chunk_len, const uint8_t *buf_base);
void lexx_tof_port_end(int rc);
#endif

_Static_assert(VL53L7CX_MAX_RESULTS_SIZE <= VL53L7CX_PORT_MAX_TRANSFER,
               "the selected L7 grid fields must fit one proven transfer");

static const struct device *const l7_bus = DEVICE_DT_GET(DT_NODELABEL(i2c2));
static atomic_t first_error;

static void remember_error(int rc)
{
    if (rc != 0) {
        (void)atomic_cas(&first_error, 0, rc);
    }
}

void vl53l7cx_port_clear_error(void)
{
    atomic_set(&first_error, 0);
}

int vl53l7cx_port_error(void)
{
    return (int)atomic_get(&first_error);
}

static int address7(const VL53L7CX_Platform *platform, uint16_t *out)
{
    if (platform == NULL || out == NULL || platform->address < 0x10U ||
        platform->address > 0xEEU || (platform->address & 1U) != 0U) {
        return -EINVAL;
    }
    *out = platform->address >> 1;
    return 0;
}

static int write_chunk(uint16_t address, uint16_t reg, const uint8_t *data, uint32_t size)
{
    uint8_t index[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFFU)};
    struct i2c_msg messages[2] = {
        {.buf = index, .len = sizeof index, .flags = I2C_MSG_WRITE},
        {.buf = (uint8_t *)data, .len = size, .flags = I2C_MSG_WRITE | I2C_MSG_STOP},
    };

    if (size == 0U) {
        messages[0].flags |= I2C_MSG_STOP;
        return i2c_transfer(l7_bus, messages, 1, address);
    }
    return i2c_transfer(l7_bus, messages, 2, address);
}

static int read_chunk(uint16_t address, uint16_t reg, uint8_t *data, uint32_t size)
{
    uint8_t index[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFFU)};
    struct i2c_msg messages[2] = {
        {.buf = index, .len = sizeof index, .flags = I2C_MSG_WRITE},
        {.buf = data, .len = size, .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP},
    };

    return i2c_transfer(l7_bus, messages, 2, address);
}

static uint8_t transfer(VL53L7CX_Platform *platform, uint16_t register_address, uint8_t *values,
                        uint32_t size, bool read)
{
    uint16_t address;
    int rc = address7(platform, &address);

    if (rc != 0 || (size != 0U && values == NULL)) {
        remember_error(rc != 0 ? rc : -EINVAL);
        return VL53L7CX_STATUS_INVALID_PARAM;
    }
    /* Reject the whole request before the first transaction. Detecting the overflow only when a
     * later chunk begins would leave a partial register write behind and then report an ordinary
     * parameter error, which is not an operation the caller can safely retry. */
    if (size != 0U && (size - 1U) > (uint32_t)UINT16_MAX - register_address) {
        remember_error(-EOVERFLOW);
        return VL53L7CX_STATUS_INVALID_PARAM;
    }
    if (!device_is_ready(l7_bus)) {
        remember_error(-ENODEV);
        return VL53L7CX_STATUS_ERROR;
    }

    if (size == 0U) {
        rc = write_chunk(address, register_address, NULL, 0);
        if (rc != 0) {
            remember_error(rc);
            return VL53L7CX_STATUS_ERROR;
        }
        return VL53L7CX_STATUS_OK;
    }

    uint32_t done = 0;
    uint32_t chunk_index = 0;
    do {
        const uint32_t remaining = size - done;
        const uint32_t take = remaining < VL53L7CX_PORT_MAX_TRANSFER
                                  ? remaining
                                  : VL53L7CX_PORT_MAX_TRANSFER;
        const uint32_t indexed = (uint32_t)register_address + done;

#ifdef TOF_DIAG_HANG
        /* The payload base, not the register index bytes: the controller driver splits anything
         * over 255 bytes again, and the distance from here to its current.buf is the only way to
         * see which of those segments it stopped in. */
        lexx_tof_port_begin(platform->address, address, read ? 1 : 0, (uint16_t)indexed, size,
                            chunk_index, done, take, values + done);
#endif
        rc = read ? read_chunk(address, (uint16_t)indexed, values + done, take)
                  : write_chunk(address, (uint16_t)indexed, values + done, take);
#ifdef TOF_DIAG_HANG
        lexx_tof_port_end(rc);
#endif
        ++chunk_index;
        if (rc != 0) {
            remember_error(rc);
            return VL53L7CX_STATUS_ERROR;
        }
        done += take;
    } while (done < size);

    return VL53L7CX_STATUS_OK;
}

uint8_t VL53L7CX_RdByte(VL53L7CX_Platform *platform, uint16_t register_address, uint8_t *value)
{
    return VL53L7CX_RdMulti(platform, register_address, value, 1U);
}

uint8_t VL53L7CX_WrByte(VL53L7CX_Platform *platform, uint16_t register_address, uint8_t value)
{
    return VL53L7CX_WrMulti(platform, register_address, &value, 1U);
}

uint8_t VL53L7CX_RdMulti(VL53L7CX_Platform *platform, uint16_t register_address, uint8_t *values,
                         uint32_t size)
{
    if (size == 0U) {
        remember_error(-EINVAL);
        return VL53L7CX_STATUS_INVALID_PARAM;
    }
    return transfer(platform, register_address, values, size, true);
}

uint8_t VL53L7CX_WrMulti(VL53L7CX_Platform *platform, uint16_t register_address, uint8_t *values,
                         uint32_t size)
{
    return transfer(platform, register_address, values, size, false);
}

void VL53L7CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
    if (buffer == NULL || (size % sizeof(uint32_t)) != 0U) {
        remember_error(-EINVAL);
        return;
    }
    for (uint16_t i = 0; i + sizeof(uint32_t) <= size; i += sizeof(uint32_t)) {
        const uint32_t value = sys_get_be32(&buffer[i]);
        sys_put_le32(value, &buffer[i]);
    }
}

uint8_t VL53L7CX_WaitMs(VL53L7CX_Platform *platform, uint32_t time_ms)
{
    (void)platform;
    k_msleep((int32_t)time_ms);
    return VL53L7CX_STATUS_OK;
}
