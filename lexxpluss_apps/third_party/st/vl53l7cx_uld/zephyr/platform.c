/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "platform.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "vl53l7cx_api.h"
#include "vl53l7cx_port.h"

_Static_assert(VL53L7CX_MAX_RESULTS_SIZE <= VL53L7CX_PORT_MAX_TRANSFER,
               "the selected L7 grid fields must fit one proven transfer");

/* The only state left at file scope, and it is not a device's. See vl53l7cx_port.h. */
static atomic_t argument_fault;

static void remember_error(struct vl53l7cx_port_context *port, int rc)
{
    if (port != NULL && rc != 0) {
        (void)atomic_cas(&port->first_error, 0, rc);
    }
}

/* Only where a transaction actually completed. Deliberately not "attempted": the caller uses a
 * non-zero count as evidence that something on the bus answered. */
static void remember_completion(struct vl53l7cx_port_context *port)
{
    if (port != NULL) {
        (void)atomic_inc(&port->completed_transfers);
    }
}

void vl53l7cx_port_clear_error(struct vl53l7cx_port_context *port)
{
    if (port == NULL) {
        return;
    }
    atomic_set(&port->first_error, 0);
    atomic_set(&port->completed_transfers, 0);
}

int vl53l7cx_port_error(const struct vl53l7cx_port_context *port)
{
    /* A caller with no context never issued a transfer, so there is no device errno to report.
     * -ENODEV rather than 0: reporting success for a question that could not be asked is how a
     * missing context would be mistaken for a healthy bus. */
    return port == NULL ? -ENODEV : (int)atomic_get(&port->first_error);
}

uint32_t vl53l7cx_port_completed_transfers(const struct vl53l7cx_port_context *port)
{
    if (port == NULL) {
        return 0U;
    }

    const atomic_val_t seen = atomic_get(&port->completed_transfers);

    return seen < 0 ? UINT32_MAX : (uint32_t)seen;
}

int vl53l7cx_port_argument_fault(void)
{
    return (int)atomic_get(&argument_fault);
}

void vl53l7cx_port_clear_argument_fault(void)
{
    atomic_set(&argument_fault, 0);
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

static int write_chunk(const struct device *bus, uint16_t address, uint16_t reg,
                       const uint8_t *data, uint32_t size)
{
    uint8_t index[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFFU)};
    struct i2c_msg messages[2] = {
        {.buf = index, .len = sizeof index, .flags = I2C_MSG_WRITE},
        {.buf = (uint8_t *)data, .len = size, .flags = I2C_MSG_WRITE | I2C_MSG_STOP},
    };

    if (size == 0U) {
        messages[0].flags |= I2C_MSG_STOP;
        return i2c_transfer(bus, messages, 1, address);
    }
    return i2c_transfer(bus, messages, 2, address);
}

static int read_chunk(const struct device *bus, uint16_t address, uint16_t reg, uint8_t *data,
                      uint32_t size)
{
    uint8_t index[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFFU)};
    struct i2c_msg messages[2] = {
        {.buf = index, .len = sizeof index, .flags = I2C_MSG_WRITE},
        {.buf = data, .len = size, .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP},
    };

    return i2c_transfer(bus, messages, 2, address);
}

static uint8_t transfer(VL53L7CX_Platform *platform, uint16_t register_address, uint8_t *values,
                        uint32_t size, bool read)
{
    uint16_t address;
    int rc = address7(platform, &address);
    /* Resolved before the argument checks so that their errno still lands somewhere when the
     * address is also wrong; a platform with no context records nothing, which is why the null
     * context is refused on its own below rather than being allowed to fail silently later. */
    struct vl53l7cx_port_context *const port = platform != NULL ? platform->port : NULL;

    if (rc != 0 || (size != 0U && values == NULL)) {
        remember_error(port, rc != 0 ? rc : -EINVAL);
        return VL53L7CX_STATUS_INVALID_PARAM;
    }
    /* No context means no controller to address. Refused here, before any argument is trusted,
     * because the alternative is dereferencing a compiled-in bus that this port no longer has. */
    if (port == NULL) {
        return VL53L7CX_STATUS_ERROR;
    }
    /* Reject the whole request before the first transaction. Detecting the overflow only when a
     * later chunk begins would leave a partial register write behind and then report an ordinary
     * parameter error, which is not an operation the caller can safely retry. */
    if (size != 0U && (size - 1U) > (uint32_t)UINT16_MAX - register_address) {
        remember_error(port, -EOVERFLOW);
        return VL53L7CX_STATUS_INVALID_PARAM;
    }
    if (!device_is_ready(port->bus)) {
        remember_error(port, -ENODEV);
        return VL53L7CX_STATUS_ERROR;
    }

    if (size == 0U) {
        rc = write_chunk(port->bus, address, register_address, NULL, 0);
        if (rc != 0) {
            remember_error(port, rc);
            return VL53L7CX_STATUS_ERROR;
        }
        remember_completion(port);
        return VL53L7CX_STATUS_OK;
    }

    uint32_t done = 0;
    do {
        const uint32_t remaining = size - done;
        const uint32_t take = remaining < VL53L7CX_PORT_MAX_TRANSFER
                                  ? remaining
                                  : VL53L7CX_PORT_MAX_TRANSFER;
        const uint32_t indexed = (uint32_t)register_address + done;

        rc = read ? read_chunk(port->bus, address, (uint16_t)indexed, values + done, take)
                  : write_chunk(port->bus, address, (uint16_t)indexed, values + done, take);
        if (rc != 0) {
            remember_error(port, rc);
            return VL53L7CX_STATUS_ERROR;
        }
        remember_completion(port);
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
        remember_error(platform != NULL ? platform->port : NULL, -EINVAL);
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
    /* The one callback ST gives no platform pointer, so this fault belongs to the image rather
     * than to any device. See vl53l7cx_port.h on why it is kept separate from a context's errno. */
    if (buffer == NULL || (size % sizeof(uint32_t)) != 0U) {
        (void)atomic_cas(&argument_fault, 0, -EINVAL);
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
