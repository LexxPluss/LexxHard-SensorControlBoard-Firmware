/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Platform surface consumed by the ST VL53L7CX ULD. This is intentionally smaller than the
 * upstream callback example: every L7 is on immutable i2c2, and the verified device-firmware
 * pointer is per configuration object rather than a process-global array.
 */

#ifndef LEXXPLUSS_VL53L7CX_PLATFORM_H_
#define LEXXPLUSS_VL53L7CX_PLATFORM_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    /* ST stores the 8-bit wire address here (default 0x52). The Zephyr port validates it and
     * converts exactly once to the 7-bit controller address. */
    uint16_t address;
    /* Filled only from a verified tof_l7_blob::blob_view. The upstream patch refuses init when
     * this is null and uses it for the three firmware download ranges. */
    const uint8_t *firmware;
    /* Kept with the pointer so the patched ULD can prove that all three fixed ranges fit before the
     * first I2C operation. A valid pointer without its bound is not an authorised buffer. */
    size_t firmware_size;
} VL53L7CX_Platform;

enum { VL53L7CX_FIRMWARE_DOWNLOAD_SIZE = 0x15000U };

#ifndef VL53L7CX_NB_TARGET_PER_ZONE
#define VL53L7CX_NB_TARGET_PER_ZONE (1U)
#endif

/* Keep ULD conversion enabled and every field used by the frozen grid contract present. */
#define PROCESSOR_LITTLE_ENDIAN
#define SWAP_UINT16(x) (x)
#define SWAP_UINT32(x) (x)

uint8_t VL53L7CX_RdByte(VL53L7CX_Platform *p_platform, uint16_t register_address,
                        uint8_t *value);
uint8_t VL53L7CX_WrByte(VL53L7CX_Platform *p_platform, uint16_t register_address,
                        uint8_t value);
uint8_t VL53L7CX_RdMulti(VL53L7CX_Platform *p_platform, uint16_t register_address,
                         uint8_t *values, uint32_t size);
uint8_t VL53L7CX_WrMulti(VL53L7CX_Platform *p_platform, uint16_t register_address,
                         uint8_t *values, uint32_t size);
void VL53L7CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L7CX_WaitMs(VL53L7CX_Platform *p_platform, uint32_t time_ms);

#endif
