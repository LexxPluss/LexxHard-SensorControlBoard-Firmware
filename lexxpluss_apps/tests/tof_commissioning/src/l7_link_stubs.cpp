/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Link stubs for mapping-to-publisher tests, which inject samples directly.
 * No blob is available and vendor operations fail; this does not simulate ranging.
 */

#include <stddef.h>
#include <stdint.h>

extern "C" {
#include "vl53l7cx_api.h"
}

namespace lexxhard::tof_l7_runtime {

const uint8_t *firmware_data()
{
    return nullptr;   // no verified payload here, so open() refuses at the firmware stage
}

size_t firmware_size()
{
    return 0;
}

}  // namespace lexxhard::tof_l7_runtime

extern "C" {

void vl53l7cx_port_clear_error(void) {}

int vl53l7cx_port_error(void)
{
    return 0;
}

/* Latched by the real port when VL53L7CX_SwapBuffer() is handed a bad buffer. This suite links no
 * ULD, so nothing can trip it and "no fault" is the only honest answer. */
int vl53l7cx_port_argument_fault(void)
{
    return 0;
}

uint8_t vl53l7cx_init(VL53L7CX_Configuration *)
{
    return VL53L7CX_STATUS_ERROR;
}

uint8_t vl53l7cx_set_resolution(VL53L7CX_Configuration *, uint8_t)
{
    return VL53L7CX_STATUS_ERROR;
}

uint8_t vl53l7cx_set_ranging_frequency_hz(VL53L7CX_Configuration *, uint8_t)
{
    return VL53L7CX_STATUS_ERROR;
}

uint8_t vl53l7cx_start_ranging(VL53L7CX_Configuration *)
{
    return VL53L7CX_STATUS_ERROR;
}

uint8_t vl53l7cx_stop_ranging(VL53L7CX_Configuration *)
{
    return VL53L7CX_STATUS_ERROR;
}

uint8_t vl53l7cx_check_data_ready(VL53L7CX_Configuration *, uint8_t *is_ready)
{
    if (is_ready != nullptr)
        *is_ready = 0;
    return VL53L7CX_STATUS_ERROR;
}

uint8_t vl53l7cx_get_ranging_data(VL53L7CX_Configuration *, VL53L7CX_ResultsData *)
{
    return VL53L7CX_STATUS_ERROR;
}

}  // extern "C"
