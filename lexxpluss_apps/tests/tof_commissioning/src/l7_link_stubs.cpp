/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The VL53L7CX entry points and the verified-blob accessor, present so the grid configuration of
 * this suite links.
 *
 * WHY THEY ARE NOT A SIMULATION. This configuration exists to ask one question -- does a mapping a
 * proof installed key the grid descriptors, and do keyed descriptors reach the bus -- and it asks
 * it by handing the publisher a sample directly. Nothing here is meant to range: what a real sample
 * does to the ULD belongs to tests/tof_l7_port and tests/tof_l7_sensor, and reproducing it here
 * would freeze a second opinion about the vendor's behaviour into a suite about wiring.
 *
 * SO EVERY FUNCTION FAILS, deliberately. A test in this binary that reaches a sensor gets an error
 * rather than a plausible-looking sample, and a grid bring-up during the end-to-end case refuses at
 * the firmware stage instead of pretending a device is ranging. That refusal is expected: one
 * source that will not open must not stop the others, and the cliff path these tests also drive is
 * unaffected by it.
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
