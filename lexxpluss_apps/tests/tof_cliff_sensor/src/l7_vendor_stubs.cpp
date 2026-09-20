/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The VL53L7CX's entry points, present so that this binary links, and reached by nothing.
 *
 * tof_acquisition.cpp carries the real grid ops table in an ENABLE_TOF_L7_ULD build, and that
 * table names the adapter, which names the vendor. This suite is about the scheduler, the cliff
 * adapter and the two publishers; it drives the grid path through the publisher's own entry points
 * and never through a sensor. The suite that does drive the vendor is tests/tof_commissioning,
 * where the fake is a controllable one because the question there is what reaches the ULD.
 *
 * Every function below returns a failure for that reason: if a test in this binary ever does reach
 * one, it gets an error rather than a plausible-looking sample.
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
