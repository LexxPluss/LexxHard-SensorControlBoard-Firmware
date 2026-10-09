/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The VL53L7CX's entry points, answering as a healthy sensor would.
 *
 * tof_acquisition.cpp carries the real grid ops table in an ENABLE_TOF_L7_ULD build, and that table
 * names the adapter, which names the vendor. This suite drives the grid path from the far end --
 * acquisition reads, the adapter converts, the hook carries the sample to the publisher -- so the
 * vendor half has to produce a frame. A stub that refused would leave a correctly wired path
 * looking exactly like a disconnected one, which is the thing these cases exist to tell apart.
 *
 * Stubbed rather than real because what is under test is which modules are connected to which, not
 * what ST's driver does. The suite that drives the vendor through its own state machine is
 * tests/tof_commissioning, where the fake is controllable because the question there is what
 * reaches the ULD. The contrast with l4_silent_stubs.cpp alongside is deliberate.
 */

#include <stddef.h>
#include <stdint.h>

extern "C" {
#include "vl53l7cx_api.h"
}

namespace lexxhard::tof_l7_runtime {

/* A BLOB OF THE RIGHT LENGTH AND NO CONTENT. The adapter checks the size before it touches the
 * sensor -- a short buffer fails at the firmware stage with -EPROTO -- so a grid position cannot
 * open at all without one. Its bytes are never read here, because vl53l7cx_init() below is the stub
 * that would have downloaded them. What the real blob has to satisfy is the record's length and
 * hash, and tests/tof_l7_blob is where that is checked. */
static uint8_t blob_[VL53L7CX_FIRMWARE_DOWNLOAD_SIZE];

const uint8_t *firmware_data()
{
    return blob_;
}

size_t firmware_size()
{
    return sizeof(blob_);
}

}  // namespace lexxhard::tof_l7_runtime

extern "C" {

void vl53l7cx_port_clear_error(void) {}

int vl53l7cx_port_error(void)
{
    return 0;
}

/* A SENSOR THAT ANSWERS, which the cliff suite's stubs deliberately are not: there, the grid
 * publisher is driven directly and the vendor half only has to link. Here the whole chain is under
 * test -- acquisition reads, the adapter converts, the hook fires -- so the stub has to produce a
 * frame or nothing travels and a disconnected hook would look exactly like a quiet sensor. */
uint8_t vl53l7cx_init(VL53L7CX_Configuration *)
{
    return VL53L7CX_STATUS_OK;
}

uint8_t vl53l7cx_set_resolution(VL53L7CX_Configuration *, uint8_t)
{
    return VL53L7CX_STATUS_OK;
}

uint8_t vl53l7cx_set_ranging_frequency_hz(VL53L7CX_Configuration *, uint8_t)
{
    return VL53L7CX_STATUS_OK;
}

uint8_t vl53l7cx_start_ranging(VL53L7CX_Configuration *)
{
    return VL53L7CX_STATUS_OK;
}

uint8_t vl53l7cx_stop_ranging(VL53L7CX_Configuration *)
{
    return VL53L7CX_STATUS_OK;
}

uint8_t vl53l7cx_check_data_ready(VL53L7CX_Configuration *, uint8_t *is_ready)
{
    if (is_ready != nullptr)
        *is_ready = 1;
    return VL53L7CX_STATUS_OK;
}

uint8_t vl53l7cx_get_ranging_data(VL53L7CX_Configuration *, VL53L7CX_ResultsData *out)
{
    if (out == nullptr)
        return VL53L7CX_STATUS_ERROR;

    memset(out, 0, sizeof(*out));
    for (size_t z = 0; z < VL53L7CX_RESOLUTION_8X8; ++z) {
        out->nb_target_detected[z] = 1;
        /* 5 is the ULD's "valid" verdict; the publisher's low-confidence policy is about 6 and 9,
         * and this suite is not the place to exercise it. */
        out->target_status[z] = 5;
        out->distance_mm[z] = 1234;
    }
    return VL53L7CX_STATUS_OK;
}

}  // extern "C"
