/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The cliff sensor's entry points, refusing every call.
 *
 * tof_acquisition.cpp binds l4_cliff_ops() to these, so linking it needs them -- and nothing in
 * this binary may reach them. The question here is whether the GRID half of the wiring is
 * connected: the real adapter in the descriptors, the hook into the grid publisher, the grid mask
 * in the authorisation. The four cliff positions are present only because the deployment spec puts
 * them there, and their bring-up failing is the state these cases run in.
 *
 * That is why the file stubs the C adapter rather than linking the real tof_cliff_sensor.c: the
 * real one pulls the whole VL53L4CX ULD and its bus layer into a suite that never ranges an L4.
 *
 * -ENOSYS is the fail-loud direction. bring_up() treats a source that will not open as one sensor
 * out of six -- it logs it and carries on, which is the documented behaviour -- so a cliff position
 * that silently started working here would change nothing visible, while a grid position that
 * reached this table by mistake would get an error instead of a plausible-looking sample.
 *
 * The contrast with l7_answering_stubs.cpp is the point: that one ANSWERS, because the grid path is
 * what is under test and a disconnected hook would otherwise look exactly like a quiet sensor.
 */

#include <errno.h>

#include "tof_cliff_sensor.h"

extern "C" {

int tof_cliff_sensor_open(VL53L4CX_Object_t *, uint8_t, struct tof_cliff_read_status *)
{
    return -ENOSYS;
}

int tof_cliff_sensor_configure(VL53L4CX_Object_t *, VL53LX_DistanceModes, uint32_t,
                               struct tof_cliff_read_status *)
{
    return -ENOSYS;
}

int tof_cliff_sensor_start(VL53L4CX_Object_t *, struct tof_cliff_stream_state *,
                           struct tof_cliff_read_status *)
{
    return -ENOSYS;
}

int tof_cliff_sensor_stop(VL53L4CX_Object_t *, struct tof_cliff_read_status *)
{
    return -ENOSYS;
}

int tof_cliff_read_once(VL53L4CX_Object_t *, struct tof_cliff_scratch *,
                        struct tof_cliff_stream_state *, struct tof_cliff_sample *,
                        struct tof_cliff_read_status *)
{
    return -ENOSYS;
}

/* Reached for real: acquisition logs the stage name when a source fails to come up, which in this
 * binary is every cliff position on every bring-up. */
const char *tof_cliff_stage_name(enum tof_cliff_stage)
{
    return "stub";
}

}  // extern "C"
