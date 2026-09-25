/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The two ULD calls the boot recovery pass is built on, wired to the real device.
 *
 * The pass itself (tof_l7_recovery.hpp) is pure logic over an injected pair of function pointers,
 * for the usual reason: it decides what happens on every boot and must be testable without a bus.
 * This is the other half -- the adapter that turns those two calls into I2C traffic -- and it is a
 * separate translation unit because it is the only part that needs the ULD headers.
 *
 * THE SCRATCH. Both ULD calls take a VL53L7CX_Configuration, which is well over a kilobyte because
 * of its temporary buffer, so this does not put one on a stack. The caller passes a tof_l7::sensor
 * that is not open yet -- on the product path one of the runtime's own grid objects, before
 * anything has been opened into it. The adapter zeroes it before each call and leaves it zeroed
 * afterwards, which is the state open() requires.
 *
 * WHY ZEROING IS THE POINT rather than a tidiness measure: a zeroed configuration carries
 * is_auto_stop_enabled == 0, which is what sends stop_ranging down its provoke-MCU-stop path. That
 * path is the one that works on a sensor this SCB has never spoken to -- the one whose session was
 * started by an instance of this firmware that no longer exists.
 *
 * ERROR MAPPING, and why it is worth a file's worth of care. The backported STM32 I2C driver keeps
 * patch 0001's classification: a clean NACK is -ENXIO, a bus fault is -EIO and a timeout is
 * -ETIMEDOUT. So "nobody at this address" is distinguishable from "the bus could not carry the
 * question", and the pass depends on that distinction: the first is the ordinary cold boot and
 * costs nothing, the second is a fault that must not be read as an empty chain.
 */

#pragma once

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_L7_ULD)

#include "tof_l7_recovery.hpp"

namespace lexxhard::tof_l7 {
struct sensor;
}

namespace lexxhard::tof_l7_recovery {

/* `scratch` must be a sensor object that is not open. It is zeroed on every call, so passing one
 * that holds a live session would discard the firmware's side of it -- which is why the product
 * path calls this before any open and never after. */
ops uld_ops(tof_l7::sensor *scratch);

}  // namespace lexxhard::tof_l7_recovery

#endif
