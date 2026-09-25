/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * See tof_l7_recovery_ops.hpp.
 */

#include "tof_l7_recovery_ops.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_L7_ULD)

#include <errno.h>
#include <string.h>

extern "C" {
#include "vl53l7cx_port.h"
}

#include "tof_l7_sensor.hpp"

namespace lexxhard::tof_l7_recovery {

namespace {

/* Both entry points begin here. Zeroing is not hygiene: it is what puts is_auto_stop_enabled at 0
 * and sends stop_ranging down the path that can end a session this firmware did not start. */
VL53L7CX_Configuration *addressed(void *ctx, uint8_t addr_7bit)
{
    auto *const s{static_cast<tof_l7::sensor *>(ctx)};
    if (s == nullptr)
        return nullptr;
    memset(&s->uld, 0, sizeof(s->uld));
    s->uld.platform.address = static_cast<uint16_t>(addr_7bit) << 1;
    return &s->uld;
}

int is_alive(void *ctx, uint8_t addr_7bit, bool *alive)
{
    if (alive == nullptr)
        return -EINVAL;
    *alive = false;

    VL53L7CX_Configuration *const dev{addressed(ctx, addr_7bit)};
    if (dev == nullptr)
        return -EINVAL;

    vl53l7cx_port_clear_error();
    uint8_t answered{0};
    const uint8_t uld_status{vl53l7cx_is_alive(dev, &answered)};
    const int port_errno{vl53l7cx_port_error()};

    /* A clean NACK is an answer, and on a cold boot it is the answer every address gives. It is
     * reported as a completed probe with nobody there rather than as a failure, because a pass
     * that called every cold boot a failure would say nothing about the boots that matter. */
    if (port_errno == -ENXIO)
        return 0;

    /* -EIO or -ETIMEDOUT. The question did not reach the bus, so no answer was heard, and the
     * caller must not treat this as an empty address. */
    if (port_errno != 0)
        return port_errno;

    /* The transport worked and the device did not identify as an L7. Reported the same way as a
     * NACK on purpose: this pass stops L7 sessions and has no business issuing a five-second stop
     * sequence to something that is not one. An ACK with a wrong identity is a real anomaly, and
     * the place that acts on it is enumeration, whose census exists for exactly that. */
    if (uld_status != VL53L7CX_STATUS_OK)
        return 0;

    *alive = answered != 0U;
    return 0;
}

int stop_ranging(void *ctx, uint8_t addr_7bit)
{
    /* Re-addressed rather than relying on the probe having just run. The pure layer promises only
     * that a stop follows a successful probe, not that nothing happened in between, and an adapter
     * that depended on call order would break silently the first time that changed. */
    VL53L7CX_Configuration *const dev{addressed(ctx, addr_7bit)};
    if (dev == nullptr)
        return -EINVAL;

    vl53l7cx_port_clear_error();
    const uint8_t uld_status{vl53l7cx_stop_ranging(dev)};
    if (const int port_errno{vl53l7cx_port_error()}; port_errno != 0)
        return port_errno;

    /* The ULD collapses its own failures into one status byte, and the interesting one here has no
     * transport error behind it: stop_ranging polls the device for up to five seconds waiting for
     * the MCU to stop, and folds a give-up into that byte. -EIO for the whole class, because what
     * the caller does with it is the same -- record it and leave the verdict to enumeration. */
    return uld_status == VL53L7CX_STATUS_OK ? 0 : -EIO;
}

}  // namespace

ops uld_ops(tof_l7::sensor *scratch)
{
    ops o{};
    o.is_alive = is_alive;
    o.stop_ranging = stop_ranging;
    o.ctx = scratch;
    return o;
}

}  // namespace lexxhard::tof_l7_recovery

#endif
