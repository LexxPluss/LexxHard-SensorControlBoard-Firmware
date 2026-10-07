/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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
 * and sends stop_ranging down the path that can end a session this firmware did not start.
 *
 * AND BECAUSE IT ZEROES, IT HAS TO REFUSE ANYTHING BUT AN UNOPENED SENSOR. The header has always
 * said the scratch must not be open, and only the null pointer was checked -- so an opened,
 * `running` or `stop_unconfirmed` sensor was silently re-pointed at another address with its whole
 * ULD configuration wiped, the firmware pointer included, while `current` went on saying running.
 * The next stop() or read_once() in tof_l7_sensor would then talk to a different device through an
 * empty configuration, which is exactly the guarantee stop_unconfirmed exists to provide and this
 * was quietly stepping around.
 *
 * Checked on every call rather than once in uld_ops(), because uld_ops() only stores the pointer:
 * the lifecycle can change between binding the ops and calling them, and a check that ran at
 * binding time would describe a state that no longer holds. A caller that wants to hand a used
 * sensor to this pass has tof_l7::close() for it. */
VL53L7CX_Configuration *addressed(void *ctx, uint8_t addr_7bit)
{
    auto *const s{static_cast<tof_l7::sensor *>(ctx)};
    if (s == nullptr)
        return nullptr;
    if (s->current != tof_l7::lifecycle::empty)
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

    /* A NON-OK STATUS WITH NO TRANSPORT ERROR IS A FAILED PROBE, NOT AN EMPTY ADDRESS.
     *
     * This used to return 0 with *alive false, on the reading that a device which does not identify
     * as an L7 should be passed over. That is a real case and it is NOT this one.
     * vl53l7cx_is_alive() reports a wrong identity by setting its out-parameter to zero and
     * returning OK -- the status can only become non-OK from one of the four platform calls it
     * makes, and every one of those sets the port's sticky errno, which was checked above. So
     * reaching here means the port said every transfer succeeded and the ULD still refused, which
     * is an anomaly, and calling it "nobody there" would file it as the most ordinary observation a
     * cold boot makes. */
    if (uld_status != VL53L7CX_STATUS_OK)
        return -EIO;

    /* An ACK from something that is not an L7. Reported as a completed probe with nobody to stop,
     * on purpose: this pass stops L7 sessions and has no business issuing a five-second stop
     * sequence to a device that is not one. It is a real anomaly, and the place that acts on it is
     * enumeration, whose census exists for exactly that. */
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
