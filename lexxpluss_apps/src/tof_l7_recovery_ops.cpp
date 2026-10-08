/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * See tof_l7_recovery_ops.hpp.
 */

#include "tof_l7_recovery_ops.hpp"
#include "tof_l7_probe_verdict.hpp"

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
    /* Carried across the zeroing by hand. The context is the one field here the caller owns, and
     * a probe with no bus to name would refuse every transfer with -ENODEV -- which reads exactly
     * like a cold address, the conclusion this pass exists to avoid drawing by accident. */
    vl53l7cx_port_context *const port{s->uld.platform.port};
    if (port == nullptr || port->bus == nullptr)
        return nullptr;
    memset(&s->uld, 0, sizeof(s->uld));
    s->uld.platform.port = port;
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

    vl53l7cx_port_clear_error(dev->platform.port);
    uint8_t answered{0};
    const uint8_t uld_status{vl53l7cx_is_alive(dev, &answered)};

    /* THE RULE ITSELF LIVES IN tof_l7_probe_verdict, and this adapter only gathers its inputs.
     * It was reasoned out here first and is four cases deep -- a clean NACK with nothing
     * acknowledged is the ordinary cold boot, the same NACK with transfers completed is a partial
     * exchange, any other errno never reached the bus, and a non-OK ULD status with no transport
     * error is an anomaly rather than an empty address. The Package sensor needs exactly that rule
     * and cannot reach this file, which is guarded on ENABLE_TOF_CHAIN; two copies of a
     * classification that must always agree is worse than one function both call. The reasoning
     * moved with it, so nothing here repeats it. */
    const tof_l7_probe::inputs in{
        .argument_fault = vl53l7cx_port_argument_fault(),
        .port_errno = vl53l7cx_port_error(dev->platform.port),
        .completed = vl53l7cx_port_completed_transfers(dev->platform.port),
        .uld_ok = uld_status == VL53L7CX_STATUS_OK,
        .identity_matched = answered != 0U,
    };
    const tof_l7_probe::verdict v{tof_l7_probe::classify(in)};

    *alive = v.present;
    return v.rc;
}

int stop_ranging(void *ctx, uint8_t addr_7bit)
{
    /* Re-addressed rather than relying on the probe having just run. The pure layer promises only
     * that a stop follows a successful probe, not that nothing happened in between, and an adapter
     * that depended on call order would break silently the first time that changed. */
    VL53L7CX_Configuration *const dev{addressed(ctx, addr_7bit)};
    if (dev == nullptr)
        return -EINVAL;

    vl53l7cx_port_clear_error(dev->platform.port);
    const uint8_t uld_status{vl53l7cx_stop_ranging(dev)};
    /* IMAGE FAULT FIRST, the same order is_alive() and finish() use. Easy to miss here because
     * this path already has a transport errno and a status byte to report, and neither of them
     * sees a SwapBuffer argument fault: the latch is image-wide, so one raised while another
     * context was being driven would otherwise let this stop return success. A stop that reports
     * success is the one answer the enumeration above it acts on. */
    if (const int fault{vl53l7cx_port_argument_fault()}; fault != 0)
        return fault;
    if (const int port_errno{vl53l7cx_port_error(dev->platform.port)}; port_errno != 0)
        return port_errno;

    /* The ULD collapses its own failures into one status byte, and the interesting one here has no
     * transport error behind it: stop_ranging polls the device for up to five seconds waiting for
     * the MCU to stop, and folds a give-up into that byte. -EIO for the whole class, because what
     * the caller does with it is the same -- record it and leave the verdict to enumeration. */
    return uld_status == VL53L7CX_STATUS_OK ? 0 : -EIO;
}

}  // namespace

ops uld_ops(tof_l7::sensor *scratch, vl53l7cx_port_context *port)
{
    ops o{};
    o.is_alive = is_alive;
    o.stop_ranging = stop_ranging;
    o.ctx = scratch;
    /* REFUSED WITHOUT MODIFYING THE OBJECT, which is the whole point of checking here as well as
     * in addressed(). Installing first and relying on the later lifecycle refusal would already
     * have repointed a live session's bus by the time the call was rejected: the probe would be
     * turned away, and the running sensor would be left talking to a controller nobody chose for
     * it. The call-time check stays -- the lifecycle can change between binding and calling -- but
     * it is no longer the only thing standing between a live session and a silent re-bus. */
    if (scratch != nullptr && scratch->current == tof_l7::lifecycle::empty)
        scratch->uld.platform.port = port;
    return o;
}

}  // namespace lexxhard::tof_l7_recovery

#endif
