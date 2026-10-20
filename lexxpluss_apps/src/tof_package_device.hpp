/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

/* One owner for the I2C4 port, ULD scratch, lifecycle hooks and binding.
 * Bench and auto builds are mutually exclusive, allowing serialized access
 * without locks. Callers use this layer rather than duplicating device state.
 * Retries, backoff, entry selection and rate belong to their policies.
 */

#include <stdint.h>

#if defined(ENABLE_TOF_PACKAGE_BENCH) || defined(ENABLE_TOF_PACKAGE_AUTO)

#if !defined(ENABLE_TOF_L7_ULD)
#error "the Package ToF device layer requires ENABLE_TOF_L7_ULD"
#endif

#include "tof_l7_sensor.hpp"
#include "tof_package_l7.hpp"
#include "tof_package_stream.hpp"
#include "tof_seq_boot_local.hpp"

namespace lexxhard::tof_package_device {

/* WHY A BIND WAS REFUSED, as its own answer rather than as an errno to be guessed at. Two of these
 * are -EPERM and two are the layer's own return, so the errno alone cannot tell a caller which
 * message to print or which of them is recoverable. */
enum class bind_refusal : uint8_t {
    /* Nothing was refused. */
    none,
    /* The current lifecycle still owns a session: it may be ranging, or owe a stop. NOTHING was
     * changed, deliberately -- that record is the only evidence the device may still be measuring,
     * and on this sensor the way back from losing it is a machine power cycle. */
    session_owned,
    /* i2c4 is not ready. Nothing was bound. */
    bus_not_ready,
    /* The sensor object refused to close, from its own state rather than from bookkeeping. */
    close_refused,
    /* The lifecycle refused the hooks or the configuration. */
    lifecycle_refused,
};

const char *bind_refusal_name(bind_refusal r);

/* Binds the lifecycle to i2c4 @0x29 at `frequency_hz`, opening nothing. 0, or a negative errno
 * with `*why` saying which refusal it was. `why` may be null.
 *
 * The frequency is the caller's, with no default here: the sustainable rate for this geometry has
 * not been accepted, and a default in this file would quietly become that answer. */
int bind(uint8_t frequency_hz, bind_refusal *why);

/* Whether a bind has succeeded. False means every call below acts on an unbound lifecycle, which
 * refuses on its own. */
bool bound();

/* The lifecycle, the last grid it read, the publisher and the boot-local sequence source. Handed
 * out by reference because there is exactly one of each and the owner above decides what to do
 * with them; this module adds no rule about when they may be used. */
tof_package::lifecycle &lifecycle();
const tof_l7::sample &last_sample();
tof_package_stream::publisher &publisher();
tof_seq_boot_local::source &boot_sequence();

/* Apply the round boundary, then call bring_up() or take_over_unknown_state().
 * Clear step records only when the lifecycle accepts a new round, before its
 * first step; preserve evidence on refusal. Keep the separate quiesce record.
 * All callers use this entry so the boundary ordering has one owner.
 */
int begin_round(bool by_takeover);

/* The adapter the publisher's injected sequence source needs. It exists only because a member
 * function is not a C function pointer; it adds no rule of its own. */
int issue_boot_local(void *ctx, uint32_t *out);

}  // namespace lexxhard::tof_package_device

#endif  // ENABLE_TOF_PACKAGE_BENCH || ENABLE_TOF_PACKAGE_AUTO
