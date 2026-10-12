/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "tof_l7_quiesce.hpp"

#if defined(ENABLE_TOF_L7_ULD)

#include <errno.h>
#include <stdint.h>

namespace lexxhard::tof_l7_quiesce {

namespace {

record stored{};
uint32_t recorded_so_far{0};

}  // namespace

verdict classify(const inputs &in)
{
    /* AN IMAGE FAULT OUTRANKS EVERYTHING, as it does in the probe classification and for the same
     * reason: the port was handed something it cannot process, so what the bus or the device did is
     * not what went wrong. */
    if (in.argument_fault != 0)
        return {in.argument_fault, layer::image_fault};

    /* THE TRANSPORT ANSWERS FOR ITSELF, BEFORE THE STATUS BYTE IS READ AT ALL. See the header: the
     * status byte would mostly agree on today's tree, and "mostly agree" is not what a failure
     * report can be built on. Whatever the byte says, a port that recorded an errno means the
     * exchange did not complete and the device's answer was never fully heard. */
    if (in.port_errno != 0)
        return {in.port_errno, layer::transport};

    if (in.uld_status == kUldOk)
        return {0, layer::ok};

    /* ONLY HERE, with no argument fault and no transport errno, does the timeout code mean the
     * thing it is named for: every transfer succeeded and the MCU stop was still never confirmed. */
    if (in.uld_status == kUldTimeout)
        return {-EIO, layer::handshake};

    /* Any other non-OK status: kept as the raw byte in the record, deliberately not folded into
     * `handshake`. The two have different next steps and the -5 that prompted all this was exactly
     * the folding of several answers into one. */
    return {-EIO, layer::device_status};
}

uint32_t span_us(int64_t started_us, int64_t ended_us)
{
    if (ended_us <= started_us)
        return 0U;
    const int64_t span{ended_us - started_us};
    if (span > static_cast<int64_t>(UINT32_MAX))
        return UINT32_MAX;
    return static_cast<uint32_t>(span);
}

void remember(const inputs &in, uint32_t elapsed_us)
{
    const verdict v{classify(in)};
    ++recorded_so_far;
    stored.taken = true;
    stored.sequence = recorded_so_far;
    stored.raw = in;
    stored.which = v.which;
    stored.rc = v.rc;
    stored.elapsed_us = elapsed_us;
}

const record &last()
{
    return stored;
}

void forget_all()
{
    stored = record{};
    recorded_so_far = 0;
}

}  // namespace lexxhard::tof_l7_quiesce

#endif  // ENABLE_TOF_L7_ULD
