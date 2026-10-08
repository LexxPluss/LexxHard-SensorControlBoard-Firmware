/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * What an L7 identity probe actually told us.
 *
 * WHY THIS IS ITS OWN FUNCTION. The rule was reasoned out once, inside the chain's recovery
 * adapter, and it is four distinct cases deep: a clean NACK with nothing acknowledged is the
 * ordinary cold boot, the same NACK with transfers completed is a partial exchange, any other
 * transport errno never reached the bus, and a non-OK ULD status with no transport error at all is
 * an anomaly rather than an empty address. Getting any of them wrong fails in the same direction --
 * something is reported as "nobody there" -- which is the most ordinary observation a cold boot
 * makes and therefore the one that hides.
 *
 * The Package sensor needs the identical rule and cannot reach the chain's adapter, which is
 * guarded on ENABLE_TOF_CHAIN. Two copies of a classification that must always agree is a worse
 * answer than one function both call, so the reasoning lives here and both adapters are thin.
 *
 * Pure, and free of Zephyr and ULD headers: the ULD's status and alive flag arrive as booleans so
 * that the decision can be tested exhaustively without a bus or a vendor object.
 */

#pragma once

#if defined(ENABLE_TOF_L7_ULD)

#include <stdint.h>

namespace lexxhard::tof_l7_probe {

struct inputs {
    /* Image-wide port argument fault. Outranks everything below: it means this port or the ULD was
     * handed a buffer it cannot process, so nothing about what the bus did is worth interpreting. */
    int argument_fault{0};
    /* The port's FIRST transport errno for this probe. First, not last, because a later failure is
     * usually a consequence of the first. */
    int port_errno{0};
    /* COMPLETED chunk transactions, which is a different question from whether there was an error
     * and cannot be answered from the errno alone. */
    uint32_t completed{0};
    /* The ULD's own status byte was its success value. */
    bool uld_ok{false};
    /* The ULD's alive flag: set only when BOTH identity bytes matched. */
    bool identity_matched{false};
};

struct verdict {
    /* 0 means the probe itself completed, whatever it found. Non-zero means it did not, and
     * `present` is then meaningless -- the caller must not read it as an empty address. */
    int rc{0};
    bool present{false};
};

verdict classify(const inputs &in);

}  // namespace lexxhard::tof_l7_probe

#endif  // ENABLE_TOF_L7_ULD
