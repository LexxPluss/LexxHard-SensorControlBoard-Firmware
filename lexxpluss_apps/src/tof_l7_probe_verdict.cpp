/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "tof_l7_probe_verdict.hpp"

#if defined(ENABLE_TOF_L7_ULD)

#include <errno.h>

namespace lexxhard::tof_l7_probe {

verdict classify(const inputs &in)
{
    /* AN IMAGE FAULT OUTRANKS THE CLASSIFICATION. Reading it as "nobody there" would be the worst
     * available answer: the reason nothing was heard is that this build called the port wrongly. */
    if (in.argument_fault != 0)
        return {in.argument_fault, false};

    /* A CLEAN NACK WITH NOTHING ACKNOWLEDGED is the cold boot, and on a cold boot it is the answer
     * every address gives. Reported as a completed probe with nobody there rather than as a
     * failure, because a pass that called every cold boot a failure would say nothing about the
     * boots that matter.
     *
     * BUT ONLY WITH NOTHING ACKNOWLEDGED. The port keeps the FIRST error it sees and
     * vl53l7cx_is_alive() issues all four of its transfers regardless, ORing their statuses with no
     * early return. So a survivor whose page-select write NACKs while its two ID reads answer
     * arrives here with -ENXIO AND an answer; treating that as empty would report the address as
     * cold, stop nothing, and record no failure. Asked of the TRANSPORT rather than of the identity
     * flag, because the ULD sets that flag only when both identity bytes match -- zero does not
     * mean nothing answered. */
    if (in.port_errno == -ENXIO)
        return {in.completed != 0U ? -EIO : 0, false};

    /* -EIO or -ETIMEDOUT. The question did not reach the bus, so no answer was heard, and the
     * caller must not treat this as an empty address. */
    if (in.port_errno != 0)
        return {in.port_errno, false};

    /* A NON-OK STATUS WITH NO TRANSPORT ERROR IS A FAILED PROBE, NOT AN EMPTY ADDRESS.
     *
     * vl53l7cx_is_alive() reports a wrong identity by setting its out-parameter to zero and
     * returning OK -- its status can only become non-OK from one of the four platform calls it
     * makes, and every one of those sets the port's sticky errno, which was checked above. So
     * reaching here means the port said every transfer succeeded and the ULD still refused, which
     * is an anomaly, and calling it "nobody there" would file it as the most ordinary observation a
     * cold boot makes. */
    if (!in.uld_ok)
        return {-EIO, false};

    /* A completed probe. `present` is the identity match: an ACK from something that is not an L7
     * is a real anomaly, but it is one for the caller to act on, not one to hide behind an errno. */
    return {0, in.identity_matched};
}

}  // namespace lexxhard::tof_l7_probe

#endif  // ENABLE_TOF_L7_ULD
