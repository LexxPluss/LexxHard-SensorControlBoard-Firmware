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
        return {in.argument_fault, false, false};

    /* Only a clean NACK with no completed transfer means nobody answered.
     * The ULD attempts all four transfers and retains the first errno; a later
     * successful transfer makes -ENXIO a partial exchange, not an empty address.
     * Use the transfer count: the identity flag is zero for wrong IDs too.
     */
    if (in.port_errno == -ENXIO)
        return {in.completed != 0U ? -EIO : 0, false, false};

    /* -EIO or -ETIMEDOUT. The question did not reach the bus, so no answer was heard, and the
     * caller must not treat this as an empty address. */
    if (in.port_errno != 0)
        return {in.port_errno, false, false};

    /* Non-OK ULD status without a port error is a failed probe, not an empty
     * address. is_alive() returns OK for a wrong identity; a transport failure
     * should already have supplied the sticky errno checked above.
     */
    if (!in.uld_ok)
        return {-EIO, false, false};

    /* A COMPLETED PROBE, AND THE TWO OUTCOMES IT CAN HAVE ARE REPORTED SEPARATELY.
     *
     * Every transfer succeeded, so something on the bus acknowledged. Whether it is an L7 is the
     * identity match, and a mismatch is reported in its own field rather than as "nobody there":
     * the two need opposite handling, and only the caller knows which it can afford. See
     * verdict::answered_without_matching_identity. */
    return {0, in.identity_matched, !in.identity_matched};
}

}  // namespace lexxhard::tof_l7_probe

#endif  // ENABLE_TOF_L7_ULD
