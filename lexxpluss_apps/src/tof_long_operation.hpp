/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The bookkeeping behind a declared long operation: how many are live, and when the earliest of
 * them began.
 *
 * WHY THIS IS ONE TYPE AND NOT TWO ATOMICS. The watchdog reads the count and the start time
 * together and judges one against the other, so they are a single value. Held as two atomics they
 * can be read from either side of the same transition, and -- the defect this type replaces -- a
 * begin written as "read the count, stamp if it was zero, then increment" lets two callers
 * interleave so that the LATER stamp is the one that survives:
 *
 *     A reads depth 0                      (A will stamp)
 *     B reads depth 0, stamps t_B, depth 1
 *     A stamps t_A (t_A >= t_B), depth 2
 *
 * B's declaration is then judged from t_A, so its deadline moved because somebody else arrived. A
 * cap that another caller can defer is not a cap, and the same two-step shape lets two ends both
 * see depth 1 and both decrement past zero.
 *
 * WHY IT DOES NO LOCKING ITSELF. The owner supplies mutual exclusion -- tof_watchdog_feeder holds a
 * spinlock across every call, including the read -- which is what leaves this file a pure state
 * machine that a host test can drive through orderings the hardware would be lucky to produce once.
 * The lock is what rules out the interleavings; the tests cover the orderings the lock still
 * permits.
 *
 * THE DEADLINE BELONGS TO THE EARLIEST LIVE DECLARATION. A nested begin must not restamp, so a
 * chain of overlapping declarations cannot buy more than one cap's worth of silence between them.
 * The clock is read by the caller under the same lock, so `began_ms` is the moment the first live
 * declaration was REGISTERED rather than the moment some caller happened to read a clock.
 */

#pragma once

#include <stdint.h>

namespace lexxhard::tof_long_operation {

/* What the watchdog is given. `began_ms` is meaningful only while `active`; a reader that uses it
 * otherwise is reading the previous declaration. */
struct snapshot {
    bool active{false};
    uint32_t began_ms{0};
};

class state {
public:
    /* `now_ms` is read only on the transition out of zero. Depth is 32-bit and a begin that loses
     * its end leaks one count, which the watchdog's own cap on the declaration is there to survive;
     * wrapping it would take four billion unmatched begins. */
    void begin(uint32_t now_ms)
    {
        if (depth_ == 0)
            began_ms_ = now_ms;
        ++depth_;
    }

    /* An end with no matching begin is ignored rather than allowed to take the depth below zero,
     * because a negative depth reads as a declaration nobody can ever close -- the board would be
     * declared busy until the cap fired, every time, for the rest of the boot. */
    void end()
    {
        if (depth_ != 0)
            --depth_;
    }

    snapshot read() const { return snapshot{depth_ != 0, began_ms_}; }

    /* For the suite, and for a future shell line. Not part of what the watchdog judges. */
    uint32_t depth() const { return depth_; }

private:
    uint32_t depth_{0};
    uint32_t began_ms_{0};
};

}  // namespace lexxhard::tof_long_operation

// vim: set expandtab shiftwidth=4:
