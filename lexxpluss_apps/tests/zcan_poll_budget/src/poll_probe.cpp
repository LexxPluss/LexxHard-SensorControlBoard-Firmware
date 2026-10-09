/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "poll_probe.hpp"

#include "zcan_bounded_send.hpp"
#include "zcan_poll_budget.hpp"

namespace probe {

int rc{-EAGAIN};
int sends{0};
can_frame frames[32]{};
void (*refill)(){nullptr};

void reset()
{
    rc = -EAGAIN;
    sends = 0;
    refill = nullptr;
}

}  // namespace probe

namespace lexxhard::zcan_bounded_send {

/* Replaces the production wrapper for these suites, and asserts two things on the way through.
 *
 * The timeout is checked because a send site left on the old K_MSEC(100) would still pass every
 * count-based assertion below while costing a hundred times as much per refusal -- the regression
 * would be invisible to a host test and visible only on a board with its host switched off.
 *
 * The frame-count ceiling is the termination check. A poller that ignores its budget under a
 * refilling producer does not fail an assertion afterwards; it never returns, and the suite would
 * hang until twister killed it with no indication of which loop was at fault. Failing inside send
 * at a bound no healthy pass can reach turns that hang into a named failure. */
int send(const device *, const can_frame *frame, k_timeout_t timeout)
{
    zassert_true(K_TIMEOUT_EQ(timeout, zcan_poll_budget::kMailboxWait),
                 "send site still uses its own timeout instead of kMailboxWait");
    zassert_true(probe::sends < 32, "poll() did not return under a continuously refilled queue");
    probe::frames[probe::sends++] = *frame;
    if (probe::refill)
        probe::refill();
    return probe::rc;
}

}  // namespace lexxhard::zcan_bounded_send
