/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The loop that did not terminate. zcan_imu's transmit drain is the plainest instance of the
 * pattern the budget fixes: one queue, one send site, a producer at 40 Hz and a drain that used to
 * run `while (k_msgq_get(..., K_NO_WAIT) == 0)`. With the host answering the queue empties long
 * before the producer catches up, so the old form terminated and the defect never showed. With the
 * host gone every send waits out the mailbox timeout and returns -EAGAIN, the drain rate falls
 * below the producer, and the condition for leaving the loop stops being reachable.
 *
 * The producer here refills from inside send, which is the worst arrangement rather than a
 * representative one: the queue is non-empty at every single loop test. Nothing but a budget gets
 * out of that.
 */

#include "poll_probe.hpp"

#include "zcan_imu.hpp"
#include "zcan_poll_budget.hpp"

/* The acquisition side of this queue is not built here; the queue is, and that is what is drained. */
namespace lexxhard::imu_controller {
k_msgq msgq;
}

namespace {

char alignas(4) storage[8 * sizeof (lexxhard::imu_controller::msg)];

void refill()
{
    lexxhard::imu_controller::msg m{};
    k_msgq_put(&lexxhard::imu_controller::msgq, &m, K_NO_WAIT);
}

}  // namespace

ZTEST_SUITE(zcan_poll_budget_imu, nullptr, nullptr, nullptr, nullptr, nullptr);

ZTEST(zcan_poll_budget_imu, test_refilling_producer_does_not_hold_the_pass)
{
    probe::reset();
    k_msgq_init(&lexxhard::imu_controller::msgq, storage, sizeof (lexxhard::imu_controller::msg), 8);

    refill();
    probe::refill = refill;

    lexxhard::zcan_imu::zcan_imu poller;
    poller.poll();
    probe::refill = nullptr;

    /* One message becomes two frames, acceleration and angular rate, so the budget shows up here
     * as twice itself. The assertion that matters is less the number than the fact that poll()
     * returned at all, which the frame ceiling inside send is what enforces. */
    zassert_equal(probe::sends, 2 * lexxhard::zcan_poll_budget::kTxPerPass);

    /* And it returned with work left over rather than by exhausting the queue -- the backlog is
     * carried to the next pass instead of being finished inside this one. */
    zassert_true(k_msgq_num_used_get(&lexxhard::imu_controller::msgq) > 0);
}
