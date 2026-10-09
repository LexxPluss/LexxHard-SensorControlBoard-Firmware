/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The consequence that is worse than the watchdog one. zcan_board::poll() drains telemetry towards
 * the host first and receives the host's control frames second. While the transmit drain could run
 * forever, a host that went away stopped the board from consuming that host's control frames --
 * including after the host came back, because the first drain never yielded. No watchdog takes part
 * in that, and no reset repairs it.
 *
 * So this suite asserts ordering, not just counts: with the transmit side refusing every send and
 * its queue refilled from inside send, the receive drain behind it must still run on the same pass,
 * and must take its own, larger budget.
 */

#include "poll_probe.hpp"

#include "zcan_board.hpp"
#include "zcan_poll_budget.hpp"

namespace lexxhard::can_controller {
k_msgq msgq_board, msgq_control;
}

namespace {

char alignas(4) tx[8 * sizeof (lexxhard::can_controller::msg_board)];
char alignas(4) rx[16 * sizeof (lexxhard::can_controller::msg_control)];

void refill()
{
    lexxhard::can_controller::msg_board m{};
    k_msgq_put(&lexxhard::can_controller::msgq_board, &m, K_NO_WAIT);
}

}  // namespace

ZTEST_SUITE(zcan_poll_budget_board, nullptr, nullptr, nullptr, nullptr, nullptr);

ZTEST(zcan_poll_budget_board, test_refused_transmit_still_lets_receive_run_on_the_same_pass)
{
    probe::reset();
    k_msgq_init(&lexxhard::can_controller::msgq_board, tx,
                sizeof (lexxhard::can_controller::msg_board), 8);
    k_msgq_init(&lexxhard::can_controller::msgq_control, rx,
                sizeof (lexxhard::can_controller::msg_control), 16);
    k_msgq_purge(&lexxhard::zcan_board::msgq_can_ros2board);

    /* One more waiting frame than the receive budget, so the budget is observable rather than
     * merely satisfied. */
    for (int i{0}; i < lexxhard::zcan_poll_budget::kRxPerPass + 1; ++i) {
        can_frame f{};
        f.id = CAN_ID_BOARD_RX;
        f.dlc = 5;
        f.data[0] = 1;
        f.data[3] = 1;
        zassert_equal(k_msgq_put(&lexxhard::zcan_board::msgq_can_ros2board, &f, K_NO_WAIT), 0);
    }

    refill();
    probe::refill = refill;

    lexxhard::zcan_board::zcan_board poller;
    poller.poll();
    probe::refill = nullptr;

    zassert_equal(probe::sends, lexxhard::zcan_poll_budget::kTxPerPass);
    zassert_equal(k_msgq_num_used_get(&lexxhard::can_controller::msgq_board), 1,
                  "refused telemetry is dropped, not retried");

    /* The point of the suite: the control frames went through while the transmit side was failing. */
    zassert_equal(k_msgq_num_used_get(&lexxhard::can_controller::msgq_control),
                  lexxhard::zcan_poll_budget::kRxPerPass);
    zassert_equal(k_msgq_num_used_get(&lexxhard::zcan_board::msgq_can_ros2board), 1);
}
