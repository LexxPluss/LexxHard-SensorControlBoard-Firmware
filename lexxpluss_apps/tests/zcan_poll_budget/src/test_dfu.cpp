/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The reply that must not be dropped. Shortening the mailbox wait from 100 ms to 1 ms makes refusal
 * an ordinary outcome rather than an exceptional one, and for periodic telemetry that is free: the
 * next pass carries fresher data. For a reply it is not. The updater produces one response per
 * command and never repeats it, so a response taken off the queue and then refused by a busy
 * mailbox is a response the host waits for forever -- a DFU that looks hung, with nothing in the
 * log to say why. That is the failure this path was changed to avoid, and note that it was already
 * reachable at 100 ms; the shorter wait only makes it likely.
 *
 * What is asserted, in order: a refused reply stays, it is not replaced by the next reply queued
 * behind it, holding it does not stop the request drain on the same pass, the hold ends when the
 * send is accepted, the replies then come out in the order the updater produced them, and an
 * accepted reply is not sent a second time.
 */

#include "poll_probe.hpp"

#include "zcan_dfu.hpp"

namespace lexxhard::firmware_updater {
k_msgq msgq_command, msgq_response;
}

namespace {

char alignas(4) cmds[8 * sizeof (lexxhard::firmware_updater::command_packet)];
char alignas(4) replies[8 * sizeof (lexxhard::firmware_updater::response_packet)];

}  // namespace

ZTEST_SUITE(zcan_poll_budget_dfu, nullptr, nullptr, nullptr, nullptr, nullptr);

ZTEST(zcan_poll_budget_dfu, test_refused_reply_is_held_and_retried_in_order)
{
    using namespace lexxhard;

    probe::reset();
    k_msgq_init(&firmware_updater::msgq_command, cmds,
                sizeof (firmware_updater::command_packet), 8);
    k_msgq_init(&firmware_updater::msgq_response, replies,
                sizeof (firmware_updater::response_packet), 8);
    k_msgq_purge(&zcan_dfu::msgq_can_dfu);

    firmware_updater::response_packet first{{1, 2}, 1, 0}, second{{3, 4}, 2, 0};
    zassert_equal(k_msgq_put(&firmware_updater::msgq_response, &first, K_NO_WAIT), 0);
    zassert_equal(k_msgq_put(&firmware_updater::msgq_response, &second, K_NO_WAIT), 0);

    zcan_dfu::zcan_dfu poller;

    /* Refused. The reply is now held, and the one behind it is untouched: a held reply must not be
     * overwritten by the next, or the host receives an answer to a command it did not send. */
    poller.poll();
    zassert_equal(probe::sends, 1);
    zassert_equal(k_msgq_num_used_get(&firmware_updater::msgq_response), 1);

    /* Still refused, and a command arrives meanwhile. Holding a reply must not cost the board its
     * request handling, and the retry must be the same bytes rather than a fresh read of the
     * queue. */
    can_frame request{};
    request.id = CAN_ID_DFU_DATA;
    request.dlc = 8;
    zassert_equal(k_msgq_put(&zcan_dfu::msgq_can_dfu, &request, K_NO_WAIT), 0);
    poller.poll();
    zassert_equal(k_msgq_num_used_get(&firmware_updater::msgq_command), 1);
    zassert_mem_equal(probe::frames[0].data, probe::frames[1].data, sizeof first);

    /* Accepted. The hold ends, and the reply behind it follows on the next pass in the order the
     * updater produced them. */
    probe::rc = 0;
    poller.poll();
    zassert_mem_equal(probe::frames[2].data, &first, sizeof first);
    poller.poll();
    zassert_mem_equal(probe::frames[3].data, &second, sizeof second);

    /* Nothing is left held, so a pass with no reply to send sends nothing: the retry must not turn
     * a one-shot answer into a repeating one. */
    poller.poll();
    zassert_equal(probe::sends, 4, "an accepted reply must not be sent again");
}
