/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The same reply obligation as the DFU path, in the other request/response service on this bus. The
 * actuator service answers a host request exactly once, so the hold-and-retry rule and the reasons
 * for it are the ones written down in test_dfu.cpp; what is specific here is that the reply is
 * identified by its counter, which makes the ordering assertions read directly.
 *
 * This suite also covers the receive budget on a path where falling behind costs something: the
 * requests are the host's, and the budget is deliberately larger than the transmit one because the
 * work per frame is local and cheap.
 */

#include "poll_probe.hpp"

#include "zcan_actuator_service.hpp"
#include "zcan_poll_budget.hpp"

namespace lexxhard::actuator_service_controller {
k_msgq msgq_request, msgq_response;
}

namespace {

char alignas(4) requests[16 * sizeof (lexxhard::actuator_service_controller::msg_request)];
char alignas(4) replies[8 * sizeof (lexxhard::actuator_service_controller::msg_response)];

}  // namespace

ZTEST_SUITE(zcan_poll_budget_actuator_service, nullptr, nullptr, nullptr, nullptr, nullptr);

ZTEST(zcan_poll_budget_actuator_service, test_refused_reply_is_held_while_requests_keep_flowing)
{
    using namespace lexxhard;

    probe::reset();
    k_msgq_init(&actuator_service_controller::msgq_request, requests,
                sizeof (actuator_service_controller::msg_request), 16);
    k_msgq_init(&actuator_service_controller::msgq_response, replies,
                sizeof (actuator_service_controller::msg_response), 8);
    k_msgq_purge(&zcan_actuator_service::msgq_can_actuator_service_request);

    actuator_service_controller::msg_response first{}, second{};
    first.counter = 11;
    second.counter = 12;
    zassert_equal(k_msgq_put(&actuator_service_controller::msgq_response, &first, K_NO_WAIT), 0);
    zassert_equal(k_msgq_put(&actuator_service_controller::msgq_response, &second, K_NO_WAIT), 0);

    zcan_actuator_service::zcan_actuator_service poller;

    poller.poll();
    zassert_equal(probe::frames[0].data[7], 11);

    /* A full receive budget arrives while the reply is still held. */
    for (int i{0}; i < zcan_poll_budget::kRxPerPass; ++i) {
        can_frame request{};
        request.id = CAN_ID_ACTUATOR_SERVICE_REQUEST;
        request.dlc = 8;
        zassert_equal(k_msgq_put(&zcan_actuator_service::msgq_can_actuator_service_request,
                                 &request, K_NO_WAIT), 0);
    }
    poller.poll();
    zassert_equal(probe::frames[1].data[7], 11, "the held reply is retried, not re-read");
    zassert_equal(k_msgq_num_used_get(&actuator_service_controller::msgq_request),
                  zcan_poll_budget::kRxPerPass);
    zassert_equal(k_msgq_num_used_get(&actuator_service_controller::msgq_response), 1);

    probe::rc = 0;
    poller.poll();
    zassert_equal(probe::frames[2].data[7], 11);
    poller.poll();
    zassert_equal(probe::frames[3].data[7], 12);

    poller.poll();
    zassert_equal(probe::sends, 4, "an accepted reply must not be sent again");
}
