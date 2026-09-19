/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The binding, over a REAL CAN controller.
 *
 * native_sim's loopback driver is a real Zephyr CAN device: filters are installed through the real
 * API, frames go through the real send path, and what comes back comes back through the real
 * receive callback. So this suite can ask the questions that matter about wiring -- is a filter
 * installed at all, for which identifier, is it installed when there is no session, is it NOT
 * installed when the configuration is refused, and does the board ever transmit on anything but the
 * status identifier.
 *
 * WHAT THE TEST SUPPLIES INSTEAD OF PRODUCTION: the proof, the acquisition start and the entropy
 * draw -- all three are hardware, and the binding's job with them is to pass them along. The CAN
 * path is not supplied: it is the thing under test.
 *
 * THE IDENTIFIERS ARE THE ALLOCATED ONES HERE, unlike the runtime suite, and for the opposite
 * reason: this is the file that is supposed to know them, so the test's job is to check that what
 * goes on the bus matches what the contract says.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#include <string.h>

#include "tof_cliff_contract.h"
#include "tof_commission_bind.hpp"
#include "tof_commission_entropy.hpp"
#include "tof_commission_runtime.hpp"
#include "tof_commission_wire.hpp"
#include "tof_commissioning.hpp"

namespace bind = lexxhard::tof_commission_bind;
namespace rt = lexxhard::tof_commission_runtime;
namespace wire = lexxhard::tof_commission_wire;
namespace ctr = tof_cliff_contract;

namespace {

/* What the three hardware seams do in this suite. */
bool entropy_ok_{true};
uint32_t token_{0x0BADF00DU};
int proves_{0};
int starts_{0};
bool permitted_{true};
int permitted_calls_{0};

bool test_permitted(void *)
{
    ++permitted_calls_;
    return permitted_;
}

const struct device *can_dev()
{
    return DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
}

/* The observer: a second filter on the STATUS identifier, so the test sees exactly what a host would
 * see. Installed once for the suite. */
CAN_MSGQ_DEFINE(status_msgq, 8);
int status_filter_{-1};

/* And one on the request identifier, to prove the board never transmits on it. */
CAN_MSGQ_DEFINE(request_msgq, 8);
int request_filter_{-1};

void drain()
{
    struct can_frame f {};
    while (k_msgq_get(&status_msgq, &f, K_NO_WAIT) == 0) {
    }
    while (k_msgq_get(&request_msgq, &f, K_NO_WAIT) == 0) {
    }
}

int send_request(uint32_t id, uint8_t seq, uint8_t epoch, uint32_t token)
{
    wire::request r{};
    r.raw_op = static_cast<uint8_t>(wire::opcode::prove_and_start);
    r.seq = seq;
    r.wire_epoch = epoch;
    r.session_token = token;

    struct can_frame f {};
    f.id = id;
    f.dlc = wire::kFrameLen;
    wire::encode_request(r, f.data);
    return can_send(can_dev(), &f, K_MSEC(100), nullptr, nullptr);
}

/* The board answers from the receive callback, and the loopback driver delivers on its own work
 * queue, so a moment is needed before the answer is in the queue. */
bool next_status(struct can_frame &out, k_timeout_t wait = K_MSEC(200))
{
    return k_msgq_get(&status_msgq, &out, wait) == 0;
}

bind::config configured()
{
    bind::config c{};
    c.profile_enabled = true;
    c.max_proof_attempts = 3;
    c.max_start_attempts = 3;
    c.announce_period_ms = 60000;  /* inert: this suite drives what it wants to observe */
    c.poll_ms = 60000;
    c.enumeration_permitted = test_permitted;
    return c;
}

void teardown(const bind::result &r)
{
    if (r.filter_installed && r.filter_id >= 0)
        can_remove_rx_filter(can_dev(), r.filter_id);
}

} // namespace

/* --- the three hardware seams, defined here instead of their production versions --- */

namespace lexxhard::tof_commission_entropy {

int draw_token(void *, uint32_t *out)
{
    if (!entropy_ok_)
        return -ENODEV;
    *out = token_;
    return 0;
}

bool available()
{
    return entropy_ok_;
}

} // namespace lexxhard::tof_commission_entropy

namespace lexxhard::tof_commissioning {

outcome prove(uint32_t)
{
    ++proves_;
    outcome r{};
    r.failed_at = stage::none;
    return r;
}

} // namespace lexxhard::tof_commissioning

namespace lexxhard::tof_cliff_runtime {

int start_acquisition()
{
    ++starts_;
    return 0;
}

} // namespace lexxhard::tof_cliff_runtime

/* ------------------------------------------------------------------------------------------- */

static void *suite_setup(void)
{
    const struct device *dev{can_dev()};
    zassert_true(device_is_ready(dev), "the loopback controller is ready");
    (void)can_stop(dev);
    zassert_equal(can_set_mode(dev, CAN_MODE_LOOPBACK), 0, "loopback mode");
    zassert_equal(can_start(dev), 0, "the bus is running");

    const struct can_filter status_filter {
        .id = ctr::kCommissionStatusId, .mask = CAN_STD_ID_MASK, .flags = 0,
    };
    status_filter_ = can_add_rx_filter_msgq(dev, &status_msgq, &status_filter);
    zassert_true(status_filter_ >= 0, "the observer is listening on the status identifier");

    const struct can_filter request_filter {
        .id = ctr::kCommissionRequestId, .mask = CAN_STD_ID_MASK, .flags = 0,
    };
    request_filter_ = can_add_rx_filter_msgq(dev, &request_msgq, &request_filter);
    zassert_true(request_filter_ >= 0, "and on the request identifier");
    return nullptr;
}

static void before(void *)
{
    entropy_ok_ = true;
    proves_ = 0;
    starts_ = 0;
    permitted_ = true;
    permitted_calls_ = 0;
    drain();
}

ZTEST_SUITE(tof_commission_bind, NULL, suite_setup, before, NULL, NULL);

ZTEST(tof_commission_bind, test_a_session_means_a_filter_and_a_worker)
{
    const bind::result r{bind::start(can_dev(), configured())};
    zassert_equal(r.rc, 0, "a session was drawn");
    zassert_true(r.state == bind::outcome::running, "so the board is commissionable");
    zassert_true(r.filter_installed, "the request filter is installed");
    zassert_true(r.worker_started || rt::running(), "and a worker is up");

    /* A REAL FRAME, through the real driver, on the allocated identifier. */
    zassert_equal(send_request(ctr::kCommissionRequestId, 1, 7, token_), 0, "the request goes out");

    struct can_frame answer {};
    zassert_true(next_status(answer), "the board answered");
    zassert_equal(answer.id, ctr::kCommissionStatusId, "on 0x219, the status identifier");
    zassert_equal(answer.dlc, wire::kFrameLen, "eight bytes");

    wire::transaction_status t{};
    zassert_equal(wire::decode_transaction_status(answer.data, answer.dlc, t),
                  wire::decode_error::none, "and it decodes");
    zassert_true(t.ph == wire::phase::accepted, "accepted: the receive path does no work");
    zassert_equal(t.seq, 1, "for the request that asked");

    /* NOTHING OF OURS EVER APPEARS ON THE REQUEST IDENTIFIER. The observer on 0x218 sees the
     * test's own frame and must see nothing else. */
    struct can_frame echoed {};
    zassert_equal(k_msgq_get(&request_msgq, &echoed, K_MSEC(50)), 0, "the request itself");
    zassert_equal(k_msgq_get(&request_msgq, &echoed, K_MSEC(200)), -EAGAIN,
                  "and the board transmitted nothing on it");

    teardown(r);
}

ZTEST(tof_commission_bind, test_no_session_still_installs_the_filter_and_answers)
{
    entropy_ok_ = false;
    const bind::result r{bind::start(can_dev(), configured())};
    zassert_equal(r.rc, -ENODEV, "no session could be drawn");
    zassert_true(r.state == bind::outcome::answering_only, "answering, not commissioning");
    zassert_true(r.filter_installed,
                 "THE FILTER IS STILL IN: a host holding a durable pending request needs a "
                 "terminal answer, and silence is not one");
    zassert_false(r.worker_started, "and no worker, because there is nothing for it to do");

    zassert_equal(send_request(ctr::kCommissionRequestId, 4, 9, 0x12345678U), 0, "");
    struct can_frame answer {};
    zassert_true(next_status(answer), "answered");
    zassert_equal(answer.id, ctr::kCommissionStatusId, "on the status identifier");

    wire::transaction_status t{};
    zassert_equal(wire::decode_transaction_status(answer.data, answer.dlc, t),
                  wire::decode_error::none, "");
    zassert_true(t.res == wire::result::no_session, "with no_session");
    zassert_equal(proves_, 0, "and nothing was proved");

    /* No announcement either: there is no session to announce. */
    zassert_equal(k_msgq_get(&status_msgq, &answer, K_MSEC(200)), -EAGAIN, "nothing else was sent");

    teardown(r);
}

ZTEST(tof_commission_bind, test_a_refused_configuration_installs_nothing)
{
    bind::config c{configured()};
    c.enumeration_permitted = nullptr;  /* the stationary condition is not defaulted */

    const bind::result r{bind::start(can_dev(), c)};
    zassert_equal(r.rc, -EINVAL, "refused");
    zassert_true(r.state == bind::outcome::refused, "");
    zassert_false(r.filter_installed,
                  "NOTHING is installed: a filter with no runtime behind it takes an identifier off "
                  "the bus and answers nothing on it");
    zassert_false(r.worker_started, "");

    zassert_equal(send_request(ctr::kCommissionRequestId, 5, 9, token_), 0, "a request is sent");
    struct can_frame answer {};
    zassert_equal(k_msgq_get(&status_msgq, &answer, K_MSEC(200)), -EAGAIN, "and nothing answers it");
    zassert_equal(proves_, 0, "nothing ran");
}

ZTEST(tof_commission_bind, test_a_device_that_is_not_ready_installs_nothing)
{
    const bind::result r{bind::start(nullptr, configured())};
    zassert_equal(r.rc, -ENODEV, "refused");
    zassert_true(r.state == bind::outcome::refused, "");
    zassert_false(r.filter_installed, "");
}

ZTEST(tof_commission_bind, test_only_the_request_identifier_is_received)
{
    const bind::result r{bind::start(can_dev(), configured())};
    zassert_true(r.state == bind::outcome::running, "");
    const uint32_t before_in{rt::stats().frames_in};

    /* A well-formed request on the STATUS identifier, and on a neighbour. Either would be answered
     * by a board that filtered on the wrong value, and the second is what an adjacent allocation
     * looks like. */
    zassert_equal(send_request(ctr::kCommissionStatusId, 9, 9, token_), 0, "");
    zassert_equal(send_request(ctr::kMeasId, 9, 9, token_), 0, "");
    k_msleep(100);

    zassert_equal(rt::stats().frames_in, before_in, "neither reached the runtime");
    zassert_equal(proves_, 0, "and nothing ran");

    /* The one on the status identifier is in the observer's queue -- the test's own frame, not an
     * answer to it. */
    struct can_frame echoed {};
    zassert_equal(k_msgq_get(&status_msgq, &echoed, K_MSEC(50)), 0, "the test's own frame");
    zassert_equal(k_msgq_get(&status_msgq, &echoed, K_MSEC(200)), -EAGAIN, "and no answer");

    teardown(r);
}
