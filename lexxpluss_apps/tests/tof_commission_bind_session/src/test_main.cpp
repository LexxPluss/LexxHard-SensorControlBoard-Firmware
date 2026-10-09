/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The binding WITH A SESSION, over a REAL CAN controller, as ONE ordered scenario.
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
 *
 * WHY THIS IS A SEPARATE BINARY FROM tof_commission_bind, AND ONE CASE INSIDE IT.
 *
 * bind::start() calls rt::init() every time, and tof_commission_runtime.hpp states ONE INIT PER
 * BOOT as a precondition: init() reconfigures the session layer, drawing a new token and resetting
 * the budgets, and the worker is not stoppable because start() refuses a second thread precisely
 * so the first one can live for the process. Nothing enforces it -- the header says the suites
 * respect it by isolating the case that cannot -- and this suite did not: three of its cases drew a
 * session, so every one after the first re-initialised the session layer underneath a live worker.
 * It passed on the poll period and the order ztest happened to run them in.
 *
 * So the cases that draw a session live here, in one binary, as ONE case. They were three; what
 * they asserted is unchanged and now runs in sequence against a single session. The sibling binary
 * keeps the cases that draw none, where a repeated init() has no worker to move the ground under.
 *
 * AND THE WORKER DOES ITS OWN WORK. The old version called rt::service_once() from the test thread
 * and justified it with a claim the runtime header takes back. The poll period here is short enough
 * that the worker services the transaction itself, and the test waits for the frame.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#include <errno.h>
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
/* HOW THE SUITE SEES AN init() IT WAS NOT SUPPOSED TO GET. rt::init() draws the session token, and
 * this is the only draw in the binary, so the count is a direct witness: it going up means the
 * session layer was reconfigured. Asserting on the token's value could not see it -- the fake
 * returns the same number every time, so a second init would install an identical token and look
 * like nothing had happened. */
int draws_{0};
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
 * queue, so a moment is needed before the answer is in the queue.
 *
 * SESSION FRAMES ARE SKIPPED, because the worker announces on its own schedule and a suite that
 * treated the next frame as the answer would read an announcement as a transaction status -- which
 * is exactly what a host must not do either. */
bool next_transaction(struct can_frame &out, k_timeout_t wait = K_MSEC(200))
{
    for (;;) {
        if (k_msgq_get(&status_msgq, &out, wait) != 0)
            return false;
        wire::status_kind kind{};
        if (wire::decode_status_kind(out.data, out.dlc, kind) != wire::decode_error::none)
            continue;
        if (kind == wire::status_kind::transaction)
            return true;
    }
}

/* And the other direction: nothing was ANSWERED, whatever else the board may have announced. */
bool no_transaction_within(k_timeout_t wait = K_MSEC(200))
{
    struct can_frame f {};
    return !next_transaction(f, wait);
}

/* What the authority would say. A successful prove installs a mapping in production, so the fake
 * does the same; the cases that make this disagree with the session's memory live in #116's suite,
 * which can drive the session directly. */
bool installed_proven_{false};
uint8_t installed_epoch_{0};

bool test_installed(void *, uint8_t *epoch)
{
    if (!installed_proven_)
        return false;
    *epoch = installed_epoch_;
    return true;
}

bind::config configured()
{
    bind::config c{};
    c.profile_enabled = true;
    c.max_proof_attempts = 3;
    c.max_start_attempts = 3;
    c.announce_period_ms = 60000;  /* inert: the announcement is not what this suite observes */
    /* SHORT ENOUGH THAT THE WORKER SERVICES THE TRANSACTION, which is what removes the old
     * cross-thread rt::service_once() call. The test waits for the answer frame instead. */
    c.poll_ms = 5;
    c.enumeration_permitted = test_permitted;
    c.installed_mapping = test_installed;
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
    ++draws_;
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

outcome prove(uint32_t epoch)
{
    ++proves_;
    /* A successful proof installs a mapping, so the authority view this suite hands the session
     * agrees with it from here on. Without that the session's epoch gate would refuse the next
     * request for the epoch it had just proved. */
    installed_proven_ = true;
    installed_epoch_ = static_cast<uint8_t>(epoch);
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
    /* draws_ is deliberately not reset. It counts for the life of the binary, which is the scope
     * ONE INIT PER BOOT is about; zeroing it per case would hide exactly what it is here to see. */
}

ZTEST_SUITE(tof_commission_bind_session, NULL, suite_setup, before, NULL, NULL);

/* ONE SESSION, ONE WORKER, EVERYTHING THAT NEEDS THEM -- in order. Three cases before, each of
 * which re-initialised the session layer under the live worker of the one before it.
 *
 * THE RELEASE CONDITION IS DENIED FOR THE WHOLE SCENARIO, which is not a convenience: it is the
 * default every image but the bench one has, and it is what makes one session enough. Merging the
 * old cases with permission HELD would have let the first request prove and install epoch 7, and
 * the later request naming another epoch would then be refused by #116's epoch gate rather than by
 * the condition -- a different refusal, asserted as if it were this one. The proof path has its own
 * binaries in tof_commission_runtime and tof_commission_worker.
 *
 * What the board must still do with permission denied is everything this file is about: install the
 * filter, accept the frame in the receive path, answer on the status identifier, transmit on
 * nothing else, and refuse the transaction without touching the chain. */
ZTEST(tof_commission_bind_session, test_a_session_serves_requests_on_the_allocated_identifiers)
{
    permitted_ = false;

    const bind::result r{bind::start(can_dev(), configured())};
    zassert_equal(r.rc, 0, "a session was drawn");
    zassert_true(r.state == bind::outcome::running, "so the board is commissionable");
    zassert_true(r.filter_installed, "the request filter is installed");
    /* ASSERTED ON ITS OWN, not `|| rt::running()`. That disjunction is what hid the defect: from
     * the second start onwards rt::start() returned -EALREADY, worker_started was false, and the
     * running() of the FIRST case's worker let the assertion through. */
    zassert_true(r.worker_started, "and this call is the one that started the worker");

    /* --- a real frame, through the real driver, on the allocated identifier --- */
    zassert_equal(send_request(ctr::kCommissionRequestId, 1, 7, token_), 0, "the request goes out");

    struct can_frame answer {};
    zassert_true(next_transaction(answer), "the board answered");
    zassert_equal(answer.id, ctr::kCommissionStatusId, "on 0x219, the status identifier");
    zassert_equal(answer.dlc, wire::kFrameLen, "eight bytes");

    wire::transaction_status t{};
    zassert_equal(wire::decode_transaction_status(answer.data, answer.dlc, t),
                  wire::decode_error::none, "and it decodes");
    zassert_true(t.ph == wire::phase::accepted, "accepted: the receive path does no work");
    zassert_equal(t.seq, 1, "for the request that asked");

    /* --- and the worker's own tick refuses it, with no cross-thread service_once() here --- */
    zassert_true(next_transaction(answer, K_MSEC(500)), "and then answered, by the worker");
    zassert_equal(wire::decode_transaction_status(answer.data, answer.dlc, t),
                  wire::decode_error::none, "");
    zassert_true(t.ph == wire::phase::refused, "refused");
    zassert_true(t.res == wire::result::not_permitted, "because the condition said no");
    zassert_equal(t.seq, 1, "for that same request");
    zassert_true(permitted_calls_ >= 1, "and it was ASKED, not assumed");
    zassert_equal(proves_, 0, "NOT ONE PROOF RAN");
    zassert_equal(starts_, 0, "and acquisition was never started");

    /* NOTHING OF OURS EVER APPEARS ON THE REQUEST IDENTIFIER. The observer on 0x218 sees the
     * test's own frame and must see nothing else. */
    struct can_frame echoed {};
    zassert_equal(k_msgq_get(&request_msgq, &echoed, K_MSEC(50)), 0, "the request itself");
    zassert_equal(k_msgq_get(&request_msgq, &echoed, K_MSEC(200)), -EAGAIN,
                  "and the board transmitted nothing on it");

    /* --- only the request identifier is received --- */
    const uint32_t before_in{rt::stats().frames_in};
    zassert_equal(send_request(ctr::kCommissionStatusId, 2, 7, token_), 0, "on the status id");
    zassert_equal(send_request(ctr::kCommissionRequestId + 2, 3, 7, token_), 0, "and a neighbour");
    k_msleep(50);
    zassert_equal(rt::stats().frames_in, before_in,
                  "neither reached the receive path, so neither was answered");

    /* --- AND A SECOND START IS REFUSED BEFORE IT CAN DO ANY DAMAGE ---
     *
     * A second bind::start() under this live worker is what tof_commission_runtime.hpp forbids: a
     * second rt::init(), which draws a new token, purges the queues and resets the budgets while
     * the worker is mid-pass. An earlier revision let it run and only reported differently at the
     * end; being last in the binary did not make it safe, it made it unobserved.
     *
     * So the assertions are about what did NOT happen, not about what was reported. */
    const int draws_before{draws_};
    const bool running_before{rt::running()};
    const uint32_t in_before{rt::stats().frames_in};

    const bind::result again{bind::start(can_dev(), configured())};
    zassert_true(again.state == bind::outcome::refused, "a second start is refused");
    zassert_equal(again.rc, -EALREADY, "and says why: a worker is already running");
    zassert_false(again.worker_started, "it started nothing");
    zassert_false(again.filter_installed, "and installed nothing");

    /* THE SESSION LAYER WAS NEVER TOUCHED, which is the whole point of refusing early. */
    zassert_equal(draws_, draws_before, "no second token was drawn, so there was no second init");
    zassert_true(rt::has_session(), "the session this boot drew is still the session");
    zassert_true(running_before && rt::running(), "and the original worker is still the worker");

    /* THE OBSERVER IS EMPTIED FIRST, and not as hygiene. The step above deliberately sent a
     * REQUEST onto the status identifier to prove the board does not listen there -- and the
     * observer in this suite listens there, so that frame is sitting in its queue. Worse,
     * next_transaction() does not filter it out: decode_status_kind() reads a request's bytes as a
     * transaction status, so the next read would return the test's own frame and decode it as a
     * phase of 13 with a result of 240. Nothing before this needed a frame after that step, which
     * is why the trap had never been stepped in. */
    drain();

    /* AND THE ORIGINAL FILTER IS STILL THE ONE ON THE BUS. A request on the allocated identifier
     * still reaches the receive path and is still answered under the original token -- which a
     * re-init would have replaced, leaving this frame answered `no_session` instead. */
    zassert_equal(send_request(ctr::kCommissionRequestId, 4, 7, token_), 0, "the request goes out");
    zassert_true(next_transaction(answer), "and the board answered it");
    zassert_equal(wire::decode_transaction_status(answer.data, answer.dlc, t),
                  wire::decode_error::none, "");
    zassert_true(t.ph == wire::phase::accepted, "accepted, not no_session");
    zassert_equal(t.seq, 4, "for the request that asked");
    zassert_true(rt::stats().frames_in > in_before, "through the one filter that is installed");

    teardown(r);
}
