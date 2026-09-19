/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The runtime adapter: which identifier a frame is answered on, what a frame under some other
 * identifier does, when the session frame goes out, and that there is one worker.
 *
 * THE IDENTIFIERS HERE ARE TEST VALUES, and that is the point rather than a convenience. The
 * allocated pair is 0x218/0x219 and lives in the generated contract; using it here would make this
 * suite pass identically against a runtime that ignored its configuration and used the real values
 * directly. Two arbitrary numbers cannot.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <string.h>

#include "tof_commission_runtime.hpp"
#include "tof_commission_wire.hpp"

namespace rt = lexxhard::tof_commission_runtime;
namespace cs = lexxhard::tof_commission;
namespace wire = lexxhard::tof_commission_wire;
namespace cm = lexxhard::tof_commissioning;

namespace {

/* Deliberately NOT 0x218/0x219. */
constexpr uint32_t kTestRequestId{0x5A1};
constexpr uint32_t kTestStatusId{0x5A2};

struct frame {
    uint32_t id{0};
    uint8_t data[wire::kFrameLen]{};
};

struct fakes {
    uint32_t token{0x77665544U};
    bool token_available{true};
    bool permitted{true};
    int prove_rc{0};
    int start_rc{0};
    bool send_fails{false};

    int proves{0};
    int starts{0};
    int draws{0};

    frame sent[16]{};
    size_t sent_count{0};
};

fakes f_{};

int fake_send(void *, uint32_t id, const uint8_t *data, size_t len)
{
    if (f_.send_fails)
        return -EIO;
    if (f_.sent_count < (sizeof f_.sent / sizeof f_.sent[0]) && len == wire::kFrameLen) {
        f_.sent[f_.sent_count].id = id;
        memcpy(f_.sent[f_.sent_count].data, data, len);
        ++f_.sent_count;
    }
    return 0;
}

bool fake_permitted(void *)
{
    return f_.permitted;
}

int fake_prove(void *, uint32_t, cm::outcome *out)
{
    ++f_.proves;
    *out = cm::outcome{};
    return f_.prove_rc;
}

int fake_start(void *)
{
    ++f_.starts;
    return f_.start_rc;
}

int fake_draw(void *, uint32_t *out)
{
    ++f_.draws;
    if (!f_.token_available)
        return -ENODEV;
    *out = f_.token;
    return 0;
}

rt::hooks wired()
{
    return rt::hooks{fake_send, fake_permitted, fake_prove, fake_start, fake_draw, nullptr};
}

rt::config configured()
{
    rt::config c{};
    c.request_id = kTestRequestId;
    c.status_id = kTestStatusId;
    c.announce_period_ms = 1000;
    c.poll_ms = 20;
    c.profile_enabled = true;
    c.max_proof_attempts = 3;
    c.max_start_attempts = 3;
    return c;
}

void build_request(uint8_t out[wire::kFrameLen], uint8_t seq, uint8_t epoch, uint32_t token)
{
    wire::request r{};
    r.raw_op = static_cast<uint8_t>(wire::opcode::prove_and_start);
    r.seq = seq;
    r.wire_epoch = epoch;
    r.session_token = token;
    wire::encode_request(r, out);
}

/* The session frame the board has sent, if any -- which is where a host learns the token. */
bool last_session(wire::session_status &out)
{
    for (size_t i = f_.sent_count; i > 0; --i) {
        wire::status_kind kind{};
        if (wire::decode_status_kind(f_.sent[i - 1].data, wire::kFrameLen, kind) !=
            wire::decode_error::none)
            continue;
        if (kind != wire::status_kind::session)
            continue;
        return wire::decode_session_status(f_.sent[i - 1].data, wire::kFrameLen, out) ==
               wire::decode_error::none;
    }
    return false;
}

bool last_transaction(wire::transaction_status &out)
{
    for (size_t i = f_.sent_count; i > 0; --i) {
        wire::status_kind kind{};
        if (wire::decode_status_kind(f_.sent[i - 1].data, wire::kFrameLen, kind) !=
            wire::decode_error::none)
            continue;
        if (kind != wire::status_kind::transaction)
            continue;
        return wire::decode_transaction_status(f_.sent[i - 1].data, wire::kFrameLen, out) ==
               wire::decode_error::none;
    }
    return false;
}

} // namespace

static void before(void *)
{
    f_ = fakes{};
}

ZTEST_SUITE(tof_commission_runtime, NULL, NULL, before, NULL, NULL);

/* ---- configuration is required, never defaulted ---- */

ZTEST(tof_commission_runtime, test_a_configuration_without_identifiers_is_refused)
{
    /* THE WHOLE REASON THIS MODULE NAMES NO NUMBER. A default identifier is a frame on somebody
     * else's conversation, and a runtime that supplied one would be choosing an allocation. */
    rt::config c{configured()};
    c.request_id = 0;
    zassert_equal(rt::init(c, wired()), -EINVAL, "no request identifier");

    c = configured();
    c.status_id = 0;
    zassert_equal(rt::init(c, wired()), -EINVAL, "no status identifier");

    c = configured();
    c.status_id = c.request_id;
    zassert_equal(rt::init(c, wired()), -EINVAL,
                  "one value for both would answer a request with a frame the sender reads back "
                  "as a request");

    c = configured();
    c.announce_period_ms = 0;
    zassert_equal(rt::init(c, wired()), -EINVAL, "a period of zero announces nothing, for ever");

    /* Standard 11-bit identifiers. A value that does not fit is a configuration error to report
     * now, not one for the CAN driver to truncate into somebody else's identifier later. */
    c = configured();
    c.request_id = 0x800;
    zassert_equal(rt::init(c, wired()), -EINVAL, "0x800 is not an 11-bit identifier");
    c = configured();
    c.status_id = 0x1FFFFFFF;
    zassert_equal(rt::init(c, wired()), -EINVAL, "nor is a 29-bit one on this bus");
}

ZTEST(tof_commission_runtime, test_a_missing_hook_is_refused_rather_than_defaulted)
{
    rt::hooks h{wired()};
    h.send = nullptr;
    zassert_equal(rt::init(configured(), h), -EINVAL, "no transport");

    h = wired();
    h.enumeration_permitted = nullptr;
    zassert_equal(rt::init(configured(), h), -EINVAL,
                  "the stationary condition is asked, never assumed");

    h = wired();
    h.draw_token = nullptr;
    zassert_equal(rt::init(configured(), h), -EINVAL, "no entropy, and no fallback to invent one");
}

ZTEST(tof_commission_runtime, test_no_entropy_announces_nothing_but_still_answers)
{
    f_.token_available = false;
    zassert_equal(rt::init(configured(), wired()), -ENODEV, "no session could be drawn");
    zassert_false(rt::has_session(), "and it says so");

    /* NOT ANNOUNCED: a board that cannot tell this boot from the last one has no session to
     * announce, and announcing one anyway is how a host addresses a boot that does not exist. */
    (void)rt::service_once(0);
    (void)rt::service_once(10000);
    zassert_equal(f_.sent_count, 0u, "no session frame, at any time");

    /* BUT STILL ANSWERED, and this is the difference that matters. A host coming back with a
     * durable pending request from a previous boot retransmits it; silence leaves it retransmitting
     * for ever, while `no_session` is terminal and tells it to stop and report. */
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, 0x77665544U);
    rt::on_frame(kTestRequestId, frame, sizeof frame);

    zassert_equal(rt::stats().frames_in, 1u, "the frame was taken in");
    wire::transaction_status t{};
    zassert_true(last_transaction(t), "and answered");
    zassert_true(t.ph == wire::phase::refused, "refused");
    zassert_true(t.res == wire::result::no_session, "with no_session");
    zassert_equal(t.seq, 1, "for the sequence that asked");
    zassert_equal(f_.sent[f_.sent_count - 1].id, kTestStatusId, "on the status identifier");
    zassert_equal(f_.proves, 0, "and nothing was proved or started");
    zassert_equal(f_.starts, 0, "");

    /* There is nothing for a worker to do on such a board. */
    zassert_equal(rt::start(), -EPERM, "no worker is created");
    zassert_false(rt::running(), "");
}

/* ---- identifiers are used as configured ---- */

ZTEST(tof_commission_runtime, test_a_frame_under_another_identifier_is_ignored_and_counted)
{
    zassert_equal(rt::init(configured(), wired()), 0, "");
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, f_.token);

    rt::on_frame(kTestStatusId, frame, sizeof frame);  /* our own status identifier */
    rt::on_frame(0x123, frame, sizeof frame);          /* somebody else's */
    zassert_equal(f_.sent_count, 0u, "nothing was answered");
    zassert_equal(rt::stats().frames_ignored, 2u, "counted: a runtime handed the wrong identifier "
                                                  "looks like one handed nothing");
    zassert_equal(rt::stats().frames_in, 0u, "");
    zassert_equal(f_.proves, 0, "and nothing ran");
}

ZTEST(tof_commission_runtime, test_a_request_is_answered_on_the_status_identifier)
{
    zassert_equal(rt::init(configured(), wired()), 0, "");
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, f_.token);
    rt::on_frame(kTestRequestId, frame, sizeof frame);

    zassert_equal(rt::stats().frames_in, 1u, "taken in");
    zassert_true(f_.sent_count >= 1, "answered");
    for (size_t i = 0; i < f_.sent_count; ++i)
        zassert_equal(f_.sent[i].id, kTestStatusId,
                      "every frame the board sends goes out on the STATUS identifier");

    wire::transaction_status t{};
    zassert_true(last_transaction(t), "a transaction status");
    zassert_true(t.ph == wire::phase::accepted, "accepted -- the receive path does no work");
    zassert_equal(f_.proves, 0, "and it did not prove in the receive path");
}

ZTEST(tof_commission_runtime, test_the_session_frame_carries_the_token_and_is_periodic)
{
    zassert_equal(rt::init(configured(), wired()), 0, "");

    /* The first turn announces whatever the clock says: a host that has just come up must be able to
     * learn the token without waiting a whole period for the first one. */
    (void)rt::service_once(0);
    wire::session_status s{};
    zassert_true(last_session(s), "a session frame went out");
    zassert_equal(s.session_token, f_.token, "carrying the token that was drawn");
    zassert_true(s.profile_enabled, "and the profile the runtime was configured with");
    zassert_equal(f_.sent[f_.sent_count - 1].id, kTestStatusId, "on the status identifier");

    const uint32_t after_first{rt::stats().sessions_sent};
    (void)rt::service_once(999);
    zassert_equal(rt::stats().sessions_sent, after_first, "not before the period is up");
    (void)rt::service_once(1000);
    zassert_equal(rt::stats().sessions_sent, after_first + 1, "and then again");
}

/* ---- the worker ---- */

ZTEST(tof_commission_runtime, test_the_worker_runs_the_transaction_and_answers_on_the_status_id)
{
    zassert_equal(rt::init(configured(), wired()), 0, "");
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, f_.token);
    rt::on_frame(kTestRequestId, frame, sizeof frame);
    zassert_equal(f_.proves, 0, "nothing ran in the receive path");

    const rt::service_result r{rt::service_once(0)};
    zassert_true(r.terminal, "the worker ran it to a terminal status");
    zassert_true(r.sent_status, "which reached the bus");
    zassert_equal(f_.proves, 1, "one proof");
    zassert_equal(f_.starts, 1, "one start");

    wire::transaction_status t{};
    zassert_true(last_transaction(t), "");
    zassert_true(t.ph == wire::phase::done, "done");
    zassert_true(t.res == wire::result::ok, "and ok");
    zassert_equal(t.seq, 1, "for the request that asked");
    for (size_t i = 0; i < f_.sent_count; ++i)
        zassert_equal(f_.sent[i].id, kTestStatusId, "all on the status identifier");
}

ZTEST(tof_commission_runtime, test_a_send_that_fails_is_counted_and_does_not_stop_the_runtime)
{
    /* A board that blocked its receive path on a full mailbox would be a worse failure than a lost
     * frame: the host's retransmission is what recovers this. */
    zassert_equal(rt::init(configured(), wired()), 0, "");
    f_.send_fails = true;

    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, f_.token);
    rt::on_frame(kTestRequestId, frame, sizeof frame);
    const rt::service_result r{rt::service_once(0)};
    /* `sent_` means it REACHED THE BUS. A caller that logged an attempt as traffic would be
     * reporting frames nobody can see. */
    zassert_true(r.terminal, "the transaction ran");
    zassert_false(r.sent_status, "and its status did not go out");
    zassert_false(r.sent_session, "nor did the announcement");

    zassert_true(rt::stats().send_failures >= 2, "counted");
    zassert_equal(f_.sent_count, 0u, "nothing reached the bus");
    zassert_equal(f_.proves, 1, "but the transaction still ran, and its answer is in the table");

    f_.send_fails = false;
    rt::on_frame(kTestRequestId, frame, sizeof frame);
    wire::transaction_status t{};
    zassert_true(last_transaction(t), "the retransmission is answered");
    zassert_true(t.ph == wire::phase::done, "from the table");
    zassert_equal(f_.proves, 1, "without running again");
}

namespace {

K_THREAD_STACK_DEFINE(starter_a_stack, 2048);
K_THREAD_STACK_DEFINE(starter_b_stack, 2048);
struct k_thread starter_a;
struct k_thread starter_b;
int start_rc_a_{-1};
int start_rc_b_{-1};

void call_start_a(void *, void *, void *)
{
    start_rc_a_ = rt::start();
}

void call_start_b(void *, void *, void *)
{
    start_rc_b_ = rt::start();
}

} // namespace

ZTEST(tof_commission_runtime, test_two_callers_racing_to_start_produce_one_worker)
{
    /* ONE TEST FOR BOTH CLAIMS, deliberately: the worker thread outlives the test that created it,
     * so a second test that expected to create its own would be the one that failed, depending on
     * the order ztest happened to run them in.
     *
     * The config makes the worker inert once it exists -- it wakes a minute of uptime from now --
     * so it cannot disturb the tests that follow. */
    rt::config c{configured()};
    c.poll_ms = 60000;
    c.announce_period_ms = 60000;
    zassert_equal(rt::init(c, wired()), 0, "");
    zassert_false(rt::running(), "no worker until one is asked for");

    start_rc_a_ = -1;
    start_rc_b_ = -1;
    k_thread_create(&starter_a, starter_a_stack, K_THREAD_STACK_SIZEOF(starter_a_stack),
                    call_start_a, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_create(&starter_b, starter_b_stack, K_THREAD_STACK_SIZEOF(starter_b_stack),
                    call_start_b, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    zassert_equal(k_thread_join(&starter_a, K_SECONDS(5)), 0, "both callers returned");
    zassert_equal(k_thread_join(&starter_b, K_SECONDS(5)), 0, "");

    /* EXACTLY ONE WINS. What this pins is the CONTRACT, and it is worth saying what it does not
     * pin: on single-CPU native_sim the first starter runs `start()` to completion before the second
     * is scheduled, so removing the lock leaves this test green -- checked by mutation rather than
     * assumed. The lock is what makes the contract hold where a preemption can land between the
     * check and the set, which is the board, not this simulator. */
    const int wins{(start_rc_a_ == 0 ? 1 : 0) + (start_rc_b_ == 0 ? 1 : 0)};
    const int refusals{(start_rc_a_ == -EALREADY ? 1 : 0) + (start_rc_b_ == -EALREADY ? 1 : 0)};
    zassert_equal(wins, 1, "one caller created the worker (a=%d b=%d)", start_rc_a_, start_rc_b_);
    zassert_equal(refusals, 1, "and the other was refused (a=%d b=%d)", start_rc_a_, start_rc_b_);
    zassert_true(rt::running(), "there is a worker");

    /* And a later sequential caller is refused too, including after a re-init: the k_thread object
     * cannot be reused while the thread it carries is alive. */
    zassert_equal(rt::start(), -EALREADY, "");
    zassert_equal(rt::init(c, wired()), 0, "re-initialised");
    zassert_equal(rt::start(), -EALREADY, "and still refused");
}
