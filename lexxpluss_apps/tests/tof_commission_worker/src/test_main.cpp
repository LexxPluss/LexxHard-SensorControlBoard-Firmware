/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * THE WORKER, IN ITS OWN BINARY. One case, because it creates the worker thread and a thread cannot
 * be unmade: it outlives the case that created it, and the k_thread object it was created on cannot
 * be reused while it lives. Anything that ran after it in the same binary ran against a live worker.
 *
 * It used to live in the runtime suite, where it also called init() a second time to show that
 * start() stays refused -- and init() reconfigures the session layer, drawing a new token and
 * resetting the attempt budgets, which #116 states is a once-per-boot operation. So the case was
 * establishing a refusal by performing the thing the contract forbids, and every later case in that
 * binary inherited the result. Separating the binaries is the fix; see tof_commission_runtime.hpp
 * for the precondition, which is documented rather than enforced.
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

bool installed_proven_{false};
uint8_t installed_epoch_{0};

/* WHAT THE AUTHORITY WOULD SAY. A successful prove installs a mapping in production, so the fake
 * does the same and the existing cases behave as they did; the ones that make this disagree with
 * the session's own memory belong to #116's suite, which can drive it directly. */
bool fake_installed(void *, uint8_t *epoch)
{
    if (!installed_proven_)
        return false;
    *epoch = installed_epoch_;
    return true;
}

rt::hooks wired()
{
    return rt::hooks{fake_send,  fake_permitted, fake_prove,
                     fake_start, fake_installed, fake_draw,
                     nullptr};
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
ZTEST_SUITE(tof_commission_worker, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_commission_worker, test_two_callers_racing_to_start_produce_one_worker)
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

    /* And a later sequential caller is refused too: the k_thread object cannot be reused while the
     * thread it carries is alive.
     *
     * IT DELIBERATELY NO LONGER RE-INITIALISES to show that. init() draws a new session token and
     * resets the attempt budgets, which is a once-per-boot operation -- asserting the refusal that
     * way meant breaking the precondition in order to observe it. The precondition is documented in
     * tof_commission_runtime.hpp; it is not a behaviour under test. */
    zassert_equal(rt::start(), -EALREADY, "");
}
