/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The one property that matters here cannot be tested by sending on a real bus, because a real
 * driver always completes eventually. The defect being fixed is what happens when it NEVER
 * completes -- a bus with no other node awake, where bxCAN retransmits forever -- so the driver
 * under the wrapper is a stub whose send() can be told to accept a frame and then never call the
 * completion callback at all. If the wrapper waited for completion, the first case below would not
 * fail; it would hang, and the suite would time out. That is the test.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/ztest.h>

#include "zcan_bounded_send.hpp"

namespace
{

namespace bs = lexxhard::zcan_bounded_send;

/* What the stub driver should do with the next frame. */
int stub_rc{0};
bool stub_complete_immediately{false};
int stub_complete_error{0};
can_tx_callback_t held_callback{nullptr};
void *held_user_data{nullptr};
int stub_calls{0};
k_timeout_t stub_seen_timeout{};

int stub_send(const device *dev, const can_frame *frame, k_timeout_t timeout,
              can_tx_callback_t callback, void *user_data)
{
    ARG_UNUSED(frame);
    ++stub_calls;
    stub_seen_timeout = timeout;

    /* The real drivers assert this, and the whole point of the wrapper is that it is never null.
     * Checking it here means a regression in the wrapper fails as an assertion rather than as a
     * mysterious hang. */
    zassert_not_null(callback, "the wrapper must pass a completion callback");

    if (stub_rc != 0)
        return stub_rc;

    held_callback = callback;
    held_user_data = user_data;
    if (stub_complete_immediately)
        callback(dev, stub_complete_error, user_data);
    return 0;
}

/* Only `send` is filled in: nothing in this suite reaches another entry point, and a stub that
 * pretends to implement the rest would be a worse lie than a null. */
const can_driver_api stub_api{
    .get_capabilities{nullptr},
    .start{nullptr},
    .stop{nullptr},
    .set_mode{nullptr},
    .set_timing{nullptr},
    .send{stub_send},
};

device stub_dev{};

can_frame a_frame()
{
    can_frame f{};
    f.id = 0x214;
    f.dlc = 8;
    memset(f.data, 0xA5, sizeof f.data);
    return f;
}

void before(void *)
{
    stub_dev = device{};
    stub_dev.name = "stub_can";
    stub_dev.api = &stub_api;
    stub_rc = 0;
    stub_complete_immediately = false;
    stub_complete_error = 0;
    held_callback = nullptr;
    held_user_data = nullptr;
    stub_calls = 0;
    bs::reset_counts();
}

}  // namespace

ZTEST_SUITE(zcan_bounded_send, nullptr, nullptr, before, nullptr, nullptr);

/* THE DEFECT, DIRECTLY. The controller takes the frame and nothing ever acknowledges it. The old
 * call form would be parked in k_sem_take(&ctx.done, K_FOREVER) at this point and this test would
 * never return; the elapsed-time assertion is secondary to the fact that the line after the call
 * runs at all. */
ZTEST(zcan_bounded_send, test_a_frame_that_is_never_acknowledged_does_not_block_the_sender)
{
    const can_frame f{a_frame()};
    const int64_t before_ms{k_uptime_get()};

    const int rc{bs::send(&stub_dev, &f, K_MSEC(100))};

    const int64_t elapsed{k_uptime_get() - before_ms};
    zassert_equal(rc, 0, "rc %d", rc);
    zassert_true(elapsed < 50, "returned after %lld ms: something is waiting on the wire", elapsed);
    zassert_is_null(held_callback ? nullptr : (void *)1, "the callback was never handed over");

    /* Accepted by the controller, not acknowledged by anyone. The two are now counted apart, which
     * is the only way a caller can still tell the difference. */
    const bs::counts c{bs::snapshot()};
    zassert_equal(c.queued, 1U);
    zassert_equal(c.completed, 0U);
    zassert_equal(c.failed, 0U);
    zassert_equal(c.refused, 0U);
}

/* A SILENT BUS MUST BECOME VISIBLE SOMEWHERE, and with the completion no longer in the return value
 * this is where. The frame goes out, the controller gives up on it, and `failed` rises while
 * `queued` keeps up -- the shape that says "transmitting into nothing". */
ZTEST(zcan_bounded_send, test_a_frame_the_controller_gives_up_on_is_counted_as_failed)
{
    stub_complete_immediately = true;
    stub_complete_error = -EIO;
    const can_frame f{a_frame()};

    zassert_equal(bs::send(&stub_dev, &f, K_MSEC(100)), 0);

    const bs::counts c{bs::snapshot()};
    zassert_equal(c.queued, 1U);
    zassert_equal(c.failed, 1U, "a silent bus must be visible in the counters");
    zassert_equal(c.completed, 0U);
}

ZTEST(zcan_bounded_send, test_an_acknowledged_frame_is_counted_as_completed)
{
    stub_complete_immediately = true;
    const can_frame f{a_frame()};

    zassert_equal(bs::send(&stub_dev, &f, K_MSEC(100)), 0);

    const bs::counts c{bs::snapshot()};
    zassert_equal(c.queued, 1U);
    zassert_equal(c.completed, 1U);
    zassert_equal(c.failed, 0U);
}

/* MAILBOX EXHAUSTION IS STILL LOSSY AND THAT IS DELIBERATE. The timeout is the caller's, it is
 * passed through unchanged, and a frame that cannot get in is refused rather than queued -- the
 * cliff publisher reads exactly this to withhold its health frame. */
ZTEST(zcan_bounded_send, test_a_frame_that_cannot_reach_a_mailbox_is_refused_not_queued)
{
    stub_rc = -EAGAIN;
    const can_frame f{a_frame()};

    zassert_equal(bs::send(&stub_dev, &f, K_MSEC(7)), -EAGAIN);

    const bs::counts c{bs::snapshot()};
    zassert_equal(c.refused, 1U);
    zassert_equal(c.queued, 0U);
    zassert_equal(c.completed, 0U);
    zassert_equal(c.failed, 0U);
}

/* The caller's timeout reaches the driver unaltered. Worth pinning: the wrapper's whole claim is
 * that the call is bounded BY THAT VALUE, and a wrapper that quietly substituted K_FOREVER here
 * would pass every other case in this file. */
ZTEST(zcan_bounded_send, test_the_callers_timeout_is_passed_through_unchanged)
{
    const can_frame f{a_frame()};

    (void)bs::send(&stub_dev, &f, K_MSEC(1));

    zassert_equal(stub_seen_timeout.ticks, K_MSEC(1).ticks, "the timeout was rewritten");
    zassert_false(K_TIMEOUT_EQ(stub_seen_timeout, K_FOREVER), "the timeout became K_FOREVER");
}

/* A bus that is down should not reach the driver's assertions, and a caller that passes nothing
 * should not reach the driver at all. */
ZTEST(zcan_bounded_send, test_a_missing_device_or_frame_never_reaches_the_driver)
{
    const can_frame f{a_frame()};

    zassert_equal(bs::send(nullptr, &f, K_MSEC(100)), -EINVAL);
    zassert_equal(bs::send(&stub_dev, nullptr, K_MSEC(100)), -EINVAL);
    zassert_equal(stub_calls, 0, "the driver was called with nothing to send");

    /* Not counted either way: these never became transmissions. */
    const bs::counts c{bs::snapshot()};
    zassert_equal(c.queued + c.refused + c.completed + c.failed, 0U);
}

/* Counters accumulate across calls rather than reporting only the last one, which is what makes
 * them readable as a rate. Saturation at UINT32_MAX is NOT covered here: reaching it needs four
 * billion sends, and a test that slow would be skipped and then rot. It is a three-line loop in
 * the implementation, reviewed rather than exercised. */
ZTEST(zcan_bounded_send, test_counters_accumulate_across_calls)
{
    const can_frame f{a_frame()};
    stub_complete_immediately = true;
    (void)bs::send(&stub_dev, &f, K_MSEC(100));
    (void)bs::send(&stub_dev, &f, K_MSEC(100));
    stub_rc = -EAGAIN;
    (void)bs::send(&stub_dev, &f, K_MSEC(100));

    const bs::counts c{bs::snapshot()};
    zassert_equal(c.queued, 2U);
    zassert_equal(c.completed, 2U);
    zassert_equal(c.refused, 1U);
}
