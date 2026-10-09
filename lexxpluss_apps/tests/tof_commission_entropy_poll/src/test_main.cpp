/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The entropy read's deadline.
 *
 * WHY THIS SUITE EXISTS. The session token's read used Zephyr's blocking entropy call, which on the
 * STM32 driver cannot fail: it waits on a semaphore with K_FOREVER until the pool refills and
 * returns zero either way, while the refill interrupt leaves early on a seed or clock error without
 * giving that semaphore. An RNG that stopped producing therefore parked the caller for ever instead
 * of failing -- and tof_commission_entropy.hpp's contract, that a read which fails ends in no
 * session, was unreachable because the read never returned. Nothing covered it: every other suite
 * replaces draw_token wholesale, and the module itself cannot be compiled on a host.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_commission_entropy_poll.hpp"

namespace
{

namespace ep = lexxhard::tof_commission_entropy::detail;

struct script {
    /* Bytes handed over per attempt, consumed in order; the last value repeats. */
    int per_attempt[8]{};
    int attempts_scripted{0};
    int negative_after{-1};  /* attempt index at which take reports an errno; -1 = never */
    int errno_value{-EIO};
    int over_report{0};      /* bytes to claim beyond what was asked, on the first attempt */

    int attempts{0};
    int waits{0};
    int64_t now{1000};
    int64_t ms_per_wait{1};
};

script s_{};

int take(void *, uint8_t *dst, uint16_t len)
{
    const int i{s_.attempts++};
    if (s_.negative_after >= 0 && i >= s_.negative_after)
        return s_.errno_value;
    if (s_.over_report != 0 && i == 0)
        return static_cast<int>(len) + s_.over_report;

    const int idx{i < s_.attempts_scripted ? i : (s_.attempts_scripted > 0 ? s_.attempts_scripted - 1 : 0)};
    const int give{s_.attempts_scripted > 0 ? s_.per_attempt[idx] : 0};
    const int n{give > static_cast<int>(len) ? static_cast<int>(len) : give};
    for (int k{0}; k < n; ++k)
        dst[k] = static_cast<uint8_t>(0xA0 + k);
    return n;
}

int64_t now_ms(void *)
{
    return s_.now;
}

void wait(void *)
{
    ++s_.waits;
    s_.now += s_.ms_per_wait;
}

ep::poll_io io()
{
    return ep::poll_io{take, now_ms, wait, nullptr};
}

void before(void *)
{
    s_ = script{};
}

}  // namespace

ZTEST_SUITE(tof_commission_entropy_poll, nullptr, nullptr, before, nullptr, nullptr);

/* THE DEFECT, AS A RETURN VALUE. An RNG that never hands over a byte used to block for ever. It now
 * gives up, and the errno is what the caller turns into "no session". */
ZTEST(tof_commission_entropy_poll, test_an_rng_that_never_produces_times_out_instead_of_blocking)
{
    s_.attempts_scripted = 1;
    s_.per_attempt[0] = 0;

    uint8_t buf[4]{0x5A, 0x5A, 0x5A, 0x5A};
    zassert_equal(ep::fill(io(), buf, sizeof buf, 50), -ETIMEDOUT, "");
    zassert_true(s_.attempts > 1, "it kept asking: %d attempts", s_.attempts);
    zassert_true(s_.waits > 0, "and waited between asking");
    /* The deadline is honoured rather than merely reached: with 1 ms per wait, a 50 ms budget must
     * not spend 500. */
    zassert_true(s_.waits <= 51, "it waited %d times for a 50 ms deadline", s_.waits);
}

/* A WORKING RNG IS NOT PENALISED, which is the half a timeout makes easy to break. */
ZTEST(tof_commission_entropy_poll, test_a_full_first_read_succeeds_without_waiting)
{
    s_.attempts_scripted = 1;
    s_.per_attempt[0] = 4;

    uint8_t buf[4]{};
    zassert_ok(ep::fill(io(), buf, sizeof buf, 50), "");
    zassert_equal(s_.attempts, 1, "one attempt was enough");
    zassert_equal(s_.waits, 0, "so nothing waited");
    zassert_equal(buf[0], 0xA0);
    zassert_equal(buf[3], 0xA3);
}

/* THE ORDINARY CASE ON REAL HARDWARE: the ISR pool hands over what it has, which is rarely four
 * bytes at once, so they accumulate across attempts and the buffer is filled in order. */
ZTEST(tof_commission_entropy_poll, test_bytes_accumulate_across_attempts)
{
    s_.attempts_scripted = 4;
    s_.per_attempt[0] = 1;
    s_.per_attempt[1] = 0;   /* a pass with nothing available is not a failure */
    s_.per_attempt[2] = 2;
    s_.per_attempt[3] = 1;

    uint8_t buf[4]{};
    zassert_ok(ep::fill(io(), buf, sizeof buf, 50), "");
    zassert_equal(s_.attempts, 4, "");
    /* Each attempt writes from the start of what it was given, so the bytes land in sequence. */
    zassert_equal(buf[0], 0xA0);
    zassert_equal(buf[1], 0xA0);
    zassert_equal(buf[2], 0xA1);
    zassert_equal(buf[3], 0xA0);
}

/* A deadline of zero still looks once. On working hardware the first read usually satisfies the
 * request, and refusing to look at all would turn a tight deadline into a guaranteed failure. */
ZTEST(tof_commission_entropy_poll, test_a_zero_deadline_still_makes_one_attempt)
{
    s_.attempts_scripted = 1;
    s_.per_attempt[0] = 4;

    uint8_t buf[4]{};
    zassert_ok(ep::fill(io(), buf, sizeof buf, 0), "");
    zassert_equal(s_.attempts, 1, "");

    /* And with nothing available it gives up after that one look rather than waiting. */
    s_ = script{};
    s_.attempts_scripted = 1;
    s_.per_attempt[0] = 0;
    zassert_equal(ep::fill(io(), buf, sizeof buf, 0), -ETIMEDOUT, "");
    zassert_equal(s_.attempts, 1, "one look, no wait");
    zassert_equal(s_.waits, 0, "");
}

/* An errno from the read is passed through rather than flattened into the timeout: "the RNG is not
 * producing" and "the driver refused" are different faults and the log line should say which. */
ZTEST(tof_commission_entropy_poll, test_a_read_error_is_reported_as_itself)
{
    s_.attempts_scripted = 1;
    s_.per_attempt[0] = 0;
    s_.negative_after = 0;
    s_.errno_value = -ENODEV;

    uint8_t buf[4]{};
    zassert_equal(ep::fill(io(), buf, sizeof buf, 50), -ENODEV, "");
    zassert_equal(s_.attempts, 1, "and it stopped there");
}

/* An error partway through is still an error: a half-filled buffer must not be reported as a token.
 */
ZTEST(tof_commission_entropy_poll, test_an_error_after_some_bytes_is_not_a_partial_success)
{
    s_.attempts_scripted = 1;
    s_.per_attempt[0] = 2;
    s_.negative_after = 1;

    uint8_t buf[4]{};
    zassert_equal(ep::fill(io(), buf, sizeof buf, 50), -EIO, "");
}

/* A seam that claims more than it was asked for is a broken seam, not a windfall. Reporting success
 * there would mean a token assembled from bytes nobody wrote. */
ZTEST(tof_commission_entropy_poll, test_a_read_claiming_more_than_asked_is_refused)
{
    s_.attempts_scripted = 1;
    s_.per_attempt[0] = 4;
    s_.over_report = 1;

    uint8_t buf[4]{};
    zassert_equal(ep::fill(io(), buf, sizeof buf, 50), -EIO, "");
}

ZTEST(tof_commission_entropy_poll, test_an_unwired_seam_is_refused_rather_than_called)
{
    uint8_t buf[4]{};
    ep::poll_io bad{io()};
    bad.take = nullptr;
    zassert_equal(ep::fill(bad, buf, sizeof buf, 50), -EINVAL, "");
    bad = io();
    bad.now_ms = nullptr;
    zassert_equal(ep::fill(bad, buf, sizeof buf, 50), -EINVAL, "");
    bad = io();
    bad.wait = nullptr;
    zassert_equal(ep::fill(bad, buf, sizeof buf, 50), -EINVAL, "");
    zassert_equal(ep::fill(io(), nullptr, sizeof buf, 50), -EINVAL, "");
}
