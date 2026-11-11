/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * WHAT THIS SUITE IS FOR, stated precisely because it is easy to read more into it.
 *
 * The property under test is that the depth and the start time stay a consistent pair under every
 * ORDERING of begin and end. It is not a proof of thread safety: concurrency is excluded by the
 * spinlock tof_watchdog_feeder holds across every call, which no host test can exercise. The two
 * are complementary -- the lock reduces every concurrent execution to one of the orderings below,
 * and these cases say what each ordering must produce.
 *
 * The shape that reached a robot was a bare flag with no suite covering it. The two-atomic depth
 * count written to replace it never left the bench: review found that its begin -- read the count,
 * stamp if it was zero, then increment -- lets the LATER stamp survive and defer the deadline of a
 * declaration that was already running. That is the defect these cases exist to keep out, and the
 * reason the bookkeeping is a type rather than two variables in the feeder.
 */

#include <zephyr/ztest.h>

#include "tof_long_operation.hpp"

namespace lo = lexxhard::tof_long_operation;

ZTEST_SUITE(tof_long_operation, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_long_operation, test_a_fresh_state_declares_nothing)
{
    const lo::state s{};
    zassert_false(s.read().active, "nothing has been declared yet");
    zassert_equal(s.depth(), 0U);
}

ZTEST(tof_long_operation, test_one_declaration_is_active_for_exactly_its_own_extent)
{
    lo::state s{};
    s.begin(1000);
    zassert_true(s.read().active);
    zassert_equal(s.read().began_ms, 1000U);
    s.end();
    zassert_false(s.read().active, "the declaration ended, so nothing is covered");
}

/* THE ONE THE CAP DEPENDS ON. bounds::long_operation_ms is measured from `began_ms`, so if a nested
 * begin restamped it, two overlapping declarations could hand the deadline forward indefinitely and
 * the cap would never fire -- which is the hang the cap exists to end. */
ZTEST(tof_long_operation, test_a_nested_declaration_does_not_defer_the_earliest_deadline)
{
    lo::state s{};
    s.begin(1000);
    s.begin(9000);
    zassert_equal(s.read().began_ms, 1000U,
                  "the deadline belongs to the earliest live declaration; a later begin must not "
                  "move it, or an overlapping pair buys unbounded silence");
    zassert_equal(s.depth(), 2U);

    /* And the inner end does not uncover the outer one -- the defect that made this a count. */
    s.end();
    zassert_true(s.read().active, "the outer declaration is still live");
    zassert_equal(s.read().began_ms, 1000U, "and it is still judged from when it began");

    s.end();
    zassert_false(s.read().active);
}

/* The interleaving that the two-atomic begin admitted, replayed as the ordering the lock reduces it
 * to. Whichever caller registers first owns the timestamp, and the second must not touch it. */
ZTEST(tof_long_operation, test_whichever_declaration_registers_first_owns_the_timestamp)
{
    lo::state a{};
    a.begin(100);   /* the caller that got the lock first */
    a.begin(101);   /* the one that had already read a later clock */
    zassert_equal(a.read().began_ms, 100U, "the later stamp must not survive");

    /* The mirror ordering, to show the result follows registration order and nothing else. */
    lo::state b{};
    b.begin(101);
    b.begin(100);
    zassert_equal(b.read().began_ms, 101U,
                  "began_ms is the registration time of the first live declaration, not the "
                  "minimum of the clocks the callers read");
}

/* Deep nesting, because a count that saturated or wrapped would read as a declaration nobody can
 * close, and the symptom would be a board declared busy until the cap fired on every boot. */
ZTEST(tof_long_operation, test_depth_unwinds_exactly_and_only_the_last_end_uncovers)
{
    lo::state s{};
    constexpr uint32_t kDepth{64};
    for (uint32_t i{0}; i < kDepth; ++i)
        s.begin(2000 + i);
    zassert_equal(s.depth(), kDepth);
    zassert_equal(s.read().began_ms, 2000U);

    for (uint32_t i{0}; i < kDepth - 1; ++i) {
        s.end();
        zassert_true(s.read().active, "%u ends in, one declaration is still live", i + 1);
    }
    s.end();
    zassert_false(s.read().active);
    zassert_equal(s.depth(), 0U);
}

/* An end that never had a begin is the shape a missed return path leaves behind. It must not take
 * the depth below zero: on an unsigned count that is a huge positive number, so the board would be
 * covered for the rest of the boot and the only thing left to stop it would be the cap. */
ZTEST(tof_long_operation, test_an_unpaired_end_is_ignored_rather_than_wrapping_the_depth)
{
    lo::state s{};
    s.end();
    s.end();
    s.end();
    zassert_equal(s.depth(), 0U, "an unpaired end must not wrap the count");
    zassert_false(s.read().active, "and must not leave the board declared busy");

    /* The state still works afterwards, with a FRESH timestamp rather than a stale one. */
    s.begin(5000);
    zassert_true(s.read().active);
    zassert_equal(s.read().began_ms, 5000U);
}

ZTEST(tof_long_operation, test_more_ends_than_begins_leaves_the_state_usable)
{
    lo::state s{};
    s.begin(1000);
    s.end();
    s.end();   /* the extra one */
    zassert_equal(s.depth(), 0U);

    s.begin(8000);
    zassert_equal(s.read().began_ms, 8000U,
                  "a new declaration is timed from itself; carrying the previous stamp forward "
                  "would make the cap fire against an operation that had already finished");
}

/* The stale-timestamp half of the original defect: after everything is released, `began_ms` is only
 * meaningful again once a new declaration has registered. */
ZTEST(tof_long_operation, test_a_new_declaration_is_never_judged_from_the_previous_one)
{
    lo::state s{};
    s.begin(1000);
    s.end();
    zassert_false(s.read().active);

    s.begin(40000);
    const lo::snapshot taken{s.read()};
    zassert_true(taken.active);
    zassert_equal(taken.began_ms, 40000U,
                  "judged from 1000 instead, a 30 s cap would fire immediately on a declaration "
                  "that had just started");
}
