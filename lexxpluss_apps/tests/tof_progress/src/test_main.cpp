/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The six heartbeats the watchdog feeder reads.
 *
 * The one that needs proving rather than describing is the CAN slot. Everything the board sends goes
 * through one send(), and which of the two senders it is depends entirely on who called it -- the
 * acquisition thread or the system work queue. The whole reason the watchdog can tell a wedged
 * acquisition sender from a live heartbeat is that this discrimination is right, and it is decided
 * by a single comparison against k_sys_work_q.thread that no amount of reading will confirm. So the
 * suite runs the query from inside a work item and checks it comes back the other way.
 *
 * The counters are file-static and never reset, which is deliberate -- a watchdog input nobody can
 * zero is one fewer way to lie to it -- so every check here is a delta rather than an absolute.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "tof_progress.hpp"

namespace
{

namespace prog = lexxhard::tof_progress;

size_t idx(prog::activity a) { return static_cast<size_t>(a); }

/* What the work queue saw when it asked which slot it was. */
prog::activity from_workq_{prog::activity::count};
K_SEM_DEFINE(workq_done_, 0, 1);

void ask_from_workq(k_work *)
{
    from_workq_ = prog::current_send_slot();
    k_sem_give(&workq_done_);
}

K_WORK_DEFINE(workq_probe_, ask_from_workq);

}  // namespace

ZTEST_SUITE(tof_progress, nullptr, nullptr, nullptr, nullptr, nullptr);

/* THE DISCRIMINATION THE TWO-SLOT DESIGN RESTS ON. A send from the system work queue is the 0x217
 * heartbeat; anything else is the acquisition slot. Get this backwards and a live heartbeat keeps
 * refreshing the timestamp of a sender that died. */
ZTEST(tof_progress, test_the_work_queue_and_everything_else_are_different_send_slots)
{
    zassert_equal(prog::current_send_slot(), prog::activity::send_acq,
                  "a thread that is not the work queue is the acquisition slot");

    from_workq_ = prog::activity::count;
    k_work_submit(&workq_probe_);
    zassert_equal(k_sem_take(&workq_done_, K_SECONDS(2)), 0, "the work item never ran");

    zassert_equal(from_workq_, prog::activity::send_workq,
                  "a send issued from the system work queue is the heartbeat slot");
    zassert_not_equal(from_workq_, prog::current_send_slot(),
                      "the two slots must not collapse into one");
}

/* begun and ended move independently, which is what lets the feeder tell a task that is inside its
 * work from one that has finished. */
ZTEST(tof_progress, test_begin_and_end_move_their_own_halves)
{
    const prog::snapshot before{prog::read()};

    prog::begin(prog::activity::acquisition);
    const prog::snapshot mid{prog::read()};
    zassert_equal(mid.at[idx(prog::activity::acquisition)].begun,
                  before.at[idx(prog::activity::acquisition)].begun + 1);
    zassert_equal(mid.at[idx(prog::activity::acquisition)].ended,
                  before.at[idx(prog::activity::acquisition)].ended,
                  "nothing has finished yet");

    prog::end(prog::activity::acquisition);
    const prog::snapshot after{prog::read()};
    zassert_equal(after.at[idx(prog::activity::acquisition)].ended,
                  before.at[idx(prog::activity::acquisition)].ended + 1);
}

/* Six activities, six counters. A hook wired to the wrong one would make the watchdog blame the
 * wrong subsystem, which is worse than not watching it. */
ZTEST(tof_progress, test_each_activity_has_its_own_counter)
{
    const prog::snapshot before{prog::read()};

    prog::begin(prog::activity::l7);
    prog::end(prog::activity::l7);

    const prog::snapshot after{prog::read()};
    zassert_equal(after.at[idx(prog::activity::l7)].ended,
                  before.at[idx(prog::activity::l7)].ended + 1);
    /* Written as an array rather than an initializer list: this image has no libstdc++ headers,
     * which is a property of how the product is configured and not an inconvenience to work round. */
    const prog::activity others[]{prog::activity::acquisition, prog::activity::send_acq,
                                  prog::activity::send_workq, prog::activity::health};
    for (const prog::activity a : others) {
        zassert_equal(after.at[idx(a)].begun, before.at[idx(a)].begun, "leaked into a neighbour");
        zassert_equal(after.at[idx(a)].ended, before.at[idx(a)].ended);
    }
}

/* The free-running loop has one counter because an iteration has no inside to be stuck in. */
ZTEST(tof_progress, test_the_zcan_loop_has_a_single_counter)
{
    const uint32_t before{prog::read().zcan_loops};
    prog::zcan_tick();
    prog::zcan_tick();
    zassert_equal(prog::read().zcan_loops, before + 2);
}

/* An out-of-range activity must not write past the array. It cannot happen through the enum, and
 * that is exactly the sort of thing that stops being true when somebody adds a seventh. */
ZTEST(tof_progress, test_an_out_of_range_activity_touches_nothing)
{
    const prog::snapshot before{prog::read()};
    prog::begin(prog::activity::count);
    prog::end(prog::activity::count);
    const prog::snapshot after{prog::read()};

    for (size_t i{0}; i < idx(prog::activity::count); ++i) {
        zassert_equal(after.at[i].begun, before.at[i].begun);
        zassert_equal(after.at[i].ended, before.at[i].ended);
    }
}

/* A read never reports more completions than starts. That ordering is what makes a torn read
 * harmless: the worst it can show is a pair that looks in-flight for one sample. */
ZTEST(tof_progress, test_a_read_never_shows_more_completions_than_starts)
{
    for (int i{0}; i < 200; ++i) {
        prog::begin(prog::activity::health);
        prog::end(prog::activity::health);
        const prog::snapshot s{prog::read()};
        for (size_t k{0}; k < idx(prog::activity::count); ++k)
            zassert_true(s.at[k].begun >= s.at[k].ended,
                         "ended is read first precisely so this cannot happen");
    }
}
