/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * See tof_task_watchdog.hpp.
 */

#include "tof_task_watchdog.hpp"

namespace lexxhard::tof_task_watchdog {

namespace {

/* Every elapsed-time test goes through here, so the millisecond counter's wrap is handled once
 * rather than at a dozen call sites. Unsigned subtraction is correct across the wrap; writing
 * `now > then + bound` would not be. */
bool longer_than(uint32_t now_ms, uint32_t since_ms, uint32_t bound_ms)
{
    return static_cast<uint32_t>(now_ms - since_ms) > bound_ms;
}

/* Equality, not ordering, because these wrap and a wrapped counter has not gone backwards -- it has
 * moved, which is the only thing being asked. */
bool moved(uint32_t now, uint32_t before)
{
    return now != before;
}

/* The two questions asked of every periodic activity, kept together so that adding one activity
 * cannot accidentally answer only one of them.
 *
 * `in_flight` is a task that went in and did not come out. `silent` is a task that stopped running
 * at all -- the failure an in-flight test cannot see, because a thread that dies between two cycles
 * leaves begun == ended and looks exactly like one that is idle. */
uint32_t judge(const progress &p, uint32_t now_ms, uint32_t seen_ms, uint32_t in_flight_bound_ms,
               uint32_t silence_bound_ms, uint32_t in_flight_reason, uint32_t silent_reason)
{
    uint32_t why{0};
    if (p.begun != p.ended && longer_than(now_ms, seen_ms, in_flight_bound_ms))
        why |= in_flight_reason;
    if (longer_than(now_ms, seen_ms, silence_bound_ms))
        why |= silent_reason;
    return why;
}

void note_progress(state &st, const input &in)
{
    if (moved(in.acquisition.ended, st.last_acq.ended))
        st.acq_seen_ms = in.now_ms;
    if (moved(in.send_acq.ended, st.last_send_acq.ended))
        st.send_acq_seen_ms = in.now_ms;
    if (moved(in.send_workq.ended, st.last_send_workq.ended))
        st.send_workq_seen_ms = in.now_ms;
    if (moved(in.health.ended, st.last_health.ended))
        st.health_seen_ms = in.now_ms;
    if (moved(in.l7.ended, st.last_l7.ended))
        st.l7_seen_ms = in.now_ms;
    if (moved(in.zcan_loops, st.last_zcan))
        st.zcan_seen_ms = in.now_ms;

    st.last_acq = in.acquisition;
    st.last_send_acq = in.send_acq;
    st.last_send_workq = in.send_workq;
    st.last_health = in.health;
    st.last_l7 = in.l7;
    st.last_zcan = in.zcan_loops;
}

/* Has every activity that CAN have finished something on this boot done so, and is nothing inside
 * one right now?
 *
 * The L7 is deliberately absent from this. The baseline is taken after commissioning and before any
 * L7 has been opened, so at that moment an L7 has necessarily completed nothing; requiring
 * otherwise would make the documented arming point unreachable and was a real contradiction in the
 * first version of this file. The L7 monitor starts later, at its first open.
 *
 * The rest is what keeps a board waiting for its host off the reset path: a CAN sender that has
 * never been ACKed has an `ended` of zero, and a zero is not a baseline. The in-flight half is
 * subtler -- a baseline taken while a cycle is inside would make the first judgement after the
 * grace period about work that began before anybody was watching. */
bool baseline_ready(const input &in)
{
    if (in.acquisition.ended == 0 || in.send_acq.ended == 0 || in.send_workq.ended == 0 ||
        in.health.ended == 0 || in.zcan_loops == 0)
        return false;
    if (in.acquisition.begun != in.acquisition.ended)
        return false;
    if (in.send_acq.begun != in.send_acq.ended)
        return false;
    if (in.send_workq.begun != in.send_workq.ended)
        return false;
    if (in.health.begun != in.health.ended)
        return false;
    return true;
}

}  // namespace

bool feed_allowed(state &st, const bounds &b, const input &in)
{
    /* THE LATCH, first so nothing below can undo it. A watchdog that changes its mind gives a board
     * that hangs intermittently an unlimited number of chances to look healthy between hangs, and
     * the reset never happens. */
    if (st.current == phase::stopped)
        return false;

    note_progress(st, in);

    /* The L7 monitor starts at the first open, wherever in the boot that falls. From this instant an
     * open that never returns is a fault, which is the case the arming point exists to cover. */
    if (!st.l7_watching && in.l7_expected && in.l7.begun != 0) {
        st.l7_watching = true;
        st.l7_seen_ms = in.now_ms;
    }

    if (st.current == phase::waiting) {
        /* BEFORE THE BASELINE, ONE THING IS STILL JUDGED: a declared long operation that never
         * ends. The ordinary heartbeats are not, because nothing periodic is expected yet -- but
         * the operation this phase is mostly spent on, verifying the stored blob, runs here and
         * would otherwise be unbounded. An earlier version returned true from this phase before
         * reaching the cap, so a blob verification that wedged was fed forever, which is the exact
         * hang the subsystem exists to catch and the one place it could not. */
        if (in.long_operation &&
            longer_than(in.now_ms, in.long_operation_began_ms, b.long_operation_ms)) {
            st.why = long_operation_over;
            st.current = phase::stopped;
            return false;
        }
        if (in.baseline_point && baseline_ready(in)) {
            st.current = phase::armed;
            st.armed_ms = in.now_ms;
            st.acq_seen_ms = in.now_ms;
            st.send_acq_seen_ms = in.now_ms;
            st.send_workq_seen_ms = in.now_ms;
            st.health_seen_ms = in.now_ms;
            st.zcan_seen_ms = in.now_ms;
        }
        return true;
    }

    if (!longer_than(in.now_ms, st.armed_ms, b.grace_ms))
        return true;

    uint32_t why{0};

    why |= judge(in.acquisition, in.now_ms, st.acq_seen_ms, b.cycle_ms, b.cycle_silence_ms,
                 stuck_cycle, silent_cycle);
    why |= judge(in.send_acq, in.now_ms, st.send_acq_seen_ms, b.send_ms, b.send_silence_ms,
                 stuck_send_acq, silent_send_acq);
    why |= judge(in.send_workq, in.now_ms, st.send_workq_seen_ms, b.send_ms, b.send_silence_ms,
                 stuck_send_workq, silent_send_workq);
    why |= judge(in.health, in.now_ms, st.health_seen_ms, b.health_ms, b.health_silence_ms,
                 stuck_health, silent_health);

    if (st.l7_watching) {
        /* The silence half applies only once an L7 operation has completed. Before that the only
         * thing the L7 has done is the open that is still running, and the in-flight bound is what
         * judges it -- a silence reason there would say "it has completed nothing" about an
         * operation that has not had a chance to. */
        why |= judge(in.l7, in.now_ms, st.l7_seen_ms, b.l7_ms,
                     in.l7.ended != 0 ? b.l7_silence_ms : UINT32_MAX, stuck_l7, silent_l7);
    }

    /* No inside to be stuck in, so silence is the whole symptom. */
    if (longer_than(in.now_ms, st.zcan_seen_ms, b.zcan_silence_ms))
        why |= silent_zcan;

    if (in.long_operation) {
        /* A declared operation holds the chain, so it holds acquisition and the L7 -- and NOTHING
         * else. It says nothing about whether the heartbeat is still going out or the zcan loop is
         * still turning, and an earlier version of this file suspended everything, which bought
         * thirty seconds of silence for tasks the operation never touched. */
        why &= ~kLongOperationSuspends;

        /* The declaration does not suspend itself. "The firmware said it was busy" is the shape of
         * excuse a hang would offer, so it carries a cap, and the first L7 open never returning is
         * that cap firing. */
        if (longer_than(in.now_ms, in.long_operation_began_ms, b.long_operation_ms))
            why |= long_operation_over;
    }

    if (why == 0)
        return true;

    st.why = why;
    st.current = phase::stopped;
    return false;
}

const char *reason_name(reason r)
{
    switch (r) {
    case stuck_cycle:         return "stuck_cycle";
    case silent_cycle:        return "silent_cycle";
    case stuck_send_acq:      return "stuck_send_acq";
    case silent_send_acq:     return "silent_send_acq";
    case stuck_send_workq:    return "stuck_send_workq";
    case silent_send_workq:   return "silent_send_workq";
    case stuck_health:        return "stuck_health";
    case silent_health:       return "silent_health";
    case stuck_l7:            return "stuck_l7";
    case silent_l7:           return "silent_l7";
    case silent_zcan:         return "silent_zcan";
    case long_operation_over: return "long_operation_over";
    case feed_api_failed:     return "feed_api_failed";
    }
    return "unknown";
}

}  // namespace lexxhard::tof_task_watchdog
