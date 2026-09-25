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

/* Every elapsed-time test in this file goes through here, so the millisecond counter's wrap is
 * handled in one place rather than at seven call sites. Unsigned subtraction is correct across the
 * wrap; writing `now > then + bound` would not be. */
bool longer_than(uint32_t now_ms, uint32_t since_ms, uint32_t bound_ms)
{
    return static_cast<uint32_t>(now_ms - since_ms) > bound_ms;
}

/* A counter moved. Equality, not ordering, because these wrap and a wrapped counter has not gone
 * backwards -- it has moved, which is the only thing being asked. */
bool moved(uint32_t now, uint32_t before)
{
    return now != before;
}

void note_progress(state &st, const input &in)
{
    if (moved(in.acquisition.ended, st.last_acq.ended))
        st.acq_seen_ms = in.now_ms;
    if (moved(in.can_send.ended, st.last_send.ended))
        st.send_seen_ms = in.now_ms;
    if (moved(in.health.ended, st.last_health.ended))
        st.health_seen_ms = in.now_ms;
    if (moved(in.l7.ended, st.last_l7.ended))
        st.l7_seen_ms = in.now_ms;
    if (moved(in.zcan_loops, st.last_zcan))
        st.zcan_seen_ms = in.now_ms;

    st.last_acq = in.acquisition;
    st.last_send = in.can_send;
    st.last_health = in.health;
    st.last_l7 = in.l7;
    st.last_zcan = in.zcan_loops;
}

/* Has every watched activity been seen to finish something on THIS boot, and is nothing inside one
 * right now?
 *
 * The first half is what keeps a board waiting for its host off the reset path: a CAN sender that
 * has never been ACKed has an `ended` of zero, and a zero is not a baseline. The second half is
 * subtler -- a baseline taken while a cycle is in flight would make the first judgement after the
 * grace period about work that began before anybody was watching. */
bool baseline_ready(const input &in)
{
    if (in.acquisition.ended == 0 || in.can_send.ended == 0 || in.health.ended == 0 ||
        in.zcan_loops == 0)
        return false;
    if (in.l7_expected && in.l7.ended == 0)
        return false;
    if (in.acquisition.begun != in.acquisition.ended)
        return false;
    if (in.can_send.begun != in.can_send.ended)
        return false;
    if (in.health.begun != in.health.ended)
        return false;
    if (in.l7_expected && in.l7.begun != in.l7.ended)
        return false;
    return true;
}

}  // namespace

bool feed_allowed(state &st, const bounds &b, const input &in)
{
    /* THE LATCH, and it comes first so that nothing below can undo it. A watchdog that changes its
     * mind gives a board that hangs intermittently an unlimited number of chances to look healthy
     * between hangs, and the reset never happens. */
    if (st.current == phase::stopped)
        return false;

    note_progress(st, in);

    if (st.current == phase::waiting) {
        if (in.baseline_point && baseline_ready(in)) {
            st.current = phase::armed;
            st.armed_ms = in.now_ms;
            st.acq_seen_ms = in.now_ms;
            st.send_seen_ms = in.now_ms;
            st.health_seen_ms = in.now_ms;
            st.zcan_seen_ms = in.now_ms;
            st.l7_seen_ms = in.now_ms;
        }
        /* Judging nothing until there is something to judge against. */
        return true;
    }

    if (!longer_than(in.now_ms, st.armed_ms, b.grace_ms))
        return true;

    uint32_t why{0};

    /* A declared long operation suspends the per-activity bounds, because an ULD download or a
     * commissioning pass legitimately holds the chain far longer than a cycle. It does not suspend
     * itself: "the firmware said it was busy" is the shape of excuse a hang would offer, so the
     * declaration has a cap and blowing through it is its own reason to stop. */
    if (in.long_operation) {
        if (longer_than(in.now_ms, in.long_operation_began_ms, b.long_operation_ms))
            why |= long_operation_over;
    } else {
        /* `begun != ended` means something is inside the activity. An activity sitting idle between
         * cycles is not stuck, it is idle, and judging it would reset a board for being quiet. */
        if (in.acquisition.begun != in.acquisition.ended &&
            longer_than(in.now_ms, st.acq_seen_ms, b.cycle_ms))
            why |= stuck_cycle;
        if (in.can_send.begun != in.can_send.ended &&
            longer_than(in.now_ms, st.send_seen_ms, b.send_ms))
            why |= stuck_send;
        if (in.health.begun != in.health.ended &&
            longer_than(in.now_ms, st.health_seen_ms, b.health_ms))
            why |= stuck_health;
        if (in.l7_expected && in.l7.begun != in.l7.ended &&
            longer_than(in.now_ms, st.l7_seen_ms, b.l7_ms))
            why |= stuck_l7;

        /* The one activity judged on movement alone. There is no "inside" a zcan loop, so silence
         * is the whole symptom. */
        if (longer_than(in.now_ms, st.zcan_seen_ms, b.zcan_ms))
            why |= stuck_zcan;
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
    case stuck_send:          return "stuck_send";
    case stuck_health:        return "stuck_health";
    case stuck_zcan:          return "stuck_zcan";
    case stuck_l7:            return "stuck_l7";
    case long_operation_over: return "long_operation_over";
    }
    return "unknown";
}

}  // namespace lexxhard::tof_task_watchdog
