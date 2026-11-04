/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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
uint32_t judge(const progress &p, uint32_t now_ms, uint32_t begun_ms, uint32_t seen_ms,
               uint32_t in_flight_bound_ms, uint32_t silence_bound_ms, uint32_t in_flight_reason,
               uint32_t silent_reason)
{
    uint32_t why{0};
    /* MEASURED FROM WHEN THIS OPERATION STARTED, not from the last completion. Both clocks used to
     * be `seen_ms`, which asked "how long since anything finished" of a question that is about one
     * operation: a send in flight for a millisecond was declared stuck because the previous send
     * had completed three seconds earlier, which on a quiet bus is ordinary and after a
     * commissioning pause is certain. */
    if (p.begun != p.ended && longer_than(now_ms, begun_ms, in_flight_bound_ms))
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
    if (moved(in.l7_chain.ended, st.last_l7_chain.ended))
        st.l7_chain_seen_ms = in.now_ms;
    if (moved(in.l7_package.ended, st.last_l7_package.ended))
        st.l7_package_seen_ms = in.now_ms;
    if (moved(in.zcan_loops, st.last_zcan))
        st.zcan_seen_ms = in.now_ms;

    /* The in-flight clocks, stamped when `begun` moves. A counter that moved twice between two
     * samples still stamps now: whatever is inside now started no earlier than the previous
     * sample, so `now` is the earliest instant the evidence supports. */
    if (moved(in.acquisition.begun, st.last_acq.begun))
        st.acq_begun_ms = in.now_ms;
    if (moved(in.send_acq.begun, st.last_send_acq.begun))
        st.send_acq_begun_ms = in.now_ms;
    if (moved(in.send_workq.begun, st.last_send_workq.begun))
        st.send_workq_begun_ms = in.now_ms;
    if (moved(in.health.begun, st.last_health.begun))
        st.health_begun_ms = in.now_ms;
    if (moved(in.l7_chain.begun, st.last_l7_chain.begun))
        st.l7_chain_begun_ms = in.now_ms;
    if (moved(in.l7_package.begun, st.last_l7_package.begun))
        st.l7_package_begun_ms = in.now_ms;

    st.last_acq = in.acquisition;
    st.last_send_acq = in.send_acq;
    st.last_send_workq = in.send_workq;
    st.last_health = in.health;
    st.last_l7_chain = in.l7_chain;
    st.last_l7_package = in.l7_package;
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
    /* The baseline still requires every watched activity to have finished once, INCLUDING the
     * senders. That is not in tension with the host gate: the gate stops an armed board resetting
     * when the host goes away, and this requires the host to have been there once before arming at
     * all. A board that has never reached its host simply stays in `waiting`, which is the
     * documented behaviour and the one the post-DFU boot needs. */
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

/* ONE L7 POPULATION'S JUDGEMENT, so the chain's and the Package's cannot drift apart as the file is
 * edited. Everything that differs between them is a parameter: the counters, the two clocks, the
 * reason bits, and whether that population's own declaration is in force.
 *
 * `silence_bound_ms` is the caller's because the two phases ask different questions of it -- before
 * the baseline nothing periodic is expected, so silence is not askable and the caller passes
 * UINT32_MAX.
 *
 * WHEN A DECLARATION IS IN FORCE, both of this population's clocks are held at `now`. The judgement
 * above has already been computed and discarded, so holding them cannot hide a fault: the only
 * clocks that move are the ones whose reasons are not being asked. Without the hold, the first
 * sample after a declaration ends would measure the declaration instead of the work that followed
 * it. */
uint32_t judge_l7(const progress &p, const input &in, uint32_t &begun_ms, uint32_t &seen_ms,
                  const bounds &b, uint32_t silence_bound_ms, uint32_t in_flight_reason,
                  uint32_t silent_reason, bool declared, uint32_t suspends)
{
    uint32_t why{judge(p, in.now_ms, begun_ms, seen_ms, b.l7_ms, silence_bound_ms, in_flight_reason,
                       silent_reason)};
    if (declared) {
        why &= ~suspends;
        seen_ms = in.now_ms;
        begun_ms = in.now_ms;
    }
    return why;
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

    /* Each L7 monitor starts at its own population's first operation, wherever in the boot that
     * falls. From that instant an operation that never returns is a fault for THAT population,
     * which is the case the arming point exists to cover. They arm independently because the two
     * subsystems start independently -- and because one of them may not be in the image at all. */
    if (!st.l7_chain_watching && in.l7_chain_expected && in.l7_chain.begun != 0) {
        st.l7_chain_watching = true;
        st.l7_chain_seen_ms = in.now_ms;
        st.l7_chain_begun_ms = in.now_ms;
    }
    if (!st.l7_package_watching && in.l7_package_expected && in.l7_package.begun != 0) {
        st.l7_package_watching = true;
        st.l7_package_seen_ms = in.now_ms;
        st.l7_package_begun_ms = in.now_ms;
    }

    if (st.current == phase::waiting) {
        /* BEFORE THE BASELINE, ONE THING IS STILL JUDGED: a declared long operation that never
         * ends. The ordinary heartbeats are not, because nothing periodic is expected yet -- but
         * the operation this phase is mostly spent on, verifying the stored blob, runs here and
         * would otherwise be unbounded. An earlier version returned true from this phase before
         * reaching the cap, so a blob verification that wedged was fed forever, which is the exact
         * hang the subsystem exists to catch and the one place it could not. */
        if (in.long_operation_chain &&
            longer_than(in.now_ms, in.long_operation_chain_began_ms, b.long_operation_ms)) {
            st.why = long_operation_over;
            st.current = phase::stopped;
            return false;
        }
        /* The Package declaration is capped on its own extent. Judged separately so a record says
         * whose declaration ran away, and so one population's cap cannot be reset by the other's
         * declaration starting. */
        if (in.long_operation_package &&
            longer_than(in.now_ms, in.long_operation_package_began_ms, b.long_operation_ms)) {
            st.why = long_operation_over_package;
            st.current = phase::stopped;
            return false;
        }
        /* AND A FIRST L7 OPEN, because it is the one activity that can PREVENT the baseline it
         * would otherwise be judged after -- so waiting for the baseline to judge it is waiting
         * for something it is holding up.
         *
         * The production thread brings every sensor up before it runs a cycle, and the baseline
         * requires acquisition AND the acquisition sender to have completed once. An open that
         * never returns leaves both counters at zero for ever, baseline_ready() stays false, and
         * this phase answered "feed" on every sample -- so the L7 in-flight bound, which the
         * header offers as the thing that covers the two-second ULD download at open(), was never
         * reached. The whole point of arming the L7 monitor at the first open rather than at the
         * baseline was to cover this operation, and the judgement was on the wrong side of the
         * phase test.
         *
         * THE IN-FLIGHT HALF ONLY, which is what UINT32_MAX says: nothing periodic is expected
         * before the baseline, so "it has completed nothing" is the normal state here and silence
         * is not a question that can be asked yet.
         *
         * NOT EVERY ACTIVITY, and this does not close the class. A cliff open() that hangs is
         * invisible to all of this: nothing brackets it, and bring_up() is not inside the
         * acquisition pair either, so no counter moves and there is nothing to time. Bounding a
         * bring-up that reports no progress at all needs its own mechanism and is not this. */
        uint32_t l7_why{0};
        if (st.l7_chain_watching) {
            l7_why |= judge_l7(in.l7_chain, in, st.l7_chain_begun_ms, st.l7_chain_seen_ms, b,
                               UINT32_MAX, stuck_l7_chain, silent_l7_chain,
                               in.long_operation_chain, kChainLongOperationSuspends);
        }
        /* EACH POPULATION IS COVERED ONLY BY ITS OWN DECLARATION. This is the shape the Package
         * blob download needs: it declares, and the ordinary bound stops applying to ITS operation
         * while the chain's keeps applying to the chain's. */
        if (st.l7_package_watching) {
            l7_why |= judge_l7(in.l7_package, in, st.l7_package_begun_ms, st.l7_package_seen_ms, b,
                               UINT32_MAX, stuck_l7_package, silent_l7_package,
                               in.long_operation_package, kPackageLongOperationSuspends);
        }
        if (l7_why != 0) {
            st.why = l7_why;
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
            /* The in-flight clocks start here as well. baseline_ready() has already required that
             * nothing is inside any of these, so there is nothing to time from earlier. */
            st.acq_begun_ms = in.now_ms;
            st.send_acq_begun_ms = in.now_ms;
            st.send_workq_begun_ms = in.now_ms;
            st.health_begun_ms = in.now_ms;
        }
        return true;
    }

    if (!longer_than(in.now_ms, st.armed_ms, b.grace_ms))
        return true;

    uint32_t why{0};

    why |= judge(in.acquisition, in.now_ms, st.acq_begun_ms, st.acq_seen_ms, b.cycle_ms,
                 b.cycle_silence_ms, stuck_cycle, silent_cycle);
    why |= judge(in.send_acq, in.now_ms, st.send_acq_begun_ms, st.send_acq_seen_ms, b.send_ms,
                 b.send_silence_ms, stuck_send_acq, silent_send_acq);
    why |= judge(in.send_workq, in.now_ms, st.send_workq_begun_ms, st.send_workq_seen_ms, b.send_ms,
                 b.send_silence_ms, stuck_send_workq, silent_send_workq);
    why |= judge(in.health, in.now_ms, st.health_begun_ms, st.health_seen_ms, b.health_ms,
                 b.health_silence_ms, stuck_health, silent_health);

    /* The silence half applies only once that population has COMPLETED an operation. Before that
     * the only thing it has done is the open that is still running, and the in-flight bound is what
     * judges it -- a silence reason there would say "it has completed nothing" about an operation
     * that has not had a chance to. Asked per population, because one of them completing says
     * nothing about the other.
     *
     * The declaration is applied here rather than in the block below, so that each population's
     * exemption is decided with its own judgement rather than after both have been merged. */
    if (st.l7_chain_watching) {
        why |= judge_l7(in.l7_chain, in, st.l7_chain_begun_ms, st.l7_chain_seen_ms, b,
                        in.l7_chain.ended != 0 ? b.l7_silence_ms : UINT32_MAX, stuck_l7_chain,
                        silent_l7_chain, in.long_operation_chain, kChainLongOperationSuspends);
    }
    if (st.l7_package_watching) {
        why |= judge_l7(in.l7_package, in, st.l7_package_begun_ms, st.l7_package_seen_ms, b,
                        in.l7_package.ended != 0 ? b.l7_silence_ms : UINT32_MAX, stuck_l7_package,
                        silent_l7_package, in.long_operation_package,
                        kPackageLongOperationSuspends);

        /* A PACKAGE THAT HAS STOPPED ACQUIRING IS NOT A PACKAGE THAT DIED. report_only makes no
         * further ULD calls by design, so its completions stop while the thread keeps publishing
         * diagnostics -- and before the two L7 populations were split, the chain's operations on
         * the shared slot hid that. Now it has to be said rather than inferred.
         *
         * The silence clock is held at `now`, every sample, for the same reason the stopped
         * acquisition holds its clocks: nothing moves while the activity is stopped, so a clock
         * left standing would make the first sample after a resumption read the whole pause as a
         * gap. The judgement above has already been computed and discarded, so this cannot hide a
         * fault -- the only clock that moves is the one whose question is not being asked. */
        if (!in.l7_package_acquiring) {
            why &= ~kPackageStoppedSuspends;
            st.l7_package_seen_ms = in.now_ms;
        }
    }

    /* No inside to be stuck in, so silence is the whole symptom. */
    if (longer_than(in.now_ms, st.zcan_seen_ms, b.zcan_silence_ms))
        why |= silent_zcan;

    /* A STOPPED ACQUISITION, which is what a commissioning pass produces -- and what a FAILED proof
     * leaves behind, with no end-of-operation to wait for.
     *
     * MASKING THE REASON IS NOT ENOUGH ON ITS OWN, and the previous comment here claimed otherwise:
     * it said note_progress() keeps the bookkeeping, so "an activity that resumes is seen to
     * resume". note_progress() only stamps a clock when a counter MOVES, and nothing moves while
     * the activity is stopped -- so the clocks stood still through the pause while `now_ms` did
     * not. The pause outlasts the silence bound by design, which meant the first sample after
     * acquisition came back judged a gap the mask had been hiding all along, and latched
     * silent_cycle before the resumed thread could possibly have completed anything.
     *
     * So a suspended clock is held at `now`, every sample, for exactly the halves the mask covers.
     * The judgement above has already been computed and discarded, so this cannot hide a fault:
     * the only reasons whose clocks move here are the ones that are not being asked. When the
     * suspension lifts the clocks read "last seen now", and the bound then measures the resumed
     * activity rather than the pause. */
    if (!in.acquisition_expected) {
        why &= ~kAcquisitionStoppedSuspends;
        st.acq_seen_ms = in.now_ms;
        st.acq_begun_ms = in.now_ms;
        st.send_acq_seen_ms = in.now_ms;
        st.send_acq_begun_ms = in.now_ms;
        /* THE CHAIN'S SILENCE CLOCK ONLY, on both counts.
         *
         * Only the silence half, because stuck_l7_chain is deliberately not in the mask -- an
         * operation already in flight when acquisition stopped is still in flight and is still a
         * hang -- so holding its in-flight clock would suspend a judgement still being asked.
         *
         * And only the chain's, because acquisition is what stops asking the CHAIN's sensors for
         * grids. The Package sensor is on another bus, driven by its own thread, and a stopped
         * acquisition says nothing about whether it is still working. */
        st.l7_chain_seen_ms = in.now_ms;
    }

    /* A CHAIN DECLARATION HOLDS ACQUISITION, because `tof cliff prove` and the chain's blob
     * bootstrap both quiesce it. Its own L7 exemption was applied with its judgement above; what is
     * left here is the acquisition half, which no judgement of a single population could clear.
     *
     * It says nothing about whether the heartbeat is still going out or the zcan loop is still
     * turning, and an earlier version of this file suspended everything, which bought thirty
     * seconds of silence for tasks the operation never touched. */
    if (in.long_operation_chain) {
        why &= ~kChainLongOperationSuspends;

        /* Held forward for the same reason as above: without it the first sample after the
         * declaration would measure the declaration rather than the work that followed it. The
         * cap below is what bounds this; holding a clock is not a licence to run forever. */
        st.acq_seen_ms = in.now_ms;
        st.acq_begun_ms = in.now_ms;

        /* The declaration does not suspend itself. "The firmware said it was busy" is the shape of
         * excuse a hang would offer, so it carries a cap. */
        if (longer_than(in.now_ms, in.long_operation_chain_began_ms, b.long_operation_ms))
            why |= long_operation_over;
    }

    /* A PACKAGE DECLARATION SUSPENDS NOTHING BEYOND ITS OWN L7, which judge_l7() has already done.
     * It owns one sensor on i2c4 and its own thread, so it cannot stop the chain's acquisition and
     * must not buy that silence -- the single shared mask did, which left a wedged chain exempt for
     * the whole of an eight-second blob download that was going perfectly.
     *
     * So all that is left is its cap, on its own extent. */
    if (in.long_operation_package &&
        longer_than(in.now_ms, in.long_operation_package_began_ms, b.long_operation_ms))
        why |= long_operation_over_package;

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
    case stuck_l7_chain:      return "stuck_l7_chain";
    case silent_l7_chain:     return "silent_l7_chain";
    case stuck_l7_package:    return "stuck_l7_package";
    case silent_l7_package:   return "silent_l7_package";
    case silent_zcan:         return "silent_zcan";
    case long_operation_over: return "long_operation_over";
    case long_operation_over_package: return "long_operation_over_package";
    case feed_api_failed:     return "feed_api_failed";
    }
    return "unknown";
}

}  // namespace lexxhard::tof_task_watchdog
