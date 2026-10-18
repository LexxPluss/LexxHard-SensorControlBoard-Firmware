/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

/*
 * WHAT AN UNATTENDED PACKAGE ToF RUNTIME DOES NEXT, as a function of what it can see.
 *
 * The bench drives this sensor from a shell, where a human reads each refusal and decides. A
 * runtime has nobody to ask, so the deciding has to be written down -- and written down HERE,
 * apart from the thread, because every rule in it is a rule about sequence rather than about
 * hardware. Nothing in this file touches a device, a bus, a clock or Zephyr, so each rule is a
 * case rather than a claim about a board.
 *
 * THE THREE THINGS IT IS FOR:
 *
 *   - Getting the sensor measuring at boot WITHOUT a human typing `bind`, `takeover` and `stream`.
 *   - Not hammering the hardware when it cannot. Every claim on the device is counted against a
 *     budget, and when the budget is gone the runtime keeps REPORTING and stops TOUCHING.
 *   - Keeping the stop defence that the bench has. `bring_up()` stops a device that answered
 *     before opening a second session on it, which is the one ordering that protects a sensor
 *     this board cannot power-cycle. The runtime attempts that first and takes over only when the
 *     quiesce is what failed, which is the same judgement a human made at the bench on 2026-10-12.
 *
 * WHY A TAKEOVER IS BUDGETED RATHER THAN FORBIDDEN OR FREE. `take_over_unknown_state()` skips the
 * quiesce, and the lifecycle's own header says what that means: `idle` says the software holds no
 * session, never that the sensor stopped, and nothing establishes that the device lost power. At
 * the bench that authorisation is a human typing a command. Here the authorisation is the build
 * flag plus this budget: one takeover per boot by default, so an unattended board may get itself
 * measuring once and may not spend the night re-initialising a sensor that keeps failing.
 *
 * WHAT IT DOES NOT DECIDE: the rate, the identifiers, whether to publish, what a grid means, or
 * when a stream is stale. Those belong to the configuration, the transport and
 * tof_package_stream's publication rule.
 */

#include <stdint.h>

#if defined(ENABLE_TOF_L7_ULD)

#include "tof_package_l7.hpp"

namespace lexxhard::tof_package_auto {

/* What the runtime should do on this pass. Exactly one, and the thread does only that: a pass that
 * both claimed the device and read from it would leave no observation between the two. */
enum class action : uint8_t {
    /* No lifecycle is bound to the bus. Binding opens nothing and starts no measurement. */
    bind,
    /* bring_up(): verify, probe, STOP A SURVIVOR, open, configure, start. */
    claim_with_quiesce,
    /* take_over_unknown_state(): the same without the quiesce, from idle only. Authorised by the
     * budget, spent whether or not it succeeds. */
    claim_by_takeover,
    /* Discard a lifecycle that is blocked in a state no measurement can be behind, so a later
     * claim can start from idle. The lifecycle's own discardable() is what says this is allowed;
     * this policy never second-guesses it. */
    rebind_to_retry,
    /* Read once and publish whatever happened, including nothing. */
    acquire,
    /* A stop is owed. Nothing may be read until it is paid, and the lifecycle refuses anyway. */
    pay_stop,
    /* NOTHING LEFT TO TRY WITHOUT A HUMAN. The runtime keeps publishing diagnostics -- a consumer
     * must be able to tell a dead producer from a silent one -- and touches no hardware at all.
     * This is a destination, not a pause: nothing in this policy leaves it, because everything
     * that could has already been spent. */
    report_only,
};

const char *action_name(action a);

/* What the runtime can see when it asks. Every field is read from the lifecycle except `bound`,
 * which only the device layer knows. */
struct observation {
    bool bound{false};
    tof_package::state current{tof_package::state::idle};
    /* Where the last claim stopped. Meaningful when `current` is blocked; ignored otherwise. */
    tof_package::step failed_step{tof_package::step::none};
    /* The lifecycle's own answer about whether this state may be discarded. Taken rather than
     * derived: it is the layer that knows which blocked steps a live session can be behind. */
    bool discardable{false};
};

struct limits {
    /* Claims on the device per boot, takeovers included. Zero is refused: a runtime that may never
     * claim the device is a thread that exists to do nothing. */
    uint8_t claims{6};
    /* Takeovers per boot, which must not exceed `claims`. Zero is legal and means the quiesce is
     * never skipped -- the stop defence absolute, and a device that cannot be quiesced is reported
     * rather than taken over. */
    uint8_t takeovers{1};
    /* Whether the FIRST claim of a boot attempts the quiesce. True keeps the stop defence in front
     * of every takeover, at the cost of the quiesce's full polling budget on a device that cannot
     * answer it -- measured at 5.43 s on this geometry on 2026-10-12. Set false only where that
     * failure has been OBSERVED on the board being configured, and never as a default: skipping
     * straight to a takeover on an unexamined board is the one thing the bench's ordering exists
     * to prevent. */
    bool quiesce_first{true};
};

/* WHAT ONE ACQUISITION PASS OWES THE BUS.
 *
 * Separate from the action above because it answers a different question at a different rate: the
 * action decides what to do to the DEVICE, this decides what to say about it, and a pass that
 * reads the sensor still has to choose between a grid, a diagnostic and silence.
 *
 * IT IS HERE BECAUSE IT WAS GOT WRONG IN THE THREAD. The first runtime passed a flag meaning "this
 * pass was an acquisition" where it needed one meaning "this is a grid", so a read that FAILED
 * took the grid's path and bypassed the diagnostic's rate limit -- one frame every poll interval,
 * reporting one unchanged condition. The same confusion left the other half silent: a sensor that
 * kept answering "nothing ready yet" published nothing at all after the first diagnostic, which on
 * the bus is indistinguishable from a producer that died. Both are rules about cadence, so they
 * are a function of the clock and nothing else, and they live where they can be tested. */
enum class report : uint8_t {
    /* A complete grid is in hand. Grids are never rate limited: the rate is the sensor's. */
    publish_grid,
    /* One diagnostic, because the acquisition failed or because no grid has arrived for longer
     * than a gap between grids. Rate limited, since both conditions persist. */
    publish_diagnostic,
    /* Nothing. Either a readiness poll that simply found nothing yet -- which is not an event --
     * or a condition already reported inside the rate limit. */
    say_nothing,
};

const char *report_name(report r);

/* One pass, as the thread saw it. The two timestamps are the thread's, because a clock is the one
 * thing this file will not own. */
struct pass {
    /* What read_once() returned. Non-zero is a transport or device failure. */
    int read_rc{0};
    /* Whether that read produced a complete grid. Only meaningful when read_rc is 0. */
    bool fresh{false};
    int64_t now_ms{0};
    /* When a grid was last in hand, or when the runtime last began expecting one -- a claim that
     * succeeds restarts the wait rather than inheriting the silence before it. */
    int64_t last_grid_ms{0};
    int64_t last_diagnostic_ms{0};
};

struct cadence {
    /* The floor between two diagnostics. A consumer cannot tell a dead producer from a silent one,
     * so a failing runtime keeps reporting -- on a cadence, not per pass. */
    int64_t diagnostic_interval_ms{500};
    /* How long without a grid stops being the ordinary gap between two and becomes a stall. Below
     * this, a readiness poll that found nothing publishes nothing: the sensor is working and the
     * grid has not landed yet, and a diagnostic for that would report a non-event. */
    int64_t grid_overdue_ms{500};
};

/* Pure, and no state of its own: the caller keeps the timestamps because the caller owns the
 * clock. A non-positive interval in `c` means "no floor" and a non-positive overdue means "every
 * poll without a grid is overdue" -- both are honoured rather than corrected, because a
 * configuration that asked for them is making a choice this file cannot second-guess. */
report what_to_send(const pass &p, const cadence &c);

struct counters {
    uint8_t claims_used{0};
    uint8_t takeovers_used{0};
    /* Whether a claim has already failed at the quiesce this boot. This is what makes the SECOND
     * claim a takeover rather than a repeat of the attempt that just failed. */
    bool quiesce_refused{false};
    action last{action::bind};
};

class policy {
public:
    /* -EINVAL on claims == 0 or takeovers > claims. Resets the counters. */
    int init(const limits &l);

    /* The action for this pass, and the budget it spends. Returns report_only until init()
     * succeeds, because a policy that was never configured has no budget to spend and must not
     * invent one. */
    action next(const observation &o);

    counters current() const { return counters_; }

private:
    limits limits_{};
    counters counters_{};
    bool initialised_{false};
};

}  // namespace lexxhard::tof_package_auto

#endif  // ENABLE_TOF_L7_ULD
