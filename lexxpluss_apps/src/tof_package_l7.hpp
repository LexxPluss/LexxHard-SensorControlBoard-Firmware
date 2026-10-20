/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The Package ToF lifecycle: ONE VL53L7CX, on its own bus, with no chain around it.
 *
 * WHY THIS IS NOT THE CHAIN'S LIFECYCLE. The hanging-object path has an enumerator and a recovery
 * pass: it walks the enable chain's positions, and an address that cannot be classified is one
 * finding among several that the pass reports and the chain can route around. The Package sensor
 * has none of that. It is a single device with no LPn, no comms gating and no reset line, so there
 * is nothing to fall back to and nothing to compare an ambiguous answer against.
 *
 * That shapes the one rule this module exists to enforce: A PROBE THAT COULD NOT TELL IS NOT A
 * PROBE THAT FOUND NOBODY, and it must not be followed by an open. Opening on top of a device that
 * may already be ranging is how a session gets stacked on an unknown one, and here the only way
 * out of that is a machine power cycle.
 *
 * Pure logic with injected operations, for the same reason the recovery pass is: the ordering
 * rules are the product, they are the part that can be got wrong silently, and they are testable
 * on a host only if nothing in them touches a bus.
 */

#pragma once

#if defined(ENABLE_TOF_L7_ULD)

#include <stddef.h>
#include <stdint.h>

namespace lexxhard::tof_package {

enum class state : uint8_t {
    /* Nothing is open, and that is accounted for: either nothing has been attempted yet or a stop
     * was acknowledged. The only state an open may start from. */
    idle,
    ranging,
    /* A stop was issued and not acknowledged. Reads are refused -- a session nobody can account
     * for must not have its data trusted -- and so is any reopen, because opening would forget a
     * device that may still be measuring. Retrying the stop is the only way out, and it is allowed
     * until the budget runs out. Mirrors tof_l7::lifecycle::stop_unconfirmed, which is where the
     * same rule is enforced one layer down. */
    stop_owed,
    /* Reported and not retried here. Everything that reaches this state needs a decision outside
     * this module -- a service action, a power cycle, a requirements answer -- and retrying would
     * only hide which one. */
    blocked,
};

/* Which step produced the current state. Carried so a caller can report WHY the sensor is
 * unavailable rather than just that it is: the operator-visible health requirement in the design
 * turns on this distinction. */
enum class step : uint8_t {
    none,
    firmware,   // the image-wide blob verification did not say the payload is usable
    probe,      // the probe did not complete, so presence is unknown
    identity,   // something answered at the address and is not a VL53L7CX
    survivor,   // something answered and could not be quiesced
    close,
    open,
    configure,
    start,
    read,
    stop,
};

struct hooks {
    /* The image-wide verification, asked rather than performed: one payload, one record, one
     * accept-list. 0 means the firmware is readable now. */
    int (*ensure_verified)(){nullptr};
    /* On rc=0, report both present and wrong_identity. Nonzero makes both invalid.
     * A responding non-L7 device at the fixed address must stop the round before
     * quiesce/open; do not collapse it into no response.
     */
    int (*probe)(void *ctx, uint8_t addr_7bit, bool *present, bool *wrong_identity){nullptr};
    /* Stop a device that answered before this run began. */
    int (*quiesce)(void *ctx, uint8_t addr_7bit){nullptr};
    /* Returns the sensor object to a state an open may start from. Called before EVERY open, not
     * only after a failure: a confirmed stop leaves the adapter `configured`, and tof_l7::open()
     * requires `empty`, so without this an ordinary up -> stop -> up sequence fails on the second
     * open. Must succeed and do nothing on an already-empty sensor. */
    int (*close)(void *ctx){nullptr};
    /* NEITHER open NOR configure MAY START A MEASUREMENT. That is what makes a failure in either of
     * them recoverable without a power cycle, and discardable() depends on it. Only start() begins
     * ranging, and its failure is a stop debt rather than a dead end for exactly that reason. */
    int (*open)(void *ctx, uint8_t addr_7bit){nullptr};
    int (*configure)(void *ctx, uint8_t frequency_hz){nullptr};
    int (*start)(void *ctx){nullptr};
    /* Sets *fresh when a complete grid was available. Returning 0 with *fresh false is a normal
     * not-ready poll, not a failure. */
    int (*read)(void *ctx, bool *fresh){nullptr};
    int (*stop)(void *ctx){nullptr};
    void *ctx{nullptr};
};

struct config {
    uint8_t address_7bit{0x29};
    uint8_t frequency_hz{0};
    /* Total stop attempts, the first included. Zero is refused: a lifecycle that may never stop
     * has no way back to idle, and the one state this module must always be able to leave is
     * stop_owed. */
    uint8_t stop_attempts{3};
};

/* Record the refusal reason explicitly: -EPERM alone cannot distinguish
 * uninitialized, wrong state and exhausted budget. In particular, blocked at
 * an earlier step does not mean the device was asked to stop.
 */
enum class stop_refusal : uint8_t {
    none = 0,
    /* init() has not run, or ran and failed. */
    not_initialised,
    /* Nothing owes a stop: the state is neither `ranging` nor `stop_owed`. `blocked` from any
     * refusal EARLIER than a stop lands here, and no device call is made. */
    wrong_state,
    /* The state guard currently precedes the budget guard: the attempt that
     * exhausts the budget enters blocked, so later calls report wrong_state.
     * Keep that order; idle after a successful final attempt owes no stop.
     * Use blocked + failed_step==stop to identify an exhausted device stop,
     * and an earlier failed_step to identify a stop never attempted.
     */
    budget_spent,
};

/* Derive stop advice from recorded state and remaining attempts.
 * A final failed attempt needs a non-retryable answer immediately, before
 * the next call is refused by the state guard.
 */
struct stop_report {
    /* True only when another stop() can still reach the device. */
    bool retryable{false};
    /* Static text; the caller prints the state and step separately. */
    const char *text{nullptr};
};

struct status {
    state current{state::idle};
    step failed_step{step::none};
    int errno_value{0};
    /* True only when the probe said something answered BEFORE this run opened anything. Recorded
     * because it is the warm-reset signal, and because a run that found one and continued has to be
     * able to say so. */
    bool survivor_found{false};
    uint8_t stop_attempts_used{0};
    /* Set by every stop() that refuses, cleared by one that succeeds. Never inferred from
     * `current`. */
    stop_refusal last_stop_refusal{stop_refusal::none};
    /* True when the round now in progress, or the last one attempted, was entered by
     * take_over_unknown_state() rather than by bring_up(). Recorded because the two rounds are not
     * the same experiment: one established that nothing was ranging before it opened, the other was
     * authorised to proceed without establishing it, and a reading of any later result has to know
     * which. */
    bool entered_by_takeover{false};
};

/* Only called after a stop() that returned non-zero. */
stop_report report_for(const status &s);

class lifecycle {
public:
    int init(const hooks &h, const config &c);

    /* ensure_verified -> probe -> quiesce a survivor -> open -> configure -> start.
     *
     * Refuses unless the state is idle. Any failure leaves a state and a step that say where it
     * stopped, and no later step is attempted -- in particular an unclassifiable probe stops the
     * run instead of being treated as an empty address. */
    int bring_up();

    /* Explicitly authorize initialization of a device whose state is unknown.
     * Skip only quiesce; keep blob verification, probe/identity refusal, close,
     * full init/configure/start, and normal failure states/stop debt.
     * This is not proof of power loss or a retry bypass: idle means only that
     * software holds no session. Failure does not reset the state for another try.
     * Require initialized + idle. Other states return -EPERM without hardware
     * access or changes to recorded state/evidence.
     */
    int take_over_unknown_state();

    /* The guard used by bring_up() itself: initialized and idle, with no side effects.
     * Call it before clearing per-round records; a refused round preserves evidence.
     * Acceptance does not imply success, and verification refusal still belongs to
     * the accepted round. Calls are serialized, so the guard cannot race the entry.
     */
    bool can_begin_round() const;

    /* One poll. Refused unless ranging. */
    int read_once(bool *fresh);

    /* Returns to idle on an acknowledged stop. On a failure the state becomes stop_owed and this
     * may be called again; once the attempt budget is spent the state becomes blocked. */
    int stop();

    status current() const { return status_; }

    /* Ask before rebind/reinitialization. Refuse disposal while ranging, owing stop,
     * or blocked behind a potentially live session. Idle and safe pre-ranging
     * failures are discardable; only confirmed stop clears a stop debt.
     */
    bool discardable() const;

private:
    /* The body both entries share. `quiesce_a_survivor` is the whole difference between them, and
     * it is a parameter rather than two copies because every other step, state and refusal has to
     * stay identical by construction -- a second copy is how the two would drift. */
    int begin_round(bool quiesce_a_survivor);

    int refuse(step s, int rc);

    hooks hooks_{};
    config config_{};
    status status_{};
    bool initialised_{false};
};

const char *state_name(state s);
const char *step_name(step s);

}  // namespace lexxhard::tof_package

#endif  // ENABLE_TOF_L7_ULD
