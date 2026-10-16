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
    /* Sets *present and *wrong_identity. Returns 0 when the probe itself completed, whatever its
     * verdict; non-zero is a transport error and both flags are then meaningless.
     *
     * THE THIRD OUTCOME IS NOT OPTIONAL HERE. `*present == false` with `*wrong_identity == true`
     * means something answered and is not a VL53L7CX, which on this sensor must stop the run: the
     * address is fixed, the device is always powered, and there is no enumerator to hand the
     * anomaly to. Collapsing it into "nobody there" would be followed by an open onto a device
     * nobody has identified. */
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

/* WHY A STOP WAS REFUSED, reported rather than inferred.
 *
 * `stop()` returns -EPERM from three unrelated places -- not initialised, a state that owes no
 * stop, and a spent retry budget -- and the shell used to tell them apart by looking at the state
 * afterwards, reading `blocked` as "the budget is spent". On 2026-10-12 that printed "the retry
 * budget is spent. The device may still be measuring" for a run whose budget was untouched: the
 * state was `blocked` because bring_up had refused at the survivor step, so `stop()` returned at
 * its state guard without calling the device at all. The message named the wrong cause AND implied
 * a device interaction that never happened.
 *
 * A state is not a reason. Three refusals that share a return value have to say which one they
 * were. */
enum class stop_refusal : uint8_t {
    none = 0,
    /* init() has not run, or ran and failed. */
    not_initialised,
    /* Nothing owes a stop: the state is neither `ranging` nor `stop_owed`. `blocked` from any
     * refusal EARLIER than a stop lands here, and no device call is made. */
    wrong_state,
    /* The device was asked and did not acknowledge, as many times as configured.
     *
     * UNREACHABLE WHILE THE STATE GUARD COMES FIRST, and the ordering is deliberate. The attempt
     * that spends the budget also sets the state to `blocked`, so every stop() after it answers at
     * the state guard and records `wrong_state`. Writing the test that expected this value is what
     * surfaced it.
     *
     * The guards are NOT reordered to make it reachable, because "does anything owe a stop" is the
     * prior question and reordering mis-answers a real case: a run that spends attempts and then
     * succeeds leaves the state `idle` with the counter still at the budget, where the truthful
     * answer is "nothing owes a stop" and a budget-first order would say "the budget is spent".
     *
     * So this enumerator is kept for the guard that produces it and the OBSERVABLE distinction
     * lives elsewhere: `blocked` with `failed_step == step::stop` is the device-was-asked case,
     * `blocked` with any earlier step is the device-was-never-asked case. That pair is what the
     * shell reports and what the regression pins. */
    budget_spent,
};

/* WHAT TO TELL THE OPERATOR AFTER A FAILED STOP, decided once and testable.
 *
 * The shell used to choose this inline, and review found the choice wrong on the attempt that
 * matters most: with a budget of one, a device that refuses the stop leaves `blocked` at step
 * `stop` with no refusal recorded -- the attempt was made, so nothing refused it -- and the inline
 * branch printed "Retry it". The next stop() is then certain to be refused. The operator is told to
 * do the one thing that cannot work, on the one path where the device may still be measuring.
 *
 * So the decision is a function of the recorded state, and `retryable` is the part under test. */
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

    /* TAKE OVER A DEVICE WHOSE STATE IS UNKNOWN.
     *
     * bring_up() refuses to open on top of a device that answered and could not be quiesced, which
     * is the right default: opening a second session on a sensor that may be ranging is the one
     * thing that ordering prevents. On 2026-10-12 that defence is where the Package sensor stopped
     * -- the stop handshake timed out with every transfer succeeding -- and the ULD's own init
     * begins with a software reboot sequence, a boot wait and a full firmware download, so there is
     * a case for letting init be the thing that establishes the device's state.
     *
     * This entry makes that case something a human asks for explicitly, once, and nothing else can
     * reach. It is NOT a retry of bring_up and NOT a cold start: `idle` means only that the
     * software holds no session, never that the sensor stopped, and nothing here establishes that
     * the device lost power. What it expresses is an authorisation to proceed without the
     * quiesce -- and the full init that follows is then the only thing that will have touched the
     * device's state.
     *
     * Everything else bring_up() does, it does: the blob verification, the probe, the identity
     * refusal, close, open, configure, start, and the same state and step on every failure. A
     * failure is left exactly as bring_up() would leave it -- blocked, or stop_owed after a start
     * that did not confirm -- and is NOT wound back to idle so the caller can try again.
     *
     * Only from `idle`, and only when initialised. From ranging, stop_owed or blocked it returns
     * -EPERM having touched no hardware and changed no field, so the evidence of whatever put the
     * lifecycle there survives the attempt. */
    int take_over_unknown_state();

    /* WHETHER bring_up() WOULD ACCEPT A NEW ROUND, asked rather than predicted.
     *
     * bring_up() refuses from two places before it touches anything -- not initialised, and a
     * state that is not idle -- and in both it leaves every earlier observation exactly as it
     * found it. A caller that keeps per-round diagnostics has to know which of those happened,
     * because clearing them for a round that never began destroys the evidence of the round that
     * did. On 2026-10-13 review reproduced precisely that: a first `up` failed at open with a
     * recorded port errno, a second `up` was refused for being in `blocked`, and the second had
     * already wiped the first.
     *
     * THIS IS THE GUARD, not a copy of it. bring_up() asks this function, so the two cannot drift:
     * a caller clearing on true clears exactly when a round starts. It is const and reads only the
     * state, and nothing between the two calls can change that state -- every entry point here
     * runs to completion on the caller's thread.
     *
     * It says nothing about whether the round will SUCCEED. A round that is accepted and then
     * refused by the verification has still begun: it consulted something, and its diagnostics are
     * its own. */
    bool can_begin_round() const;

    /* One poll. Refused unless ranging. */
    int read_once(bool *fresh);

    /* Returns to idle on an acknowledged stop. On a failure the state becomes stop_owed and this
     * may be called again; once the attempt budget is spent the state becomes blocked. */
    int stop();

    status current() const { return status_; }

    /* WHETHER THIS OBJECT CAN BE THROWN AWAY, which is the question a caller must ask before
     * re-binding or re-initialising it. False whenever a device may still be measuring or holds a
     * session nobody has accounted for: `ranging`, `stop_owed`, and `blocked` reached after
     * something was opened. True from `idle`, and from a `blocked` that never opened anything --
     * a refused verification, an unclassifiable probe, a wrong identity, a survivor that would not
     * quiesce.
     *
     * Discarding a non-discardable object does not merely lose bookkeeping: it forgets a stop debt
     * that only a successful stop may clear, and on this sensor the way back from a forgotten live
     * session is a machine power cycle. */
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
