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

struct status {
    state current{state::idle};
    step failed_step{step::none};
    int errno_value{0};
    /* True only when the probe said something answered BEFORE this run opened anything. Recorded
     * because it is the warm-reset signal, and because a run that found one and continued has to be
     * able to say so. */
    bool survivor_found{false};
    uint8_t stop_attempts_used{0};
};

class lifecycle {
public:
    int init(const hooks &h, const config &c);

    /* ensure_verified -> probe -> quiesce a survivor -> open -> configure -> start.
     *
     * Refuses unless the state is idle. Any failure leaves a state and a step that say where it
     * stopped, and no later step is attempted -- in particular an unclassifiable probe stops the
     * run instead of being treated as an empty address. */
    int bring_up();

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
