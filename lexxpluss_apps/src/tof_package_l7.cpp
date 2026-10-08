/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "tof_package_l7.hpp"

#if defined(ENABLE_TOF_L7_ULD)

#include <errno.h>

namespace lexxhard::tof_package {

int lifecycle::refuse(step s, int rc)
{
    status_.failed_step = s;
    status_.errno_value = rc;
    return rc;
}

int lifecycle::init(const hooks &h, const config &c)
{
    /* EVERY HOOK IS REQUIRED, with no optional ones. An absent hook is not a feature a caller
     * chose to leave out: this lifecycle has one device and no fallback, so a missing probe or a
     * missing stop would silently remove the only check standing between an unknown device and an
     * open. Refusing here is the only place the omission is visible. */
    if (h.ensure_verified == nullptr || h.probe == nullptr || h.quiesce == nullptr ||
        h.close == nullptr || h.open == nullptr || h.configure == nullptr || h.start == nullptr ||
        h.read == nullptr || h.stop == nullptr)
        return -EINVAL;
    /* RE-INITIALISING OVER A LIVE OR UNACCOUNTED SESSION IS REFUSED, and refused before anything
     * is written, so a rejected call leaves the object exactly as it was. init() otherwise resets
     * the state wholesale, which would discard a stop debt that only a successful stop may clear
     * -- and the way back from a forgotten live session on this sensor is a machine power cycle. */
    if (initialised_ && !discardable())
        return -EPERM;
    /* The ULD accepts 1..15 Hz; the value is the caller's because the sustainable rate for this
     * geometry has not been accepted yet. Zero is refused rather than defaulted, so a caller that
     * forgot it finds out here instead of ranging at whatever the ULD does with 0. */
    if (c.frequency_hz < 1U || c.frequency_hz > 15U)
        return -EINVAL;
    if (c.address_7bit < 0x08U || c.address_7bit > 0x77U)
        return -EINVAL;
    /* See config::stop_attempts: a budget of zero would make stop_owed inescapable. */
    if (c.stop_attempts == 0U)
        return -EINVAL;

    hooks_ = h;
    config_ = c;
    status_ = status{};
    initialised_ = true;
    return 0;
}

int lifecycle::bring_up()
{
    if (!initialised_)
        return -EPERM;
    /* Only from idle. ranging would stack a session, stop_owed cannot be left except by stop(),
     * and blocked is waiting on a decision this module does not make. */
    if (status_.current != state::idle)
        return -EPERM;

    status_.failed_step = step::none;
    status_.errno_value = 0;
    status_.survivor_found = false;

    /* ASKED, NOT PERFORMED. One payload serves every L7 in the image, so this consults the one
     * verification rather than starting another. A refusal stops the run here: without a verified
     * payload there is nothing to download, and probing a device we could not then open would only
     * disturb it. */
    if (const int rc{hooks_.ensure_verified()}; rc != 0) {
        /* -EINPROGRESS MEANS ASK AGAIN, NOT GIVE UP. Another consumer is mid-verification, so the
         * answer is not in yet; blocking on it would turn a few milliseconds of contention into a
         * permanent refusal. The state is left untouched -- still idle, nothing opened, no device
         * disturbed -- so a later call simply runs. Every other errno is a verdict and blocks. */
        if (rc != -EINPROGRESS)
            status_.current = state::blocked;
        return refuse(step::firmware, rc);
    }

    bool present{false};
    bool wrong_identity{false};
    const int probe_rc{
        hooks_.probe(hooks_.ctx, config_.address_7bit, &present, &wrong_identity)};

    /* THE RULE THIS MODULE EXISTS FOR. A probe that did not complete tells us nothing about
     * presence, and `present` is meaningless. The chain could record that and carry on to other
     * positions; here there is no other position and no enumerator to cross-check against, so
     * continuing would mean opening a device that may already be ranging. Blocked, and no open. */
    if (probe_rc != 0) {
        status_.current = state::blocked;
        return refuse(step::probe, probe_rc);
    }

    /* SOMETHING ANSWERED AND IS NOT A VL53L7CX. Stopped here, and in particular BEFORE the
     * quiesce: issuing a five-second L7 stop sequence to a device that is not one is not this
     * module's business, and opening it afterwards would download 86 KB of L7 firmware into
     * something nobody has identified.
     *
     * The chain can afford to report this and move on, because its addresses are assigned by the
     * enable chain and enumeration exists to act on a census anomaly. Here the address is fixed at
     * 0x29 and there is no enumerator, so there is nothing to hand it to. */
    if (wrong_identity) {
        status_.current = state::blocked;
        return refuse(step::identity, -ENOTSUP);
    }

    if (present) {
        /* A SURVIVOR, which on this sensor is the warm-reset case: an SCB reset does not reset the
         * device and there is no reset line to use, so it may still be powered and measuring.
         * Quiesced over the bus before anything else, and a failure to quiesce is terminal for
         * this run -- opening on top of a live session is the one thing this ordering prevents. */
        status_.survivor_found = true;
        if (const int rc{hooks_.quiesce(hooks_.ctx, config_.address_7bit)}; rc != 0) {
            status_.current = state::blocked;
            return refuse(step::survivor, rc);
        }
    }

    /* A FULL INITIALISATION EITHER WAY, survivor or cold. A successful quiesce clears the stop debt
     * and nothing more: which device state survives an SCB reset is not established, and the
     * host-side configuration object is gone with the reset regardless, so there is nothing to
     * resume into. Skipping the firmware download on a warm entry would be an optimisation that
     * needs its own evidence.
     *
     * AND IT STARTS WITH A CLOSE, BEFORE EVERY OPEN, not only after a failure. A confirmed stop
     * leaves the sensor `configured` and tof_l7::open() requires `empty`, so an ordinary
     * up -> stop -> up sequence fails on its second open without this -- the sensor would be a
     * one-session-per-boot object. From an already-empty sensor the close succeeds and does nothing,
     * which is what lets it be unconditional rather than conditional on reasoning about which
     * earlier failure left what behind. */
    if (const int rc{hooks_.close(hooks_.ctx)}; rc != 0) {
        status_.current = state::blocked;
        return refuse(step::close, rc);
    }
    if (const int rc{hooks_.open(hooks_.ctx, config_.address_7bit)}; rc != 0) {
        status_.current = state::blocked;
        return refuse(step::open, rc);
    }
    if (const int rc{hooks_.configure(hooks_.ctx, config_.frequency_hz)}; rc != 0) {
        status_.current = state::blocked;
        return refuse(step::configure, rc);
    }

    status_.stop_attempts_used = 0;
    if (const int rc{hooks_.start(hooks_.ctx)}; rc != 0) {
        /* A FAILED START OWES A STOP, not a close. The command may have gone out and the device
         * may or may not have acted on it, so the session is unaccounted for and the sensor layer
         * records the same debt. Landing in blocked here would strand a possibly-ranging device
         * with no way to stop it. */
        status_.current = state::stop_owed;
        return refuse(step::start, rc);
    }

    status_.current = state::ranging;
    return 0;
}

int lifecycle::read_once(bool *fresh)
{
    if (fresh == nullptr)
        return -EINVAL;
    *fresh = false;
    if (!initialised_)
        return -EPERM;
    /* ranging only. stop_owed refuses by construction: a session we cannot account for must not
     * have its data trusted. */
    if (status_.current != state::ranging)
        return -EPERM;

    const int rc{hooks_.read(hooks_.ctx, fresh)};
    if (rc != 0) {
        /* A READ ERROR DOES NOT MEAN RANGING STOPPED. The state stays `ranging`, so the way out is
         * still a stop() the device acknowledges; dropping to idle here would let a later open
         * start a second session on a device that never stopped. The step and errno record what
         * happened for the caller to act on. */
        *fresh = false;
        return refuse(step::read, rc);
    }
    return 0;
}

int lifecycle::stop()
{
    if (!initialised_)
        return -EPERM;
    /* Accepts both, and that is the point: stop_owed exists so that cleanup is possible after an
     * outcome nobody could confirm, and refusing it here would make the state a dead end. */
    if (status_.current != state::ranging && status_.current != state::stop_owed)
        return -EPERM;

    if (status_.stop_attempts_used >= config_.stop_attempts)
        return -EPERM;

    ++status_.stop_attempts_used;
    const int rc{hooks_.stop(hooks_.ctx)};
    if (rc == 0) {
        status_.current = state::idle;
        status_.failed_step = step::none;
        status_.errno_value = 0;
        return 0;
    }

    /* The debt is kept so the caller can try again, and only the budget running out turns it into
     * something to report. Blocked rather than silently idle: a device that may still be
     * measuring, that we have given up on stopping, is exactly the state an operator has to be
     * told about. */
    status_.current = status_.stop_attempts_used >= config_.stop_attempts ? state::blocked
                                                                         : state::stop_owed;
    return refuse(step::stop, rc);
}

bool lifecycle::discardable() const
{
    switch (status_.current) {
    case state::idle:
        return true;
    case state::ranging:
    case state::stop_owed:
        /* A device that is measuring, or one whose session nobody can account for. */
        return false;
    case state::blocked:
        /* THE DIVIDING LINE IS WHETHER A MEASUREMENT COULD HAVE BEEN STARTED, not whether the device
         * was touched. Only start() can begin one, and a start failure does not land here: it owes a
         * stop, so it is `stop_owed` above and reaches `blocked` only through a spent stop budget,
         * which is recorded as step::stop. The two steps left off this list are exactly the ones a
         * live session can be behind:
         *
         *   - step::close, because close() refuses precisely when the sensor is running or owes a
         *     stop, so arriving here means one of those was true;
         *   - step::stop, which is the spent budget: a device that may still be measuring and that
         *     this tool has given up on stopping.
         *
         * open and configure are on the list because neither starts ranging -- a failure in either
         * leaves a device that was configured at most, and the sensor layer in a state a later close
         * accepts. Listed positively so that a step added later is non-discardable until someone
         * decides otherwise. */
        return status_.failed_step == step::firmware || status_.failed_step == step::probe ||
               status_.failed_step == step::identity || status_.failed_step == step::survivor ||
               status_.failed_step == step::open || status_.failed_step == step::configure;
    }
    return false;
}

const char *state_name(state s)
{
    switch (s) {
    case state::idle:      return "idle";
    case state::ranging:   return "ranging";
    case state::stop_owed: return "stop_owed";
    case state::blocked:   return "blocked";
    }
    return "unknown";
}

const char *step_name(step s)
{
    switch (s) {
    case step::none:      return "none";
    case step::firmware:  return "firmware";
    case step::probe:     return "probe";
    case step::identity:  return "identity";
    case step::survivor:  return "survivor";
    case step::close:     return "close";
    case step::open:      return "open";
    case step::configure: return "configure";
    case step::start:     return "start";
    case step::read:      return "read";
    case step::stop:      return "stop";
    }
    return "unknown";
}

}  // namespace lexxhard::tof_package

#endif  // ENABLE_TOF_L7_ULD
