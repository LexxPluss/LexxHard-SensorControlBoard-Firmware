/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * See tof_l7_recovery.hpp for why this exists and what it does not claim.
 */

#include "tof_l7_recovery.hpp"

namespace lexxhard::tof_l7_recovery {

namespace {

/* The usable 7-bit range, the same bound the L7 adapter and the enumerator's spec validation
 * already apply. Repeated rather than shared because an address this pass cannot use is a caller
 * error it must survive, not a chain property it should go looking up. */
bool address_usable(uint8_t addr)
{
    return addr >= 0x08U && addr <= 0x77U;
}

}  // namespace

report run(const ops &o, const request &req)
{
    report r{};

    /* Clamped rather than refused. A request carrying more addresses than this chain can hold is a
     * caller defect, and dropping the whole pass on it would turn that defect into a boot that
     * silently recovers nothing -- the failure mode this module was written to end. */
    r.count = req.count < kMaxGridSensors ? req.count : kMaxGridSensors;

    /* Both calls or neither. A half-populated ops would probe and then have no way to act on what
     * it found, which is worse than not probing: it would spend the bus traffic and still report
     * every survivor as untouched. */
    if (o.is_alive == nullptr || o.stop_ranging == nullptr) {
        r.any_failure = true;
        return r;
    }

    for (size_t i{0}; i < r.count; ++i) {
        const uint8_t addr{req.addr_7bit[i]};

        if (!address_usable(addr)) {
            r.at[i] = result::not_attempted;
            r.any_failure = true;
            continue;
        }

        bool alive{false};
        if (o.is_alive(o.ctx, addr, &alive) != 0) {
            /* Deliberately not followed by a stop. A transport error means the bus did not carry
             * the question, so an answer to it was never heard -- issuing the five-second stop
             * anyway would spend that time on a bus already known not to be working. */
            r.at[i] = result::probe_failed;
            r.any_failure = true;
            continue;
        }

        if (!alive) {
            r.at[i] = result::absent;
            continue;
        }

        if (o.stop_ranging(o.ctx, addr) != 0) {
            r.at[i] = result::stop_failed;
            r.any_failure = true;
            continue;
        }

        r.at[i] = result::stopped;
        ++r.stopped;
    }

    return r;
}

const char *result_name(result r)
{
    switch (r) {
    case result::not_attempted: return "not_attempted";
    case result::absent:        return "absent";
    case result::stopped:       return "stopped";
    case result::stop_failed:   return "stop_failed";
    case result::probe_failed:  return "probe_failed";
    }
    return "unknown";
}

}  // namespace lexxhard::tof_l7_recovery
