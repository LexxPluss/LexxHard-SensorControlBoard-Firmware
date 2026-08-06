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

#include <errno.h>

#include "tof_enumerator.hpp"

namespace lexxhard::tof_enum {

namespace {

bool addr_usable(uint8_t a)
{
    return a >= 0x08 && a <= 0x77;
}

spec_error validate(const chain_spec &spec)
{
    if (spec.positions == 0 || spec.positions > chain_spec::kMaxPositions)
        return spec_error::position_count;
    if (spec.alloff_pulses + 1u < spec.positions)
        return spec_error::alloff_pulses;
    for (size_t i{0}; i < spec.positions; ++i) {
        uint8_t const t{spec.at[i].target_addr};
        if (!addr_usable(t))
            return spec_error::target_invalid;
        if (t == kDefaultAddr)
            return spec_error::target_is_default;
        for (size_t j{0}; j < i; ++j)
            if (spec.at[j].target_addr == t)
                return spec_error::target_duplicate;
    }
    bool source_seen[chain_result::kMaxSources]{};
    bool any_source{false};
    for (size_t i{0}; i < spec.positions; ++i) {
        int8_t const s{spec.at[i].source_id};
        if (s < -1 || s >= static_cast<int8_t>(chain_result::kMaxSources))
            return spec_error::source_id_range;
        if (s >= 0) {
            if (source_seen[s])
                return spec_error::source_id_duplicate;
            source_seen[s] = true;
            any_source = true;
            if (spec.at[i].expected != model::l7cx)
                return spec_error::source_on_non_l7;
        }
        if (spec.at[i].role != l4_role::unknown && spec.at[i].expected != model::l4cx)
            return spec_error::role_on_non_l4;
    }
    if (spec.watch_count > chain_spec::kMaxWatch)
        return spec_error::watch_count;
    for (size_t i{0}; i < spec.watch_count; ++i) {
        uint8_t const w{spec.watch_addrs[i]};
        if (!addr_usable(w) || w == kDefaultAddr)
            return spec_error::watch_invalid;
        for (size_t j{0}; j < spec.positions; ++j)
            if (spec.at[j].target_addr == w)
                return spec_error::watch_invalid;
        for (size_t j{0}; j < i; ++j)
            if (spec.watch_addrs[j] == w)
                return spec_error::watch_duplicate;
    }
    if (!any_source)
        return spec_error::no_source;
    if (spec.require_all_sources) {
        for (size_t s{0}; s < chain_result::kMaxSources; ++s)
            if (!source_seen[s])
                return spec_error::sources_incomplete;
    }
    return spec_error::none;
}

// One full sweep of the watched set. Every address is probed regardless of
// earlier findings (the sweep itself must be complete on the wire); the
// classification afterwards applies the fixed priority: transport error,
// then a verified device gone silent, then an unowned ACK. The default and
// the current position's target are exempt from the generic ownership
// checks -- the caller classifies those two from the returned states.
struct census {
    bool clean{false};
    outcome verdict{outcome::not_attempted};  // when !clean
    int rc{0};
    uint8_t offending{0};
    probe_state d{probe_state::transport_error};
    probe_state t{probe_state::transport_error};
};

census run_census(chain_ops &ops, const chain_spec &spec, const bool owned[],
                  size_t current)
{
    census c{};
    probe_result results[1 + chain_spec::kMaxPositions + chain_spec::kMaxWatch]{};
    uint8_t addrs[1 + chain_spec::kMaxPositions + chain_spec::kMaxWatch]{};
    size_t n{0};

    addrs[n] = kDefaultAddr;
    results[n] = ops.probe(kDefaultAddr);
    ++n;
    for (size_t i{0}; i < spec.positions; ++i) {
        addrs[n] = spec.at[i].target_addr;
        results[n] = ops.probe(spec.at[i].target_addr);
        ++n;
    }
    for (size_t i{0}; i < spec.watch_count; ++i) {
        addrs[n] = spec.watch_addrs[i];
        results[n] = ops.probe(spec.watch_addrs[i]);
        ++n;
    }

    c.d = results[0].state;
    c.t = results[1 + current].state;

    for (size_t i{0}; i < n; ++i) {
        if (results[i].state == probe_state::transport_error) {
            c.verdict = outcome::transport_failed;
            c.rc = results[i].rc;
            c.offending = addrs[i];
            return c;
        }
    }
    for (size_t i{0}; i < spec.positions; ++i) {
        if (!owned[i])
            continue;
        if (results[1 + i].state != probe_state::ack) {
            c.verdict = outcome::verified_device_missing;
            c.offending = spec.at[i].target_addr;
            return c;
        }
    }
    for (size_t i{0}; i < spec.positions; ++i) {
        if (owned[i] || i == current)
            continue;
        if (results[1 + i].state == probe_state::ack) {
            c.verdict = outcome::unexpected_address;
            c.offending = spec.at[i].target_addr;
            return c;
        }
    }
    for (size_t i{0}; i < spec.watch_count; ++i) {
        if (results[1 + spec.positions + i].state == probe_state::ack) {
            c.verdict = outcome::unexpected_address;
            c.offending = spec.watch_addrs[i];
            return c;
        }
    }
    c.clean = true;
    return c;
}

bool id_matches(model m, const id_bytes &b)
{
    if (m == model::l7cx)
        return b.first == 0xf0 && b.second == 0x02;
    return b.first == 0xeb && b.second == 0xaa;
}

}  // namespace

chain_result enumerate(chain_ops &ops, const chain_spec &spec)
{
    chain_result r{};
    r.positions = spec.positions;
    r.spec = validate(spec);
    if (r.spec != spec_error::none)
        return r;  // status stays failed; ZERO hardware operations

    // Grants are collected here and the revocation table is applied at the
    // end: the final source_allowed is decided by the table, never read
    // back off the historical per-position verdicts.
    bool granted[chain_result::kMaxSources]{};
    bool revoke_all{false};
    bool owned[chain_spec::kMaxPositions]{};

    auto freeze{[&](size_t pos_index, outcome verdict, int rc, uint8_t offending) {
        r.at[pos_index].verdict = verdict;
        r.at[pos_index].rc = rc;
        r.at[pos_index].offending_addr = offending;
        r.frozen_at = static_cast<int8_t>(pos_index + 1);
    }};

    // ---- all-off, then enable-start ----
    if (ops.set_data(false) != 0) {
        r.frozen_at = 0;
        r.control_state_known = false;
        r.status = chain_status::failed;
        return r;
    }
    ops.wait(chain_ops::wait_reason::data_settle);
    for (uint8_t i{0}; i < spec.alloff_pulses; ++i) {
        if (ops.pulse_clock() != 0) {
            r.frozen_at = 0;
            r.control_state_known = false;
            r.status = chain_status::failed;
            return r;
        }
    }
    if (ops.set_data(true) != 0) {
        r.frozen_at = 0;
        r.control_state_known = false;
        r.status = chain_status::failed;
        return r;
    }
    r.data_commanded_high = true;
    ops.wait(chain_ops::wait_reason::data_settle);

    // ---- one position per clock ----
    for (size_t k{0}; k < spec.positions; ++k) {
        if (k > 0) {
            if (ops.pulse_clock() != 0) {
                freeze(k, outcome::control_failed, 0, 0);
                r.control_state_known = false;
                revoke_all = true;
                break;
            }
            ++r.pulses_issued;
        }
        r.at[k].enable_commanded_high = true;
        ops.wait(chain_ops::wait_reason::sensor_boot);

        census const c{run_census(ops, spec, owned, k)};
        if (!c.clean) {
            freeze(k, c.verdict, c.rc, c.offending);
            if (c.verdict == outcome::transport_failed)
                revoke_all = true;
            if (c.verdict == outcome::verified_device_missing) {
                for (size_t j{0}; j < spec.positions; ++j)
                    if (spec.at[j].target_addr == c.offending && spec.at[j].source_id >= 0)
                        granted[spec.at[j].source_id] = false;
            }
            break;
        }

        position_spec const &p{spec.at[k]};
        bool frozen{false};

        if (c.d == probe_state::ack && c.t == probe_state::ack) {
            freeze(k, outcome::ambiguous_identity, 0, 0);
            frozen = true;
        } else if (c.d == probe_state::nack && c.t == probe_state::ack) {
            if (p.expected != model::l7cx) {
                freeze(k, outcome::unexpected_retained, 0, p.target_addr);
                frozen = true;
            } else {
                id_bytes seen{};
                int const rc{ops.read_id(model::l7cx, p.target_addr, seen)};
                r.at[k].seen = seen;
                if (rc != 0) {
                    freeze(k, outcome::transport_failed, rc, 0);
                    revoke_all = true;
                    frozen = true;
                } else if (!id_matches(model::l7cx, seen)) {
                    freeze(k, outcome::wrong_model, -ENODEV, p.target_addr);
                    frozen = true;
                } else {
                    r.at[k].verdict = outcome::retained;
                    r.at[k].address = p.target_addr;
                }
            }
        } else if (c.d == probe_state::ack) {  // T nack
            id_bytes seen{};
            int const rc{ops.read_id(p.expected, kDefaultAddr, seen)};
            r.at[k].seen = seen;
            if (rc != 0) {
                freeze(k, outcome::transport_failed, rc, 0);
                revoke_all = true;
                frozen = true;
            } else if (!id_matches(p.expected, seen)) {
                freeze(k, outcome::wrong_model, -ENODEV, kDefaultAddr);
                frozen = true;
            } else {
                readdress_result const move{
                    ops.readdress(p.expected, kDefaultAddr, p.target_addr)};
                r.at[k].readdress = move;
                if (move.rc != 0 || move.failed_at != readdress_stage::none) {
                    freeze(k, outcome::readdress_failed, move.rc, 0);
                    frozen = true;
                } else {
                    // Post-census before the next pulse: "readdress
                    // succeeded" must never substitute for "default proven
                    // vacant". The moved device is owned for this sweep.
                    owned[k] = true;
                    census const after{run_census(ops, spec, owned, k)};
                    if (!after.clean) {
                        owned[k] = false;
                        freeze(k, after.verdict, after.rc, after.offending);
                        if (after.verdict == outcome::transport_failed)
                            revoke_all = true;
                        frozen = true;
                    } else if (after.d != probe_state::nack) {
                        owned[k] = false;
                        freeze(k, outcome::unexpected_address, 0, kDefaultAddr);
                        frozen = true;
                    } else {
                        r.at[k].verdict = outcome::enumerated;
                        r.at[k].address = p.target_addr;
                    }
                }
            }
        } else {  // D nack, T nack, census clean: vacancy proven
            r.at[k].verdict = outcome::absent;
        }

        if (frozen)
            break;

        if (r.at[k].verdict == outcome::enumerated || r.at[k].verdict == outcome::retained) {
            owned[k] = true;
            if (p.source_id >= 0)
                granted[p.source_id] = true;
        }
    }

    // ---- interpretation ----
    // A suffix run of absent positions reaching the tail is one suspected
    // break, not N independent faults: the enable propagates through each
    // board's flip-flop.
    if (r.frozen_at < 0) {
        for (size_t k{spec.positions}; k > 0; --k) {
            if (r.at[k - 1].verdict != outcome::absent)
                break;
            r.interrupted_at = static_cast<int8_t>(k);
        }
    }

    // ---- revocation table decides the final permissions ----
    if (revoke_all) {
        for (size_t s{0}; s < chain_result::kMaxSources; ++s)
            granted[s] = false;
    }
    bool any_allowed{false};
    bool all_verified{true};
    for (size_t s{0}; s < chain_result::kMaxSources; ++s) {
        r.source_allowed[s] = granted[s];
        any_allowed = any_allowed || granted[s];
    }
    for (size_t k{0}; k < spec.positions; ++k) {
        if (r.at[k].verdict != outcome::enumerated && r.at[k].verdict != outcome::retained)
            all_verified = false;
    }

    if (all_verified)
        r.status = chain_status::complete;
    else if (any_allowed)
        r.status = chain_status::degraded;
    else
        r.status = chain_status::failed;
    return r;
}

}  // namespace lexxhard::tof_enum
