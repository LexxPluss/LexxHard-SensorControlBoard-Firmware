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

// Scenario tests for the enumeration state machine, driven by a behavioural
// fake of the chain: a distributed flip-flop enable (with cascade breakage
// at an absent board), per-model enable semantics (L4 resets its address on
// enable-low, L7 retains while powered) and fault-injection knobs. Two
// standing assertions from review run through everything:
//
//   - every successful readdress is followed by a COMPLETE census before
//     the next clock pulse (checked structurally on the op log)
//   - the final source_allowed comes from the revocation table, never from
//     the historical per-position verdicts (tests deliberately construct
//     divergence between the two)

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_chain_spec.hpp"
#include "tof_enumerator.hpp"

namespace {

using namespace lexxhard::tof_enum;

enum class op_kind : uint8_t { probe, read_id, readdress, set_data, pulse, wait };

struct op_entry {
    op_kind kind{};
    uint8_t addr{0};
    bool level{false};
};

struct fake_chain final : chain_ops {
    static constexpr size_t kMax{8};
    static constexpr size_t kLogMax{512};

    struct device {
        bool present{true};
        model m{model::l4cx};
        uint8_t addr{kDefaultAddr};
        id_bytes custom_id{};       // used when custom
        bool use_custom_id{false};
        // The vanish knobs MUTE the sensor (it stops answering) without
        // removing the board: the flip-flop chain stays intact, which is
        // the physical situation "verified device went silent". present ==
        // false is reserved for a genuinely absent board and breaks the
        // enable cascade for everything downstream.
        int vanish_at_pulse{-1};
        int vanish_at_readdress{-1};
        bool mute{false};
    };
    device dev[kMax]{};
    size_t count{0};

    // enable-chain simulation
    bool data{false};
    bool ff[kMax]{};
    bool was_enabled[kMax]{};

    // fault injection
    int fail_set_data_at_call{-1};   // 1-based call index
    int fail_pulse_at_call{-1};
    int fail_readdress_at_call{-1};
    readdress_result readdress_failure{-EIO, readdress_stage::addr_write};
    uint8_t stray_addr{0};           // ACKs regardless of devices
    int stray_after_readdress_n{0};  // stray active once readdress calls reach n (0 = from start)
    uint8_t transport_addr{0};       // this address probes as transport_error...
    int transport_after_pulse{-1};   // ...once position pulses (post-alloff) reach this

    // counters and log
    int set_data_calls{0};
    int pulse_calls{0};              // all pulses including all-off
    int position_pulses{0};          // pulses after data went high
    int readdress_calls{0};
    op_entry log[kLogMax]{};
    size_t log_n{0};

    void push(op_kind k, uint8_t addr, bool level)
    {
        if (log_n < kLogMax)
            log[log_n++] = {k, addr, level};
    }

    bool cascade_ok(size_t i) const
    {
        for (size_t j{0}; j < i; ++j)
            if (!dev[j].present)
                return false;
        return true;
    }
    bool enabled(size_t i) const
    {
        bool const raw{i == 0 ? data : ff[i - 1]};
        return raw && cascade_ok(i);
    }
    void refresh_enables()
    {
        for (size_t i{0}; i < count; ++i) {
            bool const now{enabled(i)};
            if (was_enabled[i] && !now && dev[i].m == model::l4cx)
                dev[i].addr = kDefaultAddr;  // enable-low resets an L4
            was_enabled[i] = now;
        }
    }
    int device_at(uint8_t addr) const
    {
        for (size_t i{0}; i < count; ++i)
            if (dev[i].present && !dev[i].mute && enabled(i) && dev[i].addr == addr)
                return static_cast<int>(i);
        return -1;
    }

    probe_result probe(uint8_t addr7) override
    {
        push(op_kind::probe, addr7, false);
        if (addr7 == transport_addr && transport_addr != 0 &&
            position_pulses >= transport_after_pulse && transport_after_pulse >= 0)
            return {probe_state::transport_error, -ETIMEDOUT};
        if (addr7 == stray_addr && stray_addr != 0 &&
            readdress_calls >= stray_after_readdress_n)
            return {probe_state::ack, 0};
        if (device_at(addr7) >= 0)
            return {probe_state::ack, 0};
        return {probe_state::nack, 0};
    }
    int read_id(model m, uint8_t addr7, id_bytes &out) override
    {
        push(op_kind::read_id, addr7, false);
        int const i{device_at(addr7)};
        if (i < 0)
            return -EIO;
        if (dev[i].use_custom_id) {
            out = dev[i].custom_id;
        } else if (dev[i].m == model::l7cx) {
            out = {0xf0, 0x02};
        } else {
            out = {0xeb, 0xaa};
        }
        (void)m;  // the machine asks per its expectation; the device answers as itself
        return 0;
    }
    readdress_result readdress(model m, uint8_t old7, uint8_t new7) override
    {
        push(op_kind::readdress, new7, false);
        ++readdress_calls;
        for (size_t i{0}; i < count; ++i)
            if (dev[i].vanish_at_readdress >= 0 && readdress_calls >= dev[i].vanish_at_readdress)
                dev[i].mute = true;
        if (fail_readdress_at_call > 0 && readdress_calls == fail_readdress_at_call)
            return readdress_failure;
        (void)m;
        int const i{device_at(old7)};
        if (i < 0)
            return {-EIO, readdress_stage::verify};
        dev[i].addr = new7;
        return {0, readdress_stage::none};
    }
    int set_data(bool level) override
    {
        push(op_kind::set_data, 0, level);
        ++set_data_calls;
        if (fail_set_data_at_call > 0 && set_data_calls == fail_set_data_at_call)
            return -EIO;
        data = level;
        refresh_enables();
        return 0;
    }
    int pulse_clock() override
    {
        push(op_kind::pulse, 0, false);
        ++pulse_calls;
        if (fail_pulse_at_call > 0 && pulse_calls == fail_pulse_at_call)
            return -EIO;
        for (size_t i{kMax}; i-- > 1;)
            ff[i] = ff[i - 1];
        ff[0] = data;
        if (data)
            ++position_pulses;
        for (size_t i{0}; i < count; ++i)
            if (dev[i].vanish_at_pulse >= 0 && position_pulses >= dev[i].vanish_at_pulse)
                dev[i].mute = true;
        refresh_enables();
        return 0;
    }
    void wait(wait_reason reason) override
    {
        push(op_kind::wait, 0, reason == wait_reason::sensor_boot);
    }
};

chain_spec dasher_spec()
{
    chain_spec s{};
    s.positions = 6;
    s.at[0] = {model::l7cx, 0x2A, 0, l4_role::unknown};
    s.at[1] = {model::l7cx, 0x2B, 1, l4_role::unknown};
    s.at[2] = {model::l4cx, 0x2C, -1, l4_role::unknown};
    s.at[3] = {model::l4cx, 0x2D, -1, l4_role::unknown};
    s.at[4] = {model::l4cx, 0x2E, -1, l4_role::unknown};
    s.at[5] = {model::l4cx, 0x2F, -1, l4_role::unknown};
    s.watch_addrs[0] = 0x40;
    s.watch_count = 1;
    return s;
}

fake_chain dasher_chain()
{
    fake_chain c{};
    c.count = 6;
    c.dev[0] = {true, model::l7cx, kDefaultAddr, {}, false, -1, -1};
    c.dev[1] = {true, model::l7cx, kDefaultAddr, {}, false, -1, -1};
    for (size_t i{2}; i < 6; ++i)
        c.dev[i] = {true, model::l4cx, kDefaultAddr, {}, false, -1, -1};
    return c;
}

// The canonical census is one probe of the default, one of each target in
// position order, one of each watch address. Verifies the log carries
// exactly that block starting at `from`, returning the index one past it,
// or SIZE_MAX on mismatch.
size_t census_block_at(const fake_chain &c, const chain_spec &s, size_t from)
{
    size_t idx{from};
    auto expect_probe{[&](uint8_t addr) {
        if (idx >= c.log_n || c.log[idx].kind != op_kind::probe || c.log[idx].addr != addr)
            idx = SIZE_MAX;
        else
            ++idx;
    }};
    expect_probe(kDefaultAddr);
    for (size_t i{0}; i < s.positions && idx != SIZE_MAX; ++i)
        expect_probe(s.at[i].target_addr);
    for (size_t i{0}; i < s.watch_count && idx != SIZE_MAX; ++i)
        expect_probe(s.watch_addrs[i]);
    return idx;
}

// Standing assertion: every successful readdress is followed by a complete
// census before the next pulse. Returns how many readdress ops were checked.
int assert_post_census_after_every_readdress(const fake_chain &c, const chain_spec &s)
{
    int checked{0};
    for (size_t i{0}; i < c.log_n; ++i) {
        if (c.log[i].kind != op_kind::readdress)
            continue;
        size_t const end{census_block_at(c, s, i + 1)};
        zassert_not_equal(end, SIZE_MAX,
                          "readdress at log[%u] not followed by a complete census",
                          static_cast<unsigned>(i));
        for (size_t j{i + 1}; j < end; ++j)
            zassert_true(c.log[j].kind == op_kind::probe,
                         "non-probe op inside the post-census");
        ++checked;
    }
    return checked;
}

}  // namespace

ZTEST_SUITE(tof_enumerate, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_enumerate, test_invalid_specs_rejected_with_zero_ops)
{
    struct case_t {
        spec_error want;
        void (*mutate)(chain_spec &);
    };
    static const case_t kCases[]{
        {spec_error::position_count, [](chain_spec &s) { s.positions = 0; }},
        {spec_error::alloff_pulses, [](chain_spec &s) { s.alloff_pulses = 3; }},
        {spec_error::target_invalid, [](chain_spec &s) { s.at[2].target_addr = 0x02; }},
        {spec_error::target_is_default, [](chain_spec &s) { s.at[2].target_addr = kDefaultAddr; }},
        {spec_error::target_duplicate, [](chain_spec &s) { s.at[3].target_addr = 0x2C; }},
        {spec_error::source_id_range, [](chain_spec &s) { s.at[0].source_id = 2; }},
        {spec_error::source_id_duplicate, [](chain_spec &s) { s.at[1].source_id = 0; }},
        {spec_error::source_on_non_l7, [](chain_spec &s) { s.at[2].source_id = 1; s.at[1].source_id = -1; }},
        {spec_error::role_on_non_l4, [](chain_spec &s) { s.at[0].role = l4_role::front_left; }},
        {spec_error::watch_invalid, [](chain_spec &s) { s.watch_addrs[0] = 0x2A; }},
        {spec_error::watch_duplicate, [](chain_spec &s) { s.watch_addrs[1] = 0x40; s.watch_count = 2; }},
        {spec_error::no_source, [](chain_spec &s) { s.at[0].source_id = -1; s.at[1].source_id = -1; s.require_all_sources = false; }},
        {spec_error::sources_incomplete, [](chain_spec &s) { s.at[1].source_id = -1; }},
    };
    for (auto const &tc : kCases) {
        chain_spec s{dasher_spec()};
        tc.mutate(s);
        fake_chain c{dasher_chain()};
        auto const r{enumerate(c, s)};
        zassert_true(r.spec == tc.want, "wrong spec_error for case %d",
                     static_cast<int>(tc.want));
        zassert_true(r.status == chain_status::failed);
        zassert_equal(c.log_n, 0u, "hardware was touched by an invalid spec");
        zassert_equal(r.positions, 0u, "an invalid count must not leak into the result");
    }
}

ZTEST(tof_enumerate, test_cold_chain_complete_with_exact_structure)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    auto const r{enumerate(c, s)};

    zassert_true(r.status == chain_status::complete);
    zassert_true(r.source_allowed[0] && r.source_allowed[1]);
    zassert_true(r.control_state_known);
    zassert_true(r.data_commanded_high);
    zassert_equal(r.pulses_issued, 5);
    zassert_equal(r.frozen_at, -1);
    zassert_equal(r.interrupted_at, -1);
    for (size_t k{0}; k < 6; ++k) {
        zassert_true(r.at[k].verdict == outcome::enumerated, "position %u", (unsigned)k);
        zassert_equal(r.at[k].address, s.at[k].target_addr);
        zassert_true(r.at[k].enable_commanded_high);
    }
    // setup structure: data low, settle, 8 all-off pulses, data high, settle
    zassert_true(c.log[0].kind == op_kind::set_data && !c.log[0].level);
    zassert_true(c.log[1].kind == op_kind::wait && !c.log[1].level);
    for (size_t i{2}; i < 10; ++i)
        zassert_true(c.log[i].kind == op_kind::pulse);
    zassert_true(c.log[10].kind == op_kind::set_data && c.log[10].level);
    zassert_true(c.log[11].kind == op_kind::wait && !c.log[11].level);
    // the standing assertion: a complete census after every readdress
    zassert_equal(assert_post_census_after_every_readdress(c, s), 6);
}

ZTEST(tof_enumerate, test_hot_restart_retained_l7s_skip_readdress)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.dev[0].addr = 0x2A;  // both L7s kept their addresses across the restart
    c.dev[1].addr = 0x2B;
    auto const r{enumerate(c, s)};

    zassert_true(r.status == chain_status::complete);
    zassert_true(r.at[0].verdict == outcome::retained);
    zassert_true(r.at[1].verdict == outcome::retained);
    zassert_equal(c.readdress_calls, 4, "only the four L4s need a move");
    zassert_true(r.source_allowed[0] && r.source_allowed[1]);
    zassert_equal(assert_post_census_after_every_readdress(c, s), 4);
}

// The review counterexample: position 1's L7 retained at position 2's
// target. Vacancy holds at both 0x29 and 0x2A, and a two-point probe would
// have advanced and later misattributed the device; the census freezes at
// position 1 instead.
ZTEST(tof_enumerate, test_retained_at_foreign_target_freezes_at_position_one)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.dev[0].addr = 0x2B;
    auto const r{enumerate(c, s)};

    zassert_true(r.at[0].verdict == outcome::unexpected_address);
    zassert_equal(r.at[0].offending_addr, 0x2B);
    zassert_equal(r.frozen_at, 1);
    for (size_t k{1}; k < 6; ++k)
        zassert_true(r.at[k].verdict == outcome::not_attempted);
    zassert_equal(r.pulses_issued, 0, "no clock after the freeze");
    zassert_equal(c.readdress_calls, 0);
    zassert_true(r.status == chain_status::failed);
    zassert_true(r.data_commanded_high, "freeze leaves the chain as it is");
}

ZTEST(tof_enumerate, test_l4_unexpected_retained_freezes_keeping_earlier_sources)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.dev[2].addr = 0x2C;  // an L4 that somehow kept a target address
    auto const r{enumerate(c, s)};

    zassert_true(r.at[2].verdict == outcome::unexpected_retained);
    zassert_equal(r.frozen_at, 3);
    zassert_true(r.source_allowed[0] && r.source_allowed[1],
                 "a drop-sense anomaly must not revoke the verified L7s");
    zassert_true(r.status == chain_status::degraded);
}

ZTEST(tof_enumerate, test_ambiguous_identity_freezes)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.stray_addr = 0x2A;  // something answers position 1's target while the
                          // fresh device answers the default
    auto const r{enumerate(c, s)};

    zassert_true(r.at[0].verdict == outcome::ambiguous_identity);
    zassert_equal(r.frozen_at, 1);
    zassert_true(r.status == chain_status::failed);
    zassert_equal(c.readdress_calls, 0, "no move on an unresolved identity");
}

ZTEST(tof_enumerate, test_tail_absent_diagnosed_without_blocking)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.dev[5].present = false;  // the pos6 failure class
    auto const r{enumerate(c, s)};

    zassert_true(r.at[5].verdict == outcome::absent);
    zassert_equal(r.frozen_at, -1, "absent is not a freeze");
    zassert_equal(r.interrupted_at, 6);
    zassert_true(r.source_allowed[0] && r.source_allowed[1]);
    zassert_true(r.status == chain_status::degraded);
    zassert_equal(r.pulses_issued, 5, "the chain still advanced to the tail");
}

ZTEST(tof_enumerate, test_mid_chain_break_flagged_as_one_interruption)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.dev[3].present = false;  // board 4 missing breaks the FF chain
    auto const r{enumerate(c, s)};

    zassert_true(r.at[3].verdict == outcome::absent);
    zassert_true(r.at[4].verdict == outcome::absent);
    zassert_true(r.at[5].verdict == outcome::absent);
    zassert_equal(r.interrupted_at, 4,
                  "downstream absences are one suspected break, not three faults");
    zassert_true(r.status == chain_status::degraded);
}

// Strong assertion on the revocation table: the historical verdicts stay
// `enumerated` while every permission is revoked.
ZTEST(tof_enumerate, test_transport_error_revokes_everything)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.transport_addr = 0x2D;
    c.transport_after_pulse = 3;  // fires during position 4's census
    auto const r{enumerate(c, s)};

    zassert_true(r.at[3].verdict == outcome::transport_failed);
    zassert_equal(r.at[3].rc, -ETIMEDOUT);
    zassert_equal(r.frozen_at, 4);
    zassert_true(r.at[0].verdict == outcome::enumerated, "verdicts are history");
    zassert_true(r.at[1].verdict == outcome::enumerated);
    zassert_false(r.source_allowed[0], "bus reliability unknown: revoke all");
    zassert_false(r.source_allowed[1]);
    zassert_true(r.status == chain_status::failed);
}

// Strong assertion again: position 1 still reads `enumerated`, yet its
// source is revoked; the untouched source keeps its permission.
ZTEST(tof_enumerate, test_vanished_verified_device_revokes_only_its_source)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.dev[0].vanish_at_pulse = 2;  // gone once position 3 is activated
    auto const r{enumerate(c, s)};

    zassert_true(r.at[2].verdict == outcome::verified_device_missing);
    zassert_equal(r.at[2].offending_addr, 0x2A);
    zassert_equal(r.frozen_at, 3);
    zassert_true(r.at[0].verdict == outcome::enumerated,
                 "the historical verdict must not be rewritten");
    zassert_false(r.source_allowed[0], "the vanished device's source is revoked");
    zassert_true(r.source_allowed[1], "the other source keeps its permission");
    zassert_true(r.status == chain_status::degraded);
}

ZTEST(tof_enumerate, test_control_failure_marks_state_unknown_and_revokes_all)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.fail_pulse_at_call = 10;  // 8 all-off pulses + pos2 + pos3 activation
    auto const r{enumerate(c, s)};

    zassert_true(r.at[2].verdict == outcome::control_failed);
    zassert_equal(r.at[2].rc, -EIO, "the original errno must survive");
    zassert_true(r.control_failed_at == control_stage::advance_pulse);
    zassert_equal(r.control_rc, -EIO);
    zassert_equal(r.frozen_at, 3);
    zassert_false(r.control_state_known);
    zassert_true(r.at[0].verdict == outcome::enumerated);
    zassert_false(r.source_allowed[0]);
    zassert_false(r.source_allowed[1]);
    zassert_true(r.status == chain_status::failed);
}

ZTEST(tof_enumerate, test_setup_control_failure_freezes_before_any_position)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.fail_set_data_at_call = 1;
    auto const r{enumerate(c, s)};

    zassert_equal(r.frozen_at, 0, "setup phase");
    zassert_true(r.control_failed_at == control_stage::data_low);
    zassert_equal(r.control_rc, -EIO);
    zassert_false(r.control_state_known);
    zassert_false(r.data_commanded_high);
    zassert_true(r.status == chain_status::failed);
    for (size_t k{0}; k < 6; ++k)
        zassert_true(r.at[k].verdict == outcome::not_attempted);
}

ZTEST(tof_enumerate, test_wrong_model_freezes_keeping_earlier_sources)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.dev[2].m = model::l7cx;  // an L7 where the spec expects a drop-sense L4
    auto const r{enumerate(c, s)};

    zassert_true(r.at[2].verdict == outcome::wrong_model);
    zassert_equal(r.at[2].offending_addr, kDefaultAddr);
    zassert_equal(r.at[2].seen.first, 0xf0, "the foreign id bytes are reported");
    zassert_equal(r.frozen_at, 3);
    zassert_true(r.source_allowed[0] && r.source_allowed[1]);
    zassert_true(r.status == chain_status::degraded);
}

ZTEST(tof_enumerate, test_readdress_failure_freezes_with_stage_detail)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.fail_readdress_at_call = 3;  // position 3's move
    auto const r{enumerate(c, s)};

    zassert_true(r.at[2].verdict == outcome::readdress_failed);
    zassert_equal(r.at[2].readdress.rc, -EIO);
    zassert_true(r.at[2].readdress.failed_at == readdress_stage::addr_write);
    zassert_equal(r.frozen_at, 3);
    zassert_equal(r.pulses_issued, 2, "no clock after the freeze");
    zassert_true(r.source_allowed[0] && r.source_allowed[1]);
    zassert_true(r.status == chain_status::degraded);
}

// The post-census must apply the SAME revocation rule as the pre-census: a
// verified device vanishing between a position's pre- and post-census is
// caught in the post pass and still revokes exactly its own source.
ZTEST(tof_enumerate, test_vanish_during_post_census_revokes_only_its_source)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.dev[0].vanish_at_readdress = 3;  // disappears while position 3's move runs
    auto const r{enumerate(c, s)};

    zassert_true(r.at[2].verdict == outcome::verified_device_missing);
    zassert_equal(r.at[2].offending_addr, 0x2A);
    zassert_equal(r.frozen_at, 3);
    zassert_true(r.at[0].verdict == outcome::enumerated, "verdicts are history");
    zassert_false(r.source_allowed[0], "revoked in the POST census");
    zassert_true(r.source_allowed[1], "the untouched source keeps its permission");
    zassert_true(r.status == chain_status::degraded);
}

// A violation surfacing only in the post-readdress census: the move itself
// succeeded, but a stray appeared on a watch address. The position must
// fail, not the move be trusted.
ZTEST(tof_enumerate, test_post_census_violation_freezes_the_position)
{
    chain_spec const s{dasher_spec()};
    fake_chain c{dasher_chain()};
    c.stray_addr = 0x40;
    c.stray_after_readdress_n = 2;  // appears during position 2's post-census
    auto const r{enumerate(c, s)};

    zassert_true(r.at[1].verdict == outcome::unexpected_address);
    zassert_equal(r.at[1].offending_addr, 0x40);
    zassert_equal(r.frozen_at, 2);
    zassert_true(r.source_allowed[0], "position 1 passed a clean census earlier");
    zassert_false(r.source_allowed[1], "the failing position's source is never granted");
    zassert_true(r.status == chain_status::degraded);
}

// The glue rule is deliberately tiny; these pin it. -ENXIO is what the
// patched STM32 driver returns for a pure NACK; everything else that is
// not success must land in transport_error with the errno preserved.
ZTEST(tof_enumerate, test_probe_rc_classifier)
{
    zassert_true(classify_probe_rc(0).state == probe_state::ack);
    zassert_true(classify_probe_rc(-ENXIO).state == probe_state::nack);
    zassert_true(classify_probe_rc(-EIO).state == probe_state::transport_error);
    zassert_equal(classify_probe_rc(-EIO).rc, -EIO);
    zassert_true(classify_probe_rc(-ETIMEDOUT).state == probe_state::transport_error);
    zassert_equal(classify_probe_rc(-ETIMEDOUT).rc, -ETIMEDOUT);
    zassert_true(classify_probe_rc(-EBUSY).state == probe_state::transport_error,
                 "unknown errnos must never pass as a clean NACK");
}

// The shipped Dasher spec must be valid by the machine's own validation and
// carry the contract-owned source mapping (0 = right = position 1, 1 = left
// = position 2 per the connectivity diagram, pending the frozen J29 map).
ZTEST(tof_enumerate, test_shipped_dasher_spec_is_valid)
{
    auto const s{lexxhard::tof_chain::dasher_spec()};
    fake_chain c{dasher_chain()};
    auto const r{enumerate(c, s)};
    zassert_true(r.spec == spec_error::none, "shipped spec must validate");
    zassert_true(r.status == chain_status::complete);
    zassert_equal(s.at[0].source_id, 0);
    zassert_equal(s.at[1].source_id, 1);
    zassert_true(s.at[0].expected == model::l7cx);
    zassert_true(s.at[5].expected == model::l4cx);
}
