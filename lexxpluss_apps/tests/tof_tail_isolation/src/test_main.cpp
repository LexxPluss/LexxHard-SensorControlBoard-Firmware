/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side tests for tail isolation.
 *
 * The fake records every control operation in order, because the ORDER IS THE PROPERTY. An
 * isolation that reaches "only the tail enabled" through an all-off burst ends in the right state
 * and has destroyed the evidence it was sent to collect: the L4 enable is reset-class, so a tail
 * that was ever disabled answers the factory default address and "the tail answers its own
 * address" becomes a check on nothing. End-state assertions cannot tell the two apart, so these
 * tests assert the sequence.
 *
 * The fake also models the shift register, rather than just returning canned probe answers. That
 * is what lets a test say "the merge case" by wiring two devices to one address and letting the
 * model decide who answers.
 */

#include <zephyr/ztest.h>

#include "tof_tail_isolation.hpp"

namespace enm = lexxhard::tof_enum;
namespace pf = lexxhard::tof_proof;
namespace iso = lexxhard::tof_isolate;

namespace {

constexpr enm::id_bytes kL4Id{0xeb, 0xaa};
constexpr enm::id_bytes kL7Id{0xf0, 0x02};

enum class op : uint8_t { data_low, data_high, pulse, probe, read_id, wait };

struct recorded {
    op what;
    uint8_t addr;   // for probe / read_id
};

/* The chain as the hardware behaves: position 1's enable IS the data line, and each pulse shifts
 * every stage's enable one position down. Six stages, one device each, each with the address the
 * enumeration gave it. */
struct fake_chain : enm::chain_ops {
    static constexpr size_t kStages{6};

    /* `enabled` is the shift register's state -- what the control lines command. `present` is
     * whether a device is physically there to answer. They are separate because a test cannot model
     * a missing board by clearing `enabled`: the very next pulse shifts a 1 back into that stage.
     * The first version of this fake conflated them and produced a test that was asserting on a
     * chain the sequence had already re-enabled. */
    bool enabled[kStages]{true, true, true, true, true, true};
    bool present[kStages]{true, true, true, true, true, true};
    uint8_t addr[kStages]{0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F};
    bool data{true};

    recorded log[64]{};
    int logged{0};

    int set_data_rc{0};
    int pulse_rc{0};
    int read_id_rc{0};
    int fail_pulse_at{-1};       // 0-based pulse index that fails
    bool probe_transport_error{false};
    /* A disabled L4 loses its address, which is why an all-off destroys the evidence. Modelled
     * so a test can prove the implementation does not rely on it surviving. */
    bool enable_is_reset_class{true};

    void note(op what, uint8_t addr7 = 0)
    {
        if (logged < static_cast<int>(sizeof log / sizeof log[0]))
            log[logged++] = {what, addr7};
    }

    int set_data(bool level) override
    {
        note(level ? op::data_high : op::data_low);
        if (set_data_rc != 0)
            return set_data_rc;
        data = level;
        enabled[0] = level;
        if (!level && enable_is_reset_class)
            addr[0] = enm::kDefaultAddr;
        return 0;
    }

    int pulse_clock() override
    {
        note(op::pulse);
        if (fail_pulse_at >= 0 && pulses_seen == fail_pulse_at) {
            ++pulses_seen;
            return -EIO;
        }
        ++pulses_seen;
        if (pulse_rc != 0)
            return pulse_rc;

        for (size_t i{kStages - 1}; i > 0; --i) {
            const bool was{enabled[i]};
            enabled[i] = enabled[i - 1];
            if (was && !enabled[i] && enable_is_reset_class)
                addr[i] = enm::kDefaultAddr;
        }
        enabled[0] = data;
        return 0;
    }

    int pulses_seen{0};

    enm::probe_result probe(uint8_t addr7) override
    {
        note(op::probe, addr7);
        if (probe_transport_error)
            return {enm::probe_state::transport_error, -EIO};
        for (size_t i{0}; i < kStages; ++i) {
            if (present[i] && enabled[i] && addr[i] == addr7)
                return {enm::probe_state::ack, 0};
        }
        return {enm::probe_state::nack, 0};
    }

    int read_id(enm::model m, uint8_t addr7, enm::id_bytes &out) override
    {
        note(op::read_id, addr7);
        if (read_id_rc != 0)
            return read_id_rc;
        for (size_t i{0}; i < kStages; ++i) {
            if (present[i] && enabled[i] && addr[i] == addr7) {
                out = (i < 2) ? kL7Id : kL4Id;
                return 0;
            }
        }
        return -ENXIO;
    }

    enm::readdress_result readdress(enm::model, uint8_t, uint8_t) override
    {
        /* Isolation must never re-address anything. If this is ever reached the sequence has
         * mutated the chain it was sent to observe. */
        zassert_unreachable("isolation attempted a readdress");
        return {};
    }

    void wait(wait_reason) override { note(op::wait); }
};

enm::chain_spec product_spec()
{
    enm::chain_spec s{};
    s.positions = 6;
    s.at[0] = {enm::model::l7cx, 0x2A, 0, enm::l4_role::unknown};
    s.at[1] = {enm::model::l7cx, 0x2B, 1, enm::l4_role::unknown};
    s.at[2] = {enm::model::l4cx, 0x2C, -1, enm::l4_role::front_left};
    s.at[3] = {enm::model::l4cx, 0x2D, -1, enm::l4_role::rear_left};
    s.at[4] = {enm::model::l4cx, 0x2E, -1, enm::l4_role::rear_right};
    s.at[5] = {enm::model::l4cx, 0x2F, -1, enm::l4_role::front_right};
    s.alloff_pulses = 8;
    return s;
}

int count_op(const fake_chain &c, op what)
{
    int n{0};
    for (int i = 0; i < c.logged; ++i)
        if (c.log[i].what == what)
            ++n;
    return n;
}

} // namespace

ZTEST_SUITE(tof_tail_isolation, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_tail_isolation, test_a_healthy_chain_leaves_the_tail_on_its_own_address)
{
    fake_chain c{};
    const enm::chain_spec spec{product_spec()};
    pf::isolation_observation o{};

    zassert_equal(iso::observe_tail(c, spec, o), 0);

    zassert_true(o.attempted);
    zassert_equal(o.tail_probe, enm::probe_state::ack);
    zassert_equal(o.answering_addr, 0x2F, "the tail must still hold the address it was given");
    zassert_true(o.id_read_ok);
    zassert_equal(o.seen.first, kL4Id.first);
    zassert_equal(o.seen.second, kL4Id.second);
    zassert_equal(o.prev_addr, 0x2E);
    zassert_equal(o.prev_probe, enm::probe_state::nack, "the neighbour must be silent");
}

ZTEST(tof_tail_isolation, test_the_sequence_never_disables_the_tail)
{
    /* The property end-state assertions cannot see. With the data line low, zeros walk in from the
     * head: positions 1-5 go dark one at a time and the tail is never disabled, which is the only
     * reason it still holds the address being checked. */
    fake_chain c{};
    const enm::chain_spec spec{product_spec()};
    pf::isolation_observation o{};

    zassert_equal(iso::observe_tail(c, spec, o), 0);

    /* positions - 2, not positions - 1: the data line darkens position 1 by itself, and one pulse
     * too many shifts the zero into the tail. The count is asserted exactly because both
     * neighbouring values are wrong in ways the end state hides. */
    zassert_equal(count_op(c, op::pulse), 4, "four pulses for six positions, no more and no fewer");
    zassert_equal(count_op(c, op::data_high), 0,
                  "driving the data line HIGH would re-enable position 1 and inject a token");
    zassert_equal(c.log[0].what, op::data_low, "the data line goes low first, before any pulse");

    /* End state: only the tail. */
    for (size_t i = 0; i < 5; ++i)
        zassert_false(c.enabled[i], "position %u should be dark", (unsigned)(i + 1));
    zassert_true(c.enabled[5]);
    zassert_equal(c.addr[5], 0x2F, "and it never lost its address");
}

ZTEST(tof_tail_isolation, test_no_all_off_burst_is_issued)
{
    /* The wrong implementation of this function reaches the same end state with an all-off burst
     * followed by walking a single 1 down to the tail. The tail then answers the factory default
     * address, and the check it exists for silently becomes a check on nothing. The pulse count is
     * what separates them: an all-off spec asks for eight pulses, and injecting a token afterwards
     * needs five more. */
    fake_chain c{};
    const enm::chain_spec spec{product_spec()};
    pf::isolation_observation o{};

    zassert_equal(iso::observe_tail(c, spec, o), 0);
    zassert_true(count_op(c, op::pulse) < static_cast<int>(spec.alloff_pulses),
                 "the pulse count is consistent with an all-off burst having been issued");
    zassert_not_equal(o.answering_addr, enm::kDefaultAddr,
                      "the tail answered the default address: it had been reset");
}

ZTEST(tof_tail_isolation, test_the_merge_shows_up_as_the_neighbours_address)
{
    /* The failure this whole check exists for, 4/4 reproducible on DS20001 before the 50 ohm
     * resistor: one pulse enabled two boards, both were written to one address. The tail is alive
     * and answering -- just not where it should be -- so reporting "the tail did not answer" would
     * send an operator looking for a dead board. */
    fake_chain c{};
    c.addr[5] = 0x2E; // position 6 was written to position 5's address
    const enm::chain_spec spec{product_spec()};
    pf::isolation_observation o{};

    zassert_equal(iso::observe_tail(c, spec, o), 0);

    zassert_equal(o.tail_probe, enm::probe_state::ack, "it answered, and that matters");
    zassert_equal(o.answering_addr, 0x2E, "on its neighbour's address");
    zassert_true(o.id_read_ok, "and the identity is read where it answered, not where we hoped");
    zassert_equal(o.seen.first, kL4Id.first);
}

ZTEST(tof_tail_isolation, test_a_silent_tail_is_reported_as_a_clean_nack)
{
    fake_chain c{};
    /* The board is not there. Not `enabled[5] = false`: that is the shift register's state, and the
     * next pulse would put a 1 straight back into it. */
    c.present[5] = false;
    const enm::chain_spec spec{product_spec()};
    pf::isolation_observation o{};

    zassert_equal(iso::observe_tail(c, spec, o), 0);
    zassert_equal(o.tail_probe, enm::probe_state::nack);
    zassert_equal(o.answering_addr, 0);
    zassert_false(o.id_read_ok, "there was nothing to read an identity from");
}

ZTEST(tof_tail_isolation, test_a_transport_error_is_not_flattened_into_silence)
{
    /* A clean NACK means the board is silent; a transport error means the bus could not tell us
     * either way. The evaluator refuses those for different reasons, so they must not arrive as
     * the same value. */
    fake_chain c{};
    c.probe_transport_error = true;
    const enm::chain_spec spec{product_spec()};
    pf::isolation_observation o{};

    zassert_equal(iso::observe_tail(c, spec, o), 0);
    zassert_equal(o.tail_probe, enm::probe_state::transport_error);
    zassert_equal(o.prev_probe, enm::probe_state::transport_error);
    zassert_equal(o.answering_addr, 0);
}

ZTEST(tof_tail_isolation, test_a_neighbour_that_still_answers_is_recorded)
{
    /* The isolation did not take. Recorded rather than judged here -- the evaluator is what turns
     * it into a refusal -- but it has to be visible: with the neighbour awake, the tail answering
     * its own address proves nothing. */
    fake_chain c{};
    c.enable_is_reset_class = true;
    const enm::chain_spec spec{product_spec()};
    pf::isolation_observation o{};

    /* A chain whose shift register does not shift: the pulses do nothing, so everything stays
     * enabled. */
    c.pulse_rc = 0;
    struct stuck_chain final : fake_chain {
        int pulse_clock() override
        {
            note(op::pulse);
            ++pulses_seen;
            return 0; // acknowledged, and nothing moves
        }
    } stuck{};

    zassert_equal(iso::observe_tail(stuck, spec, o), 0);
    zassert_equal(o.tail_probe, enm::probe_state::ack);
    zassert_equal(o.answering_addr, 0x2F);
    zassert_equal(o.prev_probe, enm::probe_state::ack, "the neighbour is still awake");
}

ZTEST(tof_tail_isolation, test_a_control_failure_reports_no_observation_at_all)
{
    /* If the lines could not be driven, the enable state is unknown -- and an unknown enable state
     * has nothing to say about addressing. `attempted` stays false so the proof refuses rather
     * than interpreting an observation of an unknown chain. */
    fake_chain data_fails{};
    data_fails.set_data_rc = -EIO;
    const enm::chain_spec spec{product_spec()};
    pf::isolation_observation o{};

    zassert_equal(iso::observe_tail(data_fails, spec, o), -EIO);
    zassert_false(o.attempted);
    zassert_equal(count_op(data_fails, op::probe), 0, "nothing may be probed after a control failure");

    fake_chain pulse_fails{};
    pulse_fails.fail_pulse_at = 1;
    pf::isolation_observation o2{};
    zassert_equal(iso::observe_tail(pulse_fails, spec, o2), -EIO);
    zassert_false(o2.attempted);
    zassert_equal(count_op(pulse_fails, op::probe), 0);
}

ZTEST(tof_tail_isolation, test_a_chain_too_short_to_isolate_is_refused)
{
    fake_chain c{};
    enm::chain_spec spec{product_spec()};
    spec.positions = 1;
    pf::isolation_observation o{};

    zassert_equal(iso::observe_tail(c, spec, o), -EINVAL);
    zassert_false(o.attempted);
    zassert_equal(c.logged, 0, "a refused spec must not touch the chain");
}

ZTEST(tof_tail_isolation, test_the_observation_feeds_the_evaluator_unchanged)
{
    /* The two halves fit together: what this records is exactly what the proof's evaluator reads.
     * A healthy isolation produces the values its isolation checks accept, and the merge produces
     * the one they refuse -- which is the only end-to-end statement worth making here, since the
     * evaluator has its own suite. */
    fake_chain healthy{};
    const enm::chain_spec spec{product_spec()};
    pf::isolation_observation good{};
    zassert_equal(iso::observe_tail(healthy, spec, good), 0);
    zassert_equal(good.answering_addr, spec.at[5].target_addr);
    zassert_equal(good.prev_addr, spec.at[4].target_addr);
    zassert_equal(good.prev_probe, enm::probe_state::nack);

    fake_chain merged{};
    merged.addr[5] = spec.at[4].target_addr;
    pf::isolation_observation bad{};
    zassert_equal(iso::observe_tail(merged, spec, bad), 0);
    zassert_not_equal(bad.answering_addr, spec.at[5].target_addr);
}
