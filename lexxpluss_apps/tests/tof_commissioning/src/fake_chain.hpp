/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The chain as the hardware behaves, shared by every suite in this test application.
 *
 * Extracted rather than copied. Two suites now need a chain that can be enumerated -- the
 * commissioning transaction and the runtime bootstrap -- and a second copy of a fake this detailed
 * would drift: the L4/L7 reset-class asymmetry and the tail-races-ahead defect are the parts the
 * verdicts depend on, so two versions of them would mean two different definitions of what the
 * hardware does.
 *
 * Each translation unit that includes this gets its OWN instance, which is what keeps the suites
 * isolated; only the model is shared.
 */

#pragma once

#include <errno.h>

#include <zephyr/kernel.h>

#include "tof_chain_spec.hpp"
#include "tof_enumerator.hpp"

namespace fake {

namespace enm = lexxhard::tof_enum;

constexpr enm::id_bytes kL4Id{0xeb, 0xaa};
constexpr enm::id_bytes kL7Id{0xf0, 0x02};

struct fake_chain final : enm::chain_ops {
    static constexpr size_t kStages{6};

    bool present[kStages]{true, true, true, true, true, true};
    bool enabled[kStages]{false, false, false, false, false, false};
    uint8_t addr[kStages]{0x29, 0x29, 0x29, 0x29, 0x29, 0x29};
    bool l7[kStages]{true, true, false, false, false, false};
    bool data{false};

    /* L7 keeps its address across an enable-low while powered; L4's enable is reset-class. Modelling
     * both is what makes walk 2 report `retained` for the grid sensors and `enumerated` for the
     * cliff ones -- the exact asymmetry the fingerprint has to normalise. */
    bool reset_class(size_t i) const { return !l7[i]; }

    int set_data_rc{0};
    int pulse_rc{0};
    int fail_pulse_at{-1};
    int pulses_seen{0};
    /* One clock pulse enables TWO stages at the last hop -- the DS20001 defect, 4/4 reproducible
     * before the 50 ohm series resistor. Modelled in the shift rather than in readdress(), because
     * that is where it happens: the data edge beat the clock edge into the last flip-flop. */
    bool tail_races_ahead{false};
    /* Probes fail only while exactly this many pulses have been issued. An exact count rather than
     * "from here on", because the isolation and walk 2 share this probe: a fault that persisted would
     * break walk 2 as well, and the evaluator checks walk 2 BEFORE the isolation -- so the test would
     * pass on the wrong refusal and prove nothing about the isolation. */
    int error_probes_at_pulse_count{-1};

    int set_data(bool level) override
    {
        if (set_data_rc != 0)
            return set_data_rc;
        data = level;
        apply(0, level);
        return 0;
    }

    void apply(size_t i, bool level)
    {
        const bool was{enabled[i]};
        enabled[i] = level;
        if (was && !level && reset_class(i))
            addr[i] = enm::kDefaultAddr;
    }

    int pulse_clock() override
    {
        if (fail_pulse_at >= 0 && pulses_seen == fail_pulse_at) {
            ++pulses_seen;
            return -EIO;
        }
        ++pulses_seen;
        if (pulse_rc != 0)
            return pulse_rc;
        for (size_t i{kStages - 1}; i > 0; --i)
            apply(i, enabled[i - 1]);
        apply(0, data);
        if (tail_races_ahead)
            apply(kStages - 1, enabled[kStages - 2]);
        return 0;
    }

    int answerer(uint8_t addr7) const
    {
        for (size_t i{0}; i < kStages; ++i) {
            if (present[i] && enabled[i] && addr[i] == addr7)
                return static_cast<int>(i);
        }
        return -1;
    }

    enm::probe_result probe(uint8_t addr7) override
    {
        if (error_probes_at_pulse_count >= 0 && pulses_seen == error_probes_at_pulse_count)
            return {enm::probe_state::transport_error, -EIO};
        return answerer(addr7) >= 0 ? enm::probe_result{enm::probe_state::ack, 0}
                                    : enm::probe_result{enm::probe_state::nack, 0};
    }

    int read_id(enm::model, uint8_t addr7, enm::id_bytes &out) override
    {
        const int i{answerer(addr7)};
        if (i < 0)
            return -ENXIO;
        out = l7[i] ? kL7Id : kL4Id;
        return 0;
    }

    enm::readdress_result readdress(enm::model, uint8_t old7, uint8_t new7) override
    {
        const int i{answerer(old7)};
        if (i < 0)
            return {-ENXIO, enm::readdress_stage::collision};
        /* Every device currently answering the old address moves. That is what the bus does, and it
         * is how a merge becomes one address with two devices behind it. */
        for (size_t k{0}; k < kStages; ++k) {
            if (present[k] && enabled[k] && addr[k] == old7)
                addr[k] = new7;
        }
        (void)i;
        return {};
    }

    void wait(wait_reason) override {}
};

inline enm::chain_spec provable_spec()
{
    enm::chain_spec s{lexxhard::tof_chain::dasher_spec()};
    s.at[2].role = enm::l4_role::front_left;
    s.at[3].role = enm::l4_role::rear_left;
    s.at[4].role = enm::l4_role::rear_right;
    s.at[5].role = enm::l4_role::front_right;
    return s;
}

}  // namespace fake
