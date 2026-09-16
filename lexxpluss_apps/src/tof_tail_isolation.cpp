/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_tail_isolation.hpp"

#include <errno.h>

namespace lexxhard::tof_isolate {

namespace enm = tof_enum;

int observe_tail(enm::chain_ops &ops, const enm::chain_spec &spec,
                 tof_proof::isolation_observation &out)
{
    out = tof_proof::isolation_observation{};

    if (spec.positions < 2 || spec.positions > enm::chain_spec::kMaxPositions)
        return -EINVAL;

    const size_t tail{spec.positions - 1};
    const uint8_t tail_addr{spec.at[tail].target_addr};
    const uint8_t prev_addr{spec.at[tail - 1].target_addr};

    /* Data low, then POSITIONS - 2 pulses. No all-off: see the header.
     *
     * Two rather than one, and the arithmetic is worth spelling out because the obvious
     * `positions - 1` turns the tail off as well. Position 1's enable IS the data line, so driving
     * it low darkens position 1 with no pulse at all. Each pulse then shifts that zero one stage
     * further, darkening positions 2..N-1 -- four pulses for a six-position chain. A fifth pulse
     * would shift the zero into the tail, and the tail losing its enable is precisely the thing
     * this sequence exists to avoid.
     *
     * That off-by-one was in the first version of this function, and it was caught by a test fake
     * that models the shift register rather than returning canned probe answers: the end state it
     * produced was "nothing enabled", which no amount of asserting on probe results would have
     * distinguished from a silent tail. */
    if (const int rc{ops.set_data(false)}; rc != 0)
        return rc;
    ops.wait(enm::chain_ops::wait_reason::data_settle);

    for (size_t i{0}; i + 2 < spec.positions; ++i) {
        if (const int rc{ops.pulse_clock()}; rc != 0)
            return rc;
    }

    /* Only from here is there an observation to report: the lines were driven as intended, so
     * whatever the bus says now is evidence. Before this point a failure leaves the enable state
     * unknown, and an unknown enable state has nothing to say about addressing. */
    out.attempted = true;
    out.prev_addr = prev_addr;

    ops.wait(enm::chain_ops::wait_reason::data_settle);

    /* Two probes, and the field names say what each one means.
     *
     * `tail_probe` is "the isolated device answered SOMEWHERE", and `answering_addr` is where.
     * That framing is deliberate: in the failure this check exists for, the tail answers its
     * NEIGHBOUR's address because both were written to it, and reporting that as "the tail did not
     * answer" would send an operator looking for a dead board instead of a merge. So the tail's
     * own address is tried first, then the neighbour's, and the address that answered is recorded
     * rather than assumed.
     *
     * `prev_probe` is the negative half and is about the neighbour's address specifically: it must
     * be silent, because an address that still answers while its device is supposed to be dark
     * means the isolation did not take -- and then the positive half proves nothing. */
    const enm::probe_result tail_at_own{ops.probe(tail_addr)};
    const enm::probe_result at_prev{ops.probe(prev_addr)};

    out.prev_probe = at_prev.state;

    if (tail_at_own.state == enm::probe_state::ack) {
        out.tail_probe = enm::probe_state::ack;
        out.answering_addr = tail_addr;
    } else if (at_prev.state == enm::probe_state::ack) {
        /* The merge signature. The tail is alive and answering, just not where it should be. */
        out.tail_probe = enm::probe_state::ack;
        out.answering_addr = prev_addr;
    } else {
        /* Nothing answered. Carry the tail's own probe state rather than flattening it: a clean
         * NACK means the board is silent, a transport error means the bus could not tell us
         * either way, and the evaluator refuses those for different reasons. */
        out.tail_probe = tail_at_own.state;
        out.answering_addr = 0;
    }

    if (out.answering_addr != 0) {
        enm::id_bytes seen{};
        /* Read the identity at the address that ANSWERED, not at the one we hoped for. Reading at
         * the expected address would fail with -ENXIO in the merge case and lose the very
         * distinction the isolation is for. */
        out.id_read_ok = ops.read_id(spec.at[tail].expected, out.answering_addr, seen) == 0;
        out.seen = seen;
    }

    return 0;
}

}  // namespace lexxhard::tof_isolate
