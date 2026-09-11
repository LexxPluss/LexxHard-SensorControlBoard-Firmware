/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/*
 * Tail isolation: the middle step of the mapping proof's transaction.
 *
 * It leaves ONLY the last chain position enabled and records what answers, which is the one
 * check that catches a silent merge -- two devices written to one address by a clock pulse that
 * enabled two boards at once. That failure was 4/4 reproducible on DS20001 before the 50 ohm
 * series resistor, and six matching identity reads did not catch it.
 *
 * THERE IS EXACTLY ONE SEQUENCE THAT WORKS, AND THE OBVIOUS ONE DESTROYS THE EVIDENCE
 *
 * The enable chain is a shift register: position 1's enable IS the data line, and each board's
 * flip-flop forwards its own enable to the next on a shared clock. The obvious way to reach "only
 * the tail enabled" is an all-off burst followed by walking a single 1 down to the tail. That
 * sequence is wrong, and quietly so:
 *
 *   the L4 enable is RESET-CLASS. A position that has been disabled comes back at the factory
 *   default address, so a tail reached that way answers 0x29 -- and "the tail answers its OWN
 *   address" becomes unobservable. The check would still pass or fail, on nothing.
 *
 * The sequence that works starts from a chain where every position is enabled, which is exactly
 * the state a completed enumeration leaves behind: drive the data line LOW and issue
 * (positions - 2) clock pulses.
 *
 * TWO, not one. Position 1's enable IS the data line, so driving it low darkens position 1 with no
 * pulse at all; the pulses darken positions 2..N-1 -- four of them for a six-position chain. A
 * fifth would shift the zero into the tail and turn off the very board being isolated, which is
 * what the first version of this function did. THE TAIL IS NEVER DISABLED, and that is the only
 * reason it still holds the address the enumeration gave it.
 *
 * The cost is stated in the contract rather than hidden here: the isolation destroys the
 * addresses of every position it darkens, so a second enumeration must follow before any
 * measurement can be taken. That is why the proof is a transaction rather than three checks.
 *
 * WHAT IT DOES NOT DECIDE
 *
 * Nothing. It fills in observations and returns; tof_mapping_proof decides what they mean. The
 * split exists because a component that both gathers evidence and grades it can always be
 * reduced to the grade.
 */

#include <stdint.h>

#include "tof_enumerator.hpp"
#include "tof_mapping_proof.hpp"

namespace lexxhard::tof_isolate {

/*
 * Runs the isolation and fills `out`.
 *
 * PRECONDITION: every position is enabled and addressed, i.e. a `complete` enumeration has just
 * finished and nothing has touched the chain since. This function cannot verify that -- there is
 * no feedback line on the enable chain, only what was commanded -- so the caller owns it. The
 * commissioning session is what makes it true by holding the chain for walk, isolation and walk.
 *
 * Returns 0 when the sequence was carried out, whatever it observed: a chain that answers wrongly
 * is a successful observation of a broken chain, and the evaluator's job to refuse. Returns a
 * negative errno only when a CONTROL operation failed, in which case `out.attempted` stays false
 * -- the enable state is then unknown, so there is no observation to report and the proof must
 * refuse rather than interpret one.
 *
 * -EINVAL for a spec with fewer than two positions: with no neighbour there is nothing to prove
 * silent, and the evaluator refuses such a spec anyway.
 */
int observe_tail(tof_enum::chain_ops &ops, const tof_enum::chain_spec &spec,
                 tof_proof::isolation_observation &out);

}  // namespace lexxhard::tof_isolate
