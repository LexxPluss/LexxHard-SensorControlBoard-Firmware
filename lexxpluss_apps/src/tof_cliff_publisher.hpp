/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/*
 * Turns the acquisition layer's neutral facts into cliff CAN frames.
 *
 * Sits between tof_acquisition's sinks and a CAN sink, and does exactly two things: run a
 * cliff sample through the contract's packer and hand the bytes to the sink, and turn a
 * heartbeat snapshot into a health frame. It owns no state machine, no cycle correlation,
 * no staleness and no notion of READY -- those need timing values that are still
 * unresolved and event-multiset vectors that do not exist.
 *
 * NO CAN DEPENDENCY HERE, ON PURPOSE
 *
 * The sink is an injected function pointer, so this translation unit compiles and is fully
 * testable without Zephyr's CAN driver. The real glue is a separate commit and a separate
 * file, which is also what lets the flash cost of the glue be measured on its own.
 *
 * THE PUBLICATION GATE, AND WHY IT IS INJECTED
 *
 * A measurement frame may only be sent while the mapping is PROVEN, and
 * tof_acq::publication_allowed() is structurally false today -- effective_mapping_state()
 * clamps PROVEN unconditionally, with no flag to lift it. Production MUST wire the gate to
 * that function. It is injectable for exactly one reason: with the gate hard-wired shut,
 * the measurement path would have no way to be exercised at all, and an untested encoder
 * on a safety path is worse than an injectable predicate.
 *
 * What this deliberately is NOT: there is no build flag, no constant, no shell command and
 * no configuration value in production that opens the gate. The earlier
 * TOF_ACQ_CHAIN_HW_FIXED was removed for being exactly that, and nothing here reintroduces
 * it. The test suite asserts that the production gate is closed, so lifting the clamp
 * without review fails a test rather than shipping.
 *
 * WHAT THE HEALTH FRAME CANNOT SAY YET
 *
 * Most of the contract's health fields cannot be filled truthfully from what the
 * acquisition layer knows, and a fabricated field is worse than a zero one:
 *
 *   - enumerated_mask, model_verified_mask: enumeration is not on this path at all. Zero
 *     is what UNKNOWN means, and is consistent with it.
 *   - sample_produced_mask, sensor_fault_mask: per-cycle fields, and the contract forbids
 *     them being non-zero while cycle_valid is clear. Setting cycle_valid needs the cycle
 *     correlation this layer does not do.
 *   - flags bit 2, "a transport-level bus fault was seen": the snapshot's error bit is
 *     transport OR protocol OR usage, so it cannot substantiate a specifically transport
 *     claim.
 *   - failing_chain_position: NONE, for the same reason as the enumeration masks.
 *
 * So health is an honest UNKNOWN heartbeat. It keeps flowing from startup and through
 * bring-up failure, which is what the contract requires of it; it does not yet describe a
 * cycle, and does not pretend to.
 */

#include <cstdint>

#include "tof_acquisition.hpp"
#include "tof_cliff_contract.h"
#include "tof_cliff_packer.hpp"
#include "tof_cliff_sample.h"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

namespace lexxhard::tof_cliff_pub {

/* Where an encoded frame goes. Returns 0 on success; any other value is a transport
 * failure and is counted separately from a sensor or protocol failure, because they need
 * different follow-up and collapsing them would hide which one happened. */
struct can_sink {
    int (*send)(uint16_t can_id, const uint8_t *data, uint8_t dlc);
};

struct config {
    struct can_sink sink{};
    /* The acquisition descriptor table, read for `kind` and `role_id` only and never
     * written. Needed because the snapshot's bits are per acquisition index while the
     * contract's masks are per cliff source_id; this is the only thing that bridges them,
     * and being immutable is what keeps the heartbeat off the acquisition path. */
    const tof_acq::source_desc *sources{nullptr};
    int source_count{0};
    /* Production MUST pass tof_acq::publication_allowed. See the note above. */
    bool (*publication_allowed)(){nullptr};
    /* The mapping epoch the frames are stamped with. Injected rather than counted here:
     * the epoch belongs to whatever proves the mapping, not to whatever publishes. */
    uint8_t (*mapping_epoch)(){nullptr};
};

/* Counters, not logs: a host test can assert on them, and they keep the three reasons a
 * measurement was not sent distinct from the one reason a send failed. */
struct counters {
    uint32_t measurements_sent{0};
    uint32_t health_sent{0};
    /* Suppressed before the packer ran. */
    uint32_t suppressed_not_proven{0};
    uint32_t suppressed_wrong_model{0};
    /* Suppressed by the packer: no frame is owed, or the input was unencodable. */
    uint32_t suppressed_no_frame{0};
    uint32_t suppressed_packer_refused{0};
    /* The frame was built and the transport rejected it. Deliberately separate from every
     * counter above: "the sensor said something we will not send" and "the bus would not
     * take it" have nothing to do with each other. */
    uint32_t send_failed_measurement{0};
    uint32_t send_failed_health{0};
    /* The last packer refusal reason, for triage. */
    tof_cliff_packer::reason last_refusal{tof_cliff_packer::reason::none};
};

/* Returns -EINVAL for a missing sink, gate, epoch provider or source table. */
int init(const struct config &cfg);

/* Wire these to tof_acq::sinks. Signatures match on purpose, so the wiring is a
 * one-liner and there is nowhere to insert a transformation. */
void on_cliff_sample(int index, uint32_t cycle_seq, const tof_acq::source_facts &facts,
                     const struct tof_cliff_sample &sample);
void on_cliff_health(uint32_t snapshot, tof_acq::mapping_state state);

void copy_counters(struct counters &out);

/* The contract's mapping_state for one of the acquisition layer's. Exposed because a
 * straight cast is a real trap: the two enumerations disagree on every value except zero,
 * so casting tof_acq::mapping_state::fault would report the contract's PROVEN. */
uint8_t wire_mapping_state(tof_acq::mapping_state state);

}  // namespace lexxhard::tof_cliff_pub

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
