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
 * NOTHING IS SENT UNDER THE CHAIN LOCK
 *
 * tof_acquisition calls on_cliff_sample from inside run_cycle(), which holds the chain
 * lock. A synchronous CAN send there would let bus backpressure block the whole I2C
 * schedule -- one slow arbitration and every sensor's next read is late. So a sample is
 * encoded into a bounded pending buffer under the lock and nothing touches the bus until
 * on_cycle_complete(), which the acquisition layer calls after unlocking.
 *
 * Frames are attempted exactly once and then dropped, whatever the outcome. They are never
 * retried and never carried into the next cycle: a range that failed to send is a stale
 * range by the time the bus recovers, and "old values are never re-sent" is the one thing
 * the contract forbids outright. So the queue cannot grow across cycles, and queue_full
 * counts a defect rather than ordinary backpressure.
 *
 * Health is sent directly, because it runs on its own work item with no lock held. That is
 * the whole reason the contract puts it on a separate timer.
 *
 * NO CAN DEPENDENCY HERE, ON PURPOSE
 *
 * The sink is an injected function pointer, so this translation unit compiles and is fully
 * testable without Zephyr's CAN driver. The real glue is a separate commit and a separate
 * file, which is also what lets the flash cost of the glue be measured on its own.
 *
 * ONE AUTHORISATION PER CYCLE, RE-CHECKED BEFORE ANYTHING IS SENT
 *
 * The mapping state and the epoch are one value, read once at the start of a cycle and
 * shared by every frame that cycle produces. Reading per sensor would let four frames from
 * one cycle carry different epochs, which is a correlation the consumer is entitled to rely
 * on.
 *
 * That is still not enough on its own: encoding happens under the lock and sending happens
 * after it, so the mapping can be lost in between and an already-encoded frame would go out
 * authorised under a mapping that no longer holds. So the authorisation is read again before
 * the flush, and if it was revoked or the epoch moved, the WHOLE cycle is discarded. Not the
 * offending frame -- the cycle, because a partially published cycle is a correlation the
 * consumer cannot detect as broken.
 *
 * Health reads the same value and takes both halves from it. The state the sink passes in is
 * deliberately not used: taking the state from one source and the epoch from another is the
 * bug this exists to remove.
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

/* Read once per cycle, so no two frames of one cycle can disagree about either half.
 * Carries the state rather than a bool, because health needs the state and the epoch to
 * come from the same read. */
struct authorisation {
    tof_acq::mapping_state state{tof_acq::mapping_state::not_ready};
    uint8_t epoch{0};
};

struct config {
    struct can_sink sink{};
    /* The acquisition descriptor table, read for `kind` and `role_id` only and never
     * written. Used to cross-check the facts the sink was handed against the descriptor
     * for that index: a mismatch means the sink was called with arguments that do not
     * belong together, which is a wiring defect and not something a sensor can cause. */
    const tof_acq::source_desc *sources{nullptr};
    int source_count{0};
    /* Production MUST return {tof_acq::effective_mapping_state(), <the proving epoch>}
     * read together. Publication is allowed only for PROVEN, which that function clamps
     * away unconditionally. See the notes above. */
    struct authorisation (*authorise)(){nullptr};
};

/* Counters, not logs: a host test can assert on them, and they keep the three reasons a
 * measurement was not sent distinct from the one reason a send failed. */
struct counters {
    uint32_t measurements_sent{0};
    uint32_t health_sent{0};
    /* Suppressed before the packer ran. */
    uint32_t suppressed_not_proven{0};
    uint32_t suppressed_wrong_model{0};
    /* The facts disagreed with the descriptor for that index. A wiring defect. */
    uint32_t suppressed_role_mismatch{0};
    /* Suppressed by the packer: no frame is owed, or the input was unencodable. */
    uint32_t suppressed_no_frame{0};
    uint32_t suppressed_packer_refused{0};
    /* The frame was encoded and there was no room to queue it. Cannot happen while the
     * queue is drained every cycle and holds one slot per source, so this counts a defect. */
    uint32_t queue_full{0};
    /* Whole cycles thrown away at the flush, because the authorisation was revoked or its
     * epoch moved between encoding and sending. Counted in cycles, not frames: the unit
     * that was discarded is the cycle. */
    uint32_t cycles_discarded_unauthorised{0};
    /* Frames found queued from a cycle other than the one being flushed. The queue is
     * per cycle by construction, so this counts a defect. */
    uint32_t discarded_stale_cycle{0};
    /* The frame was built and the transport rejected it. Deliberately separate from every
     * counter above: "the sensor said something we will not send" and "the bus would not
     * take it" have nothing to do with each other. */
    uint32_t send_failed_measurement{0};
    uint32_t send_failed_health{0};
    /* The health fields contradicted the contract, so the packer refused to encode them.
     * Kept apart from send_failed_health: one is a defect in what this layer assembled, the
     * other is the bus refusing a correct frame. */
    uint32_t health_encode_refused{0};
    /* The last packer refusal reason, for triage. */
    tof_cliff_packer::reason last_refusal{tof_cliff_packer::reason::none};
};

/* Returns -EINVAL for a missing sink, authorisation callback or source table. */
int init(const struct config &cfg);

/* Wire these to tof_acq::sinks. Signatures match on purpose, so the wiring is a
 * one-liner and there is nowhere to insert a transformation. */
void on_cliff_sample(int index, uint32_t cycle_seq, const tof_acq::source_facts &facts,
                     const struct tof_cliff_sample &sample);
/* Wire to tof_acq::sinks::on_cycle. Drains the pending buffer; this is where the bus is
 * actually touched, and it runs after the chain lock has been released. */
void on_cycle_complete(const tof_acq::cycle_facts &facts);
void on_cliff_health(uint32_t snapshot, tof_acq::mapping_state state);

void copy_counters(struct counters &out);

/* The contract's mapping_state for one of the acquisition layer's. Exposed because a
 * straight cast is a real trap: the two enumerations disagree on every value except zero,
 * so casting tof_acq::mapping_state::fault would report the contract's PROVEN. */
uint8_t wire_mapping_state(tof_acq::mapping_state state);

}  // namespace lexxhard::tof_cliff_pub

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
