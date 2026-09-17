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
 * TWO KINDS OF HEALTH FRAME, AND THE DIFFERENCE IS NORMATIVE
 *
 * The timer heartbeat describes NO cycle: cycle_valid clear, cycle_seq zero, both per-cycle
 * masks zero. It carries the mapping state, the chain flags, the failing position and the
 * enumeration masks, and it keeps flowing from startup, through bring-up failure, and while no
 * acquisition runs at all -- which is what makes it useful during UNKNOWN.
 *
 * The cycle health frame describes exactly one completed cycle: cycle_valid set, cycle_seq
 * naming that cycle, and the per-cycle masks describing it. It is built from the values
 * LATCHED for that cycle, never from a fresh read of the mapping at send time. That is what
 * lets it arrive after a newer heartbeat reporting LOST and still legitimately authorise its
 * own cycle -- the consumer correlates on (epoch, cycle_seq), not on arrival order.
 *
 * TWO FIELDS STILL CANNOT BE FILLED HONESTLY
 *
 *   - flags bit 2, "a transport-level bus fault was seen": the acquisition snapshot's error
 *     bit is transport OR protocol OR usage, so it cannot substantiate a specifically
 *     transport claim. It comes from the authority's chain flags instead, and the authority
 *     only sets it from an enumeration result.
 *   - the per-source distinction between an I2C failure and a no-update, which the contract
 *     itself does not carry: it is separated only at chain level by flags bit 2.
 *
 * MEASUREMENTS FIRST, AND A FAILED SEND CANCELS THE CYCLE HEALTH
 *
 * sample_produced_mask asserts that a measurement frame for that source exists in that cycle,
 * and the consumer cross-checks both directions of it. So the mask can only be built from
 * sends that actually succeeded, and if ANY measurement of the cycle failed to reach the bus,
 * the cycle health frame is not sent at all. Withholding it is the safe direction: the
 * measurements that did go out are left without their authorising health frame and a
 * conforming consumer will not accept them, which is exactly right -- the alternative is a
 * frame claiming four samples when three are on the wire, a contradiction the contract forbids
 * outright.
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

/* Read ONCE per cycle, and carrying every health field that comes from the mapping, not just
 * the two the gate needs.
 *
 * The reason it grew: the authority publishes state, epoch, the enumeration masks, the chain
 * flags and the failing position as one atomic snapshot precisely so a reader cannot pair
 * fields that were never true together. A publisher that took the state from here and then
 * asked for the masks separately would reintroduce the tear at the consumer's end -- LOST with
 * a new epoch, or an enumeration mask from after a revocation attached to a state from before
 * it. So the whole snapshot travels as one value. */
struct authorisation {
    tof_acq::mapping_state state{tof_acq::mapping_state::not_ready};
    uint8_t epoch{0};
    /* Last enumeration attempt, keyed by source_id. Meaningful with cycle_valid clear, which
     * is what makes an UNKNOWN heartbeat worth sending. */
    uint8_t enumerated_mask{0};
    uint8_t model_verified_mask{0};
    uint8_t chain_flags{0}; // contract flags bits 0-2; bit 3 is cycle_valid and not from here
    uint8_t failing_position{tof_cliff_contract::kChainPositionNone};
};

struct config {
    struct can_sink sink{};
    /* The acquisition descriptor table, read for `kind` and `role_id` only and never
     * written. Used to cross-check the facts the sink was handed against the descriptor
     * for that index: a mismatch means the sink was called with arguments that do not
     * belong together, which is a wiring defect and not something a sensor can cause. */
    const tof_acq::source_desc *sources{nullptr};
    int source_count{0};
    /* Production MUST build this from ONE tof_authority::current() with the clamp applied to
     * its state. Publication is allowed only for PROVEN, which the clamp removes
     * unconditionally. See the notes above. */
    struct authorisation (*authorise)(){nullptr};
};

/* Counters, not logs: a host test can assert on them, and they keep the three reasons a
 * measurement was not sent distinct from the one reason a send failed. */
struct counters {
    uint32_t measurements_sent{0};
    /* Timer heartbeats and cycle health frames are counted apart. They answer different
     * questions -- "is the producer alive" and "did that cycle complete on the wire" -- and one
     * number for both would hide a subsystem whose heartbeat runs while no cycle ever
     * completes. */
    uint32_t health_sent{0};
    uint32_t cycle_health_sent{0};
    /* A cycle whose measurements were sent but whose health frame was deliberately withheld
     * because at least one of those sends failed. Not an error of this layer: it is the safe
     * outcome, and it is counted so it cannot be silent. */
    uint32_t cycle_health_withheld{0};
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
    /* A sample arrived for a cycle nobody announced. on_cycle_begin is what latches the
     * authorisation, so this is a wiring defect rather than anything a sensor can cause. */
    uint32_t suppressed_cycle_not_begun{0};
    /* Cycles whose health frame was withheld because something STRUCTURAL went wrong in them:
     * a packer refusal, a full queue, a stale queue entry, a role or model mismatch. Kept apart
     * from cycle_health_withheld, which counts the transport case. Both drop the whole cycle;
     * they differ in who has a defect. */
    uint32_t cycles_invalid{0};
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
void on_cycle_begin(uint32_t cycle_seq);
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
