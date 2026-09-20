/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The grid transport: one complete 8x8 read becomes 16 data frames on
 * TOF_GRID_DATA_ID and the health frame that closes it on TOF_GRID_HEALTH_ID.
 *
 * WHAT THIS LAYER OWNS, and it is deliberately little: per-source generation
 * numbers, the recovered-flag accumulation the contract's health byte 3 is
 * defined in terms of, the queue that keeps CAN off the chain lock, and the
 * refusals. It owns no zone policy (the packer has it), no mapping (the
 * authority has it) and no scheduling (acquisition has it).
 *
 * WHAT IT IS NOT: 0x215 is not a heartbeat. The cliff pair has a health frame
 * on its own timer that keeps flowing when no cycle completes; this pair does
 * not, by contract -- "a sensor whose model ID did not verify produces no grids
 * and no health frames at all. It does not announce itself on this pair of
 * identifiers." A silent sensor is therefore invisible here on purpose, and
 * detecting it is the decoder's watchdog, not a frame this layer can send.
 */

#pragma once

/* THE CLIFF FLAG IS IN THE CONDITION, and not because this layer touches a VL53L4CX. It consumes
 * tof_acquisition.hpp, which is itself compiled only for ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD,
 * so a build with the grid and without the cliff would compile this file against an empty header
 * and fail somewhere unrelated. Stating the dependency here makes it visible instead. */
#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD) && defined(ENABLE_TOF_L7_ULD)

#include <stdint.h>

#include "tof_acquisition.hpp"
#include "tof_l7_sample.hpp"

namespace lexxhard::tof_grid_pub {

/* Where an encoded frame goes. Returns 0 on success; anything else is a transport failure,
 * counted apart from every sensor and protocol refusal because they need different
 * follow-up. */
struct can_sink {
    int (*send)(uint16_t can_id, const uint8_t *data, uint8_t dlc);
};

/* Read ONCE per cycle, at its start, and carrying every mapping- and chain-level field a
 * health frame reports -- not just the two the gate needs.
 *
 * One snapshot rather than several reads, for the same reason the cliff publisher takes one:
 * the fields are only meaningful together. A publisher that took `state` here and asked for
 * the chain flags separately could pair a PROVEN state from before a revocation with a chain
 * count from after it, and the consumer has no way to see that the two were never true at the
 * same moment.
 *
 * The two chain-level booleans are contract flag bits 2 and 3 and nothing else. Bit 3 is
 * "ANOTHER sensor on the chain failed enumeration": the failing sensor's own health frame can
 * never carry it, because a sensor that failed enumeration transmits nothing at all. */
struct authorisation {
    tof_acq::mapping_state state{tof_acq::mapping_state::not_ready};
    uint8_t epoch{0};
    // Health byte 4, high nibble. Diagnostics only; the contract says so explicitly.
    uint8_t boards_detected{0};
    bool chain_length_unexpected{false};            // flag bit 2
    bool other_position_enumeration_failed{false};  // flag bit 3
};

struct config {
    struct can_sink sink{};
    /* The descriptor table acquisition reads, so a sample's facts can be checked against the
     * descriptor for the same index. A disagreement is a wiring defect, and publishing it
     * would attach a grid to a source id that did not produce it. */
    const tof_acq::source_desc *sources{nullptr};
    int source_count{0};
    struct authorisation (*authorise)(){nullptr};
    /* The zone-confidence policy the packer applies: VL53L7CX target_status 6 and 9 are
     * low-confidence, and whether they are trusted is a product decision that belongs to
     * whoever configures this layer. There is no default here for the same reason the grid
     * frequency has none: a number invented at this level becomes the specification by being
     * the only one available. */
    bool accept_low_confidence{false};
};

struct counters {
    /* A grid that passed every gate, was packed and was queued. It has NOT necessarily
     * reached the bus -- grids_sent is that. */
    uint32_t grids_admitted{0};
    /* All 16 data frames and the health frame of one grid reached the bus. */
    uint32_t grids_sent{0};
    uint32_t data_frames_sent{0};
    uint32_t health_frames_sent{0};
    /* A generation that was allocated and will never be transmitted again: a send failed
     * part way through its grid, or its cycle was dropped after it was packed. It is counted
     * because the number a consumer sees jumping is this one, and an unexplained gap in
     * generations reads like lost frames. */
    uint32_t generations_retired{0};
    /* Frames of a grid that were not offered to the bus at all, because an earlier frame of
     * the SAME grid failed. Sending the rest would be bus traffic for a grid the consumer can
     * only discard. */
    uint32_t frames_abandoned{0};
    /* Suppressed before the packer ran. */
    uint32_t suppressed_not_proven{0};
    uint32_t suppressed_wrong_model{0};
    uint32_t suppressed_wrong_domain{0};
    uint32_t suppressed_role_mismatch{0};
    /* role_id is not a grid source id. The mapping install is what assigns one, so this counts
     * a grid whose position was never keyed -- and guessing it from the descriptor index is
     * exactly the mistake the contract's source_id -> physical position promise forbids. */
    uint32_t suppressed_role_unassigned{0};
    /* Two sources in one cycle claimed the same source id. One of them is mis-keyed, and
     * which one cannot be known here. */
    uint32_t suppressed_duplicate_source{0};
    /* The sample and the facts disagreed about whether a sample exists. */
    uint32_t suppressed_sample_contradiction{0};
    /* from_read() refused: the transmit obligation was not met. Its inputs are passed through
     * as they were observed, never softened to get past the gate, so this counts a real
     * refusal rather than a shape this layer could have avoided. */
    uint32_t suppressed_not_admitted{0};
    /* Encoded and no room to queue. The queue holds a whole cycle's worth of frames for every
     * grid source, so this counts a defect rather than back-pressure. */
    uint32_t queue_full{0};
    uint32_t suppressed_cycle_not_begun{0};
    /* Whole cycles dropped before anything was sent: something structural went wrong, so the
     * cycle costs every frame rather than only its health frames. */
    uint32_t cycles_invalid{0};
    /* Whole cycles dropped at the flush because the authorisation was revoked or its epoch
     * moved between packing and sending. */
    uint32_t cycles_discarded_unauthorised{0};
    uint32_t discarded_stale_cycle{0};
    uint32_t send_failed_data{0};
    uint32_t send_failed_health{0};
};

/* -EINVAL for a missing sink, authorisation callback or source table, or a source count
 * outside the table's bounds. */
int init(const struct config &cfg);

/* The three acquisition hooks. They are separate functions rather than a sinks struct because
 * the cliff publisher already owns those hook slots: whichever commit wires this up must fan
 * one call out to both publishers, NOT replace the cliff sink. */
void on_cycle_begin(uint32_t cycle_seq);
void on_grid_sample(int index, uint32_t cycle_seq, const tof_acq::source_facts &facts,
                    const tof_l7::sample &sample);
void on_cycle_complete(const tof_acq::cycle_facts &facts);

void copy_counters(struct counters &out);

/* THE last_error BYTE (health byte 5). The contract leaves it "device/driver specific, 0 =
 * none", so this is the firmware's definition of it: the tof_l7::stage number of the most
 * recent failed operation on that source since the last health frame that reported it. A
 * stage rather than a ULD status because one byte cannot carry both, and because the stage
 * says where it broke -- which is what a diagnostic byte is for. It is advisory: the decoder
 * is forbidden from gating on it.
 *
 * If a later consumer wants to decode it, the encoding moves into the contract document and
 * both sides pin it. Until then it stays here, where it cannot be mistaken for something the
 * driver is entitled to interpret. */

} // namespace lexxhard::tof_grid_pub

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD && ENABLE_TOF_L7_ULD
