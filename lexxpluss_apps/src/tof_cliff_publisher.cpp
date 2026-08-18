/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_cliff_publisher.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <errno.h>
#include <string.h>

namespace lexxhard::tof_cliff_pub {

namespace {

namespace ctr = tof_cliff_contract;
namespace pk = tof_cliff_packer;

config cfg_{};
counters counters_{};
bool ready_{false};

/* One slot per source is enough by construction: at most one measurement per source per
 * cycle, and the queue is drained at the end of every cycle. Sized to kMaxSources rather
 * than to the cliff count so a table with more cliff sources cannot silently overflow it. */
struct pending_frame {
    uint16_t can_id;
    uint8_t dlc;
    uint8_t data[8];
};
pending_frame queue_[tof_acq::kMaxSources]{};
int queued_{0};

bool enqueue(uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (queued_ >= static_cast<int>(sizeof queue_ / sizeof queue_[0]))
        return false;
    pending_frame &f{queue_[queued_++]};
    f.can_id = can_id;
    f.dlc = dlc;
    for (int i = 0; i < 8; ++i)
        f.data[i] = data[i];
    return true;
}

/* The snapshot's bit layout is deliberately NOT restated here. This commit fills none of
 * the fields that would need it, and a private copy of a layout that lives in another
 * translation unit would drift with nothing to catch it. Whichever commit first needs the
 * per-source bits should export the shifts from tof_acquisition.hpp so there is one
 * definition, rather than a second one here. */

} // namespace

uint8_t wire_mapping_state(tof_acq::mapping_state state)
{
    /* Explicit, never a cast. The two enumerations agree on nothing but zero:
     *
     *   tof_acq   not_ready = 0   fault = 1   proven = 2
     *   contract  UNKNOWN   = 0   PROVEN = 1  LOST  = 2   FAULT = 3
     *
     * so casting `fault` would put PROVEN on the wire, and casting `proven` would put
     * LOST. That is a silent mis-report of the one field the consumer gates on.
     *
     * LOST has no acquisition equivalent: it means "was proven, then a sensor was lost at
     * runtime", which needs the state machine this layer does not have. Nothing here can
     * produce it, and inventing a mapping to it would be a guess. */
    switch (state) {
    case tof_acq::mapping_state::proven:
        return 0x1; // PROVEN
    case tof_acq::mapping_state::fault:
        return 0x3; // FAULT
    case tof_acq::mapping_state::not_ready:
    default:
        return 0x0; // UNKNOWN -- never proven since the last enumeration attempt
    }
}

int init(const struct config &cfg)
{
    if (cfg.sink.send == nullptr || cfg.authorise == nullptr || cfg.sources == nullptr)
        return -EINVAL;
    if (cfg.source_count <= 0 || cfg.source_count > tof_acq::kMaxSources)
        return -EINVAL;

    cfg_ = cfg;
    counters_ = counters{};
    queued_ = 0;
    ready_ = true;
    return 0;
}

void on_cliff_sample(int index, uint32_t cycle_seq, const tof_acq::source_facts &facts,
                     const struct tof_cliff_sample &sample)
{
    if (!ready_)
        return;
    if (index < 0 || index >= cfg_.source_count)
        return;

    /* Defence in depth, not the primary gate: tof_acquisition already calls this sink only
     * for a fresh sample of an l4_cliff source. Kept because the sink is a function pointer
     * -- a wiring mistake, or a future caller, would otherwise have an L7 stub sample
     * silently packed as a cliff range. */
    if (facts.kind != tof_acq::model::l4_cliff) {
        counters_.suppressed_wrong_model++;
        return;
    }

    /* The descriptor for this index must agree with the facts the sink was handed. A
     * disagreement means the two arguments do not belong together, which no sensor can
     * cause -- it is a wiring defect, and packing it would attach a range to a role that
     * did not produce it. */
    const tof_acq::source_desc &d{cfg_.sources[index]};
    if (d.kind != facts.kind || d.role_id != facts.role_id) {
        counters_.suppressed_role_mismatch++;
        return;
    }

    /* Gate and epoch read together, once. Two separate reads could straddle a mapping
     * change and authorise a frame under a mapping that no longer holds. */
    const authorisation auth{cfg_.authorise()};
    if (!auth.allowed) {
        /* Outside PROVEN a measurement frame carries a source_id that is this firmware's
         * unproven guess rather than a physical position, so there is no such thing as
         * trustworthy position-numbered production data. */
        counters_.suppressed_not_proven++;
        return;
    }

    const pk::reduction r{pk::reduce(sample)};
    if (r.outcome == pk::result::no_frame) {
        /* A stale sample or a surviving NO_SAMPLE status. Correct to send nothing. */
        counters_.suppressed_no_frame++;
        return;
    }
    if (r.outcome != pk::result::frame_ready) {
        counters_.suppressed_packer_refused++;
        counters_.last_refusal = r.why;
        return;
    }

    uint8_t frame[8]{};
    pk::reason why{pk::reason::none};
    if (!pk::encode_measurement(r, facts.role_id, auth.epoch,
                                static_cast<uint8_t>(cycle_seq & 0xFF), frame, &why)) {
        counters_.suppressed_packer_refused++;
        counters_.last_refusal = why;
        return;
    }

    /* Queued, not sent. This runs under the chain lock; touching the bus here would let
     * CAN backpressure stall every sensor's next read. */
    if (!enqueue(ctr::kMeasId, frame, ctr::kDlc))
        counters_.queue_full++;
}

void on_cycle_complete(const tof_acq::cycle_facts &facts)
{
    (void)facts;
    if (!ready_)
        return;

    /* The lock is released by the time this runs, so blocking here costs a late cycle
     * rather than a stalled I2C schedule. Each frame is offered exactly once and then
     * dropped whatever happens: carrying a failed range into the next cycle would put a
     * stale distance on the wire, which the contract forbids outright. */
    for (int i = 0; i < queued_; ++i) {
        const pending_frame &f{queue_[i]};
        if (cfg_.sink.send(f.can_id, f.data, f.dlc) != 0) {
            counters_.send_failed_measurement++;
            continue;
        }
        counters_.measurements_sent++;
    }
    queued_ = 0;
}

void on_cliff_health(uint32_t snapshot, tof_acq::mapping_state state)
{
    if (!ready_)
        return;

    pk::health_fields h{};
    /* The epoch comes from the same authorisation read as the gate would; health is sent
     * whether or not publication is allowed, so only the epoch half is used here. */
    h.mapping_epoch = cfg_.authorise().epoch;
    h.health_seq = static_cast<uint8_t>(counters_.health_sent & 0xFF);
    h.mapping_state = wire_mapping_state(state);
    h.failing_chain_position = ctr::kChainPositionNone;

    /* Everything below stays zero, and the header says why for each. In short: the
     * enumeration masks are not on this path, the per-cycle masks may not be set while
     * cycle_valid is clear, and the snapshot cannot substantiate the contract's
     * specifically-transport fault bit. The snapshot is therefore unused -- named in the
     * signature because the sink's shape is the acquisition layer's, and because the
     * commit that fills these fields will need it. */
    (void)snapshot;
    h.flags = 0; // no cycle_valid: this frame describes no cycle
    h.enumerated_mask = 0;
    h.model_verified_mask = 0;
    h.sample_produced_mask = 0;
    h.sensor_fault_mask = 0;
    h.cycle_seq = 0;

    uint8_t frame[8]{};
    if (!pk::encode_health(h, frame)) {
        /* A defect in the fields assembled just above, not the bus refusing a correct
         * frame. Counted apart from send_failed_health for exactly that reason. */
        counters_.health_encode_refused++;
        return;
    }
    if (cfg_.sink.send(ctr::kHealthId, frame, ctr::kDlc) != 0) {
        counters_.send_failed_health++;
        return;
    }
    counters_.health_sent++;
}

void copy_counters(struct counters &out)
{
    out = counters_;
}

} // namespace lexxhard::tof_cliff_pub

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
