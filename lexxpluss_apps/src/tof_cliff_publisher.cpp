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
    if (cfg.sink.send == nullptr || cfg.publication_allowed == nullptr ||
        cfg.mapping_epoch == nullptr || cfg.sources == nullptr)
        return -EINVAL;
    if (cfg.source_count <= 0 || cfg.source_count > tof_acq::kMaxSources)
        return -EINVAL;

    cfg_ = cfg;
    counters_ = counters{};
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

    /* The gate, before any encoding. Outside PROVEN a measurement frame carries a
     * source_id that is this firmware's unproven guess rather than a physical position,
     * so there is no such thing as trustworthy position-numbered production data. */
    if (!cfg_.publication_allowed()) {
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
    if (!pk::encode_measurement(r, facts.role_id, cfg_.mapping_epoch(),
                                static_cast<uint8_t>(cycle_seq & 0xFF), frame, &why)) {
        counters_.suppressed_packer_refused++;
        counters_.last_refusal = why;
        return;
    }

    if (cfg_.sink.send(ctr::kMeasId, frame, ctr::kDlc) != 0) {
        /* Counted apart from every suppression above: the frame was correct and the bus
         * would not take it, which is a transport problem and not a sensor one. */
        counters_.send_failed_measurement++;
        return;
    }
    counters_.measurements_sent++;
}

void on_cliff_health(uint32_t snapshot, tof_acq::mapping_state state)
{
    if (!ready_)
        return;

    pk::health_fields h{};
    h.mapping_epoch = cfg_.mapping_epoch();
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
        /* The packer refuses contradictory health, so reaching this is a defect in the
         * fields assembled just above rather than anything the sensors did. */
        counters_.send_failed_health++;
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
