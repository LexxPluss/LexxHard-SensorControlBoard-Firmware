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

#include <zephyr/kernel.h>

namespace lexxhard::tof_cliff_pub {

namespace {

namespace ctr = tof_cliff_contract;
namespace pk = tof_cliff_packer;

config cfg_{};
counters counters_{};
bool ready_{false};

/* Two contexts reach this layer: the acquisition cycle, and the health work item on its own
 * timer. They share the queue, the latched authorisation and the counters, so all of it is
 * behind one mutex.
 *
 * Lock order is always chain_lock -> this, never the reverse: on_cliff_sample is called with
 * the chain lock held and takes this one, while the health work item and copy_counters take
 * only this one. Nothing here ever reaches for the chain lock.
 *
 * The bus is NOT touched while this is held. A can_send can block for as long as its timeout,
 * and holding a shared mutex across it would let a slow arbitration on the measurement path
 * stall the heartbeat -- which is the one thing that must keep flowing. So the state is
 * snapshotted under the lock, the sends happen outside it, and the counters are updated under
 * it again.
 *
 * The consequence, stated rather than glossed: the SENDS ARE NOT SERIALISED. A measurement
 * send and a health send can be in the sink at the same time, which is what makes the sink's
 * own thread safety a requirement rather than a convenience, and which means there is no
 * ordering guarantee between a health frame and the measurements of the cycle it describes.
 * The wire contract already allows that interleaving. */
K_MUTEX_DEFINE(lock_);

struct guard {
    guard() { k_mutex_lock(&lock_, K_FOREVER); }
    ~guard() { k_mutex_unlock(&lock_); }
    guard(const guard &) = delete;
    guard &operator=(const guard &) = delete;
};

/* One slot per source is enough by construction: at most one measurement per source per
 * cycle, and the queue is drained at the end of every cycle. Sized to kMaxSources rather
 * than to the cliff count so a table with more cliff sources cannot silently overflow it. */
struct pending_frame {
    /* The FULL cycle number, not the wire byte. The flush accepts only frames from the
     * cycle it was called for, and a truncated value would alias every 256th cycle. */
    uint32_t cycle_seq;
    uint16_t can_id;
    uint8_t dlc;
    uint8_t data[8];
};
pending_frame queue_[tof_acq::kMaxSources]{};
int queued_{0};

/* Latched once per cycle and shared by every frame it produces. */
bool latched_{false};
uint32_t latched_cycle_{0};
authorisation latched_auth_{};

bool allowed(const authorisation &a)
{
    return a.state == tof_acq::mapping_state::proven;
}

bool enqueue(uint32_t cycle_seq, uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (queued_ >= static_cast<int>(sizeof queue_ / sizeof queue_[0]))
        return false;
    pending_frame &f{queue_[queued_++]};
    f.cycle_seq = cycle_seq;
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
     *   tof_acq   not_ready = 0   fault = 1   proven = 2   lost = 3
     *   contract  UNKNOWN   = 0   PROVEN = 1  LOST  = 2    FAULT = 3
     *
     * so casting `fault` would put PROVEN on the wire, casting `proven` would put LOST, and
     * casting `lost` would put FAULT. That is a silent mis-report of the one field the
     * consumer gates on -- and note that every one of those three mistakes is wrong in a
     * direction the consumer acts on. */
    switch (state) {
    case tof_acq::mapping_state::proven:
        return 0x1; // PROVEN
    case tof_acq::mapping_state::lost:
        return 0x2; // LOST
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

    const guard held;
    cfg_ = cfg;
    counters_ = counters{};
    queued_ = 0;
    latched_ = false;
    latched_cycle_ = 0;
    latched_auth_ = authorisation{};
    ready_ = true;
    return 0;
}

void on_cliff_sample(int index, uint32_t cycle_seq, const tof_acq::source_facts &facts,
                     const struct tof_cliff_sample &sample)
{
    const guard held;
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

    /* One authorisation for the whole cycle. Reading it per sensor would let four frames of
     * one cycle carry different epochs, and the consumer correlates on exactly that. A new
     * cycle number is the only signal available for "the cycle changed", because the
     * acquisition layer has no begin-of-cycle hook. */
    if (!latched_ || latched_cycle_ != cycle_seq) {
        /* Anything still queued belongs to a cycle that never got flushed. Dropping it is
         * the same rule as everywhere else: an unsent range is a stale range. */
        if (queued_ > 0) {
            counters_.discarded_stale_cycle += static_cast<uint32_t>(queued_);
            queued_ = 0;
        }
        latched_ = true;
        latched_cycle_ = cycle_seq;
        latched_auth_ = cfg_.authorise();
    }
    const authorisation auth{latched_auth_};
    if (!allowed(auth)) {
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
    if (!enqueue(cycle_seq, ctr::kMeasId, frame, ctr::kDlc))
        counters_.queue_full++;
}

void on_cycle_complete(const tof_acq::cycle_facts &facts)
{
    /* Snapshot under the lock, send outside it. Holding a mutex the health work item also
     * needs, across a can_send that can block for its whole timeout, would let a slow
     * arbitration on the measurement path stall the heartbeat. */
    pending_frame outgoing[tof_acq::kMaxSources];
    int count{0};
    struct can_sink sink{};
    uint32_t stale{0};

    {
        const guard held;
        if (!ready_)
            return;
        if (queued_ == 0) {
            latched_ = false;
            return;
        }

        /* Re-read the authorisation. Encoding happened under the chain lock and this runs
         * after it, so the mapping may have been lost in between -- and an already-encoded
         * frame would otherwise go out authorised under a mapping that no longer holds. If it
         * was revoked, or the epoch moved, the WHOLE cycle goes: a partially published cycle
         * is a broken correlation the consumer has no way to detect. */
        const authorisation now{cfg_.authorise()};
        if (!allowed(now) || now.epoch != latched_auth_.epoch) {
            counters_.cycles_discarded_unauthorised++;
            queued_ = 0;
            latched_ = false;
            return;
        }

        for (int i = 0; i < queued_; ++i) {
            /* The queue is per cycle by construction, so a frame from another cycle is a
             * defect rather than something to publish late. */
            if (queue_[i].cycle_seq != facts.cycle_seq) {
                ++stale;
                continue;
            }
            outgoing[count++] = queue_[i];
        }
        counters_.discarded_stale_cycle += stale;
        queued_ = 0;
        latched_ = false;
        sink = cfg_.sink;
    }

    /* Each frame is offered exactly once and then dropped whatever happens: carrying a
     * failed range into the next cycle would put a stale distance on the wire. */
    uint32_t sent{0}, failed{0};
    for (int i = 0; i < count; ++i) {
        const pending_frame &f{outgoing[i]};
        if (sink.send(f.can_id, f.data, f.dlc) != 0)
            ++failed;
        else
            ++sent;
    }

    const guard held;
    counters_.measurements_sent += sent;
    counters_.send_failed_measurement += failed;
}

void on_cliff_health(uint32_t snapshot, tof_acq::mapping_state state)
{
    uint8_t frame[8]{};
    struct can_sink sink{};

    {
        const guard held;
        if (!ready_)
            return;

    /* Both halves from one read. The `state` parameter is deliberately unused: taking the
     * state from one source and the epoch from another is exactly the disagreement this
     * layer exists to avoid, and the sink's shape is the acquisition layer's rather than a
     * second authority. */
    (void)state;
    const authorisation auth{cfg_.authorise()};

    pk::health_fields h{};
    h.mapping_epoch = auth.epoch;
    h.health_seq = static_cast<uint8_t>(counters_.health_sent & 0xFF);
    h.mapping_state = wire_mapping_state(auth.state);
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

        if (!pk::encode_health(h, frame)) {
            /* A defect in the fields assembled just above, not the bus refusing a correct
             * frame. Counted apart from send_failed_health for exactly that reason. */
            counters_.health_encode_refused++;
            return;
        }
        sink = cfg_.sink;
    }

    const int rc{sink.send(ctr::kHealthId, frame, ctr::kDlc)};

    const guard held;
    if (rc != 0)
        counters_.send_failed_health++;
    else
        counters_.health_sent++;
}

void copy_counters(struct counters &out)
{
    /* Not a bare copy: two contexts write these, so an unsynchronised read could tear a
     * multi-word struct and report a state that never existed. */
    const guard held;
    out = counters_;
}

} // namespace lexxhard::tof_cliff_pub

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
