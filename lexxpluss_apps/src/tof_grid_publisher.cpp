/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_grid_publisher.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD) && defined(ENABLE_TOF_L7_ULD)

#include <errno.h>

#include <zephyr/kernel.h>

#include "tof_can_ids.hpp"
#include "tof_grid_packer.hpp"
#include "tof_l7_status.hpp"

namespace lexxhard::tof_grid_pub {

namespace {

namespace gp = tof_grid;
namespace ids = tof_can_ids;

config cfg_{};
counters counters_{};
bool ready_{false};

/* Two contexts reach this layer: the acquisition cycle, and whoever reads the counters for
 * diagnostics. Everything below is behind one mutex.
 *
 * Lock order is chain_lock -> this, never the reverse: on_grid_sample is called WITH the chain
 * lock held and takes this one; nothing here ever reaches for the chain lock.
 *
 * THE BUS IS NOT TOUCHED WHILE THIS IS HELD, and for the grid path that is not a refinement.
 * can_send can block for its whole timeout, on_grid_sample runs inside the chain lock, and one
 * grid is 17 frames rather than one: sending there would hold the chain -- every sensor's next
 * read, and commissioning's only chance to take it -- across seventeen blocking sends. So the
 * frames are built under the lock and offered to the bus after it is dropped, at the end of the
 * cycle. */
K_MUTEX_DEFINE(lock_);

struct guard {
    guard() { k_mutex_lock(&lock_, K_FOREVER); }
    ~guard() { k_mutex_unlock(&lock_); }
    guard(const guard &) = delete;
    guard &operator=(const guard &) = delete;
};

/* Two grid sources, per the contract's source_id domain. Not a count of what the chain happens
 * to carry: a third grid source would be a contract change, and this array being the wrong size
 * is caught here rather than by writing past it. */
constexpr int kGridSources{2};
constexpr int kFramesPerGrid{static_cast<int>(gp::kDataFrames) + 1};
/* WORST CASE, WHOLE: every grid source producing a complete grid in one cycle. 34 frames. A
 * queue that fits one grid would work only until both sensors reported in the same cycle --
 * which is the normal case, not the corner. */
constexpr int kQueueFrames{kGridSources * kFramesPerGrid};

struct pending_frame {
    /* The FULL cycle number, not the wire byte: the flush accepts only frames from the cycle it
     * was called for, and a truncated value would alias every 256th cycle. */
    uint32_t cycle_seq;
    uint16_t can_id;
    uint8_t dlc;
    uint8_t data[8];
    uint8_t source_id;
    /* The frame that CLOSES a grid. It is sent last and only if every data frame of the same
     * grid was sent, because a health frame asserts a complete grid: sending it after a failed
     * data frame would claim 64 zones the consumer will never see. */
    bool is_health;
};
pending_frame queue_[kQueueFrames]{};
int queued_{0};

bool latched_{false};
uint32_t latched_cycle_{0};
authorisation latched_auth_{};
bool cycle_invalid_{false};
/* Which source ids have already been packed in this cycle. Two descriptors claiming one source
 * id is a keying defect, and publishing both would put two different sensors' grids on the wire
 * under the same physical position. */
uint8_t seen_sources_{0};

/* Per source, and independent by contract: "the two sources' counters are independent and
 * unsynchronised". Allocated when a grid is packed -- a generation is spent by being
 * transmitted, and a grid that failed part way is NOT RETRIED UNDER ITS OWN NUMBER: a consumer
 * that saw some of its frames would otherwise assemble the next grid's frames into a false
 * complete one under the same generation.
 *
 * That is a rule about retrying, not about the counter's range. The counter is a uint8 and wraps
 * modulo 256 exactly as the contract specifies -- a number comes round again after 255 others,
 * by which time the decoder has long retired it. */
uint8_t next_generation_[kGridSources]{};

/* Health byte 3, bits 0 and 1: "occurred AND RECOVERED since the previous health frame". They
 * are set when a cycle's read fails and reported on the next grid that succeeds -- which is
 * what makes the report true, since a grid exists only after a complete successful read. They
 * are cleared when the frame carrying them reaches the bus, not when it is built. */
uint8_t pending_flags_[kGridSources]{};
/* Health byte 5, this firmware's definition: the tof_l7::stage of the most recent failure.
 * Same lifetime rule as the flags. */
uint8_t pending_error_[kGridSources]{};

/* What this cycle's health frame claims for each source, kept so the clear can be exact: only
 * the bits that actually reached the bus are cleared, and a bit set after the frame was built
 * survives to be reported by the next one. */
bool reported_[kGridSources]{};
uint8_t reported_flags_[kGridSources]{};
uint8_t reported_error_[kGridSources]{};

bool allowed(const authorisation &a)
{
    return a.state == tof_acq::mapping_state::proven;
}

/* Every way this cycle's read could have failed, from the two places that record one: the
 * scheduler's four per-cycle outcome bits, and the adapter's own status. A grid whose read
 * failed cannot be admitted, and the point of passing this through rather than asserting it is
 * that the refusal happens in the packer's gate, where it is one rule for both publishers,
 * instead of being decided here. */
bool io_ok(const tof_acq::source_facts &f)
{
    return !f.transport_error && !f.protocol_error && !f.unsupported && !f.usage_error &&
           f.status.port_errno == 0 && f.status.uld_status == 0;
}

uint8_t flags_for(uint8_t src, const authorisation &auth)
{
    uint8_t flags{static_cast<uint8_t>(pending_flags_[src] & 0x03)};

    /* Bits 2 and 3 are chain-level and come from the mapping snapshot, so they are a statement
     * about this cycle rather than something accumulated: the chain is either the expected
     * length now or it is not. */
    if (auth.chain_length_unexpected)
        flags |= 1U << 2;
    if (auth.other_position_enumeration_failed)
        flags |= 1U << 3;
    return flags;
}

bool enqueue(uint32_t cycle_seq, uint16_t can_id, const uint8_t *data, uint8_t src, bool health)
{
    if (queued_ >= kQueueFrames)
        return false;
    pending_frame &f{queue_[queued_++]};
    f.cycle_seq = cycle_seq;
    f.can_id = can_id;
    f.dlc = 8;
    for (int i{0}; i < 8; ++i)
        f.data[i] = data[i];
    f.source_id = src;
    f.is_health = health;
    return true;
}

/* A generation that was allocated and will not be transmitted -- or not completely. Counted per
 * source so an unexplained jump in the numbers a consumer sees has something to be read against. */
void retire_packed_generations()
{
    for (int s{0}; s < kGridSources; ++s) {
        if (!reported_[s])
            continue;
        counters_.generations_retired++;
        reported_[s] = false;
    }
}

void reset_cycle()
{
    queued_ = 0;
    latched_ = false;
    cycle_invalid_ = false;
    seen_sources_ = 0;
}

/* This cycle's failures, for the NEXT grid's health frame. Runs for every completed cycle,
 * including one that was dropped: what happened on the chain is true whether or not the cycle
 * reached the bus. */
void accumulate_failures(const tof_acq::cycle_facts &facts)
{
    for (int i{0}; i < facts.source_count && i < tof_acq::kMaxSources; ++i) {
        const tof_acq::source_facts &f{facts.sources[i]};

        if (f.kind != tof_acq::model::l7_grid || !f.started)
            continue;
        if (f.role_id >= kGridSources)
            continue;   // never keyed: it cannot report anything, so nothing is accumulated
        if (io_ok(f))
            continue;

        const uint8_t src{f.role_id};

        /* Bit 0 is the I2C transfer error. The two sources of that truth are the scheduler's
         * classification of the return code and the port's own errno, and either alone would
         * miss half the cases. */
        if (f.transport_error || f.status.port_errno != 0)
            pending_flags_[src] |= 1U << 0;
        /* Bit 1 is the data-ready timeout. In this adapter the readiness check is the only
         * operation that can time out: read_once does ONE non-blocking check and returns, so a
         * sensor that is merely not ready yet is not a failure and never gets here. */
        if (f.status.domain == tof_acq::status_domain::l7 &&
            f.status.stage == static_cast<uint8_t>(tof_l7::stage::ready_check))
            pending_flags_[src] |= 1U << 1;

        if (f.status.domain == tof_acq::status_domain::l7)
            pending_error_[src] = f.status.stage;
    }
}

} // namespace

int init(const struct config &cfg)
{
    if (cfg.sink.send == nullptr || cfg.authorise == nullptr || cfg.sources == nullptr)
        return -EINVAL;
    if (cfg.source_count <= 0 || cfg.source_count > tof_acq::kMaxSources)
        return -EINVAL;

    const guard held;
    cfg_ = cfg;
    counters_ = counters{};
    reset_cycle();
    latched_cycle_ = 0;
    latched_auth_ = authorisation{};
    for (int s{0}; s < kGridSources; ++s) {
        /* Generations restart with the configuration. A consumer that reconnects after a
         * re-init must not be handed a number that continues a stream it never saw -- and the
         * decoder retires a generation on completion, so a repeat would be discarded rather
         * than published. */
        next_generation_[s] = 0;
        pending_flags_[s] = 0;
        pending_error_[s] = 0;
        reported_[s] = false;
        reported_flags_[s] = 0;
        reported_error_[s] = 0;
    }
    ready_ = true;
    return 0;
}

void on_cycle_begin(uint32_t cycle_seq)
{
    const guard held;
    if (!ready_)
        return;

    /* Anything still queued belongs to a cycle that was never flushed, and its generations are
     * spent. An unsent grid is a stale grid. */
    if (queued_ > 0) {
        counters_.discarded_stale_cycle += static_cast<uint32_t>(queued_);
        retire_packed_generations();
    }
    reset_cycle();

    latched_ = true;
    latched_cycle_ = cycle_seq;
    latched_auth_ = cfg_.authorise();
}

void on_grid_sample(int index, uint32_t cycle_seq, const tof_acq::source_facts &facts,
                    const tof_l7::sample &sample)
{
    const guard held;
    if (!ready_)
        return;
    if (index < 0 || index >= cfg_.source_count)
        return;

    /* The cycle must have been announced: the authorisation is latched by on_cycle_begin, so a
     * sample for an unannounced cycle has no authorisation to be published under, and inventing
     * one here is how a frame ends up carrying an epoch nobody proved. */
    if (!latched_ || latched_cycle_ != cycle_seq) {
        counters_.suppressed_cycle_not_begun++;
        return;
    }

    if (facts.kind != tof_acq::model::l7_grid) {
        counters_.suppressed_wrong_model++;
        cycle_invalid_ = true;
        return;
    }
    /* The diagnostic record must be written in this model's vocabulary. A cliff-domain status on
     * a grid sample means the two were paired by a wiring mistake, and its stage number would be
     * read in the wrong enumeration -- a plausible wrong answer rather than an obvious one. */
    if (facts.status.domain != tof_acq::status_domain::l7) {
        counters_.suppressed_wrong_domain++;
        cycle_invalid_ = true;
        return;
    }

    /* ONE authorisation for the whole cycle, taken at its start. Reading it per sensor would let
     * the two grids of one cycle carry different epochs, and the consumer correlates on exactly
     * that.
     *
     * Checked BEFORE the role, deliberately: outside PROVEN the descriptors are not keyed, so an
     * unassigned role there is the ordinary state of an unproven chain rather than a defect. */
    const authorisation auth{latched_auth_};
    if (!allowed(auth)) {
        counters_.suppressed_not_proven++;
        return;
    }

    /* The descriptor for this index must agree with the facts. A disagreement means the two
     * arguments do not belong together, which no sensor can cause: it is a wiring defect, and
     * packing it would attach a grid to a source id that did not produce it. */
    const tof_acq::source_desc &d{cfg_.sources[index]};
    if (d.kind != facts.kind || d.role_id != facts.role_id) {
        counters_.suppressed_role_mismatch++;
        cycle_invalid_ = true;
        return;
    }

    /* THE SOURCE ID COMES FROM THE INSTALLED MAPPING AND FROM NOWHERE ELSE. Not from the
     * descriptor index, not from the address, not from the order samples arrive in. The contract
     * promises that source_id names a physical position; the mapping proof is the only thing that
     * establishes which, and a position it never keyed has no source id to publish under. */
    const uint8_t src{facts.role_id};
    if (src >= kGridSources) {
        counters_.suppressed_role_unassigned++;
        cycle_invalid_ = true;
        return;
    }
    if ((seen_sources_ & static_cast<uint8_t>(1U << src)) != 0) {
        counters_.suppressed_duplicate_source++;
        cycle_invalid_ = true;
        return;
    }
    /* The scheduler sets sample_produced from sample.fresh and calls this only when it is set,
     * so a disagreement is a defect in the caller rather than anything a sensor can do. */
    if (!sample.fresh || !facts.sample_produced) {
        counters_.suppressed_sample_contradiction++;
        cycle_invalid_ = true;
        return;
    }

    gp::sensor_read read{};

    /* COMPLETE is the adapter's all-or-nothing read, reported rather than asserted: read_once
     * clears the sample on entry and leaves it non-fresh on every refusal, including a grid
     * whose metadata was impossible, so a fresh sample is a whole one. */
    read.complete = sample.fresh;
    read.io_success = io_ok(facts);
    /* MODEL_VERIFIED is the mapping's assertion, not the descriptor's type. "This position holds
     * a VL53L7CX" is proved by the walk that keyed it, and the evidence for it here is a PROVEN
     * authorisation plus a descriptor keyed under it -- both re-read as an expression rather
     * than assumed from the checks above, so that reordering them cannot turn this into a lie. */
    read.model_verified = allowed(auth) && d.role_id == src && src < kGridSources;
    read.source_id = src;
    read.generation = next_generation_[src];
    /* Diagnostics only, both of them, and the contract says so explicitly: "nothing in the decoder
     * may branch on it". The number reported is the INDEX INTO THE DESCRIPTOR TABLE, which is built
     * in chain order -- the same number the scheduler logs a source under, so a health frame and a
     * log line can be read against each other. The chain spec's prose numbers boards from one;
     * this is the array index, not that, and the two differ by one on purpose rather than by
     * accident. */
    read.chain_position = static_cast<uint8_t>(index & 0x0F);
    read.boards_detected = static_cast<uint8_t>(auth.boards_detected & 0x0F);
    read.recovered_flags = flags_for(src, auth);
    read.last_error = pending_error_[src];
    for (size_t z{0}; z < gp::kZones; ++z) {
        read.zones_mm[z] = sample.distance_mm[z];
        read.target_status[z] = sample.target_status[z];
    }

    const gp::completed_verified_grid grid{
        gp::completed_verified_grid::from_read(read, cfg_.accept_low_confidence)};
    if (!grid.admitted()) {
        /* The transmit obligation was not met. Nothing above softened an input to get past it:
         * io_success and complete are reported as observed, so this is a real refusal. */
        counters_.suppressed_not_admitted++;
        cycle_invalid_ = true;
        return;
    }

    gp::can_frame_out data[gp::kDataFrames];
    gp::can_frame_out health;
    if (!gp::packer::pack(grid, data, health)) {
        counters_.suppressed_not_admitted++;
        cycle_invalid_ = true;
        return;
    }

    /* Room for the WHOLE grid or none of it. Queueing fifteen frames and discovering there is no
     * room for the sixteenth would put a partial grid on the bus, which is the one thing the
     * transmit obligation forbids. */
    if (queued_ + kFramesPerGrid > kQueueFrames) {
        counters_.queue_full++;
        cycle_invalid_ = true;
        return;
    }
    for (size_t chunk{0}; chunk < gp::kDataFrames; ++chunk)
        (void)enqueue(cycle_seq, ids::TOF_GRID_DATA_ID, data[chunk].bytes, src, false);
    (void)enqueue(cycle_seq, ids::TOF_GRID_HEALTH_ID, health.bytes, src, true);

    seen_sources_ |= static_cast<uint8_t>(1U << src);
    next_generation_[src] = static_cast<uint8_t>(next_generation_[src] + 1);
    reported_[src] = true;
    reported_flags_[src] = static_cast<uint8_t>(read.recovered_flags & 0x03);
    reported_error_[src] = read.last_error;
    counters_.grids_admitted++;
}

void on_cycle_complete(const tof_acq::cycle_facts &facts)
{
    pending_frame outgoing[kQueueFrames];
    int count{0};
    struct can_sink sink{};

    {
        const guard held;
        if (!ready_)
            return;

        accumulate_failures(facts);

        /* A cycle nobody announced cannot be completed either. */
        if (!latched_ || latched_cycle_ != facts.cycle_seq) {
            counters_.suppressed_cycle_not_begun++;
            counters_.discarded_stale_cycle += static_cast<uint32_t>(queued_);
            retire_packed_generations();
            reset_cycle();
            return;
        }

        /* Re-read the authorisation. The grids were packed under the chain lock and this runs
         * after it was dropped, so the mapping may have been revoked in between -- and an
         * already-packed grid would otherwise go out authorised under a mapping that no longer
         * holds. If it was revoked, or the epoch moved, the WHOLE cycle goes: a half-published
         * cycle is a broken correlation the consumer cannot detect. */
        const authorisation now{cfg_.authorise()};
        if (!allowed(now) || now.epoch != latched_auth_.epoch) {
            counters_.cycles_discarded_unauthorised++;
            retire_packed_generations();
            reset_cycle();
            return;
        }

        uint32_t stale{0};
        for (int i{0}; i < queued_; ++i) {
            if (queue_[i].cycle_seq != facts.cycle_seq) {
                ++stale;
                continue;
            }
            outgoing[count++] = queue_[i];
        }
        if (stale != 0) {
            counters_.discarded_stale_cycle += stale;
            cycle_invalid_ = true;
        }

        /* A STRUCTURAL failure is known before anything is sent, so nothing is sent. A grid
         * refused by the packer, a duplicate source id, a full queue: each of them means this
         * cycle's account of the chain is wrong, and a well-formed grid published beside it would
         * disguise a producer defect as ordinary quiet. */
        if (cycle_invalid_) {
            counters_.cycles_invalid++;
            retire_packed_generations();
            reset_cycle();
            return;
        }

        sink = cfg_.sink;
        reset_cycle();
    }

    /* Outside the lock. Each frame is offered exactly once and then dropped whatever happens:
     * carrying a failed grid into the next cycle would put stale distances on the wire under a
     * generation the consumer has already started assembling. */
    uint32_t data_sent{0}, health_sent{0}, failed_data{0}, failed_health{0}, abandoned{0};
    bool src_failed[kGridSources]{};
    int src_data_sent[kGridSources]{};
    bool src_health_sent[kGridSources]{};

    for (int i{0}; i < count; ++i) {
        const pending_frame &f{outgoing[i]};
        const uint8_t s{static_cast<uint8_t>(f.source_id < kGridSources ? f.source_id : 0)};

        /* One failed frame ends its grid. The remaining frames describe a generation the
         * consumer can now only discard, and the health frame would assert zones that never
         * arrived. */
        if (src_failed[s]) {
            ++abandoned;
            continue;
        }
        if (sink.send(f.can_id, f.data, f.dlc) != 0) {
            src_failed[s] = true;
            if (f.is_health)
                ++failed_health;
            else
                ++failed_data;
            continue;
        }
        if (f.is_health) {
            ++health_sent;
            src_health_sent[s] = true;
        } else {
            ++data_sent;
            ++src_data_sent[s];
        }
    }

    {
        const guard held;
        if (!ready_)
            return;

        counters_.data_frames_sent += data_sent;
        counters_.health_frames_sent += health_sent;
        counters_.send_failed_data += failed_data;
        counters_.send_failed_health += failed_health;
        counters_.frames_abandoned += abandoned;

        for (int s{0}; s < kGridSources; ++s) {
            if (!reported_[s])
                continue;
            reported_[s] = false;

            const bool whole{src_health_sent[s] &&
                             src_data_sent[s] == static_cast<int>(gp::kDataFrames)};
            if (!whole) {
                counters_.generations_retired++;
                continue;
            }
            counters_.grids_sent++;
            /* Cleared only now, and only the bits this frame carried. A failure recorded after
             * the frame was built keeps its bit and is reported by the next grid, which is what
             * "since the previous health frame" means. */
            pending_flags_[s] &= static_cast<uint8_t>(~reported_flags_[s]);
            if (pending_error_[s] == reported_error_[s])
                pending_error_[s] = 0;
        }
    }
}

void copy_counters(struct counters &out)
{
    const guard held;
    out = counters_;
}

} // namespace lexxhard::tof_grid_pub

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD && ENABLE_TOF_L7_ULD
