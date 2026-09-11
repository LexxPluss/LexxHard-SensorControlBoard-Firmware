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
    /* What this frame will claim in the cycle health masks, carried with the frame rather
     * than recomputed later. The bit is only ever set from a send that succeeded, because
     * sample_produced_mask asserts a frame EXISTS for that source in that cycle. */
    uint8_t source_id;
    bool sensor_fault;
};
pending_frame queue_[tof_acq::kMaxSources]{};
int queued_{0};

/* Latched at the START of a cycle -- by on_cycle_begin, never by the first sample. A cycle may
 * legally produce zero measurements, and latching on the first one would mean such a cycle was
 * never latched and therefore never reported. */
bool latched_{false};
uint32_t latched_cycle_{0};
authorisation latched_auth_{};

/* Something structural went wrong in this cycle: a packer refusal, a full queue, a stale queue
 * entry, a model or role mismatch. Not a sensor outcome -- a NO_SAMPLE is an ordinary empty bit
 * and leaves the cycle valid.
 *
 * It exists because withholding only on a transport failure was not enough. A packer refusal
 * drops one measurement and would otherwise leave a perfectly legal-looking health frame with
 * one bit missing, which is indistinguishable from a sensor that had nothing to report. That
 * disguises a producer defect as ordinary quiet, which is the worst of the three outcomes: the
 * bug becomes invisible precisely because the frame is well formed. */
bool cycle_invalid_{false};

/* The health sequence counter.
 *
 * Its own counter, deliberately NOT derived from health_sent. Two reasons, and both are
 * contract requirements rather than preferences. The contract makes health_seq the liveness
 * signal -- a NEW snapshot must carry a new number -- so a snapshot whose CAN send failed must
 * still advance it, or the next successful frame would repeat a number and read as a
 * retransmission of a stale snapshot. And the timer heartbeat and the cycle health frame are
 * two producers of snapshots; deriving from a success counter would let them collide on one
 * number. Allocation happens under the lock, so the two can never take the same one.
 *
 * A gap in the sequence therefore means "a snapshot was formed and did not reach the bus",
 * which is exactly what happened and exactly what the consumer should see. */
uint8_t next_health_seq_{0};

uint8_t take_health_seq()
{
    return next_health_seq_++;
}

bool allowed(const authorisation &a)
{
    return a.state == tof_acq::mapping_state::proven;
}

bool enqueue(uint32_t cycle_seq, uint16_t can_id, const uint8_t *data, uint8_t dlc,
             uint8_t source_id, bool sensor_fault)
{
    if (queued_ >= static_cast<int>(sizeof queue_ / sizeof queue_[0]))
        return false;
    pending_frame &f{queue_[queued_++]};
    f.cycle_seq = cycle_seq;
    f.can_id = can_id;
    f.dlc = dlc;
    for (int i = 0; i < 8; ++i)
        f.data[i] = data[i];
    f.source_id = source_id;
    f.sensor_fault = sensor_fault;
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
    /* Reset with the rest of the state. It is the liveness counter for THIS configuration, and
     * a consumer that reconnects after a re-init must not be handed a number that looks like a
     * continuation of a stream it never saw. */
    next_health_seq_ = 0;
    ready_ = true;
    return 0;
}

void on_cycle_begin(uint32_t cycle_seq)
{
    const guard held;
    if (!ready_)
        return;

    /* Anything still queued belongs to a cycle that never got flushed. Dropping it is the same
     * rule as everywhere else: an unsent range is a stale range. */
    if (queued_ > 0) {
        counters_.discarded_stale_cycle += static_cast<uint32_t>(queued_);
        queued_ = 0;
    }

    latched_ = true;
    latched_cycle_ = cycle_seq;
    latched_auth_ = cfg_.authorise();
    cycle_invalid_ = false;
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
    /* The cycle must have been announced. This is not defensiveness for its own sake: the
     * authorisation is latched by on_cycle_begin, so a sample for an unannounced cycle has no
     * authorisation to be published under, and inventing one here is how the zero-measurement
     * hole got created in the first place. */
    if (!latched_ || latched_cycle_ != cycle_seq) {
        counters_.suppressed_cycle_not_begun++;
        return;
    }

    if (facts.kind != tof_acq::model::l4_cliff) {
        counters_.suppressed_wrong_model++;
        cycle_invalid_ = true;
        return;
    }

    /* The descriptor for this index must agree with the facts the sink was handed. A
     * disagreement means the two arguments do not belong together, which no sensor can
     * cause -- it is a wiring defect, and packing it would attach a range to a role that
     * did not produce it. */
    const tof_acq::source_desc &d{cfg_.sources[index]};
    if (d.kind != facts.kind || d.role_id != facts.role_id) {
        counters_.suppressed_role_mismatch++;
        cycle_invalid_ = true;
        return;
    }

    /* One authorisation for the whole cycle, taken at its start. Reading it per sensor would let
     * four frames of one cycle carry different epochs, and the consumer correlates on exactly
     * that. */
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
        cycle_invalid_ = true;
        return;
    }

    uint8_t frame[8]{};
    pk::reason why{pk::reason::none};
    if (!pk::encode_measurement(r, facts.role_id, auth.epoch,
                                static_cast<uint8_t>(cycle_seq & 0xFF), frame, &why)) {
        counters_.suppressed_packer_refused++;
        counters_.last_refusal = why;
        cycle_invalid_ = true;
        return;
    }

    /* Queued, not sent. This runs under the chain lock; touching the bus here would let
     * CAN backpressure stall every sensor's next read. */
    /* SENSOR_FAULT is the one class that sets both mask bits: a sample exists -- that is what
     * makes the frame owed at all -- and the sensor reports it as unusable. NO_SAMPLE never
     * gets here, because reduce() returns no_frame for it, so its bit stays clear by
     * construction rather than by a rule someone has to remember. */
    if (!enqueue(cycle_seq, ctr::kMeasId, frame, ctr::kDlc, facts.role_id,
                 r.cls == ctr::status_class::sensor_fault)) {
        counters_.queue_full++;
        cycle_invalid_ = true;
    }
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
    authorisation cycle_auth{};
    bool invalid{false};

    {
        const guard held;
        if (!ready_)
            return;
        /* A cycle nobody announced cannot be completed either. Deliberately NOT the same as an
         * empty cycle: an empty ANNOUNCED cycle still owes a health frame, which is the whole
         * point of the change that introduced on_cycle_begin. */
        if (!latched_ || latched_cycle_ != facts.cycle_seq) {
            /* Drop the cycle as a unit, and attribute the defect where it was detected. Anything
             * queued belongs to a cycle whose completion is now past, so leaving it would let an
             * unsent range wait for a later flush -- which is a stale range on the wire, the one
             * thing every path here refuses to do. */
            counters_.suppressed_cycle_not_begun++;
            counters_.discarded_stale_cycle += static_cast<uint32_t>(queued_);
            queued_ = 0;
            latched_ = false;
            cycle_invalid_ = false;
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
            cycle_invalid_ = false;
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
        /* A queued frame from another cycle is a producer defect, so the cycle it landed in is
         * not trustworthy either. */
        if (stale != 0)
            cycle_invalid_ = true;
        invalid = cycle_invalid_;
        queued_ = 0;
        latched_ = false;
        cycle_invalid_ = false;

        /* A STRUCTURAL failure is known before anything is sent, so nothing is sent.
         *
         * This is the difference between dropping a cycle and dropping it ON THE WIRE. The first
         * version queued the measurements, sent them, and then withheld the health frame -- safe
         * only because a conforming consumer refuses an unauthorised measurement, which made
         * "the whole cycle is dropped" true at the consumer and false on the bus. Two reasons to
         * do it here instead: a claim that is only true one layer away is the kind that gets
         * quoted without the qualifier, and a non-conforming or half-written consumer would
         * happily take the frames.
         *
         * A send FAILURE is the one case that cannot be handled this way -- it is not knowable
         * until the send is attempted -- so that path alone can leave measurements on the bus
         * with no authorising health frame. */
        if (invalid) {
            counters_.cycles_invalid++;
            count = 0;
            return;
        }
        sink = cfg_.sink;
        /* Carried out of the lock as a VALUE. The cycle health frame is built from what was
         * latched for this cycle and never from a fresh read -- that is what lets it arrive
         * after a newer heartbeat reporting LOST and still legitimately authorise its own
         * cycle, because the consumer correlates on (epoch, cycle_seq) rather than on arrival
         * order. Re-reading here would make the frame describe a mapping that is not the one
         * its measurements were authorised under. */
        cycle_auth = latched_auth_;
    }

    /* Each frame is offered exactly once and then dropped whatever happens: carrying a
     * failed range into the next cycle would put a stale distance on the wire. */
    uint32_t sent{0}, failed{0};
    uint8_t produced_mask{0}, fault_mask{0};
    for (int i = 0; i < count; ++i) {
        const pending_frame &f{outgoing[i]};
        if (sink.send(f.can_id, f.data, f.dlc) != 0) {
            ++failed;
            continue;
        }
        ++sent;
        /* Only a send that succeeded may set a bit. sample_produced_mask asserts that a
         * measurement frame for that source EXISTS in this cycle, and the consumer checks both
         * directions of that. */
        produced_mask |= static_cast<uint8_t>(1U << (f.source_id & 0x3));
        if (f.sensor_fault)
            fault_mask |= static_cast<uint8_t>(1U << (f.source_id & 0x3));
    }

    /* THE CYCLE HEALTH FRAME, AND THE TWO WAYS A CYCLE LOSES IT
     *
     * A measurement that did not reach the bus must not be claimed by a mask. The alternative
     * would be a frame asserting four samples with three on the wire -- the contradiction the
     * contract forbids outright, and one a consumer would wait on forever.
     *
     * A structural failure -- a packer refusal, a full queue, a stale queue entry, a role or
     * model mismatch -- is caught earlier, above, and costs the cycle every frame rather than
     * only its health frame. The sharper reason: a health frame with one bit missing is
     * perfectly well formed and indistinguishable from a sensor that had nothing to report, so
     * publishing the rest of the cycle would disguise a producer defect as ordinary quiet. The
     * bug becomes invisible BECAUSE the frame looks right.
     *
     * The transport case is the residue. A failed send is not knowable until it is attempted, so
     * this path alone can leave measurements on the bus with no authorising health frame. That
     * is still the safe direction -- a conforming consumer accepts no measurement whose cycle has
     * no cycle_valid health -- but it is a revocation at the CONSUMER, not on the wire, and it is
     * worth saying so rather than describing both cases with one sentence.
     *
     * A cycle with NO measurements at all is not one of these cases. The contract allows a cycle
     * to carry between zero and four, and such a cycle still owes its health frame with both
     * per-cycle masks empty: without it, "completed, all four sources had nothing" and "the cycle
     * never happened" are the same silence. */
    uint8_t health[8]{};
    {
        const guard held;
        counters_.measurements_sent += sent;
        counters_.send_failed_measurement += failed;

        if (failed != 0) {
            /* Under the lock with the rest of the counters, not after it: the timer heartbeat and
             * copy_counters() touch these from other contexts.
             *
             * Only the transport case reaches here. A structural failure already returned above,
             * before anything was offered to the bus. */
            counters_.cycle_health_withheld++;
            return;
        }

        if (!ready_)
            return;

        pk::health_fields h{};
        h.mapping_epoch = cycle_auth.epoch;
        h.health_seq = take_health_seq();
        h.mapping_state = wire_mapping_state(cycle_auth.state);
        h.flags = static_cast<uint8_t>((cycle_auth.chain_flags & 0x7) | ctr::kCycleValidBit);
        h.enumerated_mask = cycle_auth.enumerated_mask;
        h.model_verified_mask = cycle_auth.model_verified_mask;
        h.sample_produced_mask = produced_mask;
        h.sensor_fault_mask = fault_mask;
        h.failing_chain_position = cycle_auth.failing_position;
        h.cycle_seq = static_cast<uint8_t>(facts.cycle_seq & 0xFF);

        if (!pk::encode_health(h, health)) {
            counters_.health_encode_refused++;
            return;
        }
    }

    const int rc{sink.send(ctr::kHealthId, health, ctr::kDlc)};

    const guard held;
    if (rc != 0)
        counters_.send_failed_health++;
    else
        counters_.cycle_health_sent++;
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
    /* Its own counter, shared with the cycle health frame and allocated under this lock, so the
     * two producers can never take the same number. Advanced by forming the snapshot, not by
     * sending it: a failed send must still consume a number or the next successful frame would
     * repeat one and read as a retransmission of something stale. */
    h.health_seq = take_health_seq();
    h.mapping_state = wire_mapping_state(auth.state);
    /* From the mapping authority, and from the SAME snapshot as the state and the epoch. These
     * describe the last enumeration attempt rather than a cycle, which is what keeps a
     * heartbeat useful while UNKNOWN. */
    h.enumerated_mask = auth.enumerated_mask;
    h.model_verified_mask = auth.model_verified_mask;
    h.failing_chain_position = auth.failing_position;
    h.flags = static_cast<uint8_t>(auth.chain_flags & 0x7); // no cycle_valid: describes no cycle

    /* The per-cycle fields stay zero here, and the contract requires exactly that while
     * cycle_valid is clear -- encode_health() refuses the combination, so this is enforced
     * rather than remembered. The completed-cycle frame is produced by on_cycle_complete().
     *
     * The acquisition snapshot is still unused: its per-source error bit is transport OR
     * protocol OR usage, so it cannot substantiate the contract's specifically-transport flag,
     * and its produced bits describe the cycle this frame explicitly does not describe. */
    (void)snapshot;
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
