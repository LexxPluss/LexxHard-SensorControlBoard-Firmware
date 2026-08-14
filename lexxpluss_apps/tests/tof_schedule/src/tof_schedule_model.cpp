/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_schedule_model.hpp"

namespace tof_schedule {

namespace {

// One transfer is the unit of non-preemptibility: the acquisition thread holds the bus
// for its whole duration and can only switch devices between transfers.
uint64_t transfer_ns(const bus_config &bus, uint32_t payload_bytes)
{
    const uint64_t bytes{static_cast<uint64_t>(payload_bytes) + bus.per_transfer_overhead_bytes};
    return (bytes * bus.bits_per_byte * 1000000000ull) / bus.speed_hz;
}

// Seeded so a configuration always produces the same schedule. A wall clock or an
// unseeded generator would make the CI comparison meaningless.
struct rng {
    uint32_t state;
    uint32_t next()
    {
        state = state * 1664525u + 1013904223u;
        return state >> 16;
    }
};

// What a device is waiting to have done for it.
struct pending {
    bool active{false};
    uint64_t ready_ns{0};       // when the operation became available
    uint32_t bytes_left{0};     // of the read
    bool rearm_left{false};     // point sensors: the re-arm still owed
};

}  // namespace

sim_result run(const sim_config &cfg)
{
    sim_result out{};
    out.device_count = cfg.device_count;
    out.horizon_ns = cfg.horizon_ns;

    pending pend[kMaxDevices]{};
    uint64_t next_frame_ns[kMaxDevices]{};
    uint64_t last_complete_ns[kMaxDevices]{};
    bool have_last[kMaxDevices]{};
    uint64_t interval_sum[kMaxDevices]{};
    rng r{cfg.seed};

    for (int i = 0; i < cfg.device_count; ++i) {
        const device_config &d{cfg.devices[i]};

        out.devices[i].sort = d.sort;
        next_frame_ns[i] = d.phase_ns;
        if (d.sort == kind::point) {
            // A point sensor is measuring from the start; its first result is ready one
            // timing budget in. Without this arming it would never ask for the bus at
            // all, because the only other place that arms one is the end of its own read.
            pend[i] = pending{true, d.phase_ns + d.period_ns, d.read_bytes, true};
        }
    }

    uint64_t now{0};
    while (now < cfg.horizon_ns) {
        // Arrivals first: a grid frame appears whether or not anyone read the last one.
        for (int i = 0; i < cfg.device_count; ++i) {
            const device_config &d{cfg.devices[i]};
            if (d.sort != kind::grid)
                continue;
            while (next_frame_ns[i] <= now) {
                if (pend[i].active) {
                    // The buffer holds the newest frame; the read in flight will return
                    // whatever is there. Whether that is coherent is exactly the question
                    // this model cannot answer -- see the header.
                    ++out.devices[i].frames_skipped;
                } else {
                    pend[i] = pending{true, next_frame_ns[i], d.read_bytes, false};
                }
                next_frame_ns[i] += d.period_ns;
            }
        }

        // Pick the device that has been waiting longest. Deterministic on ties.
        int chosen{-1};
        uint64_t best_ready{0};
        for (int i = 0; i < cfg.device_count; ++i) {
            if (!pend[i].active || pend[i].ready_ns > now)
                continue;
            if (chosen < 0 || pend[i].ready_ns < best_ready) {
                chosen = i;
                best_ready = pend[i].ready_ns;
            }
        }

        if (chosen < 0) {
            // Nothing to do: jump to the next arrival rather than stepping time.
            uint64_t next{cfg.horizon_ns};
            for (int i = 0; i < cfg.device_count; ++i) {
                if (pend[i].active && pend[i].ready_ns > now && pend[i].ready_ns < next)
                    next = pend[i].ready_ns;
                if (cfg.devices[i].sort == kind::grid && next_frame_ns[i] < next)
                    next = next_frame_ns[i];
            }
            now = (next > now) ? next : now + 1;
            continue;
        }

        const device_config &d{cfg.devices[chosen]};
        pending &p{pend[chosen]};

        if (cfg.scheduler_jitter_max_ns != 0)
            now += r.next() % (cfg.scheduler_jitter_max_ns + 1);

        // One transfer only, then the scheduler decides again. For an atomic read that is
        // the whole frame; for a chunked read it is one chunk, which is where another
        // device gets its turn.
        uint32_t this_transfer{0};
        if (p.bytes_left > 0) {
            const uint32_t chunk{d.chunk_bytes == 0 ? p.bytes_left : d.chunk_bytes};
            this_transfer = (chunk < p.bytes_left) ? chunk : p.bytes_left;
            p.bytes_left -= this_transfer;
        } else if (p.rearm_left) {
            this_transfer = d.rearm_bytes;
            p.rearm_left = false;
        }

        const uint64_t dur{transfer_ns(cfg.bus, this_transfer)};
        now += dur;
        out.bus_busy_ns += dur;
        if (dur > out.longest_uninterruptible_ns)
            out.longest_uninterruptible_ns = dur;

        const bool read_done{p.bytes_left == 0};
        if (read_done && d.sort == kind::point && p.rearm_left) {
            // A point sensor's read and its re-arm are issued back to back: letting
            // another device in between would delay this sensor's own next measurement
            // for no gain. Preemption within a device happens only between grid chunks.
            const uint64_t rearm{transfer_ns(cfg.bus, d.rearm_bytes)};
            now += rearm;
            out.bus_busy_ns += rearm;
            if (rearm > out.longest_uninterruptible_ns)
                out.longest_uninterruptible_ns = rearm;
            p.rearm_left = false;
        }

        if (!read_done)
            continue;

        // The operation completed.
        if (cfg.can_enqueue_ns != 0)
            now += cfg.can_enqueue_ns;

        device_result &res{out.devices[chosen]};
        ++res.completed_reads;

        const uint64_t latency{now - p.ready_ns};
        if (latency > res.max_latency_ns)
            res.max_latency_ns = latency;

        if (have_last[chosen]) {
            const uint64_t interval{now - last_complete_ns[chosen]};
            interval_sum[chosen] += interval;
            if (interval > res.max_interval_ns)
                res.max_interval_ns = interval;
            if (d.sort == kind::point && interval > cfg.point_deadline_ns)
                ++res.deadline_misses;
        }
        last_complete_ns[chosen] = now;
        have_last[chosen] = true;

        p.active = false;
        if (d.sort == kind::point) {
            // The next measurement starts only now, which is the whole reason a long read
            // on another sensor costs this one its rate.
            pend[chosen] = pending{true, now + d.period_ns, d.read_bytes, true};
        }
    }

    for (int i = 0; i < cfg.device_count; ++i) {
        device_result &res{out.devices[i]};
        if (res.completed_reads > 1)
            res.mean_interval_ns = interval_sum[i] / (res.completed_reads - 1);
    }
    out.bus_utilisation_permille =
        cfg.horizon_ns == 0 ? 0
                            : static_cast<uint32_t>((out.bus_busy_ns * 1000) / cfg.horizon_ns);
    return out;
}

sim_config six_sensor_chain(uint32_t speed_hz, uint32_t grid_read_bytes,
                            uint32_t grid_chunk_bytes, bool grids_in_phase,
                            uint64_t point_budget_ns)
{
    sim_config cfg{};
    cfg.bus.speed_hz = speed_hz;
    cfg.device_count = 6;

    // Two grids at 5 Hz. In phase is the case that has to be tested: both frames become
    // ready together, and the point sensors wait behind both reads.
    for (int i = 0; i < 2; ++i) {
        cfg.devices[i].sort = kind::grid;
        cfg.devices[i].read_bytes = grid_read_bytes;
        cfg.devices[i].chunk_bytes = grid_chunk_bytes;
        cfg.devices[i].period_ns = 200ull * 1000 * 1000;
        cfg.devices[i].phase_ns = grids_in_phase ? 0 : (i * 100ull * 1000 * 1000);
    }
    // Four point sensors. 133 bytes is the assumed read size and the timing budget is a
    // caller-supplied input; both are provisional, which is why nothing asserts a rate.
    for (int i = 2; i < 6; ++i) {
        cfg.devices[i].sort = kind::point;
        cfg.devices[i].read_bytes = 133;
        cfg.devices[i].rearm_bytes = 2;
        cfg.devices[i].period_ns = point_budget_ns;
    }
    return cfg;
}

}  // namespace tof_schedule
