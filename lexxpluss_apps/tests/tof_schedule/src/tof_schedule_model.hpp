/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

// A scheduling model for the six-sensor ToF chain: two VL53L7CX grids and four VL53L4CX
// point sensors on one I2C bus, serviced by one acquisition thread.
//
// WHAT IT IS FOR
//
// The binding constraint on this chain is not bus bandwidth, it is scheduling: a VL53L4CX
// does not free-run, so its next measurement starts only after the host has read the
// result and re-armed it. A long blocking read on another sensor is therefore added
// directly to a point sensor's period. This model exists to answer one design question
// before the interface is frozen -- **how much would chunked grid reads gain** -- because
// that decides whether the L7 read interface has to express partial frames.
//
// WHAT IT CANNOT ANSWER, AND MUST NOT BE READ AS ANSWERING
//
//   - It cannot show that chunked reads are LEGAL. Splitting a grid read assumes the
//     device's result buffer stays coherent between the chunks; if it does not, chunking
//     buys latency by returning torn frames. That has to be established from the ULD and
//     the part, not from a simulation, before any chunk-aware interface is designed.
//   - Its inputs are provisional. The block sizes (1452 untrimmed, 328 trimmed, 133 per
//     point read) and the bus speeds are design-time figures that no logic analyser has
//     confirmed. Every number this model produces inherits that status, so the tests
//     assert structure -- no starvation, determinism, monotonicity -- and deliberately do
//     not assert that any product rate is met.
//
// WHAT IS DELIBERATELY NOT MODELLED, IN v1
//
// The CAN transport, the RTOS, and the existing safety tasks. Each would need assumptions
// broad enough to swamp the result. Instead there are two injection points -- scheduler
// jitter and a per-frame CAN enqueue cost -- to be replaced by measurements later.

#include <cstddef>
#include <cstdint>

namespace tof_schedule {

inline constexpr int kMaxDevices{8};

enum class kind : uint8_t {
    grid,  // VL53L7CX: free-running, a new frame every period whether read or not
    point, // VL53L4CX: does not free-run; the next measurement starts after the re-arm
};

struct device_config {
    kind sort{kind::point};
    uint32_t read_bytes{133};
    // 0 means one atomic transfer. Otherwise the read is split, and the scheduler may
    // serve another device between chunks -- which is the entire point of chunking.
    uint32_t chunk_bytes{0};
    uint32_t rearm_bytes{2};  // point sensors only: clear interrupt and restart
    uint64_t period_ns{0};    // grid: frame period. point: measurement timing budget
    uint64_t phase_ns{0};     // grid only: offset of the first frame
};

struct bus_config {
    uint32_t speed_hz{400000};
    // 9 bits per byte: eight data bits plus the acknowledge. This is the same arithmetic
    // the design tables use, so the model reproduces them when the overhead below is 0.
    uint32_t bits_per_byte{9};
    // Address plus the 16-bit register index. Set to 0 to reproduce the hand-calculated
    // tables exactly; 3 is the honest figure for a real transaction.
    uint32_t per_transfer_overhead_bytes{3};
};

struct sim_config {
    bus_config bus{};
    device_config devices[kMaxDevices]{};
    int device_count{0};
    uint64_t horizon_ns{10ull * 1000 * 1000 * 1000};
    // Injection points, both zero by default. Deterministic: the jitter is drawn from a
    // seeded generator so that two runs of one configuration are identical.
    uint64_t scheduler_jitter_max_ns{0};
    uint64_t can_enqueue_ns{0};
    uint32_t seed{1};
    // Reported, never asserted: the interval a point sensor would have to hold to meet a
    // rate someone hopes for.
    uint64_t point_deadline_ns{50ull * 1000 * 1000};
};

struct device_result {
    kind sort{kind::point};
    uint32_t completed_reads{0};
    uint64_t max_interval_ns{0};   // longest gap between two completed reads
    uint64_t mean_interval_ns{0};
    uint64_t max_latency_ns{0};    // grid: ready -> last chunk done
    uint32_t frames_skipped{0};    // grid: a new frame arrived before the old was read
    uint32_t deadline_misses{0};   // point: intervals longer than point_deadline_ns
};

struct sim_result {
    device_result devices[kMaxDevices]{};
    int device_count{0};
    uint64_t bus_busy_ns{0};
    uint64_t horizon_ns{0};
    uint64_t longest_uninterruptible_ns{0};  // the longest single transfer issued
    uint32_t bus_utilisation_permille{0};
};

// Deterministic for a given config: same input, same output, every time.
sim_result run(const sim_config &cfg);

// Helpers for building the standard six-sensor chain used by the tests.
// point_budget_ns is the VL53L4CX measurement timing budget. It is an INPUT, not a
// property of the schedule, and nothing has fixed it yet: the model's point-sensor rates
// move with it, which is why the sweep prints a sensitivity table rather than one number.
sim_config six_sensor_chain(uint32_t speed_hz, uint32_t grid_read_bytes,
                            uint32_t grid_chunk_bytes, bool grids_in_phase,
                            uint64_t point_budget_ns = 20ull * 1000 * 1000);

}  // namespace tof_schedule
