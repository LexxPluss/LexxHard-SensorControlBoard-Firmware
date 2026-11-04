/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Timing constants of the RS-485 link to the load cell boards and the reception deadline of one frame.
 * The time base is a free-running uint32 tick of kTickUs microseconds.
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "lcs_frame.hpp"

namespace lexxhard::lcs {

inline constexpr uint32_t kBaud{500000};
inline constexpr uint32_t kTickUs{100};

/* Bytes of SOF and LEN: the only part of a frame whose length is known before LEN has arrived. */
inline constexpr size_t kHeaderBytes{2};

static_assert(kTickUs > 0);
static_assert(1000000 % kBaud == 0, "a bit time must be a whole number of microseconds");

/* One byte is 10 bits on the wire (start, 8 data, stop). */
constexpr uint32_t frame_tx_us(const size_t len)
{
    return static_cast<uint32_t>(len * 10U * (1000000U / kBaud));
}

/* The idle line detection of the receiver takes one character time. */
inline constexpr uint32_t kIdleDetectUs{frame_tx_us(1)};

/* Margin on the frame time and on the idle detection; it is not applied to the drain latency. */
inline constexpr uint32_t kDeadlineMarginPercent{100};

/* PROVISIONAL: the longest delay between the arrival of a byte and its drain by the receive thread. */
inline constexpr uint32_t kDrainLatencyMaxUs{500};

/*
 * Ticks, rounded up, in which a frame of len bytes must have been received, measured from its SOF.
 * A candidate expires when its elapsed ticks are strictly greater than this value.
 */
constexpr uint32_t deadline_ticks(const size_t len)
{
    const uint64_t numerator{(static_cast<uint64_t>(frame_tx_us(len)) + kIdleDetectUs) * (100U + kDeadlineMarginPercent)
                             + 100U * static_cast<uint64_t>(kDrainLatencyMaxUs)};
    const uint64_t denominator{100U * static_cast<uint64_t>(kTickUs)};
    return static_cast<uint32_t>((numerator + denominator - 1U) / denominator);
}

/* Deadline of a candidate whose LEN byte has not arrived yet. */
constexpr uint32_t header_deadline_ticks()
{
    return deadline_ticks(kHeaderBytes);
}

/* LEN is not trusted before the CRC matches, so a LEN above kMaxLen gets the deadline of kMaxLen. */
constexpr uint32_t len_deadline_ticks(const size_t len)
{
    return deadline_ticks(std::min(len, kMaxLen));
}

static_assert(kMaxLen >= kDownlinkSize && kMaxLen >= kUplinkSize);
static_assert(deadline_ticks(kHeaderBytes) <= deadline_ticks(kDownlinkSize));
static_assert(deadline_ticks(kDownlinkSize) <= deadline_ticks(kUplinkSize));
static_assert(deadline_ticks(kMaxLen) * kTickUs >= frame_tx_us(kMaxLen) + kIdleDetectUs,
              "the deadline covers the frame time and the idle detection");
static_assert(deadline_ticks(kMaxLen) < 0x80000000U, "elapsed ticks are compared wrap-safe");

}
