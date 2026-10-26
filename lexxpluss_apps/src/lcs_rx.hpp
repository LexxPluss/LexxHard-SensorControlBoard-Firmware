/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Receive core of the SCB as host of the RS-485 link to the load cell boards. Pure logic: bytes, errors and time
 * are passed in, no Zephyr or HAL include, no dynamic memory.
 *
 * CRC first: LEN is not trusted until a CRC matches, it only selects the length of the window that is checked.
 * Every SOF byte starts a candidate with its own deadline (see lcs_timing.hpp); a candidate is evaluated at the byte
 * that completes its window, the oldest first. A matching frame consumes every younger candidate, because their SOF
 * lies inside its span. A mismatch counts crc_error and drops only that candidate, so a frame hidden behind a
 * corrupted LEN is still found. A CRC-valid frame of a LEN other than 8 or 12 (4..11) is consumed silently (extension
 * point); a LEN below 4 is dropped at the LEN byte. A CRC-valid uplink of a sender outside the known boards is
 * dropped and counted.
 *
 * The ring of the last kMaxLen bytes is enough: a window never exceeds kMaxLen. A candidate with LEN above kMaxLen
 * is not CRC-checked and needs no bytes.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "lcs_crc16.hpp"
#include "lcs_frame.hpp"
#include "lcs_timing.hpp"

namespace lexxhard::lcs {

/*
 * Monotonic counters; neither reset() nor a timeout clears them. sof_mismatch counts a byte that is not a SOF and
 * arrives while no candidate is in progress.
 */
struct RxStats {
    uint32_t delivered{0};
    uint32_t crc_error{0};
    uint32_t sof_mismatch{0};
    uint32_t timeout{0};
    uint32_t uart_error{0};
    uint32_t foreign_downlink{0};
    uint32_t unknown_sender{0};
};

class UplinkReceiver {
public:
    /* One received byte with the tick of its drain. Returns the uplink this byte completes, if any. */
    std::optional<Uplink> on_byte(const uint8_t byte, const uint32_t tick)
    {
        expire(tick);
        const bool idle{count_ == 0};
        ring_[head_] = byte;
        head_ = (head_ + 1U) % kMaxLen;
        advance_candidates(byte);
        if (byte == kSof) {
            start_candidate(tick);
        } else if (idle) {
            ++stats_.sof_mismatch;
        }
        return evaluate_windows();
    }

    /* Counts every candidate whose elapsed ticks are strictly greater than its deadline. */
    void expire(const uint32_t tick)
    {
        size_t kept{0};
        for (size_t i{0}; i < count_; ++i) {
            if (tick - candidates_[i].sof_tick > deadline_of(candidates_[i])) {
                ++stats_.timeout;
            } else {
                candidates_[kept++] = candidates_[i];
            }
        }
        count_ = kept;
    }

    /* The bytes around an error are unknowable, so the candidates in progress go; counters are kept. */
    void on_uart_error(const uint32_t /* tick */)
    {
        count_ = 0;
        ++stats_.uart_error;
    }

    void reset()
    {
        count_ = 0;
    }

    const RxStats &stats() const
    {
        return stats_;
    }

private:
    /* An oversize LEN is kept as the candidate length so that the deadline of kMaxLen applies. */
    static constexpr size_t kOversize{kMaxLen + 1};
    /* SOF, LEN and two CRC bytes: a shorter LEN cannot be a frame. */
    static constexpr size_t kMinLen{4};
    static constexpr size_t kLenUnknown{0};
    static constexpr uint8_t kSeenMax{0xFF};

    struct Candidate {
        uint32_t sof_tick;
        size_t len;
        uint8_t seen;
    };

    static constexpr uint32_t deadline_of(const Candidate &candidate)
    {
        return candidate.len == kLenUnknown ? header_deadline_ticks() : len_deadline_ticks(candidate.len);
    }

    void erase(const size_t index)
    {
        for (size_t i{index + 1}; i < count_; ++i) {
            candidates_[i - 1] = candidates_[i];
        }
        --count_;
    }

    void advance_candidates(const uint8_t byte)
    {
        size_t kept{0};
        for (size_t i{0}; i < count_; ++i) {
            Candidate candidate{candidates_[i]};
            if (candidate.seen < kSeenMax) {
                ++candidate.seen;
            }
            if (candidate.seen == kHeaderBytes) {
                /* Below the shortest frame a window cannot hold a valid frame: drop at once, no counter. */
                if (byte < kMinLen) {
                    continue;
                }
                candidate.len = byte > kMaxLen ? kOversize : byte;
            }
            candidates_[kept++] = candidate;
        }
        count_ = kept;
    }

    void start_candidate(const uint32_t tick)
    {
        if (count_ == candidates_.size()) {
            /* Only candidates with an oversize LEN outlive their ring bytes, so the oldest one is given up. */
            ++stats_.timeout;
            erase(0);
        }
        candidates_[count_++] = Candidate{tick, kLenUnknown, 1};
    }

    std::optional<Uplink> evaluate_windows()
    {
        std::array<uint8_t, kMaxLen> frame{};
        size_t i{0};
        while (i < count_) {
            const size_t len{candidates_[i].len};
            if (len < kMinLen || len > kMaxLen || candidates_[i].seen != len) {
                ++i;
                continue;
            }
            for (size_t k{0}; k < len; ++k) {
                frame[k] = ring_[(head_ + kMaxLen - len + k) % kMaxLen];
            }
            const uint16_t received{static_cast<uint16_t>((frame[len - 2] << 8) | frame[len - 1])};
            if (crc16(frame.data(), len - 2) != received) {
                ++stats_.crc_error;
                erase(i);
                continue;
            }
            count_ = i;
            return classify(frame, len);
        }
        return std::nullopt;
    }

    /* After the CRC: LEN and the direction bit must agree, anything else is dropped silently. */
    std::optional<Uplink> classify(const std::array<uint8_t, kMaxLen> &frame, const size_t len)
    {
        const bool dir_set{(frame[3] & kFrameDirBit) != 0U};
        if (len == kDownlinkSize && !dir_set) {
            ++stats_.foreign_downlink;
        } else if (len == kUplinkSize && dir_set) {
            if (frame[2] >= kBoardCount) {
                ++stats_.unknown_sender;
                return std::nullopt;
            }
            ++stats_.delivered;
            return Uplink{frame[2], frame[3], {frame[4], frame[5], frame[6]}, {frame[7], frame[8], frame[9]}};
        }
        return std::nullopt;
    }

    RxStats stats_{};
    std::array<Candidate, kMaxLen> candidates_{};
    size_t count_{0};
    std::array<uint8_t, kMaxLen> ring_{};
    size_t head_{0};
};

}
