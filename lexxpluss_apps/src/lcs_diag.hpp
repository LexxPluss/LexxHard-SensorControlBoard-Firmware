/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Diagnostic counters of the LCS host and their shell text form. Pure: no Zephyr include.
 * Diag is a flat struct of uint32_t, so its words can be read in declaration order. A line is the group name
 * and the words of the group in decimal, with their names in the named build. Every counter has one writer;
 * the ISR-side counter is incremented atomically by its owner.
 */

#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

#include "lcs_rx.hpp"

namespace lexxhard::lcs {

struct RxGroup {
    RxStats stats;
    uint32_t restarts;
};

struct TxGroup {
    uint32_t sync_sent;
    uint32_t skipped;
    uint32_t rejected_busy;
    uint32_t timeout;
    uint32_t uart_err;
};

struct SyncGroup {
    uint32_t late;
    uint32_t max_interval;
};

struct IsrGroup {
    uint32_t rx_queue_overflow;
};

struct ReqGroup {
    uint32_t received;
    uint32_t queued;
    uint32_t dropped_invalid;
    uint32_t dropped_full;
    uint32_t forwarded;
};

struct CmdGroup {
    uint32_t zero_reset;
    uint32_t temp_drift_calib;
    uint32_t abort_cmd;
};

struct Diag {
    RxGroup rx;
    TxGroup tx;
    SyncGroup sync;
    IsrGroup isr;
    ReqGroup req;
    CmdGroup cmd;
};

inline constexpr size_t kDiagWords{24};

/* Four characters per field, in struct order. */
inline constexpr std::string_view kDiagNames{"dlvrcrcesofmtoutuerrfdwnunknrstrsyncskipbusytouttxerlatemaxiovfl"
                                             "recvquedinvlfullfwrd"
                                             "zrsttdclabrt"};

static_assert(sizeof(Diag) == kDiagWords * 4);
static_assert(std::is_trivially_copyable_v<Diag>);
static_assert(kDiagNames.size() == kDiagWords * 4);

struct DiagGroup {
    std::string_view name;
    size_t first;
    size_t count;
};

inline constexpr std::array<DiagGroup, 6> kDiagGroups{{
    {"rx", 0, 8},
    {"tx", 8, 5},
    {"sync", 13, 2},
    {"isr", 15, 1},
    {"req", 16, 5},
    {"cmd", 21, 3},
}};

/* Counts a failed uart_tx (any non-zero return code) of a downlink frame. */
inline void count_uart_tx(TxGroup &tx, const int rc)
{
    if (rc != 0)
        ++tx.uart_err;
}

/* A template because the stats type lives in a header that includes Zephyr; leaves forwarded untouched. */
template <typename Stats>
void assign_request_stats(ReqGroup &group, const Stats &stats)
{
    group.received = stats.received;
    group.queued = stats.queued;
    group.dropped_invalid = stats.dropped_invalid;
    group.dropped_full = stats.dropped_full;
}

/* A template for the same reason; copies the per-command counts of the request statistics. */
template <typename Stats>
void assign_cmd_stats(CmdGroup &group, const Stats &stats)
{
    group.zero_reset = stats.zero_reset;
    group.temp_drift_calib = stats.temp_drift_calib;
    group.abort_cmd = stats.abort_cmd;
}

/* true: "<group> <name>=<value> ..."; false: "<group> <value> ...". The names cost flash, so the build selects. */
#ifdef ENABLE_LCS_DIAG_NAMES
inline constexpr bool kDiagNamed{true};
#else
inline constexpr bool kDiagNamed{false};
#endif

/* Longest line is "rx" plus 8 values of " 4294967295" (90 characters) or " xxxx=4294967295" (130), plus the NUL. */
inline constexpr size_t kDiagLineMax{kDiagNamed ? 131 : 91};

namespace diag_detail {

/* Appends text; with out == nullptr it only counts. */
class Sink {
public:
    explicit Sink(char *const out) : out_{out} {}

    void put(const char c)
    {
        if (out_ != nullptr)
            out_[len_] = c;
        ++len_;
    }

    void put(const std::string_view s)
    {
        for (const char c : s)
            put(c);
    }

    void put_decimal(const uint32_t value)
    {
        char digits[10];
        size_t n{0};
        uint32_t v{value};
        do {
            digits[n++] = static_cast<char>('0' + v % 10);
            v /= 10;
        } while (v != 0);
        while (n > 0)
            put(digits[--n]);
    }

    size_t length() const
    {
        return len_;
    }

private:
    char *out_;
    size_t len_{0};
};

inline size_t build(const Diag &d, const DiagGroup &group, char *const out)
{
    const auto words{std::bit_cast<std::array<uint32_t, kDiagWords>>(d)};
    Sink sink{out};
    sink.put(group.name);
    for (size_t i{group.first}; i < group.first + group.count; ++i) {
        sink.put(' ');
        if constexpr (kDiagNamed) {
            sink.put(kDiagNames.substr(i * 4, 4));
            sink.put('=');
        }
        sink.put_decimal(words[i]);
    }
    return sink.length();
}

}  // namespace diag_detail

/* Writes one NUL-terminated line without a newline and returns its length. Returns 0 and writes nothing when the
 * group is out of range or the line and its NUL do not fit in cap. */
inline size_t format_group(const Diag &d, const size_t group, char *const buf, const size_t cap)
{
    if (group >= kDiagGroups.size())
        return 0;
    const DiagGroup &g{kDiagGroups[group]};
    const size_t len{diag_detail::build(d, g, nullptr)};
    if (buf == nullptr || len + 1 > cap)
        return 0;
    diag_detail::build(d, g, buf);
    buf[len] = '\0';
    return len;
}

/* Largest distance between consecutive SYNC starts. The first start gives no interval. */
class IntervalTracker {
public:
    void on_start(const uint32_t tick)
    {
        if (started_) {
            const uint32_t interval{tick - last_};
            if (interval > max_)
                max_ = interval;
        }
        started_ = true;
        last_ = tick;
    }

    uint32_t max() const
    {
        return max_;
    }

    void reset()
    {
        started_ = false;
        last_ = 0;
        max_ = 0;
    }

private:
    bool started_{false};
    uint32_t last_{0};
    uint32_t max_{0};
};

}  // namespace lexxhard::lcs
