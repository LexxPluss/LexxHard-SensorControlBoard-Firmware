/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Diagnostic counters of the LCS host and their shell text form. Pure: no Zephyr include.
 * Diag is a flat struct of uint32_t, so its words can be read in declaration order. A line is the group name
 * and the decimal words of the group. Every counter has one writer; the ISR-side counter is incremented
 * atomically by its owner.
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
    uint32_t sent;
    uint32_t skipped;
    uint32_t rejected_busy;
    uint32_t timeout;
};

struct SyncGroup {
    uint32_t late;
    uint32_t max_interval;
};

struct IsrGroup {
    uint32_t rx_queue_overflow;
};

struct Diag {
    RxGroup rx;
    TxGroup tx;
    SyncGroup sync;
    IsrGroup isr;
};

inline constexpr size_t kDiagWords{15};

static_assert(sizeof(Diag) == kDiagWords * 4);
static_assert(std::is_trivially_copyable_v<Diag>);

struct DiagGroup {
    std::string_view name;
    size_t first;
    size_t count;
};

inline constexpr std::array<DiagGroup, 4> kDiagGroups{{
    {"rx", 0, 8},
    {"tx", 8, 4},
    {"sync", 12, 2},
    {"isr", 14, 1},
}};

/* Longest line is "rx" plus 8 values of " 4294967295", 90 characters, plus the NUL. */
inline constexpr size_t kDiagLineMax{91};

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
