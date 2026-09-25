/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * See tof_watchdog_tombstone.hpp. The write ORDER in write_once() is the integrity guarantee.
 */

#include "tof_watchdog_tombstone.hpp"

namespace lexxhard::tof_watchdog_tombstone {

namespace {

constexpr size_t kWords{kSize / sizeof(uint32_t)};

/* Every word except the committed magic and the checksum slot itself. Deliberately covers the end
 * magic, so a record truncated before its tail fails the checksum as well as the tail check -- two
 * independent reasons to refuse it rather than one. */
uint32_t checksum_of(const volatile uint32_t *w)
{
    uint32_t sum{0};
    for (size_t i{0}; i < kWords; ++i) {
        if (i == kOffCommitted || i == kOffChecksum)
            continue;
        /* Rotate before adding so that two words swapped by a bug do not produce the same sum. */
        sum = (sum << 1) | (sum >> 31);
        sum += w[i];
    }
    return sum;
}

}  // namespace

void write_once(volatile uint32_t *dst, const record &r, void (*fence)(void *), void *fence_ctx)
{
    if (dst == nullptr)
        return;

    /* Cleared first, including the committed magic. A stale record from an earlier boot that this
     * one is about to replace must not be readable as a half-valid mixture of the two. */
    for (size_t i{0}; i < kWords; ++i)
        dst[i] = 0;

    dst[kOffVersion] = kVersion;
    dst[kOffSize] = static_cast<uint32_t>(kSize);
    for (size_t i{0}; i < 4; ++i)
        dst[kOffBuildId + i] = r.build_id[i];
    dst[kOffPhase] = r.phase;
    dst[kOffReason] = r.reason;
    dst[kOffStoppedMs] = r.stopped_ms;
    dst[kOffLastFedMs] = r.last_fed_ms;
    dst[kOffAcqBegun] = r.acq_begun;
    dst[kOffAcqEnded] = r.acq_ended;
    dst[kOffSendAcqBegun] = r.send_acq_begun;
    dst[kOffSendAcqEnded] = r.send_acq_ended;
    dst[kOffSendWorkqBegun] = r.send_workq_begun;
    dst[kOffSendWorkqEnded] = r.send_workq_ended;
    dst[kOffHealthBegun] = r.health_begun;
    dst[kOffHealthEnded] = r.health_ended;
    dst[kOffL7Begun] = r.l7_begun;
    dst[kOffL7Ended] = r.l7_ended;
    dst[kOffZcanLoops] = r.zcan_loops;
    dst[kOffLongActive] = r.long_active;
    dst[kOffLongBeganMs] = r.long_began_ms;
    dst[kOffBootSeq] = r.boot_seq;
    dst[kOffEndMagic] = kMagicEnd;

    dst[kOffChecksum] = checksum_of(dst);

    /* EVERYTHING ABOVE MUST BE VISIBLE BEFORE THE MAGIC BELOW. A reset can land between any two
     * stores; this is what makes "the magic is present" mean "the body was already complete". */
    if (fence != nullptr)
        fence(fence_ctx);

    dst[kOffCommitted] = kMagicCommitted;
}

status read(const volatile uint32_t *src, record &out)
{
    out = record{};
    if (src == nullptr)
        return status::not_committed;

    if (src[kOffCommitted] != kMagicCommitted)
        return status::not_committed;
    /* Version before size, because a future format may legitimately be a different size and the
     * useful message is "newer image wrote this", not "the size is wrong". */
    if (src[kOffVersion] != kVersion)
        return status::wrong_version;
    if (src[kOffSize] != kSize)
        return status::wrong_size;
    if (src[kOffEndMagic] != kMagicEnd)
        return status::bad_end_magic;
    if (src[kOffChecksum] != checksum_of(src))
        return status::bad_checksum;

    for (size_t i{0}; i < 4; ++i)
        out.build_id[i] = src[kOffBuildId + i];
    out.phase = src[kOffPhase];
    out.reason = src[kOffReason];
    out.stopped_ms = src[kOffStoppedMs];
    out.last_fed_ms = src[kOffLastFedMs];
    out.acq_begun = src[kOffAcqBegun];
    out.acq_ended = src[kOffAcqEnded];
    out.send_acq_begun = src[kOffSendAcqBegun];
    out.send_acq_ended = src[kOffSendAcqEnded];
    out.send_workq_begun = src[kOffSendWorkqBegun];
    out.send_workq_ended = src[kOffSendWorkqEnded];
    out.health_begun = src[kOffHealthBegun];
    out.health_ended = src[kOffHealthEnded];
    out.l7_begun = src[kOffL7Begun];
    out.l7_ended = src[kOffL7Ended];
    out.zcan_loops = src[kOffZcanLoops];
    out.long_active = src[kOffLongActive];
    out.long_began_ms = src[kOffLongBeganMs];
    out.boot_seq = src[kOffBootSeq];
    return status::valid;
}

const char *status_name(status s)
{
    switch (s) {
    case status::valid:         return "valid";
    case status::not_committed: return "not_committed";
    case status::wrong_version: return "wrong_version";
    case status::wrong_size:    return "wrong_size";
    case status::bad_end_magic: return "bad_end_magic";
    case status::bad_checksum:  return "bad_checksum";
    }
    return "unknown";
}

}  // namespace lexxhard::tof_watchdog_tombstone
