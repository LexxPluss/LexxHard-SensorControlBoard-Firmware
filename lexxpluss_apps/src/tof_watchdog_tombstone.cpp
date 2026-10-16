/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * See tof_watchdog_tombstone.hpp. The write ORDER in write_once() is the integrity guarantee.
 */

#include "tof_watchdog_tombstone.hpp"

#if defined(__ZEPHYR__)
#include <zephyr/devicetree.h>
#include <zephyr/sys/barrier.h>
#define TOMBSTONE_BARRIER 1
#else
#define TOMBSTONE_BARRIER 0
#endif

/* THE RESERVATION IS REQUIRED ON THE BOARD, not merely checked when it happens to be there.
 *
 * kAddress is a real DTCM address on the SCB, so an image that writes the record without the
 * reservation writes into memory the linker is still free to allocate from -- and the earlier form
 * of this guard, which asked only whether the node existed, was satisfied by its own absence. A
 * build whose CMakeLists stopped applying overlays/forensics_dtcm.overlay would have compiled
 * silently, and the record would have been left where the linker may put something else on top of
 * it. The board is the discriminator rather than the node, because the host suites are native_sim
 * builds with a real devicetree that simply has no such node, and on them the address is never
 * dereferenced.
 *
 * WHAT THE RESERVATION BUYS, stated narrowly: the record is not allocated over, so it is still
 * there to be read after a reset that keeps the rail up. It says nothing about a power cut, which
 * clears DTCM whatever the devicetree says. */
#if defined(CONFIG_BOARD_LEXXPLUSS_SCB) && !DT_NODE_EXISTS(DT_NODELABEL(forensics_dtcm))
#error "the watchdog tombstone needs overlays/forensics_dtcm.overlay; see lexxpluss_apps/CMakeLists.txt"
#endif

/* Bounded against the reserved region rather than against DTCM, for the reason
 * overlays/forensics_dtcm.overlay gives. */
#if TOMBSTONE_BARRIER && DT_NODE_EXISTS(DT_NODELABEL(forensics_dtcm))
static_assert(lexxhard::tof_watchdog_tombstone::kAddress >=
              DT_REG_ADDR(DT_NODELABEL(forensics_dtcm)));
static_assert(lexxhard::tof_watchdog_tombstone::kAddress +
                  lexxhard::tof_watchdog_tombstone::kSize <=
              DT_REG_ADDR(DT_NODELABEL(forensics_dtcm)) +
                  DT_REG_SIZE(DT_NODELABEL(forensics_dtcm)));
#endif

/* AND OUT OF REACH OF THE ALLOCATOR, which the two assertions above do not say.
 *
 * They place the record inside the declared region. Declaring a region does not remove it from the
 * DTCM node the linker allocates from -- that is the second half of the overlay, the one that
 * shrinks &dtcm to 124 KiB -- and a tree carrying only the first half satisfies both of them while
 * leaving the record exactly where a future translation unit asking for DTCM would be placed. This
 * is the assertion that fails on such a tree. */
#if TOMBSTONE_BARRIER && DT_NODE_EXISTS(DT_NODELABEL(dtcm))
static_assert(lexxhard::tof_watchdog_tombstone::kAddress >=
                  DT_REG_ADDR(DT_NODELABEL(dtcm)) + DT_REG_SIZE(DT_NODELABEL(dtcm)),
              "the record overlaps the DTCM the linker allocates from: the overlay's &dtcm "
              "shrink is missing");
#endif

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

const bool kBarrierAvailable{TOMBSTONE_BARRIER != 0};

void write_once(volatile uint32_t *dst, const record &r, void (*observer)(void *),
                void *observer_ctx)
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
    dst[kOffFeedRc] = static_cast<uint32_t>(r.feed_rc);
    dst[kOffEndMagic] = kMagicEnd;

    dst[kOffChecksum] = checksum_of(dst);

    /* An observation point, and only that: the host suite looks at the region here to prove the
     * body is complete and the magic absent. It is deliberately before the barrier and it supplies
     * nothing -- an earlier version let the caller pass the barrier, which made the format's one
     * guarantee depend on every call site getting it right. */
    if (observer != nullptr)
        observer(observer_ctx);

#if TOMBSTONE_BARRIER
    /* EVERYTHING ABOVE MUST BE VISIBLE BEFORE THE MAGIC BELOW. A reset can land between any two
     * stores; this is what makes "the magic is present" mean "the body was already complete". */
    barrier_dmem_fence_full();
    dst[kOffCommitted] = kMagicCommitted;
#else
    /* No barrier on this port, so the magic is NOT written. A region that reads as empty is worth
     * more than a record whose body may not have landed before the word that vouches for it. */
#endif
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
    out.feed_rc = static_cast<int32_t>(src[kOffFeedRc]);
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
