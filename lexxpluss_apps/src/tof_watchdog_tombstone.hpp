/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The one thing a task watchdog owes the person who finds the board afterwards.
 *
 * WHY THIS EXISTS. The task watchdog deliberately causes resets. Without a record of why, the field
 * sees "the SCB restarted" and nothing else, and a safety mechanism that cannot be diagnosed is a
 * safety mechanism nobody will trust for long -- the first unexplained reset turns it into the prime
 * suspect for every other fault on the machine. So it leaves a tombstone: which activity stopped,
 * what every counter read at that moment, and when.
 *
 * WHY IT IS NOT THE DIAGNOSTIC BLOCK. The hang-isolation images carry a 1.5 KiB forensics record
 * with a 32-entry ring, written continuously. That is the right tool for an investigation and the
 * wrong one for a product: it costs RAM, it costs writes on every cycle, and most of what it holds
 * answers questions nobody asks about a machine that is working. This is 256 bytes, written exactly
 * once, on the single transition from armed to stopped, and never touched again.
 *
 * WHAT IT DOES NOT PROMISE. Survival across a power cut. DTCM keeps its contents across an IWDG
 * reset, a software reset and an MCUboot revert because none of those drop the rail; pulling the
 * battery clears it, and a reader who assumes otherwise will eventually read somebody else's boot.
 *
 * THE COMMIT ORDER IS THE WHOLE INTEGRITY STORY. A reset can land between any two stores, so a
 * record that is written front to back can be read back half-written and believed. The body and its
 * checksum are written first, a memory barrier follows, and the committed magic is written last. A
 * record whose magic is present therefore has a body that was complete before the magic existed.
 * Anything else -- a missing magic, a version this build does not know, a size that disagrees, a
 * checksum that fails -- is refused rather than interpreted, because a half-read tombstone pointing
 * at the wrong subsystem is worse than no tombstone at all.
 *
 * IT IS WRITE-ONCE PER BOOT. The watchdog latches, so there is exactly one armed-to-stopped
 * transition, and this is written inside it. Nothing rewrites it afterwards: a second write could
 * only overwrite the first fault with its consequences.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace lexxhard::tof_watchdog_tombstone {

/* 256 bytes at the very top of DTCM, inside the 4 KiB the devicetree overlay removes from the
 * region the linker allocates from. The reservation is explicit for a reason: the diagnostic blocks
 * below rely on "the application links nothing into DTCM", which is a true observation about today's
 * images rather than a property anybody enforces. */
inline constexpr uintptr_t kAddress{0x2001FF00};
inline constexpr size_t kSize{256};

/* Deliberately not sequential with the diagnostic magics, so a stale block from a DEV image cannot
 * be mistaken for this one. */
inline constexpr uint32_t kMagicCommitted{0x31544457};  // "WDT1"
inline constexpr uint32_t kMagicEnd{0x444E4557};        // "WEND"
inline constexpr uint32_t kVersion{1};

/* Fixed 32-bit word offsets. Part of the reading procedure, so APPEND ONLY: a reader in the field
 * may be older than the image that wrote the record, and the version check is what protects it. */
enum : size_t {
    kOffCommitted      = 0,   // written LAST. Its absence means the record is not to be believed
    kOffVersion        = 1,
    kOffSize           = 2,
    kOffChecksum       = 3,   // over every word except this one and the committed magic
    kOffBuildId        = 4,   // 4 words, the first 16 bytes of the version string
    kOffPhase          = 8,
    kOffReason         = 9,
    kOffStoppedMs      = 10,
    kOffLastFedMs      = 11,
    kOffAcqBegun       = 12,
    kOffAcqEnded       = 13,
    kOffSendAcqBegun   = 14,
    kOffSendAcqEnded   = 15,
    kOffSendWorkqBegun = 16,
    kOffSendWorkqEnded = 17,
    kOffHealthBegun    = 18,
    kOffHealthEnded    = 19,
    kOffL7Begun        = 20,
    kOffL7Ended        = 21,
    kOffZcanLoops      = 22,
    kOffLongActive     = 23,
    kOffLongBeganMs    = 24,
    kOffBootSeq        = 25,  // optional; zero when the image does not track one
    kOffEndMagic       = (kSize / sizeof(uint32_t)) - 1,
};

struct record {
    uint32_t build_id[4]{};
    uint32_t phase{0};
    uint32_t reason{0};
    uint32_t stopped_ms{0};
    uint32_t last_fed_ms{0};
    uint32_t acq_begun{0}, acq_ended{0};
    uint32_t send_acq_begun{0}, send_acq_ended{0};
    uint32_t send_workq_begun{0}, send_workq_ended{0};
    uint32_t health_begun{0}, health_ended{0};
    uint32_t l7_begun{0}, l7_ended{0};
    uint32_t zcan_loops{0};
    uint32_t long_active{0};
    uint32_t long_began_ms{0};
    uint32_t boot_seq{0};
};

enum class status : uint8_t {
    valid,
    not_committed,    // no magic: either nothing was written or a reset landed mid-write
    wrong_version,    // written by an image this reader does not understand
    wrong_size,
    bad_end_magic,    // the tail is missing, so the record is short or something overwrote it
    bad_checksum,
};

/* Fills every word except the committed magic, calls `fence`, then writes the magic. The fence is a
 * parameter so a host suite can inspect the region at exactly that instant and prove the body was
 * complete and the magic still absent -- which is the property the whole format rests on. */
void write_once(volatile uint32_t *dst, const record &r, void (*fence)(void *), void *fence_ctx);

status read(const volatile uint32_t *src, record &out);

const char *status_name(status s);

}  // namespace lexxhard::tof_watchdog_tombstone
