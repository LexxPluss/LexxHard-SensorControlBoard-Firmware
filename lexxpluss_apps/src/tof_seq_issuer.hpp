/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

/* Persistent 32-bit sequence issuer with injected read/program operations.
 * The caller selects a valid physical partition and sector geometry; an offset
 * check alone cannot prove the correct region was selected. No runtime erase.
 *
 * Rules:
 * - init() validates a commissioned journal; blank is a refusal, not a reset.
 * - Persist and verify a block reservation before issuing any number from it.
 * - Restart skips unused numbers in reserved blocks; never reclaim them.
 * - Refuse absent/corrupt/incompatible/raced journals, write/readback failures
 *   and exhaustion. Never fall back, erase or skip a failed reservation.
 * - Use one non-copyable, non-movable issuer per journal and serialize calls.
 *   Require an erased mark on reservation to detect a second initialized writer.
 *   Copies could duplicate a cached range without touching flash; only the
 *   deleted copy/move operations prevent that case.
 *
 * Candidate journal format: 12-byte header plus fixed-size, checkable marks
 * that distinguish erased and torn writes. The header stores the mapping
 * parameters, including numbers_per_block and the first number (1).
 * SEQ2 rejects SEQ1 and vice versa; changing those parameters would reuse ranges.
 * Do not erase/recommission an incompatible journal: use a separate unused
 * region/board, or explicitly record acceptance of overlapping numbers.
 * Capacity depends on restart frequency and the candidate format.
 * Pure module: no flash driver, Zephyr, allocation or clock.
 */

#include <stddef.h>
#include <stdint.h>

namespace lexxhard::tof_seq_issuer {

/* "SEQ2", big-endian, then the numbers-per-block the journal was commissioned with, then the first
 * number its numbering starts from. A header that disagrees with the running configuration is a
 * corrupt or an incompatible journal, not something to adapt to. */
inline constexpr size_t kHeaderBytes{12};
inline constexpr uint32_t kHeaderMagic{0x53455132UL};  // 'S','E','Q','2'
/* The predecessor, kept only so that finding one can be refused by name rather than as a generic
 * corruption. SEQ1 numbered from zero; see the header comment on why that cannot be adapted to. */
inline constexpr uint32_t kLegacyHeaderMagicSeq1{0x53455131UL};  // 'S','E','Q','1'
inline constexpr size_t kMarkBytes{4};

/* The first number ever issued. ONE, not zero: zero is reserved on the wire as the independent
 * diagnostic frame's placeholder (tof_package_wire_n), so a grid numbered 0 could not be
 * transmitted -- its metadata frame would be a diagnostic with one flag bit flipped. A number that
 * cannot be sent must not be handed out, and this is the only place a number is born. */
inline constexpr uint32_t kFirstNumber{1};
/* A mark for block i holds i XOR this, big-endian. Chosen so that no plausible index produces the
 * erased pattern 0xFFFFFFFF -- init() refuses a capacity that could reach the one index that
 * would. A torn write leaves some bytes erased, so the value will not match and the journal reads
 * as corrupt rather than as a consumed block. */
inline constexpr uint32_t kMarkKey{0xA5A5A5A5UL};

struct storage_ops {
    /* 0, or a negative errno. */
    int (*read)(void *ctx, size_t offset, uint8_t *dst, size_t len){nullptr};
    /* Programs bytes. Flash semantics: the caller guarantees the target is erased. 0, or a
     * negative errno. NOTE THE ABSENCE OF AN ERASE OPERATION -- see the header comment. */
    int (*program)(void *ctx, size_t offset, const uint8_t *src, size_t len){nullptr};
    void *ctx{nullptr};
};

struct config {
    /* The journal region's size. Header plus a whole number of marks; the remainder is unused and
     * init() says so rather than silently rounding. */
    size_t region_bytes{0};
    /* How many sequence numbers one mark reserves. The trade is restart waste against journal
     * capacity, and it is not decided here. Zero is refused. */
    uint32_t numbers_per_block{0};
    storage_ops ops{};
};

enum class state : uint8_t {
    /* init() has not run, or it refused. */
    uninitialised,
    /* Ready to issue. */
    ready,
    /* The region is erased: no header. NOT a fresh start -- commissioning is a separate path. */
    journal_absent,
    /* A header that does not match, a mark that is neither erased nor its expected value, or a
     * written mark after an erased one. */
    journal_corrupt,
    /* A well-formed journal of a format this firmware does not number the same way: SEQ1, or a
     * SEQ2 header whose first number is not this build's. Distinct from corrupt because nothing is
     * damaged -- the media is intelligible and its numbering is not ours, which is a different
     * service action and not an erase. */
    journal_incompatible,
    /* A mark could not be programmed. The mark may be torn, so nothing further is issued. */
    write_failed,
    /* A mark was programmed and did not read back as itself. */
    readback_failed,
    /* No marks left, or the 32-bit sequence space is spent. */
    exhausted,
    /* The next mark to reserve was already consumed: a second writer is on this journal. Refused
     * rather than reconciled, because the numbers that writer reserved are already being handed
     * out. See the note on ONE ISSUER PER JOURNAL above -- this is the runtime half of it, and it
     * cannot catch a COPY, which reserves nothing and leaves no trace. */
    journal_raced,
};

class issuer {
public:
    issuer() = default;
    /* NOT COPYABLE, NOT MOVABLE. A copy inherits a reservation whose mark is already written, so
     * both objects issue the same numbers and the journal shows one mark either way. Move is
     * deleted for the same reason and not merely for symmetry: every member is a scalar, so a move
     * would BE a copy and would leave the source fully armed. */
    issuer(const issuer &) = delete;
    issuer &operator=(const issuer &) = delete;
    issuer(issuer &&) = delete;
    issuer &operator=(issuer &&) = delete;

    /* Scans and validates. Returns 0 only when the journal is usable; otherwise a negative errno,
     * and current() says which refusal it was. */
    int init(const config &c);

    /* COMMISSIONING ONLY, and deliberately not part of the runtime path: writes the header into a
     * region that is already erased. Performs no erase -- a region that is not blank is refused,
     * because making it blank is a service action with its own authorisation, not something a
     * sequence issuer decides. */
    int commission(const config &c);

    /* Hands out the next number, reserving and verifying a block first when the current one is
     * spent. Returns 0 and writes *out, or a negative errno with *out untouched. */
    int issue(uint32_t *out);

    state current() const { return state_; }
    /* Marks written, which is also the number of blocks that may never be reused. */
    uint32_t blocks_consumed() const { return blocks_consumed_; }
    /* How many numbers remain in the block currently held. Zero means the next issue() reserves. */
    uint32_t remaining_in_block() const { return held_ ? block_remaining_ : 0U; }
    uint32_t capacity_blocks() const { return capacity_blocks_; }

private:
    /* Back to the just-constructed state. Used by init() and commission(), which used to say
     * `*this = issuer{}` -- an assignment that no longer exists, and whose absence is the point. */
    void reset();
    int scan();
    int reserve_next_block();

    config config_{};
    state state_{state::uninitialised};
    uint32_t capacity_blocks_{0};
    uint32_t blocks_consumed_{0};
    bool held_{false};
    uint32_t next_number_{0};
    uint32_t block_remaining_{0};
};

const char *state_name(state s);
/* The on-media value of a mark, exposed so a test and a commissioning tool agree with the module
 * rather than reimplementing it. */
uint32_t mark_value(uint32_t block_index);

}
