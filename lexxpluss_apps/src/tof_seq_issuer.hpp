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

/*
 * Persistent sequence issuer: hands out the 32-bit acquisition numbers that the Package grid's
 * identity is built on, and never hands the same one out twice -- across power loss.
 *
 * WHY IT EXISTS. The transport direction chosen is N: the sequence does not restart, so a restart
 * cannot repeat a value, and the whole cross-restart splicing hazard collapses into one question --
 * does the issuer ever repeat? That question is answerable by injection, which is the point of
 * concentrating the risk here.
 *
 * NO FLASH IN THIS MODULE. Storage arrives as injected read and program operations, which is what
 * lets power loss be injected at every write point on a host. It also means this module cannot be
 * the place a partition is chosen wrongly -- and it is worth being explicit that choosing right is
 * somebody else's job: a flash_area handle checks that an offset falls inside the region it was
 * given, and proves nothing about whether that region was the right one. Real wiring must still
 * check the partition's address, its size and the physical sector geometry against each other.
 *
 * THERE IS NO ERASE IN THE RUNTIME INTERFACE, and that is a design statement rather than an
 * omission: the journal is append-only, a consumed block is never reclaimed, and anything that
 * would need an erase -- a blank region, a corrupt journal, a migration -- is refused at runtime
 * and handled by a separate commissioning path. storage_ops has no erase member, so the runtime
 * cannot perform one even by mistake.
 *
 * THE RULES, each of which a test names:
 *
 * - EXPLICIT INITIALISATION. init() scans and validates. A journal that is ABSENT (a blank region)
 *   is not a fresh start: it is refused, because starting from zero is exactly how a sequence
 *   repeats after someone replaces a board or wipes a region.
 * - RESERVE, PERSIST, READ BACK, THEN ISSUE. A block of numbers is marked consumed in the journal
 *   and the mark is read back and verified BEFORE any number from that block is handed out. A
 *   number that was never persisted cannot survive a power loss, so it must not be issued.
 * - A RESTART SKIPS THE WHOLE RESERVED RANGE. Numbers reserved but not issued are discarded, never
 *   reclaimed and never reused. The cost is that a restart burns the remainder of a block, which is
 *   why the service life depends on restart frequency and not only on acquisition rate.
 * - EVERY FAILURE REFUSES. Absent, corrupt, incompatible, raced, program failure, readback failure
 *   and exhaustion all stop issuance. None of them falls back to zero, and none of them retries into
 *   the next block: a program failure may have left a torn mark, and stepping over it would be
 *   issuing numbers whose persistence was never established.
 * - ONE ISSUER PER JOURNAL, AND IT CANNOT BE COPIED. The object holds a reservation that exists
 *   on media as a single mark, so a second object holding the same reservation hands out the same
 *   numbers while the journal records nothing unusual -- the duplication is invisible to a later
 *   scan, which is what makes it worse than a torn write. Copy and move are therefore deleted,
 *   and the compile-time regression that keeps them deleted is in tof_seq_issuer.cpp.
 *
 *   TWO INITIALISED ISSUERS ARE A SEPARATE DEFECT, AND IT WAS REAL. The second one's scan does see
 *   the first one's mark and starts a block later, so for a while their numbers differ -- but each
 *   object caches its own count, so the moment the first one's block is spent it reserves the index
 *   the second one already took, programs the same value over the same mark (which flash accepts
 *   and reads back as itself), and the two hand out identical numbers with nothing on the media to
 *   show it. reserve_next_block() therefore requires the mark it is about to write to be ERASED and
 *   refuses with journal_raced otherwise. The suite measures this; the assertion was written
 *   expecting disjoint ranges and failed, which is how the defect was found.
 *
 *   WHAT NEITHER DEFENCE COVERS: a copy reserves nothing, writes no mark and leaves no trace, so
 *   only the deleted constructor stops it. Nor can this module tell that two issuers share a
 *   storage_ops ctx before one of them races; choosing the region and running one issuer over it
 *   remains the wiring's responsibility.
 * - CALLS ARE SERIAL. There is no locking here and no atomic. issue() reads and writes plain
 *   members, so two threads in it concurrently can hand out one number twice for the same reason a
 *   copy can. The caller serialises -- one thread, or its own mutex.
 *
 * THE JOURNAL FORMAT IS A CANDIDATE, AND IT IS VERSIONED. A 12-byte header then fixed-size marks;
 * a mark must be distinguishable from the erased state AND from a torn write, which is why it is
 * four bytes of checkable content rather than one byte of "something was written here". Capacity
 * arithmetic depends on this choice, so any year-count quoted elsewhere is provisional until it is
 * frozen.
 *
 * WHY THE HEADER CARRIES EVERY PARAMETER OF THE MAPPING. A mark means "block i is spent", and
 * which NUMBERS that block covers is computed as i * numbers_per_block + kFirstNumber. Both
 * factors are therefore part of the on-media meaning, not of the running code: firmware that
 * reads the same marks with a different numbers_per_block or a different first number derives a
 * different range from the same journal, and the ranges overlap. The format was SEQ1 with an
 * 8-byte header while numbering started at zero; numbering now starts at one, so the magic is
 * SEQ2 and the first number is written into the header and checked on every scan. A SEQ1 journal
 * is refused by name, and -- the half that matters more -- SEQ1 firmware refuses a SEQ2 journal,
 * because unknown magic was already a refusal there. A rollback thus fails closed instead of
 * reinterpreting the marks.
 *
 * AN INCOMPATIBLE JOURNAL IS NOT "ERASE AND RECOMMISSION". Making the region blank is what lets a
 * number repeat: the numbers issued under the old semantics were sent, and the new semantics would
 * reissue most of them. The service answer is a different region or a board whose journal was
 * never commissioned -- or an explicit, recorded decision to accept the overlap. The refusal says
 * which journal it found; it does not offer a reset.
 *
 * This change is free TODAY and would not be later: no SEQ1 journal exists on any device, so there
 * is nothing to migrate, and the versioning exists to make the next change refusable rather than
 * silent.
 *
 * Pure: no Zephyr, no allocation, no clock.
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
