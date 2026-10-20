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

#include "tof_seq_issuer.hpp"

#include <errno.h>
#include <string.h>

namespace lexxhard::tof_seq_issuer {

namespace {

constexpr uint32_t kErasedWord{0xFFFFFFFFUL};

/* THE COMPILE-TIME REGRESSION FOR THE ONE DEFECT A RUNTIME TEST CANNOT SEE. A copied issuer hands
 * out numbers its original has already reserved, and the program that proves it does not build
 * once copy and move are deleted -- so the assertion has to be about the type, not about a run.
 *
 * Hand-rolled rather than <type_traits>: this build has no libstdc++ headers (the same reason
 * tests/tof_mapping_proof hand-rolls its own), and pulling in a standard library to ask one
 * question about one class would be a larger change than the question deserves. */
template <bool B> struct answer { static constexpr bool value = B; };

template <typename T, typename = void> struct copy_constructible : answer<false> {};
template <typename T>
struct copy_constructible<T, decltype(void(new T(*static_cast<const T *>(nullptr))))>
    : answer<true> {};

template <typename T, typename = void> struct move_constructible : answer<false> {};
template <typename T>
struct move_constructible<T, decltype(void(new T(static_cast<T &&>(*static_cast<T *>(nullptr)))))>
    : answer<true> {};

template <typename T, typename = void> struct copy_assignable : answer<false> {};
template <typename T>
struct copy_assignable<T, decltype(void(*static_cast<T *>(nullptr) =
                                            *static_cast<const T *>(nullptr)))> : answer<true> {};

template <typename T, typename = void> struct move_assignable : answer<false> {};
template <typename T>
struct move_assignable<T, decltype(void(*static_cast<T *>(nullptr) =
                                            static_cast<T &&>(*static_cast<T *>(nullptr))))>
    : answer<true> {};

void put_be32(uint8_t *at, uint32_t value)
{
    at[0] = static_cast<uint8_t>(value >> 24);
    at[1] = static_cast<uint8_t>(value >> 16);
    at[2] = static_cast<uint8_t>(value >> 8);
    at[3] = static_cast<uint8_t>(value);
}

uint32_t get_be32(const uint8_t *at)
{
    return static_cast<uint32_t>(at[0]) << 24 | static_cast<uint32_t>(at[1]) << 16 |
           static_cast<uint32_t>(at[2]) << 8 | static_cast<uint32_t>(at[3]);
}

bool config_sane(const config &c)
{
    if (c.ops.read == nullptr || c.ops.program == nullptr)
        return false;
    if (c.numbers_per_block == 0)
        return false;
    if (c.region_bytes < kHeaderBytes + kMarkBytes)
        return false;
    const size_t marks{(c.region_bytes - kHeaderBytes) / kMarkBytes};
    if (marks == 0 || marks > 0xFFFFFFFEUL)
        return false;
    /* The one index whose mark would read as the erased pattern must be out of reach, so that
     * "erased" and "consumed" can never be the same four bytes. */
    if (static_cast<uint32_t>(marks) > (kErasedWord ^ kMarkKey))
        return false;
    return true;
}

}  // namespace

static_assert(!copy_constructible<issuer>::value,
              "a copyable issuer is a sequence number that can be issued twice");
static_assert(!copy_assignable<issuer>::value,
              "a copyable issuer is a sequence number that can be issued twice");
static_assert(!move_constructible<issuer>::value,
              "every member is a scalar, so a move leaves the source still able to issue");
static_assert(!move_assignable<issuer>::value,
              "every member is a scalar, so a move leaves the source still able to issue");

uint32_t mark_value(uint32_t block_index)
{
    return block_index ^ kMarkKey;
}

int issuer::scan()
{
    uint8_t header[kHeaderBytes]{};
    if (config_.ops.read(config_.ops.ctx, 0, header, sizeof header) != 0) {
        state_ = state::journal_corrupt;
        return -EIO;
    }

    const uint32_t magic{get_be32(&header[0])};
    const uint32_t per_block{get_be32(&header[4])};
    const uint32_t first_number{get_be32(&header[8])};

    /* A fully erased header is an ABSENT journal, not an empty one. Refused: a blank region is how
     * a replaced board or a wiped sector looks, and starting from zero there is precisely how a
     * sequence comes to repeat. */
    if (magic == kErasedWord && per_block == kErasedWord && first_number == kErasedWord) {
        state_ = state::journal_absent;
        return -ENOENT;
    }
    /* THE PREDECESSOR, REFUSED BY NAME. SEQ1 numbered from zero and had an 8-byte header, so its
     * first mark sits where this format keeps the first number: the same bytes mean different
     * blocks and different numbers. Adapting would reissue numbers that were already sent. */
    if (magic == kLegacyHeaderMagicSeq1) {
        state_ = state::journal_incompatible;
        return -ENOTSUP;
    }
    if (magic != kHeaderMagic) {
        state_ = state::journal_corrupt;
        return -EILSEQ;
    }
    /* A journal commissioned with a different block size is not something to adapt to: every
     * consumed mark would mean a different range of numbers than this configuration believes. */
    if (per_block != config_.numbers_per_block) {
        state_ = state::journal_corrupt;
        return -EILSEQ;
    }
    /* The other factor of the same mapping. A journal whose numbering starts elsewhere is
     * intelligible and not ours -- incompatible, not corrupt. */
    if (first_number != kFirstNumber) {
        state_ = state::journal_incompatible;
        return -ENOTSUP;
    }

    capacity_blocks_ = static_cast<uint32_t>((config_.region_bytes - kHeaderBytes) / kMarkBytes);

    /* Count the written prefix, and require it to BE a prefix. An erased mark followed by a written
     * one means something wrote out of order or a mark was lost, and in either case the count of
     * consumed blocks is unknowable -- which is a corrupt journal, not a number to guess. */
    uint32_t consumed{0};
    bool seen_erased{false};
    for (uint32_t i{0}; i < capacity_blocks_; ++i) {
        uint8_t mark[kMarkBytes]{};
        if (config_.ops.read(config_.ops.ctx, kHeaderBytes + static_cast<size_t>(i) * kMarkBytes,
                             mark, sizeof mark) != 0) {
            state_ = state::journal_corrupt;
            return -EIO;
        }
        const uint32_t value{get_be32(mark)};
        if (value == kErasedWord) {
            seen_erased = true;
            continue;
        }
        if (seen_erased) {
            state_ = state::journal_corrupt;
            return -EILSEQ;
        }
        /* Neither erased nor its own value: a torn write, or something else's data. */
        if (value != mark_value(i)) {
            state_ = state::journal_corrupt;
            return -EILSEQ;
        }
        consumed = i + 1U;
    }

    blocks_consumed_ = consumed;
    /* A RESTART SKIPS THE WHOLE RESERVED RANGE. Every consumed mark is treated as spent in full,
     * however few of its numbers were actually handed out, so nothing is reissued. */
    held_ = false;
    block_remaining_ = 0;
    next_number_ = 0;

    if (consumed >= capacity_blocks_) {
        state_ = state::exhausted;
        return -ENOSPC;
    }
    state_ = state::ready;
    return 0;
}

void issuer::reset()
{
    config_ = config{};
    state_ = state::uninitialised;
    capacity_blocks_ = 0;
    blocks_consumed_ = 0;
    held_ = false;
    next_number_ = 0;
    block_remaining_ = 0;
}

int issuer::init(const config &c)
{
    reset();
    if (!config_sane(c)) {
        state_ = state::uninitialised;
        return -EINVAL;
    }
    config_ = c;
    return scan();
}

int issuer::commission(const config &c)
{
    reset();
    if (!config_sane(c))
        return -EINVAL;
    config_ = c;

    /* Refuses anything that is not blank. Making a region blank is a service action with its own
     * authorisation; this function performs no erase and cannot, because storage_ops has none. */
    const size_t marks{(c.region_bytes - kHeaderBytes) / kMarkBytes};
    for (size_t off{0}; off < kHeaderBytes + marks * kMarkBytes; off += kMarkBytes) {
        uint8_t word[kMarkBytes]{};
        if (c.ops.read(c.ops.ctx, off, word, sizeof word) != 0)
            return -EIO;
        if (get_be32(word) != kErasedWord)
            return -EEXIST;
    }

    /* Every parameter of "which numbers does mark i cover" goes on the media, so that firmware
     * which numbers differently is refused instead of deriving an overlapping range. */
    uint8_t header[kHeaderBytes]{};
    put_be32(&header[0], kHeaderMagic);
    put_be32(&header[4], c.numbers_per_block);
    put_be32(&header[8], kFirstNumber);
    if (c.ops.program(c.ops.ctx, 0, header, sizeof header) != 0)
        return -EIO;

    uint8_t back[kHeaderBytes]{};
    if (c.ops.read(c.ops.ctx, 0, back, sizeof back) != 0)
        return -EIO;
    if (memcmp(header, back, sizeof header) != 0)
        return -EIO;
    return 0;
}

int issuer::reserve_next_block()
{
    if (blocks_consumed_ >= capacity_blocks_) {
        state_ = state::exhausted;
        return -ENOSPC;
    }
    /* Reserve numbering from 1; zero is the wire's independent-diagnostic marker.
     * Check 32-bit range before touching the journal. Refuse a block that cannot
     * fit in full, sacrificing at most numbers_per_block-1 top-end values.
     */
    const uint64_t first{static_cast<uint64_t>(blocks_consumed_) * config_.numbers_per_block +
                         kFirstNumber};
    if (first + config_.numbers_per_block > 0x100000000ULL) {
        state_ = state::exhausted;
        return -ENOSPC;
    }

    const uint32_t index{blocks_consumed_};
    uint8_t mark[kMarkBytes]{};
    put_be32(mark, mark_value(index));
    const size_t offset{kHeaderBytes + static_cast<size_t>(index) * kMarkBytes};

    /* Require an erased mark before reserving it. Reprogramming an existing mark
     * can succeed and read back correctly, allowing a second issuer to duplicate
     * the first's range. Refuse journal_raced rather than reconciling counts.
     */
    uint8_t prior[kMarkBytes]{};
    if (config_.ops.read(config_.ops.ctx, offset, prior, sizeof prior) != 0) {
        /* Reported as a corrupt journal, as scan() reports its read failures: this module cannot
         * tell a failing bus from failing media, and either way the mark's state is unknown. */
        state_ = state::journal_corrupt;
        return -EIO;
    }
    if (get_be32(prior) != kErasedWord) {
        state_ = state::journal_raced;
        return -EBUSY;
    }

    /* PERSIST FIRST. A number handed out before its block was recorded would not survive the power
     * loss that happens one instruction later. */
    if (config_.ops.program(config_.ops.ctx, offset, mark, sizeof mark) != 0) {
        /* The mark may be torn. Nothing further is issued and the next block is NOT tried: stepping
         * over a possibly-torn mark would hand out numbers whose persistence was never
         * established, and the torn mark is exactly what a later scan will refuse. */
        state_ = state::write_failed;
        return -EIO;
    }

    /* THEN READ BACK. A program that reported success and did not take is indistinguishable from
     * one that never happened, until someone looks. */
    uint8_t back[kMarkBytes]{};
    if (config_.ops.read(config_.ops.ctx, offset, back, sizeof back) != 0) {
        state_ = state::readback_failed;
        return -EIO;
    }
    if (memcmp(mark, back, sizeof mark) != 0) {
        state_ = state::readback_failed;
        return -EIO;
    }

    blocks_consumed_ = index + 1U;
    next_number_ = static_cast<uint32_t>(first);
    block_remaining_ = config_.numbers_per_block;
    held_ = true;
    return 0;
}

int issuer::issue(uint32_t *out)
{
    if (out == nullptr)
        return -EINVAL;
    if (state_ != state::ready)
        return -EPERM;

    if (!held_ || block_remaining_ == 0U) {
        if (const int rc{reserve_next_block()}; rc != 0)
            return rc;
    }

    *out = next_number_;
    ++next_number_;
    --block_remaining_;
    return 0;
}

const char *state_name(state s)
{
    switch (s) {
    case state::uninitialised:   return "uninitialised";
    case state::ready:           return "ready";
    case state::journal_absent:  return "journal_absent";
    case state::journal_corrupt: return "journal_corrupt";
    case state::journal_incompatible: return "journal_incompatible";
    case state::journal_raced:   return "journal_raced";
    case state::write_failed:    return "write_failed";
    case state::readback_failed: return "readback_failed";
    case state::exhausted:       return "exhausted";
    }
    return "unknown";
}

}
