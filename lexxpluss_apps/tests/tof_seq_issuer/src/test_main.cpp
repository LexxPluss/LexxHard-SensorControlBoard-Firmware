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

/*
 * Power-loss injection for the persistent sequence issuer.
 *
 * The journal is a byte array with FLASH SEMANTICS -- erased is 0xFF and a program may only clear
 * bits -- so the media survives a simulated power loss exactly as real flash would, and a fresh
 * issuer can be scanned over it. The invariant every scenario is measured against is one sentence:
 * NO NUMBER IS EVER ISSUED TWICE, however the power is cut.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "tof_seq_issuer.hpp"

namespace si = lexxhard::tof_seq_issuer;

namespace {

constexpr size_t kRegionBytes{128};                       // header 12 + 29 marks
constexpr uint32_t kBlocks{(kRegionBytes - si::kHeaderBytes) / si::kMarkBytes};
constexpr uint32_t kPerBlock{4};                          // small, so exhaustion is reachable

/* Where a program may be cut short, counted in program CALLS. */
struct media {
    uint8_t bytes[kRegionBytes]{};
    /* Program calls to let through before the injected fault. -1 disables injection. */
    int program_faults_after{-1};
    /* Bytes of the faulting program to apply before "power is lost". A torn write. */
    size_t partial_bytes{0};
    /* Report success for the faulting program even though it was torn or skipped: the case where
     * the write reports OK and did not take, which only a readback can catch. */
    bool fault_reports_success{false};
    int read_faults_after{-1};

    uint32_t programs{0};
    uint32_t reads{0};
    bool powered{true};
};

void wipe(media &m)
{
    memset(m.bytes, 0xFF, sizeof m.bytes);
    m.programs = 0;
    m.reads = 0;
    m.powered = true;
}

int media_read(void *ctx, size_t offset, uint8_t *dst, size_t len)
{
    media &m{*static_cast<media *>(ctx)};
    if (!m.powered)
        return -EIO;
    if (offset + len > kRegionBytes)
        return -EINVAL;
    ++m.reads;
    if (m.read_faults_after >= 0 && static_cast<int>(m.reads) > m.read_faults_after)
        return -EIO;
    memcpy(dst, &m.bytes[offset], len);
    return 0;
}

int media_program(void *ctx, size_t offset, const uint8_t *src, size_t len)
{
    media &m{*static_cast<media *>(ctx)};
    if (!m.powered)
        return -EIO;
    if (offset + len > kRegionBytes)
        return -EINVAL;

    ++m.programs;
    const bool faulting{m.program_faults_after >= 0 &&
                        static_cast<int>(m.programs) > m.program_faults_after};
    const size_t apply{faulting ? (m.partial_bytes < len ? m.partial_bytes : len) : len};

    /* FLASH SEMANTICS: a program may only clear bits. */
    for (size_t i{0}; i < apply; ++i)
        m.bytes[offset + i] &= src[i];

    if (faulting) {
        if (!m.fault_reports_success)
            return -EIO;
        /* Reported success and (possibly) did not take: the readback is the only defence. */
    }
    return 0;
}

si::config config_for(media &m)
{
    si::config c{};
    c.region_bytes = kRegionBytes;
    c.numbers_per_block = kPerBlock;
    c.ops.read = media_read;
    c.ops.program = media_program;
    c.ops.ctx = &m;
    return c;
}

/* Every number this test run has ever seen, so "issued twice" is a measurement and not a
 * judgement. */
struct ledger {
    bool seen[kBlocks * kPerBlock]{};
    uint32_t count{0};
};

void record(ledger &l, uint32_t number)
{
    /* NUMBERING STARTS AT kFirstNumber, so the ledger is indexed from it. Zero is reserved on the
     * wire as the independent diagnostic frame's placeholder, and a number that cannot be
     * transmitted must never be handed out -- asserted here on every number this suite ever sees,
     * not only on the first. */
    zassert_not_equal(0U, number, "the reserved number zero was issued");
    zassert_true(number >= si::kFirstNumber && number - si::kFirstNumber < kBlocks * kPerBlock,
                 "number %u out of the test's range", number);
    const uint32_t index{number - si::kFirstNumber};
    zassert_false(l.seen[index], "NUMBER %u WAS ISSUED TWICE", number);
    l.seen[index] = true;
    ++l.count;
}

}  // namespace

ZTEST(tof_seq_issuer, test_a_blank_region_is_refused_not_a_fresh_start)
{
    media m{};
    wipe(m);
    si::issuer a{};
    zassert_equal(a.init(config_for(m)), -ENOENT);
    zassert_equal(a.current(), si::state::journal_absent);

    uint32_t n{0xDEADBEEF};
    zassert_equal(a.issue(&n), -EPERM, "a blank region issued a number");
    zassert_equal(n, 0xDEADBEEF, "the output was written on refusal");
    zassert_equal(m.programs, 0U, "a refused issuer wrote to the journal");
}

ZTEST(tof_seq_issuer, test_commissioning_is_separate_and_refuses_a_used_region)
{
    media m{};
    wipe(m);
    si::issuer a{};
    zassert_equal(a.commission(config_for(m)), 0);
    /* Commissioning twice is refused: it would otherwise be a way to reset the sequence, which is
     * the one thing the whole module exists to prevent. */
    zassert_equal(a.commission(config_for(m)), -EEXIST);

    zassert_equal(a.init(config_for(m)), 0);
    zassert_equal(a.current(), si::state::ready);
    zassert_equal(a.blocks_consumed(), 0U);
}

ZTEST(tof_seq_issuer, test_numbers_are_dense_and_ordered_within_a_run)
{
    media m{};
    wipe(m);
    si::issuer a{};
    zassert_equal(a.commission(config_for(m)), 0);
    zassert_equal(a.init(config_for(m)), 0);

    ledger l{};
    for (uint32_t i{0}; i < kPerBlock * 3U; ++i) {
        uint32_t n{0};
        zassert_equal(a.issue(&n), 0, "issue %u", i);
        zassert_equal(n, si::kFirstNumber + i, "numbers are not dense inside one run");
        record(l, n);
    }
    /* Three blocks reserved for three blocks' worth of numbers: no mark is spent early. */
    zassert_equal(a.blocks_consumed(), 3U);
}

/* A RESTART BURNS THE REST OF THE BLOCK. One number issued, then power is lost: the next run starts
 * at the next block boundary and the remainder is never reclaimed. This is the cost of N, and it is
 * why the service life depends on restart frequency. */
ZTEST(tof_seq_issuer, test_a_restart_skips_the_whole_reserved_block)
{
    media m{};
    wipe(m);
    si::issuer boot1{};
    zassert_equal(boot1.commission(config_for(m)), 0);
    zassert_equal(boot1.init(config_for(m)), 0);

    ledger l{};
    uint32_t n{0};
    zassert_equal(boot1.issue(&n), 0);
    zassert_equal(n, si::kFirstNumber);
    record(l, n);

    for (uint32_t boot{2}; boot <= 5; ++boot) {
        si::issuer next{};
        zassert_equal(next.init(config_for(m)), 0, "boot %u", boot);
        zassert_equal(next.blocks_consumed(), boot - 1U);
        zassert_equal(next.issue(&n), 0);
        zassert_equal(n, (boot - 1U) * kPerBlock + si::kFirstNumber,
                      "boot %u reissued or went backwards", boot);
        record(l, n);
    }
    zassert_equal(l.count, 5U);
}

/* THE CORE INJECTION: cut the power at every program call, at every partial length, and after every
 * number already issued. A fresh issuer then scans the surviving media, and the ledger holds every
 * number from every run. */
ZTEST(tof_seq_issuer, test_power_loss_at_every_write_point_never_repeats_a_number)
{
    for (uint32_t cut_after{0}; cut_after < kBlocks; ++cut_after) {
        for (size_t partial{0}; partial <= si::kMarkBytes; ++partial) {
            for (int reports_success{0}; reports_success < 2; ++reports_success) {
                media m{};
                wipe(m);
                ledger l{};

                si::issuer first{};
                zassert_equal(first.commission(config_for(m)), 0);
                /* The commissioning program is call 1, so the injection counts from there. */
                m.program_faults_after = static_cast<int>(cut_after) + 1;
                m.partial_bytes = partial;
                m.fault_reports_success = reports_success != 0;

                zassert_equal(first.init(config_for(m)), 0);
                for (uint32_t i{0}; i < kBlocks * kPerBlock; ++i) {
                    uint32_t n{0};
                    if (first.issue(&n) != 0)
                        break;
                    record(l, n);
                }
                /* Whatever the first run managed, it stopped in one of the refusing states or ran
                 * out -- it never continued past a write it could not establish. */
                const si::state after{first.current()};
                zassert_true(after == si::state::write_failed ||
                                 after == si::state::readback_failed ||
                                 after == si::state::exhausted || after == si::state::ready,
                             "unexpected state %s", si::state_name(after));

                /* Power returns. Injection is over; the media is whatever survived. */
                m.program_faults_after = -1;
                m.read_faults_after = -1;
                si::issuer second{};
                const int rc{second.init(config_for(m))};
                if (rc == 0) {
                    for (uint32_t i{0}; i < kBlocks * kPerBlock; ++i) {
                        uint32_t n{0};
                        if (second.issue(&n) != 0)
                            break;
                        record(l, n);
                    }
                } else {
                    /* A torn mark is refused rather than guessed at, which is the only safe
                     * reading: the count of consumed blocks is unknowable. */
                    zassert_true(second.current() == si::state::journal_corrupt ||
                                     second.current() == si::state::exhausted,
                                 "cut_after=%u partial=%zu state=%s", cut_after, partial,
                                 si::state_name(second.current()));
                }
            }
        }
    }
}

/* A PROGRAM FAILURE IS TERMINAL, AND A RETRY MUST NOT STEP OVER IT.
 *
 * This is the case the injection loop above cannot see, because it stops at the first refusal
 * instead of retrying -- and a mutation that advanced past a failed mark and returned a retryable
 * errno passed the whole suite. A caller that retries after a write failure would be issuing
 * numbers from the NEXT block while the previous mark is possibly torn, which is exactly the state
 * a later scan refuses to interpret: the count of consumed blocks would be unknowable while numbers
 * had already been handed out against it. */
ZTEST(tof_seq_issuer, test_a_program_failure_is_terminal_and_a_retry_does_not_advance)
{
    media m{};
    wipe(m);
    si::issuer a{};
    zassert_equal(a.commission(config_for(m)), 0);
    zassert_equal(a.init(config_for(m)), 0);

    /* The next program fails outright, having written nothing. */
    m.program_faults_after = static_cast<int>(m.programs);
    m.partial_bytes = 0;
    m.fault_reports_success = false;

    uint32_t n{0x77};
    zassert_equal(a.issue(&n), -EIO);
    zassert_equal(a.current(), si::state::write_failed);
    zassert_equal(n, 0x77);
    zassert_equal(a.blocks_consumed(), 0U, "a failed write consumed a block");

    /* Retry, several times, with the fault removed: it must STILL refuse, and must never advance
     * to another mark. -EPERM, not a retryable errno. */
    m.program_faults_after = -1;
    for (int i{0}; i < 4; ++i) {
        zassert_equal(a.issue(&n), -EPERM, "retry %d was accepted", i);
        zassert_equal(a.current(), si::state::write_failed);
        zassert_equal(a.blocks_consumed(), 0U, "retry %d advanced the journal", i);
        zassert_equal(n, 0x77);
    }

    /* The same holds for a readback failure: terminal, and no stepping over. */
    media r{};
    wipe(r);
    si::issuer b{};
    zassert_equal(b.commission(config_for(r)), 0);
    zassert_equal(b.init(config_for(r)), 0);
    /* +1 BECAUSE A RESERVATION NOW READS TWICE: first to prove the mark is still erased, then to
     * read back what it programmed. Letting the first one through is what keeps this test about
     * the readback. */
    r.read_faults_after = static_cast<int>(r.reads) + 1;
    zassert_equal(b.issue(&n), -EIO);
    zassert_equal(b.current(), si::state::readback_failed);
    const uint32_t consumed_then{b.blocks_consumed()};
    r.read_faults_after = -1;
    for (int i{0}; i < 4; ++i) {
        zassert_equal(b.issue(&n), -EPERM, "readback retry %d was accepted", i);
        zassert_equal(b.blocks_consumed(), consumed_then, "readback retry %d advanced", i);
    }
}

/* A program that reports success and did not take is caught by the readback, not by the program's
 * return value. */
ZTEST(tof_seq_issuer, test_a_write_that_did_not_take_is_caught_by_the_readback)
{
    media m{};
    wipe(m);
    si::issuer a{};
    zassert_equal(a.commission(config_for(m)), 0);
    zassert_equal(a.init(config_for(m)), 0);

    /* The next program (the first mark) applies nothing and claims success. */
    m.program_faults_after = static_cast<int>(m.programs);
    m.partial_bytes = 0;
    m.fault_reports_success = true;

    uint32_t n{0xABCD};
    zassert_equal(a.issue(&n), -EIO);
    zassert_equal(a.current(), si::state::readback_failed);
    zassert_equal(n, 0xABCD, "a number was issued despite the readback failing");

    /* And it stays refused: no stepping over to the next mark. */
    m.program_faults_after = -1;
    zassert_equal(a.issue(&n), -EPERM);
    zassert_equal(a.blocks_consumed(), 0U);
}

ZTEST(tof_seq_issuer, test_a_readback_that_cannot_be_performed_refuses)
{
    media m{};
    wipe(m);
    si::issuer a{};
    zassert_equal(a.commission(config_for(m)), 0);
    zassert_equal(a.init(config_for(m)), 0);

    /* The SECOND read of this reservation fails -- the readback. The first one, which proves the
     * mark is still erased, is let through. */
    m.read_faults_after = static_cast<int>(m.reads) + 1;
    uint32_t n{0x1234};
    zassert_equal(a.issue(&n), -EIO);
    zassert_equal(a.current(), si::state::readback_failed);
    zassert_equal(n, 0x1234);

    /* The mark WAS programmed, so a later scan sees one consumed block and the numbers in it are
     * never reissued -- the conservative direction. */
    m.read_faults_after = -1;
    si::issuer next{};
    zassert_equal(next.init(config_for(m)), 0);
    zassert_equal(next.blocks_consumed(), 1U);
    zassert_equal(next.issue(&n), 0);
    zassert_equal(n, kPerBlock + si::kFirstNumber,
                  "the block whose readback failed was reused");
}

/* THE OTHER READ IN A RESERVATION: the one that proves the mark is still erased. If it cannot be
 * performed, the mark's state is unknown, so nothing is programmed and nothing is issued. Reported
 * as a corrupt journal, as scan() reports its read failures -- this module cannot tell a failing
 * bus from failing media. */
ZTEST(tof_seq_issuer, test_the_erased_check_read_failing_programs_nothing)
{
    media m{};
    wipe(m);
    si::issuer a{};
    zassert_equal(a.commission(config_for(m)), 0);
    zassert_equal(a.init(config_for(m)), 0);
    const uint32_t programs_before{m.programs};

    m.read_faults_after = static_cast<int>(m.reads);   // the next read, which is the check
    uint32_t n{0x4321};
    zassert_equal(a.issue(&n), -EIO);
    zassert_equal(a.current(), si::state::journal_corrupt);
    zassert_equal(n, 0x4321U);
    zassert_equal(m.programs, programs_before, "a mark was programmed without being checked");
    zassert_equal(a.blocks_consumed(), 0U);
}

ZTEST(tof_seq_issuer, test_a_torn_mark_is_a_corrupt_journal)
{
    for (size_t partial{1}; partial < si::kMarkBytes; ++partial) {
        media m{};
        wipe(m);
        si::issuer a{};
        zassert_equal(a.commission(config_for(m)), 0);

        /* Tear the first mark by hand, with flash semantics. */
        const uint32_t value{si::mark_value(0)};
        uint8_t full[si::kMarkBytes]{static_cast<uint8_t>(value >> 24),
                                    static_cast<uint8_t>(value >> 16),
                                    static_cast<uint8_t>(value >> 8),
                                    static_cast<uint8_t>(value)};
        for (size_t i{0}; i < partial; ++i)
            m.bytes[si::kHeaderBytes + i] &= full[i];

        si::issuer b{};
        zassert_equal(b.init(config_for(m)), -EILSEQ, "partial=%zu", partial);
        zassert_equal(b.current(), si::state::journal_corrupt);
        uint32_t n{0x55};
        zassert_equal(b.issue(&n), -EPERM);
        zassert_equal(n, 0x55);
    }
}

/* A written mark after an erased one makes the consumed count unknowable, so it is corrupt rather
 * than something to interpret. */
ZTEST(tof_seq_issuer, test_a_hole_in_the_journal_is_corrupt)
{
    media m{};
    wipe(m);
    si::issuer a{};
    zassert_equal(a.commission(config_for(m)), 0);

    /* Mark 0 and mark 2 written, mark 1 erased. */
    static constexpr uint32_t kWritten[]{0, 2};
    for (uint32_t idx : kWritten) {
        const uint32_t value{si::mark_value(idx)};
        uint8_t full[si::kMarkBytes]{static_cast<uint8_t>(value >> 24),
                                    static_cast<uint8_t>(value >> 16),
                                    static_cast<uint8_t>(value >> 8),
                                    static_cast<uint8_t>(value)};
        for (size_t i{0}; i < si::kMarkBytes; ++i)
            m.bytes[si::kHeaderBytes + idx * si::kMarkBytes + i] &= full[i];
    }

    si::issuer b{};
    zassert_equal(b.init(config_for(m)), -EILSEQ);
    zassert_equal(b.current(), si::state::journal_corrupt);
}

ZTEST(tof_seq_issuer, test_a_header_that_disagrees_is_corrupt)
{
    media m{};
    wipe(m);
    si::issuer a{};
    si::config c{config_for(m)};
    zassert_equal(a.commission(c), 0);

    /* The same journal read with a different block size: every consumed mark would mean a
     * different range of numbers, so adapting to it would be inventing history. */
    si::config other{c};
    other.numbers_per_block = kPerBlock * 2U;
    si::issuer b{};
    zassert_equal(b.init(other), -EILSEQ);
    zassert_equal(b.current(), si::state::journal_corrupt);

    /* A corrupt magic likewise. */
    media n{};
    wipe(n);
    n.bytes[0] = 0x00;
    si::issuer d{};
    zassert_equal(d.init(config_for(n)), -EILSEQ);
    zassert_equal(d.current(), si::state::journal_corrupt);
}

/* THE HEADER IS THE MAPPING, PINNED BYTE BY BYTE. A mark says "block i is spent"; which numbers
 * that covers is i * numbers_per_block + first number. Both factors are on the media, so firmware
 * that would compute a different range is refused instead of computing it. */
ZTEST(tof_seq_issuer, test_the_header_records_every_factor_of_the_mapping)
{
    media m{};
    wipe(m);
    si::issuer a{};
    zassert_equal(a.commission(config_for(m)), 0);

    const uint8_t expect[si::kHeaderBytes]{
        'S', 'E', 'Q', '2',
        0, 0, 0, static_cast<uint8_t>(kPerBlock),
        0, 0, 0, static_cast<uint8_t>(si::kFirstNumber),
    };
    zassert_equal(0, memcmp(expect, m.bytes, sizeof expect), "the commissioned header changed");
    /* And the marks begin after it, which is why the predecessor's layout cannot be reused. */
    zassert_equal(0xFFU, m.bytes[si::kHeaderBytes], "a mark was written by commissioning");
}

/* THE PREDECESSOR IS REFUSED BY NAME, NOT ADAPTED TO. SEQ1 kept an 8-byte header and numbered from
 * zero: block i covered i * per_block + 0 there and i * per_block + 1 here, so the same marks mean
 * overlapping ranges, and most of what the old firmware sent would be sent again. The refusal is
 * its own state because nothing is damaged -- and because "corrupt" invites an erase, which is the
 * one action that makes a number repeat. */
ZTEST(tof_seq_issuer, test_a_seq1_journal_is_incompatible_rather_than_corrupt)
{
    media m{};
    wipe(m);
    /* An otherwise healthy SEQ1 journal: its magic, its block size, and one consumed mark at the
     * old header's offset of 8. */
    const uint8_t seq1[]{'S', 'E', 'Q', '1', 0, 0, 0, static_cast<uint8_t>(kPerBlock)};
    memcpy(m.bytes, seq1, sizeof seq1);
    const uint32_t value{si::mark_value(0)};
    m.bytes[8] = static_cast<uint8_t>(value >> 24);
    m.bytes[9] = static_cast<uint8_t>(value >> 16);
    m.bytes[10] = static_cast<uint8_t>(value >> 8);
    m.bytes[11] = static_cast<uint8_t>(value);

    si::issuer a{};
    zassert_equal(a.init(config_for(m)), -ENOTSUP);
    zassert_equal(a.current(), si::state::journal_incompatible);

    uint32_t n{0x5A5A};
    zassert_equal(a.issue(&n), -EPERM, "an incompatible journal issued a number");
    zassert_equal(n, 0x5A5AU);
    zassert_equal(m.programs, 0U, "a refused issuer wrote to the journal");

    /* Nor may it be commissioned over: commission() refuses anything that is not blank, so the
     * recovery this refusal does NOT offer is also unavailable by accident. */
    si::issuer b{};
    zassert_equal(b.commission(config_for(m)), -EEXIST);
}

/* A SEQ2 JOURNAL WHOSE NUMBERING IS NOT THIS BUILD'S. The same refusal for the other factor, which
 * is what makes the next change to kFirstNumber refusable rather than silent. */
ZTEST(tof_seq_issuer, test_a_different_first_number_is_incompatible)
{
    media m{};
    wipe(m);
    si::issuer a{};
    zassert_equal(a.commission(config_for(m)), 0);

    /* Clear the low bit of the stored first number, leaving zero: a SEQ2 header commissioned by a
     * build that numbered from zero. That is the scenario this field exists for, and flash can
     * reach it by clearing bits, so it is a state a real medium can actually hold. */
    m.bytes[11] = 0x00;
    si::issuer b{};
    zassert_equal(b.init(config_for(m)), -ENOTSUP);
    zassert_equal(b.current(), si::state::journal_incompatible);
    zassert_equal(b.capacity_blocks(), 0U, "an incompatible journal reported capacity");
}

/* TWO OWNERS ON ONE JOURNAL, WHICH IS WHERE THIS SUITE FOUND A DEFECT. The copy case cannot be
 * tested at runtime -- the program that copies an issuer does not compile, and tof_seq_issuer.cpp
 * carries the static_assert that keeps it that way. This is the other shape: two issuers each
 * init()ed over the same region.
 *
 * It was written expecting disjoint ranges, on the reasoning that the second one's scan sees the
 * first one's mark. The reasoning was half right and the assertion failed. Each object caches its
 * own count, so once the first one's block is spent it reserves the index the second one already
 * took -- and programming the same value over an already-written mark SUCCEEDS on flash and reads
 * back as itself, so both owners then issued identical numbers with nothing on the media to show
 * it. reserve_next_block() now requires the mark to be erased first.
 *
 * What is asserted here: no number is ever issued twice, and the owner that reaches a taken mark
 * refuses by name instead of reissuing. */
ZTEST(tof_seq_issuer, test_a_second_owner_on_one_journal_is_refused_not_reconciled)
{
    media m{};
    wipe(m);
    si::issuer first{};
    zassert_equal(first.commission(config_for(m)), 0);
    zassert_equal(first.init(config_for(m)), 0);

    ledger l{};
    uint32_t n{0};
    zassert_equal(first.issue(&n), 0);
    record(l, n);
    zassert_equal(n, si::kFirstNumber);

    si::issuer second{};
    zassert_equal(second.init(config_for(m)), 0);
    /* The second one did not inherit the block: its scan sees one mark consumed. */
    zassert_equal(second.blocks_consumed(), 1U);

    /* Drain both, interleaved, through the ledger that fails on any repeat. Each owner keeps going
     * until it refuses; the refusal is the expected outcome, not an error in the test. */
    struct owner {
        si::issuer *who;
        bool refused{false};
    };
    owner one{&first};
    owner two{&second};
    /* One step for one owner: a number goes in the ledger, the FIRST refusal must be the race by
     * name, and every refusal after it must be terminal rather than a retry. */
    auto step = [&l](owner &o) {
        uint32_t v{0};
        const int rc{o.who->issue(&v)};
        if (rc == 0) {
            zassert_false(o.refused, "a refused owner issued again");
            record(l, v);
            return;
        }
        if (!o.refused) {
            zassert_equal(rc, -EBUSY, "the first refusal was %d", rc);
            zassert_equal(o.who->current(), si::state::journal_raced);
            o.refused = true;
        } else {
            zassert_equal(rc, -EPERM, "a refused owner returned %d", rc);
        }
    };
    for (uint32_t i{0}; i < 3U * kPerBlock; ++i) {
        step(one);
        step(two);
    }
    zassert_true(one.refused || two.refused,
                 "two owners drew from one journal and neither was refused");
    zassert_true(l.count >= 2U * kPerBlock, "too few numbers to prove anything (%u)", l.count);

    /* And it stays refused: no stepping over the taken mark, ever. */
    si::issuer &loser{one.refused ? first : second};
    uint32_t again{0x77};
    zassert_equal(loser.issue(&again), -EPERM);
    zassert_equal(again, 0x77U);

    /* And every number that came out is backed by a mark of its own -- counted from the media,
     * since neither object can see the other's reservations. */
    uint32_t marks{0};
    for (size_t off{si::kHeaderBytes}; off + si::kMarkBytes <= kRegionBytes; off += si::kMarkBytes) {
        bool erased{true};
        for (size_t i{0}; i < si::kMarkBytes; ++i)
            erased = erased && m.bytes[off + i] == 0xFF;
        if (!erased)
            ++marks;
    }
    zassert_true(marks * kPerBlock >= l.count,
                 "%u numbers came out of %u reserved blocks", l.count, marks);
}

/* THE SAME GUARD SEEN FROM THE MEDIA SIDE: a mark that is already written is never reprogrammed,
 * whoever wrote it. A single owner can never meet this, which is exactly why the condition means
 * "someone else is on this journal" and not "retry". */
ZTEST(tof_seq_issuer, test_a_reservation_refuses_a_mark_that_is_not_erased)
{
    media m{};
    wipe(m);
    si::issuer a{};
    zassert_equal(a.commission(config_for(m)), 0);
    zassert_equal(a.init(config_for(m)), 0);

    /* Block 0's mark, written behind this issuer's back, with the value it would have written
     * itself -- the case a readback cannot catch. */
    const uint32_t value{si::mark_value(0)};
    m.bytes[si::kHeaderBytes + 0] = static_cast<uint8_t>(value >> 24);
    m.bytes[si::kHeaderBytes + 1] = static_cast<uint8_t>(value >> 16);
    m.bytes[si::kHeaderBytes + 2] = static_cast<uint8_t>(value >> 8);
    m.bytes[si::kHeaderBytes + 3] = static_cast<uint8_t>(value);
    const uint32_t programs_before{m.programs};

    uint32_t n{0xABCD};
    zassert_equal(a.issue(&n), -EBUSY);
    zassert_equal(a.current(), si::state::journal_raced);
    zassert_equal(n, 0xABCDU, "a refused issuer wrote its output");
    zassert_equal(m.programs, programs_before, "a refused reservation programmed the journal");
}

ZTEST(tof_seq_issuer, test_the_runtime_interface_has_no_erase)
{
    /* Asserted structurally: storage_ops has exactly the two operations, so a runtime that wanted
     * to erase could not express it. If an erase member is ever added, this fails and the design
     * statement in the header has to be revisited deliberately. */
    zassert_equal(sizeof(si::storage_ops),
                  sizeof(int (*)(void *, size_t, uint8_t *, size_t)) +
                      sizeof(int (*)(void *, size_t, const uint8_t *, size_t)) + sizeof(void *),
                  "storage_ops gained or lost a member");
}

ZTEST(tof_seq_issuer, test_configuration_refusals)
{
    media m{};
    wipe(m);
    si::issuer a{};

    si::config c{config_for(m)};
    c.numbers_per_block = 0;
    zassert_equal(a.init(c), -EINVAL);

    c = config_for(m);
    c.region_bytes = si::kHeaderBytes;      // no room for a single mark
    zassert_equal(a.init(c), -EINVAL);

    c = config_for(m);
    c.ops.program = nullptr;
    zassert_equal(a.init(c), -EINVAL);

    c = config_for(m);
    c.ops.read = nullptr;
    zassert_equal(a.init(c), -EINVAL);
    zassert_equal(a.current(), si::state::uninitialised);
}

ZTEST(tof_seq_issuer, test_state_names)
{
    zassert_equal(0, strcmp(si::state_name(si::state::journal_absent), "journal_absent"));
    zassert_equal(0, strcmp(si::state_name(si::state::journal_corrupt), "journal_corrupt"));
    zassert_equal(0,
                  strcmp(si::state_name(si::state::journal_incompatible), "journal_incompatible"));
    zassert_equal(0, strcmp(si::state_name(si::state::write_failed), "write_failed"));
    zassert_equal(0, strcmp(si::state_name(si::state::readback_failed), "readback_failed"));
    zassert_equal(0, strcmp(si::state_name(si::state::exhausted), "exhausted"));
    zassert_equal(0, strcmp(si::state_name(si::state::journal_raced), "journal_raced"));
}

ZTEST_SUITE(tof_seq_issuer, NULL, NULL, NULL, NULL, NULL);
