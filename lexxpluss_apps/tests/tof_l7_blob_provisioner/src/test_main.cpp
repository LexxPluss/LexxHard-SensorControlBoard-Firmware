/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host tests for the L7 blob provisioner, against a fake flash with NOR semantics.
 *
 * THE RECORD IS THE REAL ONE. The Makefile generates it with the L7 blob-record generator from the
 * vendored VL53L7CX_FIRMWARE -- the same tool and inputs as the provisioner image -- and the
 * accept-list is the committed one the L7 runtime uses. A synthetic record would need a second
 * implementation of the format in this file, and the property that matters most, "what this image
 * writes is what the L7 image accepts", would then be tested against the copy.
 *
 * THE FAKE IS NOR FLASH, not a byte array: erase sets bytes to 0xff and a write can only clear bits
 * (stored &= written). That is what makes "an interrupted attempt" and "a torn marker" real states
 * here rather than states the test has to invent, and it is what the classification rule is about.
 *
 * POWER LOSS is modelled as an operation that has a partial effect and then fails. The provisioner
 * stops on the first failure and issues no further erase or write, so for the flash the two are the
 * same thing -- and the reboot is a fresh run over the same flash.
 *
 * TWO WINDOWS DO NOT RECOVER, by design, and the tests say so rather than skipping them: a torn
 * header write, and a torn erase at the start of a retry. In both the next boot must see `foreign`
 * and touch nothing. Every other interruption must recover on the next boot.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_l7_blob_provisioner.hpp"
#include "vl53l7cx_blob_expectation.hpp"

namespace prov = lexxhard::tof_l7_blob_provisioner;
namespace blob = lexxhard::tof_l7_blob;

namespace {

const uint8_t kRecord[] = {
#include "l7_blob_record.inc"
};

constexpr size_t kRegion{0x20000};
constexpr size_t kHeader{blob::kHeaderSize};
constexpr size_t kMarkerAt{sizeof kRecord};
constexpr uint8_t kMarker[4]{'L', '7', 'O', 'K'};

enum class op_kind : uint8_t { erase, write };
struct op {
    op_kind kind;
    size_t offset;
    size_t len;
};

struct fake_flash {
    uint8_t mem[kRegion];
    /* erase/write operations, in order */
    op log[1024];
    size_t log_len{0};
    int ops{0};
    int reads{0};
    /* Fail (after a partial effect) at this erase/write index; -1 never. */
    int fail_op_at{-1};
    bool partial{true};
    /* Fail this read index; -1 never. */
    int fail_read_at{-1};
    bool out_of_bounds{false};
    /* Set if the marker was ever written while the body was not exactly the record. */
    bool marker_over_bad_body{false};
    /* Body bytes read since the last body write, up to the marker write. */
    size_t body_read_since_write{0};
    bool marker_write_seen{false};
    /* Clears one bit of the marker area right after the last body write: flash that changed under
     * the provisioner between the scan and the commit. */
    bool stray_marker_bit_after_body{false};
    /* Silent faults: the call returns 0 but the flash did not do all of it. -1 never. */
    long silent_unprogrammed_at{-1};  // this byte keeps its erased value on a successful write
    long silent_unerased_at{-1};      // this byte keeps its old value on a successful erase
    /* Flips a payload byte of the SOURCE record during the first payload write: the embedded copy
     * changed after the self-check, so the read-back compares against the same wrong bytes. */
    uint8_t *corrupt_source{nullptr};
};

fake_flash g_flash;

bool in_bounds(size_t off, size_t len)
{
    return off <= kRegion && len <= kRegion - off;
}

int f_read(void *ctx, size_t off, void *dst, size_t len)
{
    auto *f{static_cast<fake_flash *>(ctx)};
    if (!in_bounds(off, len)) {
        f->out_of_bounds = true;
        return -EINVAL;
    }
    if (f->reads++ == f->fail_read_at)
        return -EIO;
    memcpy(dst, f->mem + off, len);
    if (!f->marker_write_seen && off < kMarkerAt) {
        const size_t end{off + len < kMarkerAt ? off + len : kMarkerAt};
        f->body_read_since_write += end - off;
    }
    return 0;
}

int f_erase(void *ctx, size_t off, size_t len)
{
    auto *f{static_cast<fake_flash *>(ctx)};
    if (!in_bounds(off, len)) {
        f->out_of_bounds = true;
        return -EINVAL;
    }
    if (f->log_len < sizeof f->log / sizeof f->log[0])
        f->log[f->log_len++] = {op_kind::erase, off, len};
    if (f->ops++ == f->fail_op_at) {
        /* A torn erase: raises bits across the sector without finishing it. Every other byte is
         * one such state; the point is that the result is neither the old content nor blank. */
        if (f->partial)
            for (size_t i{0}; i < len; i += 2)
                f->mem[off + i] = 0xff;
        return -EIO;
    }
    const uint8_t kept{f->silent_unerased_at >= 0 ? f->mem[f->silent_unerased_at] : uint8_t{0}};
    memset(f->mem + off, 0xff, len);
    if (f->silent_unerased_at >= 0)
        f->mem[f->silent_unerased_at] = kept;
    return 0;
}

int f_write(void *ctx, size_t off, const void *src, size_t len)
{
    auto *f{static_cast<fake_flash *>(ctx)};
    if (!in_bounds(off, len)) {
        f->out_of_bounds = true;
        return -EINVAL;
    }
    if (f->log_len < sizeof f->log / sizeof f->log[0])
        f->log[f->log_len++] = {op_kind::write, off, len};
    const bool is_marker{off >= kMarkerAt};
    if (is_marker) {
        f->marker_write_seen = true;
        if (memcmp(f->mem, kRecord, sizeof kRecord) != 0)
            f->marker_over_bad_body = true;
    } else {
        f->body_read_since_write = 0;
    }
    if (f->corrupt_source != nullptr && off == kHeader) {
        f->corrupt_source[kHeader + 30000] ^= 0x01;
        f->corrupt_source = nullptr;
    }
    const auto *s{static_cast<const uint8_t *>(src)};
    const bool fail{f->ops++ == f->fail_op_at};
    const size_t n{fail ? (f->partial ? len / 2 : 0) : len};
    for (size_t i{0}; i < n; ++i)
        if (static_cast<long>(off + i) != f->silent_unprogrammed_at)
            f->mem[off + i] &= s[i];
    /* A torn byte: half its bits programmed. Only the bits the target clears can change. */
    if (fail && f->partial && n < len)
        f->mem[off + n] &= static_cast<uint8_t>(s[n] | 0xf0);
    if (!fail && !is_marker && off + len == kMarkerAt && f->stray_marker_bit_after_body)
        f->mem[kMarkerAt] &= 0xfe;
    return fail ? -EIO : 0;
}

prov::flash_ops ops_for(fake_flash &f)
{
    return prov::flash_ops{f_read, f_erase, f_write, &f};
}

prov::config cfg_for(fake_flash &f)
{
    prov::config c{};
    c.region_size = kRegion;
    c.record = kRecord;
    c.record_len = sizeof kRecord;
    c.accepted = blob::kAcceptedPayloadList;
    c.mapped_base = f.mem;
    return c;
}

void reset_counters(fake_flash &f)
{
    f.log_len = 0;
    f.ops = 0;
    f.reads = 0;
    f.fail_op_at = -1;
    f.partial = true;
    f.fail_read_at = -1;
    f.out_of_bounds = false;
    f.marker_over_bad_body = false;
    f.body_read_since_write = 0;
    f.marker_write_seen = false;
    f.stray_marker_bit_after_body = false;
    f.silent_unprogrammed_at = -1;
    f.silent_unerased_at = -1;
    f.corrupt_source = nullptr;
}

void make_blank(fake_flash &f)
{
    memset(f.mem, 0xff, kRegion);
    reset_counters(f);
}

void make_committed(fake_flash &f)
{
    make_blank(f);
    memcpy(f.mem, kRecord, sizeof kRecord);
    memcpy(f.mem + kMarkerAt, kMarker, sizeof kMarker);
}

void make_uncommitted(fake_flash &f)
{
    make_blank(f);
    memcpy(f.mem, kRecord, sizeof kRecord);
}

/* What a reset part-way through the payload leaves: the full header and the first `bytes` of the
 * record, the rest still erased. */
void make_interrupted(fake_flash &f, size_t bytes)
{
    make_blank(f);
    memcpy(f.mem, kRecord, bytes);
}

size_t count(const fake_flash &f, op_kind k)
{
    size_t n{0};
    for (size_t i{0}; i < f.log_len; ++i)
        n += f.log[i].kind == k;
    return n;
}

bool marker_present(const fake_flash &f)
{
    return memcmp(f.mem + kMarkerAt, kMarker, sizeof kMarker) == 0;
}

/* The number of erase/write operations a clean run from blank issues. */
int clean_run_ops()
{
    make_blank(g_flash);
    (void)prov::run(ops_for(g_flash), cfg_for(g_flash));
    return g_flash.ops;
}

/* The first bit the target has set, in the target byte at `at`: clearing it gives a byte that no
 * partial program of this record can produce. */
uint8_t with_a_target_one_cleared(size_t at)
{
    const uint8_t t{kRecord[at]};
    return static_cast<uint8_t>(t & ~(t & (0u - t)));
}

size_t first_nonzero_from(size_t at)
{
    while (at < sizeof kRecord && kRecord[at] == 0)
        ++at;
    return at;
}

void expect_refused_untouched(const char *what)
{
    static uint8_t before_bytes[kRegion];
    memcpy(before_bytes, g_flash.mem, kRegion);
    const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};
    zassert_equal(r.initial, prov::found::foreign, "%s was classified %s, not foreign", what,
                  prov::found_name(r.initial));
    zassert_equal(r.result, prov::outcome::foreign_or_unknown_data, "%s", what);
    zassert_equal(g_flash.log_len, 0u, "%s was erased or written", what);
    zassert_false(r.flash_operation_attempted, "%s", what);
    zassert_mem_equal(g_flash.mem, before_bytes, kRegion, "%s changed", what);
}

void before(void *)
{
    make_blank(g_flash);
}

}  // namespace

ZTEST_SUITE(tof_l7_blob_provisioner, NULL, NULL, before, NULL, NULL);

/* --- the embedded record is the one the L7 image accepts --- */

ZTEST(tof_l7_blob_provisioner, test_the_embedded_record_is_accepted_by_the_production_reader)
{
    zassert_equal(sizeof kRecord, 86080u, "the generated record is not 64 + 86,016 bytes");
    zassert_equal(prov::check_embedded(cfg_for(g_flash)), blob::status::ok,
                  "the production reader would refuse this image's own record");
    /* And it is literally the accept-list's digest, not merely a digest the list happens to hold. */
    zassert_equal(blob::kAcceptedPayloadList.count, 1u);
    zassert_mem_equal(kRecord + 12, blob::kAcceptedPayloadList.entries[0].payload_digest,
                      blob::kDigestSize, "the payload digest is not the accept-list's");
}

ZTEST(tof_l7_blob_provisioner, test_a_record_the_reader_would_refuse_touches_nothing)
{
    /* One payload byte changed in a RAM copy: the header's digest no longer matches. */
    static uint8_t damaged[sizeof kRecord];
    memcpy(damaged, kRecord, sizeof kRecord);
    damaged[kHeader + 1000] ^= 0x01;
    prov::config c{cfg_for(g_flash)};
    c.record = damaged;
    prov::report r{prov::run(ops_for(g_flash), c)};
    zassert_equal(r.result, prov::outcome::bad_embedded_record, "got %s", prov::outcome_name(r.result));
    zassert_equal(g_flash.log_len, 0u);
    zassert_equal(g_flash.reads, 0, "the partition was even read");

    /* An intact record the accept-list does not name. */
    static const blob::expectation other[]{{86016, {}}};
    c = cfg_for(g_flash);
    c.accepted = blob::accept_list{other, 1};
    r = prov::run(ops_for(g_flash), c);
    zassert_equal(r.result, prov::outcome::bad_embedded_record, "got %s", prov::outcome_name(r.result));
    zassert_equal(g_flash.log_len, 0u);
}

/* --- the four states that are allowed, and what each costs --- */

ZTEST(tof_l7_blob_provisioner, test_blank_is_provisioned_and_verified_in_order)
{
    const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};

    zassert_equal(r.initial, prov::found::blank);
    zassert_equal(r.result, prov::outcome::provisioned_and_verified, "got %s",
                  prov::outcome_name(r.result));
    zassert_equal(r.final_status, blob::status::ok);
    zassert_true(r.flash_operation_attempted);
    zassert_false(g_flash.out_of_bounds, "an operation left the partition");
    zassert_equal(r.erase_attempts, 1u);
    zassert_equal(r.write_attempts, static_cast<uint32_t>(g_flash.log_len - 1));

    /* Exactly one erase, first, of exactly the partition. */
    zassert_equal(count(g_flash, op_kind::erase), 1u);
    zassert_equal(g_flash.log[0].kind, op_kind::erase, "the first operation is not the erase");
    zassert_equal(g_flash.log[0].offset, 0u);
    zassert_equal(g_flash.log[0].len, kRegion, "the erase is not exactly the partition");

    /* Then the header, alone: the one write whose interruption cannot be recovered is this small. */
    zassert_equal(g_flash.log[1].kind, op_kind::write);
    zassert_equal(g_flash.log[1].offset, 0u);
    zassert_equal(g_flash.log[1].len, kHeader, "the first write is not the 64-byte header alone");

    /* Then the payload, contiguous and inside the record; the marker last, four bytes. */
    size_t expect{kHeader};
    for (size_t i{2}; i + 1 < g_flash.log_len; ++i) {
        zassert_equal(g_flash.log[i].kind, op_kind::write);
        zassert_equal(g_flash.log[i].offset, expect, "payload writes are not contiguous");
        expect += g_flash.log[i].len;
    }
    zassert_equal(expect, kMarkerAt, "the payload writes do not end where the marker starts");
    const op &last{g_flash.log[g_flash.log_len - 1]};
    zassert_equal(last.kind, op_kind::write);
    zassert_equal(last.offset, kMarkerAt, "the last write is not the marker");
    zassert_equal(last.len, 4u);

    /* The whole body was read back after the last body write and before the marker. */
    zassert_true(g_flash.body_read_since_write >= sizeof kRecord,
                 "the body was not read back in full before the commit");
    zassert_false(g_flash.marker_over_bad_body);
    zassert_true(marker_present(g_flash));
    zassert_mem_equal(g_flash.mem, kRecord, sizeof kRecord);
}

ZTEST(tof_l7_blob_provisioner, test_a_committed_record_costs_no_erase_and_no_write)
{
    make_committed(g_flash);
    const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};

    zassert_equal(r.initial, prov::found::committed_exact);
    zassert_equal(r.result, prov::outcome::already_provisioned);
    zassert_equal(r.final_status, blob::status::ok);
    zassert_false(r.flash_operation_attempted, "an already-provisioned board was written to");
    zassert_equal(g_flash.log_len, 0u, "an erase or a write happened");
}

ZTEST(tof_l7_blob_provisioner, test_an_uncommitted_exact_record_gets_only_its_marker)
{
    make_uncommitted(g_flash);
    const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};

    zassert_equal(r.initial, prov::found::uncommitted_exact);
    zassert_equal(r.result, prov::outcome::provisioned_and_verified);
    zassert_equal(count(g_flash, op_kind::erase), 0u, "an uncommitted exact record was erased");
    zassert_equal(g_flash.log_len, 1u, "more than the marker was written");
    zassert_equal(g_flash.log[0].offset, kMarkerAt);
    zassert_equal(g_flash.log[0].len, 4u);
    /* The scan read the body once, and the commit read it back again before the marker. */
    zassert_true(g_flash.body_read_since_write >= 2 * sizeof kRecord,
                 "the body was not read back before the marker");
    zassert_true(marker_present(g_flash));
}

ZTEST(tof_l7_blob_provisioner, test_a_complete_header_with_partial_payload_is_redone)
{
    static const size_t kWritten[]{kHeader, kHeader + 1, 40000, 80000};
    for (const size_t bytes : kWritten) {
        make_interrupted(g_flash, bytes);
        const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};
        zassert_equal(r.initial, prov::found::interrupted_exact, "%u bytes: %s",
                      static_cast<unsigned>(bytes), prov::found_name(r.initial));
        zassert_equal(r.result, prov::outcome::provisioned_and_verified);
        zassert_equal(count(g_flash, op_kind::erase), 1u);
    }
}

/* --- what must never be touched --- */

ZTEST(tof_l7_blob_provisioner, test_a_bit_compatible_byte_without_our_header_is_foreign)
{
    /* THE COUNTER-EXAMPLE: one byte that a partial program of this record COULD have produced, the
     * rest erased. The bit relation alone would call it an interrupted attempt and erase it. */
    zassert_equal(kRecord[0], 0x4c, "the record no longer starts with 'L'");
    make_blank(g_flash);
    g_flash.mem[0] = 0x4d;  // 0x4d & 0x4c == 0x4c
    expect_refused_untouched("0x4d over a target of 0x4c");

    make_blank(g_flash);
    g_flash.mem[0] = 0xfe;  // one cleared bit the target also clears
    expect_refused_untouched("a single compatible cleared bit");

    /* A compatible byte deep in the payload, header erased. */
    make_blank(g_flash);
    g_flash.mem[kHeader + 5000] = kRecord[kHeader + 5000];
    expect_refused_untouched("a payload byte under an erased header");
}

ZTEST(tof_l7_blob_provisioner, test_payload_under_an_incomplete_header_is_foreign)
{
    /* The payload written and exact, the header only partly there: whatever wrote this, the one
     * piece of evidence that it was this provisioner is missing. */
    static const size_t kHeaderWritten[]{0, 1, 32, kHeader - 1};
    for (const size_t header_bytes : kHeaderWritten) {
        make_blank(g_flash);
        memcpy(g_flash.mem, kRecord, header_bytes);
        memcpy(g_flash.mem + kHeader, kRecord + kHeader, 40000);
        expect_refused_untouched("payload under an incomplete header");
    }
}

ZTEST(tof_l7_blob_provisioner, test_foreign_or_damaged_data_is_never_erased_or_written)
{
    /* Unrelated bytes at the start. */
    make_blank(g_flash);
    memset(g_flash.mem, 0x00, 16);
    expect_refused_untouched("zeros at the start");

    /* The committed record with one header bit cleared that the record has set. */
    make_committed(g_flash);
    const size_t h{first_nonzero_from(0)};
    g_flash.mem[h] = with_a_target_one_cleared(h);
    expect_refused_untouched("the record with one header bit damaged");

    /* The same in the payload: the header is exact, but no partial program clears that bit. */
    make_committed(g_flash);
    const size_t p{first_nonzero_from(kHeader + 3000)};
    g_flash.mem[p] = with_a_target_one_cleared(p);
    expect_refused_untouched("the record with one payload bit damaged");

    /* Anything in the spare area past the marker. */
    make_committed(g_flash);
    g_flash.mem[kRegion - 1] = 0x7f;
    expect_refused_untouched("a byte in the spare area");

    /* A marker that is not ours. */
    make_uncommitted(g_flash);
    g_flash.mem[kMarkerAt] = 0x00;
    expect_refused_untouched("a foreign marker");
}

/* --- every failing step leaves the marker unwritten --- */

ZTEST(tof_l7_blob_provisioner, test_any_failed_operation_before_the_commit_never_writes_the_marker)
{
    const int total_ops{clean_run_ops()};
    for (int partial{0}; partial <= 1; ++partial) {
        for (int at{0}; at < total_ops - 1; ++at) {
            make_blank(g_flash);
            g_flash.fail_op_at = at;
            g_flash.partial = partial != 0;
            const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};
            zassert_false(r.ok(), "a failed operation %d was reported as success", at);
            zassert_false(g_flash.marker_write_seen, "the marker was written after failing op %d", at);
            zassert_false(marker_present(g_flash));
            zassert_true(r.flash_operation_attempted, "a failed operation is not counted as attempted");
            zassert_true(r.result == (at == 0 ? prov::outcome::erase_failed : prov::outcome::write_failed),
                         "op %d failed as %s", at, prov::outcome_name(r.result));
        }
    }
}

ZTEST(tof_l7_blob_provisioner, test_every_failed_read_is_a_failure_and_never_commits_over_bad_data)
{
    make_blank(g_flash);
    (void)prov::run(ops_for(g_flash), cfg_for(g_flash));
    const int total_reads{g_flash.reads};
    for (int at{0}; at < total_reads; ++at) {
        make_blank(g_flash);
        g_flash.fail_read_at = at;
        const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};
        zassert_false(r.ok(), "read %d failed and the run still succeeded (%s)", at,
                      prov::outcome_name(r.result));
        zassert_false(g_flash.marker_over_bad_body);
    }
}

ZTEST(tof_l7_blob_provisioner, test_a_write_that_silently_missed_a_byte_is_caught_before_the_marker)
{
    /* Every write returns 0, yet one payload byte stayed erased: only the read-back can see it. */
    const size_t at{first_nonzero_from(kHeader + 12345)};
    zassert_true(kRecord[at] != 0xff);
    make_blank(g_flash);
    g_flash.silent_unprogrammed_at = static_cast<long>(at);
    const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};
    zassert_equal(r.result, prov::outcome::readback_failed, "got %s", prov::outcome_name(r.result));
    zassert_equal(r.failed_offset, at, "the reported offset is not the missed byte");
    zassert_false(g_flash.marker_write_seen, "the marker was written over a body that did not match");
}

ZTEST(tof_l7_blob_provisioner, test_a_source_that_changed_after_the_self_check_fails_the_digest)
{
    /* The byte compare cannot see this -- flash and source agree, both wrong -- and it is exactly
     * what the payload digest, recomputed over what was read back, is for. */
    static uint8_t source[sizeof kRecord];
    memcpy(source, kRecord, sizeof kRecord);
    make_blank(g_flash);
    g_flash.corrupt_source = source;
    prov::config c{cfg_for(g_flash)};
    c.record = source;
    const prov::report r{prov::run(ops_for(g_flash), c)};
    zassert_equal(r.result, prov::outcome::digest_mismatch, "got %s", prov::outcome_name(r.result));
    zassert_false(g_flash.marker_write_seen, "the marker was written over a payload with the wrong digest");
}

ZTEST(tof_l7_blob_provisioner, test_an_erase_that_silently_left_a_byte_is_caught_before_any_write)
{
    /* The erase returns 0 but one byte of an interrupted attempt survived it. */
    make_interrupted(g_flash, 40000);
    const size_t at{first_nonzero_from(kHeader + 20000)};
    g_flash.silent_unerased_at = static_cast<long>(at);
    const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};
    zassert_equal(r.result, prov::outcome::erase_verify_failed, "got %s", prov::outcome_name(r.result));
    zassert_equal(r.failed_offset, at);
    zassert_equal(count(g_flash, op_kind::write), 0u, "something was written over an unverified erase");
}

ZTEST(tof_l7_blob_provisioner, test_a_failed_marker_write_is_not_success)
{
    const int marker_op{clean_run_ops() - 1};
    make_blank(g_flash);
    g_flash.fail_op_at = marker_op;
    g_flash.partial = true;
    const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};
    zassert_equal(r.result, prov::outcome::commit_failed, "got %s", prov::outcome_name(r.result));
    zassert_false(r.ok());
}

ZTEST(tof_l7_blob_provisioner, test_a_marker_area_that_changed_under_the_run_is_not_programmed)
{
    make_blank(g_flash);
    g_flash.stray_marker_bit_after_body = true;
    const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};
    zassert_equal(r.result, prov::outcome::marker_not_blank, "got %s", prov::outcome_name(r.result));
    zassert_false(g_flash.marker_write_seen, "the marker was programmed over a non-blank area");
    zassert_false(r.ok());
}

ZTEST(tof_l7_blob_provisioner, test_a_torn_marker_under_an_exact_header_is_erased_and_redone)
{
    /* Stray bits in the marker area that the marker also clears: the header proves the attempt was
     * ours, and the right recovery is erase and redo, never a program over whatever is there. */
    make_uncommitted(g_flash);
    g_flash.mem[kMarkerAt] = static_cast<uint8_t>(kMarker[0] | 0x80);
    const prov::report r{prov::run(ops_for(g_flash), cfg_for(g_flash))};
    zassert_equal(r.initial, prov::found::interrupted_exact);
    zassert_equal(r.result, prov::outcome::provisioned_and_verified);
    zassert_equal(count(g_flash, op_kind::erase), 1u, "the torn marker was not erased first");
}

/* --- power loss after every flash operation, then a reboot --- */

namespace {

/* A reset during operation `at` of a run that started from `start`: what the next boot must do. */
void check_reset_then_reboot(void (*start)(fake_flash &), int at, bool partial, bool must_recover)
{
    start(g_flash);
    g_flash.fail_op_at = at;
    g_flash.partial = partial;
    (void)prov::run(ops_for(g_flash), cfg_for(g_flash));
    zassert_false(g_flash.marker_over_bad_body, "the marker went over a bad body (op %d)", at);
    zassert_false(g_flash.out_of_bounds);

    reset_counters(g_flash);
    const prov::report again{prov::run(ops_for(g_flash), cfg_for(g_flash))};
    if (!must_recover) {
        zassert_equal(again.initial, prov::found::foreign,
                      "an unrecoverable window (op %d, partial %d) was classified %s", at, partial,
                      prov::found_name(again.initial));
        zassert_equal(g_flash.log_len, 0u, "the next boot touched an unidentifiable sector (op %d)", at);
        zassert_false(again.flash_operation_attempted);
        return;
    }
    zassert_true(again.ok(), "reset at op %d (partial %d) did not recover: found %s, %s", at, partial,
                 prov::found_name(again.initial), prov::outcome_name(again.result));
    zassert_equal(again.final_status, blob::status::ok);

    /* And a third boot changes nothing. */
    reset_counters(g_flash);
    const prov::report third{prov::run(ops_for(g_flash), cfg_for(g_flash))};
    zassert_equal(third.result, prov::outcome::already_provisioned);
    zassert_equal(g_flash.log_len, 0u, "an already-provisioned boot wrote flash");
}

void start_blank(fake_flash &f)
{
    make_blank(f);
}

void start_interrupted(fake_flash &f)
{
    make_interrupted(f, 40000);
}

}  // namespace

ZTEST(tof_l7_blob_provisioner, test_a_reset_from_blank_recovers_except_during_the_header_write)
{
    /* Operation 0 erases an already-blank sector, so even a torn erase leaves it blank. Operation 1
     * is the header alone: torn, it leaves an incomplete header -- the first stated window. Every
     * later operation happens behind a complete header and must recover. */
    const int total_ops{clean_run_ops()};
    for (int partial{0}; partial <= 1; ++partial) {
        for (int at{0}; at < total_ops; ++at) {
            const bool torn_header{partial != 0 && at == 1};
            check_reset_then_reboot(start_blank, at, partial != 0, !torn_header);
        }
    }
}

ZTEST(tof_l7_blob_provisioner, test_a_reset_during_a_retry_recovers_except_during_its_erase_or_header)
{
    /* The retry of an interrupted attempt starts by erasing a sector that holds our header. A torn
     * erase raises bits in it, the header stops matching, and nothing can tell the result from
     * unknown data: the second stated window. The torn header write after it is the first. */
    make_interrupted(g_flash, 40000);
    (void)prov::run(ops_for(g_flash), cfg_for(g_flash));
    const int total_ops{g_flash.ops};
    for (int partial{0}; partial <= 1; ++partial) {
        for (int at{0}; at < total_ops; ++at) {
            const bool window{partial != 0 && (at == 0 || at == 1)};
            check_reset_then_reboot(start_interrupted, at, partial != 0, !window);
        }
    }
}

/* --- configuration --- */

ZTEST(tof_l7_blob_provisioner, test_a_region_too_small_touches_nothing)
{
    prov::config c{cfg_for(g_flash)};
    c.region_size = sizeof kRecord + 3;
    const prov::report r{prov::run(ops_for(g_flash), c)};
    zassert_equal(r.result, prov::outcome::bad_config);
    zassert_equal(g_flash.log_len, 0u);
    zassert_equal(g_flash.reads, 0);
}

ZTEST(tof_l7_blob_provisioner, test_classify_alone_never_erases_or_writes)
{
    make_uncommitted(g_flash);
    prov::flash_ops read_only{f_read, nullptr, nullptr, &g_flash};
    int rc{0};
    zassert_equal(prov::classify(read_only, cfg_for(g_flash), rc), prov::found::uncommitted_exact);
    zassert_equal(g_flash.log_len, 0u);
}
