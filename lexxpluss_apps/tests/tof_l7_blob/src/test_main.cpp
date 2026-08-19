/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host tests for the stored VL53L7CX blob record.
 *
 * The bytes under test are the GENERATOR's, from src/golden_record.h. That is the point of having a
 * golden record at all: the packer script and this reader have to agree about a layout that will be
 * written once at manufacture and read at every boot, and the only way to keep two implementations
 * of a layout honest is to make one of them consume the other's output in CI.
 *
 * Every refusal has a case, and every case reaches it by damaging the golden record in exactly one
 * way. A status enumeration whose values are unreachable is a list of things nobody checked -- the
 * cliff proof had one such value hiding behind an earlier check, found only by a mechanical sweep.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>
#include <zephyr/ztest.h>

#include "golden_record.h"
#include "tof_l7_blob_record.hpp"
#include "tof_l7_runtime.hpp"

namespace blob = lexxhard::tof_l7_blob;
namespace runtime = lexxhard::tof_l7_runtime;

namespace {

/* The record, in RAM, so a case can damage a byte. On a board this is memory-mapped flash and the
 * reader is flash_area_read; the logic under test cannot tell the difference, which is why it is
 * injected. */
uint8_t region[blob::golden::kRecordLen + 64];
size_t region_size;
int forced_read_rc;
size_t fail_at_offset;
int reads_seen;
blob::expectation accepted[2];
bool map_for_runtime;

int read_region(void *, size_t offset, void *dst, size_t len)
{
    ++reads_seen;
    if (forced_read_rc != 0)
        return forced_read_rc;
    if (offset >= fail_at_offset)
        return -EIO;
    if (offset + len > sizeof region)
        return -EIO;
    memcpy(dst, region + offset, len);
    return 0;
}

blob::reader make_reader()
{
    blob::reader r{};

    r.read = read_region;
    r.ctx = nullptr;
    return r;
}

blob::expectation make_golden_expectation()
{
    blob::expectation e{};

    e.payload_len = blob::golden::kPayloadLen;
    memcpy(e.payload_digest, blob::golden::kPayloadDigest, sizeof e.payload_digest);
    return e;
}

blob::accept_list golden_accept_list()
{
    return {accepted, 1};
}

blob::report verify_for_runtime(const blob::accept_list &list, blob::blob_view &out)
{
    blob::report rep{};

    rep.region_size = region_size;
    (void)blob::read_header(make_reader(), region_size, rep.stored);
    rep.st = blob::verify(make_reader(), region_size, list, out,
                          map_for_runtime ? region : nullptr);
    return rep;
}

void before(void *)
{
    memset(region, 0, sizeof region);
    memcpy(region, blob::golden::kRecord, blob::golden::kRecordLen);
    region_size = blob::golden::kRecordLen;
    forced_read_rc = 0;
    fail_at_offset = SIZE_MAX;
    reads_seen = 0;
    accepted[0] = make_golden_expectation();
    accepted[1] = blob::expectation{};
    map_for_runtime = true;
    runtime::reset_for_test();
}

/* Recomputes the header CRC after a case has edited a header field, so that the case tests the field
 * it meant to and not the CRC. Every case that wants a bad CRC asks for one explicitly. */
void refresh_header_crc()
{
    const uint32_t crc{crc32_ieee(region, 60)};

    region[60] = static_cast<uint8_t>(crc & 0xFF);
    region[61] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    region[62] = static_cast<uint8_t>((crc >> 16) & 0xFF);
    region[63] = static_cast<uint8_t>((crc >> 24) & 0xFF);
}

}  // namespace

ZTEST_SUITE(tof_l7_blob, NULL, NULL, before, NULL, NULL);

ZTEST(tof_l7_blob, test_the_generators_record_verifies)
{
    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::ok, "the reader rejected the generator's own record");
    zassert_true(view.valid());
    zassert_equal(view.size(), blob::golden::kPayloadLen);
    /* The payload starts after the 64-byte header, and the view points AT the payload -- an
     * off-by-one here would push the header into a sensor as though it were firmware. */
    zassert_equal(view.data(), region + 64);
    zassert_equal(view.data()[0], blob::golden::kRecord[64]);
}

ZTEST(tof_l7_blob, test_runtime_publishes_only_the_pointer_verified_at_boot)
{
    zassert_equal(runtime::bootstrap_for_test(verify_for_runtime, golden_accept_list()), 0);

    const runtime::snapshot state{runtime::current()};
    zassert_equal(state.current_stage, runtime::stage::available);
    zassert_true(state.firmware_available);
    zassert_equal(state.firmware_size, blob::golden::kPayloadLen);
    zassert_equal(runtime::firmware_data(), region + blob::kHeaderSize);
    zassert_equal(runtime::firmware_size(), blob::golden::kPayloadLen);
}

ZTEST(tof_l7_blob, test_runtime_refusal_never_publishes_a_firmware_pointer)
{
    region[blob::kHeaderSize + 10] ^= 0x01;

    zassert_not_equal(runtime::bootstrap_for_test(verify_for_runtime, golden_accept_list()), 0);

    const runtime::snapshot state{runtime::current()};
    zassert_equal(state.current_stage, runtime::stage::refused);
    zassert_equal(state.verification.st, blob::status::digest_mismatch);
    zassert_false(state.firmware_available);
    zassert_is_null(runtime::firmware_data());
    zassert_equal(runtime::firmware_size(), 0);
}

ZTEST(tof_l7_blob, test_runtime_refuses_an_ok_verdict_without_a_mapped_view)
{
    map_for_runtime = false;

    zassert_not_equal(runtime::bootstrap_for_test(verify_for_runtime, golden_accept_list()), 0);
    zassert_equal(runtime::current().verification.st, blob::status::mapping_mismatch);
    zassert_is_null(runtime::firmware_data());
}

ZTEST(tof_l7_blob, test_runtime_bootstrap_is_single_shot)
{
    zassert_equal(runtime::bootstrap_for_test(verify_for_runtime, golden_accept_list()), 0);
    const uint8_t *const first{runtime::firmware_data()};

    region[blob::kHeaderSize + 10] ^= 0x01;
    zassert_equal(runtime::bootstrap_for_test(verify_for_runtime, golden_accept_list()), -EALREADY);
    zassert_equal(runtime::current().current_stage, runtime::stage::available);
    zassert_equal(runtime::firmware_data(), first);
}

ZTEST(tof_l7_blob, test_the_last_written_commit_marker_is_required)
{
    memset(region + blob::kHeaderSize + blob::golden::kPayloadLen, 0xFF,
           blob::kCommitMarkerSize);

    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::uncommitted);
    zassert_false(view.valid());
}

ZTEST(tof_l7_blob, test_verification_authorises_the_same_mapping_it_checked)
{
    uint8_t wrong_mapping[sizeof region];
    memcpy(wrong_mapping, region, sizeof region);
    wrong_mapping[blob::kHeaderSize + 10] ^= 0x01;

    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view,
                               wrong_mapping),
                  blob::status::mapping_mismatch);
    zassert_false(view.valid());
}

ZTEST(tof_l7_blob, test_a_refusal_revokes_a_view_from_an_earlier_success)
{
    blob::blob_view view{};
    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::ok);
    zassert_true(view.valid());

    region[blob::kHeaderSize + 7] ^= 0x01;
    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::digest_mismatch);
    zassert_false(view.valid(), "the old authorised pointer survived a failed verification");
}

ZTEST(tof_l7_blob, test_an_accept_list_can_authorise_a_rollback_compatible_blob)
{
    accepted[1] = accepted[0];
    accepted[0].payload_digest[0] ^= 0xFF;
    const blob::accept_list list{accepted, 2};
    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, list, view, region), blob::status::ok);
    zassert_true(view.valid());
}

ZTEST(tof_l7_blob, test_an_erased_partition_is_absent_not_corrupt)
{
    /* The normal state of a board nobody has provisioned. Reporting it as damage would send someone
     * looking for a broken flash controller. */
    memset(region, 0xFF, sizeof region);

    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::absent);
    zassert_false(view.valid());
}

ZTEST(tof_l7_blob, test_a_region_with_other_contents_is_bad_magic)
{
    memset(region, 0x5A, 64);

    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::bad_magic);
    zassert_false(view.valid());
}

ZTEST(tof_l7_blob, test_a_newer_format_is_refused_rather_than_parsed)
{
    /* Fail closed. The alternative is reading a field a later format moved, which produces a
     * plausible length and a plausible digest for a layout this firmware does not implement. */
    region[4] = blob::kFormatVersion + 1;
    refresh_header_crc();

    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::unsupported_format);

    before(nullptr);
    region[6] = 128;   // a header this reader does not know the shape of
    refresh_header_crc();
    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::unsupported_format);

    before(nullptr);
    region[44] = 1;    // a reserved byte in use
    refresh_header_crc();
    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::unsupported_format);
}

ZTEST(tof_l7_blob, test_a_torn_header_is_caught_by_its_crc)
{
    region[8] ^= 0x01;   // a payload length that nobody re-CRC'd

    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::bad_header);
}

ZTEST(tof_l7_blob, test_a_payload_that_cannot_fit_its_region_is_refused_before_it_is_read)
{
    /* Bounds before contents. A length that overruns the region would otherwise be hashed by
     * reading past the end of it. */
    region[8] = 0x00;
    region[9] = 0x00;
    region[10] = 0x01;   // 64 KiB, in a region of 1088 bytes
    region[11] = 0x00;
    refresh_header_crc();

    blob::blob_view view{};
    const int reads_before{reads_seen};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::length_out_of_range);
    zassert_equal(reads_seen, reads_before + 1, "the payload was read despite an impossible length");
}

ZTEST(tof_l7_blob, test_a_zero_length_payload_is_refused)
{
    memset(region + 8, 0, 4);
    refresh_header_crc();

    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::length_out_of_range);
}

ZTEST(tof_l7_blob, test_a_different_length_than_expected_is_its_own_refusal)
{
    /* Distinct from version_mismatch on purpose: "the record is 1023 bytes and this firmware wants
     * 1024" is a sentence somebody can act on, and it is the likely shape of a truncated write. */
    blob::expectation want{make_golden_expectation()};

    want.payload_len -= 1;

    blob::blob_view view{};

    const blob::accept_list list{&want, 1};
    zassert_equal(blob::verify(make_reader(), region_size, list, view, region),
                  blob::status::length_mismatch);
}

ZTEST(tof_l7_blob, test_an_intact_blob_from_another_release_is_a_version_mismatch)
{
    /* THE distinction this module exists for. The stored record is perfectly intact -- it hashes to
     * its own digest -- and it is not the blob this firmware was built against. A digest-only check
     * would pass it, and the ULD indexes fixed offsets inside the blob, so the failure would appear
     * as a sensor that does not range rather than as a provisioning mistake. */
    blob::expectation want{make_golden_expectation()};

    want.payload_digest[0] ^= 0xFF;

    blob::blob_view view{};

    const blob::accept_list list{&want, 1};
    zassert_equal(blob::verify(make_reader(), region_size, list, view, region),
                  blob::status::version_mismatch);
    zassert_false(view.valid());
}

ZTEST(tof_l7_blob, test_a_damaged_payload_is_a_digest_mismatch)
{
    region[64 + 500] ^= 0x01;   // one bit, in the middle of the payload

    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::digest_mismatch);
    zassert_false(view.valid());
}

ZTEST(tof_l7_blob, test_damage_wins_over_the_wrong_version_diagnosis)
{
    region[blob::kHeaderSize + 500] ^= 0x01;
    accepted[0].payload_digest[0] ^= 0xFF;

    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::digest_mismatch);
}

ZTEST(tof_l7_blob, test_the_last_byte_of_the_payload_is_hashed)
{
    /* The off-by-one that a chunked hash invites, and the reason the golden payload has no runs in
     * it: over a constant payload, a reader that dropped the final byte would produce the same
     * digest and this case would pass. */
    region[64 + blob::golden::kPayloadLen - 1] ^= 0x01;

    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::digest_mismatch);
}

ZTEST(tof_l7_blob, test_a_read_failure_is_never_a_verified_blob)
{
    forced_read_rc = -EIO;

    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::unreadable);
    zassert_false(view.valid());

    /* And a failure that only appears part-way through the payload is still a refusal, not a partial
     * verification: the digest would be over the bytes that did arrive. */
    before(nullptr);
    forced_read_rc = 0;
    fail_at_offset = blob::kHeaderSize + 512;
    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, region),
                  blob::status::unreadable);
}

ZTEST(tof_l7_blob, test_an_unmapped_region_verifies_but_hands_out_no_pointer)
{
    /* A caller with no memory-mapped view still gets a verdict and can stream the payload itself.
     * What it must not get is a pointer nobody verified. */
    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), region_size, golden_accept_list(), view, nullptr),
                  blob::status::ok);
    zassert_false(view.valid());
    zassert_is_null(view.data());
}

ZTEST(tof_l7_blob, test_a_caller_that_expects_nothing_gets_nothing)
{
    /* An empty expectation cannot be satisfied, and the refusal is not "ok". A firmware that has not
     * been told which blob it needs has no business pushing one. */
    blob::blob_view view{};
    const blob::accept_list nothing{};

    zassert_equal(blob::verify(make_reader(), region_size, nothing, view, region),
                  blob::status::no_expectation);
    zassert_false(view.valid());
}

ZTEST(tof_l7_blob, test_the_header_can_be_read_for_a_diagnostic)
{
    /* "expected 86,016 bytes, found 84,992" is actionable; "length_mismatch" alone is not. */
    blob::header_info info{};

    zassert_equal(blob::read_header(make_reader(), region_size, info), blob::status::ok);
    zassert_true(info.parsed);
    zassert_equal(info.format_version, blob::kFormatVersion);
    zassert_equal(info.payload_len, blob::golden::kPayloadLen);
    zassert_mem_equal(info.payload_digest, blob::golden::kPayloadDigest, 32);
}

ZTEST(tof_l7_blob, test_every_status_has_a_name)
{
    /* A refusal that prints as "?" is a refusal somebody will guess at. */
    const blob::status all[]{
        blob::status::ok,                  blob::status::unreadable,
        blob::status::absent,              blob::status::uncommitted,
        blob::status::bad_magic,           blob::status::unsupported_format,
        blob::status::bad_header,          blob::status::length_out_of_range,
        blob::status::length_mismatch,     blob::status::version_mismatch,
        blob::status::digest_mismatch,     blob::status::mapping_mismatch,
        blob::status::no_expectation,
    };

    for (const blob::status s : all) {
        zassert_true(strcmp(blob::status_name(s), "?") != 0,
                     "status %d has no name", static_cast<int>(s));
    }
}
