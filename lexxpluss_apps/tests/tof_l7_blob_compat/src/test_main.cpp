/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * THE COMPATIBILITY QUESTION, ANSWERED AGAINST BYTES RATHER THAN AGAINST CONSTANTS.
 *
 * dasher1's storage_partition was provisioned on 2026-10-08 by an image built from
 * feature/AMRSW-2322-l7-blob-provisioner. The next thing to read it will be a bench or product image
 * built from THIS branch, whose tof_l7_blob_record.cpp and accept-list header are not byte-identical
 * to that branch's. The format constants on both sides agree -- 64-byte header, 4-byte marker, magic
 * L7B1, marker L7OK, format version 2 -- and the provisioner's last step did pass its own copy of the
 * production reader. Neither of those is this branch's reader reading those bytes, which is what
 * actually has to work.
 *
 * So this suite takes the exact 131,072 bytes that should now be in the partition -- the generator's
 * record, its commit marker, and 0xff for the rest -- and runs this branch's reader over them against
 * this branch's signed accept-list.
 *
 * WHAT IT DOES NOT CLAIM. The image is reconstructed from the same generator inputs, not read back
 * from the board; the board's own `pkg_storage sha` next session is what confirms the flash matches
 * it, and the expected digest is recorded with the runbook. And a record a reader accepts is still
 * only a record: whether the sensor accepts the download is a different question entirely.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "tof_l7_blob_record.hpp"
#include "vl53l7cx_blob_expectation.hpp"

namespace blob = lexxhard::tof_l7_blob;

namespace {

const uint8_t kRegion[]{
#include "l7_region.inc"
};
static_assert(sizeof(kRegion) == 131072, "the region image is not 128 KiB");

int forced_rc;

int read_region(void *, size_t offset, void *dst, size_t len)
{
    if (forced_rc != 0)
        return forced_rc;
    if (offset + len > sizeof kRegion)
        return -EIO;
    memcpy(dst, kRegion + offset, len);
    return 0;
}

blob::reader make_reader()
{
    blob::reader r{};

    r.read = read_region;
    r.ctx = nullptr;
    return r;
}

/* A mutable copy, for the negative control. The real path reads memory-mapped flash, and the reader
 * cannot tell the difference -- which is the point of injecting it. */
uint8_t scratch[sizeof kRegion];

int read_scratch(void *, size_t offset, void *dst, size_t len)
{
    if (offset + len > sizeof scratch)
        return -EIO;
    memcpy(dst, scratch + offset, len);
    return 0;
}

blob::reader scratch_reader()
{
    blob::reader r{};

    r.read = read_scratch;
    r.ctx = nullptr;
    return r;
}

void before(void *)
{
    forced_rc = 0;
    memcpy(scratch, kRegion, sizeof scratch);
}

}  // namespace

ZTEST_SUITE(tof_l7_blob_compat, NULL, NULL, before, NULL, NULL);

/* THE ONE THAT MATTERS: this branch's reader, this branch's signed accept-list, those bytes. */
ZTEST(tof_l7_blob_compat, test_this_branch_accepts_the_provisioned_record)
{
    blob::blob_view view{};

    zassert_equal(blob::verify(make_reader(), sizeof kRegion, blob::kAcceptedPayloadList, view,
                               kRegion),
                  blob::status::ok,
                  "this branch's reader refused the record that is on dasher1");

    /* And the view it hands out is the payload itself, at the mapped offset the sensor layer will
     * pass to the ULD: 86,016 bytes starting one header in. */
    zassert_equal(view.size(), 86016U);
    zassert_equal_ptr(view.data(), kRegion + blob::kHeaderSize);
}

/* The header reads back as the accept-list entry, so a diagnostic can say what IS stored. */
ZTEST(tof_l7_blob_compat, test_the_header_matches_the_accept_list_entry)
{
    blob::header_info info{};

    zassert_equal(blob::read_header(make_reader(), sizeof kRegion, info), blob::status::ok);
    zassert_equal(info.payload_len, 86016U);
    zassert_equal(info.format_version, blob::kFormatVersion);

    /* The digest in the header is the one the signed accept-list names, which is what makes this
     * record a member of the accepted set rather than merely well-formed. */
    bool found{false};
    for (size_t i{0}; i < blob::kAcceptedPayloadList.count; ++i) {
        const blob::expectation &e{blob::kAcceptedPayloadList.entries[i]};
        if (e.payload_len == info.payload_len &&
            memcmp(e.payload_digest, info.payload_digest, sizeof e.payload_digest) == 0)
            found = true;
    }
    zassert_true(found, "the stored digest is not in this branch's accept-list");
}

/* NEGATIVE CONTROL. Without it, "ok" could mean the reader is not checking: one flipped payload byte
 * must be refused, and refused as a digest mismatch rather than as a shape problem. */
ZTEST(tof_l7_blob_compat, test_one_flipped_payload_byte_is_refused)
{
    blob::blob_view view{};

    scratch[blob::kHeaderSize + 40000] ^= 0x01U;
    zassert_equal(blob::verify(scratch_reader(), sizeof scratch, blob::kAcceptedPayloadList, view,
                               scratch),
                  blob::status::digest_mismatch);
    zassert_equal(view.size(), 0U, "a refusal must not hand out a view");
}

/* And the marker is load-bearing: cleared to 0xff -- the state an interrupted provisioning leaves --
 * the record must not be treated as committed. */
ZTEST(tof_l7_blob_compat, test_an_uncommitted_record_is_refused)
{
    blob::blob_view view{};

    memset(scratch + 86080, 0xff, blob::kCommitMarkerSize);
    zassert_not_equal(blob::verify(scratch_reader(), sizeof scratch, blob::kAcceptedPayloadList,
                                   view, scratch),
                      blob::status::ok,
                      "a record whose marker is still erased is not committed");
    zassert_equal(view.size(), 0U);
}
