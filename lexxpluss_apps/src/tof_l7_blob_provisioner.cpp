/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_l7_blob_provisioner.hpp"

#if defined(ENABLE_L7_BLOB_PROVISIONER)

#include <errno.h>
#include <string.h>

#include <tinycrypt/constants.h>
#include <tinycrypt/sha256.h>

namespace lexxhard::tof_l7_blob_provisioner {

namespace {

/* Every transfer goes through a RAM buffer of this size. Writes in particular: the source record
 * lives in the same flash bank as the partition being programmed, and copying first keeps "read the
 * source" and "program the target" from ever being the same bus operation. */
constexpr size_t kChunk{256};
constexpr uint8_t kMarker[blob::kCommitMarkerSize]{'L', '7', 'O', 'K'};
constexpr size_t kDigestOffset{12};

/* The byte this image wants at `offset` once the record is committed: the record, then the marker,
 * then erased flash for the rest of the partition. Classification and the embedded self-check are
 * both defined against this one function, so they cannot disagree about what "this record" is. */
uint8_t target_at(const config &c, size_t offset)
{
    if (offset < c.record_len)
        return c.record[offset];
    if (offset < c.record_len + blob::kCommitMarkerSize)
        return kMarker[offset - c.record_len];
    return 0xff;
}

bool config_ok(const flash_ops &f, const config &c)
{
    return f.read != nullptr && f.erase != nullptr && f.write != nullptr && c.record != nullptr &&
           c.record_len > blob::kHeaderSize &&
           c.region_size >= c.record_len + blob::kCommitMarkerSize;
}

/* The production reader, pointed at the injected flash. */
struct flash_ctx {
    const flash_ops *f;
};

int read_flash(void *ctx, size_t offset, void *dst, size_t len)
{
    const auto *fc{static_cast<const flash_ctx *>(ctx)};
    return fc->f->read(fc->f->ctx, offset, dst, len);
}

/* The production reader, pointed at the record as it WOULD look committed. */
struct virtual_ctx {
    const config *c;
};

int read_virtual(void *ctx, size_t offset, void *dst, size_t len)
{
    const auto *vc{static_cast<const virtual_ctx *>(ctx)};
    if (offset > vc->c->region_size || len > vc->c->region_size - offset)
        return -EINVAL;
    auto *out{static_cast<uint8_t *>(dst)};
    for (size_t i{0}; i < len; ++i)
        out[i] = target_at(*vc->c, offset + i);
    return 0;
}

/* The last step of every successful path, and the only thing that can report success: the
 * production reader, with the production accept-list, over what is actually in flash now. */
report finish(const flash_ops &f, const config &c, report r, outcome on_success)
{
    flash_ctx fc{&f};
    blob::reader rd{};
    rd.read = read_flash;
    rd.ctx = &fc;
    blob::blob_view view{};
    r.final_status = blob::verify(rd, c.region_size, c.accepted, view, c.mapped_base);
    r.result = r.final_status == blob::status::ok ? on_success : outcome::final_verify_failed;
    return r;
}

/* Everything after the body is in flash: prove it, then commit it. Reached both after this run
 * wrote the body and when the scan found this exact body already written but not committed. */
report commit(const flash_ops &f, const config &c, report r)
{
    uint8_t buf[kChunk];

    /* 1. Every byte back, compared with the record, and the payload hashed on the way. */
    ++r.readbacks;
    tc_sha256_state_struct sha{};
    if (tc_sha256_init(&sha) != TC_CRYPTO_SUCCESS) {
        r.result = outcome::digest_mismatch;
        return r;
    }
    for (size_t off{0}; off < c.record_len; off += kChunk) {
        const size_t n{c.record_len - off < kChunk ? c.record_len - off : kChunk};
        if (const int rc{f.read(f.ctx, off, buf, n)}; rc != 0) {
            r.result = outcome::readback_failed;
            r.last_errno = rc;
            r.failed_offset = off;
            return r;
        }
        for (size_t i{0}; i < n; ++i) {
            if (buf[i] != c.record[off + i]) {
                r.result = outcome::readback_failed;
                r.failed_offset = off + i;
                return r;
            }
        }
        /* The payload is everything after the header. The header's own bytes are covered by the
         * byte comparison above and by the header CRC the production reader checks at the end. */
        const size_t hashed_from{off < blob::kHeaderSize ? blob::kHeaderSize - off : 0};
        if (hashed_from < n &&
            tc_sha256_update(&sha, buf + hashed_from, n - hashed_from) != TC_CRYPTO_SUCCESS) {
            r.result = outcome::digest_mismatch;
            return r;
        }
    }
    uint8_t computed[blob::kDigestSize];
    if (tc_sha256_final(computed, &sha) != TC_CRYPTO_SUCCESS ||
        memcmp(computed, c.record + kDigestOffset, blob::kDigestSize) != 0) {
        r.result = outcome::digest_mismatch;
        return r;
    }

    /* 2. The marker bytes must still be erased: a write that is not preceded by an erase can only
     * clear bits, so anything already there would corrupt the marker rather than be replaced. */
    uint8_t mark[blob::kCommitMarkerSize];
    if (const int rc{f.read(f.ctx, c.record_len, mark, sizeof mark)}; rc != 0) {
        r.result = outcome::readback_failed;
        r.last_errno = rc;
        r.failed_offset = c.record_len;
        return r;
    }
    for (const uint8_t b : mark) {
        if (b != 0xff) {
            r.result = outcome::marker_not_blank;
            r.failed_offset = c.record_len;
            return r;
        }
    }

    /* 3. Commit. Four bytes, written, never erased. */
    memcpy(mark, kMarker, sizeof mark);
    ++r.write_attempts;
    r.flash_operation_attempted = true;
    if (const int rc{f.write(f.ctx, c.record_len, mark, sizeof mark)}; rc != 0) {
        r.result = outcome::commit_failed;
        r.last_errno = rc;
        r.failed_offset = c.record_len;
        return r;
    }
    if (const int rc{f.read(f.ctx, c.record_len, mark, sizeof mark)};
        rc != 0 || memcmp(mark, kMarker, sizeof mark) != 0) {
        r.result = outcome::commit_failed;
        r.last_errno = rc;
        r.failed_offset = c.record_len;
        return r;
    }

    /* 4. And only the production reader can say it worked. */
    return finish(f, c, r, outcome::provisioned_and_verified);
}

}  // namespace

found classify(const flash_ops &f, const config &c, int &read_errno)
{
    read_errno = 0;
    if (f.read == nullptr || c.record == nullptr ||
        c.region_size < c.record_len + blob::kCommitMarkerSize)
        return found::unreadable;

    bool exact{true};        // this record and its marker, the rest erased
    bool erased{true};       // all 0xff
    bool uncommitted{true};  // this record, marker and the rest erased
    bool header{true};       // the 64-byte header is exactly this record's
    bool consistent{true};   // after the header, no bit is 0 where the committed image has a 1

    uint8_t buf[kChunk];
    for (size_t off{0}; off < c.region_size; off += kChunk) {
        const size_t n{c.region_size - off < kChunk ? c.region_size - off : kChunk};
        if (const int rc{f.read(f.ctx, off, buf, n)}; rc != 0) {
            read_errno = rc;
            return found::unreadable;
        }
        for (size_t i{0}; i < n; ++i) {
            const size_t at{off + i};
            const uint8_t a{buf[i]};
            const uint8_t t{target_at(c, at)};
            exact = exact && a == t;
            erased = erased && a == 0xff;
            uncommitted = uncommitted && (at < c.record_len ? a == t : a == 0xff);
            if (at < blob::kHeaderSize)
                header = header && a == t;
            else
                consistent = consistent && (a & t) == t;
        }
    }

    /* The specific answer wins where classes overlap. The bit relation is consulted only behind an
     * exact header: without it, a mostly-erased sector of unrelated data is indistinguishable from
     * a torn attempt, and erasing it is exactly what this module must never do. */
    if (exact)
        return found::committed_exact;
    if (erased)
        return found::blank;
    if (uncommitted)
        return found::uncommitted_exact;
    if (header && consistent)
        return found::interrupted_exact;
    return found::foreign;
}

blob::status check_embedded(const config &c)
{
    virtual_ctx vc{&c};
    blob::reader rd{};
    rd.read = read_virtual;
    rd.ctx = &vc;
    blob::blob_view view{};
    return blob::verify(rd, c.region_size, c.accepted, view, nullptr);
}

report run(const flash_ops &f, const config &c)
{
    report r{};

    if (!config_ok(f, c)) {
        r.result = outcome::bad_config;
        return r;
    }
    /* Before a single flash operation: would the production reader accept this record once it is
     * committed? This is what binds the embedded payload to the L7 accept-list at run time. */
    if (check_embedded(c) != blob::status::ok) {
        r.result = outcome::bad_embedded_record;
        return r;
    }

    int rc{0};
    r.initial = classify(f, c, rc);
    switch (r.initial) {
    case found::not_scanned:
    case found::unreadable:
        r.result = outcome::scan_failed;
        r.last_errno = rc;
        return r;
    case found::foreign:
        r.result = outcome::foreign_or_unknown_data;
        return r;
    case found::committed_exact:
        return finish(f, c, r, outcome::already_provisioned);
    case found::uncommitted_exact:
        return commit(f, c, r);
    case found::blank:
    case found::interrupted_exact:
        break;
    }

    /* Erase exactly the partition, and prove it. */
    ++r.erase_attempts;
    r.flash_operation_attempted = true;
    if ((rc = f.erase(f.ctx, 0, c.region_size)) != 0) {
        r.result = outcome::erase_failed;
        r.last_errno = rc;
        return r;
    }
    uint8_t buf[kChunk];
    for (size_t off{0}; off < c.region_size; off += kChunk) {
        const size_t n{c.region_size - off < kChunk ? c.region_size - off : kChunk};
        if ((rc = f.read(f.ctx, off, buf, n)) != 0) {
            r.result = outcome::erase_verify_failed;
            r.last_errno = rc;
            r.failed_offset = off;
            return r;
        }
        for (size_t i{0}; i < n; ++i) {
            if (buf[i] != 0xff) {
                r.result = outcome::erase_verify_failed;
                r.failed_offset = off + i;
                return r;
            }
        }
    }

    /* The body, through a RAM copy. The marker is NOT part of it.
     *
     * The header goes first and ALONE. It is what later identifies a torn attempt as this record's,
     * so the window in which a reset leaves an unidentifiable sector is this one 64-byte write --
     * not the first 256-byte chunk, which would also carry payload. After it, the payload in
     * chunks, each chunk inside the payload. */
    for (size_t off{0}; off < c.record_len;) {
        const size_t limit{off < blob::kHeaderSize ? blob::kHeaderSize - off : kChunk};
        const size_t n{c.record_len - off < limit ? c.record_len - off : limit};
        memcpy(buf, c.record + off, n);
        ++r.write_attempts;
        if ((rc = f.write(f.ctx, off, buf, n)) != 0) {
            r.result = outcome::write_failed;
            r.last_errno = rc;
            r.failed_offset = off;
            return r;
        }
        off += n;
    }

    return commit(f, c, r);
}

const char *found_name(found f)
{
    switch (f) {
    case found::not_scanned: return "not_scanned";
    case found::blank: return "blank";
    case found::committed_exact: return "committed_exact";
    case found::uncommitted_exact: return "uncommitted_exact";
    case found::interrupted_exact: return "interrupted_exact";
    case found::foreign: return "foreign";
    case found::unreadable: return "unreadable";
    }
    return "?";
}

const char *outcome_name(outcome o)
{
    switch (o) {
    case outcome::not_run: return "not_run";
    case outcome::already_provisioned: return "already_provisioned";
    case outcome::provisioned_and_verified: return "provisioned_and_verified";
    case outcome::foreign_or_unknown_data: return "foreign_or_unknown_data";
    case outcome::bad_embedded_record: return "bad_embedded_record";
    case outcome::bad_config: return "bad_config";
    case outcome::scan_failed: return "scan_failed";
    case outcome::erase_failed: return "erase_failed";
    case outcome::erase_verify_failed: return "erase_verify_failed";
    case outcome::write_failed: return "write_failed";
    case outcome::readback_failed: return "readback_failed";
    case outcome::digest_mismatch: return "digest_mismatch";
    case outcome::marker_not_blank: return "marker_not_blank";
    case outcome::commit_failed: return "commit_failed";
    case outcome::final_verify_failed: return "final_verify_failed";
    }
    return "?";
}

}  // namespace lexxhard::tof_l7_blob_provisioner

#endif  // ENABLE_L7_BLOB_PROVISIONER
