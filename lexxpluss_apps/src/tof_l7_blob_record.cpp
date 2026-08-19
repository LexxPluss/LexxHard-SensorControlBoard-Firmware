/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_l7_blob_record.hpp"

#if defined(ENABLE_TOF_CHAIN)

#include <string.h>

#include <zephyr/sys/crc.h>
#include <tinycrypt/constants.h>
#include <tinycrypt/sha256.h>

namespace lexxhard::tof_l7_blob {

namespace {

constexpr size_t kMagicOffset{0};
constexpr size_t kFormatOffset{4};
constexpr size_t kHeaderSizeOffset{6};
constexpr size_t kPayloadLenOffset{8};
constexpr size_t kDigestOffset{12};
constexpr size_t kReservedOffset{44};
constexpr size_t kReservedLen{16};
constexpr size_t kCrcOffset{60};

/* The payload is hashed in pieces, because the whole point of keeping it out of the image is that
 * there is no 84 KiB of RAM to hash it in. 256 B is a compromise with nothing clever behind it: big
 * enough that the per-call overhead disappears, small enough to sit on a stack that also has to
 * hold a Zephyr thread's frames. */
constexpr size_t kChunk{256};

uint32_t le32(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

uint16_t le16(const uint8_t *p)
{
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

bool all_of(const uint8_t *p, size_t len, uint8_t value)
{
    for (size_t i{0}; i < len; ++i) {
        if (p[i] != value)
            return false;
    }
    return true;
}

/* Constant time is NOT claimed and is not needed. This digest answers "are the bytes intact and are
 * they the ones we were built against" -- an attacker who can write the storage partition can write
 * the application too, so there is no secret here to leak through timing. Saying so explicitly
 * because a digest comparison is exactly where somebody later assumes otherwise. */
bool same_digest(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, kDigestSize) == 0;
}

status parse_header(const reader &r, size_t region_size, uint8_t (&raw)[kHeaderSize],
                    header_info &info)
{
    if (region_size < kHeaderSize)
        return status::length_out_of_range;
    if (r.read == nullptr)
        return status::unreadable;
    if (r.read(r.ctx, 0, raw, kHeaderSize) != 0)
        return status::unreadable;

    /* Erased flash first, and as its own answer. An unprovisioned board is the normal state of a
     * board nobody has provisioned yet, and reporting it as a corrupt record would send somebody
     * looking for damage. */
    if (all_of(raw, kHeaderSize, 0xFF))
        return status::absent;
    if (le32(raw + kMagicOffset) != kMagic)
        return status::bad_magic;

    info.format_version = le16(raw + kFormatOffset);
    if (info.format_version != kFormatVersion)
        return status::unsupported_format;
    /* The size the writer used, checked against the size this reader implements. A later format that
     * grows the header must be refused by an older reader rather than parsed with the fields it
     * happens to recognise -- the alternative is reading a v2 field as a v1 one. */
    if (le16(raw + kHeaderSizeOffset) != kHeaderSize)
        return status::unsupported_format;

    if (crc32_ieee(raw, kCrcOffset) != le32(raw + kCrcOffset))
        return status::bad_header;
    /* Reserved means reserved: zero today, and a record that uses them is from a format this
     * firmware does not implement even if it kept the version number. Fail closed. */
    if (!all_of(raw + kReservedOffset, kReservedLen, 0x00))
        return status::unsupported_format;

    info.payload_len = le32(raw + kPayloadLenOffset);
    memcpy(info.payload_digest, raw + kDigestOffset, kDigestSize);
    info.parsed = true;

    if (info.payload_len == 0 || info.payload_len > region_size - kHeaderSize)
        return status::length_out_of_range;

    return status::ok;
}

}  // namespace

const char *status_name(status s)
{
    switch (s) {
    case status::ok:                  return "ok";
    case status::unreadable:          return "unreadable";
    case status::absent:              return "absent";
    case status::bad_magic:           return "bad_magic";
    case status::unsupported_format:  return "unsupported_format";
    case status::bad_header:          return "bad_header";
    case status::length_out_of_range: return "length_out_of_range";
    case status::length_mismatch:     return "length_mismatch";
    case status::version_mismatch:    return "version_mismatch";
    case status::digest_mismatch:     return "digest_mismatch";
    }
    return "?";
}

status read_header(const reader &r, size_t region_size, header_info &out)
{
    uint8_t raw[kHeaderSize];

    out = header_info{};
    return parse_header(r, region_size, raw, out);
}

status verify(const reader &r, size_t region_size, const expectation &want, blob_view &out,
              const uint8_t *mapped_base)
{
    uint8_t raw[kHeaderSize];
    header_info info{};

    if (want.payload_len == 0)
        return status::length_mismatch;   // a caller that does not know what it wants gets nothing

    if (const status st{parse_header(r, region_size, raw, info)}; st != status::ok)
        return st;

    if (info.payload_len != want.payload_len)
        return status::length_mismatch;

    /* IDENTITY, before spending 84 KiB of hashing on it. The stored record's own digest against the
     * one this firmware was built against: equal means the right blob is here, different means the
     * flash is fine and somebody provisioned another release. Checking it first also means the
     * expensive integrity pass only ever runs on a blob we actually want. */
    if (!same_digest(info.payload_digest, want.payload_digest))
        return status::version_mismatch;

    /* INTEGRITY. Streamed in chunks: there is no 84 KiB buffer to do otherwise, which is the whole
     * reason this record exists. */
    struct tc_sha256_state_struct sha{};

    if (tc_sha256_init(&sha) != TC_CRYPTO_SUCCESS)
        return status::unreadable;

    uint8_t chunk[kChunk];
    size_t done{0};

    while (done < info.payload_len) {
        const size_t take{(info.payload_len - done) < kChunk ? (info.payload_len - done) : kChunk};

        if (r.read(r.ctx, kHeaderSize + done, chunk, take) != 0)
            return status::unreadable;
        if (tc_sha256_update(&sha, chunk, take) != TC_CRYPTO_SUCCESS)
            return status::unreadable;
        done += take;
    }

    uint8_t computed[kDigestSize];

    if (tc_sha256_final(computed, &sha) != TC_CRYPTO_SUCCESS)
        return status::unreadable;
    /* Against the HEADER's digest, which the identity check above has already proved equal to the
     * expectation. Comparing against the expectation here instead would give the same answer today
     * and would stop being a check of "the stored bytes match their own record" the moment the two
     * comparisons drift apart. */
    if (!same_digest(computed, info.payload_digest))
        return status::digest_mismatch;

    /* Only now, and only if the region is mapped. A caller with no mapping still gets `ok` and can
     * stream the payload itself; what it must not get is a pointer that was never verified. */
    if (mapped_base != nullptr) {
        out.data_ = mapped_base + kHeaderSize;
        out.size_ = info.payload_len;
    }
    return status::ok;
}

}  // namespace lexxhard::tof_l7_blob

#endif  // ENABLE_TOF_CHAIN
