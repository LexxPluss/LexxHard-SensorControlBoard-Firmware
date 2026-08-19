/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The stored VL53L7CX device-firmware blob, and what has to be true before a byte of it is pushed
 * to a sensor.
 *
 * WHY THE BLOB IS NOT IN THE IMAGE. Measured, not estimated: VL53L7CX_FIRMWARE[] is 86,016 B,
 * plus 972 B of default configuration and 776 B of xtalk table. The B6 image already used 239,516 B
 * of the 261,712-byte MCUboot-aware ceiling before this L7 work, so embedding the payload alone
 * would exceed it by at least 63,820 B. With the external record gate already reachable, forcing
 * init/start/ready/fetch/stop plus the port and retained configuration/xtalk arrays into B6 adds
 * another measured 7,332 B. The constant blob is therefore the first hard problem, and
 * storage_partition (131,072 B at offset 0x20000) is the one region no application code referenced.
 *
 * WHAT THIS MODULE IS FOR. Flash is memory-mapped, so once a record has been verified the payload
 * can be handed to the I2C port as a pointer and streamed straight out -- no RAM copy of 84 KiB.
 * That is only safe if nothing can obtain that pointer without the verification, which is why a
 * blob_view has no public constructor that fills it in: verify() is the only producer.
 *
 * TWO DIFFERENT QUESTIONS, DELIBERATELY SEPARATE:
 *
 *   integrity -- do the stored bytes still hash to what the record says they do? A failure here
 *                means damaged flash.
 *   identity  -- is this the blob THIS firmware was built against? A failure here means the flash
 *                is fine and the wrong thing was provisioned.
 *
 * A hash-only check answers the first and reads as though it answered both. The ULD and its device
 * firmware are a matched pair -- the API indexes fixed offsets inside the blob (0x8000, 0x10000) --
 * so pushing an intact blob from a different ULD release is exactly the failure a digest check
 * looks like it prevents and does not. The caller therefore supplies the digest it expects, and the
 * two refusals are separate values.
 *
 * THE RECORD, normatively specified in docs/tof_l7_blob_record.md. Little-endian, 64-byte header:
 *
 *   0   4   magic 'L','7','B','1'
 *   4   2   format_version
 *   6   2   header_size (64; a later format may grow it, and an older reader must refuse)
 *   8   4   payload_len
 *   12  32  payload_digest, SHA-256 of the payload alone
 *   44  16  reserved, all zero
 *   60  4   header_crc32, CRC-32/IEEE over bytes 0..59
 *   64..    payload
 *   end 4   commit marker 'L','7','O','K', written last
 *
 * The header carries a CRC and the payload carries a SHA-256 on purpose. The digest is the thing
 * that matters and is over the payload only, so a caller can compute the expected value from the
 * vendor file alone -- no knowledge of this layout. The header needs torn-write detection and
 * nothing stronger, and a second SHA pass over 84 KiB to protect 60 bytes would be a cost with no
 * question behind it.
 */

#pragma once

/* Guarded like every other file in this family, and for the same reason: every source cpp is
 * globbed, so without it this module would compile into the production image and drag SHA-256 and
 * CRC-32 in with it. The production image is byte-identical until the chain is enabled. */
#if defined(ENABLE_TOF_CHAIN)

#include <stddef.h>
#include <stdint.h>

namespace lexxhard::tof_l7_blob {

inline constexpr size_t kHeaderSize{64};
inline constexpr size_t kDigestSize{32};
inline constexpr size_t kCommitMarkerSize{4};
inline constexpr uint32_t kMagic{0x3142374CU};   // 'L','7','B','1' little-endian
inline constexpr uint32_t kCommitMarker{0x4B4F374CU}; // 'L','7','O','K' little-endian
inline constexpr uint16_t kFormatVersion{2};

/* One value per distinct diagnosis. The list is longer than a bool because every entry sends
 * whoever reads it somewhere different: `absent` means provision the board, `version_mismatch`
 * means provision a DIFFERENT blob, `digest_mismatch` means the flash is damaged, and
 * `unreadable` means the flash controller is. Collapsing them would produce one message that fits
 * every case and helps in none. */
enum class status : uint8_t {
    ok = 0,
    unreadable,          // the reader failed; nothing can be said about the contents
    absent,              // erased flash: nothing has ever been provisioned here
    uncommitted,         // header/payload may exist, but the last-written marker does not
    bad_magic,           // something is there and it is not one of these records
    unsupported_format,  // a record, from a format this firmware does not implement
    bad_header,          // header CRC failed: a torn or damaged write
    length_out_of_range, // the payload cannot fit the region it claims to be in
    length_mismatch,     // intact, but not the length this firmware expects
    version_mismatch,    // intact, and NOT the blob this firmware was built against
    digest_mismatch,     // the payload does not hash to what its own header says
    mapping_mismatch,    // flash_area bytes differ from the pointer that would be handed out
    no_expectation,      // the signed image carries no accepted blob identity
};

const char *status_name(status s);

// What the caller knows about the blob it needs. Both fields come from the vendor file the firmware
// was built against, via the generated expectation header -- never from the record being checked.
struct expectation {
    size_t payload_len{0};
    uint8_t payload_digest[kDigestSize]{};
};

/* Compatibility is a set, not a single digest. A rollback can pair an older application with a
 * newer provisioned blob, so a release may deliberately accept more than one bench-proven payload.
 * The list itself lives in the signed image; no identity beside the stored blob is authoritative. */
struct accept_list {
    const expectation *entries{nullptr};
    size_t count{0};
};

/* Reads bytes out of wherever the record lives. Injected so that the logic in this file is testable
 * without flash, and so that the same logic serves a mapped-flash provider and a host test buffer.
 *
 * read() must fill exactly len bytes or return non-zero; a short read is a failure, not a partial
 * success -- a digest over a partly-filled buffer is a digest over the wrong thing. */
struct reader {
    int (*read)(void *ctx, size_t offset, void *dst, size_t len){nullptr};
    void *ctx{nullptr};
};

/* A verified payload. The pointer is only meaningful for a memory-mapped region; a provider that
 * cannot map hands back size and offset instead and streams.
 *
 * There is no way to construct a populated one outside verify(). That is the same rule the mapping
 * proof's token follows, for the same reason: the value IS the authorisation, and a type that can
 * be filled in by anybody authorises nothing. */
class blob_view {
public:
    blob_view() = default;
    const uint8_t *data() const { return data_; }
    size_t size() const { return size_; }
    bool valid() const { return data_ != nullptr && size_ != 0; }

private:
    friend status verify(const reader &, size_t, const accept_list &, blob_view &,
                         const uint8_t *);
    const uint8_t *data_{nullptr};
    size_t size_{0};
};

/* Checks a record and, on success only, fills `out`.
 *
 * region_size bounds the record: header plus payload must fit inside it. mapped_base, when not
 * null, is where the region is visible in the address space -- the view's pointer is
 * mapped_base + kHeaderSize. Pass null when the region is not mapped; the status is then still
 * meaningful and `out` stays empty.
 *
 * On any refusal `out` is cleared, including when it carried a view from an earlier success. A
 * caller that checks only valid() therefore cannot reuse stale authorisation after a failed check.
 */
status verify(const reader &r, size_t region_size, const accept_list &accepted, blob_view &out,
              const uint8_t *mapped_base);

/* The header fields, for a diagnostic that wants to say what IS stored when it does not match --
 * "expected 86,016 bytes, found 84,992" is actionable where "length_mismatch" alone is not.
 * Populated on a best-effort basis: whatever was parseable before the refusal. */
struct header_info {
    bool parsed{false};
    uint16_t format_version{0};
    uint32_t payload_len{0};
    uint8_t payload_digest[kDigestSize]{};
};

status read_header(const reader &r, size_t region_size, header_info &out);

}  // namespace lexxhard::tof_l7_blob

#endif  // ENABLE_TOF_CHAIN
