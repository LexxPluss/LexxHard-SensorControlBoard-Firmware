# Stored VL53L7CX device-firmware record

Status: **reader, vendor identity and boot-time gate implemented; board unprovisioned.** The reader
and packer agree, the VL53L7CX 2.0.0 / MM1.8 snapshot supplies the signed-image expectation, and
boot refuses L7 unless storage verifies. No board has yet been programmed with this record.

Owner of this format: this document. The reader
(`lexxpluss_apps/src/tof_l7_blob_record.cpp`) and the packer (`docs/can/gen_l7_blob_record.py`)
implement it; where they disagree with this text, they are wrong.

## Why the blob is not in the signed image

Measured 2026-08-11/12, on this board and this toolchain:

| thing | bytes |
| --- | --- |
| `VL53L7CX_FIRMWARE[]` | 86,016 |
| `VL53L7CX_DEFAULT_CONFIGURATION` | 972 |
| `VL53L7CX_DEFAULT_XTALK` | 776 |
| unpadded signed-image ceiling | 261,712 |
| B6 signed image before this L7 work | 239,516 |
| B6 after the record gate, vendor import and expectation | 243,604 |
| B6 with init/start/ready/fetch/stop forced reachable | 250,936 |

The pre-L7 B6 point had 22,196 B left under the measured MCUboot-aware ceiling; embedding the
86,016-byte payload would exceed it by at least 63,820 B before adding the L7 adapter. Compression
does not rescue the design: LZMA 61,848 and `xz -9e` 62,168 would consume nearly all of that tail
before a decompressor or the remaining L7 path. Garbage collection cannot help either — init needs
every byte.

The 2026-08-19 forced-reachability build is the current planning bound for the next phase. Making the
ULD lifecycle plus port and retained configuration/xtalk arrays reachable adds 7,332 B over the
Phase 1 image and leaves 10,776 B. That is below the agreed 15 KiB stop line, before the adapter,
two live L7 objects, scheduling code or CAN glue. Phase 1 may land, but the data path must not simply
continue from here without a capacity decision or a measured reduction.

Only the 86,016-byte device-firmware blob moves out of the image. The 972-byte default
configuration and 776-byte xtalk table remain with the ULD in the signed image, because rolling
application code back while silently retaining newer configuration would create another
compatibility problem. `storage_partition` — 131,072 B at offset `0x20000` on `lexxpluss_scb` — is
the one flash region no application code referenced. **One blob fits; two do not**, which is the
fact that decides the open question below.

## The record

Little-endian throughout. A 64-byte header, the payload, then a four-byte commit marker, written at
offset 0 of the region. Format version 2 added the trailing marker before any board was provisioned.

| offset | size | field |
| --- | --- | --- |
| 0 | 4 | magic, `'L' '7' 'B' '1'` (`0x3142374C` read as LE32) |
| 4 | 2 | `format_version`, currently 2 |
| 6 | 2 | `header_size`, currently 64 |
| 8 | 4 | `payload_len` |
| 12 | 32 | `payload_digest`: SHA-256 of the payload **alone** |
| 44 | 16 | reserved, all zero |
| 60 | 4 | `header_crc32`: CRC-32/IEEE over bytes 0..59 |
| `64 + payload_len` | 4 | commit marker, `'L' '7' 'O' 'K'`; written last |

### Why a CRC on the header and a SHA-256 on the payload

They answer different questions and cost differently. The header needs torn-write detection over 60
bytes, which CRC-32 covers completely; a second SHA-256 pass to protect it would be a cost with no
question behind it. The payload digest is over the payload **only**, so that whoever needs the
expected value can compute it from the vendor file with `sha256sum` and no knowledge of this layout.

## What a reader must check, and in this order

1. `region_size >= 64`, and the header reads without error → otherwise `unreadable`.
2. All 64 header bytes `0xFF` → `absent`. An unprovisioned board is a normal state, not damage, and
   must not be reported as corruption.
3. Magic → `bad_magic`.
4. `format_version` and `header_size` both exactly as implemented → `unsupported_format`. An older
   reader must **refuse** a newer format rather than parse the fields it happens to recognise.
5. `header_crc32` → `bad_header`.
6. Reserved bytes all zero → `unsupported_format`. Fail closed: a record using them is from a format
   this reader does not implement, whatever its version number says.
7. `payload_len` non-zero and `64 + payload_len + 4 <= region_size` → `length_out_of_range`. Bounds
   before contents, so an impossible length is never hashed by reading past the region.
8. The trailing commit marker is present → otherwise `uncommitted`.
9. Every payload chunk read through `flash_area_read` equals the same bytes at the mapped address
   that will be handed to the ULD → otherwise `mapping_mismatch`.
10. The payload hashes to its own header's `payload_digest` → `digest_mismatch`.
11. `payload_len` appears in the signed image's accept-list → `length_mismatch` otherwise.
12. `(payload_len, payload_digest)` appears in that accept-list → `version_mismatch` otherwise.

### Identity and integrity are separate refusals

Steps 9-10 establish that the stored and mapped bytes are intact. Steps 11-12 then ask *is this one
of the blobs this signed image was built to accept*. Integrity deliberately comes first: before the
payload hashes to its own header, “intact but another version” has not been proved.

It matters because the ULD and its device firmware are a matched pair — the API indexes fixed
offsets inside the blob (`0x8000`, `0x10000`, and a final write of `0x8000+0x8000+0x5000` = 86,016)
— so an intact blob from a different ULD release is exactly the failure a hash looks like it
prevents. The two also send whoever reads them to different places: `version_mismatch` means
provision a different blob, `digest_mismatch` means the flash is damaged.

The accept-list of `(length, digest)` pairs is compiled into the image, generated from bench-proven
vendor files by `docs/can/gen_l7_blob_record.py pack --expect-header` plus optional
`--accept-blob`. It is never taken from the record being checked. A list rather than one value is
required because rollback can pair an older application with a newer provisioned blob; every extra
entry is therefore a compatibility claim that needs a bench result with that ULD.

## Handing out the payload

Internal flash is memory-mapped and readable while code executes from it — only erase and program
stall the bus — so a verified payload is handed to the I2C port as a pointer and streamed straight
into transfers. There is no 84 KiB RAM copy, and there is no RAM to make one in.

Two rules follow:

- The verification reads through `flash_area_read` and compares every chunk with the mapped bytes
  it will authorise. A wrong mapped base therefore becomes `mapping_mismatch`; proving one access
  path and handing out another is forbidden.
- Nothing can obtain the pointer without the verification: `blob_view` has no public way to be
  filled in, and `verify()` is its only producer.

## Provisioning

`docs/can/gen_l7_blob_record.py pack <blob.bin> --out record.uncommitted.img
--commit-marker-out marker.bin --expect-header <header>` produces three deliberately separate
artefacts: header+payload, the four commit bytes, and the signed-image accept-list. The safe ordering
is therefore the natural use of the tool rather than a comment attached to an already-committed
image. A provisioner must erase, write `record.uncommitted.img`, read it back and verify the payload
digest, then write `marker.bin` at offset `64 + payload_len`.

The initial manufacturing path uses SWD at physical address `0x08020000` (`storage_partition` is at
offset `0x20000` inside internal flash); 86 KiB over the shell console is not a serious proposition.
A future in-field writer must preserve exactly the same ordering.

**Open, and deliberately not decided here:** the blob is not part of the A/B image pair. One copy
fits the partition and two do not, so a firmware update cannot carry a new blob the way it carries a
new application, and a board whose blob and application disagree refuses to range rather than
ranging wrongly (step 12). Whether provisioning stays a manufacturing step, moves to a CAN transfer,
or forces a repartition is an ADR, not a code change — the repartition option costs an entire 256 KiB
erase sector, i.e. the second slot or the scratch area, i.e. the ability to roll back.

## Cost

Every boot pays one SHA-256 over the payload. Its real duration is not yet measured and remains part
of hardware acceptance. It is expected to be smaller than downloading 86,016 B to each sensor, but
that comparison is not a timing claim until both are measured on the board.

## Not covered by this document

The wire contract for grid measurements (`0x214`/`0x215`) is in `docs/can/`. The vendored ULD and
the Zephyr port are separate modules beside the reader; this document still ends where the verified
payload is handed over.
