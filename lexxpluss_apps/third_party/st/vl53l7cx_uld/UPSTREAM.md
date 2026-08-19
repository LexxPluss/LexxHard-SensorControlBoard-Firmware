# VL53L7CX ULD — vendored snapshot

A fixed source snapshot, not a submodule or west module. It follows the same ownership model as the
VL53L4CX ULD beside it: upstream bytes are reviewable and buildable offline, local Zephyr code stays
outside `upstream/`, and the production source list is explicit.

## Provenance

| | |
| --- | --- |
| Source repository | `LexxHard-ToFSensorBoard-Firmware` |
| Source path | `Drivers/BSP/Components/vl53l7cx/` |
| Repository commit when copied | `ec23b724d573b448a5376727205ec3978db8f930` |
| Commit that introduced the subtree | `f86fcefcfb9dee665c8a8990a4012591cb791572` |
| ST ULD revision | **VL53L7CX 2.0.0** |
| Device firmware | **MM1.8**, 86,016 bytes |
| Version evidence | `upstream/modules/vl53l7cx_api.h` and `vl53l7cx_buffers.h` |
| Licence | BSD-3-Clause, `upstream/LICENSE.md`, copied verbatim |
| Copied on | 2026-08-19 |

The source `vl53l7cx_api.c` is byte-identical to the copy used by the existing NECM transmitter
project (`f7b12482…`). The unrelated `NECM/temp_vl53l7cx_c_driver` copy has a different hash and was
not used.

## Integrity

`SHA256SUMS` covers every file under `upstream/`, sorted by relative path. The only exclusion is
`.clang-format`, which is ours and prevents tools from rewriting vendor files:

    cd upstream && sha256sum -c ../SHA256SUMS

## Snapshot boundary

This is the complete compilable component source: the BSP wrapper, ULD core, all three plugins,
upstream example platform, headers and licence. The release-note HTML and `_htmresc/` presentation
assets are omitted. Version evidence remains in the source macros named above.

Nothing inside `upstream/` is edited. The upstream example `porting/platform.c` is retained as
provenance and never compiled: it delegates into user callbacks and provides neither the 328-byte
segmentation bound nor sticky Zephyr errno reporting.

## Local patch

`zephyr/patches/0001-use-verified-external-firmware.patch` is applied to copies in the build
directory. It makes two narrow changes:

1. omit the 86,016-byte `VL53L7CX_FIRMWARE[]` when `VL53L7CX_EXTERNAL_FIRMWARE` is defined, while
   retaining the signed-image default-configuration and xtalk arrays;
2. require a verified `platform.firmware` plus a bound covering all 86,016 bytes, and use that
   pointer for the ULD's three fixed download ranges.

The patch is never applied in place, so `SHA256SUMS` continues to answer whether upstream changed.
Its build-directory application also fails configuration if the upstream context drifts.

## What is compiled

`vl53l7cx_sources.cmake` contains no glob:

- **production**: patched `vl53l7cx_api.c` and the Zephyr platform implementation;
- **optional**: xtalk, motion-indicator and detection-threshold plugins, plus the STM32 component
  wrapper. They remain available but are not part of the measured grid path.

The Zephyr platform keeps the ST 8-bit address convention at the ULD boundary and converts once to
Zephyr's 7-bit address. Every transfer carries a 16-bit big-endian index; requests larger than 328
bytes are split into independently indexed transactions. The port has no bounce buffer.

The ULD collapses transport failures into an 8-bit status, so the original errno is recorded as a
sticky first error. Because the ST callback surface has no per-device context for that record, all
ULD lifecycle and ranging calls must remain on the one acquisition thread.
