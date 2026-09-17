# L7 product integration — status

**NOT MERGEABLE. Capacity blocked pending a product decision on the SD/FAT stack.**

This branch carries the VL53L7CX base layer only, rebuilt on the product stack rather than merged
from the bench branch. It is pushed for review, not for merge.

## What is here

Cut from `origin/pr-b3/publisher-can-runtime` at `25c316ab`, four commits, pure addition:

  - the vendored ULD with the device firmware externalised behind `VL53L7CX_EXTERNAL_FIRMWARE`
  - the blob record, its provider and the boot gate
  - the single-sensor adapter and its two host suites
  - the build gating, off by default

Three host suites pass on this branch: `tof_l7_blob` 15, `tof_l7_port` 12, `tof_l7_sensor` 16.

## What turning the option off actually costs

Measured, not asserted. Same parameters on both sides, fixed `VERSION=99.99.99`, raw
`zephyr.bin` compared byte for byte against `origin/pr-b3/publisher-can-runtime` at `25c316ab`:

| shape | pr-b3 | this branch | result |
| --- | --- | --- | --- |
| production (`firmware`: no chain, no L7) | 194,688 | 194,688 | **byte-identical**, same SHA-256 |
| chain (`firmware_tof_chain`: chain on, L7 off) | 197,160 | 197,184 | **+24 bytes** |

The production image is unchanged, and for a stronger reason than "the files are not
compiled": all four `tof_l7_*.cpp` **are** compiled -- `src/*.cpp` is globbed -- and their
guards leave them empty, so the linker emits the same image.

The chain build is **not** identical, and the earlier blanket claim that it was has been
narrowed. `tof_l7_blob_record.cpp` and `tof_l7_blob_provider.cpp` guard on `ENABLE_TOF_CHAIN`
alone, so they carry real content there -- 852 and 468 bytes of object -- while
`tof_l7_runtime` and `tof_l7_sensor` stay empty. Nothing calls them, `--gc-sections` removes
almost all of it, and 24 bytes survive.
`ENABLE_TOF_L7_ULD` off — which is everywhere, no named target sets it — the production image is
unchanged.

## What is deliberately not here

No acquisition binding. `grid_source_ops` is untouched, the grid descriptors are still empty for
non-cliff positions, `on_grid_stub()` still discards its samples, and there is no 0x214/0x215
publisher and no blob writer. That work waits on the capacity gate below.

## Why it cannot merge yet

Measured with the repository's Docker builder and the commissioning reachability probe, rooting all
nine ULD entry points the adapter actually calls:

| build point | signed.bin | headroom to 261,712 |
| --- | --- | --- |
| L7 code present, no lifecycle caller | 240,672 | 21,040 |
| lifecycle forced reachable | 248,116 | **13,596** |

The planning stop line is 15,360 bytes. 13,596 is **1,764 below it**, and that is before the typed
grid ops, two `VL53L7CX_Configuration` objects, sensor scratch, descriptor wiring and the publisher.

The acceptance condition is **the complete L7 path linked, still leaving at least 15,360 bytes** —
not this probe closing its own gap. Each restored piece of wiring is to be measured as its own
commit against that bar.

A same-parameter A/B shows the gap is closable without touching the cliff, ToF, CAN or DFU paths:
removing the SD-card FAT stack, which no application code calls, saves 26,276 bytes and lifts
headroom to 39,872. **That removal is a separate product decision and a separate commit.** It takes
away shell access to the SD card, and a repository search cannot establish whether manufacturing,
service or field staff rely on it — release and service have to say. It is not folded into this
branch.

Full evidence, including the ROM/RAM reports and the A/B fragment, is in
`allmemory/hanging_object/l7_capacity_2026-09-17/`.
