# L7 product integration — status

**NOT MERGEABLE.** Under construction: the L7 path is being wired one commit at a time, and each
commit is measured in the fully-enabled configuration before the next one starts.

This branch carries the VL53L7CX base layer, rebuilt on the product stack rather than merged from
the bench branch, plus the typed acquisition interface above it. It is offered for review, not
for merge.

## What is here

Cut from `origin/pr-b3/publisher-can-runtime` at `25c316ab`, pure addition. Four implementation
commits carry the base layer, one more carries the typed interface, and the remainder are status and
check commits, this note among them. The base layer is:

  - the vendored ULD with the device firmware externalised behind `VL53L7CX_EXTERNAL_FIRMWARE`
  - the blob record, its provider and the boot gate
  - the single-sensor adapter and its two host suites
  - the build gating, off by default

On top of the base layer, the acquisition interface is now typed per model: `grid_source_ops` with
its explicit frequency argument, `l7_grid_stub_ops()` still entirely `-ENOSYS`, the `on_grid_sample`
sink, and a model-neutral `source_status` carrying the domain its stage number belongs to. No real
sensor is bound and nothing packs or publishes a grid.

Host suites passing on this branch: `tof_l7_blob` 15, `tof_l7_port` 12, `tof_l7_sensor` 16,
`tof_acquisition` 58, `tof_cliff_adapter` 37, `tof_cliff_port` 12, `tof_cliff_publisher` 41,
`tof_cliff_runtime` 15, `tof_commissioning` 16.

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

## What is deliberately not here

No real sensor. The grid descriptors carry the named `-ENOSYS` stub, `on_grid_stub()` is a sink
nothing can reach, there are no `VL53L7CX_Configuration` objects and no scratch, and there is no
0x214/0x215 publisher and no blob writer.

**An obligation on whoever binds a real grid table:** `on_grid_stub()` must be replaced in the same
commit, with a test showing a grid actually reaches its publisher. Leaving the empty sink in place
would read grids and discard them silently, which on the wire is indistinguishable from a sensor
that is not there.

## Capacity, per commit

Measured with the repository's Docker builder and the commissioning reachability probe, rooting all
nine ULD entry points the adapter actually calls. The probe's non-release guard — `VERSION=99.99.99`
and the safety-lidar bypass — is untouched, so a forced-reachability image cannot enter release
signing.

| commit | build point | signed.bin | headroom to 261,712 |
| --- | --- | --- | --- |
| `3dca2d30` base layer | code present, no lifecycle caller | 240,672 | 21,040 |
| `3dca2d30` base layer | lifecycle forced reachable | 248,116 | 13,596 |
| typed acquisition interface | code present, no lifecycle caller | 241,140 | 20,572 |
| **typed acquisition interface** | **lifecycle forced reachable** | **248,588** | **13,124** |

The typed interface costs **472 bytes** of image and **52 bytes** of static RAM.

It also costs stack, which the RAM report does not show. `run_cycle()`'s frame grows from 60 to 292
bytes because the 258-byte `tof_l7::sample` is a local in the grid arm, and the frame is reserved
whether or not that arm runs: **+232 bytes on the acquisition thread's deepest frame**. The measured
watermark before this change was 1144 of 2048 on dasher2, so the projection is about 1376 of 2048.
It fits, it is a projection rather than an observation, and it has to be re-measured on hardware
before release.

## The ceiling, and the planning line

The hard limit is **261,712 bytes** — imgtool's, with the 432-byte trailer, not the 262,144 slot
size. Nothing here is near it.

15,360 bytes is an internal planning margin, not a product gate. A build that lands below it is not
automatically refused: it is recorded, and the release owner is asked to accept the margin. Treating
it as a hard bar was over-conservative, and it had the practical effect of proposing to delete
working functionality before the code whose cost is in question had even been written.

So the order is: finish wiring the L7 path, measure the complete thing, and only then decide what, if
anything, has to give.

## The reduction candidate, if one turns out to be needed

A same-parameter A/B shows a gap of this size is closable without touching the cliff, ToF, CAN or DFU
paths: removing the SD-card FAT stack, which no application code calls, saves 26,276 bytes. **That
removal is a separate product decision and a separate commit**, it takes away shell access to the SD
card, and a repository search cannot establish whether manufacturing, service or field staff rely on
it — release and service have to say. It is not folded into this branch, and it is not a
prerequisite for continuing the work above.

Full evidence, including the ROM/RAM reports and the A/B fragment, is in
`allmemory/hanging_object/l7_capacity_2026-09-17/`, and this commit's measurement in
`allmemory/hanging_object/l7_capacity_2026-09-18-gridops/`.
