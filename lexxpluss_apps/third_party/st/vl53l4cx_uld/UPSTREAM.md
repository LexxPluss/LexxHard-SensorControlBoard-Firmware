# VL53L4CX ULD — vendored snapshot

A fixed source snapshot, not a submodule and not a west module. The reasons are
recorded here because the choice constrains everything downstream.

`LexxHard-ToFSensorBoard-Firmware` is a complete application rather than a reusable
Zephyr module, so adding it to the west manifest would drag in unrelated firmware and
a new network dependency; the SCB manifest currently pins Zephyr and nothing else, so
there is no precedent for managing internal modules. A submodule would add checkout,
CI, offline-build and version-synchronisation complexity for a driver that does not
move. And safety-related code is better pinned and reviewed inside one SCB commit
than left exposed to an external branch changing under the build.

## Provenance

| | |
| --- | --- |
| Source repository | `LexxHard-ToFSensorBoard-Firmware` |
| Source path | `Drivers/BSP/Components/vl53l4cx/` |
| Repository commit when copied | `ec23b72` |
| Commit that last introduced the ULD subtree | `f86fcef` (initial commit) |
| ST package version | **1.2.13 rev 2676** |
| Version evidence in tree | `upstream/modules/LXReleaseNotes.txt` line 2, and `VL53LX_IMPLEMENTATION_VER_MAJOR/MINOR/SUB` = 1/2/13 in `upstream/modules/vl53lx_def.h` |
| Licence | BSD-3-Clause, `upstream/LICENSE.md`, copied verbatim |
| Copied on | 2026-08-12 |

The licence file must travel with the tree: every ST source header says only that the
software "is licensed under terms that can be found in the LICENSE file", so without
`upstream/LICENSE.md` those headers point at nothing.

## Integrity

`SHA256SUMS` covers **`upstream/` only**, with sorted relative paths and no comment
lines, so it verifies cleanly:

    cd upstream && sha256sum -c ../SHA256SUMS

The one file inside `upstream/` that the manifest deliberately excludes is
`.clang-format`, which is ours: it carries `DisableFormat: true` so that neither an
editor nor a future repository-wide formatter rewrites vendor code. Nothing else in
`upstream/` is ours, and nothing in `zephyr/` is upstream's, so the manifest answers
"has anyone edited upstream?" with one command.

## Local modifications

**None to upstream files.** Upstream is never edited in place. All local code lives in
`zephyr/`:

| File | Role |
| --- | --- |
| `zephyr/vl53lx_platform.c` | The platform layer the ULD requires, on Zephyr I2C |
| `zephyr/vl53l4cx_bus_io.c/.h` | The `VL53L4CX_IO_t` block registered with the BSP wrapper |

If an upstream change ever becomes unavoidable, it lands as a numbered patch file
under `zephyr/patches/` with a note here — never as an in-place edit, because that
would break the manifest's meaning.

`upstream/porting/vl53lx_platform.c` is kept **as provenance and is not compiled**. It
is the upstream example layer: it carries a single file-scope `_I2CBuffer[256]` shared
by every device plus its own tracing, so it is not reentrant across the four cliff
sensors on one bus and does not fit our locking or test boundaries.

## Deliberately omitted

This is a **source** snapshot, not a complete release package. Omitted from
`Drivers/BSP/Components/vl53l4cx/`:

- `Release_Notes.html`
- `_htmresc/st_logo_2020.png`, `_htmresc/favicon.png`
- `modules/VL53LX_API.chm`

`modules/LXReleaseNotes.txt` is kept, because it is the version evidence.

## What is compiled

`vl53l4cx_sources.cmake` holds two explicit lists and never globs:

- **production** — the ranging path: BSP wrapper, ULD core, histogram algorithms,
  register access, wait, NVM, crosstalk processing, the IPP shim, **calibration**, and our
  two local files. This group is what the flash budget measures.
- **optional** — `vl53lx_api_debug.c` and `vl53lx_nvm_debug.c`. Present in the snapshot so
  provenance is complete and enabling one is a single line, but not compiled today so the
  budget figures mean what they say.

`vl53lx_api_calibration.c` is in **production**, not optional, and that placement is a
link-report finding rather than a judgement: `VL53LX_PerformRefSpadManagement` lives in
`vl53lx_api.c`, is reachable from the BSP init path, and calls `VL53LX_run_ref_spad_char`
inside the calibration unit. Excluding it by its name failed to link.

Nothing is deleted from `upstream/` to shrink the build. Linker garbage collection
already drops unreachable code, so removing an unreachable file would save nothing;
the split exists to keep the budget honest and the review surface small. Third-party
warnings are silenced by a narrow per-source exemption in the app's `CMakeLists.txt`,
never by deleting a file and never by a global `-Wno-*`.

## Two constraints the port inherits, and one it enforces

**The BSP IO callbacks carry no per-device context.** Their signature is
`(uint16_t address, uint8_t *data, uint16_t size)`, so a Zephyr bus handle can only be
resolved at file scope. That is safe here **only** because all four VL53L4CX sit on the
same immutable `i2c2` controller and are told apart by `IO.Address`. Moving any L4 to a
second bus would silently address the wrong controller, and this file must gain an
explicit per-device binding before that happens.

**`IO.Init` and `IO.DeInit` must exist.** `VL53L4CX_RegisterBusIO` returns
`VL53L4CX_ERROR` when `IO.Init` is null, and `VL53L4CX_DeInit` calls `IO.DeInit()` with
no null check at all, so omitting either is a failure or a null-pointer jump rather
than a shortcut. Both are checked no-ops: the bus, its speed and the enable chain
belong to the chain controller. `IO.GetTick` is **real**, not a stub, because
`upstream/vl53l4cx.c` polls with it directly.

**There is exactly one real I2C path**, in `zephyr/vl53lx_platform.c`, using two
`i2c_msg` segments for the 16-bit big-endian register index and the payload.
`IO.WriteReg` and `IO.ReadReg` are therefore placeholders that fail loudly and count
their calls; any traffic through them means a second transport path appeared behind the
port's back, and the tests assert the counter stays at zero.

**Every GPIO hook returns `VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED` (-12).** The
enable line is a distributed shift register owned by the chain controller, and dropping
an L4's enable returns it to address `0x29` and destroys the chain's addressing, so
there is no correct implementation. A successful no-op would let a caller believe it had
reset a sensor.
