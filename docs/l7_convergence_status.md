# L7 convergence — status

**NOT MERGEABLE.** This is where the L7 grid path is finished: on the baseline that actually runs
on a machine, rather than on the branch it was written on.

## Why this branch exists, and why it is not a merge

Cut from `feature/AMRSW-2994-auto-commission` at `741db238`. That baseline already carries the L4
fixed-period scheduling and timing instrumentation, the L4 ranging profile injection, the removal of
the PROVEN clamp, and the whole auto-commissioning downlink. The L7 work was written on
`feature/AMRSW-2322-l7-grid-ops`, cut from the product stack at `25c316ab`.

The two lineages have diverged by **105 and 47 commits**, over 117 overlapping files. Merging the
branch wholesale would re-merge two histories of the same feature, and the file it would have to
reconcile is `tof_acquisition` — 846 lines added and 302 removed between them — where the newer side
is the one with the scheduling work. The risk is not a messy diff: it is reverting scheduling and
clamp changes that were verified on hardware.

**So the migration is selective, and it is small because the baseline already has most of it.** The
L7 blob record, provider, runtime and adapter, and the vendored VL53L7CX ULD, are **byte-identical**
on both lineages. The typed grid interface is already here too: `grid_source_ops`,
`l7_grid_stub_ops()`, the `on_grid_sample` sink and the model-neutral `source_status`. The commit
that introduced them on the L7 branch (`27ce1d41`) is deliberately **not** migrated -- taking it
would overwrite the newer acquisition implementation with an older one that happens to contain the
same interface.

What was migrated: `9b117306`, the grid publisher and its suite, by `cherry-pick -x`. Only two
conflicts, both by hand: the test's CMakeLists, and the L7 branch's own status note, which describes
that branch and does not belong here.

## Where it stands

The publisher compiles against this baseline's acquisition types **unchanged** -- which is the
evidence that not migrating `27ce1d41` was right, rather than merely cheaper.

Host suites in `tests/tof_cliff_sensor`: `tof_acquisition` 81, `tof_cliff_adapter` 30,
`tof_cliff_port` 12, `tof_cliff_publisher` 40, `tof_cliff_stream_loop` 9, `tof_grid_publisher` 30.
In `tests/tof_commissioning`: `tof_cliff_runtime` 15, `tof_commissioning` 26, `tof_grid_wiring` 11.
Elsewhere: `tof_mapping_proof` 38, `tof_mapping_authority` 43, `tof_l7_sensor` 16. All pass.

## The two jobs on one chain, and their names

The chain carries six boards doing two different things, and the modules are named after the one
that came first. Worth stating once, here:

  - the two **VL53L7CX** at positions 1-2 look FORWARD and read an 8x8 grid. They detect **hanging
    objects** -- half-height obstacles the machine would drive into. Their frames are 0x214/0x215
  - the four **VL53L4CX** at positions 3-6 look DOWN and read one distance each. They detect a
    **drop** -- the cliff path. Their frames are 0x216/0x217

Neither is a variant of the other. `tof_cliff_runtime` and `tof_cliff_can` are named for the second
because it existed first, and both now serve the chain rather than the cliff; the new code says so
where it sits in them. Renaming those modules is worth doing and is not this work's to do.

## What this branch wired, and what it cost

The grid path is complete in firmware: the real ops table binds the scheduler to the adapter, the
mapping install keys the two hanging sensors from `fp.at[i].source_id`, the hooks fan out to both
publishers, and the grid publisher's init is a precondition for acquisition rather than a warning.
The ranging frequency comes from the devicetree, through the bootstrap, into the descriptors and out
to the ULD; there is no default on that path and 0 or >15 is refused at the bootstrap.

**The tests caught one defect that no review had:** the adapter's lifecycle is per session -- open()
takes an empty object, stop() leaves a configured one -- so a SECOND bring-up refused both hanging
sensors with -EPERM. On a machine that is commissioned twice in one boot, both would simply have
failed to come up. The objects are now returned to empty before every bring-up, by the layer that
knows a new session is starting.

### Capacity: the wired configuration does not fit

Measured with the repository's Docker builder, `VERSION=99.99.99`, same parameters throughout.

| build | FLASH used | against the 261,712 B ceiling |
| --- | --- | --- |
| cliff only, at the branch point `741db238` | 248,704 | 12,912 spare |
| cliff only, at this commit | 248,808 | 12,808 spare |
| **cliff + hanging sensors, fully wired** | **262,348** | **636 OVER** |

The last row did not link: `region FLASH overflowed by 204 bytes` against the 256 KiB slot, and the
usable ceiling is 432 bytes below that. **There is no image to flash from this configuration
today.**

The always-on part of this commit -- the mapping install's new checks, the fan-out, the config field
-- costs **104 bytes** and leaves RAM unchanged at 220,736. Everything else is the grid path, and
where it goes is not a mystery: the vendored ULD's `vl53l7cx_api.c` is 7,813 bytes of text, the
publisher 1,852, the adapter 898, the blob runtime 812, the packer 304, the port 680. Static RAM
grows by about 5,700 bytes, nearly all of it the two `VL53L7CX_Configuration` objects and their
shared scratch, against 384 KiB of it -- RAM is not the problem.

The known candidate is the SD-card FAT stack, measured at **26,276 bytes** and called by no
application code. That is a product decision with a service consequence, not a decision this branch
may take: it removes shell access to the SD card, and whether manufacturing or field staff rely on
it cannot be settled by a repository search. **Until somebody decides, the grid path is complete in
source and unflashable in practice.**

### Stack

`run_cycle()`'s frame is **300 bytes**, measured by disassembling the object in both builds, and
this commit does not change it: the 258-byte grid sample is a local in the scheduler's grid arm,
which the cliff-only build already compiles. What the frame does NOT show is the ULD's own depth
below `read_once()`, so the acquisition thread's 2,048-byte stack is still unproven -- a
`CONFIG_THREAD_ANALYZER` watermark on a board remains the final word, as it was before.

## What the next commit must do, and the rules it is held to

Three things that belong together, because any one of them alone leaves a path that reads grids and
discards them, or publishes them under a position nobody proved.

**The mapping install must learn about L7.** `install_from_mapping()` currently skips every
non-cliff descriptor, so L7 positions keep `kRoleUnassigned` and the publisher -- correctly --
refuses them. The source id must be taken from **`fp.at[i].source_id`**: not from
`source_id_of(role)`, which answers a cliff-corner question, and not from the descriptor index,
which is a guess wearing the shape of a fact. Each L7 position must be verified on all of:

  - `position == i + 1`
  - the expected model at that position is the L7
  - the address matches the descriptor's
  - `verified == true`
  - the source id is 0 or 1, and no two positions claim the same one

The existing two-pass shape stays: validate every position into a temporary, write only when all of
them passed. A single pass that wrote as it validated would leave a half-keyed table on the first
refusal -- some positions published correctly, the rest not at all, and nothing on the wire to say
which.

**The real grid ops table, and the sink, in the same commit.** `on_grid_stub()` must go with
`l7_grid_stub_ops()`, with a test showing a grid actually reaches the publisher. Leaving the empty
sink beside a real table would read grids and discard them silently, which on the wire is
indistinguishable from a sensor that is not there.

**A fan-out, never a replacement.** `sinks::on_cycle_begin` and `sinks::on_cycle` hold one function
pointer each, and the cliff publisher already owns them. Wiring the grid publisher into those slots
directly would silently stop the four cliff ranges that already work on hardware.

## The pre-merge gate that is not in this branch's way

The contract text does not state a base for `chain_position`. The publisher reports the **0-based
descriptor position** -- 0 and 1 for the two L7 boards -- which is what the contract's own golden
vectors already carry, and which differs from the cliff health frame's 1-based failing position.
Before merge, `docs/can/tof_can_wire_contract.md` says so explicitly, the version is bumped, the
vectors are regenerated and both repositories' pins are updated. It blocks merging, not building.
