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

Host suites in `tests/tof_cliff_sensor` on this branch: `tof_acquisition` 81, `tof_cliff_adapter` 30,
`tof_cliff_port` 12, `tof_cliff_publisher` 40, `tof_cliff_stream_loop` 9, `tof_grid_publisher` 30.
All pass. The full regression across every suite, and the capacity measurement, belong to the wired
configuration below -- not to this commit, where nothing calls the publisher and the linker drops it.

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
