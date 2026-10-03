# `docs/can` — CAN wire contracts, generators and conformance artefacts

This directory holds **versioned contract inputs, generators, production headers and conformance
vectors** for the CAN wire protocols between the SCB firmware and SCBDriver. Some files here are used
by production code; the others exist to make those interfaces reproducible and testable.

This README is an index and a workflow guide. **The contract files and the generated artefacts
remain authoritative**; nothing here is normative.

## File roles

Two contracts live here and **their generation relationships are not the same**. The cliff contract
emits a production header; the grid contract does not.

### Cliff ToF (`0x216` measurement, `0x217` health, `0x218`/`0x219` commissioning)

| File | Role | Used by production code |
| --- | --- | --- |
| `tof_cliff_wire_contract.md` | Human-readable normative input. Its version line is parsed and all of its bytes contribute to the contract identity | No |
| `gen_cliff_golden_vectors.py` | Executable generator and checker. Holds the encoding rules and the scenario catalogue | No |
| `tof_cliff_contract.h` | Generated production constants, layouts and the status classification | **Yes** — `src/tof_cliff_packer.hpp` and `src/tof_can_ids.hpp` include it, and `docs/can` is on the product include path. Being included is not the same as every constant reaching the image; the linker decides that |
| `tof_cliff_contract_vectors.h` | Generated C++ conformance vectors | Tests only |
| `tof_cliff_layout_vectors.json` | Language-neutral vectors for review and tooling | No |

### ToF grid (`0x214` data, `0x215` health)

| File | Role | Used by production code |
| --- | --- | --- |
| `tof_can_wire_contract.md` | Human-readable normative input | No |
| `gen_golden_vectors.py` | Executable generator and checker | No |
| `tof_contract_vectors.h` | Generated C++ conformance vectors | Tests only |
| `tof_grid_golden_vectors.json` | Language-neutral vectors for review and tooling | No |

**The grid contract has no generated production header.** Its constants live in hand-written source.
Do not assume a change that works for one contract applies to the other.

## Generation model

```
tof_cliff_wire_contract.md ─┐
                            ├─ gen_cliff_golden_vectors.py ─┬─ tof_cliff_contract.h
generator source ───────────┘                               ├─ tof_cliff_contract_vectors.h
                                                            └─ tof_cliff_layout_vectors.json
```

**What the generator does and does not read.** The cliff generator **parses two things from the
Markdown**: the contract's version line, and the ordered verdict tables in *Decoder verdicts and
precedence*, which it compares against its own evaluation order. It **hashes the complete file**.
Everything else — identifiers, field layouts, the status classification, the event vocabulary — lives
in the Python, not in the Markdown. `--check` verifies generated-artefact drift and the parity checks
that are explicitly implemented; it does **not** infer the meaning of arbitrary prose.

That is worth stating plainly because the arrangement invites the opposite assumption. The contract
and the generator are two descriptions of the same rules, and the parity checks exist to stop them
drifting where a check can be written. Where no check exists, the Markdown and the Python are kept in
step by review.

## Identity

Three values are stamped into every generated **cliff** artefact; the grid artefacts carry their own
version and contract SHA and have no artefact-set id. Their current values are printed by `--check`
and are embedded in the artefacts; they are deliberately not repeated here, because a README is not a
pin and nothing would check it.

| Value | How it is derived |
| --- | --- |
| Contract version | read from the contract's version line |
| Contract SHA | SHA-256 of the contract Markdown file's bytes |
| Artefact-set ID | SHA-256 of the contract Markdown **and** the generator source |

The artefact-set ID covers the generator because the generator can change what it emits while the
contract text stands still. A contract SHA alone would not show that.

## Editing workflow

```sh
python3 docs/can/gen_cliff_golden_vectors.py --check
python3 docs/can/gen_cliff_golden_vectors.py --emit
```

- Change the contract, the generator, or both.
- Advance the contract revision for a normative change.
- Run `--emit`.
- Run `--check`.
- Update the firmware pins.
- Re-vendor into LexxAuto's `scbdriver` package — `src/LexxAuto-Driver/scbdriver/src/vendor/` and
  `src/LexxAuto-Driver/scbdriver/test/vendor/` — and compare the copies byte-for-byte.
- Update the pins there and run the tests in both repositories.

**Never hand-edit a generated header or JSON file.** They carry a "GENERATED FILE" banner and the
identity stamps; an edit survives until the next `--emit` silently reverts it.

## Design rationale

Architectural rationale, alternatives and cross-layer design discussion live in Notion:
[Design doc for hanging object and cliff detection in dasher environment](https://app.notion.com/p/2cda91d8f6158078a471e9b14c8d4a05),
whose [L4 — CAN wire protocol](https://app.notion.com/p/3b2a91d8f61581788558e171745f35e5) layer owns
these two contracts. The **normative wire rules needed to reproduce or validate the implementation
stay in this repository**, which is why these contract files are here rather than there.

Where the cliff contract says "is in the design notes", it means one of these layer pages:

- Revision history, and the CAN identifier allocation history, are in
  [L4 — CAN wire protocol](https://app.notion.com/p/3b2a91d8f61581788558e171745f35e5), sections 9 and 10.
- Why a read-back does not establish epoch-store durability, and the provisional bus-load estimate, are in
  [L2-L3 — SCB firmware and scheduling](https://app.notion.com/p/3b2a91d8f615818b8012cc87c6d86f9c).
- Why the legacy UART path cannot carry cliff, and why a cycle count is not a timing bound, are in
  [L5 — ROS nodes and data paths](https://app.notion.com/p/3b2a91d8f61581fdbdbddc9b9f58a6a2).
- The enable-clock evidence and the outstanding hardware checks are in
  [L0-L1 — Sensor bus, harness and addressing](https://app.notion.com/p/3b2a91d8f61581b9960fd55c7421d5c7),
  as INC-L01-002.
