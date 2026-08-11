# Cliff ToF CAN wire contract (AMRSW-2994)

Contract version: **draft-2026-08-11a**
Status: **provisional draft, NOT frozen.** The byte layouts, encodings and state rules below are
written to be implementable as they stand, but three classes of content are deliberately unresolved
and are listed in *Open decisions*: the health CAN identifier, the ROS-facing topic and message
shape, and every timing value. **No implementation may be released against this version**, and no
golden vectors exist yet.

This document is the single source of truth shared by two repositories:

- `LexxHard-SensorControlBoard-Firmware` — the **packer** (SCB firmware cliff ToF producer)
- `LexxHard-SCBDriver` — the **decoder** and ROS publisher

It is a **separate contract** from `tof_can_wire_contract.md`. It shares that contract's *mechanism*
— contract text under version control, golden vectors generated from it, both sides pinning the
SHA-256 of the contract file — and shares **none** of its byte layout. Nothing in this document
changes the grid contract, its version, its SHA or its vectors.

## Scope, and the one property that makes this contract different

Covers the transport of four **VL53L4CX single-point ranges** for cliff detection, from the SCB to
`SCBDriver`, plus the health channel that makes the transport trustworthy. The 8x8 grid transport for
the two VL53L7CX hanging-detection sensors is specified in `tof_can_wire_contract.md` and is out of
scope here.

**The two features fail in opposite directions, on the same bus, from the same firmware.** Missing or
untrustworthy grid data degrades to LiDAR-only and the robot keeps driving. Missing or untrustworthy
cliff data means *there may be a cliff* and the robot must stop. Every rule below follows from that,
and **no reasoning may be transplanted from the grid contract into this one.** In particular the grid
contract's forward-compatible "reserved means ignored" rule is deliberately reversed here (see
*Reserved fields*).

## Transport

CAN classic, 11-bit identifiers, on **CAN2 at 1 Mbit/s** (the SCB-to-IPC bus).

| Constant | Meaning | Value |
| --- | --- | --- |
| `TOF_CLIFF_MEAS_ID` | one measurement frame per sensor per completed measurement | `0x216` |
| `TOF_CLIFF_HEALTH_ID` | chain-level health snapshot, published periodically | **unallocated** — see *Open decisions* |

`0x216` was reserved for this purpose on 2026-08-06 under the same team-authorized self-assignment
that allocated `0x214`/`0x215`. This contract is what un-reserves it: until this document is frozen,
no filter or handler may claim `0x216` either.

Two identifiers rather than one because measurement and health must **dispatch independently**, and
because the lower identifier wins arbitration, so a safety-relevant range never queues behind a
diagnostic snapshot. That ordering is a convenience, not a guarantee: **both frames are
safety-relevant**, and the health frame's period is a specified value, not a best effort. There is no
ordering guarantee between the two identifiers, and the decoder must accept any interleaving.

Sensor identity travels **in the payload**, not in the identifier. Four identifiers would push a
hardware detail into the filter table and make re-ordering boards a bus-level change.

## Source identity

`source_id` is a **stable logical identifier**, not the chain position. The firmware owns the
`chain_position -> source_id` mapping and is the only component that knows the physical topology.

| `source_id` | Intended role |
| --- | --- |
| 0 | `cliff_front_left` |
| 1 | `cliff_rear_left` |
| 2 | `cliff_rear_right` |
| 3 | `cliff_front_right` |

Only 0-3 exist; any other value makes the frame malformed.

**These are intended roles, not a verified runtime mapping.** The role names above become meaningful
only while `mapping_state == PROVEN`. Under the hardware installed today they never do: one enable
clock pulse enables two adjacent carriers, so two identical VL53L4CX can sit on the factory-default
address at once and a single address assignment moves both, undetectably. Consequently a correct
implementation of this contract publishes **no role-named data at all** today. That is the intended
behaviour, not a defect.

`chain_position` appears only in the health frame's `failing_chain_position`, for diagnostics.
Nothing in the decoder may branch on it. **The two numbering spaces are different**: `source_id` is
0-3 over the cliff sensors, `chain_position` is 1-6 over the whole six-board chain including the two
grid boards — a chain fault at position 2 is a grid board and can still invalidate the cliff mapping.

## Measurement frame (`TOF_CLIFF_MEAS_ID`)

DLC 8, always. One frame is one completed measurement of one sensor. No multi-frame assembly.

```
byte 0 : frame_type << 4 | source_id   (frame_type = 0x1; source_id 0-3)
byte 1 : mapping_epoch                 (uint8, wraps 255 -> 0)
byte 2 : sample_seq                    (uint8, per source, wraps 255 -> 0)
byte 3 : range_mm[15:8]                (big-endian, as in the grid contract)
byte 4 : range_mm[7:0]
byte 5 : range_status                  (the raw ULD status byte, transmitted unchanged)
byte 6 : reserved, MUST be 0 on transmit and MUST be rejected if non-zero
byte 7 : reserved, MUST be 0 on transmit and MUST be rejected if non-zero
```

- **`range_mm` is unsigned millimetres, big-endian. `0xFFFF` is the invalid / no-target sentinel**,
  and it is deliberately at the **far** end of the range. The maximum valid distance is 65534 mm.
- `range_status` is passed through unchanged. The firmware never replaces it with a derived code.
- `frame_type` exists even though the identifiers are already distinct, and the receiver MUST reject
  a frame whose `frame_type` does not match the identifier it arrived on. It costs four bits and
  catches a mis-routed filter, which is otherwise a silent mis-decode.

### Why the sentinel is at the far end

An invalid reading must never be decodable as "the floor is right there": that reads as *no cliff*
and lets the robot drive into one. The rule is written so that it holds even for a careless
implementation: **even a decoder that ignores the status byte entirely must not be able to read an
invalid measurement as a near floor.**

### `sample_seq` proves novelty, never age

`sample_seq` increments **only** when a new measurement completes, per source. It proves whether a
new sample arrived and whether frames were lost, repeated or wrapped. **It cannot establish the age
of a sample**, because a stalled sensor and a slow sensor produce the same counter. Age is a
receive-side fact: the decoder holds, per source, the time it last saw a *new* `sample_seq`, and
freshness is measured from that. Deriving age from sequence arithmetic is forbidden. If a capture
timestamp is ever required, add an SCB monotonic tick to the frame rather than reconstructing one.

## Health frame (`TOF_CLIFF_HEALTH_ID`)

DLC 8, always. One frame describes **the whole cliff subsystem**, not one sensor.

```
byte 0 : frame_type << 4 | 0x0         (frame_type = 0x2; low nibble reserved, MUST be 0)
byte 1 : mapping_epoch
byte 2 : health_seq                    (uint8, wraps 255 -> 0)
byte 3 : mapping_state << 4 | chain_fault
byte 4 : enumerated_mask << 4 | model_verified_mask
byte 5 : transport_read_completed_mask << 4 | sensor_fault_mask
byte 6 : failing_chain_position        (1-6, or 0xFF for none)
byte 7 : reserved, MUST be 0 on transmit and MUST be rejected if non-zero
```

`mapping_state`, low nibble of byte 3 is `chain_fault`:

| `mapping_state` | Meaning |
| --- | --- |
| `0x0` `UNKNOWN` | never proven since the last enumeration attempt |
| `0x1` `PROVEN` | the position-to-`source_id` mapping has been proven by the evidence this contract requires |
| `0x2` `LOST` | was proven, then a sensor was lost at runtime |
| `0x3` `FAULT` | a fault prevents any trustworthy mapping |
| `0x4`-`0xF` | malformed |

`chain_fault` is a 4-bit flag field: bit 0 chain length differs from the configured expectation,
bit 1 enumeration was frozen because a non-tail position produced no new device, bit 2 a
transport-level bus fault was seen, bit 3 reserved and MUST be 0.

### The four masks

Each mask is 4 bits, **bit `k` is `source_id` `k`**. They are *not* keyed by chain position.

| Mask | What it observes |
| --- | --- |
| `enumerated_mask` | this source enumerated to its own address |
| `model_verified_mask` | this source returned the expected model ID |
| `transport_read_completed_mask` | this source completed an I2C read in the cycle this snapshot describes |
| `sensor_fault_mask` | this source's most recent read classified as `SENSOR_FAULT` |

**`transport_read_completed_mask` is named for what it actually observes.** An I2C read can complete
successfully and still return a ULD `SENSOR_FAULT` status; a field called "OK" would collapse two
different facts into one bit. Whether a *usable range* was produced is carried by the measurement
frame's status and range encoding, and by `sensor_fault_mask`. A consumer that needs "this sensor is
delivering usable data" must look at both.

**Every mask describes exactly one acquisition cycle** — the most recently completed one. One cycle
is one pass in which the firmware attempts a read of each of the four sensors. A mask cannot be
interpreted without knowing which cycle it belongs to, which is why the cycle period is a required
timing value below.

**While `mapping_state != PROVEN` the masks are diagnostic only.** They are keyed by `source_id`, and
`source_id` has no proven physical meaning until the mapping is proven, so a consumer must not
attribute a masked bit to a physical corner in that state.

### `health_seq`, and why it exists

`health_seq` increments **only** when a new snapshot is produced. A repeated value MUST NOT refresh
health freshness on the receiving side. Without this, a firmware that stops updating but keeps
transmitting its last snapshot is indistinguishable from a healthy one, and a stale `PROVEN` would
keep authorising role data indefinitely.

## Reserved fields

Every field marked reserved MUST be zero on transmit, and the receiver MUST **reject** a frame whose
reserved fields are non-zero.

This is the opposite of the grid contract, which makes its reserved fields *ignored* so that a
firmware populating them later stays compatible with an older driver. The reversal is deliberate: on
this path a rejected frame produces no measurement, which expires the consumer's timeout and stops
the robot — the safe direction — whereas ignoring an unrecognised field means acting on a frame whose
meaning has changed. The cost is explicit and accepted: **fields cannot be added to this format
compatibly.** Any new field is a contract version bump, new vectors, and a coordinated update of both
implementations.

## Firmware obligations

**A measurement frame is produced only by a complete, successful read.** A successful read that found
no target is data, not a failure: it is transmitted with `0xFFFF` and its real status.

- An I2C read failure produces **no** measurement frame, and is reported through health only.
- Old values are never re-sent. The firmware never fabricates `0 mm`, never substitutes a previous
  distance, and never computes a cliff verdict, a temporal filter, or a left/right reduction.
- **On losing the mapping the firmware stops producing measurement frames first, then publishes
  health with `LOST` or `FAULT`.** The dangerous case is data outliving the mapping that gave it
  meaning, so the order is normative.
- Measurement frames MAY be transmitted while `mapping_state == UNKNOWN`, so that a chain can be
  commissioned at all; they MUST NOT be transmitted while `LOST` or `FAULT`. What protects production
  in the `UNKNOWN` case is the decoder's role gate, not the absence of frames. *(This resolves an
  ambiguity in the design notes and is listed under Open decisions for confirmation.)*
- `mapping_state == PROVEN` is not an optimistic verdict reached because enumeration returned without
  error. The contract owes a definition of what evidence proves it; **under the hardware installed
  today no such evidence exists**, so a conforming firmware never reaches `PROVEN`.
- Re-enumeration is permitted only once the robot is already safely stopped. A new `mapping_epoch` may
  be issued, and measurements may resume, only after the mapping has been fully proven again. While
  the two-board-wide enable window exists, any implementation that reaches `PROVEN` automatically has
  a bug.

### Two constraints the acquisition path inherits from the hardware

- **The four VL53L4CX must stay enabled continuously and be addressed individually.** On the L4
  carriers the enable line is reset-class: dropping it returns the device to the default `0x29`. A
  producer that selected sensors by walking the enable chain would destroy all four assigned
  addresses on every pass, forcing re-enumeration, an epoch bump and a safe stop.
- **One scheduler serves all six boards.** The four cliff sensors and the two grid sensors share the
  differential-I2C bus, and the requirement to trim the grid read exists precisely to protect the
  cliff rate. An acquisition loop written for the four cliff sensors alone is likely to be one the
  grid read cannot later be fitted into.

## Status classification

The contract owns exactly one table mapping the raw ULD status to a class:

`raw status -> VALID_RANGE / NO_TARGET / SENSOR_FAULT`

The rule that reconciles raw passthrough with the firmware using that table — these read as
contradictory and must be stated together:

> The raw status byte is transmitted unchanged. Firmware uses the contract-owned classification table
> only to select the range encoding and publication outcome; it never replaces the raw status with a
> derived code.

The three classes have frozen consequences, so the encoding cannot be chosen per implementation:

| Class | `range_mm` | Health | Note |
| --- | --- | --- | --- |
| `VALID_RANGE` | a finite distance; **`0xFFFF` is forbidden** | unaffected | the only case that may yield a finite range |
| `NO_TARGET` | **MUST be `0xFFFF`** | unaffected | a successful read that found nothing is data |
| `SENSOR_FAULT` | **`0xFFFF`**, raw status preserved | `sensor_fault_mask` bit set | the range also lands on the safe side, so a consumer that only reads the range still errs toward stopping |

**A frame whose status class and range sentinel contradict each other MUST be rejected, and the
receiver MUST enter a fault state.** One of the two fields is wrong, there is no safe way to guess
which, and the contradiction is itself evidence of a defect upstream. The same applies to a
`sensor_fault_mask` bit that contradicts the status class of that source's most recent measurement.

Both the packer and the decoder take their classification from the one generated table and its
vectors. Firmware-level facts — read failed, sensor absent, configuration mismatch — live in health
and must not re-express or contradict the ULD status.

The concrete status-to-class rows are **not yet filled in**: they must be transcribed from the
VL53L4CX ULD's `RangeStatus` definitions when the driver is ported, not guessed from the VL53L7CX
table. Until then this section defines the shape and the consequences, not the mapping.

## Decoder and ROS publisher obligations

### Two-level heartbeat

The one failure this whole contract exists to eliminate is silence that looks like health. It is
closed at two levels, and both are normative:

- **Firmware:** once the cliff subsystem is enabled, the health frame is published periodically
  **unconditionally** — including with zero sensors enumerated, `mapping_state == UNKNOWN`, or a chain
  fault. Health silence therefore means the producer is gone, never "nothing to report".
- **Driver:** once the cliff feature is enabled, the ROS safety-health topic is published
  periodically **from node startup**, beginning as `NOT_READY`, even if no CAN frame ever arrives. A
  consumer can therefore distinguish "the SCB is not sending health" from "the driver does not exist",
  which subscribing to an absent topic cannot.

Only the four role measurements are gated. Health is never gated.

### Validation before trust

Reject, count and report: DLC other than 8; `frame_type` not matching the arrival identifier;
`source_id` outside 0-3; any non-zero reserved field; `mapping_state` outside 0x0-0x3;
`failing_chain_position` outside 1-6 and not `0xFF`; a status/sentinel contradiction; and a
measurement whose `mapping_epoch` does not match the current authorising health snapshot.

A frame that is malformed but attributable to a source still updates that source's *last frame seen*
time before being rejected, because it is evidence the transport is alive. A frame that cannot be
attributed to any source touches no per-source tracker.

### Readiness, and why a new epoch is not immediately ready

The decoder maintains, per source, the last accepted range and the receive time of the last **new**
`sample_seq`; and, globally, the current epoch, the last **new** `health_seq` time, and a readiness
state.

On receiving a health snapshot with `PROVEN` and an epoch different from the current one, the decoder
MUST:

- discard the four cached ranges and their freshness timestamps from the previous epoch
- reject any measurement of the new epoch that arrived **before** its authorising health snapshot
- withhold overall `READY` until all four sources have produced a **fresh** measurement in that same
  epoch
- only then publish `READY`

`READY` is lost immediately if the epoch changes, if `mapping_state` leaves `PROVEN`, if health goes
stale, or if any one source goes stale. There is no partial readiness: three fresh sources out of
four is `NOT_READY`, because the missing one is exactly where an undetected drop would be.

An epoch that advances while `mapping_state` stays `PROVEN` is treated identically to any other epoch
change. It is not a continuation.

### Publishing

While `READY`, the decoder publishes the four role measurements, converting millimetres to metres at
this boundary. It does **not** decide cliffs, filter in time, reduce four channels into two, or turn
an invalid or no-target reading into `0.0`.

While not `READY`:

- **no role-named measurement is published at all** — not one of the four. Falling back to
  position-numbered names on the production interface is also forbidden, because such a topic is too
  easily wired into something that treats it as a role
- position-numbered raw values MAY be published in a **commissioning namespace** that is plainly
  outside the production interface, for bring-up only, and MUST NOT be enabled by a production launch
- the ROS safety-health topic continues to publish, carrying `NOT_READY` or `FAULT` and the reason
- `DiagnosticArray` is for the operator. It MUST NOT be the transport of any safety handshake

### Configuration failure is contained to the cliff subsystem

Two configurations are impossible and MUST fail rather than be silently resolved: the cliff feature
enabled together with `tof_transport=legacy_uart`, and any required cliff timing parameter left
unset. There is deliberately **no runnable default** for the timeout parameters.

The failure MUST NOT take down the rest of `SCBDriver`. The cliff subsystem enters `CONFIG_ERROR`:

- the cliff CAN filter entries are not installed
- no role measurement is published
- the ROS safety-health topic publishes `FAULT` continuously, so the consumer holds its stop
- every other `SCBDriver` function continues to run

### The legacy UART path is not a cliff transport

`receiver_tof` already carries a legacy UART path for the L4 sensors: it recognises **`sensor_id` 0
and 1 only**, publishes `/sensor_set/tof_front` and `/sensor_set/tof_rear` as variable-length
`Float32MultiArray` with `-1.0` for no target, and carries no raw status, no epoch and no health. It
is structurally incapable of expressing four independent roles.

Therefore: cliff is **unavailable**, not degraded, in `legacy_uart` mode; the legacy topics MUST NOT
be remapped into the cliff namespace, and the cliff production topic names are deliberately chosen so
that the established `/sensor_set/X -> /global/system/X` remap convention cannot connect them; and
this transport MUST NOT feed or bridge into `safety_manager/downward`, which stays dead for this
feature. Two decision paths would mean two threshold sources and two ways to command a stop.

## Timing values — required, and not frozen

The following are **required fields of this contract that have no values yet**. They cannot be
established offline: they are outputs of the L4 acquisition thread and of the six-board schedule
measured on real hardware.

| Symbol | Meaning |
| --- | --- |
| `T_meas_nominal` | nominal measurement period, per sensor |
| `T_meas_max_gap` | maximum tolerable gap between consecutive measurements of one sensor |
| `T_cycle_nominal` | nominal acquisition-cycle period, which is what a health mask describes |
| `T_health_nominal` | nominal health snapshot period |
| `T_health_max_gap` | maximum tolerable gap between consecutive **new** `health_seq` values |
| `T_skew_max` | worst-case sampling phase skew across the four sensors |

Two rules constrain the eventual numbers rather than the schedule:

- **The ROS-side timeouts are derived from these values, never guessed.** A consumer timeout chosen
  independently of the producer's real period is either a nuisance stop or a missed cliff.
- **`T_health_max_gap` must be strictly smaller than the consumer's health timeout, with margin**,
  because the health frame is chain-level: one dropped frame must not by itself trip a stop.

## Golden vectors

Vectors do **not** exist yet; generating them is the step after the ROS-side interface review. They
will come from a sibling generator, `gen_cliff_golden_vectors.py`, emitting a JSON file and a
dependency-free C++ header, both carrying the SHA-256 of **this** file, pinned independently by the
firmware packer test and the driver decoder test. The grid contract's generator, vectors and SHA are
untouched.

Beyond the happy path the vectors MUST cover at least:

- each status class, and the `0xFFFF` sentinel with `NO_TARGET` and with `SENSOR_FAULT`
- one source silent; all four silent
- `mapping_state` `UNKNOWN`, `PROVEN`, `LOST`, and recovery back to `PROVEN`
- health stopping while measurements continue, and `health_seq` repeated
- a measurement arriving before any health snapshot at all — cold start
- a new epoch with `PROVEN` where only some of the four sources have refreshed
- a measurement of the **previous** epoch arriving during an epoch change
- `mapping_epoch` wrap 255 -> 0, and `sample_seq` repeated, skipped and wrapped
- measurement and health epochs disagreeing
- a status class contradicting the range sentinel, and `sensor_fault_mask` contradicting the status
- non-zero reserved fields, DLC other than 8, `frame_type` mismatched to its identifier,
  `source_id` out of range, `mapping_state` out of range, `failing_chain_position` out of range
- a `CONFIG_ERROR` start, asserting that no role topic is ever advertised and that safety-health
  publishes `FAULT`

As in the grid contract, each scenario declares the **complete multiset** of events it must produce,
so an implementation cannot pass by emitting the right event alongside wrong ones. These are the
cases where an implementation silently picks a fail direction; if the vectors only cover the happy
path, the two sides will each pick one and they will not pick the same one.

## Open decisions

Nothing in this section may be resolved unilaterally, and the contract cannot be frozen while any of
it is open.

- **The health CAN identifier.** `0x217` is a *candidate only*. The 2026-08-06 sweep is stale — three
  rows were added to the table since — so allocation requires a fresh sweep of both repositories plus
  a live capture, then a self-assignment recorded here and in the team's CAN ID register.
- **The ROS interface shape**, which belongs to the cliff decision owner as much as to us. Proposed
  and not frozen: four `sensor_msgs/Range` topics under `/global/system/cliff_tof/` named for the
  roles, plus `/global/system/cliff_tof/health`. `Range` is preferred because it already carries a
  stamped header and range bounds; the alternative is one custom atomic stamped message per sample so
  that range and status cannot arrive out of step. Also open: **which message type** the typed
  safety-health topic uses, and whether `+Inf` is adopted as the out-of-range convention — it happens
  to sit on the fail-safe side, but no `Range` consumer is obliged to read it that way, so it must be
  written into the interface specification and implemented on the consumer side rather than assumed.
- **All timing values** above, and with them the consumer timeouts.
- **The status-to-class rows**, to be transcribed from the VL53L4CX ULD rather than guessed.
- **What evidence proves `mapping_state == PROVEN`**, which cannot be settled until the enable-chain
  defect is fixed.
- **Whether measurements may be transmitted while `UNKNOWN`**, as this draft currently permits.

## Freeze procedure

This draft becomes normative in two steps, not one.

First freeze, after the interface review and the identifier allocation: fill in the ROS interface,
the health identifier and the status rows; generate the vectors; both repositories pin the SHA. The
timing symbols stay explicitly marked as provisional, and the version keeps a marker that forbids
release.

Second freeze, after the acquisition thread exists and the six-board schedule has been measured:
write in the measured timing values, remove the draft marker, regenerate the vectors, and re-pin both
sides in the same round. Only then may anything be released against this contract.
