# Cliff ToF CAN wire contract (AMRSW-2994)

Contract version: **draft-2026-08-11d**
Status: **provisional draft, NOT frozen.** The byte layouts, encodings and state rules below are
written to be implementable as they stand, but four classes of content are deliberately unresolved and
are listed in *Open decisions*: the health CAN identifier, the ROS message type for safety health, the
validation column of the status classification, and every timing value. **No implementation may be
released against this version**, and no golden vectors exist yet.

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
and **no reasoning may be transplanted from the grid contract into this one.** Two places where this
contract deliberately does the opposite of the grid contract, each with its reasoning at the point of
use: reserved fields are rejected rather than ignored, and the health frame is a safety frame rather
than a diagnostic one.

## Transport

CAN classic, 11-bit identifiers, on **CAN2 at 1 Mbit/s** (the SCB-to-IPC bus).

| Constant | Meaning | Value |
| --- | --- | --- |
| `TOF_CLIFF_MEAS_ID` | one measurement frame per sensor per completed read | `0x216` |
| `TOF_CLIFF_HEALTH_ID` | one health frame per acquisition cycle, and periodically regardless | **unallocated** — see *Open decisions* |

`0x216` was reserved for this purpose on 2026-08-06 under the same team-authorized self-assignment
that allocated `0x214`/`0x215`. This contract is what un-reserves it: until this document is frozen,
no filter or handler may claim `0x216` either.

`0x217` is the candidate for health. An offline sweep on 2026-08-11 found it unused in the firmware
repository on `main` and on all three ToF branches, in `SCBDriver`, and in `LexxAuto` and
`lexxauto_msgs`; the SCB block is contiguous `0x200`-`0x213` on `main` and extends to `0x216` with the
grid allocation. **That is not an allocation.** A live bus capture is still required, because a device
can transmit an identifier that appears in no source tree we hold, and the allocation must then be
recorded here and in the team's CAN ID register.

Two identifiers rather than one because measurement and health must **dispatch independently**.
Measurement takes the lower identifier because there are four of it per cycle against one health
frame, so putting the more frequent traffic first minimises queueing overall.

**That is not an argument that health is less important.** Both frames are safety-relevant: the
health channel is what turns "no measurement arrived" from an ambiguity into a decision, and without
it the data channel cannot be trusted at all. The arbitration choice therefore carries an obligation:
**`T_health_delivery_max`, the worst-case health latency under full bus load, must be shown by
measurement or analysis to stay inside `T_health_max_gap`.** If it cannot, health takes the lower identifier instead. It is not
acceptable to justify the ordering by calling health diagnostic.

Sensor identity travels **in the payload**, not in the identifier. Four identifiers would push a
hardware detail into the filter table and make re-ordering boards a bus-level change.

### Bus load, provisional

Five frames per acquisition cycle: four measurements and one health. At a nominal 20 Hz cycle that is
100 frames/s of 8-byte payload, roughly **1.3%** of CAN2 at 1 Mbit/s including stuff bits and
inter-frame space, on top of the grid path's 2.24%. The figure is provisional because the cycle rate
is one of the open timing values.

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
address at once and a single address assignment moves both, undetectably. Consequently a conforming
implementation publishes **no role-named data at all** today, and — since measurement frames are
forbidden outside `PROVEN` — `0x216` carries **no traffic at all**. The only cliff traffic on the bus
is the health heartbeat, reporting `UNKNOWN`. That is the intended behaviour, not a defect, and it is
what a bring-up engineer should expect to see.

`chain_position` appears only in the health frame's `failing_chain_position`, for diagnostics.
Nothing in the decoder may branch on it. **The two numbering spaces are different**: `source_id` is
0-3 over the cliff sensors, `chain_position` is 1-6 over the whole six-board chain including the two
grid boards — a chain fault at position 2 is a grid board and can still invalidate the cliff mapping.

## The acquisition cycle is the unit of correlation

One **cycle** is one pass in which the firmware attempts a read of each of the four sensors. The cycle
is the only structure that lets a health snapshot be matched to the measurements it describes, and
without it none of the cross-checks in this contract are decidable.

- `cycle_seq` is a uint8 carried by **both** frame types. It is monotonic within an epoch, starts at
  `0` for the first cycle of a new `mapping_epoch`, increments by one per completed cycle and wraps
  255 -> 0.
- **At most one measurement frame exists per `(source_id, mapping_epoch, cycle_seq)`.** A second one
  is a conflict, not a retransmission to be tolerated: two different values for one sensor in one
  cycle mean a firmware fault or two streams mixing, and there is no basis for choosing one.
- A cycle in which a sensor's read failed simply has no frame for that sensor. A cycle therefore
  carries between zero and four measurement frames.
- The firmware transmits the cycle's health frame **after** that cycle's measurements. That is the
  intent, not a guarantee: the two identifiers arbitrate independently, so the decoder MUST accept
  any interleaving and correlate by `(mapping_epoch, cycle_seq)` rather than by arrival order.
- `health_seq` and `cycle_seq` have **different jobs and must not be tied together**. `health_seq`
  increments on every snapshot and is the liveness counter; `cycle_seq` increments only when a cycle
  actually completes. While no acquisition is running — enabled but not `PROVEN`, for instance — health
  keeps flowing with `health_seq` advancing and `cycle_valid` clear.
- **The firmware MUST issue a new `mapping_epoch` whenever the cliff subsystem restarts**, so a cycle
  counter never resumes mid-sequence. This is what keeps `cycle_seq` unambiguous across a reset, and it
  removes the main way an old frame could alias onto a new cycle.

### `cycle_valid`, and why cycle 0 needs distinguishing

A health frame is published even when no acquisition is running, so most of its life is spent
describing no cycle at all. `cycle_seq = 0` cannot mean both "no cycle has completed" and "the first
cycle of this epoch", or the first real cycle of an epoch would be indistinguishable from a heartbeat —
and, with the retirement rules below, could be treated as already retired.

`cycle_valid` (health `flags` bit 3) resolves it:

- **clear** — this is a state heartbeat only. It refreshes health liveness and carries `mapping_state`,
  the `flags`, `failing_chain_position` and the enumeration masks, but it describes **no cycle**. The
  decoder MUST ignore `cycle_seq` for correlation, MUST NOT retire or open any cycle on it, and MUST
  NOT read the per-cycle masks. The firmware MUST transmit `cycle_seq = 0` and both per-cycle masks as
  zero in this state.
- **set** — `cycle_seq` names a completed cycle, and the per-cycle masks describe exactly that cycle.

Note which masks this governs: `sample_produced_mask` and `sensor_fault_mask` are **per cycle**;
`enumerated_mask` and `model_verified_mask` describe the **last enumeration attempt** and stay
meaningful with `cycle_valid` clear, which is precisely what makes a heartbeat during `UNKNOWN` useful.

### Cycle assembly and retirement

The decoder holds at most **two open cycle slots** per epoch. A slot opens on the first frame carrying
its `(mapping_epoch, cycle_seq)`, measurement or health, and records that arrival time.

- A slot **completes** when its `cycle_valid` health frame has arrived and every measurement its
  `sample_produced_mask` implies has arrived.
- A slot **expires** after `T_cycle_assembly` from its first frame. On expiry it is retired and counted,
  and the outcome depends on what was missing:
  - its health arrived with `cycle_valid` and `sample_produced_mask` implied measurements that never
    came → **protocol FAULT**. Health asserted data that does not exist, which means the producer and
    the wire disagree
  - its health never arrived → the measurements are discarded unaccepted, and the event is reported as
    degraded. One lost health frame is a transport event, not a producer defect; persistent loss is
    caught by `T_health_max_gap` instead
- A slot is also retired when a **third** cycle opens: only the newest two are ever open.
- **A retired cycle is final.** A later frame carrying a retired `(epoch, cycle_seq)` is counted and
  dropped: it cannot reopen the slot, cannot refresh any freshness timestamp and cannot contribute to
  `READY`.
- An epoch change retires every cycle of the previous epoch immediately.

`T_cycle_assembly` is not a free parameter. It must exceed the worst-case intra-cycle skew plus the
worst-case health latency — otherwise cycles expire while their own frames are still legitimately in
flight — and it must stay below `T_meas_max_gap`, since a slot outliving the freshness budget of the
data in it serves no purpose:

`T_skew_max + worst-case health latency  <  T_cycle_assembly  <  T_meas_max_gap`

### Comparing cycle numbers, and the wrap

`cycle_seq` is 8 bits, so comparisons are **modulo** comparisons, never magnitude comparisons. The
decoder computes a signed difference against the current anchor:

`delta = (int8_t)(incoming_cycle_seq - anchor_cycle_seq)`

| `delta` | Meaning |
| --- | --- |
| `0` | the anchor cycle |
| `-1` | the previous, still-open cycle |
| `1` to `N_cycle_advance_max` | a new cycle: open a slot, and retire whatever falls out of the newest two |
| anything else | **implausible**: protocol FAULT |

Two rules make that table sound:

- **Only a `cycle_valid` health frame may move the anchor.** A measurement for a plausible new cycle
  may open a slot ahead of the anchor — that is the normal case, since measurements are transmitted
  before their health frame — but it is never *accepted* until that cycle's health arrives. A
  measurement can therefore never drag the anchor forward on its own.
- **An implausible `delta` resynchronises rather than guesses.** Everything is retired, the anchor is
  re-taken from the next `cycle_valid` health frame, and `READY` is withheld until a clean cycle
  completes. The reason is reported distinctly from a malformed frame: a jump beyond
  `N_cycle_advance_max` is indistinguishable from a wrap-aliased frame, and there is no safe way to
  tell which it is.

That closes the wrap: an old frame can only alias onto a live cycle if it is delayed by a full 256
cycles, which is not a CAN transport phenomenon but a producer restart or a replayed capture. Both are
covered — a restart must bump the epoch, and a replayed frame must still find an authorising health
frame with the same epoch *and* cycle, where it collides with the real measurement for that triple and
is caught as a conflict.

## Measurement frame (`TOF_CLIFF_MEAS_ID`)

DLC 8, always. One frame is one completed read of one sensor. No multi-frame assembly.

```
byte 0 : frame_type << 4 | source_id   (frame_type = 0x1; source_id 0-3)
byte 1 : mapping_epoch                 (uint8, wraps 255 -> 0)
byte 2 : cycle_seq                     (uint8, shared with the health frame, wraps 255 -> 0)
byte 3 : range_mm[15:8]                (big-endian, as in the grid contract)
byte 4 : range_mm[7:0]
byte 5 : range_status                  (the raw ULD status byte, transmitted unchanged)
byte 6 : target_count                  (0-4: the ULD's NumberOfObjectsFound, before reduction)
byte 7 : reserved, MUST be 0 on transmit and MUST be rejected if non-zero
```

- **`range_mm` is unsigned millimetres, big-endian. `0xFFFF` is the invalid / no-target sentinel**,
  deliberately at the **far** end of the range. `65534` is therefore the largest encodable non-sentinel
  value; it is a property of the encoding and says nothing about the VL53L4CX's usable range, which is
  far shorter and depends on mode, target reflectance and ambient light.
- `range_status` is passed through unchanged. The firmware never replaces it with a derived code.
- `frame_type` exists even though the identifiers are distinct, and the receiver MUST reject a frame
  whose `frame_type` does not match the identifier it arrived on. It costs four bits and catches a
  mis-routed filter, which is otherwise a silent mis-decode.
- The measurement frame carries **no protocol version**: it is only ever accepted under an authorising
  health frame, which carries one. That gate is what makes the omission safe, so it must never be
  relaxed.
- `target_count` is the number of targets the ULD reported for this read, **before** the reduction
  below. It does not change the meaning of `range_mm`; it exists so that a multi-target scene is
  diagnosable after the fact instead of being invisible. `target_count > 4` is malformed.
- Byte 7 is the designated space for a future SCB capture tick, if bounding the age of the measurement
  rather than of its arrival ever becomes a requirement. Using it is a version bump.

### Why the sentinel is at the far end, and why a real cliff looks like an invalid read

An invalid reading must never be decodable as "the floor is right there": that reads as *no cliff* and
lets the robot drive into one. The rule is written so that it holds even for a careless
implementation: **even a decoder that ignores the status byte entirely must not be able to read an
invalid measurement as a near floor.**

A consequence that will look like a defect and is not: **a genuine cliff and an unusable read produce
the same code point.** A real drop-off returns nothing within the ranging window, which is
`NO_TARGET`, which is `0xFFFF` — the same value a sensor fault produces. This is deliberate, because
both must stop the robot. What distinguishes them is the **health channel**, which is why health is a
safety frame here. Do not "fix" this by giving faults a distinguishable near value.

A shallower drop-off may instead return a valid, longer range. Both paths must therefore reach a stop,
and where the threshold between floor and drop lies is the consumer's decision, not this contract's.

### One frame carries one range, so the reduction is normative

The VL53L4CX reports **up to four targets** per measurement, each with its own range and status
(`VL53LX_MAX_RANGE_RESULTS = 4`, `NumberOfObjectsFound`, per-target `RangeStatus` and
`RangeMilliMeter`). The wire carries exactly one range and one status, so the firmware must reduce, and
**the reduction is part of this contract rather than an implementation choice.** Two implementations
reducing differently would disagree about the floor while both passing their own tests.

| Targets | Rule |
| --- | --- |
| 0 | status `255`, `range_mm = 0xFFFF`, `target_count = 0` |
| 1 | that target's status, classified by the table below |
| 2-4 | classify every target, then take the **most conservative class present**, in the order `SENSOR_FAULT` > `NO_SAMPLE` > `NO_TARGET` > `VALID_RANGE` |

When the surviving class is `VALID_RANGE` and more than one target qualifies, transmit the **farthest**
of them.

**Farthest, not nearest — and this is the opposite of the grid path.** The hanging-object path takes the
per-zone *minimum*, because there the hazard is something being closer than expected. Here the hazard is
the floor being *farther* than expected, so the conservative choice inverts: a spurious near return from
dust, a wheel edge or crosstalk must never be allowed to mask a real drop behind it. Anyone porting the
reduction from the grid packer will take the minimum and silently disable cliff detection.

Both rules are conservative by construction and both need a rate measured on hardware before the second
freeze, because each can cost availability: any faulty target condemns the whole measurement, and the
farthest-valid rule biases toward reporting a drop.

### `cycle_seq` proves novelty, never age

Correlating by `(source_id, mapping_epoch, cycle_seq)` proves whether a measurement is new, and
whether frames were lost, repeated or wrapped. **It cannot establish the age of a sample**, because a
stalled sensor and a slow sensor produce the same counters. Age is a receive-side fact: the decoder
holds, per source, the time it last accepted a measurement from a *new* cycle, and freshness is
measured from that. Deriving age from sequence arithmetic is forbidden.

## Health frame (`TOF_CLIFF_HEALTH_ID`)

DLC 8, always. One frame describes **the whole cliff subsystem**, not one sensor.

```
byte 0 : frame_type << 4 | protocol_version   (frame_type = 0x2; protocol_version = 0x1)
byte 1 : mapping_epoch
byte 2 : health_seq                    (uint8, wraps 255 -> 0)
byte 3 : mapping_state << 4 | flags
byte 4 : enumerated_mask << 4 | model_verified_mask
byte 5 : sample_produced_mask << 4 | sensor_fault_mask
byte 6 : failing_chain_position        (1-6, or 0xFF for none)
byte 7 : cycle_seq                     (the most recently completed cycle; 0 if none in this epoch)
```

### Protocol version

`protocol_version` is the **runtime** compatibility check, and it exists because the SHA pin cannot
provide one. The pin proves that two source trees were generated from the same contract; it says
nothing about a robot running a firmware from one contract revision against a driver from another,
which is the realistic failure once these images are deployed independently.

- This contract is `protocol_version = 0x1`. **Zero is not a valid version**, so an all-zero byte —
  an uninitialised field, a foreign frame, a mis-routed filter — can never be read as compatible.
- A decoder MUST accept only versions it implements. An unsupported version puts the cliff subsystem
  in protocol FAULT: no role measurement is published, safety health carries the reason, and the
  measurements of that epoch are never accepted, since their authorising frame is unusable.
- The version is in the health frame only. Measurements inherit it through the authorising-health gate.

*The grid contract has the same runtime gap and no version field. That is out of scope here and must
not be silently changed: it needs its own version bump, vector regeneration and re-pin round.*

### Mapping state and flags

| `mapping_state` | Meaning |
| --- | --- |
| `0x0` `UNKNOWN` | never proven since the last enumeration attempt |
| `0x1` `PROVEN` | the position-to-`source_id` mapping has been proven by the evidence this contract requires |
| `0x2` `LOST` | was proven, then a sensor was lost at runtime |
| `0x3` `FAULT` | a fault prevents any trustworthy mapping |
| `0x4`-`0xF` | malformed |

`flags` is the low nibble of byte 3:

| Bit | Meaning |
| --- | --- |
| 0 | chain length differs from the configured expectation |
| 1 | enumeration was frozen because a non-tail position produced no new device |
| 2 | a transport-level bus fault was seen |
| 3 | `cycle_valid` — see above |

Bits 0-2 are the **chain faults**. Where this contract says "no chain fault" it means bits 0-2 all
clear; `cycle_valid` is not a fault and must not be counted as one.

### The four masks

Each mask is 4 bits, **bit `k` is `source_id` `k`**. They are *not* keyed by chain position. Two are
per cycle and two are not, and the difference matters:

| Mask | Scope | What it observes |
| --- | --- | --- |
| `enumerated_mask` | last enumeration | this source enumerated to its own address |
| `model_verified_mask` | last enumeration | this source returned the expected model ID |
| `sample_produced_mask` | the cycle named by `cycle_seq` | this source's read completed **and produced a sample** in that cycle |
| `sensor_fault_mask` | the cycle named by `cycle_seq` | that sample classified as `SENSOR_FAULT` |

**`sample_produced_mask` was called `transport_read_completed_mask` in `draft-2026-08-11b`, and the
rename is deliberate.** A completed I2C read does not always yield a sample: the ULD returns
`SYNCRONISATION_INT` on the first read after starting, and `NONE` when there is no update, and neither
is a measurement (see *Status classification*). Defining the bit as "a read that produced a sample"
keeps both directions of the measurement-presence cross-check exact; defining it as transport success
would make one direction unenforceable.

The distinction the old name protected is preserved: bit clear means no sample at all, bit set with
`sensor_fault_mask` clear means a usable outcome — `VALID_RANGE` or `NO_TARGET` — and bit set with the
fault bit set means a sample exists but the sensor reports it as unusable. What is *not* distinguished
per source any more is an I2C failure from a no-update, which are separated only at chain level by
`flags` bit 2. Splitting them per source would need another field and therefore a version bump.

`sensor_fault_mask` is **per cycle, not latching**. A transient fault disappears from the next
snapshot, so any escalation policy — how many consecutive cycles of fault become a persistent
condition — belongs in the decoder, not in the firmware. Note that unlike a missing sample, a sensor
fault is **not** tolerated for a bounded number of cycles: it costs `READY` on the cycle it appears in.

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
this path a frame acted upon with a changed meaning is worse than a frame refused. The cost is
explicit and accepted — **fields cannot be added to this format compatibly** — and it is what the
`protocol_version` nibble exists to manage: any new field is a version bump, new vectors, and a
coordinated update of both implementations.

Rejection alone is **not** the response to a malformed frame; see *Protocol fault* below.

## Firmware obligations

**A measurement frame is produced only by a complete read that produced a sample, and only while
`mapping_state == PROVEN`.** A read that found no target is data, not a failure: it is transmitted with
`0xFFFF` and its real status. A read that completed without a sample — the two `NO_SAMPLE` statuses —
produces no frame and leaves that source's `sample_produced_mask` bit clear.

- An I2C read failure produces **no** measurement frame, and is reported through health only.
- Old values are never re-sent. The firmware never fabricates `0 mm`, never substitutes a previous
  distance, and never computes a cliff verdict, a temporal filter, or a left/right reduction.
- **On losing the mapping the firmware stops producing measurement frames first, then publishes
  health with `LOST` or `FAULT`.** The dangerous case is data outliving the mapping that gave it
  meaning, so the order is normative.
- **No measurement frame is transmitted while `UNKNOWN`, `LOST` or `FAULT`.** A frame carries only a
  logical `source_id`, which outside `PROVEN` is the firmware's unproven guess rather than a physical
  position — so there is no such thing as trustworthy "position-numbered" production data, and
  emitting it would contradict the rule that measurements resume only once the mapping is proven
  again. Commissioning is served by the `tof_diag` shell, which can enable boards individually and
  read the carrier LEDs, and which is strictly better evidence than an anonymous CAN measurement. If
  commissioning over CAN is ever required, it must use a **distinct frame type or identifier carrying
  an explicit `enumeration_slot`**, and must never be decodable as a production measurement.
- `mapping_state == PROVEN` is not an optimistic verdict reached because enumeration returned without
  error. The contract owes a definition of what evidence proves it; **under the hardware installed
  today no such evidence exists**, so a conforming firmware never reaches `PROVEN` and never transmits
  a measurement frame.
- Re-enumeration is permitted only once the robot is already safely stopped. A new `mapping_epoch` may
  be issued, and measurements may resume, only after the mapping has been fully proven again. While
  the two-board-wide enable window exists, any implementation that reaches `PROVEN` automatically has
  a bug.

### Two constraints the acquisition path inherits from the hardware

- **The four VL53L4CX must stay enabled continuously and be addressed individually.** On the L4
  carriers the enable line is reset-class: dropping it returns the device to the default `0x29`. A
  producer that selected sensors by walking the enable chain would destroy all four assigned addresses
  on every pass, forcing re-enumeration, an epoch bump and a safe stop.
- **One scheduler serves all six boards.** The four cliff sensors and the two grid sensors share the
  differential-I2C bus, and the requirement to trim the grid read exists precisely to protect the
  cliff rate. An acquisition loop written for the four cliff sensors alone is likely to be one the
  grid read cannot later be fitted into.

## Status classification

The contract owns exactly one table mapping the raw ULD status to a class:

`raw status -> VALID_RANGE / NO_TARGET / SENSOR_FAULT / NO_SAMPLE`

The rule that reconciles raw passthrough with the firmware using that table — these read as
contradictory and must be stated together:

> The raw status byte is transmitted unchanged. Firmware uses the contract-owned classification table
> only to select the range encoding and publication outcome; it never replaces the raw status with a
> derived code.

The four classes have frozen consequences, so neither the encoding nor the readiness effect can be
chosen per implementation:

| Class | Frame | `range_mm` | Health | Readiness effect |
| --- | --- | --- | --- | --- |
| `VALID_RANGE` | sent | a finite distance; **`0xFFFF` is forbidden** | `sample_produced` bit set | stays `READY`; the threshold decision is the consumer's |
| `NO_TARGET` | sent | **MUST be `0xFFFF`** | `sample_produced` bit set | stays `READY`. This is data, not a failure, and the far-side value is what makes a real cliff stop the robot |
| `SENSOR_FAULT` | sent | **`0xFFFF`**, raw status preserved | `sample_produced` **and** `sensor_fault` bits set | **loses `READY`** on that cycle: a faulty corner is an unmonitored corner, and there is no partial cliff protection |
| `NO_SAMPLE` | **not sent** | n/a | `sample_produced` bit **clear** | counts as a missing sample: tolerated for a bounded time, see *Readiness* |

`NO_SAMPLE` exists for the two **start-up artefacts**, where the ULD hands back something that is not a
measurement of the scene: the first interrupt after starting back-to-back ranging, and the first results
whose wraparound check has not completed. ST's guidance for both is to discard them. Forcing them into
`NO_TARGET` would inject a phantom cliff at every ranging start; forcing them into `SENSOR_FAULT` would
report healthy hardware as broken. If either persists, the missing-sample budget converts it into a
fault on its own, which is the correct outcome for a sensor that never leaves start-up.

Both the packer and the decoder take their classification from the one generated table below. All four
classes preserve the raw status byte where a frame is sent; firmware-level facts — read failed, sensor
absent, configuration mismatch — live in health and must not re-express or contradict the ULD status.

### The table

Transcribed from `VL53LX_define_RangeStatus_group` in the VL53L4CX ULD (`vl53lx_def.h`), which is
vendored in `LexxHard-ToFSensorBoard-Firmware`. The ULD defines what each code *means*; it does not
decide what is safe, so every class below is **our** decision and the last column is what is still owed.

| Raw | ULD symbol and meaning | Class | Safety rationale | Needs validation |
| --- | --- | --- | --- | --- |
| 0 | `RANGE_VALID` — range is valid | `VALID_RANGE` | The only unambiguous floor measurement | none |
| 1 | `SIGMA_FAIL` — sigma above threshold | `NO_TARGET` | Device healthy, measurement not trustworthy. Calling it a fault would take a robot out of service for a floor-reflectance condition | Rate over the real floor materials; if common, tune the sigma threshold rather than reclassify |
| 2 | `SIGNAL_FAIL` — return signal below threshold | `NO_TARGET` | **This is what a real drop-off looks like**: no return within range. Classifying it as a fault would report every genuine cliff as broken hardware | Rate over dark floors, which is the same signature |
| 3 | `RANGE_VALID_MIN_RANGE_CLIPPED` — target below the minimum detection threshold, range clipped | **`SENSOR_FAULT`**, provisional | A blocked or fouled lens is indistinguishable from a very near floor. Publishing the clipped value as valid makes a permanently blinded sensor look healthy, which is the exact silent failure this design exists to remove | **Provisional until validated three ways**: the as-mounted floor distance measured against the part minimum, the rate over a normal floor, and an occlusion injection. A legitimate near target must not be reported as a permanent hardware fault |
| 4 | `OUTOFBOUNDS_FAIL` — phase outside valid limits, not a wrap exit | `NO_TARGET` | Not interpretable as a distance; device healthy | If it appears in normal operation, treat it as a timing-budget or configuration problem |
| 5 | `HARDWARE_FAIL` | `SENSOR_FAULT` | The device says it failed | none |
| 6 | `RANGE_VALID_NO_WRAP_CHECK_FAIL` — range valid but the wraparound check was not done | **`NO_SAMPLE`** | A start-up artefact, not a scene property: ST states these are the first results after ranging begins, before enough data exists for the wrap check, and advises discarding them. Not sent at all, which is safer than sending it as a range — without the wrap check a far target can alias to a *near* value, and near reads as "floor present" | Whether it clears within the expected number of reads after start. If it persists, the missing-sample budget faults the source, which is correct |
| 7 | `WRAP_TARGET_FAIL` — wrapped target, no matching phase | `NO_TARGET` | The wrap check ran and rejected the target, so the real target is beyond the unambiguous range — the far side | none |
| 8 | `PROCESSING_FAIL` — internal underflow or overflow | `SENSOR_FAULT` | An arithmetic failure inside the device or driver is not a scene property | none |
| 9 | `XTALK_SIGNAL_FAIL` — crosstalk signal fail | `SENSOR_FAULT` | Points at the optical path or a missing crosstalk calibration. As `NO_TARGET` it would present as a permanent phantom cliff with no hardware indication | Whether crosstalk calibration is performed per unit with the final cover glass |
| 10 | `SYNCRONISATION_INT` — first interrupt after starting back-to-back ranging; "ignore data" | **`NO_SAMPLE`** | Not a measurement at all. As `NO_TARGET` it would stop the robot once at every ranging start | Confirm it appears only on the first read after enabling |
| 11 | `RANGE_VALID_MERGED_PULSE` — range ok but the object is several pulses merged | `NO_TARGET`, provisional | ST considers the ranging itself successful with several targets merged. Treating a merged return as untrustworthy at a cliff edge is our safety policy, not the ULD's verdict: a step edge is exactly the geometry that merges returns, and the merged value can read as an intermediate floor where the true floor is far | **Provisional**: rate over a real edge and over plain floor. Frequent on plain floor means the ROI or timing configuration needs work, not a reclassification |
| 12 | `TARGET_PRESENT_LACK_OF_SIGNAL` | `NO_TARGET` | Something is there but cannot be measured; no trustworthy floor | Same family as 1 and 2: dark or specular floors |
| 13 | `MIN_RANGE_FAIL` — the vendored ULD sets this from `VL53LX_DEVICEERROR_USERROICLIP` (`vl53lx_api.c`) | `SENSOR_FAULT` | An **ROI or configuration anomaly**: the device reports the user ROI clipped. Neither the symbol name nor the header's "SPAD array" comment describes what the implementation actually does, so the class follows the code path. Configuration faults are not scene outcomes | Confirm the ROI we configure cannot produce this in normal operation; if it can, the configuration is wrong |
| 14 | `RANGE_INVALID` — driver returned a valid range with a negative value | `SENSOR_FAULT` | A negative distance is a defect, not a scene | none |
| 255 | `NONE` — the vendored ULD sets this when `active_results == 0`, forcing the reported distance to 8191 mm (`vl53lx_api.c`) | **`NO_TARGET`** | **No target detected, and no device error** — a result, not a missing sample. It is the ordinary outcome over a genuine drop-off, so a frame is sent with `0xFFFF`, `sample_produced` set and `READY` retained, and the consumer stops | none for the class. The firmware **must map it to the `0xFFFF` sentinel and never forward 8191 mm as a finite range**, which is a packer test case |

### Two traps this table exists to prevent

**Four ULD symbols begin with `RANGE_VALID`, and only one of them may be published as a range.** Codes
3, 6 and 11 are all named `RANGE_VALID_*` and are all forbidden as finite ranges here, for three
different reasons — a blocked lens, an unchecked wrap that can alias near, and a merged edge return.
An implementer filtering by that name prefix would publish all three.

**The false-stop budget of this feature is the sum of the `NO_TARGET` rows' rates on a real floor.**
Every one of them stops the robot while the system reports itself healthy. If that rate turns out
unacceptable, the remedies are sensor configuration — distance mode, timing budget, sigma and signal
thresholds, ROI — or a temporal filter in the consumer. **Reclassifying a row toward `VALID_RANGE` is
not a remedy**, because each of those rows can carry a near-looking value that reads as "floor
present".

## Decoder and ROS publisher obligations

### Two-level heartbeat

The one failure this whole contract exists to eliminate is silence that looks like health. It is
closed at two levels, and both are normative:

- **Firmware:** once the cliff subsystem is enabled, the health frame is published periodically
  **unconditionally** — including with zero sensors enumerated, `mapping_state == UNKNOWN`, or a chain
  fault. Health silence therefore means the producer is gone, never "nothing to report".
- **Driver:** once the cliff feature is enabled, the ROS safety-health topic is published periodically
  **from node startup**, beginning as `NOT_READY`, even if no CAN frame ever arrives. A consumer can
  therefore distinguish "the SCB is not sending health" from "the driver does not exist", which
  subscribing to an absent topic cannot.

Only the four role measurements are gated. Health is never gated.

### Correlation

Slot opening, completion, expiry at `T_cycle_assembly`, retirement, the modulo comparison and the
plausibility window are specified once in *The acquisition cycle is the unit of correlation* above, and
are decoder obligations. Two of them are worth restating because they are the ones an implementation
skips: **only a `cycle_valid` health frame may move the anchor**, and **a retired cycle is final**.

### Validation, and protocol fault

Reject, count and report: DLC other than 8; `frame_type` not matching the arrival identifier; an
unsupported `protocol_version`; `source_id` outside 0-3; any non-zero reserved field; `mapping_state`
outside `0x0`-`0x3`; `failing_chain_position` outside 1-6 and not `0xFF`; `target_count` above 4; a
`target_count` of 0 that does not carry status `255` and `0xFFFF`; and, with `cycle_valid` clear, a
non-zero `cycle_seq` or a non-zero per-cycle mask.

Three **contradictions** are rejected on the same footing, because each means one of two fields is
wrong with no safe way to guess which, and the inconsistency is itself evidence of a defect upstream:

- a status class that contradicts the range sentinel
- a `sample_produced_mask` bit set with no measurement for that source in that cycle, or a measurement
  present with the bit clear — decidable only because cycles are correlated, and exactly the check that
  catches a packer bug before it becomes a silent blind corner
- `failing_chain_position != 0xFF` with no `flags` bit 0-2 set and no incomplete mask
- a measurement whose **own cycle's** health frame says `mapping_state` was not `PROVEN`, which the
  firmware obligations forbid

**That judgement is made against the health frame correlated to the measurement's own
`(mapping_epoch, cycle_seq)`, never against the current health state.** The contract permits arbitrary
interleaving, so a measurement from a cycle that was legitimately `PROVEN` can arrive after a newer
health frame has reported `LOST`; treating it as illegal would manufacture a fault out of the reordering
this contract itself allows. A measurement whose cycle is already retired is dropped and counted, not
faulted; a measurement whose cycle has no health frame yet is buffered, which is the ordinary case.

**Rejection is not enough.** Any of the above puts the cliff subsystem into **protocol FAULT
immediately**, rather than leaving the consumer to notice a timeout later. In protocol FAULT no role
measurement is published and safety health carries `FAULT` with the reason.

A malformed frame MAY update transport liveness — it is evidence the bus is alive, and reporting
`no frames` would send an investigation towards the chain or the CAN filter when the fault is
corruption. It MUST NOT refresh measurement or health freshness, and MUST NOT contribute to `READY`.

Protocol FAULT **latches**. It clears only after one complete, contradiction-free cycle at a supported
version, correlated with its health frame, and the transition is reported so an operator sees recovery
rather than only onset.

### Readiness

`READY` means: the four role measurements published now are trustworthy. It requires **all** of the
following, and there is no partial readiness — three fresh sources out of four is `NOT_READY`, because
the missing one is exactly where an undetected drop would be.

- the subsystem is not in protocol FAULT or `CONFIG_ERROR`, and `protocol_version` is supported
- a health frame with a **new** `health_seq` was received within `T_health_max_gap`
- the most recent health frame has `cycle_valid` set
- `mapping_state == PROVEN`
- `flags` bits 0-2 all clear, and `failing_chain_position == 0xFF`
- `enumerated_mask == 0xF` and `model_verified_mask == 0xF`
- `sensor_fault_mask == 0`
- each of the four sources has an accepted measurement from a **new** cycle within `T_meas_max_gap`,
  and every one of those four measurements shares the epoch and cycle of an authorising health frame
- no source has reached `N_cycle_miss_fault` consecutive cycles without a sample

### What is tolerated, and what is not

`sample_produced_mask` is deliberately **not** an instantaneous `READY` gate. A mask short of `0xF` in
one cycle is a **degraded** condition: it is reported, and it costs `READY` on its own once the source
crosses one of the two bounds below. Gating on the instantaneous mask would make the tolerance zero
cycles and contradict `T_meas_max_gap`, so a single retryable NACK on a six-device chain would stop the
robot.

The tolerance has **two triggers, and whichever is reached first ends `READY`**:

- `T_meas_max_gap` — elapsed time on a **monotonic clock** since that source's last accepted
  measurement. This is the **only guaranteed bound**, and it is the one the safety argument rests on,
  because what physically matters is how far the robot travels while a corner is unmonitored
- `N_cycle_miss_fault` — the source has produced no sample in that many **consecutive cycles**, counted
  at `>=`, not `>`. Reaching it is a `FAULT` rather than a plain `NOT_READY`, because a source that has
  missed that many cycles in a row is not late, it is broken

**The count carries no timing guarantee, and the contract makes no claim that it fires earlier.** An
earlier draft required `N x T_cycle_nominal <= T_meas_max_gap`; that formula was wrong in the unsafe
direction. Cycle periods stretch under load — the same service-delay mechanism the six-board scheduling
analysis quantifies — so N cycles can take considerably longer in wall-clock than N nominal periods, and
a count-based bound derived from the nominal period would silently be looser than it claimed. The count
is therefore a supplementary diagnostic trigger with an explicit fault outcome, never the thing that
bounds blindness.

**What is tolerated is a temporarily missing sample, never a faulty one.** A `sensor_fault_mask` bit
costs `READY` on the cycle it appears in, with no cycle budget and no grace: the sensor has told us its
sample is unusable, which is a different fact from not having produced one.

On receiving a health snapshot with `PROVEN` and an epoch different from the current one, the decoder
MUST discard the four cached ranges and their freshness timestamps from the previous epoch, reject any
measurement of the new epoch that arrived **before** its authorising health snapshot, and withhold
`READY` until the criteria above hold within that same epoch. An epoch that advances while
`mapping_state` stays `PROVEN` is treated identically: it is not a continuation.

`READY` is lost immediately when any criterion fails.

### Publishing

While `READY`, the decoder publishes the four role measurements, converting millimetres to metres at
this boundary. It does **not** decide cliffs, filter in time, reduce four channels into two, or turn
an invalid or no-target reading into `0.0`.

While not `READY`:

- **no role-named measurement is published at all** — not one of the four. Falling back to
  position-numbered names on the production interface is also forbidden, because such a topic is too
  easily wired into something that treats it as a role
- the ROS safety-health topic continues to publish, carrying `NOT_READY` or `FAULT` and the reason
- `DiagnosticArray` is for the operator. It MUST NOT be the transport of any safety handshake

The ROS-facing interface is four `sensor_msgs/Range` topics under `/global/system/cliff_tof/`, named
for the roles, plus `/global/system/cliff_tof/health`. The driver normalises strictly before
publishing: a valid measurement becomes a finite range in metres; `NO_TARGET` and `SENSOR_FAULT`
become `+Inf`. **`+Inf` is a convention this project defines, not something a `Range` consumer does
automatically** — it happens to sit on the fail-safe side and is adopted for that reason, so it must be
written into the interface specification and implemented on the consumer side rather than assumed.

### Configuration failure is contained to the cliff subsystem

Two configurations are impossible and MUST fail rather than be silently resolved: the cliff feature
enabled together with `tof_transport=legacy_uart`, and any required cliff timing parameter left unset.
There is deliberately **no runnable default** for the timeout parameters.

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
be remapped into the cliff namespace, and the cliff topic names above are deliberately chosen so that
the established `/sensor_set/X -> /global/system/X` remap convention cannot connect them; and this
transport MUST NOT feed or bridge into `safety_manager/downward`, which stays dead for this feature.
Two decision paths would mean two threshold sources and two ways to command a stop.

## Timing values — required, and not frozen

The following are **required fields of this contract that have no values yet**. They cannot be
established offline: they are outputs of the L4 acquisition thread and of the six-board schedule
measured on real hardware.

| Symbol | Meaning |
| --- | --- |
| `T_cycle_nominal` | nominal acquisition-cycle period, which is what a health mask describes |
| `T_meas_max_gap` | maximum tolerable gap between accepted measurements of one sensor |
| `T_cycle_assembly` | how long one cycle slot may stay open before it expires |
| `T_health_nominal` | nominal health snapshot period |
| `T_health_max_gap` | maximum tolerable gap between consecutive **new** `health_seq` values |
| `T_skew_max` | worst-case sampling phase skew across the four sensors within a cycle |
| `T_health_delivery_max` | worst-case health frame latency from production to reception under full bus load |
| `N_cycle_miss_fault` | consecutive cycles without a sample from one source before it faults |
| `N_cycle_advance_max` | largest plausible forward jump in `cycle_seq` before it is treated as implausible |

Five rules constrain the eventual numbers rather than the schedule:

- **The ROS-side timeouts are derived from these values, never guessed.** A consumer timeout chosen
  independently of the producer's real period is either a nuisance stop or a missed cliff.
- **`T_meas_max_gap` must be derived from the stopping distance**, not from the cycle period: what
  matters is how far the robot travels while one corner is unmonitored.
- **`T_health_max_gap` must be strictly larger than `T_health_delivery_max`**, and the consumer's health
  timeout strictly larger again with margin, because the health frame
  is chain-level: one dropped frame must not by itself trip a stop.
- **`T_skew_max` + `T_health_delivery_max` < `T_cycle_assembly` < `T_meas_max_gap`.** Below the lower
  bound, cycles expire while their own frames are still legitimately in flight; above the upper bound, a
  slot outlives the freshness of the data in it.
- **`N_cycle_miss_fault` is not tied to any period.** It is a cycle count with a fault outcome and no
  timing claim; `T_meas_max_gap` on a monotonic clock is the only bound the safety argument uses. Do not
  reintroduce a product of a count and a nominal period, because cycle periods stretch under load.

## Golden vectors

Vectors do **not** exist yet; generating them is the step after the first freeze. They will come from a
sibling generator, `gen_cliff_golden_vectors.py`, emitting a JSON file and a dependency-free C++
header, both carrying the SHA-256 of **this** file, pinned independently by the firmware packer test
and the driver decoder test. The grid contract's generator, vectors and SHA are untouched.

Beyond the happy path the vectors MUST cover at least:

- every one of the sixteen raw statuses, mapped to its class, including all four `RANGE_VALID_*` codes
  and both `NO_SAMPLE` codes
- one source missing from a cycle; all four missing; a cycle with no measurements at all
- `mapping_state` `UNKNOWN`, `PROVEN`, `LOST`, and recovery back to `PROVEN`
- health stopping while measurements continue, and `health_seq` repeated
- a measurement arriving before any health snapshot at all — cold start
- a measurement arriving while health says `UNKNOWN`, `LOST` or `FAULT`, which is a firmware fault and
  must produce protocol FAULT
- a new epoch with `PROVEN` where only some of the four sources have refreshed
- a measurement of the **previous** epoch or a **retired** cycle arriving late
- two different measurements for one `(source_id, epoch, cycle_seq)`
- `mapping_epoch` and `cycle_seq` wrapping 255 -> 0
- measurement and health disagreeing on epoch, and on cycle
- a status class contradicting the range sentinel
- `sample_produced_mask` set with no measurement in that cycle, and the converse
- a `cycle_valid`-clear heartbeat immediately followed by the genuine cycle 0 of a new epoch, asserting
  that the heartbeat neither opened nor retired a cycle
- a `cycle_valid`-clear frame carrying a non-zero `cycle_seq` or a non-zero per-cycle mask
- a cycle expiring at `T_cycle_assembly` with its health claiming a sample that never arrived (protocol
  FAULT), and one expiring with no health at all (degraded, not a fault)
- a `cycle_seq` jump of exactly `N_cycle_advance_max` (accepted) and one beyond it (implausible, protocol
  FAULT, then resynchronisation from the next `cycle_valid` health frame)
- a stale frame arriving after a wrap so that its `cycle_seq` aliases onto a live cycle, asserting it is
  caught as a conflict rather than accepted
- a source that produces no sample for several consecutive cycles while still inside `T_meas_max_gap`,
  asserting the source is reported degraded but the subsystem stays `READY` until a trigger is reached
- a single-cycle `sensor_fault_mask` bit, asserting `READY` is lost immediately with no cycle budget
- a source reaching exactly `N_cycle_miss_fault` consecutive cycles without a sample (FAULT) and one
  cycle short of it (still `READY` if within `T_meas_max_gap`)
- a late measurement from a `PROVEN` cycle arriving after a newer health frame reports `LOST`, asserting
  it is **not** a protocol fault
- status `255` with `target_count = 0`, asserting `0xFFFF` on the wire and never 8191 mm
- the multi-target reduction: two valid targets (the farther one is transmitted), a valid target
  alongside a `SENSOR_FAULT` target (the whole measurement is `SENSOR_FAULT`), a valid target alongside a
  `NO_TARGET` target, four targets, and `target_count` inconsistent with the reduced status
- `failing_chain_position` set with no fault indication
- an unsupported `protocol_version`, and a zero `protocol_version`
- non-zero reserved fields, DLC other than 8, `frame_type` mismatched to its identifier, `source_id`
  out of range, `mapping_state` out of range
- a `CONFIG_ERROR` start, asserting that no role topic is ever advertised and that safety health
  publishes `FAULT`
- protocol FAULT latching, and clearing only after one clean correlated cycle

As in the grid contract, each scenario declares the **complete multiset** of events it must produce, so
an implementation cannot pass by emitting the right event alongside wrong ones. These are the cases
where an implementation silently picks a fail direction; if the vectors only cover the happy path, the
two sides will each pick one and they will not pick the same one.

## Open decisions

Nothing in this section may be resolved unilaterally, and the contract cannot be frozen while any of
it is open.

- **The health CAN identifier.** `0x217` is a *candidate only*. The 2026-08-06 sweep is stale — three
  rows were added to the table since — so allocation requires a fresh sweep of both repositories plus
  a live capture, then a self-assignment recorded here and in the team's CAN ID register.
- **The message type for the typed safety-health topic**, which does not exist yet. The topic names,
  the four `Range` topics and the `+Inf` convention are settled above; the message definition and the
  package it lives in are not.
- **All timing values** above, and with them the consumer timeouts.
- **The validation column of the status classification.** The rows are complete and traced to the
  vendored ULD's own code paths, but two are explicitly **provisional** and can only be settled on
  hardware: code 3 as a fault rather than a near floor, which needs the as-mounted distance plus an
  occlusion injection, and code 11 as untrustworthy, which needs edge and plain-floor rates. Freezing
  requires those rates, because the `NO_TARGET` rows' rates *are* the false-stop budget.
- **The multi-target reduction's availability cost.** The rule is frozen and conservative — any faulty
  target condemns the measurement, and the farthest valid target wins — but the rate of multi-target
  scenes and of mixed-status targets over a real floor is unmeasured, and either could cost more
  availability than expected.
- **A per-source distinction between an I2C failure and a no-update**, currently only visible at chain
  level. It would need another field and therefore a version bump; deferred deliberately.
- **What evidence proves `mapping_state == PROVEN`**, which cannot be settled until the enable-chain
  defect is fixed.

## Freeze procedure

This draft becomes normative in two steps, not one.

First freeze, after the identifier allocation, the health message definition and the status rows: fill
those in, generate the vectors, and pin the SHA on both sides. The timing symbols stay explicitly
marked as provisional, and the version keeps a marker that forbids release.

Second freeze, after the acquisition thread exists and the six-board schedule has been measured: write
in the measured timing values, remove the draft marker, regenerate the vectors, and re-pin both sides
in the same round. Only then may anything be released against this contract.
