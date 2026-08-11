# Cliff ToF CAN wire contract (AMRSW-2994)

Contract version: **draft-2026-08-11b**
Status: **provisional draft, NOT frozen.** The byte layouts, encodings and state rules below are
written to be implementable as they stand, but three classes of content are deliberately unresolved
and are listed in *Open decisions*: the health CAN identifier, the ROS message type for safety health,
and every timing value. **No implementation may be released against this version**, and no golden
vectors exist yet.

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

Two identifiers rather than one because measurement and health must **dispatch independently**.
Measurement takes the lower identifier because there are four of it per cycle against one health
frame, so putting the more frequent traffic first minimises queueing overall.

**That is not an argument that health is less important.** Both frames are safety-relevant: the
health channel is what turns "no measurement arrived" from an ambiguity into a decision, and without
it the data channel cannot be trusted at all. The arbitration choice therefore carries an obligation:
**the worst-case health latency under full bus load must be shown, by measurement or analysis, to
stay inside `T_health_max_gap`.** If it cannot, health takes the lower identifier instead. It is not
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
  actually completes. While no acquisition is running — enabled but not `PROVEN`, for instance —
  health keeps flowing with `health_seq` advancing, `cycle_seq` held at the last completed cycle (or
  `0` if none in this epoch) and all masks zero.

## Measurement frame (`TOF_CLIFF_MEAS_ID`)

DLC 8, always. One frame is one completed read of one sensor. No multi-frame assembly.

```
byte 0 : frame_type << 4 | source_id   (frame_type = 0x1; source_id 0-3)
byte 1 : mapping_epoch                 (uint8, wraps 255 -> 0)
byte 2 : cycle_seq                     (uint8, shared with the health frame, wraps 255 -> 0)
byte 3 : range_mm[15:8]                (big-endian, as in the grid contract)
byte 4 : range_mm[7:0]
byte 5 : range_status                  (the raw ULD status byte, transmitted unchanged)
byte 6 : reserved, MUST be 0 on transmit and MUST be rejected if non-zero
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
- Bytes 6 and 7 are the designated space for a future SCB capture tick, if bounding the age of the
  measurement rather than of its arrival ever becomes a requirement. Using them is a version bump.

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
byte 3 : mapping_state << 4 | chain_fault
byte 4 : enumerated_mask << 4 | model_verified_mask
byte 5 : transport_read_completed_mask << 4 | sensor_fault_mask
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

### Mapping state and chain fault

| `mapping_state` | Meaning |
| --- | --- |
| `0x0` `UNKNOWN` | never proven since the last enumeration attempt |
| `0x1` `PROVEN` | the position-to-`source_id` mapping has been proven by the evidence this contract requires |
| `0x2` `LOST` | was proven, then a sensor was lost at runtime |
| `0x3` `FAULT` | a fault prevents any trustworthy mapping |
| `0x4`-`0xF` | malformed |

`chain_fault` is a 4-bit flag field: bit 0 chain length differs from the configured expectation, bit 1
enumeration was frozen because a non-tail position produced no new device, bit 2 a transport-level bus
fault was seen, bit 3 reserved and MUST be 0.

### The four masks

Each mask is 4 bits, **bit `k` is `source_id` `k`**. They are *not* keyed by chain position, and they
describe **the cycle named by `cycle_seq` in the same frame**.

| Mask | What it observes |
| --- | --- |
| `enumerated_mask` | this source enumerated to its own address |
| `model_verified_mask` | this source returned the expected model ID |
| `transport_read_completed_mask` | this source completed an I2C read in this cycle |
| `sensor_fault_mask` | this source's read in this cycle classified as `SENSOR_FAULT` |

**`transport_read_completed_mask` is named for what it actually observes.** An I2C read can complete
successfully and still return a ULD `SENSOR_FAULT` status; a field called "OK" would collapse two
different facts into one bit. Whether a *usable range* was produced is carried by the measurement
frame's status and range encoding, and by `sensor_fault_mask`. A consumer that needs "this sensor is
delivering usable data" must look at both.

`sensor_fault_mask` is **per cycle, not latching**. A transient fault disappears from the next
snapshot, so any escalation policy — how many consecutive cycles of fault become a persistent
condition — belongs in the decoder, not in the firmware.

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

**A measurement frame is produced only by a complete read, and only while `mapping_state == PROVEN`.**
A read that found no target is data, not a failure: it is transmitted with `0xFFFF` and its real
status.

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

`raw status -> VALID_RANGE / NO_TARGET / SENSOR_FAULT`

The rule that reconciles raw passthrough with the firmware using that table — these read as
contradictory and must be stated together:

> The raw status byte is transmitted unchanged. Firmware uses the contract-owned classification table
> only to select the range encoding and publication outcome; it never replaces the raw status with a
> derived code.

The three classes have frozen consequences, so the encoding cannot be chosen per implementation:

| Class | `range_mm` | Health | Consumer effect |
| --- | --- | --- | --- |
| `VALID_RANGE` | a finite distance; **`0xFFFF` is forbidden** | unaffected | the only case that may yield a finite range; the threshold decision is the consumer's |
| `NO_TARGET` | **MUST be `0xFFFF`** | unaffected | data, not a failure. Stays `READY`, and the far-side value is what makes a real cliff stop the robot |
| `SENSOR_FAULT` | **`0xFFFF`**, raw status preserved | `sensor_fault_mask` bit set for this cycle | not `READY`: a faulty corner is an unmonitored corner, and there is no partial cliff protection |

Both the packer and the decoder take their classification from the one generated table and its
vectors. Firmware-level facts — read failed, sensor absent, configuration mismatch — live in health
and must not re-express or contradict the ULD status.

The concrete status-to-class rows are **not yet filled in**: they must be transcribed from the
VL53L4CX ULD's `RangeStatus` definitions when the driver is ported, not guessed from the VL53L7CX
table.

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

### Correlation and retirement

The decoder keeps at most **two open cycles per epoch**, the current and the previous, each holding up
to four measurements and the health frame, all keyed by `(mapping_epoch, cycle_seq)`.

A cycle closes when its health frame and every measurement its masks imply have been received, or when
a newer cycle displaces it. **A closed or displaced cycle is retired**, and a later frame carrying a
retired `(epoch, cycle_seq)` is counted and dropped: it cannot revive the cycle, cannot refresh any
freshness timestamp and cannot contribute to `READY`. Without retirement, a late frame from an old
cycle could reassemble a cycle that had already been judged untrustworthy — the same reasoning that
makes a grid generation single-use.

An epoch change retires every cycle of the previous epoch immediately.

### Validation, and protocol fault

Reject, count and report: DLC other than 8; `frame_type` not matching the arrival identifier; an
unsupported `protocol_version`; `source_id` outside 0-3; any non-zero reserved field; `mapping_state`
outside `0x0`-`0x3`; `chain_fault` bit 3 set; `failing_chain_position` outside 1-6 and not `0xFF`.

Three **contradictions** are rejected on the same footing, because each means one of two fields is
wrong with no safe way to guess which, and the inconsistency is itself evidence of a defect upstream:

- a status class that contradicts the range sentinel
- a `transport_read_completed_mask` bit set with no measurement for that source in that cycle, or a
  measurement present with the bit clear — decidable only because cycles are correlated, and exactly
  the check that catches a packer bug before it becomes a silent blind corner
- `failing_chain_position != 0xFF` with no `chain_fault` bit and no incomplete mask

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
- `mapping_state == PROVEN`
- `chain_fault == 0` and `failing_chain_position == 0xFF`
- `enumerated_mask == 0xF` and `model_verified_mask == 0xF`
- `sensor_fault_mask == 0`
- each of the four sources has an accepted measurement from a **new** cycle within `T_meas_max_gap`,
  and every one of those four measurements shares the epoch and cycle of an authorising health frame

`transport_read_completed_mask` is deliberately **not** an instantaneous `READY` gate. A mask short of
`0xF` in one cycle is a **degraded** condition: it is reported, and it will cost `READY` on its own
once a source exceeds `T_meas_max_gap`. Gating on the instantaneous mask instead would make the
tolerance zero cycles and contradict `T_meas_max_gap`, so that a single retryable NACK on a six-device
chain stops the robot. What must not be tolerated is *persistence*: after `N_cycle_miss_max`
consecutive cycles with the mask short of `0xF`, the subsystem enters `FAULT` regardless of freshness.

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
| `T_health_nominal` | nominal health snapshot period |
| `T_health_max_gap` | maximum tolerable gap between consecutive **new** `health_seq` values |
| `T_skew_max` | worst-case sampling phase skew across the four sensors within a cycle |
| `N_cycle_miss_max` | consecutive cycles with an incomplete read mask before the subsystem faults |

Three rules constrain the eventual numbers rather than the schedule:

- **The ROS-side timeouts are derived from these values, never guessed.** A consumer timeout chosen
  independently of the producer's real period is either a nuisance stop or a missed cliff.
- **`T_meas_max_gap` must be derived from the stopping distance**, not from the cycle period: what
  matters is how far the robot travels while one corner is unmonitored.
- **`T_health_max_gap` must be strictly larger than the worst-case health latency under full bus
  load**, and the consumer's health timeout strictly larger again with margin, because the health frame
  is chain-level: one dropped frame must not by itself trip a stop.

## Golden vectors

Vectors do **not** exist yet; generating them is the step after the first freeze. They will come from a
sibling generator, `gen_cliff_golden_vectors.py`, emitting a JSON file and a dependency-free C++
header, both carrying the SHA-256 of **this** file, pinned independently by the firmware packer test
and the driver decoder test. The grid contract's generator, vectors and SHA are untouched.

Beyond the happy path the vectors MUST cover at least:

- each status class, and `0xFFFF` with `NO_TARGET` and with `SENSOR_FAULT`
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
- `transport_read_completed_mask` set with no measurement in that cycle, and the converse
- `failing_chain_position` set with no fault indication
- an unsupported `protocol_version`, and a zero `protocol_version`
- non-zero reserved fields, DLC other than 8, `frame_type` mismatched to its identifier, `source_id`
  out of range, `mapping_state` out of range, `chain_fault` bit 3 set
- `N_cycle_miss_max` consecutive incomplete cycles reaching `FAULT`
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
- **The status-to-class rows**, to be transcribed from the VL53L4CX ULD rather than guessed.
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
