# Cliff ToF CAN wire contract (AMRSW-2994)

Contract version: **commissioning-2026-08-18c**
Wire `PROTOCOL_VERSION`: **1** (unchanged from the draft series — the wire format did not change)
Release status: **RELEASE_FORBIDDEN.**

The `-18b` revision existed for one purpose: to let the commissioning end-to-end path be built against
byte-exact, double-pinned vectors instead of against prose. It is **not** a product release and must
never be treated as one.

`-18c` adds nothing to the wire and changes no byte of any vector. It exists because writing the
mapping-proof implementation against `-18b` surfaced three defects in the prose, and each of them would
have been resolved in code — silently, and differently in the two repositories — if the document had
been left as it was:

- **A decoder rule contradicted itself about late measurements.** The required-vector list called a
  measurement "arriving while health says `LOST`" a firmware fault, while the normative correlation rule
  says the judgement is made against the measurement's *own* cycle's health frame and that this exact
  reordering is legal. The firmware makes the reordering routine — its measurement and health sends are
  deliberately not serialised — so a decoder written to the vector list would have entered protocol
  `FAULT` on the first genuine mapping loss. Corrected in *Golden vectors*; the normative rule was
  already right and is unchanged.
- **The `PROVEN` evidence set could not be satisfied by any single chain state.** Tail isolation
  destroys the addresses of positions 3-5, so the chain that holds the live addresses can never be the
  chain that produced the isolation result. `-18b` demanded all three results without saying how the
  evidence transfers, which invites closing the gap by loosening a criterion. Now specified as a
  **transaction** with a semantic fingerprint equality between the two enumerations.
- **`mapping_epoch` issuance had no owner under this profile.** The firmware has no persistent store, so
  it cannot discharge the cross-restart obligation on its own. The commissioning profile now names the
  host as the issuing authority and states exactly what the firmware still guarantees and what it no
  longer claims.

What this revision settles, and what it does not:

- **Frame layouts, encodings and validation rules: settled.** Golden vectors for the *layout* are
  generated from this document and pinned by both repositories.
- **Health CAN identifier: `0x217`, usable here, registration outstanding.** Self-assigned 2026-08-17
  under the same team authorisation as `0x214`/`0x215`/`0x216`, after a fresh scan of both repositories
  and a live `can1` capture. But this contract's own rule is that **the team's CAN ID register is the
  deciding evidence** and a scan and a capture are only supporting — so the row is still owed, along
  with the grid identifiers' rows. It must be closed before production.
- **Status classes 3 and 11: re-decided 2026-08-18, and `validation_pending`.** Class 3 stays
  `SENSOR_FAULT`, class 11 stays `NO_TARGET` — the values the classification table already carried.
  These were **re-opened deliberately** rather than inherited, because an intermediate proposal had both
  as `SENSOR_FAULT` and the difference is behavioural, not cosmetic: `NO_TARGET` publishes the far-side
  sentinel and keeps `READY`, while `SENSOR_FAULT` sets a `sensor_fault_mask` bit and loses `READY`
  immediately with no cycle budget.

  Class 11 is `NO_TARGET` because a merged return is the signature of the hazard, not of broken
  hardware: a step edge is exactly the geometry that merges pulses, so `SENSOR_FAULT` would drop the
  subsystem into a fault state every time the robot approached a real cliff, and would attribute a scene
  property to the sensor. `NO_TARGET` stops the robot for the same reading and recovers on the next cycle
  once the edge is no longer in view. Class 3 is `SENSOR_FAULT` because a fouled lens and a very near
  floor are indistinguishable, and a permanently blinded sensor must not read as healthy.

  Authorising both for commissioning is **not** a statement that either has been validated on hardware.
  The validation column applies in full, and class 11's stated risk is the live one: if merged pulses
  turn out to be common over plain floor, that is a ROI or timing-configuration problem to fix, not a
  reason to reclassify.
- **Timing values: a named commissioning profile, not measurements.** They are model-derived and are
  marked as such throughout. **A production configuration must not inherit them**; see
  *Commissioning timing profile*.
- **Decoder state-machine golden vectors: still absent.** The scenario catalogue in
  `gen_cliff_golden_vectors.py` remains a catalogue, and the generator continues to refuse to emit
  event-multiset vectors. This revision covers layout only.

The release ban lifts only when all four of these are closed: the timing values come from the six-board
schedule measurement; the status 3 / 11 rates and the multi-target scenarios are validated on hardware;
the **frozen position-to-role table** for the four cliff carriers exists, without which no mask keyed by
`source_id` can be filled honestly and `PROVEN` is unreachable by rule; and a **firmware-side persistent
`mapping_epoch` issuer** exists, because the host-issued epoch of this profile presupposes an operator.
Closing any of them is a version bump and a re-pin on both sides.

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

`0x217` is health, **self-assigned for commissioning on 2026-08-17 and not yet recorded in the team's
CAN ID register.** Evidence gathered: a fresh source scan of both repositories that day found the
highest assigned identifier to be `0x216` and `0x217` unclaimed in either tree, and a live `can1`
capture on DS20001 showed `0x100`-`0x131`, `0x204`, `0x206`/`0x207`, `0x209`/`0x20A`, `0x20C`, `0x20F`
and `0x212` in use with `0x213`-`0x217` silent.

**Neither of those is an allocation, and the earlier wording in this section still stands.** A source
sweep cannot see a transmitter whose code we do not hold; a live capture cannot see one that stays
silent while the bus is recorded — `0x213` is in the source tree and did not appear in that capture,
which is the point made concretely. **The team's CAN ID register remains the deciding evidence.**

So the status is: usable under this commissioning revision, **with registration outstanding**. That
outstanding item is the same one the grid allocation left open — `0x214`/`0x215`/`0x216` also still owe
their rows — and it must be closed before any production release. Self-assignment authorised by the
team means being responsible for the allocation, not being excused from recording it.

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
only while `mapping_state == PROVEN`, and `PROVEN` is a claim about the machine in front of you, not
about the design.

**Updated 2026-08-17.** The defect that made `PROVEN` unreachable — one enable clock pulse advancing
two stages, so two identical VL53L4CX could sit on the factory-default address at once and a single
address assignment moved both undetectably — was root-caused and has a working fix. The cause was a
timing race, not a wiring error: the shared clock net is heavily loaded while each data line is a
single point-to-point hop, so the fast data edge beat the slow clock edge into the receiving flip-flop.
A series resistor on the data line slows and delays that edge, and with it fitted the on-machine gate
passed 5/5 rounds on DS20001 — six positions individually addressed with type-appropriate identity
reads at six distinct addresses, nothing left at the default address, and tail isolation showing
position 6 on its own address rather than position 5's.

Two things that does **not** mean. It is a **commissioning workaround on one machine**, not a
production-qualified fix; the resistor value, its placement at every hop, and the flip-flop's hold
margin are all open. And it changes nothing about the rule: on hardware that has not passed that gate,
`mapping_state` stays `UNKNOWN`, a conforming implementation publishes **no role-named data at all**,
`0x216` carries **no traffic**, and the only cliff traffic is the health heartbeat reporting `UNKNOWN`.
That remains the intended behaviour rather than a defect, and it is what a bring-up engineer should
expect to see on an ungated machine.

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
  removes the main way an old frame could alias onto a new cycle. **Under the commissioning profile the
  issuing *authority* is the host, not the firmware** — the obligation is the same, who discharges it is
  not, and the firmware makes no cross-restart claim of its own. See *Commissioning `mapping_epoch`
  issuance*.

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
| 0 | status `255`, `range_mm = 0xFFFF`, `target_count = 0`. This holds in both directions: `target_count == 0` **if and only if** the frame carries status `255` and `range_mm == 0xFFFF` |
| 1 | that target's status, classified by the table below |
| 2-4 | classify every target, then take the **most conservative class present**, in the order `SENSOR_FAULT` > `NO_SAMPLE` > `NO_TARGET` > `VALID_RANGE` |

When the surviving class is `VALID_RANGE` and more than one target qualifies, transmit the **farthest**
of them.

### Which target's raw status is transmitted

Added 2026-08-18. The rules above pinned the surviving class and the range but left the status byte
undefined whenever more than one target carried the surviving class — `[idx0: status 5, idx1: status 8]`
are both `SENSOR_FAULT`, and nothing said whether `5` or `8` reaches the wire. That is exactly the gap
that lets two implementations disagree about the floor while each passes its own tests, which is why
this reduction is contract-owned rather than an implementation choice.

> Select the highest-priority class first. If that class holds more than one target, select the one with
> the **lowest index in the ULD's raw result array**, and transmit its raw status unchanged. **Do not
> sort and then take.** `VALID_RANGE` remains the exception: select the **farthest** target, and
> transmit **that** target's raw status.

Why this rule and not the alternatives:

- The status always comes from a **real target**, which is what "transmitted unchanged" requires. Under
  `VALID_RANGE` the range and the status come from the same target, so the frame describes one target
  rather than a composite of two.
- **Lowest index, not lowest numeric value.** The numeric ordering of the status codes carries no safety
  or device meaning — one enumerator simply happens to be smaller than another. The index does carry
  meaning: it is the first target the device reported.
- An aggregate sentinel such as `0xFE` would **no longer be a ULD raw status**, so introducing one is a
  `protocol_version` bump, not a reduction rule.
- If the ULD's target ordering ever jitters, the consequence is confined to the **diagnostic** status
  byte. The surviving class, the range encoding and therefore the stopping outcome are unchanged. That
  is the property that makes depending on vendor ordering acceptable here.

This rule is exercised by the packer's reduction tests, not by the layout vectors: the pre-reduction
target list never reaches the wire, so the decoder cannot see it and there is nothing for a shared
vector to pin.

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
`SYNCRONISATION_INT` (10) on the first read after starting back-to-back ranging, and
`RANGE_VALID_NO_WRAP_CHECK_FAIL` (6) until the wraparound check has enough data, and neither is a
measurement of the scene (see *Status classification*). Those two statuses are the whole of `NO_SAMPLE`;
in particular `NONE` (255) is **not** one of them — it is a real no-target result. Defining the bit as
"a read that produced a sample"
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
  error. **The evidence that proves it is now defined** — see *What proves `mapping_state == PROVEN`
  under this profile* — and it is three specific on-machine results, not the absence of an error. On a
  machine that has not produced them, a conforming firmware never reaches `PROVEN` and never transmits
  a measurement frame.
- Re-enumeration is permitted only once the robot is already safely stopped. A new `mapping_epoch` may
  be issued, and measurements may resume, only after the mapping has been fully proven again — which
  means the whole transaction of *The proof is a transaction*, not merely an enumeration that returned
  `complete`. Authorisation is withdrawn **before** the first enable line moves, so a chain being
  re-enumerated can never be a chain producing measurements. On
  hardware where the enable chain can still advance two stages on one pulse, any implementation that
  reaches `PROVEN` automatically has a bug — that was the state of every machine before 2026-08-17, and
  it remains the state of any machine whose enable chain has not passed the gate.

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
outside `0x0`-`0x3`; `failing_chain_position` outside 1-6 and not `0xFF`; `target_count` above 4; any violation of the
biconditional `target_count == 0` <-> (status `255` and `range_mm == 0xFFFF`), in either direction; and,
with `cycle_valid` clear, a non-zero `cycle_seq` or a non-zero per-cycle mask.

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

### Never having received health is a fault, not an eternal initialisation

The absence of health has to be resolved in time, or a permanently broken link
reads as "still starting up" forever, which is the same silence-looks-like-health
failure this contract exists to remove. The mapping is normative:

| Condition | State |
| --- | --- |
| No health received, still inside `T_startup_health_grace` | `NOT_READY`, reason: startup and no health |
| No health received, grace expired | **`FAULT`**, reason: no health |
| Health received, then no new `health_seq` within `T_health_max_gap` | **`FAULT`**, reason: health stale |

The grace is its own required value rather than a reuse of `T_health_max_gap`,
because the two bound different things: the grace covers node start, SCB boot and
the worst-case arrival of the **first health heartbeat** — not the first acquisition
cycle, since health does not wait for one — while `T_health_max_gap` bounds the
interval between snapshots of a producer that is already running. The grid path keeps the same separation
between its startup grace and its staleness threshold, for the same reason.

### Two kinds of fault, and only one of them latches

- A **timeout or condition fault** — no health, stale health, a stale source, a
  sensor fault, a chain fault, a lost mapping — is computed afresh from the
  current state on every snapshot, and therefore **clears by itself** once the
  conditions hold again. It must not require a restart.
- A **protocol fault** — a malformed frame or a contract contradiction — **latches**,
  and clears only after one complete, contradiction-free cycle at a supported
  version, as specified above. The difference is deliberate: a timeout says the
  world changed, while a contradiction says one of the two ends is wrong, and that
  deserves proof of recovery rather than the mere absence of the symptom.

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
for the roles, plus `/global/system/cliff_tof/health` carrying `scbdriver/CliffTofSafetyHealth`. That
message carries the driver's own readiness decision, and the consumer may release the cliff stop only
while it reads `READY`; its `reason_mask` explains a decision and must not be used to re-derive one.
**Its heartbeat counter is judged new by inequality with the previous value, never by numeric increase**,
so the 32-bit wrap is an ordinary new snapshot rather than a regression — the same reasoning that makes
`cycle_seq` an equality comparison rather than a magnitude one. The driver normalises strictly before
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

## Commissioning timing profile — model-derived, NOT measured

**Profile name: `commissioning-cliff-only-400k`.** Every value below is arithmetic from the byte
counts and the bus rate, or a stated design choice. **None of them is a measurement.** A production
configuration MUST NOT inherit these defaults; it must carry its own profile, resolved from the
six-board schedule measurement, under a later contract version.

The profile is defined for one specific configuration and is void outside it: **four VL53L4CX
measurement reads and no VL53L7CX reads on the bus, i2c2 at 400 kHz.** Adding real grid reads changes
every number here — a trimmed 328-byte grid read is 7.45 ms of blocking and an untrimmed 1452-byte read
is 32.7 ms — so a configuration with live L7 traffic requires re-derivation, not reuse.

Arithmetic basis, at 400 kHz with 9 bits per byte and 3 bytes of address and register-index overhead
per transfer: an L4 result read of 133 B is **3.06 ms**, its re-arm of 2 B is **0.11 ms**, so serving all
four sensors once costs **12.7 ms** of bus time. The same arithmetic reproduces the independently known
32.7 ms figure for an untrimmed grid read, which is what validates it.

| Symbol | Profile value | Where it comes from |
| --- | --- | --- |
| `T_cycle_nominal` | 50 ms | 12.7 ms of bus work plus a 20 ms measurement timing budget, rounded up. The timing budget itself is not fixed yet, so this moves with it |
| `T_skew_max` | 20 ms | worst spread of the four samples inside one cycle = the 12.7 ms needed to serve all four back to back, rounded up. Valid only because this profile has no grid reads interleaved |
| `T_health_delivery_max` | 20 ms | analysis, not measurement: one frame on a 1 Mbit/s bus is sub-millisecond on the wire, and 20 ms covers queueing behind the existing control and safety traffic |
| `T_health_nominal` | 100 ms | design choice, 10 Hz. Deliberately decoupled from the acquisition cycle, because health must not wait for one |
| `T_health_max_gap` | 300 ms | 3 x `T_health_nominal`, and strictly greater than `T_health_delivery_max` as required |
| `T_cycle_assembly` | 100 ms | satisfies `T_skew_max + T_health_delivery_max` (40 ms) `< T_cycle_assembly <` `T_meas_max_gap` with margin at both ends |
| `T_startup_health_grace` | 10 s | covers ROS node start, SCB boot and the first heartbeat. Generous on purpose: it only delays the `NOT_READY` to `FAULT` transition, and `NOT_READY` already withholds motion |
| `T_meas_max_gap` | 200 ms — **PLACEHOLDER** | **This one is not derived at all.** The contract requires it to come from the stopping distance, and this revision does not have the vehicle's speed and deceleration figures. 200 ms is a placeholder chosen to sit above `T_cycle_assembly` and below any plausible safety budget. **It must be replaced before any production use, and it must not be back-derived from the cycle period** — an earlier draft's `N x T_cycle_nominal <= T_meas_max_gap` was wrong in the unsafe direction |

| Symbol | Profile value | Where it comes from |
| --- | --- | --- |
| `N_cycle_miss_fault` | 3 | design choice. **Auxiliary gate only.** It does not extend, weaken or substitute for `T_meas_max_gap`, which stays the hard limit on a monotonic clock and is the only bound the safety argument uses. One missed cycle must not fault a source, because a single retryable NACK on a six-device chain would otherwise stop the robot |
| `N_cycle_advance_max` | 16 | design choice: large enough to ride out a burst of dropped cycles, small enough that a wrap-aliased or garbage value is rejected rather than accepted as a huge forward jump |

### What proves `mapping_state == PROVEN` under this profile

During commissioning, `PROVEN` may be asserted only on the evidence of the on-machine enable-chain
gate, and only these three results count:

- all six chain positions individually addressed, each verified by a **type-appropriate** identity read
  (`VL53L7CX` at positions 1-2, `VL53L4CX` at 3-6) at six mutually distinct addresses
- **nothing left at the default address** after enumeration
- **tail isolation**: with only position 6 enabled, position 6 answers **its own** address, returns the
  expected `VL53L4CX` identity, and **position 5's address does not answer**. If position 6 answers
  position 5's address, two devices were written to one address — a silent merge, and the exact failure
  this gate exists to catch. The negative half is not redundant: an address that still answers while its
  device is supposed to be disabled means the isolation itself did not take, and then the positive half
  proves nothing.

A bus-wide address scan count is **not** admissible evidence. It was observed on DS20001 to
intermittently miss the two grid sensors' addresses inside a 112-address sweep while those same devices
passed 200/200 back-to-back probes and 100/100 register transfers with payload comparison. The cause is
unexplained; two hypotheses — degradation of the transaction following a NACK, and a back-to-back rate
effect — were each tested and disproved. An unexplained, intermittent measurement cannot gate a safety
mapping claim.

### The proof is a transaction, because the isolated chain is not the chain that produces data

The three results above cannot all come from one chain state, and no implementation should be written as
if they could. Tail isolation requires positions 3-5 to be disabled, and on the L4 carriers the enable
line is reset-class — disabling a position returns that device to the default address. So the chain that
carries the live addresses is **always** a chain enumerated *after* the isolation, and it can never
itself hold an isolation result. An implementation that tries to satisfy all three from the final state
will fail every time, and the tempting repair is to weaken one criterion.

The proof is therefore defined as one **transaction**, in this order:

- **walk 1** — a full enumeration of all six positions, reaching `complete`
- **isolation** — the tail-isolation result above, which destroys the addresses of positions 3-5
- **walk 2** — a second full enumeration, reaching `complete`, which restores the four L4 addresses and
  is the chain the acquisition path then uses

What transfers the isolation evidence from walk 1 to walk 2 is not an unstated assumption that nothing
changed. It is an explicit **semantic fingerprint** that both walks must produce identically. The
fingerprint is, per position: the position index, the expected model, the assigned target address, the
observed identity bytes, the logical `source_id` or role, and `verified`.

Normalisation is part of the definition, because the raw per-position verdicts legitimately differ
between the two walks:

- **Positions 1-2 (`VL53L7CX`)**: `enumerated` and `retained` both normalise to `verified`. The L7 keeps
  its assigned address across an enable-low while powered, so walk 2 finds it already at its target and
  reports `retained` where walk 1 reported `enumerated`. Requiring identical raw verdicts would fail on
  every healthy chain.
- **Positions 3-6 (`VL53L4CX`)**: both walks MUST report `enumerated`. A `retained` L4 contradicts the
  reset-class enable and is a fault, not a normalisation case.
- **Both walks MUST be `complete`**, and the fingerprints MUST match on every field above.
- **Not compared**: pulse counts, control history, and the raw verdict values themselves. They are
  diagnostics; two walks may reach the same proven configuration by different pulse counts, and demanding
  equality there manufactures failures with no bearing on the mapping.

Two further obligations follow from the transaction being the unit:

- **A partial transaction authorises nothing.** If any of the three steps fails or is skipped, no proof
  exists; there is no such thing as "walk 1 was clean, so proceed".
- **`PROVEN` is unreachable while any cliff position's logical role is unknown.** The masks a consumer
  reads are keyed by `source_id`, not by chain position, so a firmware without the frozen role table
  cannot fill them and cannot claim a proven position-to-`source_id` mapping. Electrical enumeration
  proves the *type* sequence; it cannot prove which of four identical carriers is mounted where.

## Commissioning `mapping_epoch` issuance — the host is the authority, and what that costs

The firmware obligation stated in *The acquisition cycle is the unit of correlation* — a new
`mapping_epoch` on every restart of the cliff subsystem — is unchanged. Under this profile **who
discharges it changes**, and that has to be written down rather than assumed, because the firmware has no
persistent store: the SCB application has `CONFIG_FLASH` and `CONFIG_FLASH_MAP` and no NVS or settings
partition, so it cannot remember an epoch across a power cycle at all.

Under `commissioning-cliff-only-400k`:

- The **commissioning host is the issuing authority**. `mapping_epoch` is supplied to the firmware as
  part of the operator-initiated proof, and the host is the component that persists it.
- The host MUST store the last epoch it issued, increment it **modulo 256** for the next proof, and record
  both the old and the new epoch in the commissioning evidence for that machine.
- The **firmware** guarantees exactly three things, and no more: it refuses an epoch equal to any it has
  already used since power-on; it advances the epoch and resets `cycle_seq` to 0 in **one** transaction;
  and it stays non-`PROVEN` if issuance fails for any reason.
- A restart returns the subsystem to `UNKNOWN`. No measurement frame can be transmitted until a fresh
  proof supplies a fresh epoch, which is what keeps a post-restart `cycle_seq` from aliasing onto the
  sequence that came before it.

**The limitation, stated plainly: cross-restart uniqueness now rests on procedure, not on firmware.** The
firmware does not claim it and must not be described as providing it. The property holds because a
restart cannot reach `PROVEN` without an operator, and it would stop holding the moment anything proves
the mapping automatically. A production configuration that proves on boot therefore requires a
firmware-side persistent epoch issuer, and that is a release blocker, not a refinement.

## Timing values — what the production release still has to resolve

The values above are a commissioning profile. The following remain **required fields with no measured
value**, and they cannot be established offline: they are outputs of the L4 acquisition thread and of the
six-board schedule measured on real hardware.

| Symbol | Meaning |
| --- | --- |
| `T_cycle_nominal` | nominal acquisition-cycle period, which is what a health mask describes |
| `T_meas_max_gap` | maximum tolerable gap between accepted measurements of one sensor |
| `T_cycle_assembly` | how long one cycle slot may stay open before it expires |
| `T_startup_health_grace` | how long after node start the absence of any health frame is `NOT_READY` rather than `FAULT` |
| `T_health_nominal` | nominal health snapshot period |
| `T_health_max_gap` | maximum tolerable gap between consecutive **new** `health_seq` values |
| `T_skew_max` | worst-case sampling phase skew across the four sensors within a cycle |
| `T_health_delivery_max` | worst-case health frame latency from production to reception under full bus load |
| `N_cycle_miss_fault` | consecutive cycles without a sample from one source before it faults |
| `N_cycle_advance_max` | largest plausible forward jump in `cycle_seq` before it is treated as implausible |

Six rules constrain the eventual numbers rather than the schedule:

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
- **`T_startup_health_grace` must cover node start, SCB boot and the worst-case arrival of the first
  health heartbeat** — and nothing more. It must **not** wait for an acquisition cycle: health is
  transmitted unconditionally once the subsystem is enabled, including with zero sensors enumerated and
  `mapping_state == UNKNOWN`, so the first heartbeat does not depend on any measurement. Sizing the grace
  around a cycle would lengthen the detection of a dead link for no reason. It is unrelated to
  `T_health_max_gap`, and it has no runnable default: unset means `CONFIG_ERROR`, like the other
  timeouts.
- **`N_cycle_miss_fault` is not tied to any period.** It is a cycle count with a fault outcome and no
  timing claim; `T_meas_max_gap` on a monotonic clock is the only bound the safety argument uses. Do not
  reintroduce a product of a count and a nominal period, because cycle periods stretch under load.

## Decoder events

Every scenario declares the **complete multiset** of events it must produce, so an implementation cannot
pass by emitting the right event alongside wrong ones. That rule needs a fixed vocabulary, or two
implementations will report the same case under different names and the completeness requirement will
have nothing to bite on. These are the names, grouped by what they describe:

| Group | Events |
| --- | --- |
| Measurement | `MEASUREMENT_ACCEPTED`, `MEASUREMENT_BUFFERED`, `MEASUREMENT_DROPPED_UNAUTHORISED`, `DUPLICATE_MEASUREMENT_IDENTICAL`, `CONFLICTING_MEASUREMENT` |
| Health | `HEALTH_ACCEPTED`, `HEALTH_HEARTBEAT_ONLY`, `HEALTH_REPEAT_IGNORED`, `HEALTH_STALE`, `HEALTH_RECOVERED` |
| Cycle | `CYCLE_COMPLETED`, `CYCLE_INCOMPLETE_BY_TIMEOUT`, `CYCLE_RETIRED_BY_NEWER`, `FRAME_FOR_RETIRED_CYCLE`, `IMPLAUSIBLE_CYCLE_ADVANCE` |
| Mapping | `EPOCH_CHANGED`, `MAPPING_LOST`, `ROLE_PUBLICATION_SUPPRESSED` |
| Rejection | `MALFORMED_FRAME`, `CONTRADICTION_STATUS_SENTINEL`, `CONTRADICTION_MASK_VS_MEASUREMENT`, `CONTRADICTION_POSITION_WITHOUT_FAULT`, `CONTRADICTION_TARGET_COUNT`, `VERSION_UNSUPPORTED` |
| Fault state | `PROTOCOL_FAULT_RAISED`, `PROTOCOL_FAULT_CLEARED`, `CONFIG_ERROR` |
| Source and readiness | `READY_ENTERED`, `READY_LOST`, `SOURCE_DEGRADED`, `SOURCE_STALE`, `SOURCE_RECOVERED`, `SAMPLE_MISS_FAULT` |

Two properties are normative rather than incidental. Alarms are **edge triggered**: a condition that has
not changed produces no further event, which is why a boundary scenario at `T+1` expects nothing rather
than a repeat. And draining the event queue **removes** the events, so each is processed exactly once
instead of the whole history replaying on every spin.

## Golden vectors

Vectors do **not** exist yet; generating them is the step after the first freeze. They will come from
`gen_cliff_golden_vectors.py`, emitting a JSON file and a dependency-free C++ header, both carrying the
SHA-256 of **this** file, pinned independently by the firmware packer test and the driver decoder test.
The grid contract's generator, vectors and SHA are untouched.

**The scenario catalogue is already written, and the generator refuses to emit.** `--list` renders every
scenario with its input sequence, its complete expected event multiset, its publication outcome, the
parameters it depends on and whether it is blocked on hardware; `--check` verifies the catalogue's
self-consistency; and plain invocation fails while the contract version carries a `draft-` marker or any
symbol is unresolved. Scenarios refer to identifiers and timings **by symbol**, never by number, so
resolving a value is one edit rather than a sweep. Writing the catalogue before the numbers exist is
what exposed the two boundary scenarios below.

**One comparison rule is frozen now**, because it decides what a boundary case means and no measurement
can change it:

> An age strictly below its bound is fresh. An age equal to or above it has timed out.

Every timing scenario therefore comes as a triple at `T-1`, `T` and `T+1`, where `T` is the **first
failing** case rather than the last passing one.

The catalogue's lower-bound case for `T_cycle_assembly` is the one the rules above imply but no earlier
draft stated: measurements spread across `T_skew_max` inside one cycle, with that cycle's health frame
arriving a further `T_health_delivery_max` later, must still complete. If a slot expired there, cycles
would die while their own frames were still legitimately in flight.

Beyond the happy path the vectors MUST cover at least:

- every one of the sixteen raw statuses, mapped to its class, including all four `RANGE_VALID_*` codes
  and both `NO_SAMPLE` codes
- one source missing from a cycle; all four missing; a cycle with no measurements at all
- `mapping_state` `UNKNOWN`, `PROVEN`, `LOST`, and recovery back to `PROVEN`
- health stopping while measurements continue, and `health_seq` repeated
- a measurement arriving before any health snapshot at all — cold start
- a measurement whose **own correlated cycle's** health frame reports `UNKNOWN`, `LOST` or `FAULT`, which
  is a firmware fault and must produce protocol FAULT. Note precisely what this is **not**: a measurement
  that merely *arrives* while the newest health frame reports `LOST` is legal, and on real hardware it is
  routine. The firmware's measurement flush and its health snapshot are deliberately not serialised
  (serialising them would put four bounded send waits in front of the heartbeat), so a cycle that was
  authorised while `PROVEN` can reach the bus after the snapshot that revokes the mapping. The
  discriminator is *which* health frame authorises the measurement, never arrival order — see
  *Validation, and protocol fault*, whose rule is normative and unchanged. The benign case has its own
  required vector further down this list ("a late measurement from a `PROVEN` cycle arriving after a newer
  health frame reports `LOST`"); the two entries are the same distinction from opposite sides, and a
  decoder that passes one and fails the other has implemented arrival order.
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
- both halves of the biconditional violated: `target_count = 0` with a finite range or a status other
  than `255`, and status `255` with a non-zero `target_count`
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

- **The health CAN identifier's registration.** The identifier itself is settled for commissioning:
  `0x217`, self-assigned 2026-08-17 after a fresh sweep of both repositories and a live `can1` capture.
  What is **still open** is the part the contract says actually settles it — the row in the team's CAN
  ID register. A silent device never appears in a capture and a sweep cannot see code we do not hold,
  so neither piece of evidence closes it. `0x214`/`0x215`/`0x216` owe their rows as well. **Registration
  must be closed before any production release**, and it is the one item in this list that costs nothing
  but a message to the team.
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
