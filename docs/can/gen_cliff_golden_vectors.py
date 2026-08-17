#!/usr/bin/env python3
"""Golden vector generator for the cliff ToF CAN wire contract (AMRSW-2994).

This is a **skeleton with a complete scenario catalogue and no output**. It cannot
emit vectors yet, and refuses to try, because the contract it would generate from
is still a draft with unresolved values. What it does carry is every scenario the
vectors will contain, with its input sequence, its complete expected event
multiset, its publication outcome, the parameters it depends on and whether it is
blocked on hardware.

The point of writing the catalogue before the numbers exist: the scenarios are
where boundary gaps show up, and finding them now is cheap. Once the live capture
allocates the health identifier, the schedule measurement fixes the timing values
and the two provisional status rows are validated on hardware, the remaining work
is to resolve the symbols, emit, and pin the SHA on both sides - not to design the
cases.

Usage:
    gen_cliff_golden_vectors.py --list     print the scenario catalogue
    gen_cliff_golden_vectors.py --check    verify the catalogue's self-consistency
    gen_cliff_golden_vectors.py           attempt generation (refuses, and says why)

Nothing here reads or writes the grid contract's artefacts.
"""

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONTRACT = HERE / "tof_cliff_wire_contract.md"

# ---------------------------------------------------------------------------
# The one comparison rule every timing scenario is written against.
#
# Frozen now, deliberately, because it decides what T-1 / T / T+1 mean and no
# measured value can change it: an age strictly below the bound is fresh, an age
# equal to or above it has timed out. Boundary scenarios therefore always come in
# threes, and the middle one is the first failing case rather than the last
# passing one.
COMPARISON_RULE = "age < T is fresh; age >= T has timed out"

# ---------------------------------------------------------------------------
# Unresolved symbols. Scenarios refer to these by name and never to a number, so
# that resolving one is an edit here rather than a sweep through the catalogue.

# Still unresolved FOR RELEASE. The commissioning profile below supplies a value for
# each so the layout artefacts can be emitted, but a value from a profile is not a
# measurement and the release ban stays until these come from the six-board schedule.
UNRESOLVED = {
    "T_cycle_nominal": "output of the six-board schedule measurement",
    "T_meas_max_gap": "derived from stopping distance, needs the schedule measurement",
    "T_cycle_assembly": "bounded by T_skew_max + T_health_delivery_max and T_meas_max_gap",
    "T_health_nominal": "output of the acquisition thread design",
    "T_health_max_gap": "must exceed T_health_delivery_max, measured under full bus load",
    "T_health_delivery_max": "worst-case health latency, measured or analysed on the real bus",
    "T_startup_health_grace": "node start + SCB boot + worst-case first heartbeat arrival",
    "T_skew_max": "worst-case intra-cycle phase skew across the four sensors",
}

RESOLVED = {
    "TOF_CLIFF_MEAS_ID": 0x216,
    # Allocated 2026-08-17 by the same team-authorised self-assignment as 0x214/0x215/0x216.
    # Evidence is the source scan of both repositories (highest assigned is 0x216, 0x217
    # unclaimed in either tree); the live can1 capture only agrees with it. A capture cannot
    # prove an identifier unused.
    "TOF_CLIFF_HEALTH_ID": 0x217,
    "PROTOCOL_VERSION": 0x1,
    "SENTINEL_INVALID": 0xFFFF,
    "SOURCE_COUNT": 4,
    "CHAIN_POSITION_NONE": 0xFF,
    # Decisions, not measurements. N_cycle_miss_fault is an AUXILIARY gate: it does not
    # extend or substitute for T_meas_max_gap, which stays the hard limit on a monotonic clock.
    "N_cycle_miss_fault": 3,
    "N_cycle_advance_max": 16,
    # The classification table already specifies both of these. Authorising them for
    # commissioning is not a claim that either has been validated -- see VALIDATION_PENDING.
    "STATUS_CLASS_3": "SENSOR_FAULT",
    "STATUS_CLASS_11": "NO_TARGET",
}

# Values carried by this revision that have NOT been validated on hardware. Emitted into
# every artefact so a consumer cannot mistake authorisation for validation.
VALIDATION_PENDING = {
    "STATUS_CLASS_3": "min-range-clipped as SENSOR_FAULT: needs the as-mounted floor distance, "
                      "the rate over a normal floor, and an occlusion injection",
    "STATUS_CLASS_11": "merged-pulse as NO_TARGET: needs the rate over a real edge and over "
                       "plain floor. Frequent on plain floor is a configuration problem",
    "T_meas_max_gap": "the profile value is a PLACEHOLDER, not derived from stopping distance",
}

# ---------------------------------------------------------------------------
# Commissioning timing profile. Model-derived, NOT measured. A production
# configuration must not inherit these; see the contract's profile section.

PROFILE_NAME = "commissioning-cliff-only-400k"
RELEASE_FORBIDDEN = True

PROFILE = {
    "T_cycle_nominal_ms": 50,
    "T_skew_max_ms": 20,
    "T_health_delivery_max_ms": 20,
    "T_health_nominal_ms": 100,
    "T_health_max_gap_ms": 300,
    "T_cycle_assembly_ms": 100,
    "T_startup_health_grace_ms": 10000,
    "T_meas_max_gap_ms": 200,
}

PROFILE_PROVENANCE = "arithmetic from byte counts at 400 kHz (L4 read 133 B = 3.06 ms, re-arm " \
                     "2 B = 0.11 ms, four sensors = 12.7 ms of bus time) plus stated design " \
                     "choices. Valid ONLY for four L4 reads and no L7 reads on the bus."

# ---------------------------------------------------------------------------
# Event vocabulary.
#
# Normative as of draft-2026-08-11f: the contract's "Decoder events" section
# enumerates these, and this list must stay identical to it. Two properties come
# with them - alarms are edge triggered, so a persisting condition produces nothing
# further, and draining the queue removes the events.

EVENTS = [
    "MEASUREMENT_ACCEPTED",
    "MEASUREMENT_BUFFERED",
    "MEASUREMENT_DROPPED_UNAUTHORISED",
    "DUPLICATE_MEASUREMENT_IDENTICAL",
    "CONFLICTING_MEASUREMENT",
    "HEALTH_ACCEPTED",
    "HEALTH_HEARTBEAT_ONLY",
    "HEALTH_REPEAT_IGNORED",
    "CYCLE_COMPLETED",
    "CYCLE_INCOMPLETE_BY_TIMEOUT",
    "CYCLE_RETIRED_BY_NEWER",
    "FRAME_FOR_RETIRED_CYCLE",
    "IMPLAUSIBLE_CYCLE_ADVANCE",
    "EPOCH_CHANGED",
    "MALFORMED_FRAME",
    "CONTRADICTION_STATUS_SENTINEL",
    "CONTRADICTION_MASK_VS_MEASUREMENT",
    "CONTRADICTION_POSITION_WITHOUT_FAULT",
    "CONTRADICTION_TARGET_COUNT",
    "PROTOCOL_FAULT_RAISED",
    "PROTOCOL_FAULT_CLEARED",
    "VERSION_UNSUPPORTED",
    "CONFIG_ERROR",
    "READY_ENTERED",
    "READY_LOST",
    "SOURCE_DEGRADED",
    "SOURCE_STALE",
    "SOURCE_RECOVERED",
    "SAMPLE_MISS_FAULT",
    "HEALTH_STALE",
    "HEALTH_RECOVERED",
    "MAPPING_LOST",
    "ROLE_PUBLICATION_SUPPRESSED",
]

PUBLICATIONS = ("NONE", "HEALTH_ONLY", "FOUR_ROLE_RANGES")
BLOCKERS = ("hardware_validation_pending",)

# ---------------------------------------------------------------------------
# Shorthands for the input sequences. These are notation, not an implementation:
# the real generator will parse the same shapes.

HEALTH_OK = "health(epoch=1, cycle=0, PROVEN, cycle_valid=1, enum=0xF, model=0xF, produced=0xF, fault=0x0)"
FOUR_VALID = "meas(src=0..3, epoch=1, cycle=0, VALID, finite, targets=1)"


def S(**kw):
    kw.setdefault("params", [])
    kw.setdefault("blocked", None)
    kw.setdefault("notes", "")
    kw.setdefault("allow_no_events", False)
    return kw


CATALOGUE = [
    # -- A. baseline ---------------------------------------------------------
    S(id="A1", group="baseline", title="one complete cycle reaches READY",
      inputs=[FOUR_VALID, HEALTH_OK],
      events=["MEASUREMENT_BUFFERED"] * 4 + ["HEALTH_ACCEPTED", "CYCLE_COMPLETED",
                                            "MEASUREMENT_ACCEPTED"] + ["READY_ENTERED"],
      publication="FOUR_ROLE_RANGES",
      notes="measurements precede health, which is the intended transmit order"),
    S(id="A2", group="baseline", title="health first, then its measurements",
      inputs=[HEALTH_OK, FOUR_VALID],
      events=["HEALTH_ACCEPTED"] + ["MEASUREMENT_ACCEPTED"] * 4 + ["CYCLE_COMPLETED", "READY_ENTERED"],
      publication="FOUR_ROLE_RANGES",
      notes="the two identifiers arbitrate independently; both orders must work"),
    S(id="A3", group="baseline", title="three consecutive healthy cycles stay READY",
      inputs=["cycle(0) complete", "cycle(1) complete", "cycle(2) complete"],
      events=["CYCLE_COMPLETED"] * 3 + ["READY_ENTERED", "CYCLE_RETIRED_BY_NEWER"],
      publication="FOUR_ROLE_RANGES", params=["T_cycle_nominal"],
      notes="cycles spaced at the nominal period; READY_ENTERED fires once, not per cycle, "
            "and retirement happens when the third opens"),

    # -- B. status classification (every raw code) ---------------------------
    S(id="B0", group="status", title="status 0 RANGE_VALID publishes a finite range",
      inputs=["meas(src=0, VALID, range=1234, targets=1)"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES"),
    S(id="B1", group="status", title="status 1 SIGMA_FAIL becomes NO_TARGET",
      inputs=["meas(src=0, status=1, range=SENTINEL_INVALID, targets=1)"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES",
      notes="stays READY; the far-side value is what stops the robot"),
    S(id="B2", group="status", title="status 2 SIGNAL_FAIL becomes NO_TARGET",
      inputs=["meas(src=0, status=2, range=SENTINEL_INVALID, targets=1)"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES",
      notes="the signature of a real drop-off"),
    S(id="B3", group="status", title="status 3 MIN_RANGE_CLIPPED is a sensor fault",
      inputs=["meas(src=0, status=3, range=SENTINEL_INVALID, targets=1)",
              "health(..., fault=0x1)"],
      events=["MEASUREMENT_ACCEPTED", "READY_LOST"], publication="HEALTH_ONLY",
      params=["STATUS_CLASS_3"], blocked="hardware_validation_pending",
      notes="provisional: a blocked lens looks the same as a very near floor"),
    S(id="B4", group="status", title="status 4 OUTOFBOUNDS_FAIL becomes NO_TARGET",
      inputs=["meas(src=0, status=4, range=SENTINEL_INVALID, targets=1)"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES"),
    S(id="B5", group="status", title="status 5 HARDWARE_FAIL is a sensor fault",
      inputs=["meas(src=0, status=5, range=SENTINEL_INVALID, targets=1)", "health(..., fault=0x1)"],
      events=["MEASUREMENT_ACCEPTED", "READY_LOST"], publication="HEALTH_ONLY"),
    S(id="B6", group="status", title="status 6 NO_WRAP_CHECK sends no frame at all",
      inputs=["read(src=0) yields status 6", "health(..., produced=0xE)"],
      events=["HEALTH_ACCEPTED", "CYCLE_COMPLETED", "SOURCE_DEGRADED"],
      publication="HEALTH_ONLY",
      notes="NO_SAMPLE: start-up artefact, mask bit clear, no measurement frame"),
    S(id="B7", group="status", title="status 7 WRAP_TARGET_FAIL becomes NO_TARGET",
      inputs=["meas(src=0, status=7, range=SENTINEL_INVALID, targets=1)"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES"),
    S(id="B8", group="status", title="status 8 PROCESSING_FAIL is a sensor fault",
      inputs=["meas(src=0, status=8, range=SENTINEL_INVALID, targets=1)", "health(..., fault=0x1)"],
      events=["MEASUREMENT_ACCEPTED", "READY_LOST"], publication="HEALTH_ONLY"),
    S(id="B9", group="status", title="status 9 XTALK_SIGNAL_FAIL is a sensor fault",
      inputs=["meas(src=0, status=9, range=SENTINEL_INVALID, targets=1)", "health(..., fault=0x1)"],
      events=["MEASUREMENT_ACCEPTED", "READY_LOST"], publication="HEALTH_ONLY"),
    S(id="B10", group="status", title="status 10 SYNCRONISATION_INT sends no frame at all",
      inputs=["read(src=0) yields status 10", "health(..., produced=0xE)"],
      events=["HEALTH_ACCEPTED", "CYCLE_COMPLETED", "SOURCE_DEGRADED"],
      publication="HEALTH_ONLY", notes="NO_SAMPLE: the first interrupt after starting ranging"),
    S(id="B11", group="status", title="status 11 MERGED_PULSE becomes NO_TARGET",
      inputs=["meas(src=0, status=11, range=SENTINEL_INVALID, targets=2)"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES",
      params=["STATUS_CLASS_11"], blocked="hardware_validation_pending",
      notes="provisional: a step edge is exactly what merges returns"),
    S(id="B12", group="status", title="status 12 TARGET_PRESENT_LACK_OF_SIGNAL becomes NO_TARGET",
      inputs=["meas(src=0, status=12, range=SENTINEL_INVALID, targets=1)"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES"),
    S(id="B13", group="status", title="status 13 MIN_RANGE_FAIL is a sensor fault",
      inputs=["meas(src=0, status=13, range=SENTINEL_INVALID, targets=1)", "health(..., fault=0x1)"],
      events=["MEASUREMENT_ACCEPTED", "READY_LOST"], publication="HEALTH_ONLY",
      notes="set from DEVICEERROR_USERROICLIP: an ROI or configuration anomaly"),
    S(id="B14", group="status", title="status 14 RANGE_INVALID is a sensor fault",
      inputs=["meas(src=0, status=14, range=SENTINEL_INVALID, targets=1)", "health(..., fault=0x1)"],
      events=["MEASUREMENT_ACCEPTED", "READY_LOST"], publication="HEALTH_ONLY"),
    S(id="B255", group="status", title="status 255 NONE is a no-target result, not a missing sample",
      inputs=["meas(src=0, status=255, range=SENTINEL_INVALID, targets=0)"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES",
      notes="the packer must map it to the sentinel and never forward 8191 mm"),
    S(id="B-VALID-SENTINEL", group="status",
      title="a VALID status carrying the sentinel is a contradiction",
      inputs=["meas(src=0, status=0, range=SENTINEL_INVALID, targets=1)"],
      events=["CONTRADICTION_STATUS_SENTINEL", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY"),
    S(id="B-NOTARGET-FINITE", group="status",
      title="a NO_TARGET status carrying a finite range is a contradiction",
      inputs=["meas(src=0, status=2, range=1000, targets=1)"],
      events=["CONTRADICTION_STATUS_SENTINEL", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY"),

    # -- C. multi-target reduction ------------------------------------------
    S(id="C0", group="reduction", title="zero targets: status 255, sentinel, target_count 0",
      inputs=["read(src=0) yields 0 targets"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES"),
    S(id="C1", group="reduction", title="one valid target passes through",
      inputs=["read(src=0) yields [VALID 900]"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES"),
    S(id="C2", group="reduction", title="two valid targets: the farther one is transmitted",
      inputs=["read(src=0) yields [VALID 400, VALID 900]"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES",
      notes="the inverse of the grid path's per-zone minimum; a near return must not mask a drop"),
    S(id="C3", group="reduction", title="a faulty target condemns the whole measurement",
      inputs=["read(src=0) yields [VALID 900, SENSOR_FAULT]"],
      events=["MEASUREMENT_ACCEPTED", "READY_LOST"], publication="HEALTH_ONLY",
      blocked="hardware_validation_pending",
      notes="conservative by design; the mixed-status rate on a real floor is unmeasured"),
    S(id="C4", group="reduction", title="valid alongside no-target reduces to no-target",
      inputs=["read(src=0) yields [VALID 900, NO_TARGET]"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES",
      blocked="hardware_validation_pending"),
    S(id="C5", group="reduction", title="four targets, all valid: the farthest is transmitted",
      inputs=["read(src=0) yields [VALID 300, VALID 500, VALID 700, VALID 1100]"],
      events=["MEASUREMENT_ACCEPTED"], publication="FOUR_ROLE_RANGES",
      blocked="hardware_validation_pending"),
    S(id="C6", group="reduction", title="target_count above four is malformed",
      inputs=["meas(src=0, targets=5)"],
      events=["MALFORMED_FRAME", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY"),
    S(id="C7", group="reduction", title="target_count 0 with a finite range violates the biconditional",
      inputs=["meas(src=0, status=0, range=1000, targets=0)"],
      events=["CONTRADICTION_TARGET_COUNT", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY"),
    S(id="C8", group="reduction", title="status 255 with a non-zero target_count violates it too",
      inputs=["meas(src=0, status=255, range=SENTINEL_INVALID, targets=2)"],
      events=["CONTRADICTION_TARGET_COUNT", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY",
      notes="both halves of the biconditional have a case"),

    # -- D. cycle assembly --------------------------------------------------
    S(id="D1", group="cycle", title="a cycle whose mask implies three samples completes with three",
      inputs=["meas(src=0..2, cycle=0)", "health(cycle=0, produced=0x7)"],
      events=["MEASUREMENT_BUFFERED"] * 3 + ["HEALTH_ACCEPTED", "CYCLE_COMPLETED",
                                            "MEASUREMENT_ACCEPTED", "SOURCE_DEGRADED"],
      publication="HEALTH_ONLY", notes="three of four fresh is NOT_READY, so no role data"),
    S(id="D2", group="cycle", title="mask claims a sample that never arrives: timeout is a protocol fault",
      inputs=["meas(src=0..2, cycle=0)", "health(cycle=0, produced=0xF)",
              "advance_clock(T_cycle_assembly)"],
      events=["MEASUREMENT_BUFFERED"] * 3 + ["HEALTH_ACCEPTED", "CYCLE_INCOMPLETE_BY_TIMEOUT",
                                            "CONTRADICTION_MASK_VS_MEASUREMENT",
                                            "PROTOCOL_FAULT_RAISED"],
      publication="HEALTH_ONLY", params=["T_cycle_assembly"],
      notes="health asserted data the wire never carried"),
    S(id="D3", group="cycle", title="a measurement with the mask bit clear is a contradiction",
      inputs=["meas(src=3, cycle=0)", "health(cycle=0, produced=0x7)"],
      events=["MEASUREMENT_BUFFERED", "HEALTH_ACCEPTED",
              "CONTRADICTION_MASK_VS_MEASUREMENT", "PROTOCOL_FAULT_RAISED"],
      publication="HEALTH_ONLY", notes="the converse direction of D2"),
    S(id="D4", group="cycle", title="no health for a cycle: expiry is degraded, not a fault",
      inputs=["meas(src=0..3, cycle=0)", "advance_clock(T_cycle_assembly)"],
      events=["MEASUREMENT_BUFFERED"] * 4 + ["CYCLE_INCOMPLETE_BY_TIMEOUT"],
      publication="NONE", params=["T_cycle_assembly"],
      notes="one lost health frame is a transport event; persistence is caught by T_health_max_gap"),
    S(id="D5", group="cycle", title="a third cycle retires the oldest open one",
      inputs=["cycle(0) partial", "cycle(1) partial", "cycle(2) opens"],
      events=["MEASUREMENT_BUFFERED"] * 3 + ["CYCLE_RETIRED_BY_NEWER"],
      publication="NONE", notes="at most two open cycles per epoch"),
    S(id="D6", group="cycle", title="a frame for a retired cycle is dropped, not faulted",
      inputs=["cycle(0) completed", "cycle(1) completed", "cycle(2) completed",
              "meas(src=0, cycle=0)"],
      events=["CYCLE_COMPLETED"] * 3 + ["READY_ENTERED", "CYCLE_RETIRED_BY_NEWER",
                                        "FRAME_FOR_RETIRED_CYCLE"],
      publication="FOUR_ROLE_RANGES",
      notes="it can neither reopen the cycle nor refresh freshness"),
    S(id="D7", group="cycle", title="an identical duplicate measurement is a benign repeat",
      inputs=["meas(src=0, cycle=0, VALID 900)", "meas(src=0, cycle=0, VALID 900)"],
      events=["MEASUREMENT_BUFFERED", "DUPLICATE_MEASUREMENT_IDENTICAL"], publication="NONE"),
    S(id="D8", group="cycle", title="two different measurements for one triple is a conflict",
      inputs=["meas(src=0, cycle=0, VALID 900)", "meas(src=0, cycle=0, VALID 400)"],
      events=["MEASUREMENT_BUFFERED", "CONFLICTING_MEASUREMENT", "PROTOCOL_FAULT_RAISED"],
      publication="HEALTH_ONLY"),

    S(id="D9", group="cycle",
      title="worst-case skew plus worst-case health latency still completes the cycle",
      inputs=["meas(src=0, cycle=0) at t0",
              "meas(src=3, cycle=0) at t0 + T_skew_max",
              "health(cycle=0, produced=0xF) at t0 + T_skew_max + T_health_delivery_max"],
      events=["MEASUREMENT_BUFFERED"] * 4 + ["HEALTH_ACCEPTED", "CYCLE_COMPLETED"] +
             ["MEASUREMENT_ACCEPTED"] + ["READY_ENTERED"],
      publication="FOUR_ROLE_RANGES",
      params=["T_skew_max", "T_health_delivery_max", "T_cycle_assembly"],
      notes="this is the lower bound on T_cycle_assembly made executable: if the slot expired "
            "here, cycles would die while their own frames were still legitimately in flight"),

    # -- E. cycle_seq comparison and wrap ----------------------------------
    S(id="E1", group="wrap", title="delta of one opens a new cycle",
      inputs=["cycle(5) complete", "health(cycle=6)"],
      events=["CYCLE_COMPLETED", "READY_ENTERED", "HEALTH_ACCEPTED"],
      publication="FOUR_ROLE_RANGES"),
    S(id="E2", group="wrap", title="delta of minus one still lands in the previous open cycle",
      inputs=["health(cycle=6)", "meas(src=0, cycle=5)"],
      events=["HEALTH_ACCEPTED", "MEASUREMENT_ACCEPTED"], publication="NONE"),
    S(id="E3", group="wrap", title="a jump of exactly N_cycle_advance_max is accepted",
      inputs=["cycle(5) complete", "health(cycle=5 + N_cycle_advance_max)"],
      events=["CYCLE_COMPLETED", "READY_ENTERED", "HEALTH_ACCEPTED", "CYCLE_RETIRED_BY_NEWER"],
      publication="FOUR_ROLE_RANGES", params=["N_cycle_advance_max"]),
    S(id="E4", group="wrap", title="a jump beyond it is implausible and resynchronises",
      inputs=["cycle(5) complete", "health(cycle=5 + N_cycle_advance_max + 1)"],
      events=["CYCLE_COMPLETED", "READY_ENTERED", "IMPLAUSIBLE_CYCLE_ADVANCE",
              "PROTOCOL_FAULT_RAISED", "READY_LOST"],
      publication="HEALTH_ONLY", params=["N_cycle_advance_max"],
      notes="indistinguishable from a wrap-aliased frame, so it is not guessed"),
    S(id="E5", group="wrap", title="cycle_seq wrapping 255 to 0 is an ordinary new cycle",
      inputs=["cycle(255) complete", "health(cycle=0, epoch unchanged)"],
      events=["CYCLE_COMPLETED", "READY_ENTERED", "HEALTH_ACCEPTED"],
      publication="FOUR_ROLE_RANGES", notes="modulo comparison, never magnitude"),
    S(id="E6", group="wrap", title="a stale frame aliasing onto a live cycle after the wrap is caught",
      inputs=["cycle(0) of the new lap in progress", "stale meas(src=0, cycle=0) from the old lap"],
      events=["MEASUREMENT_BUFFERED", "CONFLICTING_MEASUREMENT", "PROTOCOL_FAULT_RAISED"],
      publication="HEALTH_ONLY",
      notes="it must still find an authorising health with the same epoch and cycle"),
    S(id="E7", group="wrap", title="a measurement alone never moves the anchor",
      inputs=["anchor at cycle(5)", "meas(src=0, cycle=6)", "no health for 6 yet"],
      events=["MEASUREMENT_BUFFERED"], publication="NONE",
      notes="only a cycle_valid health frame advances the anchor"),

    # -- F. epoch ----------------------------------------------------------
    S(id="F1", group="epoch", title="a new epoch needs a PROVEN health before any measurement counts",
      inputs=["READY on epoch 1", "meas(src=0, epoch=2, cycle=0)"],
      events=["EPOCH_CHANGED", "READY_LOST", "MEASUREMENT_DROPPED_UNAUTHORISED"],
      publication="HEALTH_ONLY"),
    S(id="F2", group="epoch", title="a new epoch with PROVEN but only some sources refreshed",
      inputs=["health(epoch=2, cycle=0, PROVEN, produced=0x3)", "meas(src=0..1, epoch=2, cycle=0)"],
      events=["EPOCH_CHANGED", "HEALTH_ACCEPTED", "MEASUREMENT_ACCEPTED", "MEASUREMENT_ACCEPTED",
              "CYCLE_COMPLETED", "SOURCE_DEGRADED"],
      publication="HEALTH_ONLY", notes="no partial readiness"),
    S(id="F3", group="epoch", title="a previous-epoch measurement arriving after the change is dropped",
      inputs=["health(epoch=2, PROVEN)", "meas(src=0, epoch=1, cycle=7)"],
      events=["EPOCH_CHANGED", "HEALTH_ACCEPTED", "FRAME_FOR_RETIRED_CYCLE"],
      publication="HEALTH_ONLY", notes="dropped and counted, never a fault"),
    S(id="F4", group="epoch", title="mapping_epoch wrapping 255 to 0 is an ordinary new epoch",
      inputs=["READY on epoch 255", "health(epoch=0, cycle=0, PROVEN)"],
      events=["EPOCH_CHANGED", "READY_LOST", "HEALTH_ACCEPTED"], publication="HEALTH_ONLY"),
    S(id="F5", group="epoch",
      title="a late PROVEN-cycle measurement after a newer LOST health is NOT a fault",
      inputs=["cycle(9) PROVEN in flight", "health(cycle=10, LOST)", "meas(src=0, cycle=9)"],
      events=["HEALTH_ACCEPTED", "MAPPING_LOST", "READY_LOST", "FRAME_FOR_RETIRED_CYCLE"],
      publication="HEALTH_ONLY",
      notes="judged against its own cycle's health; the contract permits this interleaving"),
    S(id="F6", group="epoch", title="a measurement in a cycle whose own health is not PROVEN is a fault",
      inputs=["health(epoch=3, cycle=4, UNKNOWN)", "meas(src=0, epoch=3, cycle=4)"],
      events=["HEALTH_ACCEPTED", "MEASUREMENT_DROPPED_UNAUTHORISED", "PROTOCOL_FAULT_RAISED"],
      publication="HEALTH_ONLY", notes="the firmware obligation forbids emitting it"),

    # -- G. health frame and heartbeat ------------------------------------
    S(id="G1", group="health", title="a cycle_valid-clear heartbeat neither opens nor retires a cycle",
      inputs=["health(epoch=1, cycle=0, UNKNOWN, cycle_valid=0, produced=0x0)",
              "health(epoch=1, cycle=0, PROVEN, cycle_valid=1, produced=0xF)",
              "meas(src=0..3, epoch=1, cycle=0)"],
      events=["HEALTH_HEARTBEAT_ONLY", "HEALTH_ACCEPTED", "MEASUREMENT_ACCEPTED",
              "MEASUREMENT_ACCEPTED", "MEASUREMENT_ACCEPTED", "MEASUREMENT_ACCEPTED",
              "CYCLE_COMPLETED", "READY_ENTERED"],
      publication="FOUR_ROLE_RANGES",
      notes="the genuine cycle 0 must not be mistaken for one already retired"),
    S(id="G2", group="health", title="cycle_valid clear with a non-zero cycle_seq is malformed",
      inputs=["health(cycle=7, cycle_valid=0)"],
      events=["MALFORMED_FRAME", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY"),
    S(id="G3", group="health", title="cycle_valid clear with a non-zero per-cycle mask is malformed",
      inputs=["health(cycle=0, cycle_valid=0, produced=0x4)"],
      events=["MALFORMED_FRAME", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY"),
    S(id="G4", group="health", title="a repeated health_seq does not refresh health freshness",
      inputs=["health(health_seq=9)", "advance_clock(T_health_max_gap - 1)",
              "health(health_seq=9)", "advance_clock(1)"],
      events=["HEALTH_ACCEPTED", "HEALTH_REPEAT_IGNORED", "HEALTH_STALE", "READY_LOST"],
      publication="HEALTH_ONLY", params=["T_health_max_gap"],
      notes="the repeat is what a firmware that stopped updating would send"),
    S(id="G5", group="health", title="enumeration masks stay meaningful in a heartbeat",
      inputs=["health(UNKNOWN, cycle_valid=0, enum=0x7, model=0x7, produced=0x0, fault=0x0)"],
      events=["HEALTH_HEARTBEAT_ONLY"], publication="HEALTH_ONLY",
      params=["T_health_nominal"],
      notes="published at the nominal period even with no acquisition running; only the two "
            "per-cycle masks are governed by cycle_valid"),
    S(id="G6", group="health", title="an implicated chain position with no fault flag is a contradiction",
      inputs=["health(PROVEN, enum=0xF, model=0xF, produced=0xF, fault=0x0, failing_position=3)"],
      events=["CONTRADICTION_POSITION_WITHOUT_FAULT", "PROTOCOL_FAULT_RAISED"],
      publication="HEALTH_ONLY"),
    S(id="G7", group="health", title="failing_chain_position out of range is malformed",
      inputs=["health(failing_position=7)"],
      events=["MALFORMED_FRAME", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY",
      notes="1-6 over the whole chain, or CHAIN_POSITION_NONE"),

    # -- H. startup and timeout boundaries --------------------------------
    S(id="H1", group="timeout", title="no health inside the startup grace is NOT_READY",
      inputs=["node start", "advance_clock(T_startup_health_grace - 1)"],
      events=[], allow_no_events=True, publication="HEALTH_ONLY",
      params=["T_startup_health_grace"],
      notes="T-1 of the grace. Deliberately eventless: nothing has happened yet, and the state is "
            "carried by the health topic reading NOT_READY with the startup and no-health reasons"),
    S(id="H2", group="timeout", title="the grace expiring with no health at all is a FAULT",
      inputs=["node start", "advance_clock(T_startup_health_grace)"],
      events=["HEALTH_STALE"], publication="HEALTH_ONLY", params=["T_startup_health_grace"],
      notes="T of the grace, and the first failing case under the frozen comparison rule"),
    S(id="H3", group="timeout", title="one tick past the grace stays FAULT without re-reporting",
      inputs=["node start", "advance_clock(T_startup_health_grace + 1)"],
      events=[], allow_no_events=True, publication="HEALTH_ONLY",
      params=["T_startup_health_grace"],
      notes="T+1. Deliberately eventless: the alarm is edge triggered, so the condition persisting "
            "produces nothing further, and the state stays FAULT on the health topic"),
    S(id="H4", group="timeout", title="health arriving after the grace fault clears it",
      inputs=["node start", "advance_clock(T_startup_health_grace)", HEALTH_OK, FOUR_VALID],
      events=["HEALTH_STALE", "HEALTH_RECOVERED", "HEALTH_ACCEPTED"] +
             ["MEASUREMENT_ACCEPTED"] * 4 + ["CYCLE_COMPLETED", "READY_ENTERED"],
      publication="FOUR_ROLE_RANGES", params=["T_startup_health_grace"],
      notes="a timeout fault is recomputed from state and clears itself; a protocol fault does not"),
    S(id="H5", group="timeout", title="health going stale at the bound, in three steps",
      inputs=["READY", "advance_clock(T_health_max_gap - 1)", "advance_clock(1)", "advance_clock(1)"],
      events=["READY_ENTERED", "HEALTH_STALE", "READY_LOST"],
      publication="HEALTH_ONLY", params=["T_health_max_gap"],
      notes="T-1 fresh, T stale, T+1 unchanged"),
    S(id="H6", group="timeout", title="one source going stale at the bound, in three steps",
      inputs=["READY", "src=2 stops", "advance_clock(T_meas_max_gap - 1)", "advance_clock(1)"],
      events=["READY_ENTERED", "SOURCE_DEGRADED", "SOURCE_STALE", "READY_LOST"],
      publication="HEALTH_ONLY", params=["T_meas_max_gap"]),
    S(id="H7", group="timeout", title="a stale source recovering returns to READY",
      inputs=["source stale", "meas(src=2) resumes", "health completes the cycle"],
      events=["SOURCE_RECOVERED", "CYCLE_COMPLETED", "READY_ENTERED"],
      publication="FOUR_ROLE_RANGES", params=["T_meas_max_gap"]),
    S(id="H8", group="timeout", title="a source reaching the consecutive-miss count faults",
      inputs=["READY", "src=1 misses N_cycle_miss_fault consecutive cycles"],
      events=["READY_ENTERED", "SOURCE_DEGRADED", "SAMPLE_MISS_FAULT", "READY_LOST"],
      publication="HEALTH_ONLY", params=["N_cycle_miss_fault"],
      notes="counted at >=; it carries no timing claim, only a fault outcome"),
    S(id="H9", group="timeout", title="one cycle short of the count, still inside the age bound, stays READY",
      inputs=["READY", "src=1 misses N_cycle_miss_fault - 1 cycles inside T_meas_max_gap"],
      events=["READY_ENTERED", "SOURCE_DEGRADED"], publication="FOUR_ROLE_RANGES",
      params=["N_cycle_miss_fault", "T_meas_max_gap"],
      notes="a temporarily missing sample is tolerated; a faulty one never is"),

    # -- I. readiness criteria, one at a time -----------------------------
    S(id="I1", group="readiness", title="mapping UNKNOWN suppresses all role publication",
      inputs=["health(UNKNOWN, cycle_valid=1, produced=0xF)", "no measurements permitted"],
      events=["HEALTH_ACCEPTED", "ROLE_PUBLICATION_SUPPRESSED"], publication="HEALTH_ONLY",
      notes="the state of every robot today"),
    S(id="I2", group="readiness", title="mapping LOST after READY is a fault",
      inputs=["READY", "health(LOST)"],
      events=["READY_ENTERED", "MAPPING_LOST", "READY_LOST"], publication="HEALTH_ONLY"),
    S(id="I3", group="readiness", title="a chain fault flag costs READY",
      inputs=["READY", "health(PROVEN, fault flags = BUS_FAULT)"],
      events=["READY_ENTERED", "READY_LOST"], publication="HEALTH_ONLY"),
    S(id="I4", group="readiness", title="an incomplete enumeration mask costs READY",
      inputs=["health(PROVEN, enum=0x7)"],
      events=["HEALTH_ACCEPTED", "READY_LOST"], publication="HEALTH_ONLY"),
    S(id="I5", group="readiness", title="an incomplete model-verified mask costs READY",
      inputs=["health(PROVEN, model=0x7)"],
      events=["HEALTH_ACCEPTED", "READY_LOST"], publication="HEALTH_ONLY"),
    S(id="I6", group="readiness", title="a degraded source alone keeps READY",
      inputs=["READY", "src=3 misses one cycle inside T_meas_max_gap"],
      events=["READY_ENTERED", "SOURCE_DEGRADED"], publication="FOUR_ROLE_RANGES",
      params=["T_meas_max_gap"], notes="there is no DEGRADED readiness state"),
    S(id="I7", group="readiness", title="three of four fresh is NOT_READY",
      inputs=["health(produced=0x7)", "meas(src=0..2)"],
      events=["HEALTH_ACCEPTED"] + ["MEASUREMENT_ACCEPTED"] * 3 +
             ["CYCLE_COMPLETED", "SOURCE_DEGRADED"],
      publication="HEALTH_ONLY"),

    # -- J. protocol version ---------------------------------------------
    S(id="J1", group="version", title="the supported version is accepted",
      inputs=["health(protocol_version=PROTOCOL_VERSION)"],
      events=["HEALTH_ACCEPTED"], publication="HEALTH_ONLY"),
    S(id="J2", group="version", title="an unsupported version faults and authorises nothing",
      inputs=["health(protocol_version=PROTOCOL_VERSION + 1)", "meas(src=0..3)"],
      events=["VERSION_UNSUPPORTED", "PROTOCOL_FAULT_RAISED"] +
             ["MEASUREMENT_DROPPED_UNAUTHORISED"] * 4,
      publication="HEALTH_ONLY"),
    S(id="J3", group="version", title="version zero is never valid",
      inputs=["health(protocol_version=0)"],
      events=["VERSION_UNSUPPORTED", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY",
      notes="so an all-zero or foreign frame can never read as compatible"),

    # -- K. framing and configuration ------------------------------------
    S(id="K1", group="framing", title="a DLC other than eight is malformed",
      inputs=["meas(src=0, dlc=7)"],
      events=["MALFORMED_FRAME", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY"),
    S(id="K2", group="framing", title="a frame_type not matching its identifier is malformed",
      inputs=["frame(id=TOF_CLIFF_MEAS_ID, frame_type=0x2)"],
      events=["MALFORMED_FRAME", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY",
      notes="catches a mis-routed filter, which is otherwise a silent mis-decode"),
    S(id="K3", group="framing", title="a source_id outside zero to three is malformed",
      inputs=["meas(src=4)"],
      events=["MALFORMED_FRAME", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY"),
    S(id="K4", group="framing", title="a non-zero reserved byte is rejected",
      inputs=["meas(src=0, reserved=0x01)"],
      events=["MALFORMED_FRAME", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY",
      notes="rejected rather than ignored, unlike the grid contract"),
    S(id="K5", group="framing", title="a mapping_state outside its enum is malformed",
      inputs=["health(mapping_state=0x4)"],
      events=["MALFORMED_FRAME", "PROTOCOL_FAULT_RAISED"], publication="HEALTH_ONLY"),
    S(id="K6", group="framing", title="a malformed frame updates transport liveness but not freshness",
      inputs=["READY", "meas(src=0, reserved=0x01)", "advance_clock(T_meas_max_gap)"],
      events=["READY_ENTERED", "MALFORMED_FRAME", "PROTOCOL_FAULT_RAISED",
              "SOURCE_STALE", "READY_LOST"],
      publication="HEALTH_ONLY", params=["T_meas_max_gap"],
      notes="the corrupt frame must not have kept the source looking fresh"),
    S(id="K7", group="framing", title="a protocol fault clears only after one clean correlated cycle",
      inputs=["protocol fault raised", "cycle(n+1) complete and contradiction-free"],
      events=["PROTOCOL_FAULT_CLEARED", "CYCLE_COMPLETED", "READY_ENTERED"],
      publication="FOUR_ROLE_RANGES", notes="unlike a timeout fault, it latches until proven"),
    S(id="K8", group="config", title="a missing timeout parameter is a contained CONFIG_ERROR",
      inputs=["start with T_meas_max_gap unset"],
      events=["CONFIG_ERROR"], publication="HEALTH_ONLY", params=["T_meas_max_gap"],
      notes="no filter installed, no role topic advertised, health FAULT, rest of the driver runs"),
    S(id="K9", group="config", title="cliff enabled together with legacy_uart is a CONFIG_ERROR",
      inputs=["start with tof_transport=legacy_uart and cliff enabled"],
      events=["CONFIG_ERROR"], publication="HEALTH_ONLY",
      notes="cliff is unavailable in that mode, not degraded"),
    S(id="K10", group="config", title="the health identifier is not installable until allocated",
      inputs=["start with TOF_CLIFF_HEALTH_ID unresolved"],
      events=["CONFIG_ERROR"], publication="HEALTH_ONLY", params=["TOF_CLIFF_HEALTH_ID"],
      notes="a candidate is not an allocation; the filter must not be guessed"),
]


# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Layout vectors.
#
# These are the artefacts this revision actually emits: byte-exact encodings of the
# two frames, plus the accept/reject verdict the contract's validation rules give for
# each. They are what the packer and the decoder pin against, and they depend on NO
# timing value -- which is why they can be released while the timing profile cannot.
#
# The decoder state machine (cycle assembly, retirement, staleness, event multisets)
# is NOT covered here. That is the scenario catalogue above, and it still refuses.

MEAS_FRAME_TYPE = 0x1
HEALTH_FRAME_TYPE = 0x2

REJECT_REASONS = (
    "ACCEPT",
    "FRAME_TYPE_MISMATCH",
    "SOURCE_ID_OUT_OF_RANGE",
    "RESERVED_FIELD_NONZERO",
    "TARGET_COUNT_MALFORMED",
    "NO_TARGET_ENCODING_INCONSISTENT",
    "PROTOCOL_VERSION_ZERO",
    "PROTOCOL_VERSION_UNSUPPORTED",
    "MAPPING_STATE_MALFORMED",
    "CHAIN_POSITION_MALFORMED",
    "CYCLE_FIELDS_INCONSISTENT",
    "CHAIN_POSITION_WITHOUT_FAULT",
)

CYCLE_VALID_BIT = 0x8
CHAIN_FAULT_BITS = 0x7


def meas_bytes(source_id, epoch, cycle, range_mm, status, targets,
               frame_type=MEAS_FRAME_TYPE, reserved=0):
    return [
        ((frame_type & 0xF) << 4) | (source_id & 0xF),
        epoch & 0xFF,
        cycle & 0xFF,
        (range_mm >> 8) & 0xFF,
        range_mm & 0xFF,
        status & 0xFF,
        targets & 0xFF,
        reserved & 0xFF,
    ]


def meas_verdict(b):
    """The contract's measurement-frame validation rules, in the order they are stated."""
    if (b[0] >> 4) != MEAS_FRAME_TYPE:
        return "FRAME_TYPE_MISMATCH"
    if (b[0] & 0x0F) > 3:
        return "SOURCE_ID_OUT_OF_RANGE"
    if b[7] != 0:
        return "RESERVED_FIELD_NONZERO"
    if b[6] > 4:
        return "TARGET_COUNT_MALFORMED"
    # "target_count == 0 if and only if the frame carries status 255 and range_mm == 0xFFFF"
    no_target = (b[6] == 0)
    encoded_none = (b[5] == 255 and ((b[3] << 8) | b[4]) == RESOLVED["SENTINEL_INVALID"])
    if no_target != encoded_none:
        return "NO_TARGET_ENCODING_INCONSISTENT"
    return "ACCEPT"


def health_bytes(epoch, health_seq, mapping_state, flags, enumerated, model_verified,
                 sample_produced, sensor_fault, chain_position, cycle_seq,
                 frame_type=HEALTH_FRAME_TYPE, protocol_version=None):
    if protocol_version is None:
        protocol_version = RESOLVED["PROTOCOL_VERSION"]
    return [
        ((frame_type & 0xF) << 4) | (protocol_version & 0xF),
        epoch & 0xFF,
        health_seq & 0xFF,
        ((mapping_state & 0xF) << 4) | (flags & 0xF),
        ((enumerated & 0xF) << 4) | (model_verified & 0xF),
        ((sample_produced & 0xF) << 4) | (sensor_fault & 0xF),
        chain_position & 0xFF,
        cycle_seq & 0xFF,
    ]


def health_verdict(b):
    """The contract's health-frame validation rules, in the order they are stated."""
    if (b[0] >> 4) != HEALTH_FRAME_TYPE:
        return "FRAME_TYPE_MISMATCH"
    pv = b[0] & 0x0F
    if pv == 0:
        return "PROTOCOL_VERSION_ZERO"
    if pv != RESOLVED["PROTOCOL_VERSION"]:
        return "PROTOCOL_VERSION_UNSUPPORTED"
    mapping_state = b[3] >> 4
    if mapping_state > 0x3:
        return "MAPPING_STATE_MALFORMED"
    chain_position = b[6]
    if chain_position != RESOLVED["CHAIN_POSITION_NONE"] and not 1 <= chain_position <= 6:
        return "CHAIN_POSITION_MALFORMED"
    flags = b[3] & 0xF
    if not flags & CYCLE_VALID_BIT:
        # With cycle_valid clear the frame describes no cycle, so the per-cycle fields
        # must be empty. A non-zero one is a producer defect, not a heartbeat.
        if b[7] != 0 or (b[5] >> 4) != 0:
            return "CYCLE_FIELDS_INCONSISTENT"
    if chain_position != RESOLVED["CHAIN_POSITION_NONE"] and not flags & CHAIN_FAULT_BITS:
        return "CHAIN_POSITION_WITHOUT_FAULT"
    return "ACCEPT"


def V(name, kind, data, why=""):
    verdict = meas_verdict(data) if kind == "meas" else health_verdict(data)
    return {"name": name, "kind": kind, "bytes": data, "verdict": verdict, "why": why}


def layout_vectors():
    out = []
    sentinel = RESOLVED["SENTINEL_INVALID"]

    # -- measurement: the four roles ----------------------------------------
    for src, role in enumerate(("front_left", "rear_left", "rear_right", "front_right")):
        out.append(V(f"meas_role_{src}_{role}", "meas",
                     meas_bytes(src, 1, 0, 1234, 0, 1),
                     "source_id is a stable logical role, not a chain position"))

    # -- measurement: every status class in the classification table --------
    # target_count is 1 for every class except 255, which is the only encoding with 0.
    for status in (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14):
        rng = 1234 if status == 0 else sentinel
        out.append(V(f"meas_status_{status}", "meas",
                     meas_bytes(0, 1, 0, rng, status, 1),
                     "raw ULD status is transmitted unchanged; the class is a decoder concern"))
    out.append(V("meas_status_255_no_target", "meas",
                 meas_bytes(0, 1, 0, sentinel, 255, 0),
                 "the ULD forces 8191 mm when active_results == 0; the packer MUST send the "
                 "sentinel instead and never forward 8191 as a finite range"))

    # -- measurement: range boundaries --------------------------------------
    for rng, note in ((0, "zero is encodable and is not the sentinel"),
                      (1, "one millimetre"),
                      (65534, "largest encodable non-sentinel value"),):
        out.append(V(f"meas_range_{rng}", "meas", meas_bytes(0, 1, 0, rng, 0, 1), note))

    # -- measurement: epoch and cycle wrap ----------------------------------
    for epoch in (0, 1, 254, 255):
        out.append(V(f"meas_epoch_{epoch}", "meas", meas_bytes(0, epoch, 0, 1234, 0, 1),
                     "mapping_epoch wraps 255 -> 0"))
    for cycle in (0, 1, 254, 255):
        out.append(V(f"meas_cycle_{cycle}", "meas", meas_bytes(0, 1, cycle, 1234, 0, 1),
                     "cycle_seq wraps 255 -> 0; equality, not ordering, prevents splicing"))
    for targets in (1, 2, 3, 4):
        out.append(V(f"meas_targets_{targets}", "meas", meas_bytes(0, 1, 0, 1234, 0, targets),
                     "target_count is pre-reduction and diagnostic only"))

    # -- measurement: rejections --------------------------------------------
    out.append(V("meas_reject_frame_type", "meas",
                 meas_bytes(0, 1, 0, 1234, 0, 1, frame_type=HEALTH_FRAME_TYPE),
                 "a mis-routed filter is otherwise a silent mis-decode"))
    for src in (4, 15):
        out.append(V(f"meas_reject_source_id_{src}", "meas",
                     meas_bytes(src, 1, 0, 1234, 0, 1), "source_id must be 0-3"))
    out.append(V("meas_reject_reserved_nonzero", "meas",
                 meas_bytes(0, 1, 0, 1234, 0, 1, reserved=1),
                 "byte 7 is reserved for a future capture tick; using it is a version bump"))
    out.append(V("meas_reject_target_count_5", "meas",
                 meas_bytes(0, 1, 0, 1234, 0, 5), "target_count > 4 is malformed"))
    out.append(V("meas_reject_none_with_finite_range", "meas",
                 meas_bytes(0, 1, 0, 1234, 255, 0),
                 "status 255 with a finite range breaks the no-target encoding"))
    out.append(V("meas_reject_none_with_targets", "meas",
                 meas_bytes(0, 1, 0, sentinel, 255, 1),
                 "status 255 must carry target_count 0"))
    out.append(V("meas_reject_zero_targets_with_valid_status", "meas",
                 meas_bytes(0, 1, 0, 1234, 0, 0),
                 "target_count 0 is only legal as the status-255 encoding"))

    # -- health: accepted shapes -------------------------------------------
    out.append(V("health_proven_cycle_valid", "health",
                 health_bytes(1, 7, 0x1, CYCLE_VALID_BIT, 0xF, 0xF, 0xF, 0x0, 0xFF, 0),
                 "the ready shape: PROVEN, no chain fault, all four enumerated and produced"))
    out.append(V("health_heartbeat_unknown", "health",
                 health_bytes(1, 8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFF, 0),
                 "a heartbeat during UNKNOWN describes no cycle and depends on no measurement"))
    for state, label in ((0x0, "unknown"), (0x1, "proven"), (0x2, "lost"), (0x3, "fault")):
        out.append(V(f"health_mapping_state_{label}", "health",
                     health_bytes(1, 9, state, CYCLE_VALID_BIT, 0xF, 0xF, 0xF, 0x0, 0xFF, 3),
                     "all four mapping states are encodable"))
    for pos in (1, 2, 3, 4, 5, 6):
        out.append(V(f"health_chain_position_{pos}", "health",
                     health_bytes(1, 10, 0x3, CYCLE_VALID_BIT | 0x2, 0x7, 0x7, 0x0, 0x0, pos, 4),
                     "a failing position is reported with the chain fault that found it"))
    out.append(V("health_chain_position_none", "health",
                 health_bytes(1, 11, 0x1, CYCLE_VALID_BIT, 0xF, 0xF, 0xF, 0x0, 0xFF, 5),
                 "0xFF means no failing position"))
    for seq in (0, 1, 254, 255):
        out.append(V(f"health_seq_{seq}", "health",
                     health_bytes(1, seq, 0x1, CYCLE_VALID_BIT, 0xF, 0xF, 0xF, 0x0, 0xFF, 1),
                     "health_seq wraps 255 -> 0 and proves novelty, never age"))

    # -- health: rejections -------------------------------------------------
    out.append(V("health_reject_frame_type", "health",
                 health_bytes(1, 1, 0x1, CYCLE_VALID_BIT, 0xF, 0xF, 0xF, 0x0, 0xFF, 0,
                              frame_type=MEAS_FRAME_TYPE),
                 "frame_type must match the identifier it arrived on"))
    out.append(V("health_reject_protocol_version_zero", "health",
                 health_bytes(1, 1, 0x1, CYCLE_VALID_BIT, 0xF, 0xF, 0xF, 0x0, 0xFF, 0,
                              protocol_version=0),
                 "zero is not a valid version, so an all-zero byte cannot pass as one"))
    out.append(V("health_reject_protocol_version_2", "health",
                 health_bytes(1, 1, 0x1, CYCLE_VALID_BIT, 0xF, 0xF, 0xF, 0x0, 0xFF, 0,
                              protocol_version=2),
                 "a decoder accepts only versions it implements"))
    for state in (0x4, 0xF):
        out.append(V(f"health_reject_mapping_state_{state:x}", "health",
                     health_bytes(1, 1, state, CYCLE_VALID_BIT, 0xF, 0xF, 0xF, 0x0, 0xFF, 0),
                     "0x4-0xF are malformed mapping states"))
    for pos in (0, 7, 0xFE):
        out.append(V(f"health_reject_chain_position_{pos}", "health",
                     health_bytes(1, 1, 0x3, CYCLE_VALID_BIT | 0x1, 0xF, 0xF, 0xF, 0x0, pos, 0),
                     "a failing position is 1-6 or 0xFF"))
    out.append(V("health_reject_cycle_seq_without_cycle_valid", "health",
                 health_bytes(1, 1, 0x1, 0x0, 0xF, 0xF, 0x0, 0x0, 0xFF, 3),
                 "with cycle_valid clear the frame describes no cycle"))
    out.append(V("health_reject_produced_mask_without_cycle_valid", "health",
                 health_bytes(1, 1, 0x1, 0x0, 0xF, 0xF, 0xF, 0x0, 0xFF, 0),
                 "sample_produced_mask is a per-cycle field"))
    out.append(V("health_reject_position_without_chain_fault", "health",
                 health_bytes(1, 1, 0x1, CYCLE_VALID_BIT, 0xF, 0xF, 0xF, 0x0, 3, 1),
                 "naming a failing position with no chain fault set is contradictory"))
    return out


def layout_self_check(vectors):
    problems = []
    seen = set()
    for v in vectors:
        if v["name"] in seen:
            problems.append(f"{v['name']}: duplicate vector name")
        seen.add(v["name"])
        if len(v["bytes"]) != 8:
            problems.append(f"{v['name']}: DLC is always 8, got {len(v['bytes'])}")
        if any(not 0 <= b <= 0xFF for b in v["bytes"]):
            problems.append(f"{v['name']}: byte out of range")
        if v["verdict"] not in REJECT_REASONS:
            problems.append(f"{v['name']}: unknown verdict {v['verdict']}")
    # A suite with no rejections would not test the validation rules at all.
    if not any(v["verdict"] != "ACCEPT" for v in vectors):
        problems.append("the suite contains no rejection case")
    if not any(v["verdict"] == "ACCEPT" for v in vectors):
        problems.append("the suite contains no acceptance case")
    return problems


BANNER = "COMMISSIONING ONLY -- RELEASE_FORBIDDEN. Regenerate and re-pin both sides after the " \
         "six-board schedule measurement; status 3/11 remain unvalidated."


def emit_json(vectors, version, sha, path):
    doc = {
        "contract_file": CONTRACT.name,
        "contract_version": version,
        "contract_sha256": sha,
        "profile_name": PROFILE_NAME,
        "release_forbidden": RELEASE_FORBIDDEN,
        "banner": BANNER,
        "scope": "frame layout and validation rules only; the decoder state machine is not covered",
        "comparison_rule": COMPARISON_RULE,
        "resolved": {k: v for k, v in RESOLVED.items()},
        "validation_pending": VALIDATION_PENDING,
        "commissioning_profile": PROFILE,
        "profile_provenance": PROFILE_PROVENANCE,
        "unresolved_for_release": UNRESOLVED,
        "reject_reasons": list(REJECT_REASONS),
        "vectors": vectors,
    }
    path.write_text(json.dumps(doc, indent=2, sort_keys=False) + "\n")
    return path


def emit_header(vectors, version, sha, path):
    lines = [
        "/*",
        " * Copyright (c) 2026, LexxPluss Inc.",
        " * All rights reserved.",
        " *",
        " * SPDX-License-Identifier: BSD-3-Clause",
        " */",
        "",
        "/* GENERATED FILE -- do not edit. Regenerate with:",
        " *     docs/can/gen_cliff_golden_vectors.py --emit",
        " *",
        f" * Contract : {CONTRACT.name}",
        f" * Version  : {version}",
        f" * SHA-256  : {sha}",
        f" * Profile  : {PROFILE_NAME}",
        " *",
        f" * {BANNER}",
        " *",
        " * Scope: frame layout and validation verdicts only. The decoder state machine",
        " * (cycle assembly, retirement, staleness, event multisets) is NOT covered here.",
        " *",
        " * Zero dependencies on purpose: the SCBDriver tests have no JSON parser.",
        " */",
        "",
        "#pragma once",
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "// clang-format off",
        "",
        "namespace tof_cliff_contract {",
        "",
        f'inline constexpr char kContractVersion[]{{"{version}"}};',
        f'inline constexpr char kContractSha256[]{{"{sha}"}};',
        f'inline constexpr char kProfileName[]{{"{PROFILE_NAME}"}};',
        f"inline constexpr bool kReleaseForbidden{{{'true' if RELEASE_FORBIDDEN else 'false'}}};",
        "",
        f'inline constexpr uint16_t kMeasId{{0x{RESOLVED["TOF_CLIFF_MEAS_ID"]:03X}}};',
        f'inline constexpr uint16_t kHealthId{{0x{RESOLVED["TOF_CLIFF_HEALTH_ID"]:03X}}};',
        f'inline constexpr uint8_t kProtocolVersion{{0x{RESOLVED["PROTOCOL_VERSION"]:X}}};',
        f'inline constexpr uint16_t kSentinelInvalid{{0x{RESOLVED["SENTINEL_INVALID"]:04X}}};',
        f'inline constexpr uint8_t kSourceCount{{{RESOLVED["SOURCE_COUNT"]}}};',
        f'inline constexpr uint8_t kChainPositionNone{{0x{RESOLVED["CHAIN_POSITION_NONE"]:02X}}};',
        f'inline constexpr uint8_t kCycleMissFault{{{RESOLVED["N_cycle_miss_fault"]}}};',
        f'inline constexpr uint8_t kCycleAdvanceMax{{{RESOLVED["N_cycle_advance_max"]}}};',
        "",
        "// Commissioning profile. Model-derived, NOT measured. A production build must not",
        "// inherit these values.",
    ]
    for k, v in PROFILE.items():
        lines.append(f"inline constexpr uint32_t k{''.join(p.capitalize() for p in k.split('_'))}{{{v}}};")
    lines += [
        "",
        "enum class verdict : uint8_t {",
    ]
    for i, r in enumerate(REJECT_REASONS):
        lines.append(f"    {r.lower()} = {i},")
    lines += [
        "};",
        "",
        "enum class frame_kind : uint8_t { measurement = 0, health = 1 };",
        "",
        "struct vector {",
        "    const char *name;",
        "    frame_kind kind;",
        "    uint8_t bytes[8];",
        "    verdict expected;",
        "    const char *why;",
        "};",
        "",
        f"inline constexpr size_t kVectorCount{{{len(vectors)}}};",
        "",
        "inline constexpr vector kVectors[kVectorCount]{",
    ]
    for v in vectors:
        b = ", ".join(f"0x{x:02x}" for x in v["bytes"])
        kind = "frame_kind::measurement" if v["kind"] == "meas" else "frame_kind::health"
        why = v["why"].replace('"', '\\"')
        lines.append(f'    {{"{v["name"]}", {kind}, {{{b}}}, verdict::{v["verdict"].lower()},')
        lines.append(f'     "{why}"}},')
    lines += [
        "};",
        "",
        "}  // namespace tof_cliff_contract",
        "",
        "// clang-format on",
        "",
    ]
    path.write_text("\n".join(lines))
    return path


def contract_identity():
    text = CONTRACT.read_text()
    version = re.search(r"^Contract version: \*\*(.+?)\*\*$", text, re.M)
    if not version:
        raise SystemExit("cannot find the contract version line")
    return version.group(1), hashlib.sha256(text.encode()).hexdigest()


def self_check():
    problems = []
    seen = set()
    for sc in CATALOGUE:
        sid = sc["id"]
        if sid in seen:
            problems.append(f"{sid}: duplicate id")
        seen.add(sid)
        if not sc["events"] and not sc["allow_no_events"]:
            problems.append(f"{sid}: empty event multiset")
        for ev in sc["events"]:
            if ev not in EVENTS:
                problems.append(f"{sid}: unknown event {ev}")
        if sc["publication"] not in PUBLICATIONS:
            problems.append(f"{sid}: unknown publication {sc['publication']}")
        if sc["blocked"] not in (None,) + BLOCKERS:
            problems.append(f"{sid}: unknown blocker {sc['blocked']}")
        for p in sc["params"]:
            if p not in UNRESOLVED and p not in RESOLVED:
                problems.append(f"{sid}: undeclared parameter {p}")
    referenced = {p for sc in CATALOGUE for p in sc["params"]}
    for sym in UNRESOLVED:
        if sym not in referenced:
            problems.append(f"symbol {sym} is declared unresolved but no scenario depends on it")
    return problems


def render_list():
    version, sha = contract_identity()
    print(f"cliff ToF golden scenario catalogue")
    print(f"contract {CONTRACT.name} version {version}")
    print(f"contract sha256 {sha}")
    print(f"comparison rule: {COMPARISON_RULE}")
    print()
    group = None
    for sc in CATALOGUE:
        if sc["group"] != group:
            group = sc["group"]
            print(f"== {group} ==")
        tag = f"  [{sc['blocked']}]" if sc["blocked"] else ""
        print(f"{sc['id']:18s} {sc['title']}{tag}")
        for line in sc["inputs"]:
            print(f"{'':20s}in  : {line}")
        multiset = {}
        for ev in sc["events"]:
            multiset[ev] = multiset.get(ev, 0) + 1
        events = ", ".join(f"{k} x{v}" if v > 1 else k for k, v in multiset.items()) or "(none)"
        print(f"{'':20s}out : {events}")
        print(f"{'':20s}pub : {sc['publication']}")
        if sc["params"]:
            print(f"{'':20s}par : {', '.join(sc['params'])}")
        if sc["notes"]:
            print(f"{'':20s}why : {sc['notes']}")
        print()
    blocked = [sc["id"] for sc in CATALOGUE if sc["blocked"]]
    print(f"{len(CATALOGUE)} scenarios, {len(blocked)} blocked on hardware: {', '.join(blocked)}")
    print(f"{len(UNRESOLVED)} unresolved symbols:")
    for sym, why in UNRESOLVED.items():
        print(f"  {sym:26s} {why}")


def refuse_generation():
    """Refuses the SCENARIO vectors. The layout artefacts are a separate, emittable thing.

    The split is the point: the decoder state machine depends on every timing value, so it
    cannot be pinned yet. The frame layout depends on none of them, so it can.
    """
    version, _ = contract_identity()
    print(f"refusing to generate decoder state-machine vectors (contract {version}).",
          file=sys.stderr)
    reasons = [
        "the scenario catalogue's inputs are notation, not machine-readable structures; "
        "emitting event-multiset vectors needs a parser for that notation first",
        f"{len(UNRESOLVED)} timing values are unresolved for release: "
        f"{', '.join(sorted(UNRESOLVED))}",
    ]
    blocked = [sc["id"] for sc in CATALOGUE if sc["blocked"]]
    if blocked:
        reasons.append(f"{len(blocked)} scenarios are blocked on hardware validation: "
                       f"{', '.join(blocked)}")
    if VALIDATION_PENDING:
        reasons.append(f"{len(VALIDATION_PENDING)} values are authorised for commissioning but "
                       f"unvalidated: {', '.join(sorted(VALIDATION_PENDING))}")
    for r in reasons:
        print(f"  - {r}", file=sys.stderr)
    print("\nrun --emit for the frame-layout artefacts, which depend on no timing value.",
          file=sys.stderr)
    print("run --list to see the catalogue these will become.", file=sys.stderr)
    return 1


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0], allow_abbrev=False)
    ap.add_argument("--list", action="store_true", help="print the scenario catalogue")
    ap.add_argument("--check", action="store_true",
                    help="verify the catalogue and the layout vectors, and that committed "
                         "artefacts match what would be emitted now")
    ap.add_argument("--emit", action="store_true",
                    help="write the layout artefacts (JSON + zero-dependency C++ header)")
    args = ap.parse_args(argv)

    problems = self_check()
    if problems:
        for p in problems:
            print(f"catalogue error: {p}", file=sys.stderr)
        return 2

    vectors = layout_vectors()
    problems = layout_self_check(vectors)
    if problems:
        for p in problems:
            print(f"layout error: {p}", file=sys.stderr)
        return 2

    version, sha = contract_identity()
    json_path = HERE / "tof_cliff_layout_vectors.json"
    header_path = HERE / "tof_cliff_contract_vectors.h"

    if args.emit:
        if version.startswith("draft-"):
            print(f"refusing to emit: the contract is a draft ({version})", file=sys.stderr)
            return 1
        emit_json(vectors, version, sha, json_path)
        emit_header(vectors, version, sha, header_path)
        accepts = len([v for v in vectors if v["verdict"] == "ACCEPT"])
        print(f"emitted {len(vectors)} layout vectors "
              f"({accepts} accept / {len(vectors) - accepts} reject)")
        print(f"  {json_path.name}")
        print(f"  {header_path.name}")
        print(f"contract {version}  sha256 {sha}")
        print(f"profile {PROFILE_NAME}  release_forbidden={RELEASE_FORBIDDEN}")
        return 0

    if args.check:
        print(f"catalogue OK: {len(CATALOGUE)} scenarios, "
              f"{len([s for s in CATALOGUE if s['blocked']])} blocked, "
              f"{len(UNRESOLVED)} symbols unresolved for release")
        print(f"layout OK: {len(vectors)} vectors")
        print(f"contract {version}  sha256 {sha}")
        # Catch "the contract text was edited but the artefacts were not regenerated".
        stale = []
        for path, emit in ((json_path, emit_json), (header_path, emit_header)):
            if not path.exists():
                stale.append(f"{path.name} is missing")
                continue
            before = path.read_text()
            emit(vectors, version, sha, path)
            if path.read_text() != before:
                stale.append(f"{path.name} was out of date and has been rewritten")
        for s in stale:
            print(f"artefact drift: {s}", file=sys.stderr)
        return 1 if stale else 0

    if args.list:
        render_list()
        return 0
    return refuse_generation()


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except BrokenPipeError:
        # the catalogue is long and will be piped into head or less
        sys.stderr.close()
        sys.exit(0)
