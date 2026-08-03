#!/usr/bin/env python3
"""Normative reference packer for the ToF grid CAN wire contract (AMRSW-2322).

Emits two artefacts, both stamped with the SHA-256 of tof_can_wire_contract.md:

  tof_grid_golden_vectors.json  - language-neutral, for tooling and review
  tof_contract_vectors.h        - dependency-free C++ header, so neither repository
                                  needs a JSON parser in its test build

Both LexxHard-SensorControlBoard-Firmware (packer) and LexxHard-SCBDriver (decoder)
are tested against these. Each repository pins CONTRACT_SHA; editing the contract
breaks both pins until the vectors are regenerated and the pins updated deliberately.

Run:  python3 gen_golden_vectors.py
"""

import hashlib
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CONTRACT = "tof_can_wire_contract.md"
CONTRACT_VERSION = "2026-08-02e"

INVALID = 0xFFF
MAX_VALID = 0xFFE  # 4094 mm
ZONES = 64
CHUNKS = 16
ZONES_PER_CHUNK = 4
TIMEOUT_MS = 300
SOURCE_STALE_MS = 1000
STARTUP_GRACE_MS = 3000

ST_NEVER_SEEN = "NEVER_SEEN"
ST_HEALTHY = "HEALTHY"
ST_STALE_NOT_COMPLETING = "STALE_NOT_COMPLETING"
ST_STALE_NO_FRAMES = "STALE_NO_FRAMES"

SRC_RIGHT = 0  # hanging_front_right -> low_object_right
SRC_LEFT = 1   # hanging_front_left  -> low_object_left

# status flag bits, all recovered or chain-level by construction
FLAG_I2C_RECOVERED = 1 << 0
FLAG_TIMEOUT_RECOVERED = 1 << 1
FLAG_CHAIN_LENGTH = 1 << 2
FLAG_PEER_ENUM_FAILED = 1 << 3

EV_PUBLISHED = "GRID_PUBLISHED"
EV_TIMEOUT = "INCOMPLETE_BY_TIMEOUT"
EV_GEN_CHANGE = "INCOMPLETE_BY_GENERATION_CHANGE"
EV_DUP_IDENTICAL = "DUPLICATE_CHUNK_IDENTICAL"
EV_CONFLICT = "CONFLICTING_CHUNK"
EV_MALFORMED = "MALFORMED_HEADER"
EV_COUNT_MISMATCH = "HEALTH_COUNT_MISMATCH"
EV_DUP_HEALTH = "DUPLICATE_HEALTH_IDENTICAL"
EV_CONFLICT_HEALTH = "CONFLICTING_HEALTH"
EV_RETIRED = "FRAME_FOR_RETIRED_GENERATION"
EV_ORPHAN_HEALTH = "ORPHAN_HEALTH_TIMEOUT"
EV_NEVER_SEEN = "SOURCE_NEVER_SEEN"
EV_STALE = "SOURCE_STALE"
EV_RECOVERED = "SOURCE_RECOVERED"


def pack_chunk(generation, source_id, chunk_index, z):
    """One 8-byte data frame. z is the four 12-bit zone values for this chunk."""
    assert 0 <= generation <= 0xFF
    assert 0 <= source_id <= 0xF
    assert 0 <= chunk_index <= 0xF
    assert len(z) == ZONES_PER_CHUNK
    for v in z:
        assert 0 <= v <= INVALID, f"zone value {v} does not fit in 12 bits"
    return bytes([
        generation,
        (source_id << 4) | chunk_index,
        (z[0] & 0xFF0) >> 4,
        ((z[0] & 0x00F) << 4) | (z[1] >> 8),
        z[1] & 0x0FF,
        (z[2] & 0xFF0) >> 4,
        ((z[2] & 0x00F) << 4) | (z[3] >> 8),
        z[3] & 0x0FF,
    ])


def unpack_chunk(frame):
    """Reference decoder for one data frame."""
    assert len(frame) == 8
    b = frame
    return (
        b[0],
        b[1] >> 4,
        b[1] & 0x0F,
        [
            (b[2] << 4) | (b[3] >> 4),
            ((b[3] & 0x0F) << 8) | b[4],
            (b[5] << 4) | (b[6] >> 4),
            ((b[6] & 0x0F) << 8) | b[7],
        ],
    )


def pack_health(generation, source_id, grid, flags=0, chain_position=None,
                boards_detected=6, last_error=0, valid_count_override=None):
    if chain_position is None:
        chain_position = 0 if source_id == SRC_RIGHT else 1
    valid_zone_count = (valid_count_override if valid_count_override is not None
                        else sum(1 for v in grid if v != INVALID))
    assert 0 <= chain_position <= 0xF and 0 <= boards_detected <= 0xF
    return bytes([
        generation,
        (source_id << 4) | 0x0,
        valid_zone_count,
        flags,
        (boards_detected << 4) | chain_position,
        last_error,
        0,
        0,
    ])


def pack_grid(generation, source_id, grid):
    assert len(grid) == ZONES
    return [
        pack_chunk(generation, source_id, k,
                   grid[k * ZONES_PER_CHUNK:(k + 1) * ZONES_PER_CHUNK])
        for k in range(CHUNKS)
    ]


def hexf(frame):
    return frame.hex().upper()


# ---------------------------------------------------------------- grids

def grid_ramp():
    """Ordinary data: 0, 50, 100 ... 3150 mm, row-major."""
    return [i * 50 for i in range(ZONES)]


def grid_all_invalid():
    return [INVALID] * ZONES


def grid_boundaries():
    """Every value that has ever been an off-by-one, spread over the grid."""
    pattern = [0, 1, 4093, MAX_VALID, INVALID, 0x800, 0x0FF, 0x100]
    return [pattern[i % len(pattern)] for i in range(ZONES)]


def grid_single_close_target():
    """Realistic hanging object: one cluster near, the rest at the sentinel."""
    g = [INVALID] * ZONES
    for row in range(2, 5):
        for col in range(3, 6):
            g[row * 8 + col] = 700 + (row * 8 + col)
    return g


GRIDS = {
    "ramp": grid_ramp(),
    "all_invalid": grid_all_invalid(),
    "boundaries": grid_boundaries(),
    "single_close_target": grid_single_close_target(),
}


def round_trip_check(generation, source_id, grid, frames):
    """Self-check: the vectors must decode back to the grid they came from."""
    seen = 0
    out = [None] * ZONES
    for f in frames:
        gen, sid, idx, zs = unpack_chunk(f)
        assert gen == generation and sid == source_id
        assert not (seen >> idx) & 1, "duplicate chunk in generated vector"
        seen |= 1 << idx
        for j, v in enumerate(zs):
            out[idx * ZONES_PER_CHUNK + j] = v
    assert seen == 0xFFFF, f"bitmap {seen:#06x} is not complete"
    assert out == grid, "round trip mismatch"


def build(contract_sha):
    vectors = {
        "contract": CONTRACT,
        "contract_version": CONTRACT_VERSION,
        "contract_sha256": contract_sha,
        "note": (
            "CAN identifiers are deliberately absent: they are pending allocation and are "
            "injected by each implementation. Frames are hex, byte 0 leftmost. DLC is 8 for "
            "every frame."
        ),
        "constants": {
            "invalid_sentinel": INVALID,
            "max_valid_mm": MAX_VALID,
            "zones": ZONES,
            "chunks_per_grid": CHUNKS,
            "zones_per_chunk": ZONES_PER_CHUNK,
            "timeout_ms": TIMEOUT_MS,
            "source_stale_ms": SOURCE_STALE_MS,
            "startup_grace_ms": STARTUP_GRACE_MS,
            "source_id_hanging_front_right": SRC_RIGHT,
            "source_id_hanging_front_left": SRC_LEFT,
        },
        "grids": [],
        "scenarios": [],
    }

    # --- byte-exact grid vectors -------------------------------------
    for name, sid, gen in [
        ("ramp", SRC_RIGHT, 0),
        ("all_invalid", SRC_LEFT, 7),
        ("boundaries", SRC_RIGHT, 254),
        ("single_close_target", SRC_LEFT, 255),
    ]:
        grid = GRIDS[name]
        frames = pack_grid(gen, sid, grid)
        round_trip_check(gen, sid, grid, frames)
        vectors["grids"].append({
            "name": name,
            "source_id": sid,
            "generation": gen,
            "zones_mm": grid,
            "data_frames": [hexf(f) for f in frames],
            "health_frame": hexf(pack_health(gen, sid, grid)),
            "expected_valid_zone_count": sum(1 for v in grid if v != INVALID),
        })

    # --- decoder behaviour scenarios ---------------------------------
    ramp = GRIDS["ramp"]
    ramp_frames = pack_grid(3, SRC_RIGHT, ramp)
    ramp_health = pack_health(3, SRC_RIGHT, ramp)

    def s(name, description, frames, publishes, events, expect=None):
        # Every scenario ends with an explicit poll so the expected event multiset is
        # fully determined by the vector rather than by whatever the test harness
        # happens to do afterwards.
        last_t = max(f["t_ms"] for f in frames)
        frames = frames + [{"kind": "poll", "hex": "", "t_ms": last_t + TIMEOUT_MS + 1}]
        entry = {"name": name, "description": description, "frames": frames,
                 "publishes": publishes, "expected_events": events}
        if expect:
            entry.update(expect)
        vectors["scenarios"].append(entry)

    def data(f, t=0):
        return {"kind": "data", "hex": hexf(f), "t_ms": t}

    def health(f, t=0):
        return {"kind": "health", "hex": hexf(f), "t_ms": t}

    def poll(t):
        """Advance time and call poll(). No CAN input: this is the only way the
        watchdog cases can be expressed at all."""
        return {"kind": "poll", "hex": "", "t_ms": t}

    s("in_order",
      "All sixteen chunks in order, then health. Baseline.",
      [data(f) for f in ramp_frames] + [health(ramp_health)],
      True, {EV_PUBLISHED: 1},
      {"expected_zones_mm": ramp, "expected_source_id": SRC_RIGHT})

    s("reverse_order",
      "Chunks arrive in descending index order. Bitmap keying must not care.",
      [data(f) for f in reversed(ramp_frames)] + [health(ramp_health)],
      True, {EV_PUBLISHED: 1}, {"expected_zones_mm": ramp})

    s("health_first",
      "Health wins arbitration and arrives before any data frame. The two IDs have no "
      "mutual ordering guarantee, so this is normal traffic, not an error.",
      [health(ramp_health)] + [data(f) for f in ramp_frames],
      True, {EV_PUBLISHED: 1}, {"expected_zones_mm": ramp})

    s("health_interleaved",
      "Health arrives in the middle of the grid.",
      [data(f) for f in ramp_frames[:9]] + [health(ramp_health)]
      + [data(f) for f in ramp_frames[9:]],
      True, {EV_PUBLISHED: 1}, {"expected_zones_mm": ramp})

    s("lost_chunk",
      "Chunk 5 never arrives. Bitmap stays 0xFFDF, so nothing is published even though "
      "health is valid, and the assembly times out.",
      [data(f) for i, f in enumerate(ramp_frames) if i != 5] + [health(ramp_health)],
      False, {EV_TIMEOUT: 1}, {"expected_bitmap": 0xFFDF})

    s("missing_health",
      "All sixteen chunks arrive, health never does. The publish gate needs both.",
      [data(f) for f in ramp_frames],
      False, {EV_TIMEOUT: 1}, {"expected_bitmap": 0xFFFF})

    s("duplicate_chunk_identical",
      "Chunk 2 arrives twice with an identical payload: a benign retransmission. Counted, "
      "discarded, the grid still completes.",
      [data(f) for f in ramp_frames] + [data(ramp_frames[2])] + [health(ramp_health)],
      True, {EV_DUP_IDENTICAL: 1, EV_PUBLISHED: 1}, {"expected_zones_mm": ramp})

    s("conflicting_chunk_rejects",
      "Chunk 2 arrives twice with DIFFERENT payloads. Two values for one "
      "(source, generation, chunk) means a firmware fault or two streams mixing, and there "
      "is no basis for picking one. The generation is retired, so the health frame that "
      "follows finds nothing to close.",
      [data(f) for f in ramp_frames]
      + [data(pack_chunk(3, SRC_RIGHT, 2, [1, 2, 3, 4]))]
      + [health(ramp_health)],
      False, {EV_CONFLICT: 1, EV_RETIRED: 1})

    s("conflicting_chunk_not_resurrected",
      "A conflict arrives early, then the rest of the generation arrives normally. Every "
      "later frame of the retired generation is rejected; nothing can revive it.",
      [data(f) for f in ramp_frames[:3]]
      + [data(pack_chunk(3, SRC_RIGHT, 1, [9, 9, 9, 9]))]
      + [data(f) for f in ramp_frames[3:]] + [health(ramp_health)],
      False, {EV_CONFLICT: 1, EV_RETIRED: 14})

    gen4_frames = pack_grid(4, SRC_RIGHT, GRIDS["boundaries"])
    s("cross_grid_splice_attempt",
      "Chunks 0-7 of generation 3, then chunks 8-15 of generation 4. A decoder keyed only "
      "on chunk index would splice these into one bogus grid. Generation equality discards "
      "the generation-3 partial and retires it, so the trailing generation-3 health frame "
      "is rejected and the generation-4 remnant times out. Neither grid completes.",
      [data(f) for f in ramp_frames[:8]] + [data(f) for f in gen4_frames[8:]]
      + [health(ramp_health)],
      False, {EV_GEN_CHANGE: 1, EV_RETIRED: 1, EV_TIMEOUT: 1})

    s("generation_wrap",
      "Generation 255 completes, then generation 0 completes. Equality keying makes the "
      "wrap an ordinary new grid, not a regression.",
      [data(f) for f in pack_grid(255, SRC_LEFT, ramp)]
      + [health(pack_health(255, SRC_LEFT, ramp))]
      + [data(f) for f in pack_grid(0, SRC_LEFT, GRIDS["boundaries"])]
      + [health(pack_health(0, SRC_LEFT, GRIDS["boundaries"]))],
      True, {EV_PUBLISHED: 2}, {"expected_publish_count": 2})

    left_frames = pack_grid(11, SRC_LEFT, GRIDS["single_close_target"])
    right_frames = pack_grid(12, SRC_RIGHT, ramp)
    interleaved = []
    for a, b in zip(left_frames, right_frames):
        interleaved.append(data(a))
        interleaved.append(data(b))
    s("both_sources_interleaved",
      "The two sources' grids are interleaved frame by frame on one CAN ID. Separate "
      "assembly slots per source_id must keep them apart.",
      interleaved
      + [health(pack_health(11, SRC_LEFT, GRIDS["single_close_target"]))]
      + [health(pack_health(12, SRC_RIGHT, ramp))],
      True, {EV_PUBLISHED: 2}, {"expected_publish_count": 2})

    s("timeout",
      "Fifteen chunks arrive, the sixteenth arrives 350 ms after the first. The slot has "
      "already expired at 300 ms and its generation is retired, so the late chunk and the "
      "late health are both rejected rather than starting a fresh assembly.",
      [data(f, t=0) for f in ramp_frames[:15]]
      + [data(ramp_frames[15], t=350), health(ramp_health, t=350)],
      False, {EV_TIMEOUT: 1, EV_RETIRED: 2})

    s("just_inside_timeout",
      "The same traffic 10 ms earlier, at 290 ms, still completes. Pins the boundary.",
      [data(f, t=0) for f in ramp_frames[:15]]
      + [data(ramp_frames[15], t=290), health(ramp_health, t=290)],
      True, {EV_PUBLISHED: 1}, {"expected_zones_mm": ramp})

    s("retired_generation_not_revived_after_timeout",
      "Regression test. A generation times out, and then the complete grid arrives, same "
      "generation. If the timeout merely cleared the slot, this would reassemble and "
      "publish data that had already been judged untrustworthy. Every frame must be "
      "rejected instead.",
      [data(f, t=0) for i, f in enumerate(pack_grid(50, SRC_RIGHT, ramp)) if i != 5]
      + [poll(400)]
      + [data(f, t=500) for f in pack_grid(50, SRC_RIGHT, ramp)]
      + [health(pack_health(50, SRC_RIGHT, ramp), t=500)],
      False, {EV_TIMEOUT: 1, EV_RETIRED: 17})

    s("bad_source_id",
      "source_id 9 does not exist. The frame cannot be attributed to any source, so it is "
      "counted as malformed and dropped without touching a tracker.",
      [data(pack_chunk(3, 9, 0, [100, 200, 300, 400]))],
      False, {EV_MALFORMED: 1})

    s("health_reserved_nibble_set",
      "Health byte 1 low nibble is non-zero, which the contract forbids. The health frame "
      "is not structurally valid, so the gate stays shut and the grid times out.",
      [data(f) for f in ramp_frames]
      + [health(bytes([3, (SRC_RIGHT << 4) | 0x5]) + ramp_health[2:])],
      False, {EV_MALFORMED: 1, EV_TIMEOUT: 1})

    s("health_generation_mismatch",
      "A complete generation-3 grid, but the health frame carries generation 4. The health "
      "frame does not close the grid — it replaces the slot. The generation-3 grid is "
      "discarded and the lone health frame then times out as an orphan.",
      [data(f) for f in ramp_frames]
      + [health(pack_health(4, SRC_RIGHT, ramp))],
      False, {EV_GEN_CHANGE: 1, EV_ORPHAN_HEALTH: 1})

    s("health_count_mismatch_rejects",
      "Health claims 40 valid zones but 64 decode as valid. The packer contradicts its own "
      "summary, so one of the two is wrong and there is no way to tell which; the zone data "
      "cannot be trusted either. Reject, count, and raise an operator-visible diagnostic.",
      [data(f) for f in ramp_frames]
      + [health(pack_health(3, SRC_RIGHT, ramp, valid_count_override=40))],
      False, {EV_COUNT_MISMATCH: 1})

    s("health_count_out_of_range",
      "valid_zone_count of 200 exceeds 64. Structurally invalid, not merely mismatched. "
      "0xFF is reserved for a future status-only frame and is not defined yet.",
      [data(f) for f in ramp_frames]
      + [health(pack_health(3, SRC_RIGHT, ramp, valid_count_override=200))],
      False, {EV_MALFORMED: 1, EV_TIMEOUT: 1})

    s("duplicate_health_identical",
      "Health arrives twice with identical bytes while the grid is still incomplete. Same "
      "rule as a duplicate chunk: benign, counted, ignored.",
      [data(f) for f in ramp_frames[:15]]
      + [health(ramp_health), health(ramp_health)]
      + [data(ramp_frames[15])],
      True, {EV_DUP_HEALTH: 1, EV_PUBLISHED: 1}, {"expected_zones_mm": ramp})

    s("duplicate_health_differing_only_in_reserved_fields",
      "Health arrives twice. The second sets reserved status bits 4-7 and reserved bytes "
      "6-7, which a future firmware is allowed to do. The normalised values are equal, so "
      "this is a benign retransmission. Comparing raw bytes here would retire a good grid "
      "the day the firmware starts using those fields — forward compatibility turning "
      "into data loss.",
      [data(f) for f in ramp_frames[:15]]
      + [health(ramp_health)]
      + [health(ramp_health[:3] + bytes([ramp_health[3] | 0xF0]) + ramp_health[4:6]
                + bytes([0xAB, 0xCD]))]
      + [data(ramp_frames[15])],
      True, {EV_DUP_HEALTH: 1, EV_PUBLISHED: 1}, {"expected_zones_mm": ramp})

    s("conflicting_health_rejects",
      "Two health frames for one generation disagree. As with a conflicting chunk there is "
      "no basis for choosing either summary, so the generation is retired.",
      [data(f) for f in ramp_frames[:15]]
      + [health(ramp_health)]
      + [health(pack_health(3, SRC_RIGHT, ramp, flags=FLAG_I2C_RECOVERED))]
      + [data(ramp_frames[15])],
      False, {EV_CONFLICT_HEALTH: 1, EV_RETIRED: 1})

    s("recovered_flags_do_not_gate",
      "Health reports a recovered I2C error and a chain-length mismatch. Under the firmware "
      "transmit obligation, a grid only exists after a complete successful model-verified "
      "read, so these describe the past or the chain, never this grid. It publishes.",
      [data(f) for f in ramp_frames]
      + [health(pack_health(3, SRC_RIGHT, ramp,
                            flags=FLAG_I2C_RECOVERED | FLAG_CHAIN_LENGTH))],
      True, {EV_PUBLISHED: 1},
      {"expected_zones_mm": ramp,
       "expected_flags": FLAG_I2C_RECOVERED | FLAG_CHAIN_LENGTH})

    s("peer_enumeration_failure_reported",
      "The right source publishes normally while reporting that the OTHER sensor failed "
      "enumeration. This is how a sensor that emits nothing at all becomes visible: the "
      "surviving one says so. Publishes, and the flag must reach diagnostics.",
      [data(f) for f in ramp_frames]
      + [health(pack_health(3, SRC_RIGHT, ramp, flags=FLAG_PEER_ENUM_FAILED,
                            boards_detected=5))],
      True, {EV_PUBLISHED: 1},
      {"expected_zones_mm": ramp, "expected_flags": FLAG_PEER_ENUM_FAILED})

    s("reserved_bytes_set_are_ignored",
      "A future firmware populates health bytes 6-7 and reserved flag bits 4-7. An older "
      "decoder must ignore them rather than reject the frame.",
      [data(f) for f in ramp_frames]
      + [health(ramp_health[:3] + bytes([0xF0]) + ramp_health[4:6]
                + bytes([0xAB, 0xCD]))],
      True, {EV_PUBLISHED: 1}, {"expected_zones_mm": ramp})

    # --- watchdog scenarios ------------------------------------------
    # These have little or no CAN input by design. They are the reason poll() exists:
    # consume() is never called when a source goes silent, so the one failure mode with
    # no input to react to needs its own entry point.

    s("watchdog_never_seen_within_grace",
      "No traffic at all, polled 100 ms after startup. Both sources are NEVER_SEEN, but "
      "that is the ordinary state of a system still starting up and must not alarm.",
      [poll(100)],
      False, {},
      {"expected_state_src0": ST_NEVER_SEEN, "expected_state_src1": ST_NEVER_SEEN})

    s("watchdog_never_seen_after_grace",
      "Still no traffic at 3500 ms, past the 3000 ms startup grace. Both sources remain "
      "NEVER_SEEN and this now alarms, once each. Nothing else in the protocol can detect "
      "this: with no frames, every frame-level check is vacuous.",
      [poll(100), poll(3500)],
      False, {EV_NEVER_SEEN: 2},
      {"expected_state_src0": ST_NEVER_SEEN, "expected_state_src1": ST_NEVER_SEEN})

    s("watchdog_never_seen_then_first_grid_recovers",
      "Regression test. A source alarmed as NEVER_SEEN and then delivers its first grid. "
      "The alarm must be cleared with SOURCE_RECOVERED; an alarm that can be raised but "
      "never lowered is worse than none.",
      [poll(3500)]
      + [data(f, t=3600) for f in pack_grid(60, SRC_RIGHT, ramp)]
      + [health(pack_health(60, SRC_RIGHT, ramp), t=3600)],
      True, {EV_NEVER_SEEN: 2, EV_RECOVERED: 1, EV_PUBLISHED: 1},
      {"expected_state_src0": ST_HEALTHY, "expected_state_src1": ST_NEVER_SEEN})

    stale_frames = pack_grid(20, SRC_RIGHT, ramp)
    stale_health = pack_health(20, SRC_RIGHT, ramp)
    recov_frames = pack_grid(21, SRC_RIGHT, GRIDS["boundaries"])
    recov_health = pack_health(21, SRC_RIGHT, GRIDS["boundaries"])
    s("watchdog_stale_then_recovers",
      "The right source publishes at t=0, is HEALTHY at 500 ms, has gone STALE_NO_FRAMES "
      "by 1500 ms, then publishes again at 1600 ms and is HEALTHY at 1650 ms. Recovery is "
      "reported too, so an operator sees the clear as well as the onset. Both alarms are "
      "edge triggered: three polls produce at most one event each.",
      [data(f, t=0) for f in stale_frames] + [health(stale_health, t=0)]
      + [poll(500), poll(1500)]
      + [data(f, t=1600) for f in recov_frames] + [health(recov_health, t=1600)]
      + [poll(1650)],
      True, {EV_PUBLISHED: 2, EV_STALE: 1, EV_RECOVERED: 1},
      {"expected_publish_count": 2, "expected_state_src0": ST_HEALTHY,
       "expected_state_src1": ST_NEVER_SEEN})

    s("watchdog_frames_without_completion",
      "Frames keep arriving for the right source but no grid ever completes. Past the "
      "startup grace plus the stale threshold the source is STALE_NOT_COMPLETING, not "
      "STALE_NO_FRAMES. The distinction matters — this is chunk loss or corruption, "
      "whereas no frames at all would point at the chain, power, enumeration or the CAN "
      "filter. The left source, silent throughout, alarms as NEVER_SEEN in the same poll.",
      [data(f, t=0) for i, f in enumerate(pack_grid(30, SRC_RIGHT, ramp)) if i != 5]
      + [data(f, t=2000) for f in pack_grid(31, SRC_RIGHT, ramp)[:2]]
      + [data(f, t=4000) for f in pack_grid(32, SRC_RIGHT, ramp)[:2]]
      + [poll(4100)],
      False, {EV_TIMEOUT: 3, EV_STALE: 1, EV_NEVER_SEEN: 1},
      {"expected_state_src0": ST_STALE_NOT_COMPLETING,
       "expected_state_src1": ST_NEVER_SEEN})

    s("watchdog_malformed_frames_are_still_frames",
      "A source sends only malformed frames. They name a real source, so they are evidence "
      "the transport is alive and must count towards liveness: the source is "
      "STALE_NOT_COMPLETING, not STALE_NO_FRAMES. Reporting no frames here would send an "
      "investigation towards the chain or the CAN filter when the fault is corruption.",
      [health(bytes([1, (SRC_RIGHT << 4) | 0x5]) + ramp_health[2:], t=0)]
      + [health(bytes([2, (SRC_RIGHT << 4) | 0x5]) + ramp_health[2:], t=2000)]
      + [health(bytes([3, (SRC_RIGHT << 4) | 0x5]) + ramp_health[2:], t=4000)]
      + [poll(4100)],
      False, {EV_MALFORMED: 3, EV_STALE: 1, EV_NEVER_SEEN: 1},
      {"expected_state_src0": ST_STALE_NOT_COMPLETING,
       "expected_state_src1": ST_NEVER_SEEN})

    s("watchdog_staleness_is_per_source",
      "The left source keeps publishing while the right one stops after a single grid. At "
      "1500 ms the right is stale and the left is healthy. Staleness must never be a "
      "single global flag.",
      [data(f, t=0) for f in pack_grid(40, SRC_RIGHT, ramp)]
      + [health(pack_health(40, SRC_RIGHT, ramp), t=0)]
      + [data(f, t=0) for f in pack_grid(40, SRC_LEFT, ramp)]
      + [health(pack_health(40, SRC_LEFT, ramp), t=0)]
      + [data(f, t=1200) for f in pack_grid(41, SRC_LEFT, ramp)]
      + [health(pack_health(41, SRC_LEFT, ramp), t=1200)]
      + [poll(1500)],
      True, {EV_PUBLISHED: 3, EV_STALE: 1},
      {"expected_publish_count": 3,
       "expected_state_src0": ST_STALE_NO_FRAMES,
       "expected_state_src1": ST_HEALTHY})

    return vectors


# ---------------------------------------------------------------- C++ emit

def cpp_bytes(hexstr):
    return "{" + ",".join("0x%s" % hexstr[i:i + 2] for i in range(0, len(hexstr), 2)) + "}"


def emit_header(v, path):
    L = []
    a = L.append
    a("// AUTO-GENERATED by gen_golden_vectors.py -- DO NOT EDIT.")
    a("// Regenerate after any change to %s." % CONTRACT)
    a("//")
    a("// Both LexxHard-SensorControlBoard-Firmware and LexxHard-SCBDriver include this")
    a("// header and pin kContractSha256. A contract edit changes the SHA and fails both")
    a("// pins until the vectors are regenerated and the pins updated deliberately.")
    a("#pragma once")
    a("")
    a("#include <cstddef>")
    a("#include <cstdint>")
    a("")
    a("namespace tof_contract {")
    a("")
    a('inline constexpr const char* kContractVersion = "%s";' % v["contract_version"])
    a('inline constexpr const char* kContractSha256 = "%s";' % v["contract_sha256"])
    a("")
    for k, val in v["constants"].items():
        a("inline constexpr uint32_t k%s = %d;" % (
            "".join(p.capitalize() for p in k.split("_")), val))
    a("")
    a("// kPoll carries no CAN bytes: it means \"advance to t_ms and call poll()\".")
    a("enum class FrameKind { kData, kHealth, kPoll };")
    a("")
    a("struct Frame {")
    a("  FrameKind kind;")
    a("  uint8_t bytes[8];")
    a("  uint32_t t_ms;")
    a("};")
    a("")
    a("struct GridVector {")
    a("  const char* name;")
    a("  uint8_t source_id;")
    a("  uint8_t generation;")
    a("  const uint16_t* zones_mm;      // 64 entries")
    a("  const Frame* data_frames;      // 16 entries")
    a("  Frame health_frame;")
    a("  uint8_t expected_valid_zone_count;")
    a("};")
    a("")
    a("struct ExpectedEvent {")
    a("  const char* name;")
    a("  uint32_t count;")
    a("};")
    a("")
    a("struct Scenario {")
    a("  const char* name;")
    a("  const char* description;")
    a("  const Frame* frames;")
    a("  size_t frame_count;")
    a("  bool publishes;")
    a("  const uint16_t* expected_zones_mm;   // nullptr when not applicable")
    a("  uint32_t expected_publish_count;")
    a("  // The COMPLETE multiset of events the scenario must produce. Any event not")
    a("  // listed must not occur even once, so a decoder cannot pass by emitting the")
    a("  // right event alongside several wrong ones.")
    a("  const ExpectedEvent* expected_events;")
    a("  size_t expected_event_count;")
    a("  const char* expected_state_src0;    // nullptr when not checked")
    a("  const char* expected_state_src1;")
    a("};")
    a("")

    for i, g in enumerate(v["grids"]):
        a("inline constexpr uint16_t kGridZones%d[64] = {%s};"
          % (i, ",".join(str(z) for z in g["zones_mm"])))
        a("inline constexpr Frame kGridFrames%d[16] = {" % i)
        for f in g["data_frames"]:
            a("  {FrameKind::kData, %s, 0}," % cpp_bytes(f))
        a("};")
    a("")
    a("inline constexpr GridVector kGridVectors[%d] = {" % len(v["grids"]))
    for i, g in enumerate(v["grids"]):
        a('  {"%s", %d, %d, kGridZones%d, kGridFrames%d, '
          "{FrameKind::kHealth, %s, 0}, %d},"
          % (g["name"], g["source_id"], g["generation"], i, i,
             cpp_bytes(g["health_frame"]), g["expected_valid_zone_count"]))
    a("};")
    a("")

    zone_refs = {}
    for i, g in enumerate(v["grids"]):
        zone_refs[tuple(g["zones_mm"])] = "kGridZones%d" % i
    extra = 0
    for sc in v["scenarios"]:
        z = sc.get("expected_zones_mm")
        if z and tuple(z) not in zone_refs:
            a("inline constexpr uint16_t kScenarioZones%d[64] = {%s};"
              % (extra, ",".join(str(x) for x in z)))
            zone_refs[tuple(z)] = "kScenarioZones%d" % extra
            extra += 1

    for i, sc in enumerate(v["scenarios"]):
        ev = sc.get("expected_events", {})
        if ev:
            a("inline constexpr ExpectedEvent kScenarioEvents%d[%d] = {"
              % (i, len(ev)))
            for name, cnt in sorted(ev.items()):
                a('  {"%s", %d},' % (name, cnt))
            a("};")
        a("inline constexpr Frame kScenarioFrames%d[%d] = {" % (i, len(sc["frames"])))
        for f in sc["frames"]:
            kind = {"data": "kData", "health": "kHealth", "poll": "kPoll"}[f["kind"]]
            payload = cpp_bytes(f["hex"]) if f["hex"] else "{0,0,0,0,0,0,0,0}"
            a("  {FrameKind::%s, %s, %d}," % (kind, payload, f["t_ms"]))
        a("};")
    a("")
    a("inline constexpr Scenario kScenarios[%d] = {" % len(v["scenarios"]))
    for i, sc in enumerate(v["scenarios"]):
        z = sc.get("expected_zones_mm")
        zref = zone_refs[tuple(z)] if z else "nullptr"
        def q(key):
            v = sc.get(key)
            return '"%s"' % v if v else "nullptr"
        ev = sc.get("expected_events", {})
        eref = "kScenarioEvents%d" % i if ev else "nullptr"
        a('  {"%s",\n   "%s",\n   kScenarioFrames%d, %d, %s, %s, %d, %s, %d, %s, %s},'
          % (sc["name"],
             sc["description"].replace('"', '\\"'),
             i, len(sc["frames"]),
             "true" if sc["publishes"] else "false",
             zref,
             sc.get("expected_publish_count", 1 if sc["publishes"] else 0),
             eref, len(ev),
             q("expected_state_src0"), q("expected_state_src1")))
    a("};")
    a("")
    a("inline constexpr size_t kGridVectorCount = %d;" % len(v["grids"]))
    a("inline constexpr size_t kScenarioCount = %d;" % len(v["scenarios"]))
    a("")
    a("}  // namespace tof_contract")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


def main():
    with open(os.path.join(HERE, CONTRACT), "rb") as f:
        contract_sha = hashlib.sha256(f.read()).hexdigest()

    vectors = build(contract_sha)

    js = os.path.join(HERE, "tof_grid_golden_vectors.json")
    with open(js, "w") as f:
        json.dump(vectors, f, indent=2)
        f.write("\n")

    hh = os.path.join(HERE, "tof_contract_vectors.h")
    emit_header(vectors, hh)

    pub = sum(1 for s in vectors["scenarios"] if s["publishes"])
    print("contract %s sha256 %s" % (CONTRACT_VERSION, contract_sha))
    print("wrote %s" % js)
    print("wrote %s" % hh)
    print("  %d byte-exact grid vectors" % len(vectors["grids"]))
    print("  %d decoder scenarios (%d publishing, %d rejecting)"
          % (len(vectors["scenarios"]), pub, len(vectors["scenarios"]) - pub))


if __name__ == "__main__":
    main()
