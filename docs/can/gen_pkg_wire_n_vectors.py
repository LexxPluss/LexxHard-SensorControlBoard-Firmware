#!/usr/bin/env python3
# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Golden vectors for candidate N's Package ToF wire layout, development version: the eight bytes of
# a data frame and of a health frame.
#
# WHAT THESE PIN. Field positions and bit order, the reserved bits, the three-value state range,
# and -- the part that is not arithmetic -- the separation of the two health-frame purposes. A
# health frame is grid metadata (same sequence as its 64 data frames) or an independent diagnostic
# (no acquisition, must not reassemble, must not refresh a grid's timestamp), and the vectors below
# include every malformed combination that would let one be read as the other.
#
# WHAT THEY DO NOT SETTLE. Candidate N itself: a sequence that never restarts is only an identity
# if the issuer really survives power loss, and that is a separate acceptance item. Nor any CAN id
# -- registration is pending and no id appears here. Nor the L0 contract, which is untouched.
#
# WHY A SECOND IMPLEMENTATION. Mirroring the C++ shift arithmetic in Python proves only that the
# same mistake was made twice. This file builds the sequence and distance fields with
# struct.pack('>I'/'>H', ...) and the zone and flag bytes from an explicit string of bits, so
# agreement on eight bytes means the positions are right in a way one implementation cannot assert
# about itself.
#
# It lives in docs/can/ because scripts/check_language_boundary.sh allows Python nowhere else, and
# the header it writes says in its own first lines that it is not a frozen contract.

import io
import os
import struct

FRAME_BYTES = 8
ZONES = 64

STATE_OK, STATE_DEGRADED, STATE_ERROR = 0, 1, 2
METADATA, DIAGNOSTIC = 0, 1


def bits_to_byte(bits):
    """bits[0] is bit 7 ... bits[7] is bit 0. Deliberately not a shift expression."""
    assert len(bits) == 8, bits
    return int(''.join('1' if b else '0' for b in bits), 2)


def zone_byte(zone):
    """Bits 5..0 carry the zone; bits 7..6 are reserved and written zero."""
    assert 0 <= zone <= 63, zone
    six = format(zone, '06b')
    return bits_to_byte([False, False] + [c == '1' for c in six])


def flags_byte(purpose, inactive):
    """Bit 0 independent diagnostic, bit 1 INACTIVE, bits 7..2 reserved zero."""
    return bits_to_byte([False, False, False, False, False, False,
                         inactive, purpose == DIAGNOSTIC])


def data_frame(sequence, zone, distance, status):
    frame = struct.pack('>I', sequence) + bytes([zone_byte(zone)]) + \
        struct.pack('>H', distance) + bytes([status])
    assert len(frame) == FRAME_BYTES, len(frame)
    return list(frame)


def health_frame(sequence, state, purpose, inactive):
    frame = struct.pack('>I', sequence) + bytes([state, flags_byte(purpose, inactive)]) + \
        b'\x00\x00'
    assert len(frame) == FRAME_BYTES, len(frame)
    return list(frame)


def data_vectors():
    """(name, sequence, zone, distance, status, frame bytes)."""
    out = []

    def add(name, sequence, zone, distance, status):
        out.append((name, sequence, zone, distance, status,
                    data_frame(sequence, zone, distance, status)))

    # Sequence 0 is reserved for the diagnostic placeholder, so the smallest legal one is 1.
    add('smallest_legal', 1, 0, 0, 0)
    add('largest', 0xFFFFFFFF, 63, 0xFFFF, 0xFF)
    # Asymmetric on purpose: every byte distinct, so a swapped pair cannot pass.
    add('distinct_bytes', 0x01020304, 0x15, 0x0607, 0x08)
    # 0xFFFF is a distance like any other here. Nothing in this layer calls it invalid.
    add('sentinel_distance_is_a_value', 0x1234, 7, 0xFFFF, 0x05)
    # A deliberately contradictory pairing: carried verbatim, not reconciled.
    add('contradiction_is_carried', 9, 1, 0, 0xFF)
    add('status_no_target', 0x7FFFFFFF, 62, 0x0FFF, 255)
    add('status_not_updated', 0x80000000, 31, 0x1000, 0)
    add('status_low_confidence', 0x0000FFFF, 32, 0x0001, 9)

    # Walking one over every bit of every field. One set bit in the fields must produce exactly one
    # set bit in the frame, at the position this file computes independently.
    for k in range(32):
        add('seq_bit_%02d' % k, 1 << k, 0, 0, 0)
    for k in range(6):
        add('zone_bit_%d' % k, 1, 1 << k, 0, 0)
    for k in range(16):
        add('distance_bit_%02d' % k, 1, 0, 1 << k, 0)
    for k in range(8):
        add('status_bit_%d' % k, 1, 0, 0, 1 << k)
    return out


def data_refusals():
    """(name, frame bytes, expected result)."""
    out = []
    base = data_frame(0x01020304, 0x15, 0x0607, 0x08)

    for bit, name in ((6, 'zone_reserved_bit_6'), (7, 'zone_reserved_bit_7')):
        frame = list(base)
        frame[4] |= 1 << bit
        out.append((name, frame, 'reserved_bits_set'))
    frame = list(base)
    frame[4] |= 0xC0
    out.append(('zone_reserved_both', frame, 'reserved_bits_set'))

    # The reason the number is reserved: an all-zero frame must not read as a measurement of zone 0
    # at 0 mm. Nothing downstream could tell that from a real one.
    out.append(('all_zero_frame', [0] * FRAME_BYTES, 'data_sequence_zero'))
    frame = data_frame(0, 0x15, 0x0607, 0x08)
    out.append(('sequence_zero_with_real_fields', frame, 'data_sequence_zero'))
    return out


def health_vectors():
    """(name, sequence, state, purpose, inactive, frame bytes)."""
    out = []

    def add(name, sequence, state, purpose, inactive):
        out.append((name, sequence, state, purpose, inactive,
                    health_frame(sequence, state, purpose, inactive)))

    add('metadata_ok', 1, STATE_OK, METADATA, False)
    add('metadata_degraded', 0x01020304, STATE_DEGRADED, METADATA, False)
    add('metadata_error', 0x01020304, STATE_ERROR, METADATA, False)
    add('metadata_largest_sequence', 0xFFFFFFFF, STATE_OK, METADATA, False)
    # The diagnostic carries no acquisition, which is the whole point: it cannot reuse a grid's
    # number and so cannot rewrite that grid's health after the consumer has acted on it.
    add('diagnostic_error', 0, STATE_ERROR, DIAGNOSTIC, False)
    add('diagnostic_inactive', 0, STATE_ERROR, DIAGNOSTIC, True)
    # Structurally legal, and this layer does not police it: which state a diagnostic may carry is
    # tof_package_health's decision, not the wire's.
    add('diagnostic_ok_is_structurally_legal', 0, STATE_OK, DIAGNOSTIC, False)
    add('diagnostic_degraded_inactive', 0, STATE_DEGRADED, DIAGNOSTIC, True)

    for k in range(32):
        value = 1 << k
        add('metadata_seq_bit_%02d' % k, value, STATE_OK, METADATA, False)
    return out


def health_refusals():
    """(name, frame bytes, expected result)."""
    out = []
    base = health_frame(0x01020304, STATE_DEGRADED, METADATA, False)

    for state in (3, 4, 0x7F, 0xFF):
        frame = list(base)
        frame[4] = state
        out.append(('state_%d' % state, frame, 'state_out_of_range'))

    for bit in range(2, 8):
        frame = list(base)
        frame[5] |= 1 << bit
        out.append(('flags_reserved_bit_%d' % bit, frame, 'reserved_bits_set'))
    for index in (6, 7):
        for value in (0x01, 0x80, 0xFF):
            frame = list(base)
            frame[index] = value
            out.append(('reserved_byte_%d_is_%02x' % (index, value), frame, 'reserved_bits_set'))

    # INACTIVE on metadata: a grid that exists cannot come from a sensor that never started.
    frame = list(base)
    frame[5] = 0x02
    out.append(('inactive_on_metadata', frame, 'inactive_without_diagnostic'))

    # A diagnostic naming an acquisition -- exactly the reuse the separation prevents.
    frame = health_frame(1, STATE_ERROR, DIAGNOSTIC, False)
    frame[0:4] = list(struct.pack('>I', 1))
    out.append(('diagnostic_names_an_acquisition', frame, 'diagnostic_sequence_not_zero'))
    frame = list(struct.pack('>I', 0xFFFFFFFF)) + [STATE_ERROR, 0x03, 0, 0]
    out.append(('inactive_diagnostic_names_an_acquisition', frame,
                'diagnostic_sequence_not_zero'))

    # The alias a single flipped flag bit would produce: a diagnostic read as metadata for grid 0.
    out.append(('metadata_numbered_zero', [0, 0, 0, 0, STATE_ERROR, 0x00, 0, 0],
                'metadata_sequence_zero'))
    out.append(('all_zero_health_frame', [0] * FRAME_BYTES, 'metadata_sequence_zero'))
    return out


def emit_bytes(values):
    return '{' + ', '.join('0x%02X' % v for v in values) + '}'


def render():
    out = io.StringIO()
    out.write('''/*
 * AUTO-GENERATED by gen_pkg_wire_n_vectors.py -- DO NOT EDIT.
 *
 * Golden vectors for candidate N's Package ToF wire layout, development version:
 * 8-byte data frames (full uint32 sequence, 6-bit zone, uint16 distance, raw
 * status) and 8-byte health frames in their two separate purposes, grid metadata
 * and independent diagnostic.
 *
 * A DEVELOPMENT LAYOUT, NOT A FROZEN CONTRACT. No CAN id appears here --
 * registration is pending. Candidate N's persistence and its field storage
 * acceptance are separate open items, and no SHA is pinned on this header in
 * either repository. Production source may not include it.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace pkg_wire_n_vectors {

inline constexpr size_t kFrameBytes{8};
inline constexpr size_t kZones{64};
/* One zone per data frame plus exactly one metadata frame. Independent diagnostics
 * belong to no grid and are never one of the 65. */
inline constexpr size_t kFramesPerGrid{65};

struct data_vector {
    const char *name;
    uint32_t sequence;
    uint8_t zone;
    uint16_t distance_mm;
    uint8_t status;
    uint8_t frame[kFrameBytes];
};

struct health_vector {
    const char *name;
    uint32_t sequence;
    uint8_t state;
    /* 0 grid metadata, 1 independent diagnostic. */
    uint8_t purpose;
    uint8_t inactive;
    uint8_t frame[kFrameBytes];
};

struct refusal_vector {
    const char *name;
    uint8_t frame[kFrameBytes];
    /* The expected result_name(), compared as a string so that renumbering the
     * enum cannot make a wrong refusal pass. */
    const char *expected;
};

''')

    out.write('inline constexpr data_vector kDataVectors[]{\n')
    for name, seq, zone, dist, status, frame in data_vectors():
        out.write('    {"%s", 0x%08XU, %d, 0x%04XU, 0x%02XU, %s},\n'
                  % (name, seq, zone, dist, status, emit_bytes(frame)))
    out.write('};\n\n')

    out.write('inline constexpr refusal_vector kDataRefusals[]{\n')
    for name, frame, expected in data_refusals():
        out.write('    {"%s", %s, "%s"},\n' % (name, emit_bytes(frame), expected))
    out.write('};\n\n')

    out.write('inline constexpr health_vector kHealthVectors[]{\n')
    for name, seq, state, purpose, inactive, frame in health_vectors():
        out.write('    {"%s", 0x%08XU, %d, %d, %d, %s},\n'
                  % (name, seq, state, purpose, 1 if inactive else 0, emit_bytes(frame)))
    out.write('};\n\n')

    out.write('inline constexpr refusal_vector kHealthRefusals[]{\n')
    for name, frame, expected in health_refusals():
        out.write('    {"%s", %s, "%s"},\n' % (name, emit_bytes(frame), expected))
    out.write('};\n\n')

    out.write('inline constexpr size_t kDataVectorCount{%d};\n' % len(data_vectors()))
    out.write('inline constexpr size_t kDataRefusalCount{%d};\n' % len(data_refusals()))
    out.write('inline constexpr size_t kHealthVectorCount{%d};\n' % len(health_vectors()))
    out.write('inline constexpr size_t kHealthRefusalCount{%d};\n' % len(health_refusals()))
    out.write('\n}\n')
    return out.getvalue()


def main(argv):
    check = '--check' in argv
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'tof_pkg_wire_n_vectors.h')
    rendered = render()
    if check:
        # A stale header would let a byte-order or field-position change pass unnoticed, which is
        # the one failure golden vectors exist to prevent.
        with open(path, encoding='utf-8') as handle:
            if handle.read() != rendered:
                print('STALE: %s does not match the generator' % path)
                return 1
        print('%s is up to date' % path)
        return 0

    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(rendered)
    print('wrote %s: %d data, %d data refusals, %d health, %d health refusals'
          % (path, len(data_vectors()), len(data_refusals()),
             len(health_vectors()), len(health_refusals())))
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv[1:]))
