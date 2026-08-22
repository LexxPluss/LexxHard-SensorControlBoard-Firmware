/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side tests for the cliff measurement packer.
 *
 * Two things are being pinned. The contract SHA, which is the firmware half of the
 * cross-repository lock -- SCBDriver pins the same string on the decoder side, so a
 * contract edit fails both until the literals are updated deliberately. And the
 * normative reduction, which the layout vectors cannot cover: the pre-reduction target
 * list never reaches the wire, so the decoder has no way to see it and no shared vector
 * can express it. That asymmetry is why these cases live here.
 *
 * The ULD's sources are deliberately absent from this build. The packer includes
 * tof_cliff_sample.h rather than tof_cliff_sensor.h precisely so that stays possible.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_cliff_contract_vectors.h"
#include "tof_cliff_packer.hpp"

namespace ctr = tof_cliff_contract;
namespace pk = tof_cliff_packer;

namespace {

struct target_in {
    int16_t mm;
    uint8_t status;
};

tof_cliff_sample make_sample(uint8_t target_count, uint8_t entry_count,
                             const target_in *in, uint8_t n)
{
    tof_cliff_sample s{};
    s.fresh = true;
    s.target_count = target_count;
    s.entry_count = entry_count;
    for (uint8_t i = 0; i < n && i < TOF_CLIFF_MAX_TARGETS; ++i) {
        s.entries[i].range_mm = in[i].mm;
        s.entries[i].range_status = in[i].status;
    }
    return s;
}

const ctr::vector *find_vector(const char *name)
{
    for (size_t i = 0; i < ctr::kVectorCount; ++i)
        if (strcmp(ctr::kVectors[i].name, name) == 0)
            return &ctr::kVectors[i];
    return nullptr;
}

} // namespace

ZTEST_SUITE(tof_cliff_packer, NULL, NULL, NULL, NULL, NULL);

/* ------------------------------------------------------------------ the pin ------ */

ZTEST(tof_cliff_packer, test_contract_sha_pin)
{
    zassert_equal(0, strcmp(ctr::kContractSha256,
        "fb94706a4d2488aa9acdc7c7defcd7fac92379cba01964ab31f949fa50955188"));
    zassert_equal(0, strcmp(ctr::kContractVersion, "commissioning-2026-08-18c"));
    /* The contract SHA says which contract; this says which generated artefacts. It is
     * pinned separately because the generator has twice changed what it emits while the
     * contract text -- and so its SHA -- stood still. */
    zassert_equal(0, strcmp(ctr::kArtefactSetId,
        "3db018e9f0be3ae85a295240a8314fff97491b587ec909021b8e478745caf9aa"));
    zassert_equal(0, strcmp(ctr::kProfileName, "commissioning-cliff-only-400k"));
    /* Asserted rather than merely present: this revision is not releasable, and the day
     * someone flips it must be a deliberate act that shows up in this diff. */
    zassert_true(ctr::kReleaseForbidden);
    zassert_equal(ctr::kMeasId, 0x216);
    zassert_equal(ctr::kHealthId, 0x217);
    zassert_equal(ctr::kProtocolVersion, 0x1);
    zassert_equal(ctr::kSentinelInvalid, 0xFFFF);
    zassert_equal(ctr::kDlc, 8);
    /* Distinct concepts that happen to share a value. */
    zassert_equal(ctr::kMaxTargets, TOF_CLIFF_MAX_TARGETS);
    zassert_equal(ctr::kSourceCount, 4);
}

/* --------------------------------------------------- the zero-target rule -------- */

ZTEST(tof_cliff_packer, test_zero_targets_with_status_255_is_the_only_legal_shape)
{
    const target_in in[1]{{8191, 255}};
    const auto s{make_sample(0, 1, in, 1)};
    const auto r{pk::reduce(s)};

    zassert_equal(static_cast<int>(r.outcome), static_cast<int>(pk::result::frame_ready));
    zassert_equal(r.error, 0);
    zassert_equal(r.raw_status, 255);
    zassert_equal(r.range_mm, ctr::kSentinelInvalid, "8191 must never reach the wire");
    zassert_equal(r.target_count, 0);

    uint8_t out[8]{};
    zassert_true(pk::encode_measurement(r, 0, 1, 0, out));
    const auto *v{find_vector("meas_status_255_no_target")};
    zassert_not_null(v);
    zassert_equal(0, memcmp(out, v->bytes, 8), "must be byte-identical to the vector");
}

ZTEST(tof_cliff_packer, test_zero_targets_with_another_status_is_eproto_and_no_frame)
{
    /* Rewriting this to 255 would hide an anomaly in the read layer or the ULD behind
     * an ordinary-looking no-target frame, which for a cliff sensor reads as a cliff
     * rather than as a fault. */
    const target_in in[1]{{8191, 5}};
    const auto s{make_sample(0, 1, in, 1)};
    const auto r{pk::reduce(s)};

    zassert_equal(static_cast<int>(r.outcome), static_cast<int>(pk::result::unencodable));
    zassert_equal(r.error, -EPROTO);
    zassert_equal(static_cast<int>(r.why),
                  static_cast<int>(pk::reason::zero_targets_with_unexpected_status));
    zassert_equal(r.observed_status, 5, "the observed status must survive for the log");

    uint8_t out[8];
    memset(out, 0xA5, sizeof out);
    zassert_false(pk::encode_measurement(r, 0, 1, 0, out));
    for (size_t i = 0; i < sizeof out; ++i)
        zassert_equal(out[i], 0xA5, "a refused encode must not leave a partial payload");
}

ZTEST(tof_cliff_packer, test_zero_targets_with_entry_count_not_one_is_eproto)
{
    const target_in in[2]{{8191, 255}, {900, 0}};
    const auto s{make_sample(0, 2, in, 2)};
    const auto r{pk::reduce(s)};

    zassert_equal(static_cast<int>(r.outcome), static_cast<int>(pk::result::unencodable));
    zassert_equal(r.error, -EPROTO);
    zassert_equal(static_cast<int>(r.why),
                  static_cast<int>(pk::reason::zero_targets_entry_count_not_one));

    uint8_t out[8];
    memset(out, 0x5A, sizeof out);
    zassert_false(pk::encode_measurement(r, 0, 1, 0, out));
    zassert_equal(out[0], 0x5A);
}

/* ------------------------------------------------- which status is transmitted --- */

ZTEST(tof_cliff_packer, test_surviving_class_takes_the_lowest_uld_index)
{
    /* Both SENSOR_FAULT. Index decides, and the contract forbids sorting first. */
    const target_in in[2]{{100, 5}, {200, 8}};
    const auto s{make_sample(2, 2, in, 2)};
    const auto r{pk::reduce(s)};

    zassert_equal(static_cast<int>(r.outcome), static_cast<int>(pk::result::frame_ready));
    zassert_equal(static_cast<int>(r.cls), static_cast<int>(ctr::status_class::sensor_fault));
    zassert_equal(r.raw_status, 5);
    zassert_equal(r.range_mm, ctr::kSentinelInvalid);
    zassert_equal(r.target_count, 2);
}

ZTEST(tof_cliff_packer, test_reversing_the_targets_reverses_the_transmitted_status)
{
    /* The same two statuses in the other order. If the implementation sorted, or picked
     * the lowest numeric value, this would still report 5 -- which is exactly the
     * ambiguity the rule was added to remove. */
    const target_in in[2]{{100, 8}, {200, 5}};
    const auto s{make_sample(2, 2, in, 2)};
    const auto r{pk::reduce(s)};

    zassert_equal(static_cast<int>(r.outcome), static_cast<int>(pk::result::frame_ready));
    zassert_equal(r.raw_status, 8, "lowest index, not lowest numeric value");
}

/* ---------------------------------------------------------- class priority ------- */

ZTEST(tof_cliff_packer, test_one_faulty_target_condemns_the_measurement)
{
    const target_in in[2]{{900, 0}, {150, 5}};
    const auto s{make_sample(2, 2, in, 2)};
    const auto r{pk::reduce(s)};

    zassert_equal(static_cast<int>(r.cls), static_cast<int>(ctr::status_class::sensor_fault));
    zassert_equal(r.range_mm, ctr::kSentinelInvalid, "no trustworthy distance survives");
    zassert_equal(r.raw_status, 5);
}

ZTEST(tof_cliff_packer, test_valid_alongside_no_target_reduces_to_no_target)
{
    const target_in in[2]{{900, 0}, {0, 2}};
    const auto s{make_sample(2, 2, in, 2)};
    const auto r{pk::reduce(s)};

    zassert_equal(static_cast<int>(r.cls), static_cast<int>(ctr::status_class::no_target));
    zassert_equal(r.range_mm, ctr::kSentinelInvalid);
    zassert_equal(r.raw_status, 2);
}

ZTEST(tof_cliff_packer, test_all_valid_transmits_the_farthest_and_its_own_status)
{
    /* Farthest, not nearest: a spurious near return must never mask a real drop behind
     * it. The status comes from that same target, so the frame describes one target. */
    const target_in in[4]{{300, 0}, {500, 0}, {1100, 0}, {700, 0}};
    const auto s{make_sample(4, 4, in, 4)};
    const auto r{pk::reduce(s)};

    zassert_equal(static_cast<int>(r.cls), static_cast<int>(ctr::status_class::valid_range));
    zassert_equal(r.range_mm, 1100);
    zassert_equal(r.raw_status, 0);
    zassert_equal(r.target_count, 4);
}

ZTEST(tof_cliff_packer, test_no_sample_survives_to_no_frame_without_an_error)
{
    /* Status 6 is a start-up artefact the ULD tells us to discard. Sending nothing is
     * the correct outcome, so this is not an error -- the source's sample_produced bit
     * simply stays clear for the cycle. */
    const target_in in[2]{{900, 0}, {400, 6}};
    const auto s{make_sample(2, 2, in, 2)};
    const auto r{pk::reduce(s)};

    zassert_equal(static_cast<int>(r.outcome), static_cast<int>(pk::result::no_frame));
    zassert_equal(r.error, 0, "no frame is a correct outcome, not a failure");

    uint8_t out[8];
    memset(out, 0x3C, sizeof out);
    zassert_false(pk::encode_measurement(r, 0, 1, 0, out));
    zassert_equal(out[7], 0x3C);
}

/* ------------------------------------------------------------- refusals --------- */

ZTEST(tof_cliff_packer, test_an_unclassifiable_status_refuses_the_measurement)
{
    const target_in in[1]{{900, 100}};
    const auto s{make_sample(1, 1, in, 1)};
    const auto r{pk::reduce(s)};

    zassert_equal(r.error, -EPROTO);
    zassert_equal(static_cast<int>(r.why), static_cast<int>(pk::reason::status_undefined));
    zassert_equal(r.observed_status, 100);
}

ZTEST(tof_cliff_packer, test_a_negative_range_under_a_valid_status_refuses)
{
    /* The read layer keeps the range signed on purpose, so this arrives intact. The ULD
     * should have flagged it as status 14; seeing it as VALID means the two disagree. */
    const target_in in[1]{{-5, 0}};
    const auto s{make_sample(1, 1, in, 1)};
    const auto r{pk::reduce(s)};

    zassert_equal(r.error, -EPROTO);
    zassert_equal(static_cast<int>(r.why),
                  static_cast<int>(pk::reason::valid_range_negative));
}

ZTEST(tof_cliff_packer, test_target_count_above_the_array_refuses)
{
    const target_in in[1]{{900, 0}};
    auto s{make_sample(1, 1, in, 1)};
    s.target_count = TOF_CLIFF_MAX_TARGETS + 1;
    const auto r{pk::reduce(s)};

    zassert_equal(r.error, -EPROTO);
    zassert_equal(static_cast<int>(r.why),
                  static_cast<int>(pk::reason::target_count_malformed));
}

/* -------------------------------------------------------------- encoding -------- */

ZTEST(tof_cliff_packer, test_a_valid_frame_is_byte_identical_to_the_vector)
{
    const target_in in[1]{{1234, 0}};
    const auto s{make_sample(1, 1, in, 1)};
    const auto r{pk::reduce(s)};
    zassert_equal(static_cast<int>(r.outcome), static_cast<int>(pk::result::frame_ready));

    uint8_t out[8]{};
    zassert_true(pk::encode_measurement(r, 0, 1, 0, out));

    const auto *v{find_vector("meas_role_0_front_left")};
    zassert_not_null(v);
    zassert_equal(static_cast<int>(v->expected), static_cast<int>(ctr::verdict::accept));
    zassert_equal(0, memcmp(out, v->bytes, 8));
}

ZTEST(tof_cliff_packer, test_every_source_id_encodes_and_out_of_range_refuses)
{
    const target_in in[1]{{1234, 0}};
    const auto r{pk::reduce(make_sample(1, 1, in, 1))};

    for (uint8_t src = 0; src < ctr::kSourceCount; ++src) {
        uint8_t out[8]{};
        zassert_true(pk::encode_measurement(r, src, 1, 0, out));
        zassert_equal(out[0], static_cast<uint8_t>((ctr::kMeasFrameType << 4) | src));
    }
    uint8_t out[8];
    memset(out, 0x11, sizeof out);
    zassert_false(pk::encode_measurement(r, ctr::kSourceCount, 1, 0, out));
    zassert_equal(out[0], 0x11);
}

/* ---------------------------------------------------------------- health -------- */

ZTEST(tof_cliff_packer, test_the_ready_health_shape_matches_the_vector)
{
    pk::health_fields h{};
    h.mapping_epoch = 1;
    h.health_seq = 7;
    h.mapping_state = 0x1; // PROVEN
    h.flags = ctr::kCycleValidBit;
    h.enumerated_mask = 0xF;
    h.model_verified_mask = 0xF;
    h.sample_produced_mask = 0xF;
    h.sensor_fault_mask = 0x0;
    h.failing_chain_position = ctr::kChainPositionNone;
    h.cycle_seq = 0;

    uint8_t out[8]{};
    zassert_true(pk::encode_health(h, out));

    const auto *v{find_vector("health_proven_cycle_valid")};
    zassert_not_null(v);
    zassert_equal(0, memcmp(out, v->bytes, 8));
}

ZTEST(tof_cliff_packer, test_health_refuses_a_per_cycle_field_without_cycle_valid)
{
    pk::health_fields base{};
    base.mapping_state = 0x1;
    base.flags = 0; // cycle_valid clear: a heartbeat
    base.enumerated_mask = 0xF;
    base.model_verified_mask = 0xF;
    base.failing_chain_position = ctr::kChainPositionNone;

    uint8_t out[8]{};
    zassert_true(pk::encode_health(base, out), "the empty heartbeat itself is legal");

    auto with_cycle{base};
    with_cycle.cycle_seq = 3;
    zassert_false(pk::encode_health(with_cycle, out));

    auto with_produced{base};
    with_produced.sample_produced_mask = 0xF;
    zassert_false(pk::encode_health(with_produced, out));

    /* sensor_fault_mask is per cycle too -- the gap that was missed the first time. */
    auto with_fault{base};
    with_fault.sensor_fault_mask = 0x1;
    zassert_false(pk::encode_health(with_fault, out));
}

ZTEST(tof_cliff_packer, test_health_refuses_a_fault_bit_outside_the_produced_mask)
{
    pk::health_fields h{};
    h.mapping_state = 0x1;
    h.flags = ctr::kCycleValidBit;
    h.enumerated_mask = 0xF;
    h.model_verified_mask = 0xF;
    h.sample_produced_mask = 0x3;
    h.sensor_fault_mask = 0x4; // classifies a sample that does not exist
    h.failing_chain_position = ctr::kChainPositionNone;

    uint8_t out[8]{};
    zassert_false(pk::encode_health(h, out));

    h.sensor_fault_mask = 0x1; // a strict subset is legal
    zassert_true(pk::encode_health(h, out));
}

ZTEST(tof_cliff_packer, test_health_refuses_a_named_position_without_a_chain_fault)
{
    pk::health_fields h{};
    h.mapping_state = 0x1;
    h.flags = ctr::kCycleValidBit;
    h.enumerated_mask = 0xF;
    h.model_verified_mask = 0xF;
    h.sample_produced_mask = 0xF;
    h.failing_chain_position = 3;

    uint8_t out[8]{};
    zassert_false(pk::encode_health(h, out));

    /* The complement, and the reason the contract's rule is compound: an incomplete
     * enumeration is itself why a position can be named. */
    h.enumerated_mask = 0x7;
    h.model_verified_mask = 0x7;
    h.sample_produced_mask = 0x0;
    h.cycle_seq = 0;
    zassert_true(pk::encode_health(h, out));
}

ZTEST(tof_cliff_packer, test_health_refuses_malformed_state_and_position)
{
    pk::health_fields h{};
    h.flags = ctr::kCycleValidBit;
    h.failing_chain_position = ctr::kChainPositionNone;

    uint8_t out[8]{};
    h.mapping_state = 0x4;
    zassert_false(pk::encode_health(h, out));

    h.mapping_state = 0x1;
    h.failing_chain_position = 7;
    zassert_false(pk::encode_health(h, out));
    h.failing_chain_position = 0;
    zassert_false(pk::encode_health(h, out));
}

/* --------------------------------------------- staleness and array agreement ---- */

ZTEST(tof_cliff_packer, test_a_stale_sample_produces_no_frame)
{
    /* fresh == false means no new sample was ready, so entries hold whatever the
     * previous read left. Packing them would transmit an old distance as a fresh one,
     * which the contract forbids outright. Not an error: nothing was produced. */
    const target_in in[1]{{900, 0}};
    auto s{make_sample(1, 1, in, 1)};
    s.fresh = false;
    const auto r{pk::reduce(s)};

    zassert_equal(static_cast<int>(r.outcome), static_cast<int>(pk::result::no_frame));
    zassert_equal(r.error, 0);

    uint8_t out[8];
    memset(out, 0x77, sizeof out);
    zassert_false(pk::encode_measurement(r, 0, 1, 0, out));
    zassert_equal(out[0], 0x77);
}

ZTEST(tof_cliff_packer, test_entry_count_must_agree_with_target_count)
{
    /* Trusting target_count while the array is shorter would classify entries nobody
     * filled, and whatever is in them would look like device data. */
    const target_in in[4]{{900, 0}, {0, 0}, {0, 0}, {0, 0}};
    auto s{make_sample(3, 1, in, 1)};
    const auto r{pk::reduce(s)};

    zassert_equal(r.error, -EPROTO);
    zassert_equal(static_cast<int>(r.why), static_cast<int>(pk::reason::entry_count_mismatch));

    s = make_sample(2, 3, in, 3);
    zassert_equal(pk::reduce(s).error, -EPROTO, "the other direction is wrong too");
}

ZTEST(tof_cliff_packer, test_status_255_among_positive_targets_is_eproto)
{
    /* Reducing this would build status 255 with a non-zero target_count, which breaks
     * the bidirectional invariant -- the decoder must reject it, so the defect would
     * surface as a discarded frame instead of being diagnosed at the producer. */
    const target_in in[2]{{900, 0}, {8191, 255}};
    const auto s{make_sample(2, 2, in, 2)};
    const auto r{pk::reduce(s)};

    zassert_equal(r.error, -EPROTO);
    zassert_equal(static_cast<int>(r.why),
                  static_cast<int>(pk::reason::none_status_among_targets));
    zassert_equal(r.observed_status, 255);
}

/* ------------------------------------------------ a hand-built reduction -------- */

ZTEST(tof_cliff_packer, test_encode_refuses_a_reduction_reduce_could_not_produce)
{
    /* The struct is public so tests and diagnostics can read it, which means a caller
     * can also assemble one. Each of these would encode into a frame the decoder is
     * obliged to reject. */
    pk::reduction forged{};
    forged.outcome = pk::result::frame_ready;
    uint8_t out[8];
    pk::reason why{pk::reason::none};

    /* VALID_RANGE carrying the sentinel. */
    forged.cls = ctr::status_class::valid_range;
    forged.raw_status = 0;
    forged.range_mm = ctr::kSentinelInvalid;
    forged.target_count = 1;
    memset(out, 0x42, sizeof out);
    zassert_false(pk::encode_measurement(forged, 0, 1, 0, out, &why));
    zassert_equal(static_cast<int>(why), static_cast<int>(pk::reason::reduction_inconsistent));
    zassert_equal(out[0], 0x42, "still no partial payload");

    /* A NO_TARGET status carrying a finite range. */
    forged.cls = ctr::status_class::no_target;
    forged.raw_status = 2;
    forged.range_mm = 1234;
    zassert_false(pk::encode_measurement(forged, 0, 1, 0, out, &why));

    /* target_count 0 without the status-255 encoding. */
    forged.cls = ctr::status_class::sensor_fault;
    forged.raw_status = 5;
    forged.range_mm = ctr::kSentinelInvalid;
    forged.target_count = 0;
    zassert_false(pk::encode_measurement(forged, 0, 1, 0, out, &why));

    /* A class that disagrees with its own status. */
    forged.cls = ctr::status_class::valid_range;
    forged.raw_status = 5; // SENSOR_FAULT
    forged.range_mm = 900;
    forged.target_count = 1;
    zassert_false(pk::encode_measurement(forged, 0, 1, 0, out, &why));

    /* A NO_SAMPLE status, which is never transmitted. */
    forged.cls = ctr::status_class::no_sample;
    forged.raw_status = 6;
    forged.range_mm = ctr::kSentinelInvalid;
    zassert_false(pk::encode_measurement(forged, 0, 1, 0, out, &why));
}

ZTEST(tof_cliff_packer, test_encode_reports_an_out_of_range_source_id)
{
    const target_in in[1]{{1234, 0}};
    const auto r{pk::reduce(make_sample(1, 1, in, 1))};
    uint8_t out[8]{};
    pk::reason why{pk::reason::none};

    zassert_false(pk::encode_measurement(r, ctr::kSourceCount, 1, 0, out, &why));
    zassert_equal(static_cast<int>(why), static_cast<int>(pk::reason::source_id_out_of_range));
}

/* ---------------------------------------- health nibbles are not truncated ------ */

ZTEST(tof_cliff_packer, test_health_refuses_a_field_wider_than_its_nibble)
{
    /* Masking would produce a different *legal* frame: an enumerated mask of 0x1F
     * becoming 0x0F reports three sensors enumerated as four. */
    pk::health_fields good{};
    good.mapping_state = 0x1;
    good.flags = ctr::kCycleValidBit;
    good.enumerated_mask = 0xF;
    good.model_verified_mask = 0xF;
    good.sample_produced_mask = 0xF;
    good.failing_chain_position = ctr::kChainPositionNone;

    uint8_t out[8]{};
    zassert_true(pk::encode_health(good, out));

    const uint8_t canary{0x9C};
    struct { const char *name; uint8_t pk::health_fields::*field; } wide[]{
        {"flags", &pk::health_fields::flags},
        {"enumerated", &pk::health_fields::enumerated_mask},
        {"model_verified", &pk::health_fields::model_verified_mask},
        {"produced", &pk::health_fields::sample_produced_mask},
        {"fault", &pk::health_fields::sensor_fault_mask},
    };
    for (const auto &w : wide) {
        auto h{good};
        h.*(w.field) = 0x1F;
        uint8_t buf[8];
        memset(buf, canary, sizeof buf);
        zassert_false(pk::encode_health(h, buf), "a five-bit value must be refused");
        for (size_t i = 0; i < sizeof buf; ++i)
            zassert_equal(buf[i], canary, "refused health must not write the buffer");
    }
}
