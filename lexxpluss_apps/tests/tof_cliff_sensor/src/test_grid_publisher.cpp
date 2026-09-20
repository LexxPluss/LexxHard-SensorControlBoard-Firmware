/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The grid transport, against the contract's own vectors.
 *
 * WHAT THIS SUITE IS FOR. The packer suite already proves that a given admitted grid becomes
 * given bytes. Nothing there says where the fields of that grid come from, and that is the half
 * where a grid can be published under the wrong sensor: the source id, the generation, the
 * recovered flags and the transmit obligation's three legs are all decided HERE, from facts the
 * scheduler and the mapping hand over. So the assertions below are about provenance as much as
 * bytes -- every field is traced to the thing entitled to state it, and the refusals are checked
 * as strictly as the successes, because a publisher that sends nothing is safe and a publisher
 * that sends the wrong position's grid is not.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_acquisition.hpp"
#include "tof_can_ids.hpp"
#include "tof_contract_vectors.h"
#include "tof_grid_publisher.hpp"
#include "tof_l7_sample.hpp"
#include "tof_l7_status.hpp"

namespace acq = lexxhard::tof_acq;
namespace pub = lexxhard::tof_grid_pub;
namespace ids = lexxhard::tof_can_ids;
namespace l7 = lexxhard::tof_l7;

namespace {

constexpr int kSources{2};
constexpr uint8_t kEpoch{5};
/* The vectors were generated with six boards on the chain, and the health frame carries that in
 * byte 4's high nibble. The fake authorisation reports the same, so the comparison stays
 * byte-for-byte rather than byte-for-byte-except-one. */
constexpr uint8_t kBoards{6};
constexpr int kFramesPerGrid{17};

struct sent_frame {
    uint16_t can_id;
    uint8_t dlc;
    uint8_t data[8];
};

struct {
    sent_frame frames[64];
    int count;
    /* 1-based index of the first send that fails; 0 means never. */
    int fail_from;
    int send_calls;
} bus;

int fake_send(uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
    ++bus.send_calls;
    if (bus.fail_from > 0 && bus.send_calls >= bus.fail_from)
        return -EIO;
    if (bus.count < static_cast<int>(sizeof bus.frames / sizeof bus.frames[0])) {
        sent_frame &f{bus.frames[bus.count++]};
        f.can_id = can_id;
        f.dlc = dlc;
        memcpy(f.data, data, dlc <= 8 ? dlc : 8);
    }
    return 0;
}

acq::mapping_state state_value{acq::mapping_state::proven};
uint8_t epoch_value{kEpoch};
uint8_t boards_value{kBoards};
bool chain_length_flag{false};
bool other_enum_flag{false};
int authorise_calls{0};

struct pub::authorisation authorise()
{
    ++authorise_calls;
    struct pub::authorisation a{};
    a.state = state_value;
    a.epoch = epoch_value;
    a.boards_detected = boards_value;
    a.chain_length_unexpected = chain_length_flag;
    a.other_position_enumeration_failed = other_enum_flag;
    return a;
}

acq::source_desc descs[kSources];

void build_descs(uint8_t role0 = 0, uint8_t role1 = 1)
{
    for (int i = 0; i < kSources; ++i) {
        descs[i] = acq::source_desc{};
        descs[i].kind = acq::model::l7_grid;
        descs[i].addr_7bit = static_cast<uint8_t>(0x2A + i);
        /* Provisional and explicit, exactly as the descriptor requires: the sustainable grid
         * rate is unmeasured, and zero is refused by the scheduler rather than defaulted. */
        descs[i].grid_frequency_hz = 5;
    }
    descs[0].role_id = role0;
    descs[1].role_id = role1;
}

struct pub::config make_config(bool accept_low_confidence = false)
{
    struct pub::config c{};
    c.sink.send = fake_send;
    c.sources = descs;
    c.source_count = kSources;
    c.authorise = authorise;
    c.accept_low_confidence = accept_low_confidence;
    return c;
}

acq::source_facts make_facts(int index, uint8_t role, bool produced)
{
    acq::source_facts f{};
    f.kind = acq::model::l7_grid;
    f.addr_7bit = descs[index].addr_7bit;
    f.role_id = role;
    f.configured = true;
    f.started = true;
    f.sample_produced = produced;
    f.status.domain = acq::status_domain::l7;
    f.status.stage = static_cast<uint8_t>(l7::stage::none);
    return f;
}

/* The wire domain back into the sensor domain: a vector zone of 0xFFF is a zone the firmware
 * judged untrustworthy, so the sample carries a target_status the packer will not trust. Driving
 * the publisher with raw ULD values rather than wire values is the point -- the reduction is part
 * of what these frames prove. */
l7::sample make_sample(const uint16_t *wire_zones)
{
    l7::sample s{};
    s.fresh = true;
    for (size_t z = 0; z < l7::kZoneCount; ++z) {
        if (wire_zones[z] == tof_contract::kInvalidSentinel) {
            s.target_status[z] = 255;   // not 5, 6 or 9: untrustworthy by any policy
            s.target_count[z] = 0;
            s.distance_mm[z] = 0;
            continue;
        }
        s.target_status[z] = 5;
        s.target_count[z] = 1;
        s.distance_mm[z] = wire_zones[z];
    }
    return s;
}

uint16_t filler_zones[l7::kZoneCount];

void build_filler()
{
    for (size_t z = 0; z < l7::kZoneCount; ++z)
        filler_zones[z] = 1000;
}

acq::cycle_facts quiet_cycle(uint32_t seq)
{
    acq::cycle_facts cf{};
    cf.cycle_seq = seq;
    cf.source_count = kSources;
    for (int i = 0; i < kSources; ++i)
        cf.sources[i] = make_facts(i, descs[i].role_id, false);
    return cf;
}

void cycle_with_one(uint32_t seq, int index, uint8_t role, const uint16_t *zones)
{
    acq::cycle_facts cf{quiet_cycle(seq)};
    const acq::source_facts f{make_facts(index, role, true)};

    cf.sources[index] = f;
    pub::on_cycle_begin(seq);
    pub::on_grid_sample(index, seq, f, make_sample(zones));
    pub::on_cycle_complete(cf);
}

void cycle_with_both(uint32_t seq, const uint16_t *zones0, const uint16_t *zones1)
{
    acq::cycle_facts cf{quiet_cycle(seq)};
    const acq::source_facts f0{make_facts(0, descs[0].role_id, true)};
    const acq::source_facts f1{make_facts(1, descs[1].role_id, true)};

    cf.sources[0] = f0;
    cf.sources[1] = f1;
    pub::on_cycle_begin(seq);
    pub::on_grid_sample(0, seq, f0, make_sample(zones0));
    pub::on_grid_sample(1, seq, f1, make_sample(zones1));
    pub::on_cycle_complete(cf);
}

/* A cycle in which one source's read failed: no sample, and the outcome recorded the way the
 * scheduler records it. This is the only way the recovered flags can ever be set. */
void cycle_with_failure(uint32_t seq, int index, bool transport, l7::stage stage)
{
    acq::cycle_facts cf{quiet_cycle(seq)};
    acq::source_facts &f{cf.sources[index]};

    f.transport_error = transport;
    f.status.port_errno = transport ? -EIO : 0;
    f.status.uld_status = transport ? 0 : 255;
    f.status.stage = static_cast<uint8_t>(stage);
    pub::on_cycle_begin(seq);
    pub::on_cycle_complete(cf);
}

void advance_generations(int index, int grids)
{
    for (int i = 0; i < grids; ++i)
        cycle_with_one(static_cast<uint32_t>(100 + i), index, descs[index].role_id, filler_zones);
    bus = {};
}

void assert_vector_on_the_bus(const tof_contract::GridVector &v)
{
    zassert_equal(bus.count, kFramesPerGrid, "%s: %d frames", v.name, bus.count);
    for (int i = 0; i < 16; ++i) {
        zassert_equal(bus.frames[i].can_id, ids::TOF_GRID_DATA_ID, "%s frame %d id", v.name, i);
        zassert_equal(bus.frames[i].dlc, 8, "%s frame %d dlc", v.name, i);
        zassert_equal(memcmp(bus.frames[i].data, v.data_frames[i].bytes, 8), 0,
                      "%s data frame %d", v.name, i);
    }
    zassert_equal(bus.frames[16].can_id, ids::TOF_GRID_HEALTH_ID, "%s health id", v.name);
    zassert_equal(memcmp(bus.frames[16].data, v.health_frame.bytes, 8), 0, "%s health", v.name);
}

struct pub::counters take_counters()
{
    struct pub::counters c{};
    pub::copy_counters(c);
    return c;
}

void before(void *)
{
    bus = {};
    state_value = acq::mapping_state::proven;
    epoch_value = kEpoch;
    boards_value = kBoards;
    chain_length_flag = false;
    other_enum_flag = false;
    authorise_calls = 0;
    build_descs();
    build_filler();
    zassert_equal(pub::init(make_config()), 0);
}

} // namespace

ZTEST_SUITE(tof_grid_publisher, NULL, NULL, before, NULL, NULL);

/* ------------------------------------------------------------------ the bytes themselves --- */

ZTEST(tof_grid_publisher, test_a_proven_cycle_puts_the_contract_vector_on_the_bus)
{
    /* Vector 0 is source 0's first grid, so the generation the publisher allocates is the one the
     * vector was generated with. Nothing in the test sets it: if the publisher started
     * generations anywhere but zero, these bytes would not match. */
    const tof_contract::GridVector &v{tof_contract::kGridVectors[0]};

    zassert_equal(v.source_id, 0, "vector 0 is the right-hand sensor");
    zassert_equal(v.generation, 0, "and its first grid");
    cycle_with_one(1, 0, 0, v.zones_mm);
    assert_vector_on_the_bus(v);
    zassert_equal(bus.frames[16].data[2], v.expected_valid_zone_count, "valid zone count");

    const struct pub::counters c{take_counters()};
    zassert_equal(c.grids_admitted, 1);
    zassert_equal(c.grids_sent, 1);
    zassert_equal(c.data_frames_sent, 16);
    zassert_equal(c.health_frames_sent, 1);
    zassert_equal(c.generations_retired, 0);
}

ZTEST(tof_grid_publisher, test_the_left_sensor_reaches_its_own_vector)
{
    const tof_contract::GridVector &v{tof_contract::kGridVectors[1]};

    zassert_equal(v.source_id, 1, "vector 1 is the left-hand sensor");
    zassert_equal(v.generation, 7, "at its eighth grid");
    advance_generations(1, 7);
    cycle_with_one(1, 1, 1, v.zones_mm);
    assert_vector_on_the_bus(v);
    /* Every zone untrustworthy: a grid that carries no usable distance is still a grid, and
     * withholding it would look exactly like a sensor that is not there. */
    zassert_equal(v.expected_valid_zone_count, 0);
}

ZTEST(tof_grid_publisher, test_the_boundary_vector_matches_at_its_generation)
{
    const tof_contract::GridVector &v{tof_contract::kGridVectors[2]};

    zassert_equal(v.source_id, 0);
    zassert_equal(v.generation, 254);
    advance_generations(0, 254);
    cycle_with_one(1, 0, 0, v.zones_mm);
    assert_vector_on_the_bus(v);
}

ZTEST(tof_grid_publisher, test_the_last_generation_before_the_wrap_matches)
{
    const tof_contract::GridVector &v{tof_contract::kGridVectors[3]};

    zassert_equal(v.source_id, 1);
    zassert_equal(v.generation, 255);
    advance_generations(1, 255);
    cycle_with_one(1, 1, 1, v.zones_mm);
    assert_vector_on_the_bus(v);
}

ZTEST(tof_grid_publisher, test_the_generation_wraps_to_zero_after_255)
{
    advance_generations(0, 255);   // 0..254 spent
    cycle_with_one(1, 0, 0, filler_zones);
    zassert_equal(bus.frames[0].data[0], 255, "the 256th grid");
    bus = {};
    cycle_with_one(2, 0, 0, filler_zones);
    zassert_equal(bus.frames[0].data[0], 0, "and the one after it wraps");
    zassert_equal(bus.frames[16].data[0], 0, "health carries the same generation");
}

ZTEST(tof_grid_publisher, test_the_two_sources_count_generations_independently)
{
    cycle_with_one(1, 0, 0, filler_zones);
    cycle_with_one(2, 0, 0, filler_zones);
    bus = {};
    cycle_with_both(3, filler_zones, filler_zones);

    zassert_equal(bus.count, 2 * kFramesPerGrid);
    zassert_equal(bus.frames[0].data[0], 2, "source 0 is on its third grid");
    zassert_equal(bus.frames[kFramesPerGrid].data[0], 0, "source 1 is on its first");
    /* The contract's phrasing, and the reason they are separate counters: "the two sources'
     * counters are independent and unsynchronised". */
    zassert_equal(bus.frames[0].data[1] >> 4, 0);
    zassert_equal(bus.frames[kFramesPerGrid].data[1] >> 4, 1);
}

ZTEST(tof_grid_publisher, test_both_grids_of_one_cycle_fit_the_queue)
{
    /* 34 frames, which is the whole point of the queue's size: two sources reporting in the same
     * cycle is the normal case, and a queue that fits one grid would drop the second. */
    cycle_with_both(1, filler_zones, filler_zones);

    const struct pub::counters c{take_counters()};
    zassert_equal(bus.count, 34);
    zassert_equal(c.queue_full, 0);
    zassert_equal(c.grids_sent, 2);
    zassert_equal(c.data_frames_sent, 32);
    zassert_equal(c.health_frames_sent, 2);
}

/* ------------------------------------------------------------------------- the refusals ----- */

ZTEST(tof_grid_publisher, test_nothing_is_published_while_the_mapping_is_not_proven)
{
    state_value = acq::mapping_state::not_ready;
    cycle_with_one(1, 0, 0, filler_zones);

    const struct pub::counters c{take_counters()};
    zassert_equal(bus.count, 0, "an unproven chain publishes no grid");
    zassert_equal(c.suppressed_not_proven, 1);
    /* NOT a structural failure: an unproven cycle is the ordinary state of a chain that has not
     * been commissioned, and counting it as a defect would bury the real ones. */
    zassert_equal(c.cycles_invalid, 0);
    zassert_equal(c.grids_admitted, 0);
}

ZTEST(tof_grid_publisher, test_a_lost_mapping_publishes_no_grid)
{
    state_value = acq::mapping_state::lost;
    cycle_with_one(1, 0, 0, filler_zones);
    zassert_equal(bus.count, 0);
    zassert_equal(take_counters().suppressed_not_proven, 1);
}

ZTEST(tof_grid_publisher, test_a_position_the_mapping_never_keyed_publishes_nothing)
{
    /* 0xFF is what a descriptor carries until install_from_mapping keys it. Under PROVEN that is
     * a defect rather than a state: the grid has no physical position to be published under, and
     * the descriptor index is NOT an acceptable substitute -- that is exactly the guess the
     * contract's source_id promise forbids. */
    build_descs(0xFF, 1);
    zassert_equal(pub::init(make_config()), 0);
    cycle_with_one(1, 0, 0xFF, filler_zones);

    const struct pub::counters c{take_counters()};
    zassert_equal(bus.count, 0);
    zassert_equal(c.suppressed_role_unassigned, 1);
    zassert_equal(c.cycles_invalid, 1);
}

ZTEST(tof_grid_publisher, test_two_positions_claiming_one_source_id_lose_the_whole_cycle)
{
    build_descs(0, 0);
    zassert_equal(pub::init(make_config()), 0);
    cycle_with_both(1, filler_zones, filler_zones);

    const struct pub::counters c{take_counters()};
    zassert_equal(bus.count, 0, "not even the first of the two");
    zassert_equal(c.suppressed_duplicate_source, 1);
    zassert_equal(c.cycles_invalid, 1);
    zassert_equal(c.generations_retired, 1, "the one that was packed before the clash");
}

ZTEST(tof_grid_publisher, test_a_read_that_failed_is_never_admitted)
{
    acq::cycle_facts cf{quiet_cycle(1)};
    acq::source_facts f{make_facts(0, 0, true)};

    /* A fresh sample whose read reported a transport error is a contradiction the adapter should
     * not produce. It is passed through as observed rather than softened, so the packer's gate is
     * what refuses it -- one rule, in one place, for both publishers. */
    f.transport_error = true;
    f.status.port_errno = -EIO;
    cf.sources[0] = f;
    pub::on_cycle_begin(1);
    pub::on_grid_sample(0, 1, f, make_sample(filler_zones));
    pub::on_cycle_complete(cf);

    const struct pub::counters c{take_counters()};
    zassert_equal(bus.count, 0);
    zassert_equal(c.suppressed_not_admitted, 1);
    zassert_equal(c.cycles_invalid, 1);
}

ZTEST(tof_grid_publisher, test_a_cliff_sample_cannot_enter_the_grid_path)
{
    acq::cycle_facts cf{quiet_cycle(1)};
    acq::source_facts f{make_facts(0, 0, true)};

    f.kind = acq::model::l4_cliff;
    cf.sources[0] = f;
    pub::on_cycle_begin(1);
    pub::on_grid_sample(0, 1, f, make_sample(filler_zones));
    pub::on_cycle_complete(cf);

    zassert_equal(bus.count, 0);
    zassert_equal(take_counters().suppressed_wrong_model, 1);
}

ZTEST(tof_grid_publisher, test_a_status_written_in_the_wrong_vocabulary_is_refused)
{
    acq::cycle_facts cf{quiet_cycle(1)};
    acq::source_facts f{make_facts(0, 0, true)};

    /* The stage numbers of the two drivers are unrelated enumerations that both start at zero.
     * An l4-domain status on a grid sample would be reported in the health frame as an L7 stage:
     * a plausible wrong answer rather than an obvious one. */
    f.status.domain = acq::status_domain::l4;
    cf.sources[0] = f;
    pub::on_cycle_begin(1);
    pub::on_grid_sample(0, 1, f, make_sample(filler_zones));
    pub::on_cycle_complete(cf);

    zassert_equal(bus.count, 0);
    zassert_equal(take_counters().suppressed_wrong_domain, 1);
}

ZTEST(tof_grid_publisher, test_facts_that_disagree_with_the_descriptor_are_refused)
{
    acq::cycle_facts cf{quiet_cycle(1)};
    acq::source_facts f{make_facts(0, 1, true)};   // descriptor 0 is keyed to source 0

    cf.sources[0] = f;
    pub::on_cycle_begin(1);
    pub::on_grid_sample(0, 1, f, make_sample(filler_zones));
    pub::on_cycle_complete(cf);

    const struct pub::counters c{take_counters()};
    zassert_equal(bus.count, 0);
    zassert_equal(c.suppressed_role_mismatch, 1);
    zassert_equal(c.cycles_invalid, 1);
}

ZTEST(tof_grid_publisher, test_a_sample_for_an_unannounced_cycle_is_refused)
{
    const acq::source_facts f{make_facts(0, 0, true)};

    pub::on_grid_sample(0, 9, f, make_sample(filler_zones));
    zassert_equal(bus.count, 0);
    zassert_equal(take_counters().suppressed_cycle_not_begun, 1);
}

ZTEST(tof_grid_publisher, test_a_sample_the_facts_deny_is_refused)
{
    acq::cycle_facts cf{quiet_cycle(1)};
    const acq::source_facts f{make_facts(0, 0, false)};   // sample_produced clear

    cf.sources[0] = f;
    pub::on_cycle_begin(1);
    pub::on_grid_sample(0, 1, f, make_sample(filler_zones));
    pub::on_cycle_complete(cf);

    zassert_equal(bus.count, 0);
    zassert_equal(take_counters().suppressed_sample_contradiction, 1);
}

/* ----------------------------------------------------- authorisation between pack and send -- */

ZTEST(tof_grid_publisher, test_a_mapping_revoked_before_the_flush_discards_the_whole_cycle)
{
    acq::cycle_facts cf{quiet_cycle(1)};
    const acq::source_facts f{make_facts(0, 0, true)};

    cf.sources[0] = f;
    pub::on_cycle_begin(1);
    pub::on_grid_sample(0, 1, f, make_sample(filler_zones));
    /* Between the chain lock being dropped and the frames being offered to the bus. The grid was
     * packed under a mapping that no longer holds, so all seventeen frames go. */
    state_value = acq::mapping_state::lost;
    pub::on_cycle_complete(cf);

    const struct pub::counters c{take_counters()};
    zassert_equal(bus.count, 0);
    zassert_equal(c.cycles_discarded_unauthorised, 1);
    zassert_equal(c.generations_retired, 1);
}

ZTEST(tof_grid_publisher, test_a_moved_epoch_discards_the_whole_cycle)
{
    acq::cycle_facts cf{quiet_cycle(1)};
    const acq::source_facts f{make_facts(0, 0, true)};

    cf.sources[0] = f;
    pub::on_cycle_begin(1);
    pub::on_grid_sample(0, 1, f, make_sample(filler_zones));
    epoch_value = kEpoch + 1;   // re-enumerated in between: this grid belongs to the old mapping
    pub::on_cycle_complete(cf);

    const struct pub::counters c{take_counters()};
    zassert_equal(bus.count, 0);
    zassert_equal(c.cycles_discarded_unauthorised, 1);
}

/* ------------------------------------------------------------------------ transport failure -- */

ZTEST(tof_grid_publisher, test_a_failed_data_frame_abandons_the_rest_of_its_grid)
{
    bus.fail_from = 5;   // the fifth send onwards
    cycle_with_one(1, 0, 0, filler_zones);

    const struct pub::counters c{take_counters()};
    zassert_equal(bus.count, 4, "four frames reached the bus");
    zassert_equal(c.send_failed_data, 1);
    /* The remaining eleven data frames and the health frame are not even offered: they describe a
     * generation the consumer can now only discard, and the health frame would assert 64 zones
     * that never arrived. */
    zassert_equal(c.frames_abandoned, 12);
    zassert_equal(c.health_frames_sent, 0);
    zassert_equal(c.grids_sent, 0);
    zassert_equal(c.generations_retired, 1);
}

ZTEST(tof_grid_publisher, test_a_failed_health_frame_leaves_the_grid_unclosed)
{
    bus.fail_from = 17;   // the closing frame alone
    cycle_with_one(1, 0, 0, filler_zones);

    const struct pub::counters c{take_counters()};
    zassert_equal(bus.count, 16, "the data frames are on the wire");
    zassert_equal(c.send_failed_health, 1);
    zassert_equal(c.grids_sent, 0, "a grid nobody closed is not a published grid");
    zassert_equal(c.generations_retired, 1);
}

ZTEST(tof_grid_publisher, test_a_retired_generation_is_not_retried)
{
    bus.fail_from = 5;
    cycle_with_one(1, 0, 0, filler_zones);
    zassert_equal(bus.frames[0].data[0], 0, "generation 0 was spent on the failed grid");

    bus = {};
    cycle_with_one(2, 0, 0, filler_zones);
    /* THE POINT: if the failed grid were retried under its own number, a consumer holding four
     * frames of the first attempt would complete them with twelve frames of the second and publish
     * a grid that never existed -- assembled from two different reads, and structurally
     * indistinguishable from a good one. The counter still wraps at 255; that is the test next to
     * this one, and it is a different property. */
    zassert_equal(bus.frames[0].data[0], 1, "the next grid takes the next number");
    zassert_equal(bus.count, kFramesPerGrid);
}

ZTEST(tof_grid_publisher, test_one_sources_failure_does_not_abandon_the_other)
{
    bus.fail_from = 5;
    cycle_with_both(1, filler_zones, filler_zones);

    const struct pub::counters c{take_counters()};
    /* Four frames of source 0, then all seventeen of source 1: the grids are independent units,
     * and one bus failure is not a reason to withhold a grid that has nothing to do with it. */
    zassert_equal(c.grids_sent, 0, "the fake fails from the fifth send onwards, for both");
    zassert_equal(c.send_failed_data, 2, "each source stops at its own first failure");
    zassert_true(c.frames_abandoned > 0);
}

/* -------------------------------------------------------------------- the recovered flags ---- */

ZTEST(tof_grid_publisher, test_a_recovered_transfer_error_is_reported_on_the_next_grid)
{
    cycle_with_failure(1, 0, true, l7::stage::fetch);
    zassert_equal(bus.count, 0, "a failed read publishes nothing at all");

    cycle_with_one(2, 0, 0, filler_zones);
    zassert_equal(bus.count, kFramesPerGrid);
    /* Bit 0, and it is true by construction rather than by assertion: the grid carrying it exists
     * only because a later read succeeded, which is what "occurred and recovered" means. */
    zassert_equal(bus.frames[16].data[3] & 0x01, 0x01, "the I2C flag");
    zassert_equal(bus.frames[16].data[5], static_cast<uint8_t>(l7::stage::fetch), "last error");

    bus = {};
    cycle_with_one(3, 0, 0, filler_zones);
    zassert_equal(bus.frames[16].data[3], 0x00, "reported once, then cleared");
    zassert_equal(bus.frames[16].data[5], 0x00);
}

ZTEST(tof_grid_publisher, test_a_recovered_readiness_timeout_sets_its_own_flag)
{
    cycle_with_failure(1, 1, false, l7::stage::ready_check);
    cycle_with_one(2, 1, 1, filler_zones);

    zassert_equal(bus.frames[16].data[3] & 0x02, 0x02, "the data-ready timeout flag");
    zassert_equal(bus.frames[16].data[3] & 0x01, 0x00, "and not the transfer flag");
}

ZTEST(tof_grid_publisher, test_a_flag_survives_a_health_frame_that_did_not_reach_the_bus)
{
    cycle_with_failure(1, 0, true, l7::stage::fetch);
    bus.fail_from = 17;
    cycle_with_one(2, 0, 0, filler_zones);
    zassert_equal(take_counters().send_failed_health, 1);

    bus = {};
    cycle_with_one(3, 0, 0, filler_zones);
    /* Cleared by the bus, not by the packer. A flag dropped when the frame was BUILT would be
     * lost exactly when the transport is failing -- which is when it was worth reporting. */
    zassert_equal(bus.frames[16].data[3] & 0x01, 0x01, "still owed");
}

ZTEST(tof_grid_publisher, test_a_failure_on_one_source_is_not_reported_by_the_other)
{
    cycle_with_failure(1, 0, true, l7::stage::fetch);
    cycle_with_one(2, 1, 1, filler_zones);
    zassert_equal(bus.frames[16].data[3], 0x00, "source 1 had nothing to recover from");

    bus = {};
    cycle_with_one(3, 0, 0, filler_zones);
    zassert_equal(bus.frames[16].data[3] & 0x01, 0x01, "source 0 still reports its own");
}

ZTEST(tof_grid_publisher, test_the_chain_level_flags_are_restated_every_cycle)
{
    chain_length_flag = true;
    other_enum_flag = true;
    cycle_with_one(1, 0, 0, filler_zones);
    zassert_equal(bus.frames[16].data[3] & 0x0C, 0x0C, "bits 2 and 3 come from the mapping");

    bus = {};
    chain_length_flag = false;
    other_enum_flag = false;
    cycle_with_one(2, 0, 0, filler_zones);
    /* Not accumulated and not cleared by having been reported: they are a statement about the
     * chain now, so the moment the chain agrees again they stop being sent. */
    zassert_equal(bus.frames[16].data[3] & 0x0C, 0x00);
}

ZTEST(tof_grid_publisher, test_the_authorisation_is_read_once_per_cycle_and_once_at_the_flush)
{
    authorise_calls = 0;
    cycle_with_both(1, filler_zones, filler_zones);
    /* Once at on_cycle_begin and once at the flush -- never per sample, or the two grids of one
     * cycle could carry different epochs and the consumer correlates on exactly that. */
    zassert_equal(authorise_calls, 2);
}

ZTEST(tof_grid_publisher, test_init_refuses_an_incomplete_configuration)
{
    struct pub::config c{make_config()};

    c.sink.send = nullptr;
    zassert_equal(pub::init(c), -EINVAL);
    c = make_config();
    c.authorise = nullptr;
    zassert_equal(pub::init(c), -EINVAL);
    c = make_config();
    c.sources = nullptr;
    zassert_equal(pub::init(c), -EINVAL);
    c = make_config();
    c.source_count = 0;
    zassert_equal(pub::init(c), -EINVAL);
    c = make_config();
    c.source_count = acq::kMaxSources + 1;
    zassert_equal(pub::init(c), -EINVAL);
}
