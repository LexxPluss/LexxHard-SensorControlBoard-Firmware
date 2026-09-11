/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * acquisition -> packer -> a fake CAN sink.
 *
 * The CAN sink is a function pointer, so this exercises the whole publish path with no
 * Zephyr CAN driver anywhere near it -- the real glue is a separate commit, which is also
 * what lets its flash cost be measured on its own.
 *
 * THE ONE THING TO BE CAREFUL ABOUT HERE
 *
 * tof_acq::publication_allowed() is structurally false: effective_mapping_state() clamps
 * PROVEN unconditionally and there is no flag to lift it. So the measurement path can only
 * be reached by injecting a gate, and this file does that -- in the TEST, through the
 * publisher's config, exactly as the acquisition layer's own mapping_state_provider is
 * injected. Production wires the gate to the real function.
 *
 * What is deliberately NOT done: nothing in production gained a build flag, a constant, a
 * command or a configuration value that opens the gate. TOF_ACQ_CHAIN_HW_FIXED was deleted
 * for being exactly that. test_the_production_gate_is_still_shut below asserts the real
 * gate stays closed, so lifting the clamp without review fails here rather than shipping.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_acquisition.hpp"
#include "tof_cliff_contract_vectors.h"
#include "tof_cliff_publisher.hpp"

namespace acq = lexxhard::tof_acq;
namespace pub = lexxhard::tof_cliff_pub;
namespace ctr = tof_cliff_contract;

namespace {

constexpr int kCliffSources{4};
constexpr uint8_t kEpoch{7};

struct sent_frame {
    uint16_t can_id;
    uint8_t dlc;
    uint8_t data[8];
};

struct {
    sent_frame frames[32];
    int count;
    int fail_after;   // -1 never fails; otherwise fail from this send onwards
    int send_calls;
    bool block_measurements;  // hold a measurement send inside the sink until released
    int measurements_inside;
} bus;

/* Kept outside `bus` so `bus = {}` in before() cannot clobber an initialised object. */
K_SEM_DEFINE(sink_entered, 0, 8);
K_SEM_DEFINE(sink_release, 0, 8);

int fake_send(uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
    ++bus.send_calls;
    /* The concurrency test parks a measurement send in here. Health is never blocked, which
     * is what lets the test show that a stalled measurement does not hold the heartbeat. */
    if (bus.block_measurements && can_id == ctr::kMeasId) {
        ++bus.measurements_inside;
        k_sem_give(&sink_entered);
        k_sem_take(&sink_release, K_FOREVER);
        --bus.measurements_inside;
    }
    if (bus.fail_after >= 0 && bus.send_calls > bus.fail_after)
        return -EIO;
    if (bus.count < static_cast<int>(sizeof bus.frames / sizeof bus.frames[0])) {
        sent_frame &f{bus.frames[bus.count++]};
        f.can_id = can_id;
        f.dlc = dlc;
        memcpy(f.data, data, dlc <= 8 ? dlc : 8);
    }
    return 0;
}

int count_id(uint16_t id)
{
    int n{0};
    for (int i = 0; i < bus.count; ++i)
        if (bus.frames[i].can_id == id)
            ++n;
    return n;
}

/* The n-th frame carrying this identifier. Positional indexing into bus.frames stopped being
 * usable once a completed cycle also emits a health frame: [meas, health, meas, health, ...]. A
 * test that means "the second measurement" has to say so. */
const sent_frame *nth_of(uint16_t id, int n)
{
    int seen{0};
    for (int i = 0; i < bus.count; ++i) {
        if (bus.frames[i].can_id != id)
            continue;
        if (seen++ == n)
            return &bus.frames[i];
    }
    return nullptr;
}

const sent_frame *last_of(uint16_t id)
{
    for (int i = bus.count - 1; i >= 0; --i)
        if (bus.frames[i].can_id == id)
            return &bus.frames[i];
    return nullptr;
}

const ctr::vector *find_vector(const char *name)
{
    for (size_t i = 0; i < ctr::kVectorCount; ++i)
        if (strcmp(ctr::kVectors[i].name, name) == 0)
            return &ctr::kVectors[i];
    return nullptr;
}

/* The injected authorisation. Closed by default, so a test that forgets to open it sees
 * production behaviour rather than an accidental publish. Gate and epoch are one value, so
 * a test cannot accidentally exercise a combination production cannot produce. */
bool gate_open{false};
uint8_t epoch_value{kEpoch};
acq::mapping_state state_value{acq::mapping_state::not_ready};
uint8_t enumerated_value{0};
uint8_t model_verified_value{0};
uint8_t chain_flags_value{0};
uint8_t failing_position_value{ctr::kChainPositionNone};
int authorise_calls{0};
pub::authorisation test_authorise()
{
    ++authorise_calls;
    pub::authorisation a{};
    a.state = gate_open ? acq::mapping_state::proven : state_value;
    a.epoch = epoch_value;
    a.enumerated_mask = enumerated_value;
    a.model_verified_mask = model_verified_value;
    a.chain_flags = chain_flags_value;
    a.failing_position = failing_position_value;
    return a;
}

/* The publisher needs the descriptor table for kind and role_id. Same shape the
 * acquisition harness uses: four cliff sources then two grid stubs. */
acq::source_desc descs[acq::kMaxSources];
/* Per source, never shared: see source_desc's note on the replay history. */
struct tof_cliff_stream_state fake_stream[acq::kMaxSources];

void build_descs()
{
    for (int i = 0; i < acq::kMaxSources; ++i) {
        descs[i].kind = (i < kCliffSources) ? acq::model::l4_cliff : acq::model::l7_grid;
        descs[i].addr_7bit = static_cast<uint8_t>(0x32 + i);
        descs[i].role_id = static_cast<uint8_t>(i);
    }
}

pub::config make_pub_config()
{
    pub::config c{};
    c.sink.send = fake_send;
    c.sources = descs;
    c.source_count = acq::kMaxSources;
    c.authorise = test_authorise;
    return c;
}

struct tof_cliff_sample one_valid_target(int16_t mm)
{
    struct tof_cliff_sample s{};
    s.fresh = true;
    s.target_count = 1;
    s.entry_count = 1;
    s.entries[0].range_mm = mm;
    s.entries[0].range_status = 0; // RANGE_VALID
    return s;
}

acq::source_facts cliff_facts(uint8_t role)
{
    acq::source_facts f{};
    f.kind = acq::model::l4_cliff;
    f.role_id = role;
    f.configured = true;
    f.started = true;
    f.sample_produced = true;
    return f;
}

/* Nothing reaches the bus until the cycle ends, because on_cliff_sample runs under the
 * chain lock. Every measurement assertion therefore flushes first -- which is also the
 * property being tested. */
/* The flush is per cycle now: only frames belonging to the cycle being completed are
 * published. Taking the number as an argument is what keeps the tests honest about that. */
void flush(uint32_t cycle_seq)
{
    acq::cycle_facts f{};
    f.cycle_seq = cycle_seq;
    pub::on_cycle_complete(f);
}

/* What the acquisition layer does at the top of every cycle. Tests announce cycles the same way
 * production does -- the authorisation is latched here and nowhere else, so a test that skipped
 * it would be exercising a path production cannot take. */
void begin(uint32_t cycle_seq)
{
    pub::on_cycle_begin(cycle_seq);
}

void before(void *)
{
    bus = {};
    bus.fail_after = -1;
    k_sem_reset(&sink_entered);
    k_sem_reset(&sink_release);
    gate_open = false;
    epoch_value = kEpoch;
    state_value = acq::mapping_state::not_ready;
    enumerated_value = 0;
    model_verified_value = 0;
    chain_flags_value = 0;
    failing_position_value = ctr::kChainPositionNone;
    authorise_calls = 0;
    build_descs();
    zassert_equal(pub::init(make_pub_config()), 0);
}

} // namespace

/* The integration tests at the end of this file configure the acquisition layer, and a live one
 * refuses a second init(). Retiring after every test keeps them independent of their order. */
void retire_acquisition(void *)
{
    acq::teardown();
}

ZTEST_SUITE(tof_cliff_publisher, NULL, NULL, before, retire_acquisition, NULL);

/* ------------------------------------------------------------ the safety gate ----- */

ZTEST(tof_cliff_publisher, test_the_production_gate_is_still_shut)
{
    /* The point of this test is to fail the day someone lifts the clamp. PROVEN is not
     * something this firmware is entitled to claim while the enable chain cannot be
     * enumerated end to end, and the clamp is unconditional by design. */
    zassert_false(acq::publication_allowed(),
                  "tof_acq::publication_allowed() must be structurally false; if this "
                  "now passes, the clamp was lifted and that needs its own review");
    zassert_not_equal(static_cast<int>(acq::effective_mapping_state()),
                      static_cast<int>(acq::mapping_state::proven));
}

ZTEST(tof_cliff_publisher, test_no_measurement_is_sent_while_the_mapping_is_not_proven)
{
    /* UNKNOWN and FAULT are the two states this layer can be in. Neither may publish a
     * role-named range: outside PROVEN the source_id is an unproven guess rather than a
     * physical position. */
    gate_open = false;
    const auto s{one_valid_target(900)};
    begin(1);
    for (uint8_t role = 0; role < kCliffSources; ++role)
        pub::on_cliff_sample(role, 1, cliff_facts(role), s);
    flush(1);

    zassert_equal(count_id(ctr::kMeasId), 0, "published a range without a proven mapping");

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.suppressed_not_proven, kCliffSources);
    zassert_equal(c.measurements_sent, 0);
}

ZTEST(tof_cliff_publisher, test_health_flows_from_startup_and_through_bring_up_failure)
{
    /* Health is what tells the consumer the subsystem exists and is not ready. It must
     * keep going out when nothing came up at all -- that is the case a silent bus would
     * be indistinguishable from. */
    pub::on_cliff_health(0, acq::mapping_state::not_ready);
    state_value = acq::mapping_state::fault;
    pub::on_cliff_health(0, acq::mapping_state::not_ready);
    state_value = acq::mapping_state::not_ready;
    pub::on_cliff_health(0, acq::mapping_state::not_ready);

    zassert_equal(count_id(ctr::kHealthId), 3);
    /* The state on the wire comes from the authorisation, not from the sink's argument. */
    zassert_equal(static_cast<uint8_t>(bus.frames[1].data[3] >> 4), 0x3, "FAULT");
    zassert_equal(static_cast<uint8_t>(bus.frames[2].data[3] >> 4), 0x0, "UNKNOWN");
    zassert_equal(count_id(ctr::kMeasId), 0, "health must not drag a measurement with it");

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.health_sent, 3);
}

/* The consequence side of the authority fix (#103 review). The authority now guarantees that
 * note_chain_fault() cannot publish a snapshot the encoder refuses; these two assert what that
 * guarantee is worth HERE, where the refusal would actually be felt.
 *
 * First half: the dangerous combination really does silence the heartbeat at this layer, so the
 * guarantee is load-bearing rather than defensive. A named position, no chain-fault bit, complete
 * masks -- exactly what the pre-fix note_chain_fault(0x0, 3) produced after a PROVEN commit. */
ZTEST(tof_cliff_publisher, test_a_contradictory_authorisation_silences_the_heartbeat)
{
    state_value = acq::mapping_state::fault;
    enumerated_value = 0x0F;
    model_verified_value = 0x0F;
    chain_flags_value = 0;
    failing_position_value = 3;

    pub::on_cliff_health(0, acq::mapping_state::not_ready);

    zassert_equal(count_id(ctr::kHealthId), 0, "no frame reaches the bus");
    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.health_sent, 0);
    zassert_equal(c.health_encode_refused, 1,
                  "and the only trace is a counter with no production readout -- which is why the "
                  "authority must never produce this snapshot");
}

/* Second half: what note_chain_fault() now produces for those same arguments -- a generic fault,
 * masks untouched -- does reach the bus. */
ZTEST(tof_cliff_publisher, test_the_degraded_generic_fault_still_reaches_the_bus)
{
    state_value = acq::mapping_state::fault;
    enumerated_value = 0x0F;   // kept: the last enumeration result is still true
    model_verified_value = 0x0F;
    chain_flags_value = 0;
    failing_position_value = ctr::kChainPositionNone;

    pub::on_cliff_health(0, acq::mapping_state::not_ready);

    zassert_equal(count_id(ctr::kHealthId), 1);
    zassert_equal(static_cast<uint8_t>(bus.frames[0].data[3] >> 4), 0x3, "FAULT on the wire");
    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.health_sent, 1);
    zassert_equal(c.health_encode_refused, 0);
}

/* ------------------------------------------------- the wire mapping_state trap ---- */

ZTEST(tof_cliff_publisher, test_the_two_mapping_state_enumerations_are_translated)
{
    /* A straight cast would put PROVEN on the wire for tof_acq's `fault`, and LOST for its
     * `proven`. The one field the consumer gates on, mis-reported silently. */
    zassert_equal(pub::wire_mapping_state(acq::mapping_state::not_ready), 0x0); // UNKNOWN
    zassert_equal(pub::wire_mapping_state(acq::mapping_state::proven), 0x1);    // PROVEN
    zassert_equal(pub::wire_mapping_state(acq::mapping_state::fault), 0x3);     // FAULT
    /* LOST arrived with the mapping authority. Its own trap: tof_acq numbers it 3 and the
     * contract numbers FAULT 3, so a cast reports a fault where the mapping was merely lost,
     * and the consumer's recovery path is not the same in the two cases. */
    zassert_equal(pub::wire_mapping_state(acq::mapping_state::lost), 0x2);      // LOST
    /* And the values really do differ, which is why the switch exists. */
    zassert_not_equal(pub::wire_mapping_state(acq::mapping_state::fault),
                      static_cast<uint8_t>(acq::mapping_state::fault));
    zassert_not_equal(pub::wire_mapping_state(acq::mapping_state::lost),
                      static_cast<uint8_t>(acq::mapping_state::lost));
}

ZTEST(tof_cliff_publisher, test_a_lost_mapping_authorises_no_measurement)
{
    /* LOST is not a softer PROVEN. The contract forbids a measurement frame while UNKNOWN,
     * LOST or FAULT, and the reason is the same in all three: source_id outside PROVEN is
     * the firmware's guess rather than a physical position. */
    state_value = acq::mapping_state::lost;
    begin(0);
    pub::on_cliff_sample(0, 0, cliff_facts(0), one_valid_target(400));
    flush(0);
    zassert_equal(bus.count, 0, "a measurement went out under LOST");

    pub::counters c{};
    pub::copy_counters(c);
    zassert_true(c.suppressed_not_proven > 0, "the suppression must be counted, not silent");
}

ZTEST(tof_cliff_publisher, test_health_bytes_match_the_named_vector)
{
    /* The heartbeat this layer can honestly produce: UNKNOWN, describing no cycle, with
     * every mask empty. That is byte for byte the contract's own heartbeat vector, once
     * the epoch and sequence are matched. */
    const auto *v{find_vector("health_heartbeat_unknown")};
    zassert_not_null(v);

    /* The vector uses epoch 1 and health_seq 8. health_seq is the count BEFORE the
     * increment, so the ninth call is the one carrying 8 -- an off-by-one worth spelling
     * out, because eight calls looks right and produces seq 7. */
    epoch_value = 1;
    zassert_equal(pub::init(make_pub_config()), 0);
    for (int i = 0; i < 9; ++i)
        pub::on_cliff_health(0, acq::mapping_state::not_ready);

    const sent_frame *f{last_of(ctr::kHealthId)};
    zassert_not_null(f);
    zassert_equal(f->dlc, ctr::kDlc);
    zassert_equal(memcmp(f->data, v->bytes, 8), 0,
                  "the heartbeat must be byte-identical to the contract's vector");
}

/* --------------------------------------------------------- the measurement path --- */

ZTEST(tof_cliff_publisher, test_measurement_bytes_match_the_named_vector)
{
    gate_open = true;
    epoch_value = 1;
    zassert_equal(pub::init(make_pub_config()), 0);
    gate_open = true;

    /* The vector is source 0, epoch 1, cycle 0, range 1234, status 0, one target. */
    begin(0);
    pub::on_cliff_sample(0, 0, cliff_facts(0), one_valid_target(1234));
    flush(0);

    const auto *v{find_vector("meas_role_0_front_left")};
    zassert_not_null(v);
    const sent_frame *f{last_of(ctr::kMeasId)};
    zassert_not_null(f);
    zassert_equal(f->dlc, ctr::kDlc);
    zassert_equal(memcmp(f->data, v->bytes, 8), 0);
}

ZTEST(tof_cliff_publisher, test_the_cycle_the_sample_belongs_to_reaches_the_wire)
{
    /* The contract correlates a measurement with its health frame by cycle_seq, so a
     * hard-coded zero here would break the correlation silently. on_cycle fires after
     * every sample, which is why the sink carries the value rather than a sink latching it.
     */
    gate_open = true;
    /* One cycle at a time, flushed each time. Queueing three cycles and flushing once --
     * which an earlier version of this test did -- contradicts the rule that a frame never
     * outlives its cycle. */
    begin(0);
    pub::on_cliff_sample(0, 0, cliff_facts(0), one_valid_target(900));
    flush(0);
    begin(1);
    pub::on_cliff_sample(0, 1, cliff_facts(0), one_valid_target(900));
    flush(1);
    begin(258);
    pub::on_cliff_sample(0, 258, cliff_facts(0), one_valid_target(900));
    flush(258);

    zassert_equal(count_id(ctr::kMeasId), 3);
    /* By identifier, not by position: each completed cycle also emits its health frame now. */
    zassert_equal(nth_of(ctr::kMeasId, 0)->data[2], 0);
    zassert_equal(nth_of(ctr::kMeasId, 1)->data[2], 1);
    zassert_equal(nth_of(ctr::kMeasId, 2)->data[2], 2, "cycle_seq wraps 255 -> 0 on the wire");
    /* And each of those cycles produced exactly one cycle health frame carrying the same
     * number, which is the correlation the consumer actually uses. */
    zassert_equal(count_id(ctr::kHealthId), 3);
    zassert_equal(nth_of(ctr::kHealthId, 2)->data[7], 2);
}

ZTEST(tof_cliff_publisher, test_a_stale_sample_is_not_published)
{
    gate_open = true;
    auto s{one_valid_target(900)};
    s.fresh = false;

    begin(1);
    pub::on_cliff_sample(0, 1, cliff_facts(0), s);
    flush(1);

    zassert_equal(count_id(ctr::kMeasId), 0, "a stale sample must never reach the bus");
    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.suppressed_no_frame, 1);
    zassert_equal(c.measurements_sent, 0);
}

ZTEST(tof_cliff_publisher, test_a_packer_refusal_sends_nothing_at_all)
{
    gate_open = true;
    /* Zero targets with a status other than 255: impossible metadata, so the packer
     * refuses and there is no partial frame to send. */
    struct tof_cliff_sample s{};
    s.fresh = true;
    s.target_count = 0;
    s.entry_count = 1;
    s.entries[0].range_status = 5;

    begin(1);
    pub::on_cliff_sample(0, 1, cliff_facts(0), s);
    flush(1);

    zassert_equal(bus.send_calls, 0, "a refused frame must not be handed to the bus");
    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.suppressed_packer_refused, 1);
    zassert_not_equal(static_cast<int>(c.last_refusal),
                      static_cast<int>(lexxhard::tof_cliff_pub::counters{}.last_refusal));
}

ZTEST(tof_cliff_publisher, test_l7_stub_data_never_reaches_the_cliff_packer)
{
    /* The acquisition layer already gates the cliff sink on the model, so this is defence
     * in depth -- kept because the sink is a function pointer and a wiring mistake would
     * otherwise pack an 8x8 grid read as a floor distance. */
    gate_open = true;
    acq::source_facts grid{cliff_facts(4)};
    grid.kind = acq::model::l7_grid;

    begin(1);
    pub::on_cliff_sample(4, 1, grid, one_valid_target(900));
    flush(1);

    zassert_equal(bus.send_calls, 0);
    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.suppressed_wrong_model, 1);
    zassert_equal(c.measurements_sent, 0);
}

/* ------------------------------------------------ transport vs sensor failures ---- */

ZTEST(tof_cliff_publisher, test_a_bus_failure_is_counted_apart_from_a_sensor_failure)
{
    /* "The sensor said something we will not send" and "the bus would not take it" need
     * different follow-up, so they must not share a counter. */
    gate_open = true;
    bus.fail_after = 0; // every send fails

    begin(1);
    pub::on_cliff_sample(0, 1, cliff_facts(0), one_valid_target(900));
    flush(1);
    pub::on_cliff_health(0, acq::mapping_state::not_ready);

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.send_failed_measurement, 1);
    zassert_equal(c.send_failed_health, 1);
    zassert_equal(c.measurements_sent, 0);
    zassert_equal(c.health_sent, 0);
    /* And none of the suppression counters moved: the frame was correct. */
    zassert_equal(c.suppressed_not_proven, 0);
    zassert_equal(c.suppressed_no_frame, 0);
    zassert_equal(c.suppressed_packer_refused, 0);
    zassert_equal(c.suppressed_wrong_model, 0);
}

ZTEST(tof_cliff_publisher, test_health_keeps_flowing_after_a_measurement_bus_failure)
{
    /* A dead measurement path must not take health down with it: health is how the
     * consumer learns anything at all. */
    gate_open = true;
    bus.fail_after = 1; // the first send succeeds, everything after fails

    pub::on_cliff_health(0, acq::mapping_state::not_ready); // succeeds
    begin(1);
    pub::on_cliff_sample(0, 1, cliff_facts(0), one_valid_target(900));
    flush(1);                                               // this send fails
    pub::on_cliff_health(0, acq::mapping_state::not_ready); // fails too, but is attempted

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.health_sent, 1);
    zassert_equal(c.send_failed_measurement, 1);
    zassert_equal(c.send_failed_health, 1);
    zassert_equal(bus.send_calls, 3, "every frame must still be offered to the bus");
}

/* ------------------------------------------------------------------ config -------- */

ZTEST(tof_cliff_publisher, test_init_refuses_an_incomplete_configuration)
{
    pub::config c{make_pub_config()};

    auto missing = [](pub::config base, void (*wreck)(pub::config &)) {
        wreck(base);
        return pub::init(base);
    };
    zassert_equal(missing(c, [](pub::config &x) { x.sink.send = nullptr; }), -EINVAL);
    zassert_equal(missing(c, [](pub::config &x) { x.authorise = nullptr; }), -EINVAL);
    zassert_equal(missing(c, [](pub::config &x) { x.sources = nullptr; }), -EINVAL);
    zassert_equal(missing(c, [](pub::config &x) { x.source_count = 0; }), -EINVAL);
    zassert_equal(missing(c, [](pub::config &x) { x.source_count = acq::kMaxSources + 1; }),
                  -EINVAL);
}

/* ------------------------------------------------------- nothing under the lock --- */

ZTEST(tof_cliff_publisher, test_nothing_reaches_the_bus_before_the_cycle_ends)
{
    /* on_cliff_sample runs under the chain lock. A synchronous send there would let CAN
     * backpressure stall every sensor's next read, so the frame is queued and the bus is
     * only touched after the lock is released. */
    gate_open = true;
    begin(3);
    for (uint8_t role = 0; role < kCliffSources; ++role)
        pub::on_cliff_sample(role, 3, cliff_facts(role), one_valid_target(700));

    zassert_equal(bus.send_calls, 0, "the bus was touched while the chain lock was held");

    flush(3);
    zassert_equal(count_id(ctr::kMeasId), kCliffSources);
}

ZTEST(tof_cliff_publisher, test_a_failed_frame_is_never_carried_into_the_next_cycle)
{
    /* A range that failed to send is a stale range by the time the bus recovers, and the
     * contract forbids re-sending old values outright. So a frame is offered exactly once
     * and then dropped, whatever happened. */
    gate_open = true;
    bus.fail_after = 0; // every send fails
    begin(1);
    pub::on_cliff_sample(0, 1, cliff_facts(0), one_valid_target(900));
    flush(1);
    zassert_equal(bus.send_calls, 1);

    bus.fail_after = -1; // the bus recovers
    flush(2);            // the next cycle ends with nothing queued
    zassert_equal(bus.send_calls, 1, "a failed frame was retried on a later cycle");
    zassert_equal(count_id(ctr::kMeasId), 0);

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.send_failed_measurement, 1);
    zassert_equal(c.queue_full, 0);
}

/* ------------------------------------------------------- descriptor agreement ----- */

ZTEST(tof_cliff_publisher, test_facts_disagreeing_with_the_descriptor_are_refused)
{
    /* No sensor can cause this: it means the sink was handed an index and a set of facts
     * that do not belong together, and packing it would attach a range to a role that did
     * not produce it. */
    gate_open = true;
    acq::source_facts wrong{cliff_facts(0)};
    wrong.role_id = 2; // descriptor 0 carries role 0

    begin(1);
    pub::on_cliff_sample(0, 1, wrong, one_valid_target(900));
    flush(1);

    zassert_equal(bus.send_calls, 0);
    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.suppressed_role_mismatch, 1);
}

/* ------------------------------------------------ one authorisation per cycle ----- */

ZTEST(tof_cliff_publisher, test_every_frame_of_a_cycle_shares_one_authorisation)
{
    /* Reading per sensor would let four frames of one cycle carry different epochs, and the
     * consumer correlates a measurement with its health frame on exactly that. */
    gate_open = true;
    epoch_value = 4;
    authorise_calls = 0;

    begin(9);
    for (uint8_t role = 0; role < kCliffSources; ++role) {
        pub::on_cliff_sample(role, 9, cliff_facts(role), one_valid_target(600));
        epoch_value = static_cast<uint8_t>(epoch_value + 1); // moves under our feet
    }
    /* Put it back so the flush's re-check agrees with what was latched. */
    epoch_value = 4;
    flush(9);

    zassert_equal(count_id(ctr::kMeasId), kCliffSources);
    for (int i = 0; i < kCliffSources; ++i)
        zassert_equal(bus.frames[i].data[1], 4, "frame %d carried a different epoch", i);
    zassert_equal(authorise_calls, 2,
                  "one read to latch the cycle and one to re-check before sending");
}

ZTEST(tof_cliff_publisher, test_losing_the_mapping_before_the_flush_discards_the_cycle)
{
    /* Encoding happens under the lock, sending after it. If the mapping goes in between, an
     * already-encoded frame would otherwise be published under a mapping that no longer
     * holds. The whole cycle goes, not the offending frame: a partly published cycle is a
     * broken correlation the consumer cannot detect. */
    gate_open = true;
    begin(5);
    for (uint8_t role = 0; role < kCliffSources; ++role)
        pub::on_cliff_sample(role, 5, cliff_facts(role), one_valid_target(800));

    gate_open = false; // the mapping is lost between encoding and sending
    flush(5);

    zassert_equal(bus.send_calls, 0, "published under a mapping that no longer holds");
    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.cycles_discarded_unauthorised, 1);
    zassert_equal(c.measurements_sent, 0);
}

ZTEST(tof_cliff_publisher, test_a_moved_epoch_before_the_flush_discards_the_cycle)
{
    /* Still authorised, but under a different epoch -- so the frames describe a mapping
     * that has been re-proven since they were built. */
    gate_open = true;
    epoch_value = 2;
    begin(6);
    for (uint8_t role = 0; role < kCliffSources; ++role)
        pub::on_cliff_sample(role, 6, cliff_facts(role), one_valid_target(800));

    epoch_value = 3;
    flush(6);

    zassert_equal(bus.send_calls, 0);
    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.cycles_discarded_unauthorised, 1);
}

ZTEST(tof_cliff_publisher, test_a_frame_never_outlives_its_cycle)
{
    /* Two things are checked. A frame queued for one cycle is not published by the flush of
     * another; and a cycle that never got flushed is dropped when the next one starts,
     * rather than trailing along behind it. */
    gate_open = true;
    begin(10);
    pub::on_cliff_sample(0, 10, cliff_facts(0), one_valid_target(900));
    flush(11); // the wrong cycle completes

    zassert_equal(bus.send_calls, 0, "a frame was published by another cycle's flush");
    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.discarded_stale_cycle, 1);

    /* And an unflushed cycle does not survive into the next one. */
    begin(12);
    pub::on_cliff_sample(0, 12, cliff_facts(0), one_valid_target(900));
    begin(13);
    pub::on_cliff_sample(0, 13, cliff_facts(0), one_valid_target(900));
    flush(13);

    zassert_equal(count_id(ctr::kMeasId), 1, "only cycle 13's frame may be published");
    pub::copy_counters(c);
    zassert_equal(c.discarded_stale_cycle, 2);
}

/* ------------------------------------------------------------ real integration ---- */

namespace integration {

int fake_dev[acq::kMaxSources];
int fake_scratch[acq::kMaxSources];
int16_t canned_mm{1234};
bool start_fails{false};

int op_open(void *, uint8_t, acq::op_status *) { return 0; }
int op_configure(void *, acq::op_status *) { return 0; }
int op_start(void *, void *, acq::op_status *st)
{
    if (start_fails) {
        st->stage = TOF_CLIFF_STAGE_START;
        return -EIO;
    }
    return 0;
}
int op_stop(void *, acq::op_status *) { return 0; }
int op_read(void *, void *, void *, struct tof_cliff_sample *out, acq::op_status *)
{
    *out = tof_cliff_sample{};
    out->fresh = true;
    out->target_count = 1;
    out->entry_count = 1;
    out->entries[0].range_mm = canned_mm;
    out->entries[0].range_status = 0;
    return 0;
}

const acq::source_ops kOps{op_open, op_configure, op_start, op_read, op_stop};

acq::mapping_state provider() { return acq::mapping_state::proven; }
uint32_t clock_ms() { return 0; }

acq::config make_acq_config()
{
    acq::config c{};
    for (int i = 0; i < acq::kMaxSources; ++i) {
        descs[i].dev = &fake_dev[i];
        descs[i].scratch = &fake_scratch[i];
        descs[i].stream = &fake_stream[i];
        descs[i].ops = &kOps;
    }
    c.sources = descs;
    c.source_count = acq::kMaxSources;
    c.periods.cycle_period_ms = 50;
    c.periods.health_period_ms = 100;
    /* The real wiring: the acquisition layer's sinks are the publisher's entry points, and
     * nothing sits in between to transform anything. */
    c.hooks.on_cycle_begin = pub::on_cycle_begin;
    c.hooks.on_cycle = pub::on_cycle_complete;
    c.hooks.on_cliff_sample = pub::on_cliff_sample;
    c.hooks.on_cliff_health = pub::on_cliff_health;
    c.mapping_state_provider = provider;
    c.now_ms = clock_ms;
    return c;
}

} // namespace integration

ZTEST(tof_cliff_publisher, test_a_real_cycle_reaches_the_bus_through_the_packer)
{
    /* The whole path, not an interface splice: acq::init with the publisher's own functions
     * as the sinks, bring_up over fake device ops, then a real run_cycle that reads every
     * source, calls the sink under the lock, and flushes on the way out. */
    integration::start_fails = false;
    integration::canned_mm = 1234;
    build_descs();
    zassert_equal(acq::init(integration::make_acq_config()), 0);
    epoch_value = 1;
    zassert_equal(pub::init(make_pub_config()), 0);
    gate_open = true;

    zassert_equal(acq::bring_up(), 0);
    acq::run_cycle();

    /* Four cliff sources produced a sample; the two grid stubs are gated out by the
     * acquisition layer itself, before the publisher ever sees them. */
    zassert_equal(count_id(ctr::kMeasId), kCliffSources,
                  "the real cycle did not deliver one frame per cliff source");

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.measurements_sent, kCliffSources);
    zassert_equal(c.suppressed_wrong_model, 0, "the grid stubs must not reach this layer");
    zassert_equal(c.suppressed_role_mismatch, 0);

    /* Byte-exact against the contract's vector, cycle byte included: the vector is cycle 0
     * and so is the first cycle of an epoch. An earlier version of this test asserted 1
     * here, with a comment explaining it -- pinning a contract violation as expected
     * behaviour, which is worse than the violation on its own. */
    const auto *v{find_vector("meas_role_0_front_left")};
    zassert_not_null(v);
    zassert_equal(memcmp(bus.frames[0].data, v->bytes, 8), 0,
                  "the first cycle of an epoch is 0, and the frame must match the vector");

    acq::stop();
}

ZTEST(tof_cliff_publisher, test_a_real_cycle_publishes_nothing_with_the_production_gate)
{
    /* The same path with the gate wired the way production wires it. Every sensor answers,
     * every sample is fresh, and not one frame is published -- which is the behaviour a
     * bring-up engineer should see on hardware today. */
    integration::start_fails = false;
    build_descs();
    zassert_equal(acq::init(integration::make_acq_config()), 0);

    pub::config c{make_pub_config()};
    /* Exactly how production wires it: the state from effective_mapping_state(), which
     * clamps PROVEN away unconditionally, read together with the epoch. */
    c.authorise = [] { return pub::authorisation{acq::effective_mapping_state(), 1}; };
    zassert_equal(pub::init(c), 0);

    zassert_equal(acq::bring_up(), 0);
    acq::run_cycle();
    acq::run_cycle();

    zassert_equal(count_id(ctr::kMeasId), 0,
                  "the production gate let a range onto the bus");
    pub::counters got{};
    pub::copy_counters(got);
    zassert_equal(got.suppressed_not_proven, 2 * kCliffSources);
    zassert_equal(got.measurements_sent, 0);

    acq::stop();
}

ZTEST(tof_cliff_publisher, test_a_real_bring_up_failure_still_publishes_health)
{
    /* Nothing starts, so no cycle produces anything -- and health still has to go out, or
     * a dead subsystem is indistinguishable from a silent bus. */
    integration::start_fails = true;
    build_descs();
    zassert_equal(acq::init(integration::make_acq_config()), 0);
    zassert_equal(pub::init(make_pub_config()), 0);

    (void)acq::bring_up();
    acq::run_cycle();
    pub::on_cliff_health(acq::snapshot(), acq::effective_mapping_state());

    zassert_equal(count_id(ctr::kMeasId), 0);
    zassert_equal(count_id(ctr::kHealthId), 1);

    const sent_frame *h{last_of(ctr::kHealthId)};
    zassert_not_null(h);
    zassert_equal(static_cast<uint8_t>(h->data[3] >> 4), 0x0,
                  "a chain that never started is UNKNOWN, not PROVEN");

    acq::stop();
}

/* --------------------------------------------------------- concurrency ------------ */

namespace concurrency {

/* The flush runs on its own thread so the test can observe the system while a measurement
 * send is parked inside the sink. Everything asserted here is about what remains possible
 * while that is true. */
K_THREAD_STACK_DEFINE(flush_stack, 2048);
struct k_thread flush_thread;

void flush_entry(void *, void *, void *)
{
    acq::cycle_facts f{};
    f.cycle_seq = 21;
    pub::on_cycle_complete(f);
}

} // namespace concurrency

ZTEST(tof_cliff_publisher, test_a_stalled_measurement_send_does_not_hold_the_heartbeat)
{
    /* The publisher's mutex is released before the sink is called, on purpose: holding it
     * across a can_send that can block for its whole timeout would put four measurements --
     * worst case four milliseconds of bounded waiting -- in front of the heartbeat.
     *
     * The consequence is that the SENDS are not serialised, only the publisher's state is.
     * That was true of the previous commit and went undocumented and untested; this is the
     * test that makes it a checked property rather than a claim. */
    gate_open = true;
    bus.block_measurements = true;

    begin(21);
    for (uint8_t role = 0; role < kCliffSources; ++role)
        pub::on_cliff_sample(role, 21, cliff_facts(role), one_valid_target(1100));

    k_thread_create(&concurrency::flush_thread, concurrency::flush_stack,
                    K_THREAD_STACK_SIZEOF(concurrency::flush_stack), concurrency::flush_entry,
                    NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);

    /* Wait until a measurement send is genuinely parked inside the sink. */
    zassert_equal(k_sem_take(&sink_entered, K_MSEC(500)), 0,
                  "the flush never reached the sink");
    zassert_true(bus.measurements_inside > 0);

    /* Health must get through while that is true. If the mutex were held across the send,
     * this would block until the release below and the timeout would fire. */
    pub::on_cliff_health(0, acq::mapping_state::not_ready);
    zassert_equal(count_id(ctr::kHealthId), 1,
                  "the heartbeat was held behind a stalled measurement send");

    /* And a counter snapshot must still be obtainable, and be self-consistent: the health
     * frame is already counted, the parked measurements are not yet. */
    pub::counters mid{};
    pub::copy_counters(mid);
    zassert_equal(mid.health_sent, 1);
    zassert_equal(mid.measurements_sent, 0, "counted a send that has not returned");

    /* Release every parked send and let the flush finish. */
    for (int i = 0; i < kCliffSources; ++i)
        k_sem_give(&sink_release);
    zassert_equal(k_thread_join(&concurrency::flush_thread, K_MSEC(500)), 0,
                  "the flush thread did not finish");

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.measurements_sent, kCliffSources, "the counts are not exact");
    zassert_equal(c.health_sent, 1);
    zassert_equal(c.send_failed_measurement, 0);
    zassert_equal(c.queue_full, 0, "the queue was corrupted under concurrent access");
    zassert_equal(c.discarded_stale_cycle, 0);
    zassert_equal(count_id(ctr::kMeasId), kCliffSources);
    zassert_equal(bus.measurements_inside, 0);

    /* The queue is empty again, so the next cycle starts clean. */
    begin(22);
    pub::on_cliff_sample(0, 22, cliff_facts(0), one_valid_target(900));
    bus.block_measurements = false;
    flush(22);
    zassert_equal(count_id(ctr::kMeasId), kCliffSources + 1);
}

/* --------------------------------------------------- the cycle health frame -------- */

namespace {

/* A sample the packer classifies as SENSOR_FAULT: a frame IS owed -- that is what makes the
 * fault reportable -- and the range is the far sentinel. */
struct tof_cliff_sample one_faulted_target()
{
    struct tof_cliff_sample s{};
    s.fresh = true;
    s.target_count = 1;
    s.entry_count = 1;
    s.entries[0].range_mm = 300;
    s.entries[0].range_status = 3; // SENSOR_FAULT in the contract's table
    return s;
}

/* A read that completed without producing a sample. The packer owes no frame for it, so the
 * produced bit must stay clear by construction rather than by a rule someone remembers. */
struct tof_cliff_sample one_no_sample()
{
    struct tof_cliff_sample s{};
    s.fresh = true;
    s.target_count = 1;
    s.entry_count = 1;
    s.entries[0].range_mm = 0;
    s.entries[0].range_status = 10; // SYNCRONISATION_INT -> NO_SAMPLE
    return s;
}

uint8_t health_flags(const sent_frame *f) { return static_cast<uint8_t>(f->data[3] & 0xF); }
uint8_t health_produced(const sent_frame *f) { return static_cast<uint8_t>(f->data[5] >> 4); }
uint8_t health_fault(const sent_frame *f) { return static_cast<uint8_t>(f->data[5] & 0xF); }
uint8_t health_enumerated(const sent_frame *f) { return static_cast<uint8_t>(f->data[4] >> 4); }

} // namespace

ZTEST(tof_cliff_publisher, test_a_completed_cycle_publishes_a_cycle_valid_health_frame)
{
    gate_open = true;
    begin(5);
    for (int i = 0; i < kCliffSources; ++i)
        pub::on_cliff_sample(i, 5, cliff_facts(static_cast<uint8_t>(i)), one_valid_target(400));
    flush(5);

    zassert_equal(count_id(ctr::kMeasId), 4);
    zassert_equal(count_id(ctr::kHealthId), 1, "the cycle owes exactly one health frame");

    /* Order is intent, not a wire guarantee -- but the producer must still put the
     * measurements first, because the mask it publishes describes sends that already happened. */
    zassert_equal(bus.frames[4].can_id, ctr::kHealthId, "health must follow its measurements");

    const sent_frame *h{last_of(ctr::kHealthId)};
    zassert_not_null(h);
    zassert_true((health_flags(h) & ctr::kCycleValidBit) != 0, "cycle_valid must be set");
    zassert_equal(h->data[7], 5, "cycle_seq must name the cycle just completed");
    zassert_equal(h->data[1], kEpoch);
    zassert_equal(health_produced(h), 0xF, "all four measurements reached the bus");
    zassert_equal(health_fault(h), 0x0);

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.cycle_health_sent, 1);
    zassert_equal(c.cycle_health_withheld, 0);
}

ZTEST(tof_cliff_publisher, test_the_masks_come_from_the_reduction_not_from_the_sample)
{
    /* Source 0 valid, source 1 faulted, source 2 produced no sample at all, source 3 valid. All
     * four samples are `fresh`, so anything keyed off freshness would report 0xF produced -- the
     * mask has to follow what the packer decided a frame was owed for. */
    gate_open = true;
    begin(9);
    pub::on_cliff_sample(0, 9, cliff_facts(0), one_valid_target(400));
    pub::on_cliff_sample(1, 9, cliff_facts(1), one_faulted_target());
    pub::on_cliff_sample(2, 9, cliff_facts(2), one_no_sample());
    pub::on_cliff_sample(3, 9, cliff_facts(3), one_valid_target(900));
    flush(9);

    zassert_equal(count_id(ctr::kMeasId), 3, "NO_SAMPLE owes no measurement frame");

    const sent_frame *h{last_of(ctr::kHealthId)};
    zassert_not_null(h);
    /* Bits 0, 1 and 3: source 2 produced nothing. */
    zassert_equal(health_produced(h), 0xB, "produced mask %02x", health_produced(h));
    /* A SENSOR_FAULT is in BOTH masks: a sample exists, and the sensor says it is unusable. */
    zassert_equal(health_fault(h), 0x2, "fault mask %02x", health_fault(h));
}

ZTEST(tof_cliff_publisher, test_a_failed_measurement_send_withholds_the_cycle_health)
{
    /* The contradiction this prevents: a mask claiming four samples with three on the wire. The
     * measurements that did go out are then left without an authorising health frame, so a
     * conforming consumer accepts none of them -- the whole cycle is dropped, which is the safe
     * direction and the same rule the revocation path uses. */
    gate_open = true;
    bus.fail_after = 2; // the third send onwards fails
    begin(3);
    for (int i = 0; i < kCliffSources; ++i)
        pub::on_cliff_sample(i, 3, cliff_facts(static_cast<uint8_t>(i)), one_valid_target(400));
    flush(3);

    zassert_equal(count_id(ctr::kHealthId), 0, "a health frame claimed sends that failed");

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.cycle_health_withheld, 1, "withholding must be counted, not silent");
    zassert_true(c.send_failed_measurement > 0);
    zassert_equal(c.cycle_health_sent, 0);
}

ZTEST(tof_cliff_publisher, test_the_cycle_health_uses_the_values_latched_for_that_cycle)
{
    /* Not a fresh read at send time. This is what lets a cycle's health frame arrive after a
     * newer heartbeat reporting LOST and still legitimately authorise its own cycle -- the
     * consumer correlates on (epoch, cycle_seq), never on arrival order. The enumeration mask is
     * the observable: it changes between the latch and the flush, and the frame must carry the
     * value that was true for the cycle it describes. */
    gate_open = true;
    enumerated_value = 0xF;
    model_verified_value = 0xF;
    begin(11);
    pub::on_cliff_sample(0, 11, cliff_facts(0), one_valid_target(400));

    enumerated_value = 0x3; // the authority moved on mid-cycle
    flush(11);

    const sent_frame *h{last_of(ctr::kHealthId)};
    zassert_not_null(h);
    zassert_equal(health_enumerated(h), 0xF, "the frame re-read the authority at send time");
}

ZTEST(tof_cliff_publisher, test_the_heartbeat_describes_no_cycle_but_does_describe_the_chain)
{
    /* The division of labour: the heartbeat carries the mapping and the last enumeration
     * attempt, and never a cycle. The per-cycle masks must be zero with cycle_valid clear --
     * encode_health refuses otherwise, so this is enforced rather than remembered. */
    enumerated_value = 0xF;
    model_verified_value = 0x7;
    chain_flags_value = 0x4;
    failing_position_value = 6;

    pub::on_cliff_health(0, acq::mapping_state::not_ready);

    const sent_frame *h{last_of(ctr::kHealthId)};
    zassert_not_null(h);
    zassert_equal(health_flags(h) & ctr::kCycleValidBit, 0, "a heartbeat must not claim a cycle");
    zassert_equal(h->data[7], 0, "cycle_seq must be zero with cycle_valid clear");
    zassert_equal(health_produced(h), 0);
    zassert_equal(health_fault(h), 0);
    zassert_equal(health_enumerated(h), 0xF);
    zassert_equal(static_cast<uint8_t>(h->data[4] & 0xF), 0x7);
    zassert_equal(static_cast<uint8_t>(health_flags(h) & 0x7), 0x4);
    zassert_equal(h->data[6], 6, "the failing position travels with the snapshot");
}

ZTEST(tof_cliff_publisher, test_health_seq_advances_even_when_the_send_fails)
{
    /* health_seq is the liveness signal: a NEW snapshot must carry a new number. Deriving it
     * from a success counter -- which it used to be -- makes the frame after a failure repeat a
     * number, and a repeated health_seq reads as a retransmission of something stale rather than
     * as fresh evidence that the producer is alive. */
    pub::on_cliff_health(0, acq::mapping_state::not_ready);
    const sent_frame *first{last_of(ctr::kHealthId)};
    zassert_not_null(first);
    const uint8_t seq_before{first->data[2]};

    bus.fail_after = bus.send_calls; // the next send fails
    pub::on_cliff_health(0, acq::mapping_state::not_ready);
    bus.fail_after = -1;
    pub::on_cliff_health(0, acq::mapping_state::not_ready);

    const sent_frame *third{last_of(ctr::kHealthId)};
    zassert_not_null(third);
    zassert_equal(static_cast<uint8_t>(third->data[2]),
                  static_cast<uint8_t>(seq_before + 2),
                  "the failed snapshot did not consume a sequence number");
}

ZTEST(tof_cliff_publisher, test_the_two_health_producers_never_share_a_sequence_number)
{
    /* One counter, allocated under the lock, so a heartbeat and a cycle health frame cannot take
     * the same number. Two producers deriving from one success counter would have. */
    gate_open = true;
    pub::on_cliff_health(0, acq::mapping_state::not_ready);
    begin(2);
    for (int i = 0; i < kCliffSources; ++i)
        pub::on_cliff_sample(i, 2, cliff_facts(static_cast<uint8_t>(i)), one_valid_target(400));
    flush(2);
    pub::on_cliff_health(0, acq::mapping_state::not_ready);

    uint8_t seqs[8]{};
    int n{0};
    for (int i = 0; i < bus.count; ++i)
        if (bus.frames[i].can_id == ctr::kHealthId && n < 8)
            seqs[n++] = bus.frames[i].data[2];
    zassert_equal(n, 3, "expected heartbeat, cycle health, heartbeat");
    zassert_not_equal(seqs[0], seqs[1]);
    zassert_not_equal(seqs[1], seqs[2]);
    zassert_not_equal(seqs[0], seqs[2]);
}

/* ------------------------------------------- zero-measurement and invalid cycles --- */

ZTEST(tof_cliff_publisher, test_a_cycle_with_no_measurements_still_publishes_its_health)
{
    /* The contract allows a cycle to carry between zero and four measurements. Without this
     * frame, "completed, and all four sources had nothing to report" and "the cycle never
     * happened" are the same silence -- and the consumer's only remaining signal would be a
     * timeout, which means a stopped producer.
     *
     * This is also why the authorisation is latched by on_cycle_begin rather than by the first
     * sample: for this cycle there is no first sample to latch on. */
    gate_open = true;
    begin(4);
    flush(4);

    zassert_equal(count_id(ctr::kMeasId), 0);
    zassert_equal(count_id(ctr::kHealthId), 1, "an empty cycle owes a health frame too");

    const sent_frame *h{last_of(ctr::kHealthId)};
    zassert_not_null(h);
    zassert_true((health_flags(h) & ctr::kCycleValidBit) != 0);
    zassert_equal(h->data[7], 4, "it names the cycle that completed");
    zassert_equal(health_produced(h), 0x0, "nothing was produced, and it says so");
    zassert_equal(health_fault(h), 0x0);

    /* The shape is one the contract's own vectors accept: cycle_valid set with an empty produced
     * mask is `health_position_with_incomplete_mask`, so this is not a frame only we believe in. */
    const auto *v{find_vector("health_position_with_incomplete_mask")};
    zassert_not_null(v);
    zassert_true((v->bytes[3] & ctr::kCycleValidBit) != 0);
    zassert_equal(static_cast<uint8_t>(v->bytes[5] >> 4), 0);
}

ZTEST(tof_cliff_publisher, test_an_empty_cycle_outside_proven_publishes_nothing)
{
    /* The empty-cycle rule does not become a way around the gate. Outside PROVEN there is no
     * trustworthy source_id at all, so there is no cycle worth naming -- liveness is the
     * heartbeat's job, and it is still running. */
    state_value = acq::mapping_state::not_ready;
    begin(4);
    flush(4);
    zassert_equal(bus.count, 0);
}

ZTEST(tof_cliff_publisher, test_a_packer_refusal_invalidates_the_whole_cycle)
{
    /* The sharpest of the withholding cases. A refusal drops one measurement, and the health
     * frame that followed would be well formed with one bit missing -- indistinguishable from a
     * sensor with nothing to report. That disguises a producer defect as ordinary quiet: the bug
     * becomes invisible BECAUSE the frame looks right. */
    gate_open = true;
    struct tof_cliff_sample bad{};
    bad.fresh = true;
    bad.target_count = 2;
    bad.entry_count = 1; // count disagrees with the entries: the packer must refuse
    bad.entries[0].range_mm = 400;
    bad.entries[0].range_status = 0;

    begin(7);
    pub::on_cliff_sample(0, 7, cliff_facts(0), one_valid_target(400));
    pub::on_cliff_sample(1, 7, cliff_facts(1), bad);
    flush(7);

    /* Nothing at all reaches the bus. An earlier version sent the good measurement and only
     * withheld the health frame, which was safe -- a conforming consumer refuses an unauthorised
     * measurement -- but it made "the whole cycle is dropped" true at the consumer and false on
     * the wire. A claim that holds only one layer away gets quoted without the qualifier. */
    zassert_equal(bus.count, 0, "a structural failure must not put anything on the bus");
    zassert_equal(bus.send_calls, 0, "and must not even offer it");

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.cycles_invalid, 1);
    zassert_true(c.suppressed_packer_refused > 0);
}

ZTEST(tof_cliff_publisher, test_a_role_mismatch_invalidates_the_whole_cycle)
{
    /* A wiring defect, not a sensor outcome: the facts and the descriptor for that index do not
     * belong together, so nothing about that cycle can be trusted. */
    gate_open = true;
    acq::source_facts wrong{cliff_facts(0)};
    wrong.role_id = 3; // index 0's descriptor says role 0

    begin(8);
    pub::on_cliff_sample(0, 8, cliff_facts(0), one_valid_target(400));
    pub::on_cliff_sample(0, 8, wrong, one_valid_target(500));
    flush(8);

    zassert_equal(bus.count, 0, "the whole cycle goes, on the wire and not just in the consumer");
    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.cycles_invalid, 1);
    zassert_true(c.suppressed_role_mismatch > 0);
}

ZTEST(tof_cliff_publisher, test_a_no_sample_read_leaves_the_cycle_valid)
{
    /* The line between "a defect happened" and "a sensor had nothing". A NO_SAMPLE is ordinary:
     * its bit stays clear and the cycle keeps its health frame. Treating it as invalid would
     * throw away every cycle in which one sensor was still warming up. */
    gate_open = true;
    begin(6);
    pub::on_cliff_sample(0, 6, cliff_facts(0), one_valid_target(400));
    pub::on_cliff_sample(1, 6, cliff_facts(1), one_no_sample());
    flush(6);

    zassert_equal(count_id(ctr::kMeasId), 1);
    const sent_frame *h{last_of(ctr::kHealthId)};
    zassert_not_null(h);
    zassert_equal(health_produced(h), 0x1);

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.cycles_invalid, 0, "a quiet sensor is not a defect");
}

ZTEST(tof_cliff_publisher, test_a_sample_for_a_cycle_nobody_announced_is_refused)
{
    /* on_cycle_begin is what latches the authorisation, so a sample outside an announced cycle
     * has nothing to be published under. Inventing one here is precisely how the empty-cycle
     * hole was created: the latch used to happen on the first sample. */
    gate_open = true;
    pub::on_cliff_sample(0, 30, cliff_facts(0), one_valid_target(400));
    flush(30);

    zassert_equal(bus.count, 0);
    pub::counters c{};
    pub::copy_counters(c);
    zassert_true(c.suppressed_cycle_not_begun >= 2, "the sample and the completion both count");
}

ZTEST(tof_cliff_publisher, test_only_a_send_failure_can_leave_measurements_unauthorised)
{
    /* The residue, and the reason the two cases are described separately. A failed send is not
     * knowable until it is attempted, so this path alone can put measurements on the bus with no
     * authorising health frame -- a revocation at the CONSUMER rather than on the wire. */
    gate_open = true;
    bus.fail_after = 2;
    begin(40);
    for (int i = 0; i < kCliffSources; ++i)
        pub::on_cliff_sample(i, 40, cliff_facts(static_cast<uint8_t>(i)), one_valid_target(400));
    flush(40);

    zassert_equal(count_id(ctr::kMeasId), 2, "the sends that succeeded are on the bus");
    zassert_equal(count_id(ctr::kHealthId), 0, "and nothing authorises them");

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.cycle_health_withheld, 1);
    zassert_equal(c.cycles_invalid, 0, "a transport failure is not a structural defect");
}
