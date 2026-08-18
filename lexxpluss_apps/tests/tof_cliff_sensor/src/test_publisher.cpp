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
} bus;

int fake_send(uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
    ++bus.send_calls;
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

/* The injected gate. Closed by default, so a test that forgets to open it sees production
 * behaviour rather than an accidental publish. */
bool gate_open{false};
bool test_gate() { return gate_open; }
uint8_t epoch_provider() { return kEpoch; }

/* The publisher needs the descriptor table for kind and role_id. Same shape the
 * acquisition harness uses: four cliff sources then two grid stubs. */
acq::source_desc descs[acq::kMaxSources];

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
    c.publication_allowed = test_gate;
    c.mapping_epoch = epoch_provider;
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

void before(void *)
{
    bus = {};
    bus.fail_after = -1;
    gate_open = false;
    build_descs();
    zassert_equal(pub::init(make_pub_config()), 0);
}

} // namespace

ZTEST_SUITE(tof_cliff_publisher, NULL, NULL, before, NULL, NULL);

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
    for (uint8_t role = 0; role < kCliffSources; ++role)
        pub::on_cliff_sample(role, 1, cliff_facts(role), s);

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
    pub::on_cliff_health(0, acq::mapping_state::fault);
    pub::on_cliff_health(0, acq::mapping_state::not_ready);

    zassert_equal(count_id(ctr::kHealthId), 3);
    zassert_equal(count_id(ctr::kMeasId), 0, "health must not drag a measurement with it");

    pub::counters c{};
    pub::copy_counters(c);
    zassert_equal(c.health_sent, 3);
}

/* ------------------------------------------------- the wire mapping_state trap ---- */

ZTEST(tof_cliff_publisher, test_the_two_mapping_state_enumerations_are_translated)
{
    /* A straight cast would put PROVEN on the wire for tof_acq's `fault`, and LOST for its
     * `proven`. The one field the consumer gates on, mis-reported silently. */
    zassert_equal(pub::wire_mapping_state(acq::mapping_state::not_ready), 0x0); // UNKNOWN
    zassert_equal(pub::wire_mapping_state(acq::mapping_state::proven), 0x1);    // PROVEN
    zassert_equal(pub::wire_mapping_state(acq::mapping_state::fault), 0x3);     // FAULT
    /* And the values really do differ, which is why the switch exists. */
    zassert_not_equal(pub::wire_mapping_state(acq::mapping_state::fault),
                      static_cast<uint8_t>(acq::mapping_state::fault));
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
    pub::config c{make_pub_config()};
    static uint8_t epoch_one_value{1};
    c.mapping_epoch = [] { return epoch_one_value; };
    zassert_equal(pub::init(c), 0);
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
    pub::config c{make_pub_config()};
    static uint8_t one{1};
    c.mapping_epoch = [] { return one; };
    zassert_equal(pub::init(c), 0);
    gate_open = true;

    /* The vector is source 0, epoch 1, cycle 0, range 1234, status 0, one target. */
    pub::on_cliff_sample(0, 0, cliff_facts(0), one_valid_target(1234));

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
    pub::on_cliff_sample(0, 0, cliff_facts(0), one_valid_target(900));
    pub::on_cliff_sample(0, 1, cliff_facts(0), one_valid_target(900));
    pub::on_cliff_sample(0, 258, cliff_facts(0), one_valid_target(900));

    zassert_equal(count_id(ctr::kMeasId), 3);
    zassert_equal(bus.frames[0].data[2], 0);
    zassert_equal(bus.frames[1].data[2], 1);
    zassert_equal(bus.frames[2].data[2], 2, "cycle_seq wraps 255 -> 0 on the wire");
}

ZTEST(tof_cliff_publisher, test_a_stale_sample_is_not_published)
{
    gate_open = true;
    auto s{one_valid_target(900)};
    s.fresh = false;

    pub::on_cliff_sample(0, 1, cliff_facts(0), s);

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

    pub::on_cliff_sample(0, 1, cliff_facts(0), s);

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

    pub::on_cliff_sample(4, 1, grid, one_valid_target(900));

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

    pub::on_cliff_sample(0, 1, cliff_facts(0), one_valid_target(900));
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
    pub::on_cliff_sample(0, 1, cliff_facts(0), one_valid_target(900)); // fails
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
    zassert_equal(missing(c, [](pub::config &x) { x.publication_allowed = nullptr; }), -EINVAL);
    zassert_equal(missing(c, [](pub::config &x) { x.mapping_epoch = nullptr; }), -EINVAL);
    zassert_equal(missing(c, [](pub::config &x) { x.sources = nullptr; }), -EINVAL);
    zassert_equal(missing(c, [](pub::config &x) { x.source_count = 0; }), -EINVAL);
    zassert_equal(missing(c, [](pub::config &x) { x.source_count = acq::kMaxSources + 1; }),
                  -EINVAL);
}
