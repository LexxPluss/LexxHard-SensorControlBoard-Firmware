/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host tests for the one runtime bootstrap.
 *
 * Everything real except the bus: the authority, publisher, packer, acquisition and the runtime
 * itself are the production modules, and only the CAN glue and the vendor sensor entry points are
 * stubbed. That matters here more than usual, because the whole point of this module is that there
 * is exactly ONE wiring -- a test that wired the modules up itself would be a third wiring, testing
 * nothing about the second one it was written to eliminate.
 *
 * The end-to-end case runs a REAL proof over the shared chain fake, because "role_id comes from the
 * installed mapping and never from the descriptor's index" cannot be checked without a mapping that
 * a proof actually installed.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "fake_chain.hpp"
#include "tof_acquisition.hpp"
#include "tof_chain_controller.hpp"
#include "tof_cliff_can.hpp"
#include "tof_cliff_publisher.hpp"
#include "tof_cliff_runtime.hpp"
#include "tof_can_ids.hpp"
#include "tof_cliff_contract.h"
#include "tof_grid_publisher.hpp"
#include "tof_l7_sensor.hpp"
#include "tof_commissioning.hpp"
#include "tof_mapping_authority.hpp"

namespace acq = lexxhard::tof_acq;
namespace au = lexxhard::tof_authority;
namespace cm = lexxhard::tof_commissioning;
namespace enm = lexxhard::tof_enum;
namespace pf = lexxhard::tof_proof;
namespace pub = lexxhard::tof_cliff_pub;
namespace rt = lexxhard::tof_cliff_runtime;
namespace gpub = lexxhard::tof_grid_pub;
namespace ids = lexxhard::tof_can_ids;
namespace l7 = lexxhard::tof_l7;

void ignore_grid(int, uint32_t, const acq::source_facts &, const lexxhard::tof_l7::sample &) {}

namespace {

int can_init_rc{0};
int send_rc{0};
int frames_sent{0};
uint16_t last_can_id{0};

/* Every frame either publisher offered, in order. The grid tests need the BYTES -- a generation
 * and a source id are what say which sensor a grid came from -- and the fan-out test needs to see
 * both identifier pairs in one cycle. 96 is three cycles' worth of a fully populated chain. */
struct sent_frame {
    uint16_t can_id;
    uint8_t data[8];
};
sent_frame bus_frames[96];
int bus_count{0};

int fake_send(uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
    ++frames_sent;
    last_can_id = can_id;
    if (bus_count < static_cast<int>(sizeof bus_frames / sizeof bus_frames[0])) {
        sent_frame &f{bus_frames[bus_count++]};
        f.can_id = can_id;
        memcpy(f.data, data, dlc <= 8 ? dlc : 8);
    }
    return send_rc;
}

int frames_with_id(uint16_t id)
{
    int n{0};
    for (int i = 0; i < bus_count; ++i)
        if (bus_frames[i].can_id == id)
            ++n;
    return n;
}

const sent_frame *first_with_id(uint16_t id)
{
    for (int i = 0; i < bus_count; ++i)
        if (bus_frames[i].can_id == id)
            return &bus_frames[i];
    return nullptr;
}

/* The grid publisher's sink, failed on purpose. The runtime builds the publisher's configuration
 * itself, so this is the only way to make that init fail from outside it -- and it is the case
 * that matters: a publisher that never came up answers every call by returning, and the grids
 * would be read and dropped with nothing on the bus to say so. */
bool grid_sink_broken{false};

fake::fake_chain chain{};

}  // namespace

/* The CAN glue, stubbed at the boundary the runtime actually uses. Linking the real one would pull
 * in a Zephyr CAN controller; what production does with these three entry points is covered by the
 * publisher suite. production_authorisation() is reproduced faithfully -- ONE authority read, with
 * the state and the epoch taken from that single snapshot -- because a permissive stub would make
 * every gating assertion in this file meaningless. (It also used to apply the acquisition layer's
 * PROVEN clamp, which production no longer has.) */
namespace lexxhard::tof_cliff_can {

int init()
{
    return can_init_rc;
}

struct tof_cliff_pub::can_sink sink()
{
    return {fake_send};
}

struct tof_grid_pub::can_sink grid_sink()
{
    if (grid_sink_broken)
        return {nullptr};
    return {fake_send};
}

/* Reproduced from production for the same reason as the cliff's: one authority read, and the two
 * chain-level flags false because a grid is only ever published under a proven whole chain. A
 * permissive stub here would make every gating assertion in the grid tests meaningless. */
struct tof_grid_pub::authorisation grid_production_authorisation()
{
    const au::snapshot now{au::current()};
    struct tof_grid_pub::authorisation a{};

    a.state = now.state;
    a.epoch = now.epoch;
    a.boards_detected = now.state == acq::mapping_state::proven
                            ? static_cast<uint8_t>(lexxhard::tof_proof::kCommissioningPositions)
                            : 0;
    a.chain_length_unexpected = false;
    a.other_position_enumeration_failed = false;
    return a;
}

struct tof_cliff_pub::authorisation production_authorisation()
{
    const au::snapshot now{au::current()};
    struct tof_cliff_pub::authorisation a{};

    /* Mirrors production: one snapshot, state taken straight from it. It used to run the state
     * through the PROVEN clamp, which production no longer has. */
    a.state = now.state;
    a.epoch = now.epoch;
    a.enumerated_mask = now.enumerated_mask;
    a.model_verified_mask = now.model_verified_mask;
    a.chain_flags = now.chain_flags;
    a.failing_position = now.failing_position;
    return a;
}

}  // namespace lexxhard::tof_cliff_can

/* ------------------------------------------------------- the VL53L7CX ULD, faked -------
 *
 * The real adapter is linked (tof_l7_sensor.cpp): what this suite is about is the path from a
 * devicetree number to the vendor call, so the adapter has to be the production one and only the
 * vendor's entry points are replaced. A fake that stood in for the ADAPTER would prove the
 * scheduler talks to the fake.
 *
 * Deliberately not shared with tests/tof_l7_sensor's fake, which reproduces the ULD's observable
 * defects for a lifecycle suite. This one exists to be driven: a data-ready flag the test owns and
 * a grid it can recognise on the bus. One fake serving both would have to grow both jobs.
 */
namespace {

uint8_t uld_ready_value{0};
uint8_t uld_init_status{VL53L7CX_STATUS_OK};
uint8_t uld_frequency_status{VL53L7CX_STATUS_OK};
int uld_frequency_calls{0};
uint8_t uld_last_frequency{0};
/* Per device, so "both sensors were configured" is a statement about two objects rather than
 * about the last one to be touched. */
const VL53L7CX_Configuration *uld_frequency_devices[4]{};
int uld_frequency_device_count{0};
uint8_t uld_firmware_byte{0xA5};
VL53L7CX_ResultsData uld_results{};

void good_grid()
{
    uld_results = VL53L7CX_ResultsData{};
    for (size_t zone{0}; zone < l7::kZoneCount; ++zone) {
        uld_results.nb_target_detected[zone] = 1;
        /* A ramp, so a frame's bytes identify the grid rather than just its shape. */
        uld_results.distance_mm[zone] = static_cast<int16_t>(100 + zone);
        uld_results.target_status[zone] = 5;
    }
}

void reset_uld_fake()
{
    uld_ready_value = 0;
    uld_init_status = VL53L7CX_STATUS_OK;
    uld_frequency_status = VL53L7CX_STATUS_OK;
    uld_frequency_calls = 0;
    uld_last_frequency = 0;
    uld_frequency_device_count = 0;
    for (auto &d : uld_frequency_devices)
        d = nullptr;
    good_grid();
}

}  // namespace

/* The verified-firmware boundary. tof_l7_runtime.cpp is not linked here -- it reads a flash
 * partition -- and the adapter has no test-only path that takes a raw pointer, so the only way to
 * give it a payload is to be the runtime. */
namespace lexxhard::tof_l7_runtime {

const uint8_t *firmware_data()
{
    return &uld_firmware_byte;
}

size_t firmware_size()
{
    return VL53L7CX_FIRMWARE_DOWNLOAD_SIZE;
}

}  // namespace lexxhard::tof_l7_runtime

extern "C" {

void vl53l7cx_port_clear_error(void) {}
int vl53l7cx_port_error(void)
{
    return 0;
}

uint8_t vl53l7cx_init(VL53L7CX_Configuration *)
{
    return uld_init_status;
}

uint8_t vl53l7cx_set_resolution(VL53L7CX_Configuration *, uint8_t)
{
    return VL53L7CX_STATUS_OK;
}

uint8_t vl53l7cx_set_ranging_frequency_hz(VL53L7CX_Configuration *p_dev, uint8_t frequency_hz)
{
    ++uld_frequency_calls;
    uld_last_frequency = frequency_hz;
    if (uld_frequency_device_count < 4)
        uld_frequency_devices[uld_frequency_device_count++] = p_dev;
    return uld_frequency_status;
}

uint8_t vl53l7cx_start_ranging(VL53L7CX_Configuration *)
{
    return VL53L7CX_STATUS_OK;
}

uint8_t vl53l7cx_stop_ranging(VL53L7CX_Configuration *)
{
    return VL53L7CX_STATUS_OK;
}

uint8_t vl53l7cx_check_data_ready(VL53L7CX_Configuration *, uint8_t *is_ready)
{
    *is_ready = uld_ready_value;
    return VL53L7CX_STATUS_OK;
}

uint8_t vl53l7cx_get_ranging_data(VL53L7CX_Configuration *, VL53L7CX_ResultsData *results)
{
    *results = uld_results;
    return VL53L7CX_STATUS_OK;
}

}  // extern "C"

/* The chain controller's readiness, stubbed. The runtime refuses to bootstrap while this is false,
 * which is how the ordering between "the control lines are configured" and "sensors may be brought
 * up" stopped being a convention about where two calls sit. */
bool glue_is_ready{true};

namespace lexxhard::tof_chain_controller {
bool glue_ready()
{
    return glue_is_ready;
}
}  // namespace lexxhard::tof_chain_controller

namespace {

/* Injected, as production injects it from the devicetree: cadence, health period, join timeout,
 * thread priority. Nothing here has a default anywhere in the module. */
/* The last two are the L4 ranging profile: what VL53LX_DataInit already leaves, stated because
 * the runtime refuses to bootstrap without it. */
/* SEVEN Hz, and not the overlay's five. The production overlay says 5, so a suite that also said
 * 5 would pass against an implementation that ignored the configuration and hard-coded the
 * overlay's number -- which is exactly the defect this path can have. Every assertion below about
 * the frequency is an assertion that SEVEN arrived. */
constexpr uint8_t kGridHz{7};
constexpr rt::config kTiming{50, 20, 400, K_PRIO_PREEMPT(5), 33333, 2, kGridHz};

/* The acquisition thread's stack. The runtime sizes its own from a devicetree property; there is no
 * devicetree here, and a fallback compiled into the module for tests would be a size nobody chose
 * that shipped anyway -- so the suite hands one over. */
K_THREAD_STACK_DEFINE(runtime_acq_stack, 2048);

int quiesce_via_acquisition()
{
    return acq::try_stop();
}

void before(void *)
{
    glue_is_ready = true;
    rt::reset_for_test();
    rt::set_thread_stack_for_test(runtime_acq_stack, K_THREAD_STACK_SIZEOF(runtime_acq_stack));
    au::reset_epoch_history_for_test();
    chain = fake::fake_chain{};
    can_init_rc = 0;
    send_rc = 0;
    frames_sent = 0;
    last_can_id = 0;
    bus_count = 0;
    grid_sink_broken = false;
    reset_uld_fake();
}

/* The roles used to be injected here, because dasher_spec() carried l4_role::unknown and PROVEN was
 * unreachable by rule. They are now frozen in the production spec itself (from
 * dasher_connectivity.png), so these cases exercise the REAL configuration -- which is strictly
 * better: a test that installs its own role table cannot notice the shipped one being wrong. */
/* The runtime suite drives the REAL chain glue over a fake bus, so it has no i2c to retime. The
 * transaction still owns the speed, and a hook that always succeeds is the honest stand-in: these
 * cases are about the runtime's keying and start-up rules, and the speed transaction's own failure
 * paths are covered in test_main.cpp where the hook can be made to fail on demand. */
int speed_ok(cm::bus_speed)
{
    return 0;
}

cm::outcome prove_over_the_fake_chain(uint32_t epoch)
{
    cm::config ccfg{};

    ccfg.chain = &lexxhard::tof_chain_controller::chain_lock();
    ccfg.ops = &chain;
    ccfg.spec = &rt::spec();
    ccfg.quiesce = quiesce_via_acquisition;
    ccfg.set_bus_speed = speed_ok;
    zassert_equal(cm::init(ccfg), 0);
    return cm::prove(epoch);
}

}  // namespace

ZTEST_SUITE(tof_cliff_runtime, NULL, NULL, before, NULL, NULL);

ZTEST(tof_cliff_runtime, test_one_call_brings_the_whole_subsystem_up)
{
    zassert_equal(rt::bootstrap(kTiming), 0);
    zassert_true(rt::ready());
    zassert_equal(rt::current_stage(), rt::stage::ready);
}

ZTEST(tof_cliff_runtime, test_the_heartbeat_starts_unproven_and_does_not_wait_for_a_proof)
{
    /* The boundary that says a board must be audible from power-on. Nothing has been proven, nothing
     * has been enumerated, and no sensor has been opened -- and the health frame is already on the
     * bus reporting that. Silence until commissioning would make an unprovisioned board
     * indistinguishable from a dead one. */
    zassert_equal(rt::bootstrap(kTiming), 0);
    zassert_false(rt::mapping_applied());

    k_msleep(kTiming.health_period_ms * 4);

    pub::counters c{};
    pub::copy_counters(c);
    zassert_true(c.health_sent >= 3, "heartbeats sent: %u", c.health_sent);
    zassert_equal(c.measurements_sent, 0u, "an unproven subsystem published a measurement");
    zassert_true(frames_sent >= 3);
    zassert_equal(last_can_id, 0x217, "the heartbeat went out on the wrong id");
    zassert_not_equal(au::current().state, acq::mapping_state::proven);
}

ZTEST(tof_cliff_runtime, test_a_second_bootstrap_does_not_re_initialise_the_authority)
{
    /* The specific hazard that made two wirings dangerous rather than merely untidy. A second
     * au::init() clears the installed mapping and any open attempt, so a build where something else
     * also bootstrapped would have a different authority state machine -- and it was the budget
     * build, whose numbers get quoted. */
    zassert_equal(rt::bootstrap(kTiming), 0);

    const au::attempt a{au::begin_proof()};
    zassert_true(a.opened(), "could not open an attempt to have something to lose");
    const uint32_t nonce{au::attempt_nonce()};
    zassert_not_equal(nonce, 0u);

    zassert_equal(rt::bootstrap(kTiming), -EALREADY, "the second bootstrap ran anyway");
    zassert_equal(au::attempt_nonce(), nonce, "the second bootstrap re-initialised the authority");
    zassert_true(rt::ready());

    au::abort_proof(a.challenge);
}

ZTEST(tof_cliff_runtime, test_timing_has_no_defaults_here_either)
{
    /* Both periods are unresolved symbols in the wire contract. Refusing zero is what keeps this
     * layer from becoming the place a placeholder quietly turns into the specification. */
    zassert_equal(rt::bootstrap(rt::config{0, 20, 400, K_PRIO_PREEMPT(5), 33333, 2, kGridHz}), -EINVAL);
    zassert_equal(rt::current_stage(), rt::stage::not_started,
                  "a refused config still wired something up");
    zassert_equal(rt::bootstrap(rt::config{50, 0, 400, K_PRIO_PREEMPT(5), 33333, 2, kGridHz}), -EINVAL);
    zassert_equal(rt::current_stage(), rt::stage::not_started);

    /* The ranging profile has no default either, and is refused HERE rather than as a generic
     * -EINVAL from the acquisition layer: this is the stage that can say which value was wrong.
     * Until this commit configure() was a no-op, so the parts ran on the vendor's default and
     * there was nothing to refuse. */
    zassert_equal(rt::bootstrap(rt::config{50, 20, 400, K_PRIO_PREEMPT(5), 0, 2, kGridHz}), -EINVAL,
                  "a bootstrap with no timing budget was accepted");
    zassert_equal(rt::current_stage(), rt::stage::not_started);
    zassert_equal(rt::bootstrap(rt::config{50, 20, 400, K_PRIO_PREEMPT(5), 33333, 0, kGridHz}), -EINVAL);
    zassert_equal(rt::current_stage(), rt::stage::not_started);
    zassert_equal(rt::bootstrap(rt::config{50, 20, 400, K_PRIO_PREEMPT(5), 33333, 4, kGridHz}), -EINVAL,
                  "a distance mode the ULD does not define was accepted");
    zassert_equal(rt::current_stage(), rt::stage::not_started);
    /* SHORT. Defined by the enumeration, refused by the ULD for an L4 part, so refused here too
     * rather than left to fail at bring-up on every sensor. */
    zassert_equal(rt::bootstrap(rt::config{50, 20, 400, K_PRIO_PREEMPT(5), 33333, 1, kGridHz}), -EINVAL,
                  "SHORT was accepted, and the L4 ULD rejects it");
    zassert_equal(rt::current_stage(), rt::stage::not_started);
    zassert_false(rt::ready());
}

ZTEST(tof_cliff_runtime, test_a_failing_step_stops_the_sequence_and_is_named)
{
    /* Acquisition made to fail by already being live: init() answers -EALREADY while a health work
     * item may be reading its configuration. The bootstrap must stop there and say so -- "not ready"
     * alone sends an operator to look at the chain, which is the wrong place. */
    acq::config c{};
    acq::source_desc d{};

    d.kind = acq::model::l7_grid;
    d.grid_ops = &acq::l7_grid_stub_ops();
    c.sources = &d;
    c.source_count = 1;
    c.periods.cycle_period_ms = 50;
    c.periods.health_period_ms = 20;
    c.hooks.on_cycle_begin = pub::on_cycle_begin;
    c.hooks.on_cycle = pub::on_cycle_complete;
    c.hooks.on_cliff_sample = pub::on_cliff_sample;
    c.hooks.on_grid_sample = ignore_grid;
    c.hooks.on_cliff_health = pub::on_cliff_health;
    c.mapping_state_provider = au::state_provider;
    c.now_ms = k_uptime_get_32;
    c.now_cycles = k_cycle_get_32;
    zassert_equal(acq::init(c), 0);

    zassert_equal(rt::bootstrap(kTiming), -EALREADY);
    zassert_equal(rt::current_stage(), rt::stage::acquisition_failed);
    zassert_false(rt::ready(), "a half-wired subsystem reported ready");
    zassert_true(strcmp(rt::stage_name(rt::current_stage()), "acquisition_failed") == 0,
                 "stage reported as %s", rt::stage_name(rt::current_stage()));
}

ZTEST(tof_cliff_runtime, test_a_board_with_no_can_still_comes_up)
{
    /* Tolerated deliberately: "this board cannot reach the bus" is a diagnosis a consumer can only
     * reach from a board that is otherwise alive, so the health path has to run. */
    can_init_rc = -ENODEV;
    zassert_equal(rt::bootstrap(kTiming), 0);
    zassert_true(rt::ready());
}

ZTEST(tof_cliff_runtime, test_the_gates_refuse_before_the_bootstrap_has_run)
{
    zassert_equal(rt::current_stage(), rt::stage::not_started);
    zassert_false(rt::mapping_applied());
    zassert_equal(rt::start_acquisition(), -EPERM);
}

ZTEST(tof_cliff_runtime, test_descriptors_come_from_the_spec_and_are_keyed_only_by_a_proof)
{
    /* The end-to-end shape of Step 4B's first half, with a real proof over the chain fake.
     *
     * Before: addresses from the chain spec (0x2A..0x2F, the ones enumeration assigns -- the old
     * probe wiring used 0x30.. which no spec ever assigns) and role_id unassigned everywhere.
     * After a committed proof: every cliff descriptor carries the contract's source_id for the role
     * the mapping proved at that position, and NOT its index. Those two disagree for this chain,
     * which is what makes the assertion worth making: position 3 is index 2 and front_left is
     * source 0. */
    zassert_equal(rt::bootstrap(kTiming), 0);

    const acq::source_desc *d{rt::descriptors_for_test()};

    zassert_equal(d[0].addr_7bit, 0x2A);
    zassert_equal(d[2].addr_7bit, 0x2C);
    zassert_equal(d[5].addr_7bit, 0x2F);
    zassert_equal(d[0].kind, acq::model::l7_grid);
    zassert_equal(d[2].kind, acq::model::l4_cliff);
    for (int i{0}; i < 6; ++i)
        zassert_equal(d[i].role_id, rt::kRoleUnassigned, "position %d was keyed early", i + 1);

    /* The keying happens INSIDE the commit now, through the authority's install callback -- there is
     * no separate step for a caller to forget, get wrong, or do from a mapping nobody proved. */
    const cm::outcome r{prove_over_the_fake_chain(7)};
    zassert_true(r.proven(), "the proof failed at stage %d", static_cast<int>(r.failed_at));
    zassert_equal(au::current().state, acq::mapping_state::proven);
    zassert_true(rt::mapping_applied());

    zassert_equal(d[2].role_id, 0, "position 3 (index 2) must key to front_left = source 0");
    zassert_equal(d[3].role_id, 1);
    zassert_equal(d[4].role_id, 2);
    zassert_equal(d[5].role_id, 3);
    /* The grid positions are keyed too, and from their own field: the fingerprint's source_id,
     * which the contract owns. They used to stay unassigned because nothing could publish them. */
    zassert_equal(d[0].role_id, 0);
    zassert_equal(d[1].role_id, 1);

    /* Only now may the acquisition thread start -- and from here on it is the only thing allowed to
     * touch a sensor. */
    zassert_equal(rt::start_acquisition(), 0);
    zassert_true(acq::thread_running());
    k_msleep(kTiming.cycle_period_ms * 2);
    zassert_equal(acq::try_stop(), 0, "the thread did not stop cleanly");
}

ZTEST(tof_cliff_runtime, test_nothing_is_keyed_or_started_before_a_proof)
{
    zassert_equal(rt::bootstrap(kTiming), 0);
    zassert_false(rt::mapping_applied());
    zassert_equal(rt::start_acquisition(), -EPERM, "acquisition started with unkeyed descriptors");
}

/* ------------------------------------------- a refused keying ----------------------- */

/* Staged the way it would really happen: the descriptors are built from one spec, the spec is then
 * edited, and the proof that follows proves the EDITED chain. matches_runtime() therefore passes --
 * the proven chain is the configured chain -- and the mismatch is only visible where it matters, at
 * the descriptors acquisition will actually read. */
void make_the_descriptors_stale_at(size_t position_index, uint8_t built_addr)
{
    const uint8_t real_addr{rt::spec().at[position_index].target_addr};

    rt::spec().at[position_index].target_addr = built_addr;
    zassert_equal(rt::force_rebuild_descriptors_for_test(), 0);
    rt::spec().at[position_index].target_addr = real_addr;
}

ZTEST(tof_cliff_runtime, test_a_refused_keying_is_never_observed_as_proven)
{
    zassert_equal(rt::bootstrap(kTiming), 0);
    make_the_descriptors_stale_at(3, 0x3D);

    const cm::outcome r{prove_over_the_fake_chain(7)};

    zassert_false(r.proven());
    zassert_equal(r.failed_at, cm::stage::commit_refused);
    zassert_equal(r.commit, au::commit_refusal::mapping_install_failed);
    zassert_not_equal(au::current().state, acq::mapping_state::proven,
                      "a failed keying still published PROVEN");
    zassert_equal(au::installed_mapping().positions, 0u);
    zassert_false(rt::mapping_applied());
    zassert_equal(rt::start_acquisition(), -EPERM);
}

ZTEST(tof_cliff_runtime, test_a_keying_that_fails_late_leaves_no_earlier_role_written)
{
    /* The two-pass rule. Position 6 is the last cliff descriptor, so a single-pass implementation
     * would have written positions 3, 4 and 5 before refusing -- a table where some corners are
     * keyed and the rest are not, with nothing on the wire to say which. */
    zassert_equal(rt::bootstrap(kTiming), 0);
    make_the_descriptors_stale_at(5, 0x3F);

    const acq::source_desc *d{rt::descriptors_for_test()};

    zassert_equal(prove_over_the_fake_chain(7).commit, au::commit_refusal::mapping_install_failed);

    for (int i{0}; i < 6; ++i)
        zassert_equal(d[i].role_id, rt::kRoleUnassigned,
                      "position %d kept a role from a keying that failed", i + 1);
}

ZTEST(tof_cliff_runtime, test_a_failed_re_proof_does_not_leave_the_old_mapping_startable)
{
    /* The stale-flag case, and the reason mapping_applied() asks the authority instead of
     * remembering. The first proof keys the descriptors and acquisition may start. A second proof
     * revokes on the way in and then fails -- so those keys describe a mapping that is no longer
     * proven, and a latched "applied" flag would still be saying otherwise. */
    zassert_equal(rt::bootstrap(kTiming), 0);
    zassert_true(prove_over_the_fake_chain(7).proven());
    zassert_true(rt::mapping_applied());
    zassert_equal(rt::start_acquisition(), 0);
    zassert_equal(acq::try_stop(), 0);

    make_the_descriptors_stale_at(3, 0x3D);
    zassert_equal(prove_over_the_fake_chain(8).commit, au::commit_refusal::mapping_install_failed);

    zassert_false(rt::mapping_applied(), "the previous mapping still counted as applied");
    zassert_equal(rt::start_acquisition(), -EPERM,
                  "acquisition started on keys from a revoked mapping");
}

/* ------------------------------------------- the ordering against the chain glue ---- */

ZTEST(tof_cliff_runtime, test_the_bootstrap_refuses_before_the_chain_glue_is_up)
{
    /* Acquisition drives i2c2 and the enable lines, so bootstrapping before the chain controller has
     * configured them would bring sensors up against unconfigured pins. That is exactly what the B6
     * image did, from a SYS_INIT that ran before main(): the ordering was expressed only by where
     * the calls happened to sit, and it was wrong. Now it is a precondition. */
    glue_is_ready = false;

    zassert_equal(rt::bootstrap(kTiming), -ENODEV);
    zassert_equal(rt::current_stage(), rt::stage::chain_not_ready);
    zassert_false(rt::ready());
    zassert_equal(rt::start_acquisition(), -EPERM);

    /* And it is not a permanent refusal: the same call succeeds once the glue is up, which is the
     * order production uses -- one call site, after the control lines. */
    rt::reset_for_test();
    glue_is_ready = true;
    zassert_equal(rt::bootstrap(kTiming), 0);
    zassert_true(rt::ready());
}

ZTEST(tof_cliff_runtime, test_a_re_proof_stops_the_thread_through_request_and_join)
{
    /* Rule 8: losing or re-proving a mapping goes down the same stop-and-join path as anything else.
     * The commissioning quiesce is tof_acq::try_stop, which with a thread running asks and waits
     * rather than stopping devices itself -- so the thread is what stops them, at a cycle boundary,
     * from the thread that owns them. */
    zassert_equal(rt::bootstrap(kTiming), 0);
    zassert_true(prove_over_the_fake_chain(7).proven());
    zassert_equal(rt::start_acquisition(), 0);
    k_msleep(kTiming.cycle_period_ms * 2);
    zassert_true(acq::thread_running());

    const uint32_t foreign_before{acq::foreign_lifecycle_calls()};

    /* A second proof. Its quiesce has to bring the thread down on its own terms. */
    zassert_true(prove_over_the_fake_chain(8).proven(), "the re-proof could not quiesce the thread");

    zassert_false(acq::thread_running(), "the thread survived a re-proof");
    zassert_equal(acq::foreign_lifecycle_calls(), foreign_before,
                  "something other than the thread drove the ULD during the re-proof");
    zassert_true(rt::mapping_applied());
    zassert_equal(au::current().epoch, 8);

    /* And the new epoch's first cycle starts from 0, because begin_epoch() ran inside that commit and
     * starting the thread again does not renumber. */
    zassert_equal(rt::start_acquisition(), 0);
    k_msleep(kTiming.cycle_period_ms * 2);
    zassert_equal(acq::try_stop(), 0);
}

ZTEST(tof_cliff_runtime, test_starting_twice_is_refused_rather_than_creating_a_second_thread)
{
    /* `tof cliff start` is a separate operator action from `prove`, so it can be issued twice. The
     * second one must be refused: a second thread on the same descriptors would put two callers
     * inside the ULD, whose port keeps one transport record. */
    zassert_equal(rt::bootstrap(kTiming), 0);
    zassert_true(prove_over_the_fake_chain(7).proven());
    zassert_equal(rt::start_acquisition(), 0);
    zassert_true(acq::thread_running());

    zassert_equal(rt::start_acquisition(), -EALREADY, "a second acquisition thread was created");
    zassert_true(acq::thread_running(), "the refused start disturbed the running thread");

    zassert_equal(acq::try_stop(), 0);
    /* And after a clean stop it can be started again -- the refusal is about concurrency, not a
     * one-shot latch. */
    zassert_equal(rt::start_acquisition(), 0);
    zassert_equal(acq::try_stop(), 0);
}



/* ==================================================================== the grid wiring ======
 *
 * Everything between a number in the devicetree and an 8x8 grid on the bus. The suite above owns
 * the bootstrap's shape; this one owns the path the L7s take through it, which did not exist
 * until the ops table, the mapping install and the fan-out arrived together.
 */

namespace {

/* A fingerprint the install would accept, so each test can break exactly one thing about it.
 * Built from the spec the descriptors were built from, which is what a committed proof hands
 * over. */
pf::fingerprint good_fingerprint()
{
    pf::fingerprint fp{};

    fp.positions = rt::spec().positions;
    for (size_t i{0}; i < fp.positions; ++i) {
        const enm::position_spec &ps{rt::spec().at[i]};

        fp.at[i].position = static_cast<uint8_t>(i + 1);
        fp.at[i].expected = ps.expected;
        fp.at[i].address = ps.target_addr;
        fp.at[i].source_id = ps.source_id;
        fp.at[i].role = ps.role;
        fp.at[i].verified = true;
    }
    return fp;
}

int grid_frames_for_source(uint8_t source_id)
{
    int n{0};
    for (int i = 0; i < bus_count; ++i)
        if (bus_frames[i].can_id == ids::TOF_GRID_DATA_ID && (bus_frames[i].data[1] >> 4) == source_id)
            ++n;
    return n;
}

int health_frames_for_source(uint8_t source_id)
{
    int n{0};
    for (int i = 0; i < bus_count; ++i)
        if (bus_frames[i].can_id == ids::TOF_GRID_HEALTH_ID &&
            (bus_frames[i].data[1] >> 4) == source_id)
            ++n;
    return n;
}

/* Bring the whole thing up and let it range. The cliff ops are -ENOSYS stubs in this binary, so
 * the four L4s fail to start and contribute nothing -- which is deliberate here: what reaches the
 * bus then comes from the grid path alone, and the cliff publisher's presence is still visible in
 * its own frames. */
void run_cycles(int cycles)
{
    zassert_equal(rt::bootstrap(kTiming), 0);
    const cm::outcome r{prove_over_the_fake_chain(7)};
    zassert_true(r.proven(), "the proof failed at stage %d", static_cast<int>(r.failed_at));
    bus_count = 0;
    zassert_equal(rt::start_acquisition(), 0);
    k_msleep(static_cast<int>(kTiming.cycle_period_ms) * cycles);
    zassert_equal(acq::try_stop(), 0, "the thread did not stop cleanly");
}

}  // namespace

ZTEST_SUITE(tof_grid_wiring, NULL, NULL, before, NULL, NULL);

/* ------------------------------------------------------- the frequency, end to end -------- */

ZTEST(tof_grid_wiring, test_the_configured_frequency_reaches_both_sensors)
{
    /* devicetree -> runtime config -> descriptor -> grid_ops.configure() -> ULD, and the number
     * is SEVEN so that an implementation quietly using the overlay's five would fail here. */
    zassert_equal(rt::bootstrap(kTiming), 0);

    const acq::source_desc *d{rt::descriptors_for_test()};

    zassert_equal(d[0].grid_frequency_hz, kGridHz, "descriptor 0 did not take the configuration");
    zassert_equal(d[1].grid_frequency_hz, kGridHz, "descriptor 1 did not take the configuration");

    zassert_true(prove_over_the_fake_chain(7).proven());
    zassert_equal(rt::start_acquisition(), 0);
    k_msleep(static_cast<int>(kTiming.cycle_period_ms));
    zassert_equal(acq::try_stop(), 0);

    zassert_equal(uld_frequency_calls, 2, "the ULD was configured %d times", uld_frequency_calls);
    zassert_equal(uld_last_frequency, kGridHz);
    zassert_equal(uld_frequency_device_count, 2);
    /* TWO OBJECTS. One configuration shared by both sensors would be one sensor: the object holds
     * the device address and the stream count. */
    zassert_not_equal(uld_frequency_devices[0], uld_frequency_devices[1],
                      "both sensors were configured through one ULD object");
}

ZTEST(tof_grid_wiring, test_a_frequency_the_uld_cannot_honour_is_refused_at_bootstrap)
{
    rt::config zero{kTiming};
    rt::config too_fast{kTiming};

    zero.grid_frequency_hz = 0;
    too_fast.grid_frequency_hz = 16;   // one past the ULD's 8x8 ceiling

    zassert_equal(rt::bootstrap(zero), -EINVAL, "a frequency nobody chose was accepted");
    zassert_equal(rt::current_stage(), rt::stage::not_started);
    zassert_equal(rt::bootstrap(too_fast), -EINVAL, "16 Hz was accepted at 8x8");
    zassert_equal(rt::current_stage(), rt::stage::not_started);
    /* And the boundary is where it says it is, rather than a range nobody stated. */
    rt::config ceiling{kTiming};
    ceiling.grid_frequency_hz = 15;
    zassert_equal(rt::bootstrap(ceiling), 0);
}

/* --------------------------------------------------------------- the mapping install ------ */

ZTEST(tof_grid_wiring, test_the_grid_source_comes_from_the_fingerprint_not_the_index)
{
    /* THE TEST THAT SEPARATES THE TWO. With the spec's source ids swapped, the fingerprint says
     * position 1 is source 1 and position 2 is source 0 -- the opposite of their indices. An
     * install that used the index would key them the other way round and publish the left sensor's
     * grid as the right one's, with every frame individually well formed. */
    zassert_equal(rt::bootstrap(kTiming), 0);
    rt::spec().at[0].source_id = 1;
    rt::spec().at[1].source_id = 0;

    zassert_true(prove_over_the_fake_chain(7).proven());

    const acq::source_desc *d{rt::descriptors_for_test()};

    zassert_equal(d[0].role_id, 1, "position 1 was keyed from its index, not the fingerprint");
    zassert_equal(d[1].role_id, 0);
}

ZTEST(tof_grid_wiring, test_the_swapped_mapping_reaches_the_wire)
{
    /* The same swap, followed all the way to the bytes: what the fingerprint said is what the
     * frames carry. Without this the keying could be right and the publishing wrong. */
    zassert_equal(rt::bootstrap(kTiming), 0);
    rt::spec().at[0].source_id = 1;
    rt::spec().at[1].source_id = 0;
    zassert_true(prove_over_the_fake_chain(7).proven());

    uld_ready_value = 1;
    bus_count = 0;
    zassert_equal(rt::start_acquisition(), 0);
    k_msleep(static_cast<int>(kTiming.cycle_period_ms) * 2);
    zassert_equal(acq::try_stop(), 0);

    zassert_true(grid_frames_for_source(0) > 0, "nothing was published as source 0");
    zassert_true(grid_frames_for_source(1) > 0, "nothing was published as source 1");
}

ZTEST(tof_grid_wiring, test_each_refusal_the_install_owes_the_contract)
{
    /* Straight at the install, because a real proof cannot produce most of these: the enumerator
     * validates the spec's source ids and the authority refuses a fingerprint that disagrees with
     * the spec. They are the last gate before a descriptor is keyed, and this is the only place
     * their shape can be seen. */
    zassert_equal(rt::bootstrap(kTiming), 0);

    zassert_equal(rt::install_from_mapping_for_test(good_fingerprint(), 7), 0,
                  "the unmodified fingerprint was refused");

    struct {
        const char *what;
        void (*break_it)(pf::fingerprint &);
    } cases[]{
        {"the entry belongs to another position",
         [](pf::fingerprint &fp) { fp.at[0].position = 3; }},
        {"an L4 at a grid position",
         [](pf::fingerprint &fp) { fp.at[0].expected = enm::model::l4cx; }},
        {"an address acquisition will not read",
         [](pf::fingerprint &fp) { fp.at[0].address = 0x3A; }},
        {"a position nothing verified",
         [](pf::fingerprint &fp) { fp.at[0].verified = false; }},
        {"no source id at all",
         [](pf::fingerprint &fp) { fp.at[0].source_id = -1; }},
        {"a source id the contract cannot express",
         [](pf::fingerprint &fp) { fp.at[0].source_id = 2; }},
        {"two positions claiming one source",
         [](pf::fingerprint &fp) { fp.at[1].source_id = fp.at[0].source_id; }},
    };

    for (const auto &c : cases) {
        pf::fingerprint fp{good_fingerprint()};

        c.break_it(fp);
        zassert_equal(rt::install_from_mapping_for_test(fp, 7), -EINVAL, "accepted: %s", c.what);
    }
}

ZTEST(tof_grid_wiring, test_a_grid_refusal_leaves_not_one_descriptor_keyed)
{
    /* The two-pass rule, from the grid end. Position 2 is the one that fails, so a single-pass
     * install would already have written position 1 -- one sensor publishing under a proven
     * source id while the other publishes nothing, and no frame able to say so. */
    zassert_equal(rt::bootstrap(kTiming), 0);
    zassert_equal(rt::install_from_mapping_for_test(good_fingerprint(), 7), 0);
    zassert_equal(rt::force_rebuild_descriptors_for_test(), 0);

    pf::fingerprint fp{good_fingerprint()};

    fp.at[1].verified = false;
    zassert_equal(rt::install_from_mapping_for_test(fp, 7), -EINVAL);

    const acq::source_desc *d{rt::descriptors_for_test()};

    for (int i{0}; i < 6; ++i)
        zassert_equal(d[i].role_id, rt::kRoleUnassigned,
                      "position %d kept a key from an install that refused", i + 1);
}

ZTEST(tof_grid_wiring, test_a_stale_grid_address_is_refused_by_a_real_proof)
{
    /* The one grid refusal a committed proof CAN reach: the descriptors were built from one spec
     * and the proof proves another, so the address the mapping proved is not the address
     * acquisition will read. */
    zassert_equal(rt::bootstrap(kTiming), 0);
    make_the_descriptors_stale_at(0, 0x3A);

    const cm::outcome r{prove_over_the_fake_chain(7)};

    zassert_false(r.proven());
    zassert_equal(r.commit, au::commit_refusal::mapping_install_failed);
    zassert_false(rt::mapping_applied());
}

/* ------------------------------------------------------- the publisher, and the fan-out --- */

ZTEST(tof_grid_wiring, test_acquisition_does_not_start_when_the_grid_publisher_does_not)
{
    /* A publisher that never initialised returns from every call. Acquisition would range, read
     * grids and hand them to nothing -- indistinguishable on the bus from two sensors that are
     * not there, which is the one failure mode this pair of identifiers cannot report. */
    grid_sink_broken = true;

    zassert_equal(rt::bootstrap(kTiming), -EINVAL);
    zassert_equal(rt::current_stage(), rt::stage::grid_publisher_failed);
    zassert_false(rt::ready());
    zassert_equal(rt::start_acquisition(), -EPERM);
    zassert_false(acq::thread_running());
}

ZTEST(tof_grid_wiring, test_a_ready_grid_reaches_the_bus_through_the_real_scheduler)
{
    uld_ready_value = 1;
    run_cycles(2);

    /* 16 data frames and one health frame per grid per source. */
    zassert_true(frames_with_id(ids::TOF_GRID_DATA_ID) >= 32,
                 "only %d data frames", frames_with_id(ids::TOF_GRID_DATA_ID));
    zassert_true(frames_with_id(ids::TOF_GRID_HEALTH_ID) >= 2);
    zassert_true(health_frames_for_source(0) > 0);
    zassert_true(health_frames_for_source(1) > 0);

    const sent_frame *health{first_with_id(ids::TOF_GRID_HEALTH_ID)};

    zassert_not_null(health);
    zassert_equal(health->data[2], 64, "a grid of 64 trusted zones reported %u", health->data[2]);
    zassert_equal(health->data[3], 0x00, "flags on a grid whose chain never failed");
    /* byte 4: boards_detected in the high nibble, the 0-based chain position in the low one. */
    zassert_equal(health->data[4] >> 4, 6, "the proven chain's length");
    zassert_equal(health->data[5], 0x00, "no error to report");
}

ZTEST(tof_grid_wiring, test_the_cliff_publisher_still_gets_the_cycle_it_used_to_own)
{
    /* THE FAN-OUT. Wiring the grid publisher into the two sink slots by replacement would have
     * left this suite green everywhere except here: the cliff publisher would simply stop being
     * told a cycle happened.
     *
     * The assertion is on the cliff publisher's COUNTERS and not on frames at 0x217, and that
     * distinction is the test. Its heartbeat runs on its own timer and keeps emitting health
     * frames on that identifier whether or not any cycle reaches it, so counting frames there
     * would pass against a fan-out that had dropped the cliff publisher entirely -- which is
     * exactly what a mutation of this wiring does. cycle_health_sent moves only for a cycle that
     * was both announced and completed, and suppressed_cycle_not_begun is what a completion
     * without an announcement leaves behind. */
    uld_ready_value = 1;
    run_cycles(2);

    struct pub::counters cliff{};
    struct gpub::counters grid{};

    pub::copy_counters(cliff);
    gpub::copy_counters(grid);

    zassert_true(grid.grids_sent > 0, "the grid publisher saw no cycle");
    zassert_true(cliff.cycle_health_sent > 0,
                 "the cliff publisher was not told a cycle happened (%u announced, %u suppressed)",
                 cliff.cycle_health_sent, cliff.suppressed_cycle_not_begun);
    zassert_equal(cliff.suppressed_cycle_not_begun, 0,
                  "a cycle completed at the cliff publisher that nobody had announced to it");
    zassert_equal(grid.suppressed_cycle_not_begun, 0,
                  "a cycle completed at the grid publisher that nobody had announced to it");
}

ZTEST(tof_grid_wiring, test_a_sensor_with_nothing_ready_is_not_a_sensor_that_failed)
{
    /* At 7 Hz against a 50 ms cycle, most cycles find nothing ready. That is the ordinary case:
     * the adapter returns success with a non-fresh sample and the scheduler records no outcome.
     *
     * The assertion is made on the WIRE rather than on a counter, because the observable that
     * matters is the health frame: had those quiet cycles been recorded as I/O failures, the
     * publisher would owe a recovered-transfer flag and the next grid would carry it. */
    uld_ready_value = 0;
    run_cycles(3);

    zassert_equal(frames_with_id(ids::TOF_GRID_DATA_ID), 0, "a quiet sensor published a grid");
    zassert_equal(frames_with_id(ids::TOF_GRID_HEALTH_ID), 0);

    uld_ready_value = 1;
    bus_count = 0;
    zassert_equal(rt::start_acquisition(), 0);
    k_msleep(static_cast<int>(kTiming.cycle_period_ms) * 2);
    zassert_equal(acq::try_stop(), 0);

    const sent_frame *health{first_with_id(ids::TOF_GRID_HEALTH_ID)};

    zassert_not_null(health, "nothing was published once a grid was ready");
    zassert_equal(health->data[3] & 0x03, 0x00,
                  "quiet cycles were reported as a recovered failure (flags %02x)",
                  health->data[3]);
}
