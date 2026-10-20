/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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
#include "tof_chain_spec.hpp"
#include "tof_cliff_runtime.hpp"
#include "tof_commissioning.hpp"
#include "tof_mapping_authority.hpp"
#if defined(ENABLE_TOF_L7_ULD)
#include "tof_can_ids.hpp"
#include "tof_grid_publisher.hpp"
#include "tof_l7_sample.hpp"
#endif

namespace acq = lexxhard::tof_acq;
namespace au = lexxhard::tof_authority;
namespace cm = lexxhard::tof_commissioning;
namespace enm = lexxhard::tof_enum;
namespace pf = lexxhard::tof_proof;
namespace pub = lexxhard::tof_cliff_pub;
namespace rt = lexxhard::tof_cliff_runtime;
#if defined(ENABLE_TOF_L7_ULD)
namespace gpub = lexxhard::tof_grid_pub;
namespace l7 = lexxhard::tof_l7;
#endif

namespace {

int can_init_rc{0};
int send_rc{0};
int frames_sent{0};
uint16_t last_can_id{0};

int fake_send(uint16_t can_id, const uint8_t *, uint8_t)
{
    ++frames_sent;
    last_can_id = can_id;
    return send_rc;
}

fake::fake_chain chain{};

#if defined(ENABLE_TOF_L7_ULD)
/* THE GRID FRAMES, COUNTED BY IDENTIFIER rather than collected. The question this suite asks of the
 * grid path is whether a mapping a proof installed reaches the bus at all, and the frames' contents
 * are the grid packer suite's subject -- it drives them from the contract's own vectors, which is a
 * stronger check than anything this file could restate. */
int grid_data_frames{0};
int grid_health_frames{0};
int grid_other_frames{0};

int grid_fake_send(uint16_t can_id, const uint8_t *, uint8_t)
{
    if (can_id == lexxhard::tof_can_ids::TOF_GRID_DATA_ID)
        ++grid_data_frames;
    else if (can_id == lexxhard::tof_can_ids::TOF_GRID_HEALTH_ID)
        ++grid_health_frames;
    else
        ++grid_other_frames;
    return 0;
}
#endif

}  // namespace

/* The CAN glue, stubbed at the boundary the runtime actually uses. Linking the real one would pull
 * in a Zephyr CAN controller; what production does with these three entry points is covered by the
 * publisher suite. production_authorisation() is reproduced faithfully -- ONE authority read, with
 * every field taken from that one snapshot -- because a permissive stub would make every gating
 * assertion in this file meaningless. */
namespace lexxhard::tof_cliff_can {

int init()
{
    return can_init_rc;
}

struct tof_cliff_pub::can_sink sink()
{
    return {fake_send};
}

#if defined(ENABLE_TOF_L7_ULD)
struct tof_grid_pub::can_sink grid_sink()
{
    return {grid_fake_send};
}

/* Reproduced from grid_authorisation_from() for the reason the comment above gives, and with its
 * one load-bearing detail kept: source_allowed comes from grid_source_mask, which is the
 * authority's statement about GRID sources. Taking it from enumerated_mask -- whose bits are
 * source_id_of(l4_role) -- would let a cliff sensor's permission admit a grid, and that is the
 * defect #118 fixed. A stub that permitted everything would also hide the whole keying question
 * this file now asks. */
struct tof_grid_pub::authorisation grid_production_authorisation()
{
    const au::snapshot now{au::current()};
    struct tof_grid_pub::authorisation a{};

    a.state = now.state;
    a.epoch = now.epoch;
    a.boards_detected = 0;
    for (int i{0}; i < tof_grid_pub::kGridSources; ++i)
        a.source_allowed[i] = (now.grid_source_mask & (1U << i)) != 0U;
    return a;
}
#endif

struct tof_cliff_pub::authorisation production_authorisation()
{
    const au::snapshot now{au::current()};
    struct tof_cliff_pub::authorisation a{};

    a.state = now.state;
    a.epoch = now.epoch;
    a.enumerated_mask = now.enumerated_mask;
    a.model_verified_mask = now.model_verified_mask;
    a.chain_flags = now.chain_flags;
    a.failing_position = now.failing_position;
    return a;
}

}  // namespace lexxhard::tof_cliff_can

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
/* The last two are the cliff ranging profile, which bootstrap() now requires. Deliberately not the
 * overlay's 15000/2: this suite is about the runtime's sequence, and values matching the deployment
 * would let a bootstrap that ignored its argument pass. */
/* The last three are the cliff ranging profile and the grid rate, all of which bootstrap() now
 * requires. Deliberately not the overlay's values: this suite is about the runtime's sequence, and
 * values matching the deployment would let a bootstrap that ignored its argument pass. */
constexpr rt::config kTiming{50, 20, 400, K_PRIO_PREEMPT(5), 21000, 3, 7};

/* The acquisition thread's stack. The runtime sizes its own from a devicetree property; there is no
 * devicetree here, and a fallback compiled into the module for tests would be a size nobody chose
 * that shipped anyway -- so the suite hands one over. */
K_THREAD_STACK_DEFINE(runtime_acq_stack, 2048);

/* This suite is about the runtime's sequence, not the bus. The transaction now requires the hook,
 * so it gets one that always succeeds; the dual-rate behaviour itself is covered next door, where
 * the fake chain records the order. */
int product_speed_rc{0};

int runtime_set_bus_speed(cm::bus_speed speed)
{
    if (speed != cm::bus_speed::product_400k) {
        chain.retimed = false;
        return 0;
    }
    if (product_speed_rc != 0)
        return product_speed_rc;
    /* Arms the fake's post-retime faults, so this suite can reach the identity re-check too. */
    chain.retimed = true;
    return 0;
}

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
    product_speed_rc = 0;
    send_rc = 0;
    frames_sent = 0;
    last_can_id = 0;
#if defined(ENABLE_TOF_L7_ULD)
    grid_data_frames = 0;
    grid_health_frames = 0;
    grid_other_frames = 0;
#endif
}

/* The roles used to be injected here, because dasher_spec() carried l4_role::unknown and PROVEN was
 * unreachable by rule. They are now frozen in the production spec itself (from
 * dasher_connectivity.png), so these cases exercise the REAL configuration -- which is strictly
 * better: a test that installs its own role table cannot notice the shipped one being wrong. */
cm::outcome prove_over_the_fake_chain(uint32_t epoch)
{
    cm::config ccfg{};

    ccfg.chain = &lexxhard::tof_chain_controller::chain_lock();
    ccfg.ops = &chain;
    ccfg.spec = &rt::spec();
    ccfg.quiesce = quiesce_via_acquisition;
    ccfg.set_bus_speed = runtime_set_bus_speed;
    zassert_equal(cm::init(ccfg), 0);
    return cm::prove(epoch);
}

}  // namespace

ZTEST_SUITE(tof_cliff_runtime, NULL, NULL, before, NULL, NULL);

/* THE MIDDLE OF THE CHAIN, which neither of the other two profile cases covers. The acquisition
 * suite fills descriptors itself and checks what reaches the ops table; its production-table case
 * calls configure() directly. Between them sits build_descriptors(), and nothing pinned that it
 * copies the configuration across unchanged -- a bootstrap that dropped the values on the floor, or
 * put them on the wrong sources, would leave both of those cases green.
 *
 * The values are deliberately neither the overlay's nor the ones kTiming uses elsewhere, so this
 * cannot pass against anything baked in. */
ZTEST(tof_cliff_runtime, test_bootstrap_copies_the_profile_onto_the_cliff_descriptors)
{
    rt::config c{kTiming};
    c.cliff_timing_budget_us = 27000;
    c.cliff_distance_mode = 2;

    zassert_equal(rt::bootstrap(c), 0);

    const acq::source_desc *d{rt::descriptors_for_test()};
    zassert_not_null(d);

    int cliff{0};
    int grid{0};
    for (size_t i = 0; i < lexxhard::tof_chain::dasher_spec().positions; ++i) {
        if (d[i].kind == acq::model::l4_cliff) {
            ++cliff;
            zassert_equal(d[i].cliff_timing_budget_us, 27000U,
                          "position %zu carries budget %u", i, d[i].cliff_timing_budget_us);
            zassert_equal(d[i].cliff_distance_mode, 2, "position %zu carries mode %u", i,
                          d[i].cliff_distance_mode);
        } else {
            ++grid;
            /* Zero on a grid descriptor, and acquisition refuses one that is not -- the grid path
             * never reads these, so a value carried there would range as if it were absent. */
            zassert_equal(d[i].cliff_timing_budget_us, 0U, "position %zu is a grid source", i);
            zassert_equal(d[i].cliff_distance_mode, 0, "position %zu is a grid source", i);
        }
    }
    zassert_equal(cliff, 4, "the spec has four cliff positions");
    zassert_equal(grid, 2, "and two grid positions");
}

/* THE RANGING PROFILE IS REFUSED HERE AS WELL AS IN tof_acq::init(), and the duplication is the
 * point rather than an oversight. This is the stage that knows it is reading a deployment's
 * devicetree; the same descriptor reaching acquisition is refused there as a generic -EINVAL with
 * nothing to say which of the two layers was misconfigured. */
/* THE GRID RATE IS REFUSED RATHER THAN CLAMPED. A rate silently moved to the nearest legal value
 * is a rate nobody chose, and the devicetree would stop being the place it is written down. 15 is
 * the ULD's ceiling at 8x8; zero is the absence of a choice. */
ZTEST(tof_cliff_runtime, test_a_bootstrap_without_a_grid_rate_is_refused)
{
    const uint8_t bad_rates[]{0, 16, 255};

    for (const uint8_t hz : bad_rates) {
        rt::config bad{kTiming};
        bad.grid_frequency_hz = hz;
        zassert_equal(rt::bootstrap(bad), -EINVAL, "grid-frequency-hz %u", hz);
    }

    rt::config edge{kTiming};
    edge.grid_frequency_hz = 15;
    zassert_equal(rt::bootstrap(edge), 0, "15 is the ceiling and is legal");
}

ZTEST(tof_cliff_runtime, test_a_bootstrap_without_a_ranging_profile_is_refused)
{
    rt::config no_budget{kTiming};
    no_budget.cliff_timing_budget_us = 0;
    zassert_equal(rt::bootstrap(no_budget), -EINVAL, "a zero budget must not be defaulted");

    const struct {
        uint8_t mode;
        int expected;
    } modes[]{
        {0, -EINVAL},   /* absent */
        {1, -EINVAL},   /* SHORT: the ULD refuses it for an L4 part */
        {4, -EINVAL},   /* beyond the enumeration */
    };

    for (const auto &k : modes) {
        rt::config bad{kTiming};
        bad.cliff_distance_mode = k.mode;
        zassert_equal(rt::bootstrap(bad), k.expected, "distance mode %u", k.mode);
    }

    /* The control: nothing above left the runtime bootstrapped, and a valid profile still works. */
    zassert_equal(rt::bootstrap(kTiming), 0, "a valid profile must still bootstrap");
}

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
    zassert_equal(rt::bootstrap(rt::config{0, 20}), -EINVAL);
    zassert_equal(rt::current_stage(), rt::stage::not_started,
                  "a refused config still wired something up");
    zassert_equal(rt::bootstrap(rt::config{50, 0}), -EINVAL);
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
    d.ops = &acq::l7_stub_ops();
    c.sources = &d;
    c.source_count = 1;
    c.periods.cycle_period_ms = 50;
    c.periods.health_period_ms = 20;
    c.hooks.on_cycle_begin = pub::on_cycle_begin;
    c.hooks.on_cycle = pub::on_cycle_complete;
    c.hooks.on_cliff_sample = pub::on_cliff_sample;
    c.hooks.on_cliff_health = pub::on_cliff_health;
    c.mapping_state_provider = au::state_provider;
    c.now_ms = k_uptime_get_32;
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
#if defined(ENABLE_TOF_L7_ULD)
    /* WITH THE GRID DRIVER IN THE IMAGE the two grid positions are keyed from the fingerprint's own
     * source_id field, which the spec assigns and the proof verifies. Unassigned here is what kept
     * every grid sample off the bus until 2026-10-20: the publisher refuses a sample whose source
     * id was never keyed, and refusing it invalidates the cycle. */
    zassert_equal(d[0].role_id, 0, "position 1 (index 0) must key to grid source 0");
    zassert_equal(d[1].role_id, 1, "position 2 (index 1) must key to grid source 1");
#else
    /* The grid positions are stubs with no cliff source id, and they stay unassigned rather than
     * being given a plausible-looking number. A stub cannot produce a sample, so there is nothing
     * to publish and nothing to key. */
    zassert_equal(d[0].role_id, rt::kRoleUnassigned);
    zassert_equal(d[1].role_id, rt::kRoleUnassigned);
#endif

    /* Only now may the acquisition thread start -- and from here on it is the only thing allowed to
     * touch a sensor. */
    zassert_equal(rt::start_acquisition(), 0);
    zassert_true(acq::thread_running());
    k_msleep(kTiming.cycle_period_ms * 2);
    zassert_equal(acq::try_stop(), 0, "the thread did not stop cleanly");
}

/* A RE-CHECK THAT FAILED AT A LATER POSITION MUST NOT LEAVE A STARTABLE MACHINE EITHER. Position 4,
 * deliberately: a transaction that gave up after the first position would never reach it, and every
 * "nothing was committed" assertion would still pass. */
ZTEST(tof_cliff_runtime, test_a_recheck_failure_at_a_later_position_leaves_nothing_startable)
{
    zassert_equal(rt::bootstrap(kTiming), 0);

    chain.fault_position = 4;
    chain.wrong_id_after_retime = true;

    const cm::outcome r{prove_over_the_fake_chain(11)};

    zassert_equal(r.failed_at, cm::stage::identity_recheck_failed, "stage %d",
                  static_cast<int>(r.failed_at));
    zassert_equal(r.recheck.position, 4, "it stopped at position %u", r.recheck.position);
    zassert_equal(r.recheck.fault, cm::recheck_fault::wrong_identity, "");

    zassert_false(rt::mapping_applied(), "no mapping may be installed");
    zassert_not_equal(au::current().state, acq::mapping_state::proven);
    zassert_equal(rt::start_acquisition(), -EPERM, "and acquisition must refuse to start");
    zassert_false(acq::thread_running(), "");
    zassert_true(r.restore_attempted, "");

    /* The control: with the fault cleared the same sequence proves and starts. */
    chain = fake::fake_chain{};
    const cm::outcome good{prove_over_the_fake_chain(12)};
    zassert_true(good.proven(), "stage %d", static_cast<int>(good.failed_at));
    zassert_equal(rt::start_acquisition(), 0);
    zassert_equal(acq::try_stop(), 0);
}

/* A RETIME THAT FAILED MUST NOT LEAVE A STARTABLE MACHINE, and that is the half the commissioning
 * suite cannot show: it checks nothing was installed, and this checks what that means one layer up.
 * The evidence was gathered at 100 kHz and nothing says the chain answers at the speed acquisition
 * reads it at, so the descriptors are unkeyed, the authority is not PROVEN, and start_acquisition()
 * refuses -- rather than ranging against a mapping nobody confirmed. */
ZTEST(tof_cliff_runtime, test_a_failed_retime_leaves_nothing_startable)
{
    zassert_equal(rt::bootstrap(kTiming), 0);
    zassert_false(rt::mapping_applied());

    product_speed_rc = -EIO;
    const cm::outcome r{prove_over_the_fake_chain(9)};

    zassert_equal(r.failed_at, cm::stage::product_speed_refused, "stage %d",
                  static_cast<int>(r.failed_at));
    zassert_false(rt::mapping_applied(), "no mapping may be installed");
    zassert_not_equal(au::current().state, acq::mapping_state::proven);
    zassert_equal(rt::start_acquisition(), -EPERM, "and acquisition must refuse to start");
    zassert_false(acq::thread_running(), "");

    /* The bus was put back, so the next proof starts from a known speed rather than inheriting
     * this failure. */
    zassert_true(r.restore_attempted, "");
    zassert_equal(r.final_bus, cm::bus_state::proof_100k, "");

    /* And the control: with the fault cleared the same sequence proves and starts. */
    product_speed_rc = 0;
    const cm::outcome good{prove_over_the_fake_chain(10)};
    zassert_true(good.proven(), "stage %d", static_cast<int>(good.failed_at));
    zassert_equal(rt::start_acquisition(), 0);
    zassert_equal(acq::try_stop(), 0);
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

#if defined(ENABLE_TOF_L7_ULD)
/* ------------------------------------------------ the grid keying, in a build that has L7 ------ */

/* THIS CONFIGURATION EXISTS BECAUSE THE OTHER ONE CANNOT ASK THE QUESTION. Without
 * ENABLE_TOF_L7_ULD every grid position carries the stub, and the suite's own end-to-end case
 * asserted the two of them stay unassigned -- which was correct for a stub and silently stood in
 * for the real wiring. The real build keys them from the fingerprint, and on 2026-10-19 DS20002
 * showed what the gap costs: a committed epoch, four cliff sensors publishing, and not one grid
 * frame on 0x214 because every sample was refused as suppressed_role_unassigned.
 *
 * The checks below are reached by handing install_from_mapping() a fingerprint directly. A proof
 * cannot stage them: the fingerprint a walk produces is built from the spec, so an out-of-range
 * source id, a neighbour's position number or two positions claiming one id are not reachable
 * through commissioning. They guard against a fingerprint that disagrees with the descriptors,
 * which is a wiring fault rather than a chain fault, and the only way to pin that is to build one.
 */
pf::fingerprint profile_fingerprint()
{
    const enm::chain_spec &spec{rt::spec()};
    pf::fingerprint fp{};

    fp.positions = spec.positions;
    for (size_t i{0}; i < spec.positions; ++i) {
        fp.at[i].position = static_cast<uint8_t>(i + 1);   // 1-based, as the operator counts them
        fp.at[i].expected = spec.at[i].expected;
        fp.at[i].address = spec.at[i].target_addr;
        fp.at[i].source_id = spec.at[i].source_id;
        fp.at[i].role = spec.at[i].role;
        fp.at[i].verified = true;
    }
    return fp;
}

/* A bootstrap with descriptors built and nothing keyed: the state every refusal case has to be
 * found in afterwards. */
void bootstrap_unkeyed()
{
    zassert_equal(rt::bootstrap(kTiming), 0);
    const acq::source_desc *d{rt::descriptors_for_test()};

    zassert_equal(d[0].role_id, rt::kRoleUnassigned, "the fixture started out keyed");
    zassert_equal(d[1].role_id, rt::kRoleUnassigned, "the fixture started out keyed");
}

void assert_nothing_was_keyed(const char *what)
{
    const acq::source_desc *d{rt::descriptors_for_test()};

    for (size_t i{0}; i < rt::spec().positions; ++i)
        zassert_equal(d[i].role_id, rt::kRoleUnassigned,
                      "%s: position %u was keyed by a refused mapping", what,
                      static_cast<unsigned>(i + 1));
    zassert_false(rt::mapping_applied(), "%s: the mapping counts as applied", what);
    zassert_equal(rt::start_acquisition(), -EPERM, "%s: acquisition would start", what);
}

ZTEST(tof_cliff_runtime, test_an_installed_mapping_keys_both_grid_positions_from_the_fingerprint)
{
    bootstrap_unkeyed();
    zassert_equal(rt::install_from_mapping_for_test(profile_fingerprint(), 5), 0);

    const acq::source_desc *d{rt::descriptors_for_test()};

    /* THE SOURCE IDS THE SPEC ASSIGNS, not the descriptor indices they coincide with today. The
     * values are equal here by construction of dasher_spec; what is being pinned is where they
     * came from, which is why the two cliff assertions are beside them -- those are keyed from the
     * role instead, through a different table. */
    zassert_equal(d[0].role_id, 0);
    zassert_equal(d[1].role_id, 1);
    zassert_equal(d[2].role_id, 0, "the cliff roles are keyed through source_id_of(), unchanged");
    zassert_equal(d[5].role_id, 3);
}

ZTEST(tof_cliff_runtime, test_a_grid_source_id_outside_the_contract_is_refused)
{
    bootstrap_unkeyed();

    pf::fingerprint fp{profile_fingerprint()};

    // Two grid sources exist. A third has no identifier on the wire to be published under.
    fp.at[1].source_id = 2;
    zassert_equal(rt::install_from_mapping_for_test(fp, 5), -EINVAL);
    assert_nothing_was_keyed("source id 2");

    fp.at[1].source_id = -1;
    zassert_equal(rt::install_from_mapping_for_test(fp, 5), -EINVAL);
    assert_nothing_was_keyed("no source id");
}

ZTEST(tof_cliff_runtime, test_two_grid_positions_claiming_one_source_id_are_refused)
{
    bootstrap_unkeyed();

    pf::fingerprint fp{profile_fingerprint()};

    /* Both sides of the machine published as one source: the publisher would drop the cycle as a
     * duplicate, and nothing downstream could tell which of the two was mis-keyed. */
    fp.at[1].source_id = 0;
    zassert_equal(rt::install_from_mapping_for_test(fp, 5), -EINVAL);
    assert_nothing_was_keyed("duplicate source id");
}

ZTEST(tof_cliff_runtime, test_a_grid_position_the_fingerprint_disagrees_about_is_refused)
{
    bootstrap_unkeyed();

    /* Four separate disagreements, each refused on its own. They are in one case because they
     * share the fixture and the same consequence, not because any one of them stands in for the
     * others. */
    pf::fingerprint moved{profile_fingerprint()};

    moved.at[0].address = 0x2B;   // the entry describes the device at the NEXT position
    zassert_equal(rt::install_from_mapping_for_test(moved, 5), -EINVAL);
    assert_nothing_was_keyed("address moved");

    pf::fingerprint wrong_model{profile_fingerprint()};

    wrong_model.at[0].expected = enm::model::l4cx;   // an 8x8 claim about a point sensor
    zassert_equal(rt::install_from_mapping_for_test(wrong_model, 5), -EINVAL);
    assert_nothing_was_keyed("wrong model");

    pf::fingerprint unverified{profile_fingerprint()};

    unverified.at[1].verified = false;
    zassert_equal(rt::install_from_mapping_for_test(unverified, 5), -EINVAL);
    assert_nothing_was_keyed("unverified position");

    pf::fingerprint neighbour{profile_fingerprint()};

    neighbour.at[0].position = 2;   // position 2's entry, offered for position 1
    zassert_equal(rt::install_from_mapping_for_test(neighbour, 5), -EINVAL);
    assert_nothing_was_keyed("a neighbour's position number");
}

ZTEST(tof_cliff_runtime, test_a_refusal_at_the_second_grid_position_leaves_the_first_unkeyed)
{
    /* THE TWO-PASS DISCIPLINE, which every case above depends on but none of them isolates: the
     * first grid position validates cleanly and only the second is refused. A single-pass install
     * would have written source 0 before it reached the refusal, leaving one side of the machine
     * publishing under a mapping the commit rejected -- and nothing on the wire to say so. */
    bootstrap_unkeyed();

    pf::fingerprint fp{profile_fingerprint()};

    fp.at[1].verified = false;
    zassert_equal(rt::install_from_mapping_for_test(fp, 5), -EINVAL);

    const acq::source_desc *d{rt::descriptors_for_test()};

    zassert_equal(d[0].role_id, rt::kRoleUnassigned,
                  "the first grid position was keyed before the second was refused");
}

ZTEST(tof_cliff_runtime, test_a_proven_mapping_puts_both_grids_on_the_bus)
{
    /* THE WHOLE CHAIN, because keyed descriptors are not frames. A proof over the fake chain
     * commits, the authority publishes its grid permission, the runtime keys the descriptors, and
     * the publisher the bootstrap already wired is handed one sample per grid source. Everything
     * between is production code; only the bus and the vendor are stubbed.
     *
     * This is the case that would have failed before the keying landed, and it fails for the right
     * reason: suppressed_role_unassigned counts, and no frame is sent. */
    zassert_equal(rt::bootstrap(kTiming), 0);

    const cm::outcome r{prove_over_the_fake_chain(9)};

    zassert_true(r.proven(), "the proof failed at stage %d", static_cast<int>(r.failed_at));
    zassert_equal(au::current().grid_source_mask, 0x3,
                  "the authority did not permit both grid sources");

    const acq::source_desc *d{rt::descriptors_for_test()};
    acq::cycle_facts facts{};

    facts.cycle_seq = 1;
    facts.source_count = static_cast<int>(rt::spec().positions);
    for (size_t i{0}; i < rt::spec().positions; ++i) {
        facts.sources[i].kind = d[i].kind;
        facts.sources[i].addr_7bit = d[i].addr_7bit;
        facts.sources[i].role_id = d[i].role_id;
        facts.sources[i].configured = true;
        facts.sources[i].started = true;
    }

    l7::sample s{};

    s.fresh = true;
    for (size_t z{0}; z < l7::kZoneCount; ++z) {
        s.target_status[z] = 5;   // a valid range by the contract's status table
        s.target_count[z] = 1;
        s.distance_mm[z] = 1000;
    }

    gpub::on_cycle_begin(facts.cycle_seq);
    for (int i{0}; i < gpub::kGridSources; ++i) {
        facts.sources[i].sample_produced = true;
        facts.sources[i].status.domain = acq::status_domain::l7;
        gpub::on_grid_sample(i, facts.cycle_seq, facts.sources[i], s);
    }
    gpub::on_cycle_complete(facts);

    gpub::counters c{};

    gpub::copy_counters(c);
    zassert_equal(c.suppressed_role_unassigned, 0U,
                  "a keyed grid source was still refused for having no source id");
    zassert_equal(c.grids_admitted, 2U, "both grids must be admitted");
    zassert_equal(c.grids_sent, 2U, "both grids must reach the bus");
    zassert_equal(grid_health_frames, 2, "one health frame closes each grid");
    zassert_true(grid_data_frames > 0, "no grid data frame was sent");
    zassert_equal(grid_other_frames, 0, "a grid frame went out under another identifier");
}
#endif  // ENABLE_TOF_L7_ULD
