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
#include "tof_commissioning.hpp"
#include "tof_mapping_authority.hpp"

namespace acq = lexxhard::tof_acq;
namespace au = lexxhard::tof_authority;
namespace cm = lexxhard::tof_commissioning;
namespace enm = lexxhard::tof_enum;
namespace pf = lexxhard::tof_proof;
namespace pub = lexxhard::tof_cliff_pub;
namespace rt = lexxhard::tof_cliff_runtime;

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

}  // namespace

/* The CAN glue, stubbed at the boundary the runtime actually uses. Linking the real one would pull
 * in a Zephyr CAN controller; what production does with these three entry points is covered by the
 * publisher suite. production_authorisation() is reproduced faithfully -- ONE authority read with
 * the acquisition clamp applied -- because a permissive stub would make every gating assertion in
 * this file meaningless. */
namespace lexxhard::tof_cliff_can {

int init()
{
    return can_init_rc;
}

struct tof_cliff_pub::can_sink sink()
{
    return {fake_send};
}

struct tof_cliff_pub::authorisation production_authorisation()
{
    const au::snapshot now{au::current()};
    struct tof_cliff_pub::authorisation a{};

    a.state = acq::clamp_mapping_state(now.state);
    a.epoch = now.epoch;
    a.enumerated_mask = now.enumerated_mask;
    a.model_verified_mask = now.model_verified_mask;
    a.chain_flags = now.chain_flags;
    a.failing_position = now.failing_position;
    return a;
}

}  // namespace lexxhard::tof_cliff_can

namespace {

constexpr rt::config kTiming{50, 20};   // injected, as production injects it from the devicetree

int quiesce_via_acquisition()
{
    return acq::try_stop();
}

void before(void *)
{
    rt::reset_for_test();
    au::reset_epoch_history_for_test();
    chain = fake::fake_chain{};
    can_init_rc = 0;
    send_rc = 0;
    frames_sent = 0;
    last_can_id = 0;
}

/* Freezes the four cliff roles in THE spec -- the one object the authority compares against and
 * commissioning walks. This is what the pos3-6 role table will look like once it is frozen for
 * real; until then dasher_spec() carries `unknown` and PROVEN is unreachable by rule. */
void freeze_the_roles()
{
    enm::chain_spec &s{rt::spec()};

    s.at[2].role = enm::l4_role::front_left;
    s.at[3].role = enm::l4_role::rear_left;
    s.at[4].role = enm::l4_role::rear_right;
    s.at[5].role = enm::l4_role::front_right;
}

cm::outcome prove_over_the_fake_chain(uint32_t epoch)
{
    cm::config ccfg{};

    ccfg.chain = &lexxhard::tof_chain_controller::chain_lock();
    ccfg.ops = &chain;
    ccfg.spec = &rt::spec();
    ccfg.quiesce = quiesce_via_acquisition;
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
    freeze_the_roles();

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

ZTEST(tof_cliff_runtime, test_nothing_is_keyed_or_started_before_a_proof)
{
    zassert_equal(rt::bootstrap(kTiming), 0);

    zassert_equal(rt::apply_installed_mapping(), -EPERM,
                  "descriptors were keyed without a proven mapping");
    zassert_false(rt::mapping_applied());
    zassert_equal(rt::start_acquisition(), -EPERM,
                  "acquisition started with unkeyed descriptors");
}

ZTEST(tof_cliff_runtime, test_the_gates_refuse_before_the_bootstrap_has_run)
{
    zassert_equal(rt::current_stage(), rt::stage::not_started);
    zassert_equal(rt::apply_installed_mapping(), -EPERM);
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
    freeze_the_roles();

    const acq::source_desc *d{rt::descriptors_for_test()};

    zassert_equal(d[0].addr_7bit, 0x2A);
    zassert_equal(d[2].addr_7bit, 0x2C);
    zassert_equal(d[5].addr_7bit, 0x2F);
    zassert_equal(d[0].kind, acq::model::l7_grid);
    zassert_equal(d[2].kind, acq::model::l4_cliff);
    for (int i{0}; i < 6; ++i)
        zassert_equal(d[i].role_id, rt::kRoleUnassigned, "position %d was keyed early", i + 1);

    const cm::outcome r{prove_over_the_fake_chain(7)};
    zassert_true(r.proven(), "the proof failed at stage %d", static_cast<int>(r.failed_at));
    zassert_equal(au::current().state, acq::mapping_state::proven);

    zassert_equal(rt::apply_installed_mapping(), 0);
    zassert_true(rt::mapping_applied());

    zassert_equal(d[2].role_id, 0, "position 3 (index 2) must key to front_left = source 0");
    zassert_equal(d[3].role_id, 1);
    zassert_equal(d[4].role_id, 2);
    zassert_equal(d[5].role_id, 3);
    /* The grid positions are stubs with no cliff source id, and they stay unassigned rather than
     * being given a plausible-looking number. */
    zassert_equal(d[0].role_id, rt::kRoleUnassigned);
    zassert_equal(d[1].role_id, rt::kRoleUnassigned);

    /* Only now may the sensors be brought up. */
    zassert_equal(rt::start_acquisition(), 0);
}

ZTEST(tof_cliff_runtime, test_a_mapping_for_other_addresses_is_refused)
{
    /* The check that makes keying safe. If the proven mapping puts a role at an address that is not
     * the one this descriptor will be read at, then keying it would publish one corner's distance
     * under another corner's name -- and nothing on the wire would look wrong. */
    zassert_equal(rt::bootstrap(kTiming), 0);
    freeze_the_roles();
    zassert_true(prove_over_the_fake_chain(8).proven());

    /* Move the descriptor's address out from under the installed mapping, the way a spec edit
     * between a proof and a restart would. */
    rt::spec().at[3].target_addr = 0x3D;
    zassert_equal(rt::force_rebuild_descriptors_for_test(), 0);

    zassert_equal(rt::apply_installed_mapping(), -EINVAL);
    zassert_false(rt::mapping_applied());
    zassert_equal(rt::start_acquisition(), -EPERM);
}
