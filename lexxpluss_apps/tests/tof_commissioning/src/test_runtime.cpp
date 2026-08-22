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
constexpr rt::config kTiming{50, 20, 400, K_PRIO_PREEMPT(5)};

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
    /* The grid positions are stubs with no cliff source id, and they stay unassigned rather than
     * being given a plausible-looking number. */
    zassert_equal(d[0].role_id, rt::kRoleUnassigned);
    zassert_equal(d[1].role_id, rt::kRoleUnassigned);

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
