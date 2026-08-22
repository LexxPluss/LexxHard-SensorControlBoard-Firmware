/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side tests for the commissioning orchestration.
 *
 * The chain mutex is a REAL k_mutex and the authority, proof, enumerator and isolation are the real
 * components. Only the bus and the quiesce hook are faked. That is deliberate: everything this
 * orchestration is responsible for -- the order of the steps, recursive locking, which exit paths
 * release the chain, what is left open when a step fails -- is a property of how the real pieces
 * interact, and faking any of them would assume the answer.
 *
 * A second thread is used to prove the lock is actually released, because "we called unlock" and "the
 * chain is available again" are different claims and only the second one matters.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "tof_acquisition.hpp"
#include "tof_chain_controller.hpp"
#include "fake_chain.hpp"
#include "tof_chain_spec.hpp"
#include "tof_commissioning.hpp"
#include "tof_tail_isolation.hpp"

namespace enm = lexxhard::tof_enum;
namespace pf = lexxhard::tof_proof;
namespace au = lexxhard::tof_authority;
namespace acq = lexxhard::tof_acq;
namespace cm = lexxhard::tof_commissioning;

namespace {


K_MUTEX_DEFINE(chain_mutex);

}  // namespace

/* The acquisition layer reaches for the chain through this, so it has to be the SAME mutex the
 * session takes -- the whole question being tested is what happens when the two contend. The chain
 * controller itself is not linked here: it drags in the bus and the shell, and the mutex is the only
 * part of it this suite is about. */
namespace lexxhard::tof_chain_controller {
k_mutex &chain_lock()
{
    return chain_mutex;
}
}  // namespace lexxhard::tof_chain_controller

/* tof_acquisition.cpp binds l4_cliff_ops() to these, so linking it needs them -- and NOTHING in this
 * suite may call them: the sources are configured with the fake ops below. Stubbed rather than
 * satisfied by linking the real sensor layer, which would pull the vendor ULD and an emulated I2C
 * controller into a suite about lock ordering. -ENOSYS is the fail-loud direction: if a test ever
 * does reach the real ops table, it gets an error rather than a plausible-looking sample. */
extern "C" {
int tof_cliff_sensor_open(VL53L4CX_Object_t *, uint8_t, struct tof_cliff_read_status *)
{
    return -ENOSYS;
}
int tof_cliff_sensor_configure(VL53L4CX_Object_t *, VL53LX_DistanceModes, uint32_t,
                               struct tof_cliff_read_status *)
{
    return -ENOSYS;
}
int tof_cliff_sensor_start(VL53L4CX_Object_t *, struct tof_cliff_stream_state *,
                           struct tof_cliff_read_status *)
{
    return -ENOSYS;
}
int tof_cliff_sensor_stop(VL53L4CX_Object_t *, struct tof_cliff_read_status *)
{
    return -ENOSYS;
}
int tof_cliff_read_once(VL53L4CX_Object_t *, struct tof_cliff_scratch *,
                        struct tof_cliff_stream_state *, struct tof_cliff_sample *,
                        struct tof_cliff_read_status *)
{
    return -ENOSYS;
}
/* Reached for real: acquisition logs the stage name when a source fails to come up. */
const char *tof_cliff_stage_name(enum tof_cliff_stage)
{
    return "stub";
}
}  // extern "C"

namespace {

/* The chain as the hardware behaves, enough of it for a full enumeration plus an isolation: a shift
 * register of enables, one device per stage, each with an address that a disabled L4 loses. */

using fake::fake_chain;
using fake::provable_spec;

/* Module-level, because the authority and the orchestration both keep pointers. */
enm::chain_spec runtime_spec{};
fake_chain chain{};
int quiesce_rc{0};
int quiesce_calls{0};
bool acquisition_idle{true};

int fake_quiesce()
{
    ++quiesce_calls;
    return quiesce_rc;
}

bool fake_is_idle()
{
    return acquisition_idle;
}

int fake_begin_epoch()
{
    return 0;
}

int install_rc{0};
int install_calls{0};

int fake_install(const pf::fingerprint &, uint8_t)
{
    ++install_calls;
    return install_rc;
}

void arrange(const enm::chain_spec &spec)
{
    runtime_spec = spec;
    chain = fake_chain{};
    quiesce_rc = 0;
    quiesce_calls = 0;
    acquisition_idle = true;
    install_rc = 0;
    install_calls = 0;

    au::config acfg{};
    acfg.runtime_spec = &runtime_spec;
    acfg.begin_epoch = fake_begin_epoch;
    acfg.acquisition_idle = fake_is_idle;
    acfg.install_mapping = fake_install;
    zassert_equal(au::init(acfg), 0);
    au::reset_epoch_history_for_test();

    cm::config ccfg{};
    ccfg.chain = &chain_mutex;
    ccfg.ops = &chain;
    ccfg.spec = &runtime_spec;
    ccfg.quiesce = fake_quiesce;
    zassert_equal(cm::init(ccfg), 0);
}

/* Holds the chain from another thread for as long as the flag says. Same-thread holding cannot
 * express "busy": the mutex is recursive for its owner, so the session's K_NO_WAIT would succeed --
 * which the first version of the busy test discovered by passing when it should not have. */
K_THREAD_STACK_DEFINE(holder_stack, 2048);
k_thread holder_thread;
K_SEM_DEFINE(holder_has_it, 0, 1);
K_SEM_DEFINE(holder_may_release, 0, 1);

void holder_entry(void *, void *, void *)
{
    if (k_mutex_lock(&chain_mutex, K_MSEC(500)) != 0)
        return;
    k_sem_give(&holder_has_it);
    /* Bounded, not K_FOREVER. A quiesce that waits for the chain would otherwise hang the whole
     * suite with no output; with a cap it returns late instead, and the elapsed-time assertion in
     * the busy test reports what actually went wrong. */
    (void)k_sem_take(&holder_may_release, K_MSEC(3000));
    k_mutex_unlock(&chain_mutex);
}

void hold_the_chain_elsewhere()
{
    k_sem_reset(&holder_has_it);
    k_sem_reset(&holder_may_release);
    k_thread_create(&holder_thread, holder_stack, K_THREAD_STACK_SIZEOF(holder_stack), holder_entry,
                    nullptr, nullptr, nullptr, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
    zassert_equal(k_sem_take(&holder_has_it, K_MSEC(500)), 0, "the holder thread never got the lock");
}

void release_the_chain_elsewhere()
{
    k_sem_give(&holder_may_release);
    (void)k_thread_join(&holder_thread, K_MSEC(500));
}

/* Proves the chain is available, from another thread -- the only way to distinguish "we called
 * unlock" from "the chain can be taken again". A recursive mutex makes the same check from this
 * thread succeed even when the lock is still held. */
K_THREAD_STACK_DEFINE(probe_stack, 2048);
k_thread probe_thread;
volatile bool probe_got_lock{false};

void probe_entry(void *, void *, void *)
{
    probe_got_lock = k_mutex_lock(&chain_mutex, K_MSEC(200)) == 0;
    if (probe_got_lock)
        k_mutex_unlock(&chain_mutex);
}

bool another_thread_can_take_the_chain()
{
    probe_got_lock = false;
    k_thread_create(&probe_thread, probe_stack, K_THREAD_STACK_SIZEOF(probe_stack), probe_entry,
                    nullptr, nullptr, nullptr, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
    (void)k_thread_join(&probe_thread, K_MSEC(500));
    return probe_got_lock;
}


/* ------------------------------------------- the REAL acquisition quiesce ----------- */

/* The production quiesce is tof_acq::try_stop, and the reason it exists is that the previous one
 * blocked: the shell called tof_acq::stop(), which takes the chain with K_FOREVER, so a run that
 * collided with a live chain user would hang one step BEFORE the session's K_NO_WAIT acquire. A fake
 * quiesce cannot show that -- an int-returning hook is non-blocking by construction -- so these
 * cases drive the real acquisition layer with fake devices and a holder thread. */

struct fake_source {
    struct tof_cliff_stream_state stream{}; // per source, never shared
    uint32_t read_delay_ms{0};   // a sensor that blocks: what a bounded join has to give up on
    int stop_calls{0};
    int start_calls{0};
    int pulses_at_stop{-1};
    struct tof_cliff_scratch scratch{};   // never used by the fake ops; init() requires one
};

uint32_t fake_now_ms()
{
    return static_cast<uint32_t>(k_uptime_get_32());
}

acq::mapping_state reported_state()
{
    /* Straight from the authority, as production wires it. The clamp inside acquisition is what
     * turns a PROVEN mapping into NOT_READY on the wire; nothing here needs to care, because these
     * cases are about the chain, not the frame. */
    return au::current().state;
}

fake_source sources[2];

int src_open(void *, uint8_t, acq::op_status *st)
{
    *st = acq::op_status{};
    return 0;
}
int src_configure(void *, acq::op_status *st)
{
    *st = acq::op_status{};
    return 0;
}
int src_start(void *dev, void *, acq::op_status *st)
{
    *st = acq::op_status{};
    ++static_cast<fake_source *>(dev)->start_calls;
    return 0;
}
int src_read(void *dev, void *, void *, struct tof_cliff_sample *out, acq::op_status *st)
{
    auto &d{*static_cast<fake_source *>(dev)};

    *st = acq::op_status{};
    *out = tof_cliff_sample{};
    if (d.read_delay_ms != 0)
        k_msleep(d.read_delay_ms);
    return 0;
}
int src_stop(void *dev, acq::op_status *st)
{
    *st = acq::op_status{};
    auto &d{*static_cast<fake_source *>(dev)};
    ++d.stop_calls;
    /* When the stop happened, measured in the only clock this suite has: how far the enumeration had
     * got. Zero means the quiesce finished before the first walk touched the chain, which is the
     * ordering the whole session depends on -- enumeration drops enable lines, and a device still
     * ranging through that is reading parts mid-re-address. */
    d.pulses_at_stop = chain.pulses_seen;
    return 0;
}

const acq::source_ops kFakeSourceOps{src_open, src_configure, src_start, src_read, src_stop};

acq::source_desc descs[2];
int acq_health_beats{0};

void on_health(uint32_t, acq::mapping_state) { ++acq_health_beats; }
void on_cycle_begin(uint32_t) {}
void on_cycle(const acq::cycle_facts &) {}
void on_sample(int, uint32_t, const acq::source_facts &, const struct tof_cliff_sample &) {}

/* Acquisition brought up for real and left RUNNING, which is the state that makes the quiesce
 * necessary in the first place. */
void arrange_running_acquisition()
{
    acq::teardown();
    sources[0] = fake_source{};
    sources[1] = fake_source{};
    acq_health_beats = 0;

    for (int i{0}; i < 2; ++i) {
        descs[i] = acq::source_desc{};
        descs[i].kind = acq::model::l4_cliff;
        descs[i].addr_7bit = static_cast<uint8_t>(0x2a + i);
        descs[i].dev = &sources[i];
        descs[i].scratch = &sources[i].scratch;
        descs[i].stream = &sources[i].stream;
        descs[i].ops = &kFakeSourceOps;
    }

    acq::config c{};
    c.sources = descs;
    c.source_count = 2;
    c.periods.cycle_period_ms = 50;
    c.periods.health_period_ms = 20;
    c.hooks.on_cycle_begin = on_cycle_begin;
    c.hooks.on_cycle = on_cycle;
    c.hooks.on_cliff_sample = on_sample;
    c.hooks.on_cliff_health = on_health;
    c.now_ms = fake_now_ms;
    c.mapping_state_provider = reported_state;
    zassert_equal(acq::init(c), 0);
    zassert_equal(acq::bring_up(), 0);
    zassert_false(acq::is_idle(), "acquisition has to be running for the quiesce to mean anything");
}

void use_the_real_quiesce()
{
    cm::config ccfg{};
    ccfg.chain = &chain_mutex;
    ccfg.ops = &chain;
    ccfg.spec = &runtime_spec;
    ccfg.quiesce = acq::try_stop;   // the production wiring, not a stand-in
    zassert_equal(cm::init(ccfg), 0);
}

} // namespace

/* Retires acquisition after every case. Most of them never bring it up, and teardown() on an
 * unconfigured subsystem is a no-op; the ones that do would otherwise leave a live health timer
 * reading a configuration the next case is entitled to replace. */
static void retire_acquisition(void *)
{
    acq::teardown();
}

ZTEST_SUITE(tof_commissioning, NULL, NULL, NULL, retire_acquisition, NULL);

ZTEST(tof_commissioning, test_a_healthy_chain_with_frozen_roles_is_proven)
{
    arrange(provable_spec());

    const cm::outcome r{cm::prove(7)};

    zassert_true(r.proven(), "stage %d proof %d commit %d", static_cast<int>(r.failed_at),
                 static_cast<int>(r.proof), static_cast<int>(r.commit));
    zassert_equal(au::current().state, acq::mapping_state::proven);
    zassert_equal(au::current().epoch, 7);
    zassert_equal(au::current().enumerated_mask, 0xF);
    zassert_equal(quiesce_calls, 1, "acquisition must be stopped exactly once, first");
    zassert_true(another_thread_can_take_the_chain(), "the session did not release the chain");
}

ZTEST(tof_commissioning, test_the_recursive_locks_inside_the_session_do_not_deadlock)
{
    /* begin_proof() -> is_idle() and commit_proof() -> begin_epoch() both take the chain mutex from
     * inside the session. Zephyr permits that for the owning thread, and a whole successful run is
     * the proof that it does -- a deadlock here would hang the suite rather than fail it, which is
     * why the test's real assertion is that it finishes at all. */
    arrange(provable_spec());
    zassert_true(cm::prove(1).proven());

    /* And a second run, so the lock count really did come back to zero rather than merely not
     * blocking once. */
    zassert_true(cm::prove(2).proven());
    zassert_true(another_thread_can_take_the_chain());
}

ZTEST(tof_commissioning, test_a_busy_chain_costs_nothing)
{
    /* The reason the lock is taken BEFORE the attempt is opened. Backwards, a busy chain would leave
     * the mapping revoked and an attempt open for a proof that never took a single step. */
    arrange(provable_spec());
    zassert_true(cm::prove(3).proven());
    const au::snapshot before{au::current()};

    hold_the_chain_elsewhere();
    const cm::outcome r{cm::prove(4)};
    release_the_chain_elsewhere();

    zassert_equal(r.failed_at, cm::stage::chain_busy);
    zassert_equal(au::current().state, before.state, "a busy chain revoked the mapping");
    zassert_equal(au::current().epoch, before.epoch);
    zassert_equal(au::attempt_nonce(), 0u, "a busy chain left an attempt open");
}

ZTEST(tof_commissioning, test_acquisition_is_stopped_before_the_chain_is_touched)
{
    /* stop-and-wait comes first, and a chain that cannot be quiesced is not walked at all. */
    arrange(provable_spec());
    quiesce_rc = -EBUSY;

    const cm::outcome r{cm::prove(5)};

    zassert_equal(r.failed_at, cm::stage::quiesce_failed);
    zassert_equal(r.rc, -EBUSY);
    zassert_equal(chain.pulses_seen, 0, "the chain was driven despite acquisition still running");
    zassert_equal(au::attempt_nonce(), 0u);
    zassert_true(another_thread_can_take_the_chain());
}

ZTEST(tof_commissioning, test_an_attempt_the_authority_refuses_releases_the_chain)
{
    arrange(provable_spec());
    acquisition_idle = false;   // the authority refuses to open an attempt

    const cm::outcome r{cm::prove(6)};

    zassert_equal(r.failed_at, cm::stage::attempt_refused);
    zassert_equal(r.begin, au::begin_refusal::acquisition_not_idle);
    zassert_equal(chain.pulses_seen, 0);
    zassert_true(another_thread_can_take_the_chain());
}

ZTEST(tof_commissioning, test_an_epoch_outside_the_wire_range_is_refused_not_truncated)
{
    /* 256 truncated to 0 would install an epoch the host never issued, and report success. */
    arrange(provable_spec());

    const cm::outcome r{cm::prove(256)};
    zassert_equal(r.failed_at, cm::stage::epoch_out_of_range);
    zassert_equal(quiesce_calls, 0, "nothing may happen before the epoch is checked");
    zassert_equal(au::current().state, acq::mapping_state::not_ready);

    /* 255 is inside the range and must work. */
    zassert_true(cm::prove(255).proven());
    zassert_equal(au::current().epoch, 255);
}

ZTEST(tof_commissioning, test_a_tail_merge_is_refused_and_nothing_is_proven)
{
    /* The DS20001 defect, driven through the real pipeline: one pulse enables two boards at the last
     * hop, so the write that addressed position 5 landed on position 6 as well.
     *
     * And it is caught by the WALK, not by the isolation -- which is worth knowing and was not
     * obvious. With both devices on position 5's address, position 6's step finds nothing at the
     * default address and the enumerator reports it `absent`, so walk 1 never reaches `complete` and
     * the isolation is not even run. The isolation remains the contract's required check and defence
     * in depth for merges at other hops and other orderings; it is not the first line here. */
    arrange(provable_spec());
    chain.tail_races_ahead = true;

    const cm::outcome r{cm::prove(8)};

    zassert_false(r.proven());
    zassert_equal(r.failed_at, cm::stage::evidence_refused);
    zassert_equal(r.proof, pf::refusal::walk1_not_complete, "refused as %d",
                  static_cast<int>(r.proof));
    zassert_false(r.isolation.attempted, "the isolation ran on a chain that never enumerated");
    zassert_equal(au::current().state, acq::mapping_state::not_ready, "a merge must not prove");
    zassert_equal(au::attempt_nonce(), 0u, "a refused run left the attempt open");
    zassert_true(another_thread_can_take_the_chain());
}

ZTEST(tof_commissioning, test_walk_two_runs_even_when_the_isolation_fails)
{
    /* The isolation has already darkened positions 1..N-1 and their addresses are gone with them.
     * Returning early would leave the chain in that half-addressed state, which nothing but a fresh
     * enumeration recovers -- so the recovery is not optional and not the operator's job. */
    arrange(provable_spec());
    /* Aimed at the isolation's two probes and nothing else. The arithmetic for this spec: walk 1
     * issues 8 all-off pulses plus 5 advances = 13, then the isolation issues positions - 2 = 4, so
     * its probes happen with exactly 17 pulses behind them. Walk 2's own all-off carries the count
     * past that immediately, which is the point -- a fault that leaked into walk 2 would be caught
     * as walk2_not_complete and this test would pass on the wrong refusal. */
    chain.error_probes_at_pulse_count = 17;

    const cm::outcome r{cm::prove(9)};

    zassert_false(r.proven());
    zassert_equal(r.failed_at, cm::stage::evidence_refused);
    zassert_equal(r.proof, pf::refusal::isolation_transport_error, "refused as %d",
                  static_cast<int>(r.proof));
    zassert_true(r.walk2.positions > 0, "walk 2 was not attempted, so the chain was left "
                                        "half-addressed with nothing to recover it");
    zassert_equal(au::attempt_nonce(), 0u);
    zassert_true(another_thread_can_take_the_chain());
}

ZTEST(tof_commissioning, test_a_control_failure_during_isolation_still_gets_a_walk_two)
{
    /* The same obligation for the harsher case: the pulses themselves failed, so the enable state is
     * unknown -- which is all the more reason to re-enumerate rather than walk away from it. */
    arrange(provable_spec());
    /* Let walk 1 finish -- thirteen pulses, eight of all-off plus five advances -- then fail the
     * isolation's first pulse and nothing else, so walk 2 can still run and be seen to. */
    chain.fail_pulse_at = 13;

    const cm::outcome r{cm::prove(10)};

    zassert_false(r.proven());
    zassert_not_equal(r.isolation_rc, 0, "the isolation should have reported a control failure");
    zassert_false(r.isolation.attempted, "and no observation with an unknown enable state");
    zassert_true(r.walk2.positions > 0, "walk 2 was not attempted after a control failure");
    zassert_equal(au::attempt_nonce(), 0u);
    zassert_true(another_thread_can_take_the_chain());
}

ZTEST(tof_commissioning, test_a_frozen_walk_one_skips_the_isolation)
{
    /* The asymmetry with the case above. Nothing has been darkened yet, so there is nothing to
     * compensate for -- and the isolation's precondition is a chain where everything is enabled and
     * addressed, which a frozen walk does not provide. The operator gets the walk's own diagnosis. */
    arrange(provable_spec());
    chain.present[3] = false;   // position 4 never answers: the walk cannot complete

    const cm::outcome r{cm::prove(11)};

    zassert_false(r.proven());
    zassert_equal(r.failed_at, cm::stage::evidence_refused);
    zassert_false(r.isolation.attempted, "the isolation ran on a chain that was not enumerated");
    zassert_true(another_thread_can_take_the_chain());
}

ZTEST(tof_commissioning, test_the_production_spec_now_proves_and_its_roles_are_the_frozen_ones)
{
    /* This case replaces "the production spec is refused because no roles are frozen", whose comment
     * read: if this ever starts passing, something has guessed. It has started passing -- so the
     * obligation moves rather than disappears. What it must now pin is WHAT was frozen, because a
     * silently wrong role table is exactly the failure the old refusal was standing in for.
     *
     * The mapping comes from dasher_connectivity.png: PCB3 front-left, PCB4 rear-left,
     * PCB5 rear-right, PCB6 front-right, in chain order. Asserted against the contract's own
     * role -> source_id table rather than against literals derived here, so that a reordering of
     * either the spec or the enum cannot make both sides agree on something wrong. */
    arrange(lexxhard::tof_chain::dasher_spec());

    const cm::outcome r{cm::prove(12)};

    zassert_true(r.proven(), "the frozen production spec no longer proves: stage %d proof %d",
                 static_cast<int>(r.failed_at), static_cast<int>(r.proof));
    zassert_equal(au::current().state, acq::mapping_state::proven);

    /* Every cliff position carries a KNOWN role, and the four of them are exactly the four corners
     * with no repeat -- which is what is_commissioning_profile() requires and what makes the four
     * source ids a permutation rather than a guess. */
    const enm::chain_spec spec{lexxhard::tof_chain::dasher_spec()};
    bool seen[4]{};

    for (size_t i{2}; i < spec.positions; ++i) {
        const int8_t src{pf::source_id_of(spec.at[i].role)};

        zassert_true(src >= 0, "position %zu still has an unknown role", i + 1);
        zassert_false(seen[src], "source id %d is claimed by two positions", src);
        seen[src] = true;
    }
    for (int i{0}; i < 4; ++i)
        zassert_true(seen[i], "no position claims source id %d", i);

    /* And the drawing's order, position by position. Spelled out so that a harness rework which
     * transposes two connectors fails HERE, in a diff somebody has to justify, rather than silently
     * publishing one corner's range under another corner's name. */
    zassert_equal(spec.at[2].role, enm::l4_role::front_left);
    zassert_equal(spec.at[3].role, enm::l4_role::rear_left);
    zassert_equal(spec.at[4].role, enm::l4_role::rear_right);
    zassert_equal(spec.at[5].role, enm::l4_role::front_right);
    zassert_equal(pf::source_id_of(spec.at[2].role), 0);
    zassert_equal(pf::source_id_of(spec.at[3].role), 1);
    zassert_equal(pf::source_id_of(spec.at[4].role), 2);
    zassert_equal(pf::source_id_of(spec.at[5].role), 3);
}

ZTEST(tof_commissioning, test_a_reused_epoch_is_refused_at_the_commit)
{
    /* The last stage that can refuse. The proof held, so the chain is fine -- the operator needs to
     * be told it is the epoch, not the hardware. */
    arrange(provable_spec());
    zassert_true(cm::prove(13).proven());

    const cm::outcome r{cm::prove(13)};
    zassert_equal(r.failed_at, cm::stage::commit_refused);
    zassert_equal(r.commit, au::commit_refusal::epoch_reused);
    zassert_equal(au::attempt_nonce(), 0u, "a matching token must spend its attempt either way");
    zassert_true(another_thread_can_take_the_chain());
}

ZTEST(tof_commissioning, test_nothing_runs_before_init)
{
    cm::config bad{};
    zassert_equal(cm::init(bad), -EINVAL);

    /* And a partially filled configuration is refused too: a missing quiesce hook would mean walking
     * the chain while acquisition is still reading it. */
    cm::config no_quiesce{};
    no_quiesce.chain = &chain_mutex;
    no_quiesce.ops = &chain;
    no_quiesce.spec = &runtime_spec;
    zassert_equal(cm::init(no_quiesce), -EINVAL);
}

/* ------------------------------------------- the REAL acquisition quiesce ----------- */

ZTEST(tof_commissioning, test_the_real_quiesce_refuses_a_busy_chain_instead_of_waiting)
{
    /* The P1 this pair of cases exists for. Every other busy-chain case here fakes the quiesce, so
     * they all prove things about the session's K_NO_WAIT acquire and nothing about the step before
     * it -- and the step before it was the one that blocked. */
    arrange(provable_spec());
    use_the_real_quiesce();
    arrange_running_acquisition();

    const au::snapshot before{au::current()};

    hold_the_chain_elsewhere();
    const int64_t started{k_uptime_get()};
    const cm::outcome r{cm::prove(9)};
    const int64_t elapsed{k_uptime_get() - started};
    release_the_chain_elsewhere();

    zassert_equal(r.failed_at, cm::stage::quiesce_failed, "the busy chain was not refused here");
    zassert_equal(r.rc, -EBUSY);
    /* The holder releases on its own after 3 s, so a quiesce that WAITED would still return a
     * result -- just late. This is the assertion that tells the two apart. */
    zassert_true(elapsed < 500, "the quiesce waited for the chain (%lld ms)", elapsed);

    /* Refused before anything was spent: no attempt, no revoke, and no enumeration. */
    zassert_equal(au::attempt_nonce(), 0u, "a refused quiesce left an attempt open");
    zassert_equal(au::current().state, before.state, "a refused quiesce revoked the mapping");
    zassert_equal(au::current().epoch, before.epoch);
    zassert_equal(chain.pulses_seen, 0, "the chain was walked after the quiesce had refused");

    /* And acquisition is untouched, which is what makes the refusal free: it is still running, with
     * no device stopped. */
    zassert_false(acq::is_idle(), "the refused quiesce stopped acquisition anyway");
    zassert_equal(sources[0].stop_calls, 0);
    zassert_equal(sources[1].stop_calls, 0);
}

ZTEST(tof_commissioning, test_the_real_quiesce_stops_acquisition_before_the_first_walk)
{
    /* The other half: when the chain IS free, the real quiesce has to actually quiesce -- and do it
     * before the first pulse, not merely at some point during the session. */
    arrange(provable_spec());
    use_the_real_quiesce();
    arrange_running_acquisition();

    const int beats_before{acq_health_beats};

    zassert_true(cm::prove(11).proven());

    zassert_true(acq::is_idle(), "the session ran with acquisition still live");
    zassert_equal(sources[0].stop_calls, 1);
    zassert_equal(sources[1].stop_calls, 1);
    zassert_equal(sources[0].pulses_at_stop, 0,
                  "a device was still ranging when the enumeration started pulsing");
    zassert_equal(sources[1].pulses_at_stop, 0);
    zassert_true(chain.pulses_seen > 0, "nothing was enumerated, so the ordering proves nothing");

    /* stop(), not teardown(): the heartbeat has to have survived the whole run. */
    k_msleep(80);
    zassert_true(acq_health_beats >= beats_before + 3,
                 "the heartbeat stopped during commissioning: %d -> %d", beats_before,
                 acq_health_beats);
}

/* The stack for the cases that need a real acquisition THREAD rather than a hand-driven cycle. Owned
 * by the suite, because the module sizes its own from a devicetree that does not exist here. */
K_THREAD_STACK_DEFINE(cm_acq_stack, 2048);

static acq::thread_config acq_thread(uint32_t join_timeout_ms)
{
    acq::thread_config t{};

    t.stack = cm_acq_stack;
    t.stack_size = K_THREAD_STACK_SIZEOF(cm_acq_stack);
    t.priority = K_PRIO_PREEMPT(5);
    t.join_timeout_ms = join_timeout_ms;
    return t;
}

ZTEST(tof_commissioning, test_a_thread_that_will_not_join_keeps_the_gate_shut)
{
    /* The acceptance boundary for the timeout path, at the gate rather than in the primitive: a
     * commissioning run that cannot get a clean stop must not start, must not revoke the mapping it
     * has, and must not open an attempt. Nothing is killed -- a thread inside a vendor driver holds
     * the chain lock and a half-finished transfer, so aborting it is strictly worse than refusing.
     *
     * Staged with a sensor read that blocks for far longer than the injected join timeout, which is
     * the real shape of this failure. */
    arrange(provable_spec());
    use_the_real_quiesce();
    arrange_running_acquisition();
    acq::stop();   // the hand-driven bring-up in the fixture; the thread does its own

    sources[0].read_delay_ms = 400;
    /* Baseline taken AFTER the fixture's own stop, which has already counted one: the property is
     * that the REFUSAL stops nothing, not that nothing has ever been stopped. */
    const int stops_before{sources[0].stop_calls};

    zassert_equal(acq::start(acq_thread(20)), 0);
    k_msleep(30);   // inside the blocking read by now

    const au::snapshot before{au::current()};
    const int64_t started{k_uptime_get()};
    const cm::outcome r{cm::prove(21)};
    const int64_t elapsed{k_uptime_get() - started};

    zassert_equal(r.failed_at, cm::stage::quiesce_failed, "the run started without a clean stop");
    zassert_equal(r.rc, -EBUSY);
    zassert_true(elapsed < 300, "the refusal waited for the thread instead of its timeout (%lld ms)",
                 elapsed);
    zassert_equal(au::attempt_nonce(), 0u, "a refused quiesce opened an attempt");
    zassert_equal(au::current().state, before.state, "a refused quiesce revoked the mapping");
    zassert_equal(chain.pulses_seen, 0, "the chain was walked without a clean stop");
    zassert_true(acq::thread_running(), "the timeout killed the thread");
    zassert_equal(sources[0].stop_calls, stops_before,
                  "something stopped the devices from outside the thread");

    /* Unblocked, it exits on the request that was already made. */
    sources[0].read_delay_ms = 0;
    zassert_equal(acq::join(500), 0);
    zassert_equal(sources[0].stop_calls, stops_before + 1,
                  "the thread did not stop its own devices on the way out");
}
