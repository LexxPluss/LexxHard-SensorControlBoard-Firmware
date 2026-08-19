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

#include "tof_chain_spec.hpp"
#include "tof_commissioning.hpp"
#include "tof_tail_isolation.hpp"

namespace enm = lexxhard::tof_enum;
namespace pf = lexxhard::tof_proof;
namespace au = lexxhard::tof_authority;
namespace acq = lexxhard::tof_acq;
namespace cm = lexxhard::tof_commissioning;

namespace {

constexpr enm::id_bytes kL4Id{0xeb, 0xaa};
constexpr enm::id_bytes kL7Id{0xf0, 0x02};

K_MUTEX_DEFINE(chain_mutex);

/* The chain as the hardware behaves, enough of it for a full enumeration plus an isolation: a shift
 * register of enables, one device per stage, each with an address that a disabled L4 loses. */
struct fake_chain final : enm::chain_ops {
    static constexpr size_t kStages{6};

    bool present[kStages]{true, true, true, true, true, true};
    bool enabled[kStages]{false, false, false, false, false, false};
    uint8_t addr[kStages]{0x29, 0x29, 0x29, 0x29, 0x29, 0x29};
    bool l7[kStages]{true, true, false, false, false, false};
    bool data{false};

    /* L7 keeps its address across an enable-low while powered; L4's enable is reset-class. Modelling
     * both is what makes walk 2 report `retained` for the grid sensors and `enumerated` for the
     * cliff ones -- the exact asymmetry the fingerprint has to normalise. */
    bool reset_class(size_t i) const { return !l7[i]; }

    int set_data_rc{0};
    int pulse_rc{0};
    int fail_pulse_at{-1};
    int pulses_seen{0};
    /* One clock pulse enables TWO stages at the last hop -- the DS20001 defect, 4/4 reproducible
     * before the 50 ohm series resistor. Modelled in the shift rather than in readdress(), because
     * that is where it happens: the data edge beat the clock edge into the last flip-flop. */
    bool tail_races_ahead{false};
    /* Probes fail only while exactly this many pulses have been issued. An exact count rather than
     * "from here on", because the isolation and walk 2 share this probe: a fault that persisted would
     * break walk 2 as well, and the evaluator checks walk 2 BEFORE the isolation -- so the test would
     * pass on the wrong refusal and prove nothing about the isolation. */
    int error_probes_at_pulse_count{-1};

    int set_data(bool level) override
    {
        if (set_data_rc != 0)
            return set_data_rc;
        data = level;
        apply(0, level);
        return 0;
    }

    void apply(size_t i, bool level)
    {
        const bool was{enabled[i]};
        enabled[i] = level;
        if (was && !level && reset_class(i))
            addr[i] = enm::kDefaultAddr;
    }

    int pulse_clock() override
    {
        if (fail_pulse_at >= 0 && pulses_seen == fail_pulse_at) {
            ++pulses_seen;
            return -EIO;
        }
        ++pulses_seen;
        if (pulse_rc != 0)
            return pulse_rc;
        for (size_t i{kStages - 1}; i > 0; --i)
            apply(i, enabled[i - 1]);
        apply(0, data);
        if (tail_races_ahead)
            apply(kStages - 1, enabled[kStages - 2]);
        return 0;
    }

    int answerer(uint8_t addr7) const
    {
        for (size_t i{0}; i < kStages; ++i) {
            if (present[i] && enabled[i] && addr[i] == addr7)
                return static_cast<int>(i);
        }
        return -1;
    }

    enm::probe_result probe(uint8_t addr7) override
    {
        if (error_probes_at_pulse_count >= 0 && pulses_seen == error_probes_at_pulse_count)
            return {enm::probe_state::transport_error, -EIO};
        return answerer(addr7) >= 0 ? enm::probe_result{enm::probe_state::ack, 0}
                                    : enm::probe_result{enm::probe_state::nack, 0};
    }

    int read_id(enm::model, uint8_t addr7, enm::id_bytes &out) override
    {
        const int i{answerer(addr7)};
        if (i < 0)
            return -ENXIO;
        out = l7[i] ? kL7Id : kL4Id;
        return 0;
    }

    enm::readdress_result readdress(enm::model, uint8_t old7, uint8_t new7) override
    {
        const int i{answerer(old7)};
        if (i < 0)
            return {-ENXIO, enm::readdress_stage::collision};
        /* Every device currently answering the old address moves. That is what the bus does, and it
         * is how a merge becomes one address with two devices behind it. */
        for (size_t k{0}; k < kStages; ++k) {
            if (present[k] && enabled[k] && addr[k] == old7)
                addr[k] = new7;
        }
        (void)i;
        return {};
    }

    void wait(wait_reason) override {}
};

enm::chain_spec provable_spec()
{
    enm::chain_spec s{lexxhard::tof_chain::dasher_spec()};
    s.at[2].role = enm::l4_role::front_left;
    s.at[3].role = enm::l4_role::rear_left;
    s.at[4].role = enm::l4_role::rear_right;
    s.at[5].role = enm::l4_role::front_right;
    return s;
}

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

void arrange(const enm::chain_spec &spec)
{
    runtime_spec = spec;
    chain = fake_chain{};
    quiesce_rc = 0;
    quiesce_calls = 0;
    acquisition_idle = true;

    au::config acfg{};
    acfg.runtime_spec = &runtime_spec;
    acfg.begin_epoch = fake_begin_epoch;
    acfg.acquisition_idle = fake_is_idle;
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
    (void)k_sem_take(&holder_may_release, K_FOREVER);
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

} // namespace

ZTEST_SUITE(tof_commissioning, NULL, NULL, NULL, NULL, NULL);

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

ZTEST(tof_commissioning, test_the_production_spec_is_refused_because_no_roles_are_frozen)
{
    /* dasher_spec() exactly as production carries it. This is the honest state of the machine: the
     * transaction is carried out, the chain is fine, and the proof refuses because nobody has frozen
     * which physical corner each carrier is. If this ever starts passing, something has guessed. */
    arrange(lexxhard::tof_chain::dasher_spec());

    const cm::outcome r{cm::prove(12)};

    zassert_false(r.proven());
    zassert_equal(r.failed_at, cm::stage::evidence_refused);
    zassert_equal(r.proof, pf::refusal::role_unknown);
    zassert_equal(au::current().state, acq::mapping_state::not_ready);
    /* The clamp is untouched by any of this, and is asserted where it lives: clamp_mapping_state()
     * is in the acquisition layer, which this image deliberately does not link -- it would drag the
     * vendor ULD into a test about lock ordering. The sensor suite owns that assertion. */
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
