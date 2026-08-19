/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_mapping_authority.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>

namespace lexxhard::tof_authority {

namespace {

namespace enm = tof_enum;

/* The published state, packed into one word so a reader never blocks and never sees half an
 * update. Layout is explicit rather than a bitfield struct: bitfield ordering is
 * implementation-defined, and this word's only job is to be written and read by this file.
 *
 *   bits  0..7   epoch
 *   bits  8..9   mapping_state
 *   bits 10..13  enumerated_mask
 *   bits 14..17  model_verified_mask
 *   bits 18..20  chain flags (contract bits 0-2)
 *   bits 21..28  failing_chain_position
 */
constexpr int kEpochShift{0};
constexpr int kStateShift{8};
constexpr int kEnumeratedShift{10};
constexpr int kModelVerifiedShift{14};
constexpr int kFlagsShift{18};
constexpr int kFailingShift{21};

atomic_t published_{};

config cfg_{};
bool initialised_{false};

pf::gate gate_{};
uint32_t attempt_{0};       // the nonce of the open attempt; 0 when none is open
pf::fingerprint installed_{};

/* Named empties, used instead of assigning a fresh `pf::fingerprint{}` or passing a fresh
 * `snapshot{}`.
 *
 * Not style: the Zephyr SDK 0.16.5-1 cross compiler (arm-zephyr-eabi-gcc 12.2.0) hits an
 * internal compiler error -- "in gimple_add_tmp_var, at gimplify.cc:772" -- on a fresh
 * temporary of an aggregate this size being assigned into a reference or a parameter, while
 * host gcc 11.4 compiles it happily. The whole host suite passed and the B6 image failed to
 * build, twice, before this pattern was named. Copying from a const instance sidesteps it and
 * reads the same. */
const pf::fingerprint kNoMapping{};
const snapshot kUnknown{};

/* Every epoch used since power-on, one bit each. Not "the last epoch": the contract's
 * requirement is that no epoch is REUSED, and comparing against only the previous value
 * would let 1, 2, 1 through -- after which two different mappings share an epoch and a
 * consumer correlating by (source_id, epoch, cycle_seq) cannot tell their frames apart.
 *
 * Thirty-two bytes, and when they run out the answer is a refusal, not a wrap. A machine
 * that has been re-proven 256 times in one power cycle has a problem that reusing an epoch
 * would hide.
 *
 * All 256 values are legal, INCLUDING 0. An earlier version reserved 0 on the grounds that it
 * is what the health frame carries while no epoch has been issued -- which was wrong twice
 * over: mapping_state is in the same frame, so PROVEN with epoch 0 and UNKNOWN with epoch 0
 * are not confusable, and the contract has the host increment modulo 256, so it WILL offer 0
 * eventually and a firmware that refused it would stall commissioning.
 *
 * It SURVIVES init(), which is the whole point: the guarantee is "not reused since power-on",
 * and clearing it on re-init would hand the entire space back -- init, epoch 7, init, epoch 7
 * would pass while breaking the contract. Nothing here clears it; statics start zeroed and the
 * only door is the CONFIG_ZTEST one.
 *
 * Power-on scope, stated plainly: this is RAM, so it is empty again after a reset. Under the
 * commissioning profile the host is the authority that persists epochs across restarts, and
 * this bitmap is only the firmware's half -- see the contract's *Commissioning mapping_epoch
 * issuance*. */
uint32_t used_epochs_[8]{};

bool epoch_used(uint8_t e)
{
    return (used_epochs_[e >> 5] & (1U << (e & 31))) != 0;
}

void mark_epoch_used(uint8_t e)
{
    used_epochs_[e >> 5] |= 1U << (e & 31);
}

bool all_epochs_used()
{
    for (const uint32_t w : used_epochs_) {
        if (w != 0xFFFFFFFFU)
            return false;
    }
    return true;
}

uint32_t pack(const snapshot &s)
{
    return (static_cast<uint32_t>(s.epoch) << kEpochShift) |
           (static_cast<uint32_t>(s.state) << kStateShift) |
           (static_cast<uint32_t>(s.enumerated_mask & 0xF) << kEnumeratedShift) |
           (static_cast<uint32_t>(s.model_verified_mask & 0xF) << kModelVerifiedShift) |
           (static_cast<uint32_t>(s.chain_flags & 0x7) << kFlagsShift) |
           (static_cast<uint32_t>(s.failing_position) << kFailingShift);
}

snapshot unpack(uint32_t w)
{
    snapshot s{};
    s.epoch = static_cast<uint8_t>((w >> kEpochShift) & 0xFF);
    s.state = static_cast<tof_acq::mapping_state>((w >> kStateShift) & 0x3);
    s.enumerated_mask = static_cast<uint8_t>((w >> kEnumeratedShift) & 0xF);
    s.model_verified_mask = static_cast<uint8_t>((w >> kModelVerifiedShift) & 0xF);
    s.chain_flags = static_cast<uint8_t>((w >> kFlagsShift) & 0x7);
    s.failing_position = static_cast<uint8_t>((w >> kFailingShift) & 0xFF);
    return s;
}

void publish(const snapshot &s)
{
    atomic_set(&published_, static_cast<atomic_val_t>(pack(s)));
}

/* Is the chain this proof describes the chain acquisition is configured for?
 *
 * Field by field, including the addresses. A proof of a correctly wired chain at the wrong
 * addresses would install a mapping whose sensors are read at addresses nobody proved, and
 * the failure would look like four healthy sensors reporting someone else's geometry. */
bool matches_runtime(const pf::fingerprint &fp, const enm::chain_spec &spec)
{
    if (fp.positions != spec.positions)
        return false;
    for (size_t i{0}; i < fp.positions; ++i) {
        if (fp.at[i].expected != spec.at[i].expected ||
            fp.at[i].address != spec.at[i].target_addr ||
            fp.at[i].source_id != spec.at[i].source_id ||
            fp.at[i].role != spec.at[i].role)
            return false;
    }
    return true;
}

/* The contract's own table, as an explicit switch.
 *
 *   source_id 0 cliff_front_left / 1 cliff_rear_left / 2 cliff_rear_right / 3 cliff_front_right
 *
 * Not `static_cast<int>(role) - 1`, even though l4_role happens to be declared in that order
 * today. The mapping is owned by the wire contract, and an arithmetic shortcut would silently
 * follow any future reordering of an enumeration that has nothing to do with the contract --
 * producing frames whose source_id names the wrong corner of the robot, which is the one error
 * class this whole subsystem exists to prevent. Returns -1 for a role that has no source. */
/* The two enumeration masks, from the chain a proof actually proved.
 *
 * Both are 0xF for a committed proof, and that is not a shortcut: the proof cannot be granted
 * unless every cliff position enumerated to its own address AND returned the expected model id
 * AND carries a known, distinct role. So the four bits are exactly the four positions the
 * fingerprint describes. Deriving them from the fingerprint rather than writing 0xF keeps the
 * derivation honest if the profile ever admits a chain with fewer cliff sources. */
void masks_from(const pf::fingerprint &fp, uint8_t &enumerated, uint8_t &model_verified)
{
    enumerated = 0;
    model_verified = 0;
    for (size_t i{0}; i < fp.positions; ++i) {
        const int8_t src{pf::source_id_of(fp.at[i].role)};
        if (src < 0 || !fp.at[i].verified)
            continue;
        const uint8_t bit{static_cast<uint8_t>(1U << src)};
        enumerated |= bit;
        /* One flag, two masks, and they are not redundant on the wire: the contract keeps
         * "enumerated to its own address" and "returned the expected model id" as separate
         * bits. This layer cannot separate them because the proof refuses unless BOTH hold --
         * a position that failed either one never reaches a fingerprint. So they agree here by
         * construction, and the day the proof admits a partial chain is the day they diverge. */
        model_verified |= bit;
    }
}

} // namespace

int init(const config &cfg)
{
    if (cfg.runtime_spec == nullptr || cfg.begin_epoch == nullptr ||
        cfg.acquisition_idle == nullptr || cfg.install_mapping == nullptr ||
        cfg.runtime_spec->positions == 0) {
        /* A failed init leaves the authority unusable rather than quietly running on whatever
         * was configured before. That direction costs a proven mapping -- but the alternative
         * is a commit compared against a configuration nobody meant to be current, and of the
         * two, losing PROVEN is the one that fails safe. */
        initialised_ = false;
        publish(kUnknown);
        return -EINVAL;
    }

    cfg_ = cfg;
    attempt_ = 0;
    installed_ = kNoMapping;
    /* used_epochs_ is deliberately NOT cleared here -- see its definition. A re-init means a
     * new configuration, not a new power cycle. */

    publish(kUnknown);
    initialised_ = true;
    return 0;
}

attempt begin_proof()
{
    attempt a{};

    if (!initialised_) {
        a.reason = begin_refusal::not_initialised;
        return a;
    }

    /* The idle check comes before the revocation, and before any challenge exists. A proof is
     * two full enumerations, which drop enable lines and re-address parts; discovering at
     * commit time that acquisition had been running means that already happened underneath a
     * live reader. Refusing to start is the only refusal that helps.
     *
     * Nothing is revoked on this path either: a running acquisition under a proven mapping is
     * the normal state, and tearing it down because someone asked at the wrong moment would
     * turn a mistimed request into an outage. */
    if (!cfg_.acquisition_idle()) {
        a.reason = begin_refusal::acquisition_not_idle;
        return a;
    }

    /* Revoke BEFORE anything else, and before the caller touches an enable line. The
     * contract's order is normative: measurements stop first, then health reports the loss.
     * Here that is one atomic publish, and the publisher's own per-cycle re-check is what
     * discards a cycle encoded moments earlier. */
    const snapshot now{current()};
    snapshot next{};
    next.state = now.state == tof_acq::mapping_state::proven ? tof_acq::mapping_state::lost
                                                             : now.state;
    /* Keep the epoch through a revocation. The measurements a consumer already accepted were
     * correlated under it, and zeroing it would retire them for the wrong reason. */
    next.epoch = now.epoch;
    next.chain_flags = now.chain_flags;
    next.failing_position = now.failing_position;
    /* The enumeration masks are deliberately NOT carried over, and this is the one place they
     * are cleared. The contract defines them as the last enumeration ATTEMPT, so state carries
     * the trust and the masks carry the observation -- which is why a runtime loss keeps them
     * (the previous enumeration really did enumerate all four) and this path does not: from
     * here the enable lines are about to move, so no completed enumeration describes the chain
     * that will exist a moment from now. Left as the default zero rather than assigned, and
     * said out loud because "it happens to be default-constructed" is not a rule. */
    publish(next);

    installed_ = kNoMapping;

    /* A fresh challenge invalidates the previous one, so a token minted before this
     * revocation can no longer be committed. */
    const pf::challenge c{gate_.issue()};
    attempt_ = c.nonce();
    a.challenge = c;
    a.reason = begin_refusal::none;
    return a;
}

pf::verdict evaluate(const pf::evidence &ev, const pf::challenge &c)
{
    if (!initialised_) {
        pf::verdict v{};
        v.reason = pf::refusal::missing_evidence;
        return v;
    }
    return gate_.evaluate(ev, c);
}

bench_result evaluate_bench(const pf::evidence &ev)
{
    bench_result r{};

    if (!initialised_)
        return r;
    /* An open attempt means a product proof is in progress on this chain. Bench evidence can
     * only have come from walking a chain, and walking it now would re-address the one that
     * attempt is about. */
    if (attempt_ != 0)
        return r;

    r.ran = true;
    r.report = gate_.evaluate_bench(ev);
    return r;
}

commit_refusal commit_proof(pf::proof_token &&token, uint8_t host_epoch)
{
    /* Take ownership immediately. Whatever happens below, the caller's token is spent: a
     * refused commit must not leave a committable object behind for a second try. */
    const pf::proof_token held{static_cast<pf::proof_token &&>(token)};

    if (!initialised_)
        return commit_refusal::not_initialised;
    if (attempt_ == 0)
        return commit_refusal::no_attempt;
    if (!held.valid())
        return commit_refusal::invalid_token;
    if (held.nonce() != attempt_)
        return commit_refusal::wrong_attempt;

    /* From here the token IS this attempt's, so the attempt is spent whatever happens below. One
     * attempt buys one commit, for the same reason one challenge buys one evaluation: otherwise a
     * caller could re-present the same evidence until a later check happened to pass, and each
     * retry would be judged against a chain that is one attempt older. */
    struct spend_attempt {
        ~spend_attempt() { attempt_ = 0; }
    } const spend{};

    /* Re-checked here rather than trusted. The evaluator refuses anything but the profile
     * today; if that ever loosens, this is the check that keeps a bench chain from becoming
     * the product's mapping. */
    if (!pf::is_commissioning_profile(held.proven()))
        return commit_refusal::not_commissioning_profile;

    if (!matches_runtime(held.proven(), *cfg_.runtime_spec))
        return commit_refusal::runtime_mapping_mismatch;

    if (all_epochs_used())
        return commit_refusal::epoch_space_exhausted;
    if (epoch_used(host_epoch))
        return commit_refusal::epoch_reused;

    /* THE TRANSACTION. Nothing above this point has changed any published state, and nothing
     * below it may publish PROVEN until every step has succeeded.
     *
     * Record the mapping, then the cycle reset, then key the descriptors, then the epoch, then
     * PROVEN -- in that order, because each step is a precondition of the meaning of the next. A consumer that saw PROVEN with the old
     * epoch would correlate against an epoch no measurement will carry; one that saw PROVEN
     * before the cycle reset would accept a cycle number from the previous epoch. */
    installed_ = held.proven();

    const int rc{cfg_.begin_epoch()};
    if (rc != 0) {
        installed_ = kNoMapping;
        return rc == -EBUSY ? commit_refusal::acquisition_busy
                            : commit_refusal::epoch_install_failed;
    }

    /* The descriptors, keyed from this mapping, BEFORE anything is published.
     *
     * This step used to live outside the transaction: the shell called it after commit_proof() had
     * already published PROVEN. Everything about that was wrong in the same direction -- a failure
     * left the authority PROVEN with descriptors that did not describe the proven chain, a partial
     * write left some positions keyed, and a later failed re-proof could leave the PREVIOUS
     * mapping's keys in place with nothing to notice it. The clamp hid the consequence; lifting the
     * clamp would have turned it into a measurement published under another corner's source_id.
     *
     * A failure here keeps the state the revocation left (LOST, or UNKNOWN if nothing was ever
     * proven) and clears the installed mapping. The epoch is NOT marked used: it was never issued,
     * for the same reason a failed cycle reset does not burn one.
     *
     * begin_epoch() has already reset the cycle counter at this point, which is safe precisely
     * because PROVEN is not published below: publication requires PROVEN, so no measurement can
     * carry a reissued (source_id, epoch, cycle_seq) triple out of this failure. */
    if (const int irc{cfg_.install_mapping(installed_, host_epoch)}; irc != 0) {
        installed_ = kNoMapping;
        return commit_refusal::mapping_install_failed;
    }

    mark_epoch_used(host_epoch);

    snapshot proven{};
    proven.state = tof_acq::mapping_state::proven;
    proven.epoch = host_epoch;
    /* Filled from the proven chain. A proof carries four known, distinct roles by rule, so the
     * masks a consumer reads are now derived from evidence rather than left at zero -- which
     * they were only for as long as nothing had proved a role at all. */
    masks_from(installed_, proven.enumerated_mask, proven.model_verified_mask);
    proven.failing_position = 0xFF;
    publish(proven);

    return commit_refusal::none;
}

bool abort_proof(const pf::challenge &c)
{
    if (!initialised_ || !c.valid() || c.nonce() != attempt_ || attempt_ == 0)
        return false;
    attempt_ = 0;
    return true;
}

void note_mapping_lost()
{
    if (!initialised_)
        return;

    const snapshot now{current()};
    /* Only a proven mapping can be lost. Reporting LOST from UNKNOWN would tell a consumer
     * that something it had been trusting has gone away, when nothing ever had been. */
    if (now.state != tof_acq::mapping_state::proven)
        return;

    snapshot next{now};
    next.state = tof_acq::mapping_state::lost;
    publish(next);
    installed_ = kNoMapping;
}

void note_chain_fault(uint8_t chain_flags, uint8_t failing_position)
{
    if (!initialised_)
        return;

    snapshot next{current()};
    next.state = tof_acq::mapping_state::fault;
    next.chain_flags = static_cast<uint8_t>(chain_flags & 0x7);
    next.failing_position = failing_position;
    publish(next);
    installed_ = kNoMapping;
}

snapshot current()
{
    return unpack(static_cast<uint32_t>(atomic_get(&published_)));
}

tof_acq::mapping_state state_provider()
{
    return current().state;
}

const pf::fingerprint &installed_mapping()
{
    return installed_;
}

uint32_t attempt_nonce()
{
    return attempt_;
}

#ifdef CONFIG_ZTEST
void reset_epoch_history_for_test()
{
    memset(used_epochs_, 0, sizeof used_epochs_);
}
#endif

uint32_t epochs_used()
{
    uint32_t n{0};
    for (const uint32_t w : used_epochs_) {
        for (int b{0}; b < 32; ++b) {
            if ((w & (1U << b)) != 0)
                ++n;
        }
    }
    return n;
}

} // namespace lexxhard::tof_authority

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
