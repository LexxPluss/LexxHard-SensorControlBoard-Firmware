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
 * that has been re-proven 255 times in one power cycle has a problem that reusing an epoch
 * would hide. Bit 0 is set at init because epoch 0 is what "no epoch" reports on the wire.
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

} // namespace

int init(const config &cfg)
{
    if (cfg.runtime_spec == nullptr || cfg.begin_epoch == nullptr ||
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
    memset(used_epochs_, 0, sizeof used_epochs_);
    /* Epoch 0 is spoken for: it is what the health frame carries while no epoch has been
     * issued, so accepting it as a proven epoch would make "proven under epoch 0" and "never
     * proven" indistinguishable to a consumer reading one frame. */
    mark_epoch_used(0);

    publish(kUnknown);
    initialised_ = true;
    return 0;
}

pf::challenge begin_proof()
{
    if (!initialised_)
        return pf::challenge{};

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
    publish(next);

    installed_ = kNoMapping;

    /* A fresh challenge invalidates the previous one, so a token minted before this
     * revocation can no longer be committed. */
    const pf::challenge c{gate_.issue()};
    attempt_ = c.nonce();
    return c;
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

pf::bench_report evaluate_bench(const pf::evidence &ev)
{
    if (!initialised_) {
        pf::bench_report b{};
        b.reason = pf::refusal::missing_evidence;
        return b;
    }
    return gate_.evaluate_bench(ev);
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

    /* Re-checked here rather than trusted. The evaluator refuses anything but the profile
     * today; if that ever loosens, this is the check that keeps a bench chain from becoming
     * the product's mapping. */
    if (!pf::is_commissioning_profile(held.proven()))
        return commit_refusal::not_commissioning_profile;

    if (!matches_runtime(held.proven(), *cfg_.runtime_spec))
        return commit_refusal::runtime_mapping_mismatch;

    if (host_epoch == 0)
        return commit_refusal::epoch_zero;
    if (all_epochs_used())
        return commit_refusal::epoch_space_exhausted;
    if (epoch_used(host_epoch))
        return commit_refusal::epoch_reused;

    /* THE TRANSACTION. Nothing above this point has changed any published state, and nothing
     * below it may publish PROVEN until every step has succeeded.
     *
     * Install, then epoch, then cycle reset, then PROVEN -- in that order, because each step
     * is a precondition of the meaning of the next. A consumer that saw PROVEN with the old
     * epoch would correlate against an epoch no measurement will carry; one that saw PROVEN
     * before the cycle reset would accept a cycle number from the previous epoch. */
    installed_ = held.proven();

    const int rc{cfg_.begin_epoch()};
    if (rc != 0) {
        installed_ = kNoMapping;
        return rc == -EBUSY ? commit_refusal::acquisition_busy
                            : commit_refusal::epoch_install_failed;
    }

    mark_epoch_used(host_epoch);

    snapshot proven{};
    proven.state = tof_acq::mapping_state::proven;
    proven.epoch = host_epoch;
    /* Still zero, and the header says why: the contract keys these by source_id and this
     * firmware has no honest source_id for a cliff position until the role table is frozen.
     * The installed fingerprint carries the roles, so filling them is a small edit in the
     * commit that freezes the table -- not a redesign. */
    proven.enumerated_mask = 0;
    proven.model_verified_mask = 0;
    proven.failing_position = 0xFF;
    publish(proven);

    /* The attempt is over either way. A committed proof is not a licence to commit again. */
    attempt_ = 0;
    return commit_refusal::none;
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
