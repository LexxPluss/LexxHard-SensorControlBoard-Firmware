/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * COMPILED WITH -Werror=switch-enum. Every switch below lists every enumerator and has no `default`;
 * adding one to any of the four enums fails the build until it is given a wire meaning. See the
 * header for why that is a build setting rather than a language guarantee.
 */

#include "tof_commission_map.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

namespace lexxhard::tof_commission_map {

namespace au = tof_authority;
namespace pf = tof_proof;
namespace cm = tof_commissioning;

using wire::result;
using wire::wire_detail;
using wire::wire_stage;

outcome map_begin_refusal(au::begin_refusal b)
{
    switch (b) {
    case au::begin_refusal::none:
        /* Not reachable with attempt_refused, and mapped anyway: a value that cannot occur still
         * needs a wire meaning, because "cannot occur" is exactly the class of claim that stops
         * being true. */
        return {result::internal_error, wire_stage::not_started, wire_detail::none};
    case au::begin_refusal::not_initialised:
        return {result::misconfigured, wire_stage::not_started, wire_detail::none};
    case au::begin_refusal::acquisition_not_idle:
        return {result::busy_chain, wire_stage::quiesce, wire_detail::chain_busy};
    }
    return {result::internal_error, wire_stage::not_started, wire_detail::none};
}

outcome map_commit_refusal(au::commit_refusal c)
{
    switch (c) {
    case au::commit_refusal::none:
        return {result::ok, wire_stage::complete, wire_detail::none};
    case au::commit_refusal::not_initialised:
        return {result::misconfigured, wire_stage::not_started, wire_detail::none};
    case au::commit_refusal::no_attempt:
    case au::commit_refusal::invalid_token:
    case au::commit_refusal::wrong_attempt:
        return {result::proof_failed, wire_stage::commit, wire_detail::commit_refused};
    case au::commit_refusal::not_commissioning_profile:
        return {result::misconfigured, wire_stage::not_started, wire_detail::none};
    case au::commit_refusal::runtime_mapping_mismatch:
        return {result::proof_failed, wire_stage::commit, wire_detail::commit_refused};
    case au::commit_refusal::epoch_reused:
        return {result::epoch_reused, wire_stage::commit, wire_detail::none};
    case au::commit_refusal::epoch_space_exhausted:
        return {result::attempts_exhausted, wire_stage::commit, wire_detail::none};
    case au::commit_refusal::acquisition_busy:
        /* The transaction RAN before reaching here -- two walks, isolation, the retime and the
         * identity re-check -- so this is busy_at_commit and not busy_chain. The host owes a new
         * ordinal for it and does not for the other. */
        return {result::busy_at_commit, wire_stage::commit, wire_detail::chain_busy};
    case au::commit_refusal::epoch_install_failed:
    case au::commit_refusal::mapping_install_failed:
        return {result::proof_failed, wire_stage::commit, wire_detail::commit_refused};
    }
    return {result::internal_error, wire_stage::commit, wire_detail::none};
}

outcome map_proof_refusal(pf::refusal p)
{
    switch (p) {
    case pf::refusal::none:
        return {result::internal_error, wire_stage::proof_evaluation, wire_detail::none};

    /* The firmware's own orchestration lost track of its challenge, or skipped a step. Not
     * proof_failed: an operator sent to the harness by these would be sent to the wrong place. */
    case pf::refusal::missing_evidence:
    case pf::refusal::challenge_invalid:
    case pf::refusal::challenge_stale:
    case pf::refusal::challenge_consumed:
        return {result::internal_error, wire_stage::proof_evaluation, wire_detail::none};

    /* The chain was never asked anything; the description it would have been asked against is
     * wrong. walk_spec_rejected sits here too, even though it surfaces during a walk. */
    case pf::refusal::spec_not_commissioning_profile:
    case pf::refusal::spec_no_tail_l4:
    case pf::refusal::spec_no_cliff:
    case pf::refusal::spec_too_few_positions:
        return {result::misconfigured, wire_stage::not_started, wire_detail::none};
    case pf::refusal::walk_spec_rejected:
        return {result::misconfigured, wire_stage::first_walk, wire_detail::none};

    case pf::refusal::walk_position_count:
    case pf::refusal::walk1_not_complete:
    case pf::refusal::l4_retained:
        return {result::proof_failed, wire_stage::first_walk, wire_detail::walk_mismatch};
    case pf::refusal::walk2_not_complete:
        return {result::proof_failed, wire_stage::second_walk, wire_detail::walk_mismatch};
    case pf::refusal::position_not_verified:
        return {result::proof_failed, wire_stage::first_walk, wire_detail::position_silent};
    case pf::refusal::address_mismatch:
    case pf::refusal::address_not_distinct:
    case pf::refusal::identity_mismatch:
        return {result::proof_failed, wire_stage::first_walk, wire_detail::identity_disagreed};
    case pf::refusal::role_unknown:
        return {result::misconfigured, wire_stage::proof_evaluation, wire_detail::none};
    case pf::refusal::role_duplicate:
        return {result::proof_failed, wire_stage::proof_evaluation, wire_detail::identity_disagreed};
    case pf::refusal::fingerprint_mismatch:
        return {result::proof_failed, wire_stage::second_walk, wire_detail::walk_mismatch};

    case pf::refusal::isolation_not_attempted:
        return {result::internal_error, wire_stage::tail_isolation, wire_detail::none};
    case pf::refusal::isolation_transport_error:
    case pf::refusal::isolation_wrong_address:
    case pf::refusal::isolation_prev_addr_wrong:
    case pf::refusal::isolation_prev_answered:
        return {result::proof_failed, wire_stage::tail_isolation, wire_detail::tail_would_not_isolate};
    case pf::refusal::isolation_no_answer:
        return {result::proof_failed, wire_stage::tail_isolation, wire_detail::position_silent};
    case pf::refusal::isolation_identity:
        return {result::proof_failed, wire_stage::tail_isolation, wire_detail::identity_disagreed};
    }
    return {result::internal_error, wire_stage::proof_evaluation, wire_detail::none};
}

outcome map_stage(cm::stage s)
{
    switch (s) {
    case cm::stage::none:
        return {result::ok, wire_stage::complete, wire_detail::none};
    case cm::stage::not_configured:
        return {result::misconfigured, wire_stage::not_started, wire_detail::none};
    case cm::stage::epoch_out_of_range:
        /* Unreachable from the wire: wire_epoch is eight bits, so every value it can carry is inside
         * 0-255. Reaching it means the value was corrupted between decoding and the transaction,
         * which is an invariant failure and reads as one rather than as a host mistake. */
        return {result::internal_error, wire_stage::not_started, wire_detail::none};
    case cm::stage::quiesce_failed:
    case cm::stage::chain_busy:
        return {result::busy_chain, wire_stage::quiesce, wire_detail::chain_busy};
    case cm::stage::attempt_refused:
        return {result::proof_failed, wire_stage::proof_evaluation, wire_detail::commit_refused};
    case cm::stage::evidence_refused:
        return {result::proof_failed, wire_stage::proof_evaluation, wire_detail::walk_mismatch};
    case cm::stage::proof_speed_refused:
        return {result::proof_failed, wire_stage::first_walk, wire_detail::proof_speed_refused};
    case cm::stage::product_speed_refused:
        return {result::proof_failed, wire_stage::retime, wire_detail::product_speed_refused};
    case cm::stage::identity_recheck_failed:
        return {result::proof_failed, wire_stage::identity_recheck, wire_detail::identity_disagreed};
    case cm::stage::commit_refused:
        return {result::proof_failed, wire_stage::commit, wire_detail::commit_refused};
    }
    return {result::internal_error, wire_stage::not_started, wire_detail::none};
}

outcome map_result(const cm::outcome &r)
{
    /* The sub-enum is consulted only where the stage says it carries the reason. Consulting it
     * unconditionally would let a stale `none` from an unrelated field decide the wire value. */
    switch (r.failed_at) {
    case cm::stage::attempt_refused:
        return map_begin_refusal(r.begin);
    case cm::stage::evidence_refused:
        return map_proof_refusal(r.proof);
    case cm::stage::commit_refused:
        return map_commit_refusal(r.commit);
    case cm::stage::none:
    case cm::stage::not_configured:
    case cm::stage::epoch_out_of_range:
    case cm::stage::quiesce_failed:
    case cm::stage::chain_busy:
    case cm::stage::proof_speed_refused:
    case cm::stage::product_speed_refused:
    case cm::stage::identity_recheck_failed:
        return map_stage(r.failed_at);
    }
    return {result::internal_error, wire_stage::not_started, wire_detail::none};
}

} // namespace lexxhard::tof_commission_map

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
