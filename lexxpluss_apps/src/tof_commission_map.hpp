/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The firmware's internal refusal enums, translated into wire values.
 *
 * WHY IT IS ITS OWN LAYER. `tof_commissioning::stage`, `commit_refusal`, `begin_refusal` and
 * `tof_proof::refusal` are internal: they are reordered when the code is refactored, and a
 * renumbering would silently change the meaning of evidence someone recorded months ago. Nothing
 * transmits them. This unit is the single place where an internal value becomes a wire value, so
 * there is one table to review and one place a new enumerator has to be answered for.
 *
 * EXHAUSTIVENESS IS A BUILD SETTING, NOT A LANGUAGE GUARANTEE. A switch over an enum with no
 * `default` is not an error in C++ -- it falls through. This translation unit is compiled with
 * `-Werror=switch-enum`, which is load-bearing rather than tidy: without it, adding an enumerator
 * produces a silent fall-through and the wire gains a meaning nobody wrote. The test suite proves
 * the flag works by compiling a mutation that adds an enumerator and showing the build fail, because
 * a guarantee resting on a flag needs the flag demonstrated rather than asserted.
 *
 * There is no "unknown" escape. `wire_stage::unknown` and `wire_detail::unknown` exist for a DECODER
 * meeting a newer firmware; a mapper that could emit them would be shipping the hole instead of
 * failing to build.
 */

#pragma once

#include "tof_commission_wire.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include "tof_commissioning.hpp"
#include "tof_mapping_authority.hpp"
#include "tof_mapping_proof.hpp"

namespace lexxhard::tof_commission_map {

namespace wire = tof_commission_wire;

struct outcome {
    wire::result res{wire::result::internal_error};
    wire::wire_stage stage{wire::wire_stage::not_started};
    wire::wire_detail detail{wire::wire_detail::none};
};

/* The whole translation, from the transaction's own result. The sub-enums are consulted only where
 * the stage says they carry the reason -- `attempt_refused` defers to begin_refusal,
 * `evidence_refused` to the proof's refusal, and `commit_refused` to the authority's. */
outcome map_result(const tof_commissioning::outcome &r);

/* Exposed for the golden-vector suite, which walks every enumerator of each enum rather than the
 * handful a transaction happens to produce. */
outcome map_stage(tof_commissioning::stage s);
outcome map_begin_refusal(tof_authority::begin_refusal b);
outcome map_commit_refusal(tof_authority::commit_refusal c);
outcome map_proof_refusal(tof_proof::refusal p);

} // namespace lexxhard::tof_commission_map

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
