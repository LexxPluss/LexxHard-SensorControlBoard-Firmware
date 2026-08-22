/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The one place the cliff subsystem is wired up.
 *
 * Before this existed there were two wirings: the shell command owned a chain_spec, and the B6
 * budget probe owned another one along with its own calls to tof_authority::init(),
 * tof_cliff_can::init(), tof_cliff_pub::init() and tof_acq::init(). Two wirings of the same
 * subsystem is not duplication to tidy up later -- it is two different machines. The probe's spec
 * was a different OBJECT from the shell's, so the authority compared a proof against one copy while
 * commissioning walked another; the probe's descriptors used addresses 0x30.. that the chain spec
 * never assigns; and a second tof_authority::init() would clear the installed mapping and any open
 * attempt, so the subsystem would behave differently in a budget build than in a product build --
 * the worst kind of difference, because the budget build is what gets measured and believed.
 *
 * So: ONE bootstrap, ONE set of static storage, ONE order.
 *
 * The budget probe has since been retired along with the whole staged ladder, so the second wiring
 * described above exists in no build any more and this is the only one. That does not make the
 * indirection redundant -- it is what stops the next caller from growing a second machine, which is
 * how the first one appeared.
 *
 * ORDER, and why it is this one:
 *
 *   authority   first, because the publisher's authorisation hook reads the authority's epoch and
 *               state. An uninitialised authority answers with an epoch no proof ever issued.
 *   CAN         next. Allowed to fail: on a board where can2 is not ready the rest of the
 *               subsystem must still come up, because the health path is what tells a consumer
 *               that this board has no bus rather than no sensors.
 *   publisher   next, so a sink exists before anything can produce a frame.
 *   acquisition last, because it is the only step that starts something running (the heartbeat).
 *
 * WHAT BOOTSTRAP DELIBERATELY DOES NOT DO: bring up the sensors, and (once it exists) start the
 * acquisition thread. The descriptors it installs carry no role_id yet -- role_id comes from a
 * mapping a proof installed, and inferring it from a descriptor's index is the exact defect this
 * subsystem exists to prevent. The heartbeat, by contrast, starts immediately and reports
 * NOT_READY: a consumer needs to hear "alive, unproven" from power-on, not silence until
 * commissioning happens.
 *
 * CONTEXT: init and teardown belong to ONE lifecycle context -- the main thread, before the
 * per-feature threads start. tof_acq's configured_/active_ flags are read outside the chain lock on
 * exactly that basis.
 */

#pragma once

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <stdint.h>

#include "tof_acquisition.hpp"
#include "tof_enumerator.hpp"

namespace lexxhard::tof_cliff_runtime {

/* How far the bootstrap got. One value, so a refusal can say which step failed rather than "not
 * ready" -- an operator who is told "not ready" goes looking at the chain, which is the wrong
 * place when the truth is that can2 never came up. */
enum class stage : uint8_t {
    not_started,
    chain_not_ready,   // the chain controller's glue never came up; nothing was wired
    authority_failed,
    publisher_failed,
    acquisition_failed,
    ready,
};

/* Timing, injected. Both remain unresolved symbols in the wire contract, so this layer has no
 * defaults and refuses zero: a placeholder invented here would become the specification by being
 * the only number anybody could find. Production reads them from the devicetree, where they are
 * required properties -- a build that has not stated them fails rather than picking something. */
struct config {
    uint32_t cycle_period_ms{0};
    uint32_t health_period_ms{0};
    /* The acquisition thread's, and injected for the same reason. The join timeout in particular
     * decides how long commissioning waits before refusing to run, which belongs to whoever owns the
     * deployment. Zero is refused for the first two here and by tof_acq::start() for the timeout. */
    uint32_t stop_join_timeout_ms{0};
    int thread_priority{0};
};

/* The value production uses. DEFINED only where the chain devicetree node exists -- which is every
 * build that can drive a chain, and no host test. Declared unconditionally so the declaration does
 * not need the same guard as the definition; a host suite simply never calls it and injects its own
 * config instead. */
config config_from_devicetree();

/* Runs the four steps in order, exactly once, and only after the chain controller's glue is up.
 *
 * Returns 0 when everything came up, -EALREADY if it has already run (and does NOT re-run any
 * step: a second tof_authority::init() would discard an installed mapping), -EINVAL for a config
 * with a zero period, -ENODEV when the chain glue is not initialised, or the failing step's errno.
 * On failure it stops at that step: a half-wired subsystem must not look ready.
 *
 * ONE call site in production, in tof_chain_controller::init(). The -ENODEV check enforces the half
 * of that rule which is about ORDER; the single-shot check enforces the half that is about count.
 */
int bootstrap(const config &cfg);

stage current_stage();
bool ready();
const char *stage_name(stage st);

/* THE chain spec -- one object, referenced by the authority and used by commissioning. Handing out
 * a reference rather than a copy is the point: the authority holds a pointer to this and compares
 * every proof against it, so a caller working from its own copy would be proving a different
 * chain. */
tof_enum::chain_spec &spec();

/* Are the descriptors keyed by the mapping the authority currently reports as PROVEN?
 *
 * There is deliberately no public "apply" entry point any more. Keying happens INSIDE the authority's
 * commit transaction, through the install_mapping callback registered by bootstrap(), and that is
 * what makes it safe: a caller that could key descriptors on its own could key them from a mapping
 * that was never proven, and the version of this module that let the shell do it after the commit
 * left a PROVEN authority whose descriptors described a different chain when the keying failed.
 *
 * The answer is computed from the authority, not from a latched flag: keys written under an epoch
 * that has since been revoked or superseded are as wrong as no keys at all, and a flag would still
 * be saying "applied".
 */
bool mapping_applied();

/* The value a descriptor carries until a proof has said what it is. Not zero: zero is front_left's
 * source id, and a descriptor that defaults to a real source id is a descriptor that lies quietly. */
constexpr uint8_t kRoleUnassigned{0xFF};

/* Brings up the sensors, and later starts the acquisition thread. Refuses with -EPERM unless the
 * bootstrap is ready AND mapping_applied() -- which means the authority is PROVEN right now, under
 * the same epoch the descriptors were keyed under. A cycle whose descriptors carry kRoleUnassigned,
 * or keys from a mapping that has since been revoked, produces facts nothing can be keyed by. */
int start_acquisition();

#ifdef CONFIG_ZTEST
// Lets a suite exercise the single-shot rule more than once per image.
void reset_for_test();
/* The acquisition thread's stack, for suites that have no devicetree to size one from. Injected
 * rather than defaulted: a fallback stack compiled in for tests would be a size nobody chose, and it
 * would be in the production image too. */
void set_thread_stack_for_test(k_thread_stack_t *stack, size_t size);
/* Read-only view of the descriptor table. The contents ARE the property under test -- addresses
 * taken from the spec, role_id absent until a proof installs one -- and there is no production
 * reason to expose them, so the door is test-only rather than a public accessor nobody needs. */
const tof_acq::source_desc *descriptors_for_test();
// Rebuilds the table from the current spec, for cases that need the two to disagree.
int force_rebuild_descriptors_for_test();
#endif

}  // namespace lexxhard::tof_cliff_runtime

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
