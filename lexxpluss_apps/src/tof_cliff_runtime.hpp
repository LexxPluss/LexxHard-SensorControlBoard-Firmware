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
 * So: ONE bootstrap, ONE set of static storage, ONE order. Both callers go through here.
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
#include "tof_cliff_stream_loop.hpp"
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
    /* The four L4s' ranging profile, injected for the same reason as the periods: until the
     * acquisition layer could take one, configure() was a no-op and the parts ran on whatever
     * VL53LX_DataInit left. Zero, or a mode outside the ULD's three, is refused. */
    uint32_t cliff_timing_budget_us{0};
    uint8_t cliff_distance_mode{0};
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

/* One diagnostic read of one already-enumerated cliff position.
 *
 * WHY THIS EXISTS AND WHAT IT IS NOT. A chain with fewer than four cliff sensors cannot reach
 * PROVEN -- is_commissioning_profile() requires six positions with all four corners present exactly
 * once -- so no measurement frame can be produced or published on a partial chain, and that is
 * deliberate. This answers a different and legitimate question: does THIS sensor range at all.
 *
 * It publishes nothing, claims nothing about the mapping, and touches neither the authority nor
 * the acquisition thread. It is the sensor-layer counterpart of `tof enum`: a commissioning diagnostic whose output
 * goes to the operator, not to the bus.
 *
 * It drives the PRODUCTION ops table (acq::l4_cliff_ops via the installed descriptor), so it
 * exercises the same open/configure/start/read/stop path acquisition uses rather than a second
 * route to the device. Note that configure is currently a no-op by design -- distance mode and
 * timing budget are unresolved in the wire contract -- so the reading reflects the vendor ULD's
 * default parameters.
 *
 * Refuses with -EBUSY while the acquisition thread runs: that thread owns the ULD, whose port keeps
 * one file-scope transport record, and a second caller inside it turns a transport error into a
 * good-looking sample. Requires the position to have been enumerated first (`tof enum`), because it
 * opens the descriptor's assigned address, not the factory default.
 *
 * ON attempts/gap_ms. tof_cliff_read_once() is by contract a single non-blocking data-ready check:
 * it never loops and never waits, because in production the scheduler -- not the sensor call -- owns
 * the decision about a cycle that produced no sample. A start immediately followed by one check
 * therefore reports "not ready" essentially always, which is the honest answer for one acquisition
 * cycle but a useless one for the bench question "does this sensor range".
 *
 * So the waiting lives here, in the diagnostic, and only here. attempts=1 with gap_ms=0 is the
 * default precisely because it reproduces what one acquisition cycle sees; an operator who wants a
 * distance has to ask for the wait explicitly. Both are bounded (see kMaxProbeAttempts and
 * kMaxProbeGapMs) because this loop sleeps while holding the chain lock.
 */
constexpr unsigned kMaxProbeAttempts{50};
constexpr unsigned kMaxProbeGapMs{200};

struct probe_result {
    bool attempted{false};
    int open_rc{0};
    int start_rc{0};
    int read_rc{0};
    unsigned attempts_used{0};
    tof_acq::op_status status{};
    struct tof_cliff_sample sample{};
    uint8_t addr_7bit{0};
    uint8_t role_id{0};
};

int probe_position(size_t position_1based, probe_result &out, unsigned attempts = 1,
                   unsigned gap_ms = 0);

#if defined(ENABLE_TOF_CLIFF_BENCH_PACK)
/* BENCH ONLY. Reads SEVERAL fresh frames from ONE open/configure/start session.
 *
 * probe_position() stops at the first fresh frame and then stops the sensor, so every L4 sample
 * this project has recorded has been the first frame after a restart, and every one of them
 * reported no target. That is not yet evidence that the sensors cannot range -- the existing
 * evidence has only ever observed frame one, so a first-frame effect cannot be ruled out either.
 * This command removes the reason we cannot tell the two apart. It decides nothing.
 *
 * The lifecycle is probe_position's, unchanged: same open, same configure, same start, same
 * read_once, same stop. Only the break is gone, which read_once already supports because it
 * re-arms the device after every fetch.
 *
 * Frames go to a sink as they arrive rather than into an array. The shell thread has well under
 * a kilobyte of stack headroom on this board and assertions are not compiled in, so a buffer of
 * frames on that stack would overflow it silently.
 *
 * Transmits nothing, touches no mapping, authorises nothing. */
struct stream_result {
    bool attempted{false};
    int open_rc{0};
    int start_rc{0};
    int last_read_rc{0};
    unsigned frames_collected{0};
    unsigned attempts_used{0};
    uint8_t addr_7bit{0};
    uint8_t role_id{0};
    tof_acq::op_status status{};
};

int stream_position(size_t position_1based, stream_result &out, unsigned want_frames,
                    unsigned gap_ms, unsigned max_attempts,
                    lexxhard::tof_cliff_stream::frame_sink sink, void *ctx);
#endif

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
