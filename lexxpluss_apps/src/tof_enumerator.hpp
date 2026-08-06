/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

// ToF chain enumeration state machine (AMRSW-2322, Phase 2).
//
// Pure logic over an injected operation interface: no Zephyr, no bus, no
// timing numbers. Every decision below is backed by measurements on DS20001
// (2026-08-05/06, see progress notes) and by the carrier design notes:
//
//   - The enable chain is a distributed D flip-flop, one per board, sharing
//     one clock. Position 1's enable is the raw data line; each board's FF
//     forwards the enable to the next. Cumulative enumeration (data held
//     high, one new board per pulse) is the flow the carrier notes
//     prescribe and the one verified on the robot.
//   - Enable semantics differ per model, MEASURED: the L7 board gates comms
//     only, so an L7 keeps a programmed address across enable-low while
//     powered (and is silent while disabled); the L4 board's enable is
//     reset-class (XSHUT), so an L4 answering anywhere but the default
//     address after an all-off is an anomaly, not a convenience.
//   - The serial data line settles slowly at the register end (~seconds on
//     DS20001). The state machine never encodes a duration: it asks the ops
//     layer to wait, and the ops layer owns the number. The measured >=2 s
//     is a DS20001 diagnostic workaround, not a product timing; the final
//     value comes from hardware documentation or a scope (Request B).
//
// TWO SAFETY INVARIANTS, and what each one cannot do alone:
//
//   1. VACANCY: the shared default address 0x29 must be PROVABLY vacant --
//      a clean NACK, not merely "no answer" -- before the clock advances.
//      Both models ship at 0x29; a pulse issued while a device may still
//      sit there merges two devices onto one address, the wired-AND answers
//      look healthy, and every later step reasons from a lie.
//   2. CENSUS: vacancy alone does not prove identity. Counterexample: an L7
//      from position 1 retained at position 2's target answers neither
//      0x29 nor position 1's target -- vacancy holds, yet adopting the
//      later ACK as "position 2 retained" would misattribute the device.
//      So every activation probes the WHOLE watched set (the default, every
//      configured target, any injected watch addresses) and additionally
//      re-verifies that already-verified devices still answer where they
//      were left. Any ACK without an owner, or any verified device gone
//      silent, freezes the chain.
//
// A probe can only feed either invariant when the transport can tell a
// clean NACK from a timeout or bus error. On the stock Zephyr STM32 driver
// all three come back as -EIO (measured); a duration heuristic is NOT a
// safety proof. The ops layer must provide the distinction (driver-level
// classification is a known follow-up); until it can, it must report
// transport_error, and this machine will freeze rather than guess.

#include <stdint.h>
#include <stddef.h>

namespace lexxhard::tof_enum {

enum class model : uint8_t { l7cx, l4cx };

struct id_bytes {
    uint8_t first{0};   // l7cx: device_id (expect 0xf0); l4cx: model_id (expect 0xeb)
    uint8_t second{0};  // l7cx: revision  (expect 0x02); l4cx: module_type (expect 0xaa)
};

// A probe answer the machine can reason about. Only a clean NACK proves an
// address vacant; transport failures prove nothing and freeze the run.
enum class probe_state : uint8_t { ack, nack, transport_error };

struct probe_result {
    probe_state state{probe_state::transport_error};
    int rc{0};  // errno detail for transport_error (0 for ack/nack)
};

// Failure stages of the guarded readdress, the union of the L7 and L4
// helper stages (tof_diag_readdress) so the per-position result can carry
// WHERE the move failed, not just that it failed.
enum class readdress_stage : uint8_t {
    none,          // success
    validate,      // argument rejection, no bus traffic
    collision,     // target already ACKs, write not attempted
    page_select,   // L7 only
    addr_write,    // L7 register 0x0004 / L4 register 0x0001 write failed
    verify,        // id read on the new address failed or mismatched
    page_restore,  // L7 only
};

struct readdress_result {
    int rc{0};
    readdress_stage failed_at{readdress_stage::none};
};

// The six injected operations. Control operations return status: a failed
// data-line write or clock pulse means the machine's belief about which
// positions are enabled is wrong, and a wrong belief here turns into a
// wrong absent/identity verdict later -- so control failure freezes the
// run immediately. wait() cannot fail and stays void; the ops layer owns
// every duration.
//
// readdress() must not weaken the tri-state guarantee from inside: its
// internal collision check may treat the target as free ONLY on a clean
// NACK, and a transport error there forbids the address write outright --
// the write count must be provably zero in that case. The diag-era helpers
// treat any non-zero probe rc as "free" and MUST NOT back this operation,
// not even behind a pre-probing adapter (they would probe again and
// re-weaken the check); the production helper is tof_readdress, which takes
// the tri-state probe natively and has its own exact-traffic tests for
// these properties. The machine additionally
// defends in depth: it has already proven the target clean-NACK in the
// census before it ever calls readdress(), and after a successful move it
// re-runs the census BEFORE the next pulse -- default clean NACK, new
// target ACK, every verified device still ACK, every unowned address still
// clean NACK -- so "readdress succeeded" is never silently substituted for
// "default proven vacant".
struct chain_ops {
    virtual probe_result probe(uint8_t addr7) = 0;
    virtual int read_id(model m, uint8_t addr7, id_bytes &out) = 0;
    virtual readdress_result readdress(model m, uint8_t old7, uint8_t new7) = 0;
    virtual int set_data(bool level) = 0;
    virtual int pulse_clock() = 0;
    enum class wait_reason : uint8_t { data_settle, sensor_boot };
    virtual void wait(wait_reason reason) = 0;
    virtual ~chain_ops() = default;
};

// Logical mounting role of a drop-sense board. Electrical enumeration can
// prove the TYPE sequence but never the mounting role of four identical
// boards; the role table is injected from the frozen mapping document
// (Request C) and defaults to unknown. The state machine never guesses.
enum class l4_role : uint8_t { unknown, front_left, rear_left, rear_right, front_right };

struct position_spec {
    model expected{model::l4cx};
    uint8_t target_addr{0};   // assigned address after enumeration
    // Hanging sources only: index into the wire contract's source table
    // (contract-owned mapping, injected -- never derived arithmetically).
    // -1 for positions that feed no grid source.
    int8_t source_id{-1};
    l4_role role{l4_role::unknown};
};

// The shared factory-default address is a property of the sensors, not of a
// deployment: it is a constant, deliberately not configurable.
inline constexpr uint8_t kDefaultAddr{0x29};

struct chain_spec {
    static constexpr size_t kMaxPositions{8};
    static constexpr size_t kMaxWatch{4};
    size_t positions{0};
    position_spec at[kMaxPositions]{};
    // Pulses used by the all-off reset; the carrier notes' rule is
    // positions - 1, validation requires at least that.
    uint8_t alloff_pulses{8};
    // Extra addresses that must stay silent throughout (e.g. legacy
    // assignments from an older scheme). Any ACK here is unexpected_address.
    //
    // SCOPE, stated honestly: the census covers the default, the configured
    // targets and THIS injected set -- it is not a sweep of the whole 7-bit
    // space, and the interface has no channel for reporting strays outside
    // it. Deployment obligation: every address a device could plausibly
    // retain (previous assignment schemes, diagnostic experiments) MUST be
    // listed here.
    uint8_t watch_addrs[kMaxWatch]{};
    size_t watch_count{0};
    // Product chains carry both hanging sources; validation requires source
    // 0 and source 1 to each appear exactly once unless a bench spec
    // explicitly relaxes this.
    bool require_all_sources{true};
};

// Spec validation failures. enumerate() checks the spec BEFORE any hardware
// operation; on any of these it returns with zero ops calls.
enum class spec_error : uint8_t {
    none,
    position_count,       // 0 or > kMaxPositions
    alloff_pulses,        // < positions - 1
    target_invalid,       // not a usable 7-bit address (0x08..0x77)
    target_is_default,    // target equals the default address
    target_duplicate,     // two positions share a target
    source_id_range,      // outside 0..kMaxSources-1
    source_id_duplicate,  // two positions share a source
    source_on_non_l7,     // a grid source attached to a drop-sense board
    role_on_non_l4,       // a mounting role attached to an L7
    watch_invalid,        // watch address invalid or colliding with
                          //   the default / a target
    watch_count,          // watch_count > kMaxWatch
    watch_duplicate,      // the watch set repeats an address
    no_source,            // no position carries a hanging source at all
    sources_incomplete,   // require_all_sources and source 0 or 1 missing
};

// Per-position verdicts. D = default address, T = this position's target,
// census = the full watched-set sweep described above.
enum class outcome : uint8_t {
    enumerated,          // D ack, T nack, census clean; id at D matches;
                         //   guarded readdress succeeded
    retained,            // L7 only: T ack, D nack, census clean; id at T
                         //   matches (address kept across enable-low while
                         //   powered -- measured semantics)
    absent,              // the default, this position's target, every
                         //   UNOWNED target and every watch address answer
                         //   with a clean NACK, every OWNED target still
                         //   ACKs, and no probe hit a transport error:
                         //   board missing or enable never arrived (the
                         //   pos6 failure class). Vacancy proven -> the
                         //   chain MAY advance.
    ambiguous_identity,  // D ack and T ack: two candidates, identity cannot
                         //   be guessed. FREEZES.
    unexpected_retained, // L4 only: T ack after a completed all-off -- the
                         //   L4 enable is reset-class, so this contradicts
                         //   the chain state model. FREEZES.
    unexpected_address,  // an ACK with no owner: a not-yet-assigned target
                         //   (other than the retained-T case above) or a
                         //   watch address answered. FREEZES.
    verified_device_missing, // a previously verified device no longer
                         //   answers at its address. FREEZES.
    wrong_model,         // a device answered but its id bytes do not match
                         //   the expected model: physical order differs from
                         //   the spec, and the device still sits on D or T.
                         //   FREEZES.
    transport_failed,    // a probe or id read came back transport_error:
                         //   neither vacancy nor identity is provable.
                         //   FREEZES.
    readdress_failed,    // the guarded move failed (stage recorded); the
                         //   device may still sit on D. FREEZES.
    control_failed,      // set_data or pulse_clock reported failure: the
                         //   enable state is no longer known. FREEZES.
    not_attempted,       // a freeze happened earlier (or spec was invalid).
};

enum class chain_status : uint8_t {
    complete,   // every position enumerated or retained
    degraded,   // at least one anomaly, but at least one hanging source is
                //   verified and allowed to produce grids. Degradation is
                //   explicit: callers must surface it, not merely keep
                //   running.
    failed,     // invalid spec, frozen before any hanging source was
                //   verified, or no hanging source usable
};

struct position_result {
    outcome verdict{outcome::not_attempted};
    id_bytes seen{};        // raw identity bytes when an id read succeeded
    uint8_t address{0};     // address the device answers on now (0 if none)
    // Commanded state only: there is no feedback line, so this records what
    // the machine asked for, never a measured pin level.
    bool enable_commanded_high{false};
    int rc{0};              // errno detail for the failure verdicts
    readdress_result readdress{};  // stage detail when verdict==readdress_failed
    uint8_t offending_addr{0};     // for unexpected_address /
                                   //   verified_device_missing: which address
};

// The final commanded state is part of the API, not an afterthought: the
// cleanup strategy on failure must be defined, because an exception path
// that leaves the chain in an unknown state is more dangerous than the
// failure itself. Policy: after the initial all-off, the data line goes
// high and is NEVER driven low again by this machine -- on any freeze it
// stops clocking and leaves everything as it is, so already-verified
// positions stay enabled on their verified addresses (evidence preserved,
// and the healthy hanging sensors keep working). Recovery from a frozen
// chain is a fresh run (which starts with all-off), not an in-place repair.
struct chain_result {
    chain_status status{chain_status::failed};
    spec_error spec{spec_error::none};
    size_t positions{0};
    position_result at[chain_spec::kMaxPositions]{};
    // Commanded state on return -- what was ASKED of the hardware, never a
    // measurement. Meaningful only while control_state_known: after a
    // control_failed freeze the hardware may not have executed the last
    // request, so consumers must treat the commanded values as unknown.
    bool control_state_known{true};
    bool data_commanded_high{false};
    uint8_t pulses_issued{0};    // excludes the all-off burst
    // interpretation
    int8_t frozen_at{-1};        // -1 none; 0 = setup phase (all-off/start);
                                 //   1..N = 1-based position of the freeze
    int8_t interrupted_at{-1};   // first position of an absent-run reaching
                                 //   the tail: enable propagates through each
                                 //   board's FF, so downstream absences after
                                 //   a break are not independent evidence
    // Grid production permission per hanging source, indexed by source_id.
    // Granted when the source's position verifies (enumerated/retained) and
    // REVOKED by the fixed table below -- per-position verdicts are
    // historical records of what happened at that step; final usability is
    // this array and nothing else (a position may still read `enumerated`
    // although its device vanished later in the run).
    //
    //   control_failed            -> revoke ALL (control state unknown)
    //   transport_failed          -> revoke ALL (bus reliability unknown)
    //   verified_device_missing   -> revoke the source owning the offending
    //                                address (none if it was not a source)
    //   current source position fails (wrong_model / readdress_failed /
    //   post-census violation)    -> that source never granted; sources that
    //                                passed a clean census earlier KEEP
    //   drop-sense position anomaly (absent / wrong_model /
    //   unexpected_retained ...)  -> no revocation of verified L7 sources
    //
    // Status follows from the array: no source allowed -> failed; all
    // positions clean -> complete; anything else -> degraded.
    static constexpr size_t kMaxSources{2};
    bool source_allowed[kMaxSources]{false, false};
};

// Runs the whole enumeration:
//
//   validate: spec checks above; any violation returns invalid, ZERO ops
//   all-off:  set_data(low), wait(data_settle), alloff_pulses x pulse_clock
//   start:    set_data(high), wait(data_settle)
//   per position K (1-based; K=1 is enabled by the data line itself,
//   K>1 by one pulse_clock each):
//       wait(sensor_boot)
//       census: probe(default), probe(every target), probe(every watch)
//       check verified devices still answer; classify per the outcome table
//       identity work (read_id / guarded readdress) as the table requires
//       after a successful readdress: census AGAIN before the next pulse
//       (default clean NACK, new target ACK, verified devices ACK, unowned
//       addresses clean NACK); a post-census violation freezes exactly like
//       a pre-census one
//       advance only while vacancy AND census both hold
//
// Hot-restart is not a separate mode: the census at every position makes a
// cold chain, a rebooted-SCB chain (L7s retained and silent-until-enabled,
// L4s back at the default) and mixed states fall out of the same table --
// including the misattribution counterexample above, which the census
// catches as unexpected_address at position 1.
chain_result enumerate(chain_ops &ops, const chain_spec &spec);

}  // namespace lexxhard::tof_enum
