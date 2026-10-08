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

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/ztest.h>

#include <cstddef>
#include <cstdint>

#include "old_logic_oracle.hpp"
#include "state_graph.hpp"
#include "v_wheel_output.hpp"

namespace {

using lexxhard::push_policy;
using lexxhard::standard_policy;
using lexxhard::v_wheel_inputs;
using lexxhard::v_wheel_state;
using wheel_power_test::graph_event_count;
using wheel_power_test::graph_tracker;
using wheel_power_test::pollsWheelRelay;

// Events 0..7 are polls (bit0 ksw, bit1 wheel_poweroff, bit2 esw), events from poll_event_count on are enters.
constexpr int poll_event_count = 8;
constexpr int event_count = poll_event_count + 8;
// Array capacity of a sequence. The enumerations use their own limit.
constexpr int max_length = 6;
// Abstract enumeration: 16 events, length 5 (length 6 would take 16.7M sequences per run).
constexpr int abstract_length = 5;
constexpr int reachable_length = 6;
constexpr int write_capacity = 32;
constexpr int text_capacity = 512;

// The first 8 enter events are the abstract ones; the last 4 (OFF_WAIT, OFF, TIMEROFF, WAIT_SW) only exist in the graph
// enumeration. OFF maps to v_wheel_state::OFF, the others to v_wheel_state::OTHER.
constexpr v_wheel_state enter_states[12] = {
    v_wheel_state::POST,        v_wheel_state::STANDBY,       v_wheel_state::NORMAL,
    v_wheel_state::SUSPEND,     v_wheel_state::RESUME_WAIT,   v_wheel_state::AUTO_CHARGE,
    v_wheel_state::MANUAL_CHARGE, v_wheel_state::LOCKDOWN,    v_wheel_state::OTHER,
    v_wheel_state::OFF,         v_wheel_state::OTHER,         v_wheel_state::OTHER};

constexpr const char *enter_names[12] = {"POST",        "STANDBY",       "NORMAL",
                                         "SUSPEND",     "RESUME_WAIT",   "AUTO_CHARGE",
                                         "MANUAL_CHARGE", "LOCKDOWN",    "OFF_WAIT",
                                         "OFF",         "TIMEROFF",      "WAIT_SW"};

constexpr v_wheel_inputs inputsOf(int e)
{
    return v_wheel_inputs{((e >> 1) & 1) != 0, (e & 1) != 0, ((e >> 2) & 1) != 0};
}

// Fixed-capacity list of pin writes. Overflow fails the test instead of dropping data.
struct write_list {
    bool items[write_capacity]{};
    int size{0};

    void push(bool v)
    {
        zassert_true(size < write_capacity, "write list overflow");
        items[size++] = v;
    }
};

bool sameWrites(const write_list &a, const write_list &b)
{
    if (a.size != b.size) {
        return false;
    }
    for (int i = 0; i < a.size; ++i) {
        if (a.items[i] != b.items[i]) {
            return false;
        }
    }
    return true;
}

bool sameWrites(const write_list &a, const bool *expected, int expected_size)
{
    if (a.size != expected_size) {
        return false;
    }
    for (int i = 0; i < a.size; ++i) {
        if (a.items[i] != expected[i]) {
            return false;
        }
    }
    return true;
}

// Records every successful pin write together with the kind and inputs the oracle reported for it.
struct fake_writer {
    bool ready{true};
    bool pin_level{false};  // the pin starts at CUT
    write_list writes;
    oracle_write_kind kinds[write_capacity]{};
    v_wheel_inputs inputs[write_capacity]{};

    // Test-only: the old-logic oracle calls this right before each write.
    void noteNext(oracle_write_kind kind, v_wheel_inputs in)
    {
        pending_kind_ = kind;
        pending_inputs_ = in;
    }

    bool write(bool supplied, lexxhard::v_wheel_write_reason)
    {
        if (!ready) {
            return false;
        }
        writes.push(supplied);
        kinds[writes.size - 1] = pending_kind_;
        inputs[writes.size - 1] = pending_inputs_;
        pin_level = supplied;
        return true;
    }

    // Test-only: drops the write log but keeps the pin level.
    void clearLog() { writes.size = 0; }

    // Test-only: forces the pin to a level without logging a write.
    void forcePin(bool level) { pin_level = level; }

private:
    oracle_write_kind pending_kind_{oracle_write_kind::ENTER};
    v_wheel_inputs pending_inputs_{false, false};
};

// Fixed-capacity text buffer for failure messages.
struct text_buffer {
    char chars[text_capacity]{};
    int length{0};

    void append(const char *literal) { append("%s", literal); }

    template <typename First, typename... Rest>
    void append(const char *format, First first, Rest... rest)
    {
        const int room = text_capacity - length;
        if (room <= 1) {
            return;
        }
        const int n = snprintk(chars + length, static_cast<std::size_t>(room), format, first, rest...);
        if (n > 0) {
            length += (n < room) ? n : room - 1;
        }
    }
};

void appendEventName(text_buffer &out, int e)
{
    if (e < poll_event_count) {
        out.append("poll(wp=%d,ksw=%d,esw=%d) ", (e >> 1) & 1, e & 1, (e >> 2) & 1);
    } else {
        out.append("enter(%s) ", enter_names[e - poll_event_count]);
    }
}

void appendSequenceName(text_buffer &out, const int *seq, int length)
{
    for (int i = 0; i < length; ++i) {
        appendEventName(out, seq[i]);
    }
}

void appendWrites(text_buffer &out, const write_list &w)
{
    if (w.size == 0) {
        out.append("(none)");
        return;
    }
    for (int i = 0; i < w.size; ++i) {
        out.append("%c", w.items[i] ? '1' : '0');
    }
}

// Harness rules: the harness owns the current state and the latest inputs.
template <typename Dut>
class harness {
public:
    explicit harness(Dut &dut) : dut_(dut) {}

    // Returns true when the event reached the dut (a poll in a non-poll state is dropped).
    bool apply(int e)
    {
        if (e < poll_event_count) {
            inputs_ = inputsOf(e);
            if (isPollState(state_)) {
                dut_.poll(inputs_);
                return true;
            }
            return false;
        }
        state_ = enter_states[e - poll_event_count];
        dut_.enter(state_, inputs_);
        return true;
    }

    v_wheel_state state() const { return state_; }
    v_wheel_inputs inputs() const { return inputs_; }

private:
    static bool isPollState(v_wheel_state s)
    {
        return s == v_wheel_state::STANDBY || s == v_wheel_state::NORMAL || s == v_wheel_state::SUSPEND ||
               s == v_wheel_state::RESUME_WAIT;
    }

    Dut &dut_;
    v_wheel_state state_{v_wheel_state::POST};
    v_wheel_inputs inputs_{false, false};
};

// Whether the dut is fed the masked ksw in a ksw rise cycle. The new owner is (the wiring masks
// is_transition_to_running); the old logic used the raw ksw in that cycle.
template <typename Dut>
struct sees_masked_ksw {
    static constexpr bool value = true;
};

template <typename Writer, bool PushBuild>
struct sees_masked_ksw<old_logic_oracle<Writer, PushBuild>> {
    static constexpr bool value = false;
};

// Harness for sequences on the board state graph. A poll event reaches the dut only in a state that polls v_wheel, and
// in a ksw rise cycle an owner dut sees ksw=0 while the oracle sees the raw ksw. enter() keeps the raw ksw.
template <typename Dut>
class reachable_harness {
public:
    explicit reachable_harness(Dut &dut) : dut_(dut) {}

    // Returns true when the event reached the dut.
    bool apply(int e)
    {
        const bool polls = pollsWheelRelay(tracker_.state);
        tracker_.apply(e);
        if (e < poll_event_count) {
            inputs_ = inputsOf(e);
            if (!polls) {
                return false;
            }
            const bool masked = sees_masked_ksw<Dut>::value && tracker_.last_poll_rise;
            dut_inputs_ = v_wheel_inputs{inputs_.wheel_poweroff, inputs_.ksw_running && !masked, inputs_.esw_asserted};
            dut_.poll(dut_inputs_);
            return true;
        }
        dut_inputs_ = inputs_;
        dut_.enter(enter_states[e - poll_event_count], dut_inputs_);
        return true;
    }

    // The inputs the dut saw for the last event that reached it.
    v_wheel_inputs inputs() const { return dut_inputs_; }

private:
    Dut &dut_;
    graph_tracker tracker_;
    v_wheel_inputs inputs_{false, false};
    v_wheel_inputs dut_inputs_{false, false};
};

// Depth-first, prefix-first lexicographic enumeration of all sequences of length 1..limit.
// The visitor is called as visit(seq, length) and returns false to stop.
template <typename Visitor>
bool enumerate(int (&seq)[max_length], int length, int limit, Visitor &visit)
{
    for (int e = 0; e < event_count; ++e) {
        seq[length] = e;
        const int new_length = length + 1;
        if (!visit(static_cast<const int *>(seq), new_length) ||
            (new_length < limit && !enumerate(seq, new_length, limit, visit))) {
            return false;
        }
    }
    return true;
}

// Same order as enumerate(), but only events that are possible on the board state graph.
template <typename Visitor>
bool enumerateReachable(int (&seq)[max_length], int length, int limit, const graph_tracker &tracker,
                        Visitor &visit)
{
    for (int e = 0; e < graph_event_count; ++e) {
        if (!tracker.allowed(e)) {
            continue;
        }
        seq[length] = e;
        graph_tracker next = tracker;
        next.apply(e);
        const int new_length = length + 1;
        if (!visit(static_cast<const int *>(seq), new_length) ||
            (new_length < limit && !enumerateReachable(seq, new_length, limit, next, visit))) {
            return false;
        }
    }
    return true;
}

template <typename Visitor>
void enumerateReachableAll(Visitor &visit)
{
    int seq[max_length] = {};
    enumerateReachable(seq, 0, reachable_length, graph_tracker{}, visit);
}

template <typename Visitor>
void enumerateAll(Visitor &visit)
{
    int seq[max_length] = {};
    enumerate(seq, 0, abstract_length, visit);
}

template <typename Dut>
fake_writer runWriter(const int *seq, int length)
{
    fake_writer writer;
    Dut dut(writer);
    harness<Dut> h(dut);
    for (int i = 0; i < length; ++i) {
        h.apply(seq[i]);
    }
    return writer;
}

template <typename Dut>
write_list runRaw(const int *seq, int length)
{
    return runWriter<Dut>(seq, length).writes;
}

template <typename Policy>
struct is_push_policy {
    static constexpr bool value = false;
};

template <>
struct is_push_policy<push_policy> {
    static constexpr bool value = true;
};

template <typename Policy>
constexpr const char *policyName()
{
    return is_push_policy<Policy>::value ? "push" : "standard";
}

template <typename Policy>
using owner_t = lexxhard::v_wheel_output<fake_writer, Policy>;
// The oracle of the build that goes with the policy: the Push policy replaces the ENABLE_PUSH_MODE build.
template <typename Policy>
using oracle_for = old_logic_oracle<fake_writer, is_push_policy<Policy>::value>;

using std_owner_t = owner_t<standard_policy>;
using std_oracle_t = old_logic_oracle<fake_writer, false>;

// Per-write context of a run: which event produced the write and what the dut saw.
struct write_trace {
    struct entry {
        int event{0};               // index of the event in the sequence
        bool is_enter{false};
        bool is_resume_wait{false};  // the event is enter(RESUME_WAIT)
        bool ksw_now{false};         // ksw_running of the inputs given with the event
        bool ksw_prev{false};        // ksw_running of the inputs given with the previous event that reached the dut
    };
    entry items[write_capacity]{};
    // Per event: whether it reached the dut and the ksw_running the dut saw, now and for the previous such event.
    bool event_reached[max_length]{};
    bool event_ksw[max_length]{};
    bool event_ksw_prev[max_length]{};
    v_wheel_inputs event_inputs[max_length]{};  // the inputs the dut saw for the event
};

// Like runWriter, and also records the context of every write.
template <typename Dut, template <typename> class Harness = harness>
fake_writer runTraced(const int *seq, int length, write_trace &trace)
{
    fake_writer writer;
    Dut dut(writer);
    Harness<Dut> h(dut);
    bool ksw_prev = false;  // inputs start as {false, false}
    for (int i = 0; i < length; ++i) {
        const int size_before = writer.writes.size;
        const bool reached = h.apply(seq[i]);
        if (!reached) {
            continue;
        }
        const bool ksw_now = h.inputs().ksw_running;
        trace.event_reached[i] = true;
        trace.event_ksw[i] = ksw_now;
        trace.event_ksw_prev[i] = ksw_prev;
        trace.event_inputs[i] = h.inputs();
        for (int w = size_before; w < writer.writes.size; ++w) {
            write_trace::entry &t = trace.items[w];
            t.event = i;
            t.is_enter = seq[i] >= poll_event_count;
            t.is_resume_wait =
                t.is_enter && enter_states[seq[i] - poll_event_count] == v_wheel_state::RESUME_WAIT;
            t.ksw_now = ksw_now;
            t.ksw_prev = ksw_prev;
        }
        ksw_prev = ksw_now;
    }
    return writer;
}

// Applies events to a fresh oracle and returns the raw writes.
write_list oracleWrites(const int *seq, int length) { return runRaw<std_oracle_t>(seq, length); }

constexpr int poll_event(bool wp, bool ksw, bool esw = false)
{
    return (esw ? 4 : 0) | (wp ? 2 : 0) | (ksw ? 1 : 0);
}
constexpr int enter_event(v_wheel_state s) { return poll_event_count + static_cast<int>(s); }

enum class mode { FORCE_CUT, DECIDE, HOLD };

mode modeOf(v_wheel_state s)
{
    switch (s) {
    case v_wheel_state::POST:
    case v_wheel_state::OFF:
    case v_wheel_state::MANUAL_CHARGE:
    case v_wheel_state::LOCKDOWN:
        return mode::FORCE_CUT;
    case v_wheel_state::STANDBY:
    case v_wheel_state::NORMAL:
    case v_wheel_state::SUSPEND:
    case v_wheel_state::RESUME_WAIT:
        return mode::DECIDE;
    case v_wheel_state::AUTO_CHARGE:
    case v_wheel_state::OTHER:
        return mode::HOLD;
    }
    return mode::HOLD;
}

enum class defect { NONE, A, B, C, D };

// Defects of the old logic, judged on the oracle's i-th write:
// A: enter wrote SUPPLIED while wheel_poweroff=1 and ksw running.
// B: enter wrote SUPPLIED while wheel_poweroff=1 and ksw not running.
// C: a poll change-detection write of SUPPLIED that the same poll immediately cuts (maintenance).
// D: the dcdc write of enter(STANDBY) wrote SUPPLIED while the target is CUT (wheel_poweroff=1 or ksw not running).
defect classifyWrite(const fake_writer &w, int i)
{
    if (!w.writes.items[i]) {
        return defect::NONE;
    }
    if (w.kinds[i] == oracle_write_kind::ENTER_DCDC) {
        return (w.inputs[i].wheel_poweroff || !w.inputs[i].ksw_running) ? defect::D : defect::NONE;
    }
    if (w.kinds[i] == oracle_write_kind::ENTER && w.inputs[i].wheel_poweroff) {
        return w.inputs[i].ksw_running ? defect::A : defect::B;
    }
    if (w.kinds[i] == oracle_write_kind::POLL_CHANGE && i + 1 < w.writes.size &&
        w.kinds[i + 1] == oracle_write_kind::POLL_MAINTENANCE && !w.writes.items[i + 1]) {
        return defect::C;
    }
    return defect::NONE;
}

// Per-event view of a run. after[e] is the pin level after event e (initially CUT); changes[e] holds the level
// changes made by the writes of event e. An event that did not reach the dut keeps the previous level.
struct event_log {
    bool after[max_length]{};
    write_list changes[max_length];
};

event_log makeEventLog(const fake_writer &run, const write_trace &trace, int length)
{
    event_log log;
    bool level{false};
    int w = 0;
    for (int e = 0; e < length; ++e) {
        while (w < run.writes.size && trace.items[w].event == e) {
            if (run.writes.items[w] != level) {
                level = run.writes.items[w];
                log.changes[e].push(level);
            }
            ++w;
        }
        log.after[e] = level;
    }
    return log;
}

// Returns the first event where the pin level or the level changes differ, or -1 when there is none.
int firstDifferingEvent(const event_log &a, const event_log &b, int length)
{
    for (int e = 0; e < length; ++e) {
        if (a.after[e] != b.after[e] || !sameWrites(a.changes[e], b.changes[e])) {
            return e;
        }
    }
    return -1;
}

// Returns the defect of the oracle's writes made by event e. A, B and C take precedence over D, so D only counts
// events whose enter writes carry no A/B/C defect.
defect oracleDefectInEvent(const fake_writer &run, const write_trace &trace, int e)
{
    defect found = defect::NONE;
    for (int w = 0; w < run.writes.size; ++w) {
        if (trace.items[w].event != e) {
            continue;
        }
        const defect d = classifyWrite(run, w);
        if (d == defect::D) {
            found = d;
        } else if (d != defect::NONE) {
            return d;
        }
    }
    return found;
}

// True when the oracle's change-detection write of SUPPLIED (wheel_poweroff 1->0) in event e happens in a ksw rise
// cycle (the dut saw ksw 0 before and 1 now) while the owner saw the masked ksw=0 in that cycle.
bool isKswRiseWpRelease(const fake_writer &oracle_run, const write_trace &oracle_trace, const write_trace &owner_trace,
                        int e)
{
    if (!oracle_trace.event_reached[e] || !oracle_trace.event_ksw[e] || oracle_trace.event_ksw_prev[e] ||
        !owner_trace.event_reached[e] || owner_trace.event_ksw[e]) {
        return false;
    }
    for (int w = 0; w < oracle_run.writes.size; ++w) {
        if (oracle_trace.items[w].event == e && !oracle_trace.items[w].is_enter && oracle_run.writes.items[w] &&
            oracle_run.kinds[w] == oracle_write_kind::POLL_CHANGE) {
            return true;
        }
    }
    return false;
}

// Returns the index of the first level-changing write made by event e, or -1.
int firstChangingWrite(const fake_writer &run, const write_trace &trace, int e)
{
    bool level{false};
    for (int w = 0; w < run.writes.size; ++w) {
        if (run.writes.items[w] != level) {
            if (trace.items[w].event == e) {
                return w;
            }
            level = run.writes.items[w];
        }
    }
    return -1;
}

void printExample(const char *variant, const char *policy, const char *label, const int *s, int length, int e,
                  const event_log &owner, const event_log &oracle)
{
    text_buffer text;
    appendSequenceName(text, s, length);
    text.append("\n  e*=%d", e);
    text.append(" owner after=%d changes=", owner.after[e] ? 1 : 0);
    appendWrites(text, owner.changes[e]);
    text.append(" oracle after=%d changes=", oracle.after[e] ? 1 : 0);
    appendWrites(text, oracle.changes[e]);
    printk("stageB-diff[%s/%s] %s example: %s\n", variant, policy, label, text.chars);
}

bool anySupplied(const write_list &w, int from)
{
    for (int i = from; i < w.size; ++i) {
        if (w.items[i]) {
            return true;
        }
    }
    return false;
}

constexpr int table_state_count = 10;

constexpr v_wheel_state table_states[table_state_count] = {
    v_wheel_state::POST,          v_wheel_state::STANDBY,   v_wheel_state::NORMAL,
    v_wheel_state::SUSPEND,       v_wheel_state::RESUME_WAIT, v_wheel_state::AUTO_CHARGE,
    v_wheel_state::MANUAL_CHARGE, v_wheel_state::LOCKDOWN,  v_wheel_state::OTHER,
    v_wheel_state::OFF};

constexpr const char *table_state_names[table_state_count] = {"POST",        "STANDBY",       "NORMAL",
                                                              "SUSPEND",     "RESUME_WAIT",   "AUTO_CHARGE",
                                                              "MANUAL_CHARGE", "LOCKDOWN",    "OTHER",
                                                              "OFF"};

// Records where the invariant check failed.
void appendViolation(text_buffer &out, int event, bool pin, bool target)
{
    out.append("\n  at event %d: pin=%d target=%d", event, pin ? 1 : 0, target ? 1 : 0);
}

// Returns true when the invariant `which` (1..5) is violated by the sequence. `why` gets the violating event.
template <typename Policy>
bool violatesInvariant(int which, const int *seq, int length, text_buffer &why)
{
    fake_writer writer;
    owner_t<Policy> dut(writer);
    harness<owner_t<Policy>> h(dut);
    for (int i = 0; i < length; ++i) {
        const int size_before = writer.writes.size;
        const bool pin_before = writer.pin_level;
        if (!h.apply(seq[i])) {
            continue;
        }
        const mode m = modeOf(h.state());
        const bool is_enter = seq[i] >= poll_event_count;
        const v_wheel_inputs in = h.inputs();
        const bool target = Policy::target(in);
        bool violated = false;
        switch (which) {
        case 1:
            violated = m == mode::DECIDE && writer.pin_level != target;
            break;
        case 2:
            violated = m == mode::FORCE_CUT && writer.pin_level;
            break;
        case 3:
            violated =
                ((m == mode::DECIDE && !target) || m == mode::FORCE_CUT) && anySupplied(writer.writes, size_before);
            break;
        case 4:
            // A ROS cut that the ESW does not override must not be undone by the enter.
            violated = is_enter && m == mode::DECIDE && in.wheel_poweroff && !in.esw_asserted && writer.pin_level;
            break;
        case 5:
            violated = is_enter && m == mode::HOLD &&
                       (writer.writes.size != size_before || writer.pin_level != pin_before);
            break;
        default:
            break;
        }
        if (violated) {
            appendViolation(why, i, writer.pin_level, target);
            return true;
        }
    }
    return false;
}

// Enumerates every sequence and reports the first one that violates the invariant.
template <typename Policy>
bool invariantHolds(int which)
{
    bool violated = false;
    text_buffer text;
    auto visit = [&](const int *s, int length) {
        text_buffer why;
        if (!violatesInvariant<Policy>(which, s, length, why)) {
            return true;
        }
        violated = true;
        appendSequenceName(text, s, length);
        text.append("%s", why.chars);
        return false;
    };
    enumerateAll(visit);
    if (violated) {
        printk("invariant T5-%d [%s] first violation: %s\n", which, policyName<Policy>(), text.chars);
    }
    return !violated;
}

// Compares the owner with the oracle on one sequence at a time and classifies the first differing event.
// The first matching rule wins, so the classes never overlap:
//   no differing event: redundant_only_diff when only the raw write lists differ
//   1. a, b, c, d: defect of the oracle's writes in the event (a, b, c take precedence over d)
//   2. intra_event_diff: same level after the event, different changes inside it
//   3. oracle=CUT, owner=SUPPLIED: c1_esw_supply (owner input esw && !ksw), else oracle_stale_cut
//   4. oracle=SUPPLIED, owner=CUT: ksw_rise_wp_release, else c1_maint_cut (owner input !esw && !ksw), else
//      oracle_stale_supplied
template <typename Policy, template <typename> class Harness>
struct stageb_classifier {
    stageb_classifier(const char *variant, std::uint64_t max_examples)
        : variant_(variant), max_examples_(max_examples)
    {
    }

    std::uint64_t total_differing{0};
    std::uint64_t cat_a{0};
    std::uint64_t cat_b{0};
    std::uint64_t cat_c{0};
    std::uint64_t cat_d{0};
    std::uint64_t ksw_rise_wp_release{0};
    std::uint64_t c1_esw_supply{0};
    std::uint64_t c1_maint_cut{0};
    std::uint64_t stale_cut{0};
    std::uint64_t stale_cut_resume_wait_enter{0};
    std::uint64_t stale_cut_enter_other{0};
    std::uint64_t stale_cut_ksw_recovery{0};
    std::uint64_t stale_cut_poll_other{0};
    std::uint64_t stale_supplied{0};
    std::uint64_t intra_event_diff{0};
    std::uint64_t redundant_only_diff{0};

    bool visit(const int *s, int length)
    {
        write_trace owner_trace;
        write_trace oracle_trace;
        const fake_writer owner_run = runTraced<owner_t<Policy>, Harness>(s, length, owner_trace);
        const fake_writer oracle_run = runTraced<oracle_for<Policy>, Harness>(s, length, oracle_trace);
        const event_log owner_log = makeEventLog(owner_run, owner_trace, length);
        const event_log oracle_log = makeEventLog(oracle_run, oracle_trace, length);
        const int e = firstDifferingEvent(owner_log, oracle_log, length);
        if (e < 0) {
            if (!sameWrites(owner_run.writes, oracle_run.writes)) {
                ++redundant_only_diff;
            }
            return true;
        }
        ++total_differing;
        const defect d = oracleDefectInEvent(oracle_run, oracle_trace, e);
        if (d == defect::A) {
            ++cat_a;
            return true;
        }
        if (d == defect::B) {
            ++cat_b;
            return true;
        }
        if (d == defect::C) {
            ++cat_c;
            return true;
        }
        if (d == defect::D) {
            ++cat_d;
            return true;
        }
        const bool owner_after = owner_log.after[e];
        const bool oracle_after = oracle_log.after[e];
        if (owner_after == oracle_after) {
            if (++intra_event_diff <= max_examples_) {
                printExample(variant_, policyName<Policy>(), "intra_event_diff", s, length, e, owner_log, oracle_log);
            }
            return true;
        }
        const v_wheel_inputs owner_in = owner_trace.event_inputs[e];
        if (owner_after) {
            // oracle=CUT, owner=SUPPLIED.
            if (owner_in.esw_asserted && !owner_in.ksw_running) {
                if (++c1_esw_supply <= max_examples_) {
                    printExample(variant_, policyName<Policy>(), "c1_esw_supply", s, length, e, owner_log, oracle_log);
                }
                return true;
            }
            // The oracle pin did not follow the target.
            ++stale_cut;
            const int w = firstChangingWrite(owner_run, owner_trace, e);
            const write_trace::entry &t = owner_trace.items[w];
            const char *label = "oracle_stale_cut(poll_other)";
            std::uint64_t seen = 0;
            if (t.is_enter) {
                if (t.is_resume_wait) {
                    seen = ++stale_cut_resume_wait_enter;
                    label = "oracle_stale_cut(resume_wait_enter)";
                } else {
                    seen = ++stale_cut_enter_other;
                    label = "oracle_stale_cut(enter_other)";
                }
            } else if (t.ksw_now && !t.ksw_prev) {
                seen = ++stale_cut_ksw_recovery;
                label = "oracle_stale_cut(ksw_recovery)";
            } else {
                seen = ++stale_cut_poll_other;
            }
            if (seen <= max_examples_) {
                printExample(variant_, policyName<Policy>(), label, s, length, e, owner_log, oracle_log);
            }
            return true;
        }
        // oracle=SUPPLIED, owner=CUT for a reason other than the known defects.
        if (isKswRiseWpRelease(oracle_run, oracle_trace, owner_trace, e)) {
            ++ksw_rise_wp_release;
            return true;
        }
        if (!owner_in.esw_asserted && !owner_in.ksw_running) {
            if (++c1_maint_cut <= max_examples_) {
                printExample(variant_, policyName<Policy>(), "c1_maint_cut", s, length, e, owner_log, oracle_log);
            }
            return true;
        }
        if (++stale_supplied <= max_examples_) {
            printExample(variant_, policyName<Policy>(), "oracle_stale_supplied", s, length, e, owner_log, oracle_log);
        }
        return true;
    }

    void print() const
    {
        printk("stageB-diff[%s/%s]: total_differing=%llu a=%llu b=%llu c=%llu d=%llu "
               "ksw_rise_wp_release=%llu c1_esw_supply=%llu c1_maint_cut=%llu oracle_stale_cut=%llu "
               "(resume_wait_enter=%llu enter_other=%llu ksw_recovery=%llu poll_other=%llu) "
               "oracle_stale_supplied=%llu intra_event_diff=%llu redundant_only_diff=%llu\n",
               variant_, policyName<Policy>(), static_cast<unsigned long long>(total_differing),
               static_cast<unsigned long long>(cat_a), static_cast<unsigned long long>(cat_b),
               static_cast<unsigned long long>(cat_c), static_cast<unsigned long long>(cat_d),
               static_cast<unsigned long long>(ksw_rise_wp_release), static_cast<unsigned long long>(c1_esw_supply),
               static_cast<unsigned long long>(c1_maint_cut), static_cast<unsigned long long>(stale_cut),
               static_cast<unsigned long long>(stale_cut_resume_wait_enter),
               static_cast<unsigned long long>(stale_cut_enter_other),
               static_cast<unsigned long long>(stale_cut_ksw_recovery),
               static_cast<unsigned long long>(stale_cut_poll_other),
               static_cast<unsigned long long>(stale_supplied), static_cast<unsigned long long>(intra_event_diff),
               static_cast<unsigned long long>(redundant_only_diff));
    }

private:
    const char *variant_;
    std::uint64_t max_examples_;
};

template <typename Policy>
void checkStageBAbstract()
{
    // Reference only: every event order is enumerated, including orders the board cannot produce.
    stageb_classifier<Policy, harness> classifier("abstract", 3);
    auto visit = [&](const int *s, int length) { return classifier.visit(s, length); };
    enumerateAll(visit);
    classifier.print();
}

template <typename Policy>
void checkStageBReachable()
{
    stageb_classifier<Policy, reachable_harness> classifier("reachable", 10);
    std::uint64_t sequences = 0;
    auto visit = [&](const int *s, int length) {
        ++sequences;
        return classifier.visit(s, length);
    };
    enumerateReachableAll(visit);
    printk("stageB-diff[reachable/%s]: sequences=%llu (length 1..%d)\n", policyName<Policy>(),
           static_cast<unsigned long long>(sequences), reachable_length);
    classifier.print();
    zassert_equal(classifier.stale_cut, 0, "[%s] oracle_stale_cut on a reachable sequence", policyName<Policy>());
    zassert_equal(classifier.stale_supplied, 0, "[%s] oracle_stale_supplied on a reachable sequence",
                  policyName<Policy>());
    zassert_equal(classifier.intra_event_diff, 0, "[%s] owner and oracle differ inside an event without a defect",
                  policyName<Policy>());
    if (!is_push_policy<Policy>::value) {
        zassert_equal(classifier.c1_esw_supply, 0, "[%s] c1_esw_supply must not occur", policyName<Policy>());
        zassert_equal(classifier.c1_maint_cut, 0, "[%s] c1_maint_cut must not occur", policyName<Policy>());
    }
}

// Expected target per input, in the order of the L4a table: wheel_poweroff, esw, ksw.
struct truth_row {
    bool wheel_poweroff;
    bool esw_asserted;
    bool ksw_running;
    bool supplied;
};

constexpr truth_row push_truth[8] = {
    {false, false, true, true},  {false, false, false, false}, {false, true, true, true},  {false, true, false, true},
    {true, false, true, false},  {true, false, false, false},  {true, true, true, true},   {true, true, false, true}};

constexpr truth_row standard_truth[8] = {
    {false, false, true, true},  {false, false, false, false}, {false, true, true, true},  {false, true, false, false},
    {true, false, true, false},  {true, false, false, false},  {true, true, true, true},   {true, true, false, false}};

// The policy itself, and the owner in DECIDE through enter and through poll.
template <typename Policy>
void checkTruthTable(const truth_row (&rows)[8], int first_id)
{
    for (int i = 0; i < 8; ++i) {
        const truth_row &r = rows[i];
        const v_wheel_inputs in{r.wheel_poweroff, r.ksw_running, r.esw_asserted};
        const int id = first_id + i;
        zassert_equal(Policy::target(in), r.supplied, "T-L4a-%d: [%s] target() wp=%d esw=%d ksw=%d", id,
                      policyName<Policy>(), r.wheel_poweroff, r.esw_asserted, r.ksw_running);

        fake_writer entered;
        owner_t<Policy> enter_dut(entered);
        enter_dut.enter(v_wheel_state::NORMAL, in);
        zassert_equal(entered.pin_level, r.supplied, "T-L4a-%d: [%s] owner enter(NORMAL) wp=%d esw=%d ksw=%d", id,
                      policyName<Policy>(), r.wheel_poweroff, r.esw_asserted, r.ksw_running);

        fake_writer polled;
        owner_t<Policy> poll_dut(polled);
        poll_dut.enter(v_wheel_state::NORMAL, v_wheel_inputs{false, false, false});
        polled.forcePin(!r.supplied);
        poll_dut.poll(in);
        zassert_equal(polled.pin_level, r.supplied, "T-L4a-%d: [%s] owner poll wp=%d esw=%d ksw=%d", id,
                      policyName<Policy>(), r.wheel_poweroff, r.esw_asserted, r.ksw_running);
    }
}

// A ksw rise cycle: the poll input of the owner carries ksw_running=false.
template <typename Policy>
void checkKswRiseCycle(bool esw, bool expected_supplied, int id)
{
    fake_writer writer;
    owner_t<Policy> dut(writer);
    reachable_harness<owner_t<Policy>> h(dut);
    h.apply(enter_event(v_wheel_state::STANDBY));
    h.apply(poll_event(false, false, false));
    h.apply(poll_event(false, true, esw));
    zassert_false(h.inputs().ksw_running, "T-L7-%d: poll input must carry ksw_running=false", id);
    zassert_equal(writer.pin_level, expected_supplied, "T-L7-%d: [%s] esw=%d", id, policyName<Policy>(), esw);
}

// T-L4b-1, T-L4b-2: the ROS cut that the ESW does not override.
template <typename Policy>
void checkEnterKeepsRosCut()
{
    for (int k = 0; k < 2; ++k) {
        const bool ksw = k == 0;
        fake_writer writer;
        owner_t<Policy> dut(writer);
        dut.enter(v_wheel_state::NORMAL, v_wheel_inputs{true, ksw, false});
        zassert_false(writer.pin_level, "[%s] ksw=%d: pin must be CUT", policyName<Policy>(), ksw);
        zassert_false(anySupplied(writer.writes, 0), "[%s] ksw=%d: no SUPPLIED write allowed", policyName<Policy>(),
                      ksw);
        if (!ksw) {
            zassert_equal(writer.writes.size, 1, "[%s] CUT must be written once", policyName<Policy>());
        }
    }
}

// T-L4b-7
template <typename Policy>
void checkMaintenanceCutsEveryCycle()
{
    fake_writer writer;
    owner_t<Policy> dut(writer);
    dut.enter(v_wheel_state::NORMAL, v_wheel_inputs{false, false, false});
    for (int i = 0; i < 3; ++i) {
        dut.poll(v_wheel_inputs{false, false, false});
    }
    zassert_equal(writer.writes.size, 4, "[%s] one write per cycle expected", policyName<Policy>());
    zassert_false(anySupplied(writer.writes, 0), "[%s] every write must be CUT", policyName<Policy>());
}

// T-L4b-8
template <typename Policy>
void checkHoldIgnoresEsw()
{
    for (int k = 0; k < 2; ++k) {
        const bool level = k != 0;
        fake_writer writer;
        owner_t<Policy> dut(writer);
        writer.forcePin(level);
        dut.enter(v_wheel_state::AUTO_CHARGE, v_wheel_inputs{false, false, true});
        dut.poll(v_wheel_inputs{false, false, true});
        dut.poll(v_wheel_inputs{false, false, false});
        zassert_equal(writer.writes.size, 0, "[%s] HOLD must not write", policyName<Policy>());
        zassert_equal(writer.pin_level, level, "[%s] HOLD must keep the pin level", policyName<Policy>());
    }
}

template <typename Policy>
void checkWriterFailure()
{
    for (int in = 0; in < poll_event_count; ++in) {
        const v_wheel_inputs inputs = inputsOf(in);
        for (int k = 0; k < table_state_count; ++k) {
            fake_writer writer;
            writer.ready = false;
            owner_t<Policy> dut(writer);
            const bool expected = modeOf(table_states[k]) == mode::HOLD;
            const bool result = dut.enter(table_states[k], inputs);
            zassert_equal(result, expected, "[%s] enter(%s) wp=%d ksw=%d esw=%d returned wrong value",
                          policyName<Policy>(), table_state_names[k], inputs.wheel_poweroff, inputs.ksw_running,
                          inputs.esw_asserted);
            zassert_equal(writer.writes.size, 0);
        }
        fake_writer off_writer;
        off_writer.ready = false;
        owner_t<Policy> off_dut(off_writer);
        zassert_false(off_dut.enter(v_wheel_state::OFF, inputs), "enter(OFF) must fail when the writer fails");
        fake_writer writer;
        writer.ready = false;
        owner_t<Policy> dut(writer);
        dut.enter(v_wheel_state::NORMAL, inputs);
        dut.poll(inputs);
        zassert_equal(writer.writes.size, 0, "poll must not log a failed write");
    }
}

template <typename Policy>
void checkForceCutAndHoldEnters()
{
    // enter(OFF) is not an event of the abstract enumeration
    for (int in = 0; in < poll_event_count; ++in) {
        for (int level = 0; level < 2; ++level) {
            fake_writer writer;
            owner_t<Policy> dut(writer);
            writer.forcePin(level != 0);
            dut.enter(v_wheel_state::OFF, inputsOf(in));
            zassert_false(writer.pin_level, "[%s] T5-2: pin must be CUT right after enter(OFF)", policyName<Policy>());
        }
    }
    const v_wheel_state hold_states[2] = {v_wheel_state::AUTO_CHARGE, v_wheel_state::OTHER};
    for (const v_wheel_state s : hold_states) {
        for (int in = 0; in < poll_event_count; ++in) {
            for (int level = 0; level < 2; ++level) {
                fake_writer writer;
                owner_t<Policy> dut(writer);
                writer.forcePin(level != 0);
                const bool ok = dut.enter(s, inputsOf(in));
                zassert_true(ok, "HOLD enter must return true");
                zassert_equal(writer.writes.size, 0, "HOLD enter must not write");
                zassert_equal(writer.pin_level, level != 0, "HOLD enter must keep the pin level");
            }
        }
    }
}

}  // namespace

ZTEST_SUITE(wheel_power, NULL, NULL, NULL, NULL, NULL);

ZTEST(wheel_power, test_harness_sequence_count)
{
    std::uint64_t count = 0;
    auto visit = [&](const int *, int) {
        ++count;
        return true;
    };
    enumerateAll(visit);
    printk("harness: enumerated %llu sequences (length 1..%d, %d events)\n", static_cast<unsigned long long>(count),
           abstract_length, event_count);
}

ZTEST(wheel_power, test_oracle_defect_a_suspend_supplies_with_ksw_running)
{
    const int seq[] = {poll_event(true, true), enter_event(v_wheel_state::SUSPEND)};
    // poll is ignored in POST, so only the enter write remains
    const write_list w = oracleWrites(seq, 2);
    zassert_equal(w.size, 1);
    zassert_true(w.items[w.size - 1], "oracle must write SUPPLIED on SUSPEND with ksw running and wp=1");
}

ZTEST(wheel_power, test_oracle_defect_b_normal_supplies_then_poll_cuts)
{
    const int seq[] = {poll_event(true, false), enter_event(v_wheel_state::NORMAL), poll_event(true, false)};
    const write_list w = oracleWrites(seq, 3);
    const bool expected[] = {true, false, false};
    text_buffer text;
    appendWrites(text, w);
    zassert_true(sameWrites(w, expected, 3), "got %s", text.chars);
}

ZTEST(wheel_power, test_oracle_defect_c_wp_release_writes_supplied_then_cut)
{
    const int seq[] = {enter_event(v_wheel_state::NORMAL), poll_event(true, false), poll_event(false, false)};
    const write_list w = oracleWrites(seq, 3);
    // enter(NORMAL): 0; poll wp=1: 0, 0; poll wp=0: 1 then 0
    const bool expected[] = {false, false, false, true, false};
    text_buffer text;
    appendWrites(text, w);
    zassert_true(sameWrites(w, expected, 5), "got %s", text.chars);
}

ZTEST(wheel_power, test_oracle_esw_guard_ignores_ros_cut_in_poll)
{
    // The first poll only latches ksw=running (the harness starts in POST, which does not poll).
    const int seq[] = {poll_event(false, true, false), enter_event(v_wheel_state::NORMAL),
                       poll_event(true, true, true), poll_event(true, true, false)};
    const write_list w = oracleWrites(seq, 4);
    // enter(NORMAL): 1; poll wp=1 esw=1: no change; poll wp=1 esw=0: the cut is written
    const bool expected[] = {true, false};
    text_buffer text;
    appendWrites(text, w);
    zassert_true(sameWrites(w, expected, 2), "got %s", text.chars);
}

ZTEST(wheel_power, test_oracle_push_build_has_no_maintenance_cut)
{
    fake_writer standard_writer;
    std_oracle_t standard_oracle(standard_writer);
    fake_writer push_writer;
    old_logic_oracle<fake_writer, true> push_oracle(push_writer);
    for (int i = 0; i < 2; ++i) {
        standard_oracle.poll(v_wheel_inputs{false, false, false});
        push_oracle.poll(v_wheel_inputs{false, false, false});
    }
    zassert_equal(standard_writer.writes.size, 2, "standard build cuts every cycle while ksw is not running");
    zassert_equal(push_writer.writes.size, 0, "Push build has no maintenance cut");
}

ZTEST(wheel_power, test_oracle_dcdc_writes_on_standby_and_off)
{
    const int standby[] = {enter_event(v_wheel_state::STANDBY)};
    const bool standby_expected[] = {true, false};
    const write_list s = oracleWrites(standby, 1);
    zassert_true(sameWrites(s, standby_expected, 2), "enter(STANDBY) must write dcdc SUPPLIED then bat_out_state");

    fake_writer writer;
    std_oracle_t oracle(writer);
    oracle.enter(v_wheel_state::OFF, {true, true});
    zassert_equal(writer.writes.size, 1);
    zassert_false(writer.writes.items[0], "enter(OFF) must write dcdc CUT");
    zassert_true(writer.kinds[0] == oracle_write_kind::ENTER_DCDC);

    fake_writer failing;
    failing.ready = false;
    std_oracle_t failing_oracle(failing);
    zassert_false(failing_oracle.enter(v_wheel_state::STANDBY, {false, true}));
}

ZTEST(wheel_power, test_stageB_diff_abstract_standard) { checkStageBAbstract<standard_policy>(); }

ZTEST(wheel_power, test_stageB_diff_abstract_push) { checkStageBAbstract<push_policy>(); }

ZTEST(wheel_power, test_stageB_diff_reachable_standard) { checkStageBReachable<standard_policy>(); }

ZTEST(wheel_power, test_stageB_diff_reachable_push) { checkStageBReachable<push_policy>(); }

ZTEST(wheel_power, test_T_L4a_push_truth_table) { checkTruthTable<push_policy>(push_truth, 1); }

ZTEST(wheel_power, test_T_L4a_standard_truth_table) { checkTruthTable<standard_policy>(standard_truth, 9); }

ZTEST(wheel_power, test_T_L7_1_push_rise_cycle_no_esw_cuts) { checkKswRiseCycle<push_policy>(false, false, 1); }

ZTEST(wheel_power, test_T_L7_2_push_rise_cycle_esw_supplies) { checkKswRiseCycle<push_policy>(true, true, 2); }

ZTEST(wheel_power, test_T_L7_3_standard_rise_cycle_no_esw_cuts) { checkKswRiseCycle<standard_policy>(false, false, 3); }

ZTEST(wheel_power, test_T_L7_4_standard_rise_cycle_esw_still_cuts)
{
    checkKswRiseCycle<standard_policy>(true, false, 4);
}

ZTEST(wheel_power, test_T_L4b_1_2_enter_keeps_ros_cut)
{
    checkEnterKeepsRosCut<standard_policy>();
    checkEnterKeepsRosCut<push_policy>();
}

ZTEST(wheel_power, test_T_L4b_3_push_enter_with_esw_supplies_in_maintenance)
{
    fake_writer writer;
    owner_t<push_policy> dut(writer);
    dut.enter(v_wheel_state::NORMAL, v_wheel_inputs{false, false, true});
    zassert_true(writer.pin_level, "pin must be SUPPLIED");
}

ZTEST(wheel_power, test_T_L4b_4_push_ros_cut_after_esw_stays_supplied)
{
    for (int k = 0; k < 2; ++k) {
        const bool ksw = k != 0;
        fake_writer writer;
        owner_t<push_policy> dut(writer);
        dut.enter(v_wheel_state::NORMAL, v_wheel_inputs{false, ksw, true});
        zassert_true(writer.pin_level, "ksw=%d: pin must be SUPPLIED after enter", ksw);
        writer.clearLog();
        dut.poll(v_wheel_inputs{true, ksw, true});
        zassert_true(writer.pin_level, "ksw=%d: pin must stay SUPPLIED", ksw);
        zassert_false(writer.writes.size > 0 && !writer.writes.items[0], "ksw=%d: no CUT write allowed", ksw);
    }
}

ZTEST(wheel_power, test_T_L4b_5_push_esw_release_with_ros_cut_cuts_at_once)
{
    for (int k = 0; k < 2; ++k) {
        const bool ksw = k != 0;
        fake_writer writer;
        owner_t<push_policy> dut(writer);
        dut.enter(v_wheel_state::NORMAL, v_wheel_inputs{true, ksw, true});
        zassert_true(writer.pin_level, "ksw=%d: pin must be SUPPLIED while the ESW is asserted", ksw);
        writer.clearLog();
        dut.poll(v_wheel_inputs{true, ksw, false});
        zassert_equal(writer.writes.size, 1, "ksw=%d: exactly one write expected", ksw);
        zassert_false(writer.writes.items[0], "ksw=%d: the write must be CUT", ksw);
        zassert_false(writer.pin_level);
    }
}

ZTEST(wheel_power, test_T_L4b_6_push_esw_release_in_maintenance_cuts_at_once)
{
    fake_writer writer;
    owner_t<push_policy> dut(writer);
    dut.enter(v_wheel_state::NORMAL, v_wheel_inputs{false, false, true});
    zassert_true(writer.pin_level, "pin must be SUPPLIED while the ESW is asserted");
    writer.clearLog();
    dut.poll(v_wheel_inputs{false, false, false});
    zassert_equal(writer.writes.size, 1, "exactly one write expected");
    zassert_false(writer.writes.items[0], "the write must be CUT");
    zassert_false(writer.pin_level);
}

ZTEST(wheel_power, test_T_L4b_7_maintenance_cuts_every_cycle)
{
    checkMaintenanceCutsEveryCycle<standard_policy>();
    checkMaintenanceCutsEveryCycle<push_policy>();
}

ZTEST(wheel_power, test_T_L4b_8_hold_ignores_esw)
{
    checkHoldIgnoresEsw<standard_policy>();
    checkHoldIgnoresEsw<push_policy>();
}

ZTEST(wheel_power, test_T4_1_running_wp_enter_suspend_cuts)
{
    fake_writer writer;
    std_owner_t dut(writer);
    dut.enter(v_wheel_state::SUSPEND, {true, true});
    zassert_false(writer.pin_level, "pin must be CUT");
    zassert_false(anySupplied(writer.writes, 0), "no SUPPLIED write allowed");
}

ZTEST(wheel_power, test_T4_2_maintenance_wp_enter_normal_never_supplies)
{
    fake_writer writer;
    std_owner_t dut(writer);
    dut.enter(v_wheel_state::NORMAL, {true, false});
    zassert_false(anySupplied(writer.writes, 0), "no SUPPLIED write allowed");
    zassert_false(writer.pin_level, "pin must be CUT");
}

ZTEST(wheel_power, test_T4_3_maintenance_wp_release_poll_writes_once_cut)
{
    fake_writer writer;
    std_owner_t dut(writer);
    dut.enter(v_wheel_state::NORMAL, {true, false});
    dut.poll({true, false});
    writer.clearLog();
    dut.poll({false, false});
    zassert_equal(writer.writes.size, 1, "exactly one write expected");
    zassert_false(writer.writes.items[0], "the write must be CUT");
    zassert_false(writer.pin_level);
}

ZTEST(wheel_power, test_T4_4_running_no_wp_enter_normal_supplies)
{
    fake_writer writer;
    std_owner_t dut(writer);
    dut.enter(v_wheel_state::NORMAL, {false, true});
    zassert_true(writer.pin_level, "pin must be SUPPLIED");
}

ZTEST(wheel_power, test_T4_5_maintenance_no_wp_enter_normal_cuts)
{
    fake_writer writer;
    std_owner_t dut(writer);
    dut.enter(v_wheel_state::NORMAL, {false, false});
    zassert_false(writer.pin_level, "pin must be CUT");
}

ZTEST(wheel_power, test_T4_6_enter_off_cuts)
{
    for (int in = 0; in < 4; ++in) {
        for (int level = 0; level < 2; ++level) {
            fake_writer writer;
            std_owner_t dut(writer);
            writer.forcePin(level != 0);
            const bool ok = dut.enter(v_wheel_state::OFF, {((in >> 1) & 1) != 0, (in & 1) != 0});
            zassert_true(ok);
            zassert_equal(writer.writes.size, 1, "enter(OFF) must write once");
            zassert_false(writer.writes.items[0], "enter(OFF) must write CUT");
            zassert_false(writer.pin_level, "pin must be CUT");
        }
    }
}

ZTEST(wheel_power, test_T5_1_decide_state_pin_equals_target_standard)
{
    zassert_true(invariantHolds<standard_policy>(1), "T5-1 violated");
}

ZTEST(wheel_power, test_T5_1_decide_state_pin_equals_target_push)
{
    zassert_true(invariantHolds<push_policy>(1), "T5-1 violated");
}

ZTEST(wheel_power, test_T5_2_force_cut_state_pin_is_cut_standard)
{
    zassert_true(invariantHolds<standard_policy>(2), "T5-2 violated");
}

ZTEST(wheel_power, test_T5_2_force_cut_state_pin_is_cut_push)
{
    zassert_true(invariantHolds<push_policy>(2), "T5-2 violated");
}

ZTEST(wheel_power, test_T5_3_cut_target_never_writes_supplied_standard)
{
    zassert_true(invariantHolds<standard_policy>(3), "T5-3 violated");
}

ZTEST(wheel_power, test_T5_3_cut_target_never_writes_supplied_push)
{
    zassert_true(invariantHolds<push_policy>(3), "T5-3 violated");
}

ZTEST(wheel_power, test_T5_4_decide_enter_keeps_ros_cut_standard)
{
    zassert_true(invariantHolds<standard_policy>(4), "T5-4 violated");
}

ZTEST(wheel_power, test_T5_4_decide_enter_keeps_ros_cut_push)
{
    zassert_true(invariantHolds<push_policy>(4), "T5-4 violated");
}

ZTEST(wheel_power, test_T5_5_hold_enter_does_not_write_standard)
{
    zassert_true(invariantHolds<standard_policy>(5), "T5-5 violated");
}

ZTEST(wheel_power, test_T5_5_hold_enter_does_not_write_push)
{
    zassert_true(invariantHolds<push_policy>(5), "T5-5 violated");
}

ZTEST(wheel_power, test_T5_force_cut_and_hold_enters_outside_the_enumeration)
{
    checkForceCutAndHoldEnters<standard_policy>();
    checkForceCutAndHoldEnters<push_policy>();
}

ZTEST(wheel_power, test_T3_4_writer_failure)
{
    checkWriterFailure<standard_policy>();
    checkWriterFailure<push_policy>();
}

ZTEST(wheel_power, test_T6_transition_table)
{
    // Columns: the 4 polls without ESW, the 8 abstract enters, then enter(OFF). Rows are the standard policy.
    constexpr int table_poll_count = 4;
    constexpr int table_event_count = table_poll_count + 8 + 1;
    printk("| prev state | wp | ksw | prev pin |");
    for (int e = 0; e < table_event_count; ++e) {
        text_buffer name;
        if (e < table_poll_count) {
            name.append("poll(wp=%d,ksw=%d,esw=0) ", (e >> 1) & 1, e & 1);
        } else if (e < table_poll_count + 8) {
            name.append("enter(%s) ", enter_names[e - table_poll_count]);
        } else {
            name.append("enter(OFF) ");
        }
        printk(" %s|", name.chars);
    }
    printk("\n|---|---|---|---|");
    for (int e = 0; e < table_event_count; ++e) {
        printk("---|");
    }
    printk("\n");
    int rows = 0;
    for (int k = 0; k < table_state_count; ++k) {
        for (int in = 0; in < 4; ++in) {
            const v_wheel_inputs inputs{((in >> 1) & 1) != 0, (in & 1) != 0};
            for (int level = 0; level < 2; ++level) {
                text_buffer row;
                row.append("| %s | %d | %d | %d |", table_state_names[k], inputs.wheel_poweroff, inputs.ksw_running,
                           level);
                for (int e = 0; e < table_event_count; ++e) {
                    fake_writer writer;
                    std_owner_t dut(writer);
                    dut.enter(table_states[k], inputs);
                    writer.forcePin(level != 0);
                    writer.clearLog();
                    if (e < table_poll_count) {
                        if (modeOf(table_states[k]) != mode::DECIDE) {
                            row.append(" n/a |");
                            continue;
                        }
                        dut.poll({((e >> 1) & 1) != 0, (e & 1) != 0});
                    } else {
                        dut.enter(e < table_poll_count + 8 ? enter_states[e - table_poll_count] : v_wheel_state::OFF,
                                  inputs);
                    }
                    if (writer.writes.size == 0) {
                        row.append(" - |");
                        continue;
                    }
                    row.append(" ");
                    for (int w = 0; w < writer.writes.size; ++w) {
                        row.append(w == 0 ? "%c" : ">%c", writer.writes.items[w] ? '1' : '0');
                    }
                    row.append(" |");
                }
                printk("%s\n", row.chars);
                ++rows;
            }
        }
    }
    zassert_equal(rows, 80, "unexpected table row count");
}
