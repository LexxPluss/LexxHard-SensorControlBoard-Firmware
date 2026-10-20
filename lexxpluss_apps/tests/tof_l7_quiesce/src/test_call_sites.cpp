/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The properties that are about WHERE something is called.
 *
 * Three of the things this round rests on are claims about call sites, not about what a function
 * computes, so the suites beside this one cannot reach them:
 *
 *   the quiesce record is written by the quiesce hook and by nothing else -- a second writer is how
 *   a later probe or stop comes to overwrite the observation the record exists to keep;
 *
 *   nothing in the application clears it, because a clear outside a test is indistinguishable from
 *   a quiesce that was never recorded;
 *
 *   the quiesce is timed from the 64-bit monotonic source -- the first version used
 *   `k_cycle_get_32()` and recorded a 30-second call as 10.116 s.
 *
 * So this reads the sources and checks them. Three earlier attempts are why it looks like this. The
 * first counted one write anywhere in the adapter, which a write moved into the probe hook would
 * have passed. The second was Python invoked from this test's CMakeLists, which the repository's
 * language boundary forbids three ways over -- product code and executable tests are C and C++, and
 * scripts/check_language_boundary.sh says so. The third banned one 32-bit clock name, and review
 * substituted another that passed.
 *
 * Plain C strings and a fixed buffer, like the rest of the host suites here: this image has no
 * libstdc++ headers. A file that does not fit the buffer FAILS rather than being truncated -- a
 * gate that reads half a file and finds nothing wrong is worse than no gate.
 */

#include <zephyr/ztest.h>

#include <stdio.h>
#include <string.h>

#if !defined(DEVICE_SOURCE) || !defined(SHELL_SOURCE) || !defined(RUNTIME_SOURCE) || \
    !defined(SOURCE_LIST_FILE)
#error "CMake must pass DEVICE_SOURCE, SHELL_SOURCE, RUNTIME_SOURCE and SOURCE_LIST_FILE"
#endif

namespace {

constexpr size_t kBufferSize{256U * 1024U};
char buffer_[kBufferSize];
char stripped_[kBufferSize];
char list_[16U * 1024U];

/* THE SOURCE WITH COMMENTS REMOVED, into `stripped_`.
 *
 * An earlier run of this gate failed on its OWN explanatory comment, which names the banned clock
 * in order to say why it is banned. A check that reads a file must not be satisfiable or defeated
 * by prose in it. String and character literals are tracked so a "//" inside a message is not
 * mistaken for a comment. */
void strip_comments(const char *text)
{
    size_t out{0};
    bool in_string{false}, in_char{false}, in_line{false}, in_block{false};
    for (size_t i = 0; text[i] != '\0';) {
        const char c{text[i]};
        const char next{text[i + 1]};
        if (in_line) {
            if (c == '\n') {
                in_line = false;
                stripped_[out++] = c;
            }
            ++i;
        } else if (in_block) {
            if (c == '*' && next == '/') {
                in_block = false;
                i += 2;
            } else {
                if (c == '\n')
                    stripped_[out++] = c;
                ++i;
            }
        } else if (in_string || in_char) {
            stripped_[out++] = c;
            if (c == '\\' && next != '\0') {
                stripped_[out++] = next;
                i += 2;
                continue;
            }
            if ((in_string && c == '"') || (in_char && c == '\''))
                in_string = in_char = false;
            ++i;
        } else if (c == '/' && next == '/') {
            in_line = true;
            i += 2;
        } else if (c == '/' && next == '*') {
            in_block = true;
            i += 2;
        } else {
            if (c == '"')
                in_string = true;
            else if (c == '\'')
                in_char = true;
            stripped_[out++] = c;
            ++i;
        }
        zassert_true(out < kBufferSize, "stripped output overran its buffer");
    }
    stripped_[out] = '\0';
}

/* Reads into `buffer_` and strips into `stripped_`. A file that does not fit fails the case. */
void read_and_strip(const char *path)
{
    FILE *const f{fopen(path, "rb")};
    zassert_not_null(f, "cannot read %s", path);
    const size_t got{fread(buffer_, 1, kBufferSize - 1U, f)};
    const bool whole{feof(f) != 0};
    fclose(f);
    zassert_true(got > 0U, "%s is empty", path);
    zassert_true(whole, "%s is larger than this gate's %zu-byte buffer, so it was NOT fully read; "
                        "raise the buffer rather than trusting a partial check",
                 path, kBufferSize);
    buffer_[got] = '\0';
    strip_comments(buffer_);
}

size_t count_of(const char *haystack, const char *needle)
{
    size_t n{0};
    const size_t len{strlen(needle)};
    for (const char *at = strstr(haystack, needle); at != nullptr; at = strstr(at + 1, needle))
        ++n;
    (void)len;
    return n;
}

/* The body of a function: from its signature's opening brace to the first closing brace in column
 * one, which is how this repository's sources are written. Returned as offsets into `stripped_`. */
/* The body that follows a point already located in `stripped_`. Split out of body_of() so that
 * every occurrence of a signature can be walked, not only the first: a gate that can see one
 * definition cannot say anything about the others. */
bool body_at(const char *start, size_t *from, size_t *to)
{
    if (start == nullptr)
        return false;
    const char *const opened{strchr(start, '{')};
    if (opened == nullptr)
        return false;
    const char *const end{strstr(opened, "\n}")};
    if (end == nullptr)
        return false;
    *from = static_cast<size_t>(opened - stripped_);
    *to = static_cast<size_t>(end + 2 - stripped_);
    return true;
}

bool body_of(const char *signature, size_t *from, size_t *to)
{
    const char *const start{strstr(stripped_, signature)};
    if (start == nullptr)
        return false;
    const char *const opened{strchr(start, '{')};
    if (opened == nullptr)
        return false;
    const char *const end{strstr(opened, "\n}")};
    if (end == nullptr)
        return false;
    *from = static_cast<size_t>(opened - stripped_);
    *to = static_cast<size_t>(end + 2 - stripped_);
    return true;
}

constexpr const char *kWriter{"tof_l7_quiesce::remember("};
constexpr const char *kClearer{"tof_l7_quiesce::forget_all("};
/* The only clock read the quiesce may make. 64-bit, monotonic, and asserted POSITIVELY: a gate that
 * merely banned `k_cycle_get_32` passed a substitution of `k_uptime_get_32()`, which review
 * demonstrated. Naming what is allowed catches every substitution rather than one. */
constexpr const char *kApprovedClock{"k_ticks_to_us_floor64(k_uptime_ticks())"};

constexpr const char *kStepWriter{"tof_l7_steps::remember("};
constexpr const char *kStepClearer{"tof_l7_steps::forget_all("};
/* HOW THE APPLICATION CLEARS, and the whole expression, not just the call. The argument is the
 * property under test: `lifecycle_.can_begin_round()` is the same guard bring_up() asks, so the
 * records are cleared exactly when a round begins. A literal, a negation, or any other predicate
 * would clear for rounds that never started -- which is the defect this replaced. */
constexpr const char *kStepBegin{"tof_l7_steps::begin_round("};
constexpr const char *kStepBeginGuarded{"tof_l7_steps::begin_round(lifecycle_.can_begin_round())"};
/* The two port-context readers a step hook must NOT use. open() and configure() refuse early,
 * before anything clears the port, so its sticky errno and completed count still belong to the
 * previous operation -- the quiesce and its 511 transfers. Reading them in a step hook would
 * attribute those to a call that never touched the bus. */
constexpr const char *kPortErrno{"vl53l7cx_port_error("};
constexpr const char *kPortCompleted{"vl53l7cx_port_completed_transfers("};

struct hook_expectation {
    const char *signature;
    const char *step;
};

/* One writer per hook, each naming its own step. */
const hook_expectation kStepHooks[]{
    { "int hook_close(", "which::close" },
    { "int hook_open(", "which::open" },
    { "int hook_configure(", "which::configure" },
    { "int hook_start(", "which::start" },
    { "int hook_read(", "which::read" },
};

char body_[kBufferSize];

void copy_quiesce_body()
{
    size_t from{0}, to{0};
    zassert_true(body_of("int hook_quiesce(", &from, &to),
                 "hook_quiesce() could not be located in %s, so where the record is written and "
                 "what clock it reads cannot be established",
                 DEVICE_SOURCE);
    const size_t len{to - from};
    zassert_true(len < kBufferSize, "the hook body does not fit");
    memcpy(body_, stripped_ + from, len);
    body_[len] = '\0';
}

}  // namespace

ZTEST_SUITE(tof_l7_quiesce_call_sites, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_l7_quiesce_call_sites, test_the_record_is_written_once_inside_the_quiesce_hook)
{
    read_and_strip(DEVICE_SOURCE);
    copy_quiesce_body();

    zassert_equal(count_of(body_, kWriter), 1U,
                  "the snapshot is recorded on the path that takes it, exactly once: zero means "
                  "nothing records it, more than one means it is written twice for one call");
    zassert_equal(count_of(stripped_, kWriter), 1U,
                  "and nowhere else in the adapter -- a probe or a stop that writes the record is "
                  "exactly how the observation it exists to keep gets overwritten");
}

ZTEST(tof_l7_quiesce_call_sites, test_no_application_source_clears_the_record_or_writes_it_elsewhere)
{
    FILE *const f{fopen(SOURCE_LIST_FILE, "rb")};
    zassert_not_null(f, "cannot read the generated source list %s", SOURCE_LIST_FILE);
    const size_t got{fread(list_, 1, sizeof list_ - 1U, f)};
    const bool whole{feof(f) != 0};
    fclose(f);
    zassert_true(whole, "the source list did not fit this gate's buffer");
    list_[got] = '\0';

    size_t seen{0};
    char *saved{nullptr};
    for (char *line = strtok_r(list_, "\n", &saved); line != nullptr;
         line = strtok_r(nullptr, "\n", &saved)) {
        if (*line == '\0')
            continue;
        ++seen;
        read_and_strip(line);
        const bool is_adapter{strstr(line, "tof_package_device.cpp") != nullptr};
        if (!is_adapter)
            zassert_equal(count_of(stripped_, kWriter), 0U, "%s writes the quiesce record", line);
        zassert_equal(count_of(stripped_, kClearer), 0U,
                      "%s clears the quiesce record. Only a test may: a cleared record is "
                      "indistinguishable from a quiesce that never happened",
                      line);
    }
    zassert_true(seen > 1U, "the generated source list had %zu entries, so this case checked "
                            "nothing -- the list is what makes it a statement about the tree",
                 seen);
}

ZTEST(tof_l7_quiesce_call_sites, test_the_quiesce_is_timed_from_the_approved_sixty_four_bit_source)
{
    read_and_strip(DEVICE_SOURCE);
    copy_quiesce_body();

    zassert_equal(count_of(body_, kApprovedClock), 2U,
                  "the call is timed from %s, read once before and once after -- stated as what IS "
                  "allowed, because banning one 32-bit name let another through",
                  kApprovedClock);

    /* AND NOTHING ELSE READS A CLOCK THERE. Every approved read is removed first, so whatever
     * remains is by definition a clock this body may not use, whatever it is called. */
    const size_t len{strlen(kApprovedClock)};
    for (char *at = strstr(body_, kApprovedClock); at != nullptr; at = strstr(body_, kApprovedClock))
        memmove(at, at + len, strlen(at + len) + 1U);

    static const char *const other_clocks[]{"k_cycle_get", "k_uptime_", "k_cyc_to_", "k_ticks_to_"};
    for (const char *const name : other_clocks)
        zassert_equal(count_of(body_, name), 0U,
                      "hook_quiesce() reads %s besides the approved source. A 32-bit counter wraps "
                      "-- 19.884 s for the cycle counter at 216 MHz -- and a failing quiesce runs "
                      "far longer than that",
                      name);
}

ZTEST(tof_l7_quiesce_call_sites, test_each_bring_up_step_records_itself_once_and_names_its_own_step)
{
    read_and_strip(DEVICE_SOURCE);

    size_t writers_in_hooks{0};
    for (const hook_expectation &expected : kStepHooks) {
        size_t from{0}, to{0};
        zassert_true(body_of(expected.signature, &from, &to), "%s not found", expected.signature);
        const size_t len{to - from};
        zassert_true(len < kBufferSize, "hook body does not fit");
        memcpy(body_, stripped_ + from, len);
        body_[len] = '\0';

        zassert_equal(count_of(body_, kStepWriter), 1U,
                      "%s must record its own outcome exactly once -- zero loses the step the next "
                      "window needs, twice overwrites it within one call",
                      expected.signature);
        zassert_not_null(strstr(body_, expected.step),
                         "%s records a step that is not its own, which puts one hook's observation "
                         "in another's slot", expected.signature);
        writers_in_hooks += 1U;
    }

    zassert_equal(count_of(stripped_, kStepWriter), writers_in_hooks,
                  "the adapter writes a step record from somewhere other than the five hooks");
}

ZTEST(tof_l7_quiesce_call_sites, test_a_step_hook_never_reads_the_port_context)
{
    /* THE BOUNDARY THIS CASE EXISTS FOR, structural because it is about what a hook may touch. The
     * step records come from `operation_status`, the adjudication the sensor layer already made.
     * Reading the port here would look like an improvement and would import the previous
     * operation's counters on every early refusal. */
    read_and_strip(DEVICE_SOURCE);

    for (const hook_expectation &expected : kStepHooks) {
        size_t from{0}, to{0};
        zassert_true(body_of(expected.signature, &from, &to), "%s not found", expected.signature);
        const size_t len{to - from};
        memcpy(body_, stripped_ + from, len);
        body_[len] = '\0';

        zassert_equal(count_of(body_, kPortErrno), 0U,
                      "%s reads %s; on an early refusal that errno is the previous operation's",
                      expected.signature, kPortErrno);
        zassert_equal(count_of(body_, kPortCompleted), 0U,
                      "%s reads %s; on an early refusal that count is the quiesce's",
                      expected.signature, kPortCompleted);
    }
}

ZTEST(tof_l7_quiesce_call_sites, test_the_step_records_are_cleared_only_when_a_round_is_accepted)
{
    /* `attempted` is only a statement about THIS run if something clears them at its start -- and
     * only if the run is one that started. Review reproduced the other case on 2026-10-13: an `up`
     * refused for being in `blocked` cleared first and asked afterwards, so a call that performed
     * no operation erased the failing open that preceded it. */
    read_and_strip(DEVICE_SOURCE);

    zassert_equal(count_of(stripped_, kStepBegin), 1U,
                  "exactly one round boundary: none makes `not called` meaningless after the first "
                  "run, more than one erases a run that is still being read");
    zassert_equal(count_of(stripped_, kStepClearer), 0U,
                  "the adapter clears the step records directly. An unconditional clear is the "
                  "defect itself -- whether a round begins is the lifecycle's answer, and "
                  "begin_round() is where it is applied");

    /* THE BOUNDARY MOVED TWICE AND THE PROPERTY DID NOT: first out of cmd_up() when the takeover
     * entry arrived, and then out of the shell altogether when an unattended runtime became a
     * second caller. It is now the device layer's round entry, which is the only place either
     * caller can reach a lifecycle round through. */
    size_t from{0}, to{0};
    zassert_true(body_of("int begin_round(", &from, &to), "begin_round() not found");
    const size_t len{to - from};
    memcpy(body_, stripped_ + from, len);
    body_[len] = '\0';
    zassert_equal(count_of(body_, kStepBeginGuarded), 1U,
                  "begin_round() must clear through `%s`. The argument is the property: anything "
                  "else clears for rounds the lifecycle refused",
                  kStepBeginGuarded);

    /* AND IT IS ASKED BEFORE THE ROUND RUNS, which the count alone does not say. Asked afterwards
     * the answer is about the state the round LEFT -- `ranging` or `blocked`, never idle -- so the
     * records would never be cleared and every run would show the first one's steps. */
    const char *const boundary{strstr(body_, kStepBeginGuarded)};
    const char *const runs_a_round{strstr(body_, "lifecycle_.")};
    zassert_not_null(runs_a_round, "begin_round() calls no lifecycle entry");
    zassert_true(boundary < runs_a_round,
                 "the round boundary is applied after the round has already run, so it reads the "
                 "state the round left rather than the one it started from");

    /* AND THE QUIESCE RECORD IS NOT CLEARED WITH THEM. The quiesce happens inside the round, so a
     * clear that took it too would discard the observation the last window needed. */
    zassert_equal(count_of(body_, "tof_l7_quiesce::forget_all("), 0U,
                  "begin_round() clears the quiesce record, which is taken during the very call it "
                  "is clearing for");
}

ZTEST(tof_l7_quiesce_call_sites, test_every_command_that_starts_a_round_goes_through_the_boundary)
{
    /* The clear lives in one place, so an entry that reached a lifecycle round without passing
     * through it would start a round with the previous one's step records still in place. There are
     * two commands now and there may be more; each must delegate rather than call a round itself. */
    read_and_strip(SHELL_SOURCE);

    static const char *const commands[]{"int cmd_up(", "int cmd_takeover("};
    for (const char *const signature : commands) {
        size_t from{0}, to{0};
        zassert_true(body_of(signature, &from, &to), "%s not found", signature);
        const size_t len{to - from};
        memcpy(body_, stripped_ + from, len);
        body_[len] = '\0';

        zassert_equal(count_of(body_, "run_round("), 1U,
                      "%s must start its round through run_round()", signature);
        zassert_equal(count_of(body_, kStepBegin), 0U,
                      "%s applies a round boundary of its own, so there are now two places that "
                      "decide when the step records are cleared",
                      signature);
        zassert_equal(count_of(body_, "lifecycle().bring_up("), 0U,
                      "%s calls a lifecycle round directly, bypassing the boundary", signature);
        zassert_equal(count_of(body_, "lifecycle().take_over_unknown_state("), 0U,
                      "%s calls a lifecycle round directly, bypassing the boundary", signature);
    }
}

ZTEST(tof_l7_quiesce_call_sites, test_nothing_in_the_adapter_reassigns_the_sequence_source)
{
    /* THE DEFECT THIS EXISTS FOR. `stream` used to assign a fresh tof_seq_boot_local::source at the
     * start of every run, so a second run in the same boot reissued 1, 2, 3 and a consumer rejected
     * all of it as not newer than the first run's. The source's documented limitation is that
     * numbers repeat after a RESET, which a reader could reasonably take to mean a reboot; a
     * per-command reset made them repeat between two commands.
     *
     * The rule is therefore about the whole adapter, not one command: the source's lifetime is the
     * device's, and nothing here may shorten it. A publisher re-initialised per run is correct and
     * untouched by this -- counts describe a run, numbers describe a device.
     *
     * Matched by scanning for the identifier followed by an assignment rather than by a fixed
     * substring, because "boot_sequence_ =" and "boot_sequence_=" are the same defect and a
     * substring check would catch only whichever one somebody happened to write. */
    read_and_strip(DEVICE_SOURCE);

    size_t mentions{0};
    size_t assignments{0};
    static const char kName[]{"boot_sequence_"};
    for (const char *at{strstr(stripped_, kName)}; at != nullptr;
         at = strstr(at + sizeof kName - 1U, kName)) {
        ++mentions;
        const char *c{at + sizeof kName - 1U};
        while (*c == ' ' || *c == '\t' || *c == '\n' || *c == '\r')
            ++c;
        /* `==` is a comparison, not an assignment, and must not be mistaken for one. */
        if (*c == '=' && *(c + 1) != '=')
            ++assignments;
    }

    zassert_equal(assignments, 0U,
                  "something in %s assigns the sequence source. One source spans the boot; "
                  "re-creating it reissues numbers a consumer has already seen", DEVICE_SOURCE);
    /* A probe that matched nothing would pass. The declaration, the issue adapter and the run's
     * report are the legitimate mentions, so there are at least three. */
    zassert_true(mentions >= 3U, "only %zu mentions of %s were found, so this gate read nothing",
                 mentions, kName);
}

ZTEST(tof_l7_quiesce_call_sites, test_no_shell_command_reaches_a_round_without_the_boundary)
{
    /* THE DERIVED HALF of the check above. That test names the two commands that DO start a round
     * and asserts they delegate, which is a positive property and cannot be derived -- a command
     * that starts no round must not call run_round either. This one needs no list: it walks every
     * `int cmd_` definition in the adapter and asserts that none of them reaches a lifecycle round
     * entry or opens a round boundary of its own.
     *
     * WHY IT WAS ADDED. A third command (`stream`) landed in this file without the named list
     * changing, correctly -- it starts no round. But that is exactly how a fourth could land that
     * DID start one and was never checked, because nothing fails when the list falls behind the
     * file. The hazard the list guards against is derivable, so it is derived, and the list is left
     * to carry only the claim that is not. */
    read_and_strip(SHELL_SOURCE);

    size_t commands{0};
    const char *at{stripped_};
    for (;;) {
        at = strstr(at, "int cmd_");
        if (at == nullptr)
            break;

        /* The command's name, for a failure message that points at a function rather than an
         * offset. */
        char name[64]{};
        size_t n{0};
        for (const char *c{at}; *c != '\0' && *c != '(' && n + 1U < sizeof name; ++c, ++n)
            name[n] = *c;
        name[n] = '\0';

        size_t from{0}, to{0};
        zassert_true(body_at(at, &from, &to), "%s has no body this gate can read", name);
        const size_t len{to - from};
        zassert_true(len < kBufferSize, "%s does not fit this gate's buffer", name);
        memcpy(body_, stripped_ + from, len);
        body_[len] = '\0';

        zassert_equal(count_of(body_, "lifecycle().bring_up("), 0U,
                      "%s calls a lifecycle round directly, bypassing the boundary that clears the "
                      "step records", name);
        zassert_equal(count_of(body_, "lifecycle().take_over_unknown_state("), 0U,
                      "%s calls a lifecycle round directly, bypassing the boundary that clears the "
                      "step records", name);
        zassert_equal(count_of(body_, kStepBegin), 0U,
                      "%s opens a round boundary of its own, so there are now two places that "
                      "decide when the step records are cleared", name);

        ++commands;
        at = stripped_ + to;
    }

    /* A gate that found nothing proves nothing. There are three commands with `int cmd_` bodies at
     * the time of writing and the count only grows, so a floor is the right assertion -- an exact
     * number would be one more list to maintain. */
    zassert_true(commands >= 3U, "only %zu shell command bodies were found in %s", commands,
                 SHELL_SOURCE);
}

ZTEST(tof_l7_quiesce_call_sites, test_only_the_device_layer_reaches_a_lifecycle_round)
{
    /* THE HAZARD A SECOND CALLER CREATED. The round boundary used to be in the shell, where one
     * command either went through it or did not. It is now the device layer's round entry, and
     * there are two callers above it -- the shell and an unattended runtime -- so the question is
     * no longer "does this command delegate" but "can anything in the tree reach a round without
     * the boundary at all". That is derivable, so it is derived: every application source except
     * the device layer must name neither lifecycle round entry.
     *
     * `bring_up` IS NAMED THROUGH ITS ACCESSOR and take_over_unknown_state() is not, which is not
     * an inconsistency: the cliff chain has an unrelated bring_up() of its own in
     * tof_acquisition.cpp, so a bare match there reports a module that has nothing to do with this
     * lifecycle. Every caller outside the device layer must reach this object through
     * lifecycle(), so that spelling is the complete set for it. take_over_unknown_state() exists
     * nowhere else in the tree and needs no qualifier. */
    FILE *const f{fopen(SOURCE_LIST_FILE, "rb")};
    zassert_not_null(f, "cannot read the generated source list");
    const size_t got{fread(list_, 1, sizeof list_ - 1U, f)};
    const bool whole{feof(f) != 0};
    fclose(f);
    zassert_true(whole, "the source list did not fit this gate's buffer");
    list_[got] = '\0';

    size_t checked{0};
    char *saved{nullptr};
    for (char *line = strtok_r(list_, "\n", &saved); line != nullptr;
         line = strtok_r(nullptr, "\n", &saved)) {
        if (*line == '\0')
            continue;
        if (strstr(line, "tof_package_device.cpp") != nullptr)
            continue;
        /* The lifecycle's own translation unit defines them. */
        if (strstr(line, "tof_package_l7.cpp") != nullptr)
            continue;
        read_and_strip(line);
        ++checked;
        zassert_equal(count_of(stripped_, "lifecycle().bring_up("), 0U,
                      "%s reaches a lifecycle round without the boundary that clears the step "
                      "records", line);
        zassert_equal(count_of(stripped_, "take_over_unknown_state("), 0U,
                      "%s reaches a lifecycle round without the boundary that clears the step "
                      "records", line);
    }
    zassert_true(checked > 1U, "only %zu sources were checked, so this gate read nothing", checked);
}

ZTEST(tof_l7_quiesce_call_sites, test_no_other_application_source_writes_or_clears_a_step_record)
{
    FILE *const f{fopen(SOURCE_LIST_FILE, "rb")};
    zassert_not_null(f, "cannot read the generated source list");
    const size_t got{fread(list_, 1, sizeof list_ - 1U, f)};
    const bool whole{feof(f) != 0};
    fclose(f);
    zassert_true(whole, "the source list did not fit this gate's buffer");
    list_[got] = '\0';

    size_t seen{0};
    char *saved{nullptr};
    for (char *line = strtok_r(list_, "\n", &saved); line != nullptr;
         line = strtok_r(nullptr, "\n", &saved)) {
        if (*line == '\0')
            continue;
        ++seen;
        read_and_strip(line);
        const bool is_adapter{strstr(line, "tof_package_device.cpp") != nullptr};
        const bool is_module{strstr(line, "tof_l7_step_record.cpp") != nullptr};
        if (is_adapter || is_module)
            continue;
        zassert_equal(count_of(stripped_, kStepWriter), 0U, "%s writes a step record", line);
        zassert_equal(count_of(stripped_, kStepClearer), 0U, "%s clears the step records", line);
        zassert_equal(count_of(stripped_, kStepBegin), 0U,
                      "%s opens a round. Only the command that drives a bring-up may, and only "
                      "with the lifecycle's own answer", line);
    }
    zassert_true(seen > 1U, "the generated source list had %zu entries", seen);
}

/* ---- the watchdog's only view of the Package path ---- */

/* WHAT MAKES A HOOK ONE THE WATCHDOG HAS TO SEE: it reaches the sensor layer or calls the ULD
 * directly, which means it can sit on a stalled I2C transfer. Derived from the source rather than
 * listed, because a listed gate says nothing about the hook somebody adds next. */
bool hook_touches_the_bus(const char *body)
{
    return strstr(body, "tof_l7::") != nullptr || strstr(body, "vl53l7cx_is_alive(") != nullptr ||
           strstr(body, "vl53l7cx_stop_ranging(") != nullptr;
}

ZTEST(tof_l7_quiesce_call_sites, test_every_hook_that_touches_the_bus_is_a_watched_operation)
{
    read_and_strip(DEVICE_SOURCE);

    size_t watched{0}, found{0};
    for (const char *at = strstr(stripped_, "\nint hook_"); at != nullptr;
         at = strstr(at + 1, "\nint hook_")) {
        size_t from{0}, to{0};
        if (!body_at(at, &from, &to))
            continue;
        const size_t len{to - from};
        zassert_true(len < kBufferSize, "a hook body does not fit");
        memcpy(body_, stripped_ + from, len);
        body_[len] = '\0';
        ++found;
        if (!hook_touches_the_bus(body_))
            continue;
        ++watched;
        zassert_true(strstr(body_, "watched_uld_operation") != nullptr,
                     "a hook in %s reaches the bus without declaring a watched_uld_operation. The "
                     "feeder's L7 monitor is the only thing judging this thread, and it judges an "
                     "operation that began and never ended -- an unbracketed call is invisible to "
                     "it. Body: %.200s",
                     DEVICE_SOURCE, body_);
    }
    zassert_true(found >= 8U, "only %zu hooks were located in %s, so this gate read nothing",
                 found, DEVICE_SOURCE);
    zassert_equal(watched, 8U,
                  "%zu hooks reach the bus; the eight ULD operations are probe, quiesce, close, "
                  "open, configure, start, read and stop. A ninth needs its own review against "
                  "bounds::l7_ms, not a quiet pass here",
                  watched);

    /* AND THE ONE THAT MUST NOT BE BRACKETED. ensure_verified() reads and verifies the stored blob,
     * which nobody has timed against bounds::l7_ms on this board; bracketing it would risk a false
     * reset on a cold boot. */
    size_t from{0}, to{0};
    zassert_true(body_of("int hook_verified(", &from, &to),
                 "hook_verified() could not be located in %s", DEVICE_SOURCE);
    const size_t len{to - from};
    memcpy(body_, stripped_ + from, len);
    body_[len] = '\0';
    zassert_true(strstr(body_, "watched_uld_operation") == nullptr,
                 "hook_verified() is bracketed as a watched operation. The blob verification is "
                 "the longest thing on the boot path and is not timed against bounds::l7_ms, so "
                 "watching it trades a missed hang for a false reset");
}

ZTEST(tof_l7_quiesce_call_sites, test_the_runtime_announces_the_grid_sensor_before_it_binds)
{
    read_and_strip(RUNTIME_SOURCE);

    zassert_equal(count_of(stripped_, "tof_watchdog_feeder::set_l7_expected("), 1U,
                  "%s must announce the grid sensor exactly once", RUNTIME_SOURCE);
    zassert_equal(count_of(stripped_, "tof_watchdog_feeder::set_l7_expected(true)"), 1U,
                  "the announcement must be positive: without it the L7 monitor never arms and the "
                  "bracketing in the device layer is inert");

    const char *const announced{strstr(stripped_, "tof_watchdog_feeder::set_l7_expected(")};
    const char *const first_bind{strstr(stripped_, "tof_package_device::bind(")};
    zassert_not_null(first_bind, "%s never binds", RUNTIME_SOURCE);
    zassert_true(announced < first_bind,
                 "the grid sensor is announced after the first bind, so the probe inside that bind "
                 "-- already a bracketed operation -- runs unjudged");
}
