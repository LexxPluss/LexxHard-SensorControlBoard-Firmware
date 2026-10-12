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

#if !defined(ADAPTER_SOURCE) || !defined(SOURCE_LIST_FILE)
#error "CMake must pass ADAPTER_SOURCE and SOURCE_LIST_FILE"
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

char body_[kBufferSize];

void copy_quiesce_body()
{
    size_t from{0}, to{0};
    zassert_true(body_of("int hook_quiesce(", &from, &to),
                 "hook_quiesce() could not be located in %s, so where the record is written and "
                 "what clock it reads cannot be established",
                 ADAPTER_SOURCE);
    const size_t len{to - from};
    zassert_true(len < kBufferSize, "the hook body does not fit");
    memcpy(body_, stripped_ + from, len);
    body_[len] = '\0';
}

}  // namespace

ZTEST_SUITE(tof_l7_quiesce_call_sites, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_l7_quiesce_call_sites, test_the_record_is_written_once_inside_the_quiesce_hook)
{
    read_and_strip(ADAPTER_SOURCE);
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
        const bool is_adapter{strstr(line, "tof_package_bench.cpp") != nullptr};
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
    read_and_strip(ADAPTER_SOURCE);
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
