/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The record the task watchdog leaves behind when it decides to reset the board.
 *
 * Almost everything here is about refusing to believe a record rather than about writing one. That
 * is the right emphasis: the watchdog causes resets on purpose, so the tombstone is what stands
 * between "the SCB restarted" and a diagnosis, and a tombstone that can be read back half-written
 * would point at the wrong subsystem with total confidence. A reset can land between any two
 * stores, which is why the body and its checksum go down first, a barrier follows, and the
 * committed magic is written last -- so a record carrying its magic has a body that was already
 * complete when the magic appeared.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "tof_watchdog_tombstone.hpp"

namespace
{

namespace tomb = lexxhard::tof_watchdog_tombstone;

constexpr size_t kWords{tomb::kSize / sizeof(uint32_t)};

uint32_t region_[kWords];

/* What the observer saw. The whole format rests on the body being complete at this instant and the
 * magic still absent, so the suite inspects it here rather than taking the comment's word. The
 * observer does NOT supply the barrier -- that is issued inside write_once, unconditionally. */
struct observation {
    bool ran{false};
    uint32_t committed_at_fence{0xDEADBEEFU};
    uint32_t reason_at_fence{0};
    uint32_t end_magic_at_fence{0};
    uint32_t checksum_at_fence{0};
};

observation obs_{};

void observe(void *)
{
    obs_.ran = true;
    obs_.committed_at_fence = region_[tomb::kOffCommitted];
    obs_.reason_at_fence = region_[tomb::kOffReason];
    obs_.end_magic_at_fence = region_[tomb::kOffEndMagic];
    obs_.checksum_at_fence = region_[tomb::kOffChecksum];
}

tomb::record sample()
{
    tomb::record r{};
    r.build_id[0] = 0x332E3620;  r.build_id[1] = 0x672D30;
    r.phase = 2;                 /* stopped */
    r.reason = 0x0480;           /* silent_health | stuck_l7, say */
    r.stopped_ms = 123456;
    r.last_fed_ms = 122000;
    r.acq_begun = 9001;          r.acq_ended = 9001;
    r.send_acq_begun = 7;        r.send_acq_ended = 6;
    r.send_workq_begun = 400;    r.send_workq_ended = 400;
    r.health_begun = 51;         r.health_ended = 50;
    r.l7_begun = 3;              r.l7_ended = 2;
    r.zcan_loops = 88888;
    r.long_active = 1;
    r.long_began_ms = 120000;
    r.boot_seq = 4;
    r.feed_rc = -5;
    return r;
}

void clear_region()
{
    memset(region_, 0, sizeof(region_));
    obs_ = observation{};
}

}  // namespace

ZTEST_SUITE(tof_watchdog_tombstone, nullptr, nullptr, nullptr, nullptr, nullptr);

/* THE ORDERING, which is the one property everything else depends on. */
ZTEST(tof_watchdog_tombstone, test_the_body_is_complete_and_the_magic_absent_before_the_barrier)
{
    clear_region();
    tomb::write_once(region_, sample(), observe, nullptr);

    zassert_true(obs_.ran, "the observation point must actually be reached");
    zassert_equal(obs_.committed_at_fence, 0U,
                  "the magic must not exist yet, or a reset here leaves a believable half-record");
    zassert_equal(obs_.reason_at_fence, 0x0480U, "the body was already down");
    zassert_equal(obs_.end_magic_at_fence, tomb::kMagicEnd);
    zassert_not_equal(obs_.checksum_at_fence, 0U, "the checksum belongs to the body, not the magic");
}

/* A reset landing before the magic leaves a record nobody may interpret. */
ZTEST(tof_watchdog_tombstone, test_a_record_interrupted_before_its_magic_is_refused)
{
    clear_region();
    tomb::write_once(region_, sample(), observe, nullptr);
    region_[tomb::kOffCommitted] = 0;   /* as if the reset arrived one store early */

    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::not_committed);
    zassert_equal(out.reason, 0U, "a refused record hands back nothing to misread");
}

ZTEST(tof_watchdog_tombstone, test_a_written_record_reads_back_field_for_field)
{
    clear_region();
    const tomb::record in{sample()};
    tomb::write_once(region_, in, observe, nullptr);

    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::valid);
    zassert_equal(out.reason, in.reason);
    zassert_equal(out.phase, in.phase);
    zassert_equal(out.stopped_ms, in.stopped_ms);
    zassert_equal(out.last_fed_ms, in.last_fed_ms);
    zassert_equal(out.acq_begun, in.acq_begun);
    zassert_equal(out.acq_ended, in.acq_ended);
    zassert_equal(out.send_acq_begun, in.send_acq_begun);
    zassert_equal(out.send_acq_ended, in.send_acq_ended);
    zassert_equal(out.send_workq_begun, in.send_workq_begun);
    zassert_equal(out.send_workq_ended, in.send_workq_ended);
    zassert_equal(out.health_begun, in.health_begun);
    zassert_equal(out.health_ended, in.health_ended);
    zassert_equal(out.l7_begun, in.l7_begun);
    zassert_equal(out.l7_ended, in.l7_ended);
    zassert_equal(out.zcan_loops, in.zcan_loops);
    zassert_equal(out.long_active, in.long_active);
    zassert_equal(out.long_began_ms, in.long_began_ms);
    zassert_equal(out.boot_seq, in.boot_seq);
    zassert_equal(out.feed_rc, in.feed_rc, "the driver's rc is why a refused feed is not a withheld one");
    zassert_equal(out.build_id[0], in.build_id[0]);
}

/* THE TWO SENDERS MUST NOT BE CONFUSABLE. They are the reason the watchdog can tell a wedged
 * acquisition sender from a live heartbeat, and a tombstone that swapped them would send somebody
 * to the wrong subsystem with the record's full authority behind it. */
ZTEST(tof_watchdog_tombstone, test_the_two_send_slots_land_in_their_own_fields)
{
    clear_region();
    tomb::record in{};
    in.send_acq_begun = 0x11111111U;   in.send_acq_ended = 0x22222222U;
    in.send_workq_begun = 0x33333333U; in.send_workq_ended = 0x44444444U;
    tomb::write_once(region_, in, observe, nullptr);

    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::valid);
    zassert_equal(out.send_acq_begun, 0x11111111U);
    zassert_equal(out.send_workq_begun, 0x33333333U);
    zassert_not_equal(out.send_acq_begun, out.send_workq_begun);
}

/* Any single word going bad after the fact is caught. */
ZTEST(tof_watchdog_tombstone, test_a_corrupted_word_fails_the_checksum)
{
    clear_region();
    tomb::write_once(region_, sample(), observe, nullptr);
    region_[tomb::kOffZcanLoops] ^= 1U;

    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::bad_checksum);
}

/* Two words swapped is the failure a plain sum would miss, which is why the checksum rotates. */
ZTEST(tof_watchdog_tombstone, test_two_swapped_words_do_not_produce_the_same_checksum)
{
    clear_region();
    tomb::write_once(region_, sample(), observe, nullptr);
    const uint32_t a{region_[tomb::kOffAcqBegun]};
    region_[tomb::kOffAcqBegun] = region_[tomb::kOffSendWorkqBegun];
    region_[tomb::kOffSendWorkqBegun] = a;

    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::bad_checksum);
}

/* A reader older than the image that wrote the record must say so rather than guess. */
ZTEST(tof_watchdog_tombstone, test_an_unknown_version_is_refused_before_anything_else_is_believed)
{
    clear_region();
    tomb::write_once(region_, sample(), observe, nullptr);
    region_[tomb::kOffVersion] = tomb::kVersion + 1;

    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::wrong_version,
                  "a newer format is a newer format, not a corrupt record");
}

ZTEST(tof_watchdog_tombstone, test_a_disagreeing_size_is_refused)
{
    clear_region();
    tomb::write_once(region_, sample(), observe, nullptr);
    region_[tomb::kOffSize] = 128;

    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::wrong_size);
}

/* The tail is what catches a record that was overwritten from the far end, or one shorter than it
 * claims. */
ZTEST(tof_watchdog_tombstone, test_a_missing_tail_is_refused)
{
    clear_region();
    tomb::write_once(region_, sample(), observe, nullptr);
    region_[tomb::kOffEndMagic] = 0;

    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::bad_end_magic);
}

/* An empty region is not a record. On a board that has never tripped the watchdog this is the
 * ordinary case, and it must not read as a fault with every field zero. */
ZTEST(tof_watchdog_tombstone, test_an_untouched_region_is_not_a_record)
{
    clear_region();
    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::not_committed);
}

/* A leftover record from an earlier boot must be replaced whole, never blended with the new one. */
ZTEST(tof_watchdog_tombstone, test_a_stale_record_is_cleared_rather_than_written_over)
{
    clear_region();
    tomb::record old{sample()};
    old.reason = 0xFFFFFFFFU;
    old.boot_seq = 99;
    tomb::write_once(region_, old, observe, nullptr);

    tomb::record fresh{};
    fresh.reason = 0x1;
    tomb::write_once(region_, fresh, observe, nullptr);

    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::valid);
    zassert_equal(out.reason, 0x1U);
    zassert_equal(out.boot_seq, 0U, "nothing of the old record may survive into the new one");
    zassert_equal(out.stopped_ms, 0U);
}

/* The placement, asserted rather than described. The diagnostic blocks are at 0x2001F000 (0x1f4
 * bytes) and 0x2001F200 (0x634 bytes); this sits above both and ends exactly at the top of DTCM. */
ZTEST(tof_watchdog_tombstone, test_the_region_clears_both_diagnostic_blocks_and_ends_at_dtcm_top)
{
    zassert_equal(tomb::kSize, 256U);
    zassert_true(tomb::kAddress >= 0x2001F200U + 0x634U, "must not overlap the i2c forensics block");
    zassert_true(tomb::kAddress >= 0x2001F000U + 0x1f4U, "must not overlap the hang record");
    zassert_equal(tomb::kAddress + tomb::kSize, 0x20020000U, "ends at the top of the 128 KiB DTCM");
    zassert_true(tomb::kAddress >= 0x2001F000U, "inside the 4 KiB the overlay removes from the region");
}

/* THE BARRIER IS NOT OPTIONAL AND NOT THE CALLER'S. An earlier version took it as a nullable
 * callback, so a call site that passed nothing still got a committed magic and the format's one
 * guarantee quietly did not hold. A record written with no observer at all must be just as valid. */
ZTEST(tof_watchdog_tombstone, test_a_record_written_with_no_observer_is_still_committed)
{
    clear_region();
    tomb::write_once(region_, sample());

    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::valid,
                  "ordering is issued inside write_once, not supplied by the caller");
    zassert_equal(out.reason, 0x0480U);
}

/* Pinned so that a port which loses its barrier fails a test here rather than shipping records
 * nobody can trust. */
ZTEST(tof_watchdog_tombstone, test_this_build_has_a_real_barrier)
{
    zassert_true(tomb::kBarrierAvailable,
                 "without a barrier write_once refuses to commit, and this build must not be that");
}

/* A record written by an older image is refused rather than half-read. The layout grew a field, so
 * this is not hypothetical: a v1 reader and a v2 record disagree about what offset 26 means. */
ZTEST(tof_watchdog_tombstone, test_the_layout_version_is_current_and_older_records_are_refused)
{
    zassert_equal(tomb::kVersion, 2U);
    clear_region();
    tomb::write_once(region_, sample());
    region_[tomb::kOffVersion] = 1;
    tomb::record out{};
    zassert_equal(tomb::read(region_, out), tomb::status::wrong_version);
}

ZTEST(tof_watchdog_tombstone, test_every_status_has_its_own_name)
{
    using tomb::status;
    zassert_true(strcmp(tomb::status_name(status::valid), "valid") == 0);
    zassert_true(strcmp(tomb::status_name(status::not_committed), "not_committed") == 0);
    zassert_true(strcmp(tomb::status_name(status::wrong_version), "wrong_version") == 0);
    zassert_true(strcmp(tomb::status_name(status::wrong_size), "wrong_size") == 0);
    zassert_true(strcmp(tomb::status_name(status::bad_end_magic), "bad_end_magic") == 0);
    zassert_true(strcmp(tomb::status_name(status::bad_checksum), "bad_checksum") == 0);
}
