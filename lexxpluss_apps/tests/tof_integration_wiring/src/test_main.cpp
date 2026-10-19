/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * THE WIRING, WITH THE GRID FLAG ON.
 *
 * Every other suite that links tof_cliff_runtime.cpp builds it WITHOUT ENABLE_TOF_L7_ULD, so the
 * grid publisher's init, the two-publisher fan-out and the grid sink are compiled in neither
 * direction there. Twenty green cases next door said nothing about any of them.
 *
 * What is production here: the runtime, both publishers, both packers, the acquisition layer, the
 * authority, and tof_cliff_can.cpp -- including the real grid authorisation conversion. What is
 * faked: the chain's control lines and the vendor ULDs, because what is under test is which
 * modules are connected to which, not what ST's drivers do.
 *
 * NOTHING REACHES A BUS, and that is a property of the clamp rather than of the fixture. can2 here
 * is a loopback controller, present because tof_cliff_can.cpp names the node at compile time; no
 * case asserts on a frame, because the PROVEN clamp refuses every grid before a send is reached.
 * That is the state these cases want: it shows the modules are connected while the gate stays shut.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "fake_chain.hpp"
#include "tof_acquisition.hpp"
#include "tof_chain_controller.hpp"
#include "tof_chain_spec.hpp"
#include "tof_cliff_can.hpp"
#include "tof_cliff_publisher.hpp"
#include "tof_cliff_runtime.hpp"
#include "tof_grid_publisher.hpp"
#include "tof_mapping_authority.hpp"

namespace acq = lexxhard::tof_acq;
namespace au = lexxhard::tof_authority;
namespace can = lexxhard::tof_cliff_can;
namespace gpub = lexxhard::tof_grid_pub;
namespace pub = lexxhard::tof_cliff_pub;
namespace rt = lexxhard::tof_cliff_runtime;

namespace {

bool glue_is_ready{true};

K_THREAD_STACK_DEFINE(wiring_acq_stack, 2048);

rt::config timing()
{
    rt::config c{};

    c.cycle_period_ms = 50;
    c.health_period_ms = 20;
    c.stop_join_timeout_ms = 400;
    c.thread_priority = K_PRIO_PREEMPT(5);
    c.cliff_timing_budget_us = 21000;
    c.cliff_distance_mode = 3;
    c.grid_frequency_hz = 7;
    return c;
}

void before(void *)
{
    glue_is_ready = true;
    rt::reset_for_test();
    rt::set_thread_stack_for_test(wiring_acq_stack, K_THREAD_STACK_SIZEOF(wiring_acq_stack));
    au::reset_epoch_history_for_test();
}

} // namespace

/* The chain controller's two symbols, supplied here rather than by linking the file: that one
 * reaches real GPIO and a real bxCAN, and nothing in this suite is about either. The lock is a real
 * k_mutex for the same reason the commissioning suite uses one -- "recursive locking does not
 * deadlock" is not a property a fake can demonstrate. */
namespace lexxhard::tof_chain_controller {

K_MUTEX_DEFINE(wiring_chain_mutex);

bool glue_ready()
{
    return glue_is_ready;
}

k_mutex &chain_lock()
{
    return wiring_chain_mutex;
}

}  // namespace lexxhard::tof_chain_controller

ZTEST_SUITE(tof_integration_wiring, NULL, NULL, before, NULL, NULL);

/* THE GRID DESCRIPTORS CARRY THE REAL TABLE AND THE DEPLOYMENT'S RATE. In a flag-off build they
 * carry the -ENOSYS stub, so this is the binding that build cannot have. */
ZTEST(tof_integration_wiring, test_the_grid_positions_are_bound_to_the_real_adapter)
{
    zassert_equal(rt::bootstrap(timing()), 0, "stage %s", rt::stage_name(rt::current_stage()));

    const acq::source_desc *d{rt::descriptors_for_test()};
    zassert_not_null(d);

    int grid{0};
    for (size_t i = 0; i < lexxhard::tof_chain::dasher_spec().positions; ++i) {
        if (d[i].kind != acq::model::l7_grid)
            continue;
        ++grid;
        zassert_true(d[i].grid_ops == &acq::l7_grid_ops(),
                     "position %zu is not bound to the production adapter", i + 1);
        zassert_equal(d[i].grid_frequency_hz, 7, "position %zu got rate %u", i + 1,
                      d[i].grid_frequency_hz);
        zassert_not_null(d[i].dev, "position %zu has no device object", i + 1);
        zassert_not_null(d[i].scratch, "position %zu has no scratch", i + 1);
        /* And no L4 profile on it -- acquisition refuses a descriptor that carries one. */
        zassert_equal(d[i].cliff_timing_budget_us, 0U, "");
        zassert_equal(d[i].cliff_distance_mode, 0, "");
    }
    zassert_equal(grid, 2, "the spec has two grid positions");
}

/* THE WHOLE PATH, AND THE COUNTER SAYS WHICH LINK HELD. A stubbed sensor answers with a full grid;
 * acquisition reads it, the adapter converts it, the hook carries it to the publisher, and the
 * clamp refuses it. Three different failures are told apart by WHICH counter moves:
 *
 *   nothing moves                 the hook is not wired, or the publisher was never initialised
 *   suppressed_cycle_not_begun    on_grid_sample arrived but on_cycle_begin did not -- the fan-out
 *                                 has one arm, which is exactly what a flag-off build compiles
 *   suppressed_not_proven         everything is connected and the clamp did its job
 *
 * In a build without the grid flag none of this exists, which is why twenty green cases next door
 * said nothing about it. */
ZTEST(tof_integration_wiring, test_a_grid_sample_reaches_the_publisher_and_the_clamp_refuses_it)
{
    zassert_equal(rt::bootstrap(timing()), 0, "stage %s", rt::stage_name(rt::current_stage()));
    zassert_equal(acq::bring_up(), 0);

    gpub::counters before{};
    gpub::copy_counters(before);

    acq::run_cycle();

    gpub::counters after{};
    gpub::copy_counters(after);

    zassert_equal(after.suppressed_cycle_not_begun, before.suppressed_cycle_not_begun,
                  "a sample arrived for a cycle the publisher was never told about: the fan-out "
                  "did not reach on_cycle_begin");
    zassert_true(after.suppressed_not_proven > before.suppressed_not_proven,
                 "no grid sample reached the publisher at all (before %u, after %u): the "
                 "on_grid_sample hook is not wired, or the publisher was never initialised",
                 before.suppressed_not_proven, after.suppressed_not_proven);
    zassert_equal(after.grids_sent, before.grids_sent,
                  "the clamp must refuse every grid; lifting it is a release decision");

    acq::stop();
}

/* THE GRID PERMISSION COMES FROM THE GRID'S OWN MASK. This is the defect the suite was written
 * for: the conversion read enumerated_mask, which masks_from() builds from the four L4 ROLES, so
 * its bits 0 and 1 are the front_left and rear_left CLIFF sensors. They agree with the grid pair's
 * permission only by coincidence of the current chain profile.
 *
 * It is checked against a hand-built snapshot rather than through the live gate because reaching a
 * snapshot with these two masks disagreeing takes a full proof, and because under the clamp a gate
 * reading the wrong mask refuses exactly as a gate reading the right one does. The two cases are
 * the two directions of the confusion: cliff bits set with no grid proven must permit nothing, and
 * a grid bit set with no cliff bit must permit that grid. */
ZTEST(tof_integration_wiring, test_the_grid_permission_ignores_the_cliff_mask)
{
    au::snapshot n{};

    n.enumerated_mask = 0x3;   /* front_left and rear_left CLIFF sensors */
    n.model_verified_mask = 0x3;
    n.grid_source_mask = 0x0;  /* no grid source proven */
    n.failing_position = au::kNoFailingPosition;

    const gpub::authorisation deny{can::grid_authorisation_from(n)};
    for (int i = 0; i < gpub::kGridSources; ++i)
        zassert_false(deny.source_allowed[i],
                      "grid source %d was permitted by a CLIFF sensor's bit", i);
}

ZTEST(tof_integration_wiring, test_a_proven_grid_source_is_permitted_by_its_own_bit)
{
    au::snapshot n{};

    n.enumerated_mask = 0x0;   /* no cliff sensor enumerated */
    n.grid_source_mask = 0x2;  /* grid source 1, and only it */
    n.failing_position = au::kNoFailingPosition;

    const gpub::authorisation a{can::grid_authorisation_from(n)};
    zassert_false(a.source_allowed[0], "grid source 0 is not in the mask");
    zassert_true(a.source_allowed[1],
                 "grid source 1 is in grid_source_mask and was refused: the conversion is not "
                 "reading that field");

    /* boards_detected is not in the snapshot, so it is zero and says so rather than carrying a
     * plausible-looking number into a diagnostics field. */
    zassert_equal(a.boards_detected, 0, "");
    /* Bit 2 -- "the binding cannot be trusted" -- is NOT a field here, and asking for it is what
     * kept this suite from compiling at all. authorisation carried a chain-level
     * binding_untrusted until #118 removed it: its only effect was to set bit 2 on every source,
     * which the packer reads as a contradiction against source_allowed and refuses, taking the
     * whole cycle down over one position. source_allowed[] already says this per source, as the
     * enumerator's verdict rather than as a status bit, so there is nothing to assert here --
     * the input a conforming producer must never set no longer exists to be set. */
    /* kNoFailingPosition is 0xFF, not 0: a healthy chain must not report a failure here. */
    zassert_false(a.other_position_enumeration_failed, "");
}

/* AND THE CLAMP KEEPS THE GATE SHUT whatever the masks say. This is why nobody would have noticed
 * the mask confusion, and it is also the thing that must not change by accident: wiring the sender
 * did not open the gate, and lifting the clamp is a separate release decision. */
ZTEST(tof_integration_wiring, test_the_live_grid_gate_is_never_proven)
{
    au::snapshot n{};

    n.state = acq::mapping_state::proven;
    n.grid_source_mask = 0x3;
    zassert_not_equal(can::grid_authorisation_from(n).state, acq::mapping_state::proven,
                      "the clamp must refuse a snapshot that believes PROVEN");

    zassert_not_equal(can::grid_production_authorisation().state, acq::mapping_state::proven,
                      "the live gate must not be PROVEN either");
}

/* Same reason as the commissioning suite: tof_cliff_runtime.cpp marks the watchdog's baseline from
 * inside the keying commit, and linking the real feeder would pull the task watchdog, the tombstone
 * and its DTCM reservation into a suite about which modules are connected to which. */
namespace lexxhard::tof_watchdog_feeder {

void set_baseline_point(bool)
{
}

void set_acquisition_expected(bool)
{
}

}  // namespace lexxhard::tof_watchdog_feeder
