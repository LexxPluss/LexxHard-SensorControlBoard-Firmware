/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "tof_cliff_runtime.hpp"

#if defined(ENABLE_TOF_L7_ULD)
#include "tof_l7_sensor.hpp"
#endif

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "tof_acquisition.hpp"
#include "tof_chain_controller.hpp"
#include "tof_chain_spec.hpp"
#include "tof_cliff_can.hpp"
#include "tof_cliff_publisher.hpp"
#if defined(ENABLE_TOF_L7_ULD)
#include "tof_grid_publisher.hpp"
#endif
#include "tof_mapping_authority.hpp"
#include "tof_mapping_proof.hpp"
#include "tof_watchdog_feeder.hpp"

namespace lexxhard::tof_cliff_runtime {

LOG_MODULE_REGISTER(tof_cliff_runtime);

namespace {

namespace acq = lexxhard::tof_acq;
namespace au = lexxhard::tof_authority;
namespace can = lexxhard::tof_cliff_can;
namespace enm = lexxhard::tof_enum;
namespace pf = lexxhard::tof_proof;
namespace pub = lexxhard::tof_cliff_pub;
#if defined(ENABLE_TOF_L7_ULD)
namespace gpub = lexxhard::tof_grid_pub;
#endif

/* ALL of the subsystem's saved state lives here, at file scope.
 *
 * Every one of these is held by pointer or reference by somebody else for the life of the image:
 * the authority keeps `spec_`, the publisher keeps `descs_`, acquisition keeps `descs_` and the
 * device/scratch pointers inside them. A stack copy anywhere in this file would dangle the instant
 * bootstrap() returned, and the symptom would not be a crash at that moment -- it would be a
 * mapping check comparing against reused stack memory some minutes later. */
enm::chain_spec spec_{tof_chain::dasher_spec()};
acq::source_desc descs_[acq::kMaxSources];

/* The ULD's own resident cost, four instances of it. One scratch for the whole chain: each sample
 * is copied out before the next sensor is read, which is what makes sharing it correct -- see the
 * concurrency note in tof_cliff_sensor.h. */
constexpr int kCliffSensors{4};
VL53L4CX_Object_t objs_[kCliffSensors];
struct tof_cliff_scratch scratch_;

/* The replay guard's history, and the one thing here that is per sensor rather than shared. A
 * stream count only means something against the previous result from the SAME device, so sharing
 * this the way scratch_ is shared would let one sensor's count refuse its neighbour's reading. It
 * lives here, beside the device objects, for the same reason they do: acquisition keeps the pointer
 * and the storage has to outlive every cycle. */
struct tof_cliff_stream_state streams_[kCliffSensors];

#if defined(ENABLE_TOF_L7_ULD)
/* The grid pair's device objects and their shared scratch.
 *
 * SHARED, like the cliff scratch and for the same reason: it holds one in-flight transfer, and the
 * acquisition thread is the only thing that reads a sensor. A second reader would be a second
 * caller inside the ULD, whose port keeps one transport record -- which is what the chain lock and
 * the single-thread rule exist to prevent, not something a second buffer would fix.
 *
 * NOT the object the boot recovery uses. That one runs before any open and is zeroed on every
 * call; these hold live sessions from bring-up onwards. */
constexpr int kGridSensors{2};
tof_l7::sensor grid_objs_[kGridSensors];
tof_l7::scratch grid_scratch_;
#endif
/* The acquisition thread's stack, sized from the devicetree.
 *
 * It lives here rather than in tof_acquisition because a Zephyr thread stack is a compile-time sized
 * object and this is the layer that reads the devicetree; keeping the size out of the acquisition
 * module is also what lets that module stay host-testable with no devicetree at all. Its cost is
 * static RAM and shows up in the budget accordingly. */
#if DT_NODE_EXISTS(DT_PATH(tof_chain))
K_THREAD_STACK_DEFINE(acq_stack_, DT_PROP(DT_PATH(tof_chain), acq_stack_size));
#endif

/* Where the stack came from, resolved once. On a board it is the devicetree's; in a host suite it is
 * whatever the suite handed over. There is deliberately no third case: a compiled-in fallback would
 * be a size nobody chose, and it would ship. */
k_thread_stack_t *stack_{nullptr};
size_t stack_size_{0};

stage stage_{stage::not_started};
/* Kept from the bootstrap, because start_acquisition() happens later -- inside a commissioning run --
 * and re-deriving these from the devicetree there would be a second source for the same numbers. */
config cfg_{};
/* The epoch the descriptors were keyed under, and whether they were keyed at all.
 *
 * NOT a plain "applied" flag. A flag can only be correct if somebody remembers to clear it, and the
 * case that matters is the one nobody would remember: a successful proof, then a LATER proof that
 * fails. The authority revokes before every attempt, so the flag would still say "applied" while the
 * keys described a mapping that is no longer proven. Recording the epoch instead lets
 * mapping_applied() ASK the authority whether those keys are still the current mapping's, which
 * cannot go stale. */
bool keyed_{false};
uint8_t keyed_epoch_{0};

uint32_t now_ms()
{
    return k_uptime_get_32();
}

/* Descriptors built FROM THE SPEC, not from a hand-written table.
 *
 * Two things follow from that, both of which the old probe wiring got wrong: the addresses are the
 * ones enumeration actually assigns (0x2A.. for this chain, not 0x30..), and which positions are
 * cliff sensors is whatever the spec says rather than "the first four". A descriptor table that
 * disagrees with the spec describes a chain nobody built.
 *
 * role_id is deliberately NOT set here. It is the key the contract's source_id and per-cycle masks
 * are built from, and the only legitimate source for it is a mapping a proof installed. */
void build_descriptors(uint32_t cliff_timing_budget_us, uint8_t cliff_distance_mode,
                       uint8_t grid_frequency_hz)
{
    int cliff_index{0};
    int grid_index{0};

    ARG_UNUSED(grid_frequency_hz);
    ARG_UNUSED(grid_index);

    for (size_t i{0}; i < spec_.positions; ++i) {
        const enm::position_spec &ps{spec_.at[i]};
        acq::source_desc &d{descs_[i]};
        const bool cliff{ps.expected == enm::model::l4cx};

        d = acq::source_desc{};
        d.kind = cliff ? acq::model::l4_cliff : acq::model::l7_grid;
        d.addr_7bit = ps.target_addr;
        d.role_id = kRoleUnassigned;
        if (cliff && cliff_index < kCliffSensors) {
            d.dev = &objs_[cliff_index];
            d.scratch = &scratch_;
            d.stream = &streams_[cliff_index];
            d.ops = &acq::l4_cliff_ops();
            /* Carried per source rather than read from a global, so a descriptor is a complete
             * description of what one sensor will do. */
            d.cliff_timing_budget_us = cliff_timing_budget_us;
            d.cliff_distance_mode = cliff_distance_mode;
            ++cliff_index;
        } else {
#if defined(ENABLE_TOF_L7_ULD)
            /* THE REAL TABLE. Until now every grid position carried the -ENOSYS stub, so the grid
             * lifecycle the acquisition layer gained was reachable only from a test. This is the
             * binding, and it is why the flag-on image's capacity moves. */
            if (grid_index < kGridSensors) {
                d.dev = &grid_objs_[grid_index];
                d.scratch = &grid_scratch_;
                d.grid_ops = &acq::l7_grid_ops();
                d.grid_frequency_hz = grid_frequency_hz;
                ++grid_index;
            } else {
                /* More grid positions than objects. The spec and the storage disagree, which is a
                 * build-time mistake; leaving the stub makes it visible as "unsupported" rather
                 * than as a sensor fault, and init() refuses an l7_grid source whose table is
                 * neither bound nor the named stub. */
                d.ops = &acq::l7_stub_ops();
            }
#else
            /* The grid path in an image without the driver, and the explicit stub rather than a
             * null table or a copy of the cliff ops: wiring L7 to the L4 driver has to be a
             * deliberate act, not an oversight. */
            d.ops = &acq::l7_stub_ops();
#endif
        }
    }
}

int install_from_mapping(const pf::fingerprint &fp, uint8_t epoch)
{
    /* The authority's install callback, called INSIDE the commit transaction with the chain lock
     * held and before anything is published. Two-pass on purpose: validate every position into a
     * temporary, and only then write. A single pass that wrote as it validated would leave a
     * half-keyed table on the first refusal -- some corners published correctly, the rest not
     * published at all, and nothing on the wire to say which. */
    uint8_t keys[acq::kMaxSources];

    if (!ready())
        return -EPERM;
    if (fp.positions != spec_.positions)
        return -EINVAL;

    for (size_t i{0}; i < spec_.positions; ++i) {
        keys[i] = kRoleUnassigned;
        if (descs_[i].kind != acq::model::l4_cliff)
            continue;   // the grid stubs have no cliff source id and stay unassigned

        /* The address, because role_id is the key a measurement is published under: if the mapping
         * proved front_left at 0x2C and the descriptor acquisition reads is 0x2D, keying it would
         * publish one corner's distance under another corner's name -- and nothing on the wire
         * would look wrong. The role, because a mapping with an unknown role has no source id and
         * the contract cannot express a measurement without one. */
        if (fp.at[i].address != descs_[i].addr_7bit)
            return -EINVAL;

        const int8_t src{pf::source_id_of(fp.at[i].role)};

        if (src < 0)
            return -EINVAL;
        keys[i] = static_cast<uint8_t>(src);
    }

    for (size_t i{0}; i < spec_.positions; ++i)
        descs_[i].role_id = keys[i];
    keyed_ = true;
    keyed_epoch_ = epoch;
    LOG_INF("descriptors keyed from the installed mapping under epoch %u", epoch);

    /* COMMISSIONING IS FINISHED, and this is the moment -- inside the commit, after the last thing
     * that could refuse it. It is the signal the watchdog waits for before it judges anything:
     * until a mapping is installed the chain is still being enumerated and every activity is
     * legitimately irregular. An atomic store, so holding the chain lock costs nothing here.
     *
     * It is not, by itself, "before the first L7 open". The L7 monitor starts at the first open
     * wherever that falls, which is what makes this safe to set from the one place that knows
     * commissioning succeeded rather than from a place that also knows what happens next. */
    tof_watchdog_feeder::set_baseline_point(true);
    return 0;
}

int init_authority()
{
    au::config cfg{};

    cfg.runtime_spec = &spec_;
    cfg.begin_epoch = acq::begin_epoch;
    cfg.acquisition_idle = acq::is_idle;
    cfg.install_mapping = install_from_mapping;
    return au::init(cfg);
}

#if defined(ENABLE_TOF_L7_ULD)
int init_grid_publisher()
{
    gpub::config cfg{};

    cfg.sink = can::grid_sink();
    cfg.sources = descs_;
    cfg.source_count = static_cast<int>(spec_.positions);
    /* THE SAME STRUCTURALLY SHUT GATE as the cliff path, and that is what makes wiring this safe:
     * the clamp inside it makes PROVEN unreachable, and the packer refuses anything that is not
     * PROVEN. So this connects the path without opening it. */
    cfg.authorise = can::grid_production_authorisation;
    /* NO DEFAULT INVENTED HERE. VL53L7CX target_status 6 and 9 are the low-confidence verdicts, and
     * whether a hanging-object detector should trust them is a product decision nobody has taken.
     * False is the conservative reading and is stated rather than assumed. */
    cfg.accept_low_confidence = false;
    return gpub::init(cfg);
}
#endif

int init_publisher()
{
    pub::config cfg{};

    cfg.sink = can::sink();
    cfg.sources = descs_;
    cfg.source_count = static_cast<int>(spec_.positions);
    /* The production gate, which is structurally shut. A permissive one here would make every
     * measurement path reachable in a build nobody audited. */
    cfg.authorise = can::production_authorisation;
    return pub::init(cfg);
}

/* The fan-out. Deliberately not a loop over a table: there are two publishers, both known at
 * compile time, and a registry would be a mechanism for a variability that does not exist. */
void on_cycle_begin_all(uint32_t cycle_seq)
{
    pub::on_cycle_begin(cycle_seq);
#if defined(ENABLE_TOF_L7_ULD)
    gpub::on_cycle_begin(cycle_seq);
#endif
}

void on_cycle_complete_all(const acq::cycle_facts &facts)
{
    pub::on_cycle_complete(facts);
#if defined(ENABLE_TOF_L7_ULD)
    gpub::on_cycle_complete(facts);
#endif
}

int init_acquisition(const config &cfg)
{
    acq::config c{};

    c.sources = descs_;
    c.source_count = static_cast<int>(spec_.positions);
    c.periods.cycle_period_ms = cfg.cycle_period_ms;
    c.periods.health_period_ms = cfg.health_period_ms;
    /* TWO CONSUMERS, ONE HOOK. Acquisition has one on_cycle_begin and one on_cycle, and both
     * publishers need both: each latches its authorisation at the start of a cycle and flushes at
     * the end, and a publisher that missed either would send under a stale snapshot or hold a
     * packed frame past the cycle it belongs to. The fan-out lives here rather than in acquisition
     * because which publishers an image has is this layer's business. */
    c.hooks.on_cycle_begin = on_cycle_begin_all;
    c.hooks.on_cycle = on_cycle_complete_all;
    c.hooks.on_cliff_sample = pub::on_cliff_sample;
    c.hooks.on_cliff_health = pub::on_cliff_health;
#if defined(ENABLE_TOF_L7_ULD)
    /* The grid sink, which until now went nowhere: acquisition fired it only in a test. */
    c.hooks.on_grid_sample = gpub::on_grid_sample;
#endif
    c.mapping_state_provider = au::state_provider;
    c.now_ms = now_ms;
    return acq::init(c);
}

}  // namespace

#if DT_NODE_EXISTS(DT_PATH(tof_chain))
config config_from_devicetree()
{
    /* Five required properties, all of them provisional and all of them somebody's decision rather
     * than this file's. */
    /* Required properties, so a build that has not stated them does not compile. That is the whole
     * mechanism: the numbers are provisional either way, but they are provisional IN THE OVERLAY,
     * where they are visible in a diff and belong to whoever owns the deployment -- rather than
     * provisional in a scheduler, where the first plausible value silently becomes the spec. */
    return config{DT_PROP(DT_PATH(tof_chain), cycle_period_ms),
                  DT_PROP(DT_PATH(tof_chain), health_period_ms),
                  DT_PROP(DT_PATH(tof_chain), stop_join_timeout_ms),
                  DT_PROP(DT_PATH(tof_chain), acq_thread_priority),
                  DT_PROP(DT_PATH(tof_chain), cliff_timing_budget_us),
                  DT_PROP(DT_PATH(tof_chain), cliff_distance_mode),
                  DT_PROP(DT_PATH(tof_chain), grid_frequency_hz)};
}
#endif

int bootstrap(const config &cfg)
{
    /* Single-shot, and the reason is sharper than idempotence hygiene: a second au::init() clears
     * the installed mapping and any open attempt. The B6 probe used to call it on its own, so a
     * budget image and a product image did not have the same authority state machine -- and the
     * budget image is the one whose numbers get quoted. */
    if (stage_ != stage::not_started)
        return -EALREADY;

    if (cfg.cycle_period_ms == 0 || cfg.health_period_ms == 0 || cfg.stop_join_timeout_ms == 0)
        return -EINVAL;

    /* The ranging profile is checked HERE as well as in tof_acq::init(), and deliberately: this is
     * the stage that knows it is reading a deployment's devicetree, so it can NAME the property.
     * The same descriptor refused inside acquisition is a bare -EINVAL, and a bring-up engineer
     * reading a console would have no way to tell it from any other malformed configuration --
     * which is why the refusal below logs rather than only returning. */
    if (cfg.cliff_timing_budget_us == 0) {
        LOG_ERR("cliff-timing-budget-us is 0: the ranging profile is required and is not defaulted");
        return -EINVAL;
    }
    /* The grid rate, checked here for the same reason as the cliff profile: this is the stage that
     * knows it is reading a deployment's devicetree and can name the property. Refused rather than
     * clamped -- a rate silently moved to the nearest legal value is a rate nobody chose, and the
     * one place it is written down would stop being the one in force. Required of every image, so
     * the deployment states it once and does not have to know which drivers this build carries. */
    if (cfg.grid_frequency_hz == 0 || cfg.grid_frequency_hz > 15) {
        LOG_ERR("grid-frequency-hz is %u: 1..15, and the ULD's ceiling at 8x8 is 15",
                cfg.grid_frequency_hz);
        return -EINVAL;
    }
    if (cfg.cliff_distance_mode < 2 || cfg.cliff_distance_mode > 3) {
        /* 1 is SHORT, which the ULD refuses for an L4 part outright; 0 is absent. Named here
         * because the same value refused inside the ULD surfaces as a vendor error code. */
        LOG_ERR("cliff-distance-mode is %u: only 2 (MEDIUM) and 3 (LONG) are supported by an L4",
                cfg.cliff_distance_mode);
        return -EINVAL;
    }

#if DT_NODE_EXISTS(DT_PATH(tof_chain))
    stack_ = acq_stack_;
    stack_size_ = K_THREAD_STACK_SIZEOF(acq_stack_);
#endif

    /* The chain glue first, as a PRECONDITION rather than a convention.
     *
     * Acquisition drives i2c2 and the enable lines, so a bootstrap that ran before the chain
     * controller configured them would bring sensors up against unconfigured pins. That used to be
     * true in the B6 image, where a SYS_INIT ran the probe before main(): the ordering was expressed
     * only by where the calls happened to sit. Checking it here makes a wrong order a loud refusal
     * instead of a silent one. */
    if (!tof_chain_controller::glue_ready()) {
        stage_ = stage::chain_not_ready;
        LOG_ERR("chain glue not initialised: refusing to bootstrap the cliff subsystem");
        return -ENODEV;
    }

    build_descriptors(cfg.cliff_timing_budget_us, cfg.cliff_distance_mode,
                      cfg.grid_frequency_hz);

    if (const int rc{init_authority()}; rc != 0) {
        stage_ = stage::authority_failed;
        LOG_ERR("authority init failed (%d)", rc);
        return rc;
    }

    /* Tolerated on purpose. A board with no usable can2 still has to run its health path, because
     * "this board cannot reach the bus" is a diagnosis a consumer can only get from the absence of
     * frames plus the presence of the board -- and the publisher counts every failed send. */
    if (const int rc{can::init()}; rc != 0)
        LOG_WRN("CAN glue not available (%d): frames will be counted as send failures", rc);

#if defined(ENABLE_TOF_L7_ULD)
    /* Before the cliff publisher only because one of them has to be first; neither depends on the
     * other, and acquisition is not configured until after both. */
    if (const int rc{init_grid_publisher()}; rc != 0) {
        stage_ = stage::publisher_failed;
        LOG_ERR("grid publisher init failed (%d)", rc);
        return rc;
    }
#endif

    if (const int rc{init_publisher()}; rc != 0) {
        stage_ = stage::publisher_failed;
        LOG_ERR("publisher init failed (%d)", rc);
        return rc;
    }

    if (const int rc{init_acquisition(cfg)}; rc != 0) {
        stage_ = stage::acquisition_failed;
        LOG_ERR("acquisition init failed (%d)", rc);
        return rc;
    }

    cfg_ = cfg;
    stage_ = stage::ready;
    /* The heartbeat is already beating at this point, and it reports NOT_READY. That is the
     * intended state from power-on: a consumer must be able to tell "alive, mapping unproven" from
     * silence, and waiting for a proof before making a sound would make every unprovisioned board
     * indistinguishable from a dead one. */
    LOG_INF("cliff runtime ready: %u sources, cycle %u ms, health %u ms, mapping unproven",
            static_cast<unsigned>(spec_.positions), cfg.cycle_period_ms, cfg.health_period_ms);
    return 0;
}

stage current_stage()
{
    return stage_;
}

bool ready()
{
    return stage_ == stage::ready;
}

const char *stage_name(stage st)
{
    switch (st) {
    case stage::not_started:
        return "not_started";
    case stage::chain_not_ready:
        return "chain_not_ready";
    case stage::authority_failed:
        return "authority_failed";
    case stage::publisher_failed:
        return "publisher_failed";
    case stage::acquisition_failed:
        return "acquisition_failed";
    case stage::ready:
        return "ready";
    }
    return "?";
}

enm::chain_spec &spec()
{
    return spec_;
}

bool mapping_applied()
{
    /* Asked, not remembered. PROVEN plus the same epoch the keys were written under is the only
     * state in which those keys describe the current mapping; a revocation (LOST) or any newer
     * epoch invalidates them without this module having to be told. */
    const au::snapshot now{au::current()};

    return keyed_ && now.state == acq::mapping_state::proven && now.epoch == keyed_epoch_;
}

int start_acquisition()
{
    /* mapping_applied() rather than a flag, so this also requires the authority to be PROVEN right
     * now: descriptors keyed under a mapping that has since been revoked are exactly as wrong as
     * descriptors that were never keyed. */
    if (!ready() || !mapping_applied())
        return -EPERM;
    /* The thread, which from here on is the only thing allowed to touch a sensor. Behind this gate
     * because a thread that started before the descriptors were keyed would publish cycles whose
     * facts carry kRoleUnassigned -- and it would be publishing them continuously, not once.
     *
     * Nothing about the cycle counter is touched here: the first cycle of a new epoch must carry
     * cycle_seq 0, and begin_epoch() already did that inside the commit. */
    acq::thread_config tcfg{};

    tcfg.stack = stack_;
    tcfg.stack_size = stack_size_;
    tcfg.priority = cfg_.thread_priority;
    tcfg.join_timeout_ms = cfg_.stop_join_timeout_ms;

    const int rc{acq::start(tcfg)};

    /* EXPECTED AGAIN, and only now that the thread is actually running. The watchdog suspends the
     * cycle and the acquisition sender while acquisition is stopped, and this is the other end of
     * that: commissioning's quiesce says "stopped", a successful start says "expected".
     *
     * Not on the failure path, deliberately. A start that was refused leaves acquisition stopped,
     * and claiming otherwise would put the cycle back under bounds it cannot meet -- which is a
     * reset for a board whose only fault is that it has nothing proven to acquire with. */
    if (rc == 0)
        tof_watchdog_feeder::set_acquisition_expected(true);
    return rc;
}

#ifdef CONFIG_ZTEST
void set_thread_stack_for_test(k_thread_stack_t *stack, size_t size)
{
    stack_ = stack;
    stack_size_ = size;
}

void reset_for_test()
{
    /* The return is dropped deliberately, and that is only safe because of what this function is
     * for. teardown() refuses while a device will not stop, keeping the subsystem configured so
     * the cleanup can be retried; a case that ends with injected stop failures is a legitimate
     * end state and this reset must not assert on it. Clearing the injection and checking the
     * retire is the CALLER's fixture's job, which is where the acquisition suite does it. */
    (void)acq::teardown();
    stage_ = stage::not_started;
    keyed_ = false;
    keyed_epoch_ = 0;
    spec_ = tof_chain::dasher_spec();
    for (auto &d : descs_)
        d = acq::source_desc{};
}

const acq::source_desc *descriptors_for_test()
{
    return descs_;
}

int force_rebuild_descriptors_for_test()
{
    build_descriptors(cfg_.cliff_timing_budget_us, cfg_.cliff_distance_mode,
                      cfg_.grid_frequency_hz);
    keyed_ = false;
    return 0;
}
#endif

}  // namespace lexxhard::tof_cliff_runtime

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
