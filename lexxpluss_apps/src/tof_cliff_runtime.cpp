/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_cliff_runtime.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "tof_acquisition.hpp"
#include "tof_chain_controller.hpp"
#include "tof_chain_spec.hpp"
#include "tof_cliff_can.hpp"
#include "tof_cliff_publisher.hpp"
#include "tof_grid_publisher.hpp"
#if defined(ENABLE_TOF_L7_ULD)
/* The only file outside the adapter that names a VL53L7CX object: the two sensors and their
 * shared scratch are resident state, and this is the layer that owns resident state. */
#include "tof_l7_sensor.hpp"
#endif
#include "tof_mapping_authority.hpp"
#include "tof_mapping_proof.hpp"

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

#if defined(ENABLE_TOF_L7_ULD)
/* The two HANGING-OBJECT sensors. They look forward for half-height obstacles and read an 8x8
 * grid; the four objects above look down for a drop and read one distance each. This file is named
 * for the latter because it existed first, and the two must not be read as variants of one job.
 *
 * The same arrangement as the cliff objects above, and the same argument for sharing the scratch: the
 * adapter's read_once() copies the grid out of it before it returns, and every operation on
 * either sensor belongs to the acquisition thread while it owns the chain -- the concurrency
 * note at the top of tof_l7_sensor.hpp. A second scratch would be another VL53L7CX_ResultsData
 * of static RAM buying no property the one has not got.
 *
 * The CONFIGURATIONS are not shared, and cannot be: each holds its own device address, stream
 * count and calibration data, so one object for two sensors would be one sensor. */
constexpr int kGridSensors{2};
tof_l7::sensor l7_objs_[kGridSensors];
tof_l7::scratch l7_scratch_;
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

/* THE START CLAIM. Non-blocking, and it exists because start_acquisition() does something
 * DESTRUCTIVE before the layer below it can refuse: it returns the two hanging sensors' ULD
 * objects to empty, and those objects belong to a thread that may be using them right now.
 *
 * tof_acq::start() does hold the authoritative -EALREADY, but it holds it too late to help -- by
 * the time it answers, the reset has already happened, and a running acquisition thread would find
 * its next read refused at the state check with both sensors apparently unopened. There are two
 * entry points into this function, the commissioning worker and the shell, and a runtime API may
 * not assume its callers agree not to overlap.
 *
 * So the claim covers the whole sequence -- test the running state, reset, start -- and a caller
 * that loses it is told -EALREADY WITHOUT ANYTHING HAVING BEEN WRITTEN. atomic_cas rather than a
 * mutex because the answer to "somebody else is already starting" is to refuse, not to wait: this
 * runs from a commissioning path that must not block. */
atomic_t starting_{};

struct start_claim {
    const bool held;

    start_claim() : held{atomic_cas(&starting_, 0, 1)} {}
    ~start_claim()
    {
        if (held)
            atomic_clear(&starting_);
    }
    start_claim(const start_claim &) = delete;
    start_claim &operator=(const start_claim &) = delete;
};

uint32_t now_ms()
{
    return k_uptime_get_32();
}

/* Raw cycles, deliberately not microseconds: the acquisition layer subtracts first and converts
 * afterwards, which is what makes the 32-bit counter's ~19.9 s wrap harmless. See
 * tof_acq::config::now_cycles. */
uint32_t now_cycles()
{
    return k_cycle_get_32();
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
/* The ranging profile is a PARAMETER rather than read from cfg_, because this runs before cfg_ is
 * assigned: bootstrap validates every stage first and only adopts the configuration once they have
 * all succeeded, so reading cfg_ here would key every cliff descriptor with a zero budget and the
 * acquisition layer would then refuse the whole table. Moving the assignment earlier would mean a
 * failed bootstrap left its configuration behind, which is worse. */
void build_descriptors(uint32_t cliff_timing_budget_us, uint8_t cliff_distance_mode,
                       uint8_t grid_frequency_hz)
{
    int cliff_index{0};
    int grid_index{0};

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
            d.ops = &acq::l4_cliff_ops();
            d.cliff_timing_budget_us = cliff_timing_budget_us;
            d.cliff_distance_mode = cliff_distance_mode;
            ++cliff_index;
        } else {
#if defined(ENABLE_TOF_L7_ULD)
            if (grid_index < kGridSensors) {
                d.dev = &l7_objs_[grid_index];
                d.scratch = &l7_scratch_;
                d.grid_ops = &acq::l7_grid_ops();
                /* From the deployment's devicetree, through bootstrap, to the descriptor, and
                 * from here to the ULD by way of grid_ops.configure() during bring-up. This is
                 * the only assignment of it: there is no default anywhere on that path, and
                 * acquisition refuses a real grid table carrying a zero. */
                d.grid_frequency_hz = grid_frequency_hz;
                ++grid_index;
            } else {
                /* More grid positions than sensor objects. The spec would have to have grown
                 * without this constant growing with it; the stub keeps such a position
                 * describable instead of pointing it at an object that does not exist. */
                d.grid_ops = &acq::l7_grid_stub_ops();
            }
#else
            /* No ULD in this image, so nothing can open an L7. The named stub keeps the
             * position describable and every operation on it an explicit -ENOSYS. */
            (void)grid_frequency_hz;
            d.grid_ops = &acq::l7_grid_stub_ops();
#endif
        }
    }
}

/* BOTH publishers, every cycle, and in this order.
 *
 * The two sink slots hold one function pointer each and the cliff publisher already owned them.
 * Wiring the grid publisher in by replacing it would have silently stopped the four ranges that
 * work on hardware today -- a change with no compile error and no log line, whose only symptom is
 * an absence.
 *
 * The order is not arbitrary. Within one cycle the cliff publisher queues at most four
 * measurement frames and the grid publisher up to 34, and each flushes its own queue to the bus
 * in the call below. Cliff first means a grid's 34 sends cannot delay the frames of the path that
 * stops the machine. */
#if !defined(ENABLE_TOF_L7_ULD)
void on_grid_unreachable(int, uint32_t, const acq::source_facts &, const tof_l7::sample &)
{
    /* Unreachable rather than empty: without the ULD every grid operation is -ENOSYS, so no
     * sample can ever be fresh. If this body ever runs, the descriptor table was built by
     * something other than build_descriptors() above. */
}
#endif

void fanout_cycle_begin(uint32_t cycle_seq)
{
    pub::on_cycle_begin(cycle_seq);
#if defined(ENABLE_TOF_L7_ULD)
    gpub::on_cycle_begin(cycle_seq);
#endif
}

void fanout_cycle(const acq::cycle_facts &facts)
{
    pub::on_cycle_complete(facts);
#if defined(ENABLE_TOF_L7_ULD)
    gpub::on_cycle_complete(facts);
#endif
}

int install_from_mapping(const pf::fingerprint &fp, uint8_t epoch)
{
    /* The authority's install callback, called INSIDE the commit transaction with the chain lock
     * held and before anything is published. Two-pass on purpose: validate every position into a
     * temporary, and only then write. A single pass that wrote as it validated would leave a
     * half-keyed table on the first refusal -- some corners published correctly, the rest not
     * published at all, and nothing on the wire to say which. */
    uint8_t keys[acq::kMaxSources];
    /* Which grid source ids this fingerprint has already claimed. The contract has exactly two,
     * and two positions claiming one of them would publish two different sensors' grids under one
     * physical position -- with nothing on the wire to say so, because each frame is individually
     * well formed. */
    uint8_t grid_claimed{0};

    if (!ready())
        return -EPERM;
    if (fp.positions != spec_.positions)
        return -EINVAL;

    for (size_t i{0}; i < spec_.positions; ++i) {
        keys[i] = kRoleUnassigned;

        if (descs_[i].kind == acq::model::l7_grid) {
            /* THE GRID SOURCE ID COMES FROM THE FINGERPRINT'S OWN FIELD.
             *
             * Not source_id_of(role): that answers a cliff question -- which corner a mounting
             * role publishes as -- and an L7 has no cliff role at all. Not the descriptor index
             * either: the index is where the board sits on the chain, and the contract's
             * source_id is which side of the machine it looks at. They agree today by
             * construction of the spec, and an index used as a source id would keep agreeing
             * right up until somebody reorders the chain, at which point left and right swap
             * with nothing on the wire looking wrong. */
            const pf::position_fingerprint &pos{fp.at[i]};

            /* Five checks, and each one is a different way to publish a grid under a position
             * nobody proved. The position, because the fingerprint entry has to be the one for
             * THIS descriptor rather than a neighbour's. The model, because a health frame from
             * an L4 at a grid position would be an 8x8 claim about a part that has none. The
             * address, because that is the device acquisition will actually read. The verified
             * flag, because an unverified position proves nothing. The range, because the
             * contract has two grid sources and a third would be unrepresentable. */
            if (pos.position != static_cast<uint8_t>(i + 1))
                return -EINVAL;
            if (pos.expected != enm::model::l7cx)
                return -EINVAL;
            if (pos.address != descs_[i].addr_7bit)
                return -EINVAL;
            if (!pos.verified)
                return -EINVAL;
            if (pos.source_id < 0 || pos.source_id > 1)
                return -EINVAL;

            const uint8_t src{static_cast<uint8_t>(pos.source_id)};

            if ((grid_claimed & static_cast<uint8_t>(1U << src)) != 0)
                return -EINVAL;
            grid_claimed |= static_cast<uint8_t>(1U << src);
            keys[i] = src;
            continue;
        }

        if (descs_[i].kind != acq::model::l4_cliff)
            continue;   // a stubbed position has no source id and stays unassigned

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

#if defined(ENABLE_TOF_L7_ULD)
int init_grid_publisher()
{
    gpub::config cfg{};

    cfg.sink = can::grid_sink();
    cfg.sources = descs_;
    cfg.source_count = static_cast<int>(spec_.positions);
    cfg.authorise = can::grid_production_authorisation;
    /* The zone-confidence policy, stated rather than defaulted. false is what the packer's own
     * suite calls the conservative reading: VL53L7CX target_status 6 and 9 are low-confidence,
     * and whether a hanging-object decision may rest on them is a safety question nobody has
     * answered. Until somebody does, those zones travel as the invalid sentinel. */
    cfg.accept_low_confidence = false;
    return gpub::init(cfg);
}
#endif

int init_acquisition(const config &cfg)
{
    acq::config c{};

    c.sources = descs_;
    c.source_count = static_cast<int>(spec_.positions);
    c.periods.cycle_period_ms = cfg.cycle_period_ms;
    c.periods.health_period_ms = cfg.health_period_ms;
    c.hooks.on_cycle_begin = fanout_cycle_begin;
    c.hooks.on_cycle = fanout_cycle;
    c.hooks.on_cliff_sample = pub::on_cliff_sample;
#if defined(ENABLE_TOF_L7_ULD)
    c.hooks.on_grid_sample = gpub::on_grid_sample;
#else
    /* No ULD, so the grid table is the -ENOSYS stub and a fresh grid sample is structurally
     * impossible. The hook is still required by acquisition's own validation, and a sink that
     * cannot be reached is the honest thing to give it. */
    c.hooks.on_grid_sample = on_grid_unreachable;
#endif
    c.hooks.on_cliff_health = pub::on_cliff_health;
    c.mapping_state_provider = au::state_provider;
    c.now_ms = now_ms;
    c.now_cycles = now_cycles;
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

    /* The ranging profile is checked HERE as well as in tof_acq::init(), and deliberately: this is
     * the stage that can name what was wrong, and a bootstrap that reached the descriptors with a
     * zero budget would be refused there as a generic -EINVAL with no stage attached. */
    if (cfg.cliff_timing_budget_us == 0 || cfg.cliff_distance_mode < 2 ||
        cfg.cliff_distance_mode > 3)
        return -EINVAL;
    if (cfg.cycle_period_ms == 0 || cfg.health_period_ms == 0 || cfg.stop_join_timeout_ms == 0)
        return -EINVAL;
    /* The grid frequency, checked here for the same reason as the cliff profile: this is the
     * stage that can name what was wrong. 15 Hz is the ULD's ceiling at 8x8 and zero is the
     * absence of a choice; the adapter refuses the identical range again during bring-up, where
     * the refusal would arrive as one sensor failing to start rather than as a configuration
     * nobody accepted.
     *
     * Checked in every build, including one with no L7 ULD, because the value is the
     * deployment's statement about the machine rather than this image's opinion of it. An
     * overlay that names a frequency no firmware can honour should be refused by whichever
     * firmware reads it. */
    if (cfg.grid_frequency_hz == 0 || cfg.grid_frequency_hz > 15)
        return -EINVAL;

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

    build_descriptors(cfg.cliff_timing_budget_us, cfg.cliff_distance_mode, cfg.grid_frequency_hz);

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

    if (const int rc{init_publisher()}; rc != 0) {
        stage_ = stage::publisher_failed;
        LOG_ERR("publisher init failed (%d)", rc);
        return rc;
    }

#if defined(ENABLE_TOF_L7_ULD)
    /* BEFORE acquisition, and a hard failure rather than a warning. Acquisition is what calls
     * the grid sink, and a grid publisher that never initialised answers every call by returning
     * immediately: the chain would range, the samples would be read, and the 8x8 grids would go
     * nowhere -- which on the bus is indistinguishable from two sensors that are not there.
     *
     * The CAN glue above is tolerated when it fails because a board that cannot reach the bus
     * still has a health path to run. This one is not: there is nothing left to report with. */
    if (const int rc{init_grid_publisher()}; rc != 0) {
        stage_ = stage::grid_publisher_failed;
        LOG_ERR("grid publisher init failed (%d): refusing to start acquisition", rc);
        return rc;
    }
#endif

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
    case stage::grid_publisher_failed:
        return "grid_publisher_failed";
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

    const start_claim claim;

    if (!claim.held)
        return -EALREADY;   // another caller is inside this sequence; nothing here has been written

    /* BEFORE THE RESET, NOT AFTER. An acquisition thread that is running owns those objects, and
     * returning them to empty under it would leave every later read refused at the adapter's state
     * check -- two hanging sensors that stop reporting, with -EPERM and a stage number as the only
     * trace, on a machine where nothing is actually wrong.
     *
     * A thread that has EXITED but has not been joined is a different case and is deliberately not
     * caught here: nothing is touching the devices, so the reset is harmless, and tof_acq::start()
     * refuses it on its own with the same -EALREADY. */
    if (acq::thread_running())
        return -EALREADY;

    /* The thread, which from here on is the only thing allowed to touch a sensor. Behind this gate
     * because a thread that started before the descriptors were keyed would publish cycles whose
     * facts carry kRoleUnassigned -- and it would be publishing them continuously, not once.
     *
     * Nothing about the cycle counter is touched here: the first cycle of a new epoch must carry
     * cycle_seq 0, and begin_epoch() already did that inside the commit. */
#if defined(ENABLE_TOF_L7_ULD)
    /* THE L7 OBJECTS GO BACK TO EMPTY BEFORE EVERY BRING-UP, and this is not hygiene.
     *
     * The adapter's lifecycle is per SESSION: open() accepts an empty object and stop() leaves a
     * configured one, so a second start would be refused at the state check -- on the second
     * commissioning of a boot, both grid sensors would simply fail to come up, with -EPERM and a
     * stage number as the only trace. The objects are resident because the ULD needs somewhere to
     * live; the session over them is not, and a bring-up follows a proof that has just toggled
     * every enable line on the chain, so the device on the other end really is starting again.
     *
     * Here rather than in the adapter: the adapter's refusal is what makes a stale reopen visible,
     * and this layer is the one that knows a new session is beginning. */
    for (auto &object : l7_objs_)
        object = tof_l7::sensor{};
#endif

    acq::thread_config tcfg{};

    tcfg.stack = stack_;
    tcfg.stack_size = stack_size_;
    tcfg.priority = cfg_.thread_priority;
    tcfg.join_timeout_ms = cfg_.stop_join_timeout_ms;
    return acq::start(tcfg);
}

int probe_position(size_t position_1based, probe_result &out, unsigned attempts, unsigned gap_ms)
{
    out = probe_result{};

    if (attempts == 0 || attempts > kMaxProbeAttempts || gap_ms > kMaxProbeGapMs)
        return -EINVAL;
    if (!ready())
        return -EPERM;
    /* The acquisition thread owns the ULD while it runs. Refusing rather than interleaving is the
     * same rule tof_acq's own guard enforces; this path bypasses that guard by driving the ops table
     * directly, so it has to check for itself. */
    if (acq::thread_running())
        return -EBUSY;
    if (position_1based == 0 || position_1based > spec_.positions)
        return -EINVAL;

    const size_t i{position_1based - 1};

    if (descs_[i].kind != acq::model::l4_cliff || descs_[i].ops == nullptr)
        return -ENOTSUP;

    out.addr_7bit = descs_[i].addr_7bit;
    out.role_id = descs_[i].role_id;

    const acq::source_ops &ops{*descs_[i].ops};
    void *dev{descs_[i].dev};

    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    out.attempted = true;
    out.open_rc = ops.open(dev, descs_[i].addr_7bit, &out.status);
    if (out.open_rc == 0) {
        /* The descriptor's own profile, not a second opinion: this diagnostic must range the way
         * acquisition does, or the number it prints comes from a sensor configured differently
         * from the one the cliff path reads.
         *
         * And the result is checked. Discarding it was harmless while configure() was a no-op;
         * now that it really writes the distance mode and the timing budget, carrying on after a
         * failure would range under the PREVIOUS configuration and report the answer as if it
         * came from the new one. */
        out.configure_rc = ops.configure(dev, descs_[i].cliff_timing_budget_us,
                                         descs_[i].cliff_distance_mode, &out.status);
        if (out.configure_rc == 0)
            out.start_rc = ops.start(dev, &out.status);
        /* configure_rc as well as start_rc: start_rc is still 0 after a configure failure, because
         * start was never reached, so testing it alone would let the read loop run anyway -- which
         * is the original defect with one more step in front of it. */
        if (out.configure_rc == 0 && out.start_rc == 0) {
            /* One start, then re-check. Restarting between checks -- which is what looping over the
             * whole open/start/stop sequence does -- puts the sensor back at "just started" every
             * time and can never observe a first frame. */
            for (unsigned n{0}; n < attempts; ++n) {
                out.attempts_used = n + 1;
                out.read_rc =
                    ops.read_cliff_sample(dev, descs_[i].scratch, &out.sample, &out.status);
                if (out.read_rc != 0 || out.sample.fresh)
                    break;
                if (gap_ms != 0 && n + 1 < attempts)
                    k_msleep(gap_ms);
            }
        }
        acq::op_status ignored{};
        (void)ops.stop(dev, &ignored);
    }
    k_mutex_unlock(&tof_chain_controller::chain_lock());
    return 0;
}

#if defined(ENABLE_TOF_CLIFF_BENCH_PACK)
int stream_position(size_t position_1based, stream_result &out, unsigned want_frames,
                    unsigned gap_ms, unsigned max_attempts,
                    lexxhard::tof_cliff_stream::frame_sink sink, void *ctx)
{
    out = stream_result{};
    /* Same preconditions as probe_position, deliberately: this is that lifecycle with the break
     * removed, not a second route to the device. */
    if (want_frames == 0 || max_attempts == 0 || max_attempts > kMaxProbeAttempts ||
        gap_ms > kMaxProbeGapMs || max_attempts < want_frames)
        return -EINVAL;
    if (!ready())
        return -EPERM;
    if (acq::thread_running())
        return -EBUSY;
    if (position_1based == 0 || position_1based > spec_.positions)
        return -EINVAL;

    const size_t i{position_1based - 1};
    if (descs_[i].kind != acq::model::l4_cliff || descs_[i].ops == nullptr)
        return -ENOTSUP;

    out.addr_7bit = descs_[i].addr_7bit;
    out.role_id = descs_[i].role_id;

    const acq::source_ops &ops{*descs_[i].ops};
    void *dev{descs_[i].dev};

    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    out.attempted = true;
    out.open_rc = ops.open(dev, descs_[i].addr_7bit, &out.status);
    if (out.open_rc == 0) {
        /* The descriptor's own profile, not a second opinion: this diagnostic must range the way
         * acquisition does, or the number it prints comes from a sensor configured differently
         * from the one the cliff path reads.
         *
         * And the result is checked. Discarding it was harmless while configure() was a no-op;
         * now that it really writes the distance mode and the timing budget, carrying on after a
         * failure would range under the PREVIOUS configuration and report the answer as if it
         * came from the new one. */
        out.configure_rc = ops.configure(dev, descs_[i].cliff_timing_budget_us,
                                         descs_[i].cliff_distance_mode, &out.status);
        if (out.configure_rc == 0)
            out.start_rc = ops.start(dev, &out.status);
        /* configure_rc as well as start_rc: start_rc is still 0 after a configure failure, because
         * start was never reached, so testing it alone would let the read loop run anyway -- which
         * is the original defect with one more step in front of it. */
        if (out.configure_rc == 0 && out.start_rc == 0) {
            const lexxhard::tof_cliff_stream::params p{want_frames, gap_ms, max_attempts};
            const auto pr{lexxhard::tof_cliff_stream::run_loop(
                ops, dev, descs_[i].scratch, out.status, p, sink, ctx,
                [](unsigned ms) { k_msleep(ms); })};
            out.frames_collected = pr.frames_collected;
            out.attempts_used = pr.attempts_used;
            out.last_read_rc = pr.last_read_rc;
        }
        acq::op_status ignored{};
        (void)ops.stop(dev, &ignored);
    }
    k_mutex_unlock(&tof_chain_controller::chain_lock());
    return 0;
}
#endif

#ifdef CONFIG_ZTEST
void set_thread_stack_for_test(k_thread_stack_t *stack, size_t size)
{
    stack_ = stack;
    stack_size_ = size;
}

void reset_for_test()
{
    acq::teardown();
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

int install_from_mapping_for_test(const pf::fingerprint &fp, uint8_t epoch)
{
    return install_from_mapping(fp, epoch);
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
