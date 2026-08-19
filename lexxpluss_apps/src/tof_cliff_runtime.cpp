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
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "tof_acquisition.hpp"
#include "tof_chain_spec.hpp"
#include "tof_cliff_can.hpp"
#include "tof_cliff_publisher.hpp"
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

stage stage_{stage::not_started};
bool mapping_applied_{false};

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
void build_descriptors()
{
    int cliff_index{0};

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
            ++cliff_index;
        } else {
            /* The grid path, and the explicit stub rather than a null table or a copy of the cliff
             * ops: wiring L7 to the L4 driver has to be a deliberate act, not an oversight. */
            d.ops = &acq::l7_stub_ops();
        }
    }
}

int init_authority()
{
    au::config cfg{};

    cfg.runtime_spec = &spec_;
    cfg.begin_epoch = acq::begin_epoch;
    cfg.acquisition_idle = acq::is_idle;
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

int init_acquisition(const config &cfg)
{
    acq::config c{};

    c.sources = descs_;
    c.source_count = static_cast<int>(spec_.positions);
    c.periods.cycle_period_ms = cfg.cycle_period_ms;
    c.periods.health_period_ms = cfg.health_period_ms;
    c.hooks.on_cycle_begin = pub::on_cycle_begin;
    c.hooks.on_cycle = pub::on_cycle_complete;
    c.hooks.on_cliff_sample = pub::on_cliff_sample;
    c.hooks.on_cliff_health = pub::on_cliff_health;
    c.mapping_state_provider = au::state_provider;
    c.now_ms = now_ms;
    return acq::init(c);
}

}  // namespace

#if DT_NODE_EXISTS(DT_PATH(tof_chain))
config config_from_devicetree()
{
    /* Required properties, so a build that has not stated them does not compile. That is the whole
     * mechanism: the numbers are provisional either way, but they are provisional IN THE OVERLAY,
     * where they are visible in a diff and belong to whoever owns the deployment -- rather than
     * provisional in a scheduler, where the first plausible value silently becomes the spec. */
    return config{DT_PROP(DT_PATH(tof_chain), cycle_period_ms),
                  DT_PROP(DT_PATH(tof_chain), health_period_ms)};
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

    if (cfg.cycle_period_ms == 0 || cfg.health_period_ms == 0)
        return -EINVAL;

    build_descriptors();

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

    if (const int rc{init_acquisition(cfg)}; rc != 0) {
        stage_ = stage::acquisition_failed;
        LOG_ERR("acquisition init failed (%d)", rc);
        return rc;
    }

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

int apply_installed_mapping()
{
    if (!ready())
        return -EPERM;
    /* PROVEN is the only state that has an installed mapping to read. Asking the authority rather
     * than trusting a caller's word about what just happened is the same rule the commit follows. */
    if (au::current().state != acq::mapping_state::proven)
        return -EPERM;

    const pf::fingerprint &fp{au::installed_mapping()};

    if (fp.positions != spec_.positions)
        return -EINVAL;

    /* Two checks per cliff position, and both matter.
     *
     * The address, because role_id is about to become the key a measurement is published under: if
     * the mapping says front_left lives at 0x2C and the descriptor acquisition reads is 0x2D, then
     * attaching front_left's source id to that descriptor would publish one corner's distance under
     * another corner's name -- which is precisely the failure this subsystem exists to prevent, and
     * it would be invisible on the wire.
     *
     * The role, because a mapping with an unknown role has no source id, and the contract has no
     * way to express a measurement without one. */
    for (size_t i{0}; i < spec_.positions; ++i) {
        if (descs_[i].kind != acq::model::l4_cliff)
            continue;
        if (fp.at[i].address != descs_[i].addr_7bit)
            return -EINVAL;

        const int8_t src{pf::source_id_of(fp.at[i].role)};

        if (src < 0)
            return -EINVAL;
        descs_[i].role_id = static_cast<uint8_t>(src);
    }

    mapping_applied_ = true;
    LOG_INF("descriptors keyed from the installed mapping");
    return 0;
}

bool mapping_applied()
{
    return mapping_applied_;
}

int start_acquisition()
{
    if (!ready() || !mapping_applied_)
        return -EPERM;
    /* Sensors only. The acquisition thread lands in the next commit and belongs behind this same
     * gate: a thread that starts before the descriptors are keyed would publish cycles whose facts
     * carry kRoleUnassigned. */
    return acq::bring_up();
}

#ifdef CONFIG_ZTEST
void reset_for_test()
{
    acq::teardown();
    stage_ = stage::not_started;
    mapping_applied_ = false;
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
    build_descriptors();
    mapping_applied_ = false;
    return 0;
}
#endif

}  // namespace lexxhard::tof_cliff_runtime

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
