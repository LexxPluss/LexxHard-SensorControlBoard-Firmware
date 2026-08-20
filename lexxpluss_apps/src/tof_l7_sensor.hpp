/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * One VL53L7CX sensor. This layer owns only the ULD lifecycle and a
 * non-blocking complete-grid read. It does not assign source ids, generations
 * or CAN policy; the scheduler owns those facts.
 */

#pragma once

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_L7_ULD)

#include <stdint.h>

extern "C" {
#include "vl53l7cx_api.h"
}

#include "tof_l7_sample.hpp"

namespace lexxhard::tof_l7 {

static_assert(kZoneCount == VL53L7CX_RESOLUTION_8X8,
              "tof_l7_sample.hpp and the ULD resolution have drifted");
static_assert(VL53L7CX_NB_TARGET_PER_ZONE == 1U,
              "the L7 sample and grid contract require one target per zone");

enum class lifecycle : uint8_t {
  empty,
  opened,
  configured,
  running,
};

enum class stage : uint8_t {
  none,
  arguments,
  state,
  address,
  firmware,
  initialise,
  resolution,
  frequency,
  start,
  stop,
  ready_check,
  fetch,
  copy,
};

const char *stage_name(stage value);

struct operation_status {
  stage failed_stage{stage::none};
  int port_errno{0};
  uint8_t uld_status{VL53L7CX_STATUS_OK};
  bool sample_present{false};
};

struct sensor {
  VL53L7CX_Configuration uld{};
  lifecycle current{lifecycle::empty};
};

struct scratch {
  VL53L7CX_ResultsData results{};
};

/* CONCURRENCY: none. Every operation belongs to the one acquisition thread
 * while it owns the chain. The port's first-error record is process-global
 * because ST's callback carries no device context; a second caller could clear
 * another operation's transport error even if it used a different sensor and
 * scratch buffer. */

/* Initialise one already-addressed device. The 7-bit address is converted
 * exactly once for ST's 8-bit field. The firmware pointer has no
 * caller-supplied path: it is assigned only from
 * tof_l7_runtime::firmware_data(), after that runtime verified storage against
 * the signed-image accept-list. */
int open(sensor *device, uint8_t address_7bit, operation_status *status);

/* The grid contract fixes 8x8. Frequency remains an explicit input because
 * scheduling and the sustainable rate have not been accepted yet; no production
 * default is hidden here. */
int configure(sensor *device, uint8_t frequency_hz, operation_status *status);
int start(sensor *device, operation_status *status);
int stop(sensor *device, operation_status *status);

/* One non-blocking readiness check and, only when ready, one complete ULD
 * result fetch. No polling loop and no partial publication. `sample` is cleared
 * on entry and stays non-fresh on every refusal. */
int read_once(sensor *device, scratch *work, sample *out,
              operation_status *status);

/* Pure copy/validation boundary, exposed so impossible ULD metadata cannot
 * bypass the same checks in a unit test or future caller. The ULD is built for
 * one target per zone and post-conversion non-negative distances. A count
 * outside 0/1, a count/status contradiction, or a negative post-conversion
 * distance rejects the whole grid and leaves `out` zeroed. */
int copy_raw(const VL53L7CX_ResultsData *input, sample *out);

} // namespace lexxhard::tof_l7

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_L7_ULD
