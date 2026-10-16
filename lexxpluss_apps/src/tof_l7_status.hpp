/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>

namespace lexxhard::tof_l7 {

/* Vendor-free operation diagnostics. The acquisition scheduler needs to
 * preserve these facts without importing the ULD object layout. */
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

inline const char *stage_name(stage value) {
  switch (value) {
  case stage::none:
    return "none";
  case stage::arguments:
    return "arguments";
  case stage::state:
    return "state";
  case stage::address:
    return "address";
  case stage::firmware:
    return "firmware";
  case stage::initialise:
    return "initialise";
  case stage::resolution:
    return "resolution";
  case stage::frequency:
    return "frequency";
  case stage::start:
    return "start";
  case stage::stop:
    return "stop";
  case stage::ready_check:
    return "ready_check";
  case stage::fetch:
    return "fetch";
  case stage::copy:
    return "copy";
  }
  return "unknown";
}

/* THE ULD'S TIMEOUT STATUS, MIRRORED so that callers above this layer can tell a timeout from any
 * other failure without including the vendor header. tof_l7_sensor.cpp binds it to
 * VL53L7CX_STATUS_TIMEOUT_ERROR with a static_assert, so the mirror cannot drift silently.
 *
 * It is needed because `failed_stage` alone does not say what went wrong: the readiness check can
 * fail as a timeout, as a bus error or as impossible device metadata, and the wire contract has a
 * flag for exactly one of those. */
inline constexpr uint8_t kUldTimeoutStatus{1};

struct operation_status {
  stage failed_stage{stage::none};
  int port_errno{0};
  uint8_t uld_status{0};
  bool sample_present{false};
};

} // namespace lexxhard::tof_l7
