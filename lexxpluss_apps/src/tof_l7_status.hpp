/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
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

struct operation_status {
  stage failed_stage{stage::none};
  int port_errno{0};
  uint8_t uld_status{0};
  bool sample_present{false};
};

} // namespace lexxhard::tof_l7
