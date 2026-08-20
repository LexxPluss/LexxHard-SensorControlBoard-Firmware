/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace lexxhard::tof_l7 {

/* Vendor-free output of one complete 8x8 read. Keeping this type separate from
 * the ULD lets the future scheduler and packer tests consume a grid without
 * importing ST's headers or object layout. It deliberately carries no source,
 * generation, model or recovery claims: those belong to the chain/scheduler
 * layer, not to a sensor read. */
inline constexpr size_t kZoneCount{64};

struct sample {
  bool fresh{false};
  int8_t silicon_temperature_degc{0};
  uint8_t target_count[kZoneCount]{};
  uint16_t distance_mm[kZoneCount]{};
  uint8_t target_status[kZoneCount]{};
};

} // namespace lexxhard::tof_l7
