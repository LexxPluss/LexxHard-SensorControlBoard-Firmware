/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/*
 * The mapping state vocabulary, on its own so that using it does not mean depending on the
 * acquisition layer.
 *
 * It lived in tof_acquisition.hpp, which includes tof_cliff_sensor.h and therefore the
 * vendor ULD. The mapping authority needs the enum and nothing else, and pulling 31k lines
 * of vendor driver into a module that touches no sensor would also have meant pulling it
 * into that module's test image. Same reason tof_cliff_sample.h exists.
 *
 * Still in namespace tof_acq: the acquisition layer is where the state is consumed most, the
 * existing call sites read well, and moving the namespace would be churn with no reader
 * benefit. Deliberately NOT behind ENABLE_TOF_CHAIN -- an enumeration costs nothing and a
 * guard here would just make every includer repeat the condition.
 */

#include <stdint.h>

namespace lexxhard::tof_acq {

// Deliberately NOT the wire encoding: the contract numbers these differently and
// tof_cliff_publisher translates with an explicit switch. `lost` means "was proven, then the
// mapping was lost at runtime", which is a strictly worse state than never having been
// proven and must not be reported as UNKNOWN -- the consumer's recovery path differs.
enum class mapping_state : uint8_t {
    not_ready = 0,
    fault,
    proven,
    lost,
};

}  // namespace lexxhard::tof_acq
