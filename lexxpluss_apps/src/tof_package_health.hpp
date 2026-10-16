/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

/*
 * Sensor health for the Package grid: the three states FW-3 requires, decided by a rule simple
 * enough to explain.
 *
 * STATELESS ON PURPOSE. This is a pure function over one acquisition, with no object to hold a
 * previous verdict, because the rule is that each complete grid is judged by its OWN content. No
 * hysteresis, no debounce, no counters -- not because those would be wrong, but because the numbers
 * they need have not been measured, and a module with nowhere to keep state cannot acquire an
 * unmeasured one by accident. When bench data justifies hysteresis it will be a deliberate change
 * here, visible in the type.
 *
 * THE DECISION TABLE, which is the whole module:
 *
 *   a complete, structurally valid grid
 *     every zone status 5, or 255, or any mixture of the two  -> OK
 *     any zone carrying any other value, known or unknown     -> DEGRADED, grid still published
 *   acquisition failed, or what arrived is not a valid grid    -> ERROR, no grid published
 *   the sensor was never started                               -> ERROR, no grid published,
 *                                                                 and availability says INACTIVE
 *
 * WHY 255 IS NOT A FAULT. ST's `target_status = 255` means the sensor returned no target (UM3038).
 * That is a measurement outcome, not a malfunction: a clear field of view produces it on every
 * zone. Treating it as degradation would report a fault for the most ordinary reading there is.
 *
 * WHY EVERY OTHER VALUE IS DEGRADED, INCLUDING 6 AND 9. The frozen chain contract has an
 * `accept_low_confidence` option that admits 6 and 9 as usable distances; this rule does not, and
 * the divergence is deliberate rather than overlooked. DEGRADED is not a refusal -- the grid is
 * still published and the raw status travels untouched -- so the consumer keeps every option the
 * chain's policy would have given it, while the health byte stops claiming that a low-confidence
 * grid is as good as a trusted one. Unknown values are degraded for the same reason: a status this
 * firmware does not recognise is not a status it may call normal.
 *
 * RAW STATUS IS NEVER REWRITTEN. This module reads the status bytes and returns a verdict. It does
 * not touch them, and nothing downstream may collapse them on the strength of this verdict --
 * per ST, 0 is "data not updated" and 255 is "no target detected", two different things that the
 * consumer needs to tell apart.
 *
 * INACTIVE DOES NOT EXTEND THE THREE-STATE ENUM. The requirement defines exactly three values and
 * this file does not add a fourth. The one interpretive step is recorded here: an inactive sensor
 * reports `sensor_state::error`, because "no data" is what the requirement's third state means and
 * there is no other value available -- and `availability::inactive` carries the distinction as an
 * INDEPENDENT diagnostic. If the owner wants an inactive sensor to report something other than
 * ERROR, that is a change to this decision, not a reinterpretation of it.
 *
 * WHAT OK DOES NOT MEAN. It means the acquisition was normal under this rule. It does NOT mean the
 * shutter may close. A grid of all 255 -- the sensor returned no target anywhere -- reads OK, and so
 * does a grid of all 5; this verdict cannot tell those apart, and the difference is exactly what a
 * closing decision turns on. FW-5's interlock keeps its own judgement for that reason, and it is
 * not this rule.
 */

#include <stddef.h>
#include <stdint.h>

namespace lexxhard::tof_package_health {

inline constexpr size_t kZones{64};
/* The only status this rule calls normal. */
inline constexpr uint8_t kStatusValid{5};
/* "No target detected" (UM3038). A measurement outcome, not a fault. */
inline constexpr uint8_t kStatusNoTarget{255};

/* The requirement's enum, with the requirement's values. Not extended. */
enum class sensor_state : uint8_t {
    ok = 0,
    degraded = 1,
    error = 2,
};

/* The independent diagnostic. Separate from sensor_state so that expressing "never started" costs
 * no change to the enum the requirement fixed. */
enum class availability : uint8_t {
    active,
    inactive,
};

enum class acquisition : uint8_t {
    /* A whole grid is in hand and its structure is valid. */
    complete_grid,
    /* The sensor or the bus did not produce a grid. */
    failed,
    /* Something arrived and is not a valid grid: a wrong zone count, a missing buffer. */
    structurally_invalid,
    /* The sensor was never started -- no verified firmware, not bound, not provisioned. */
    never_started,
};

struct input {
    acquisition outcome{acquisition::failed};
    /* Required, and exactly kZones long, when outcome is complete_grid. Read, never written. */
    const uint8_t *zone_status{nullptr};
    size_t zone_count{0};
};

struct verdict {
    sensor_state state{sensor_state::error};
    availability avail{availability::active};
    /* False for ERROR and for an inactive sensor. A grid is never manufactured to accompany a
     * fault: there is no all-sentinel grid, and no timestamp is refreshed on one. */
    bool publish_grid{false};
    /* Diagnostics, not gates. They add up to kZones for a complete grid. */
    uint8_t normal_zones{0};
    uint8_t no_target_zones{0};
    uint8_t abnormal_zones{0};
};

verdict judge(const input &in);

const char *state_name(sensor_state s);
const char *availability_name(availability a);

}
