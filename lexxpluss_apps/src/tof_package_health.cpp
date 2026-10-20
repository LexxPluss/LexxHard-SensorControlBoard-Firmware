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

#include "tof_package_health.hpp"

namespace lexxhard::tof_package_health {

verdict judge(const input &in)
{
    verdict v{};

    switch (in.outcome) {
    case acquisition::never_started:
        /* ERROR because "no data" is what the requirement's third state means, and
         * availability carries the reason. No grid, and no enum value invented. */
        v.state = sensor_state::error;
        v.avail = availability::inactive;
        v.publish_grid = false;
        return v;

    case acquisition::failed:
    case acquisition::structurally_invalid:
        v.state = sensor_state::error;
        v.publish_grid = false;
        return v;

    case acquisition::complete_grid:
        break;
    }

    /* A grid that claims to be complete and is not is a structural fault, judged here rather than
     * trusted: the caller's word about its own buffer is the one input this module can check. */
    if (in.zone_status == nullptr || in.zone_count != kZones) {
        v.state = sensor_state::error;
        v.publish_grid = false;
        return v;
    }

    for (size_t z{0}; z < kZones; ++z) {
        const uint8_t s{in.zone_status[z]};
        if (s == kStatusValid)
            ++v.normal_zones;
        else if (s == kStatusNoTarget)
            ++v.no_target_zones;
        else
            ++v.abnormal_zones;
    }

    /* One abnormal zone is enough. A threshold would need a measured number, and there is none. */
    v.state = v.abnormal_zones == 0 ? sensor_state::ok : sensor_state::degraded;
    /* DEGRADED STILL PUBLISHES. The raw status travels untouched, so the consumer keeps every
     * decision this verdict does not make for it. */
    v.publish_grid = true;
    return v;
}

const char *state_name(sensor_state s)
{
    switch (s) {
    case sensor_state::ok:       return "OK";
    case sensor_state::degraded: return "DEGRADED";
    case sensor_state::error:    return "ERROR";
    }
    return "unknown";
}

const char *availability_name(availability a)
{
    switch (a) {
    case availability::active:   return "active";
    case availability::inactive: return "INACTIVE";
    }
    return "unknown";
}

}
