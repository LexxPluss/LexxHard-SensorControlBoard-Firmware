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

#include "tof_package_grid_codec.hpp"

namespace lexxhard::tof_package_grid {

result pack(const grid &in, uint8_t *payload, size_t payload_len)
{
    if (payload == nullptr || payload_len != kPayloadBytes)
        return result::payload_length_mismatch;

    /* Byte-aligned, so no bit cursor and no shift arithmetic beyond splitting one 16-bit value.
     * Three bytes per zone divides the payload exactly: there is no padding anywhere in this
     * layout, and therefore no reserved field to check on the way back in. */
    for (size_t z{0}; z < kZones; ++z) {
        uint8_t *const at{payload + z * kBytesPerZone};
        at[0] = static_cast<uint8_t>(in.distance_mm[z] >> 8);
        at[1] = static_cast<uint8_t>(in.distance_mm[z] & 0xFFU);
        at[2] = in.status[z];
    }
    return result::ok;
}

result unpack(const uint8_t *payload, size_t payload_len, grid &out)
{
    if (payload == nullptr || payload_len != kPayloadBytes)
        return result::payload_length_mismatch;

    /* Decoded into a local and copied out only on success. The length check above is the only way
     * to fail, so this is defence rather than necessity -- but a caller that ignores the result
     * must not find a half-written grid either way. */
    grid decoded{};
    for (size_t z{0}; z < kZones; ++z) {
        const uint8_t *const at{payload + z * kBytesPerZone};
        decoded.distance_mm[z] = static_cast<uint16_t>(static_cast<uint16_t>(at[0]) << 8 | at[1]);
        decoded.status[z] = at[2];
    }

    out = decoded;
    return result::ok;
}

const char *result_name(result r)
{
    switch (r) {
    case result::ok:                      return "ok";
    case result::payload_length_mismatch: return "payload_length_mismatch";
    }
    return "unknown";
}

}
