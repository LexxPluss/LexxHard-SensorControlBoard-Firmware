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

/* RAM-only sequence source for experiments without a commissioned journal.
 * Starts at 1 each boot, never issues reserved zero, and refuses exhaustion
 * instead of wrapping. Keep one instance per boot, not per stream command.
 * Numbers repeat across resets: this does not meet candidate N's persistent
 * identity requirement. It is a separate type, never a fallback for a failed
 * persistent issuer. No storage, clock or Zephyr dependency.
 */

#include <stdint.h>

namespace lexxhard::tof_seq_boot_local {

/* The first number, matching tof_seq_issuer::kFirstNumber for the same reason -- zero is the wire's
 * reserved value -- so that a consumer cannot tell the two sources apart by their first number.
 * That is a statement about what this source CANNOT be used to prove, not a compatibility feature.
 */
inline constexpr uint32_t kFirstNumber{1};

class source {
public:
    source() = default;

    /* Starts the count somewhere other than kFirstNumber. IT EXISTS SO EXHAUSTION IS REACHABLE IN A
     * TEST -- the alternative is 2^32 calls -- and it is a seam rather than a backdoor: it can only
     * move the start forward, every number it hands out is still issued once, and a `first` of zero
     * is refused by constructing a source that is already spent, so the wire's reserved value
     * cannot be reached through it. */
    explicit source(uint32_t first)
        : next_{first}, spent_{first == 0}
    {
    }

    /* 0 with *out written, or -ENOSPC once the space is spent, with *out untouched. */
    int issue(uint32_t *out);

    /* How many numbers this boot has handed out. */
    uint32_t issued() const { return issued_; }

    /* True once issue() can no longer succeed. */
    bool spent() const { return spent_; }

private:
    uint32_t next_{kFirstNumber};
    uint32_t issued_{0};
    bool spent_{false};
};

}
