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
 * A BOOT-LOCAL sequence source. It counts in RAM from one, and every reboot starts again at one.
 *
 * SO IT REPEATS NUMBERS ACROSS A RESTART, which is the single thing candidate N's identity rests on
 * not happening. This is therefore NOT an implementation of the requirement's 32-bit acquisition
 * sequence, and a stream driven by it must not be read as evidence that N works. It exists for one
 * purpose: to make an end-to-end observation possible on a board that has no commissioned sequence
 * journal, so that the CAN path, the frame layout and the consumer can be exercised while the
 * persistent issuer's storage question is still open (tof_seq_issuer, and the field storage
 * acceptance item it names).
 *
 * IT IS NOT A FALLBACK, and nothing may make it one. tof_seq_issuer refuses to issue when its
 * journal is absent, corrupt or incompatible -- that refusal is the whole protection, and a source
 * that silently stepped in behind it would convert "this board cannot number acquisitions" into
 * "this board numbers them from one again", which is exactly the repeat the refusal exists to
 * prevent. The two sources are therefore separate types with no common interface and no selection
 * logic between them: choosing this one is an act a human performs, by name, at a shell.
 *
 * WHAT IT DOES GUARANTEE, within one boot:
 *
 * - it never issues tof_package_wire_n's reserved sequence, because it starts at one. A zero would
 *   be indistinguishable from an independent diagnostic's placeholder, so the layout refuses it end
 *   to end and a source that produced one would simply stall the stream;
 * - it REFUSES rather than wrapping when the 32-bit space is spent. Wrapping would reissue number
 *   one to a different acquisition inside a single boot, and that is the one repeat that cannot be
 *   excused by "it is boot-local": a consumer watching one continuous session would see the same
 *   number twice with no restart to explain it.
 *
 * Pure: no Zephyr, no clock, no storage. Deliberately holds nothing that could survive a reset,
 * because anything that did would start to look like persistence.
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
