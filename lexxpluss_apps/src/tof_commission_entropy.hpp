/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The session token's entropy, bound to the STM32 hardware RNG and to nothing else.
 *
 * WHAT THE TOKEN IS FOR, because it decides what is acceptable here. It tells a request addressed to
 * THIS boot from one a host is still retransmitting into a boot that is over. A token an attacker or
 * an accident could predict would let a stale frame be accepted as current, and two boards -- or two
 * boots of one board -- that draw the same token make the distinction meaningless. So the source has
 * to be a hardware entropy peripheral: a timer-seeded PRNG would produce adjacent boots that are
 * correlated by construction, which is exactly the case the token exists to separate.
 *
 * THERE IS NO FALLBACK, AND THAT IS ENFORCED AT COMPILE TIME. This file binds to the `st,stm32-rng`
 * node specifically rather than to "whatever entropy device the build happens to have", so a
 * configuration that selected Zephyr's timer, xoshiro or test generator does not build. An image
 * that quietly fell back would satisfy every API and none of the requirements.
 *
 * FAILURE IS NO SESSION, NEVER A WEAKER TOKEN. A device that is not ready and a read that fails both
 * end the same way: `tof_commission::init()` refuses, the board announces no session, and every
 * request is answered `no_session`. A board that cannot tell this boot from the last one has no
 * business acting on a request that claims to know.
 *
 * A ZERO IS NOT ONE OF THOSE. Zero is reserved as "no token", but one zero from a healthy generator
 * is an ordinary sample -- once in 2^32 -- so the session layer draws AGAIN and refuses only on a
 * second zero. Treating a single zero as a failure would leave roughly one board in four billion
 * with no session for no reason; treating two as anything but a failure would accept a generator
 * that is not generating.
 *
 * DEFAULT OFF. Compiled only into the automatic-commissioning build (ENABLE_TOF_AUTO_COMMISSION),
 * whose overlay is the only thing that enables the peripheral. No other image changes.
 */

#pragma once

#include <stdint.h>

#if defined(ENABLE_TOF_AUTO_COMMISSION)

namespace lexxhard::tof_commission_entropy {

/* Matches tof_commission::hooks::draw_token. Returns 0 and a value -- which MAY be zero, because
 * zero is an ordinary sample from a healthy generator and the session layer is what treats it as
 * "no token" and draws again -- or negative when the hardware could not be asked at all. */
int draw_token(void *ctx, uint32_t *out);

/* Whether the peripheral is present and ready. Reported separately so a diagnostic can say "no
 * entropy device" rather than only "no session". */
bool available();

} // namespace lexxhard::tof_commission_entropy

#endif // ENABLE_TOF_AUTO_COMMISSION
