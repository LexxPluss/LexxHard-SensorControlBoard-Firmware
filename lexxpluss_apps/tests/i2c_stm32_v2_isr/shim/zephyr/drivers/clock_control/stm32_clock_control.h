/*
 * Copyright (c) 2026, LexxPluss Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * TEST SHIM, host builds only. Only the type the driver's config struct needs.
 */

#ifndef LEXX_TEST_SHIM_STM32_CLOCK_CONTROL_H_
#define LEXX_TEST_SHIM_STM32_CLOCK_CONTROL_H_

#include <stdint.h>

struct stm32_pclken {
	uint32_t bus;
	uint32_t enr;
};

#endif /* LEXX_TEST_SHIM_STM32_CLOCK_CONTROL_H_ */
