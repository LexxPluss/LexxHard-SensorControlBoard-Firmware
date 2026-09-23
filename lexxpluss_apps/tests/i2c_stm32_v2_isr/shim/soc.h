/*
 * Copyright (c) 2026, LexxPluss Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * TEST SHIM, host builds only. Stands in for the vendor soc.h so the PRODUCTION driver source can
 * be compiled and run on a host. It provides the peripheral's register file and its bit
 * definitions, and nothing else: no behaviour, no state machine. What the registers DO when the
 * driver touches them lives in stm32_ll_i2c.h next to this file, and what the peripheral does on
 * its own is driven explicitly by each test.
 */

#ifndef LEXX_TEST_SHIM_SOC_H_
#define LEXX_TEST_SHIM_SOC_H_

#include <stdint.h>

/* STM32 I2C v2 register file, in hardware order. */
typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t TIMINGR;
	volatile uint32_t TIMEOUTR;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint32_t PECR;
	volatile uint32_t RXDR;
	volatile uint32_t TXDR;
} I2C_TypeDef;

#define I2C_CR1_PE      (1UL << 0)
#define I2C_CR1_TXIE    (1UL << 1)
#define I2C_CR1_RXIE    (1UL << 2)
#define I2C_CR1_ADDRIE  (1UL << 3)
#define I2C_CR1_NACKIE  (1UL << 4)
#define I2C_CR1_STOPIE  (1UL << 5)
#define I2C_CR1_TCIE    (1UL << 6)
#define I2C_CR1_ERRIE   (1UL << 7)
#define I2C_CR1_WUPEN   (1UL << 18)

#define I2C_CR2_SADD_Pos   0U
#define I2C_CR2_SADD_Msk   (0x3FFUL << I2C_CR2_SADD_Pos)
#define I2C_CR2_RD_WRN     (1UL << 10)
#define I2C_CR2_ADD10      (1UL << 11)
#define I2C_CR2_START      (1UL << 13)
#define I2C_CR2_STOP       (1UL << 14)
#define I2C_CR2_NACK       (1UL << 15)
#define I2C_CR2_NBYTES_Pos 16U
#define I2C_CR2_NBYTES_Msk (0xFFUL << I2C_CR2_NBYTES_Pos)
#define I2C_CR2_RELOAD     (1UL << 24)
#define I2C_CR2_AUTOEND    (1UL << 25)
/* The _Msk spellings the driver uses for the trigger bits. */
#define I2C_CR2_START_Msk  I2C_CR2_START
#define I2C_CR2_STOP_Msk   I2C_CR2_STOP
#define I2C_CR2_NACK_Msk   I2C_CR2_NACK
#define I2C_CR2_RELOAD_Msk I2C_CR2_RELOAD
#define I2C_CR2_ADD10_Msk  I2C_CR2_ADD10

#define I2C_ISR_TXE     (1UL << 0)
#define I2C_ISR_TXIS    (1UL << 1)
#define I2C_ISR_RXNE    (1UL << 2)
#define I2C_ISR_ADDR    (1UL << 3)
#define I2C_ISR_NACKF   (1UL << 4)
#define I2C_ISR_STOPF   (1UL << 5)
#define I2C_ISR_TC      (1UL << 6)
#define I2C_ISR_TCR     (1UL << 7)
#define I2C_ISR_BERR    (1UL << 8)
#define I2C_ISR_ARLO    (1UL << 9)
#define I2C_ISR_OVR     (1UL << 10)
#define I2C_ISR_ALERT   (1UL << 13)
#define I2C_ISR_BUSY    (1UL << 15)
#define I2C_ISR_DIR     (1UL << 16)

#define I2C_ICR_ADDRCF  I2C_ISR_ADDR
#define I2C_ICR_NACKCF  I2C_ISR_NACKF
#define I2C_ICR_STOPCF  I2C_ISR_STOPF
#define I2C_ICR_BERRCF  I2C_ISR_BERR
#define I2C_ICR_ARLOCF  I2C_ISR_ARLO
#define I2C_ICR_OVRCF   I2C_ISR_OVR
#define I2C_ICR_ALERTCF I2C_ISR_ALERT

#endif /* LEXX_TEST_SHIM_SOC_H_ */
