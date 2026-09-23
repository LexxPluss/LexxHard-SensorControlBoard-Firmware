/*
 * Copyright (c) 2026, LexxPluss Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * TEST SHIM, host builds only: the ST LL I2C layer, over the register file in soc.h.
 *
 * THE RULE THIS FILE FOLLOWS. It models what the hardware does when a register is read or written,
 * and nothing more. Reading RXDR takes the byte and clears RXNE, because that is what the
 * peripheral does. Writing TXDR clears TXIS. Writing ICR clears the matching status bits.
 * GenerateStop sets the STOP trigger and is left for the test to answer with STOPF, because on
 * real hardware that answer comes from the bus, not from the register write.
 *
 * What it does NOT contain is any part of the driver's decision making: no idea of segments,
 * reloads, messages or completion. That lives in the driver under test, once.
 */

#ifndef LEXX_TEST_SHIM_STM32_LL_I2C_H_
#define LEXX_TEST_SHIM_STM32_LL_I2C_H_

#include <soc.h>

#define LL_I2C_ADDRESSING_MODE_7BIT  0x00000000U
#define LL_I2C_ADDRESSING_MODE_10BIT I2C_CR2_ADD10
#define LL_I2C_REQUEST_WRITE         0x00000000U
#define LL_I2C_REQUEST_READ          I2C_CR2_RD_WRN
#define LL_I2C_DIRECTION_WRITE       0x00000000U
#define LL_I2C_ACK                   0x00000000U
#define LL_I2C_NACK                  I2C_CR2_NACK
#define LL_I2C_OWNADDRESS1_7BIT      0x00000000U
#define LL_I2C_OWNADDRESS1_10BIT     0x00000001U
#define LL_I2C_OWNADDRESS2_NOMASK    0x00000000U

/* Reads of the status register are counted, because one of the things under test is that the
 * handler reads it again before returning -- the synchronising read that keeps a level-triggered
 * line from re-entering on a flag the peripheral has not finished clearing. Counting a register
 * access is a property of the bus, not a decision, so it belongs here. */
extern unsigned int lexx_test_isr_reads;
static inline uint32_t lexx_test_read_ISR(I2C_TypeDef *i2c)
{
	lexx_test_isr_reads++;
	return i2c->ISR;
}
#define LEXX_TEST_RD_ISR(inst)     lexx_test_read_ISR(inst)
#define LEXX_TEST_RD_CR1(inst)     ((inst)->CR1)
#define LEXX_TEST_RD_CR2(inst)     ((inst)->CR2)
#define LEXX_TEST_RD_TIMINGR(inst) ((inst)->TIMINGR)
#define LL_I2C_ReadReg(inst, reg)  LEXX_TEST_RD_##reg(inst)
/* Writing CR2 carries the peripheral's own acknowledgements, and they matter here: TCR is what
 * the controller raises to ask for the next length, and it goes away when the length is written.
 * TC likewise goes away when a start or a stop is requested. Without this the status register
 * would keep asking for something the driver has already answered. */
static inline void lexx_test_write_CR2(I2C_TypeDef *i2c, uint32_t val)
{
	i2c->CR2 = val;
	if (((val & I2C_CR2_NBYTES_Msk) >> I2C_CR2_NBYTES_Pos) != 0U) {
		i2c->ISR &= ~I2C_ISR_TCR;
	}
	if ((val & (I2C_CR2_START | I2C_CR2_STOP)) != 0U) {
		i2c->ISR &= ~I2C_ISR_TC;
	}
}
#define LEXX_TEST_WR_CR1(inst, val)     ((inst)->CR1 = (val))
#define LEXX_TEST_WR_CR2(inst, val)     lexx_test_write_CR2((inst), (val))
#define LEXX_TEST_WR_TIMINGR(inst, val) ((inst)->TIMINGR = (val))
#define LL_I2C_WriteReg(inst, reg, val) LEXX_TEST_WR_##reg((inst), (val))
#define LL_I2C_CONVERT_TIMINGS(p, l, h, d, s) \
	(((p) << 28) | ((d) << 20) | ((s) << 16) | ((h) << 8) | (l))

static inline void LL_I2C_Enable(I2C_TypeDef *i2c) { i2c->CR1 |= I2C_CR1_PE; }
static inline void LL_I2C_Disable(I2C_TypeDef *i2c) { i2c->CR1 &= ~I2C_CR1_PE; }

static inline void LL_I2C_EnableIT_TX(I2C_TypeDef *i2c) { i2c->CR1 |= I2C_CR1_TXIE; }
static inline void LL_I2C_DisableIT_TX(I2C_TypeDef *i2c) { i2c->CR1 &= ~I2C_CR1_TXIE; }
static inline void LL_I2C_EnableIT_RX(I2C_TypeDef *i2c) { i2c->CR1 |= I2C_CR1_RXIE; }
static inline void LL_I2C_DisableIT_RX(I2C_TypeDef *i2c) { i2c->CR1 &= ~I2C_CR1_RXIE; }
static inline void LL_I2C_EnableIT_ADDR(I2C_TypeDef *i2c) { i2c->CR1 |= I2C_CR1_ADDRIE; }
static inline void LL_I2C_DisableIT_ADDR(I2C_TypeDef *i2c) { i2c->CR1 &= ~I2C_CR1_ADDRIE; }
static inline void LL_I2C_EnableIT_NACK(I2C_TypeDef *i2c) { i2c->CR1 |= I2C_CR1_NACKIE; }
static inline void LL_I2C_DisableIT_NACK(I2C_TypeDef *i2c) { i2c->CR1 &= ~I2C_CR1_NACKIE; }
static inline void LL_I2C_EnableIT_STOP(I2C_TypeDef *i2c) { i2c->CR1 |= I2C_CR1_STOPIE; }
static inline void LL_I2C_DisableIT_STOP(I2C_TypeDef *i2c) { i2c->CR1 &= ~I2C_CR1_STOPIE; }
static inline void LL_I2C_EnableIT_TC(I2C_TypeDef *i2c) { i2c->CR1 |= I2C_CR1_TCIE; }
static inline void LL_I2C_DisableIT_TC(I2C_TypeDef *i2c) { i2c->CR1 &= ~I2C_CR1_TCIE; }
static inline void LL_I2C_EnableIT_ERR(I2C_TypeDef *i2c) { i2c->CR1 |= I2C_CR1_ERRIE; }
static inline void LL_I2C_DisableIT_ERR(I2C_TypeDef *i2c) { i2c->CR1 &= ~I2C_CR1_ERRIE; }
static inline void LL_I2C_EnableWakeUpFromStop(I2C_TypeDef *i2c) { i2c->CR1 |= I2C_CR1_WUPEN; }
static inline void LL_I2C_DisableWakeUpFromStop(I2C_TypeDef *i2c) { i2c->CR1 &= ~I2C_CR1_WUPEN; }

static inline void LL_I2C_EnableReloadMode(I2C_TypeDef *i2c) { i2c->CR2 |= I2C_CR2_RELOAD; }
static inline void LL_I2C_DisableReloadMode(I2C_TypeDef *i2c) { i2c->CR2 &= ~I2C_CR2_RELOAD; }
static inline uint32_t LL_I2C_IsEnabledReloadMode(I2C_TypeDef *i2c)
{
	return (i2c->CR2 & I2C_CR2_RELOAD) != 0U;
}
static inline void LL_I2C_DisableAutoEndMode(I2C_TypeDef *i2c) { i2c->CR2 &= ~I2C_CR2_AUTOEND; }

static inline void LL_I2C_SetTransferSize(I2C_TypeDef *i2c, uint32_t n)
{
	lexx_test_write_CR2(i2c, (i2c->CR2 & ~I2C_CR2_NBYTES_Msk) | (n << I2C_CR2_NBYTES_Pos));
}
static inline void LL_I2C_SetTransferRequest(I2C_TypeDef *i2c, uint32_t req)
{
	i2c->CR2 = (i2c->CR2 & ~I2C_CR2_RD_WRN) | req;
}
static inline void LL_I2C_SetMasterAddressingMode(I2C_TypeDef *i2c, uint32_t mode)
{
	i2c->CR2 = (i2c->CR2 & ~I2C_CR2_ADD10) | mode;
}
static inline void LL_I2C_SetSlaveAddr(I2C_TypeDef *i2c, uint32_t addr)
{
	i2c->CR2 = (i2c->CR2 & ~I2C_CR2_SADD_Msk) | (addr & I2C_CR2_SADD_Msk);
}
static inline void LL_I2C_GenerateStartCondition(I2C_TypeDef *i2c)
{
	lexx_test_write_CR2(i2c, i2c->CR2 | I2C_CR2_START);
}
static inline void LL_I2C_GenerateStopCondition(I2C_TypeDef *i2c)
{
	lexx_test_write_CR2(i2c, i2c->CR2 | I2C_CR2_STOP);
}
static inline void LL_I2C_AcknowledgeNextData(I2C_TypeDef *i2c, uint32_t ack)
{
	i2c->CR2 = (i2c->CR2 & ~I2C_CR2_NACK) | ack;
}
static inline void LL_I2C_SetTiming(I2C_TypeDef *i2c, uint32_t t) { i2c->TIMINGR = t; }

/* Data registers carry the peripheral's own side effects on the status flags. */
static inline uint32_t LL_I2C_ReceiveData8(I2C_TypeDef *i2c)
{
	i2c->ISR &= ~I2C_ISR_RXNE;
	return i2c->RXDR & 0xFFU;
}
static inline void LL_I2C_TransmitData8(I2C_TypeDef *i2c, uint8_t b)
{
	i2c->TXDR = b;
	i2c->ISR &= ~I2C_ISR_TXIS;
}

#define LL_I2C_FLAG(name, bit)                                                    \
	static inline uint32_t LL_I2C_IsActiveFlag_##name(I2C_TypeDef *i2c)       \
	{                                                                         \
		return (i2c->ISR & (bit)) != 0U;                                  \
	}                                                                         \
	static inline void LL_I2C_ClearFlag_##name(I2C_TypeDef *i2c)              \
	{                                                                         \
		i2c->ICR = (bit);                                                 \
		i2c->ISR &= ~(bit);                                               \
	}

LL_I2C_FLAG(ADDR, I2C_ISR_ADDR)
LL_I2C_FLAG(NACK, I2C_ISR_NACKF)
LL_I2C_FLAG(STOP, I2C_ISR_STOPF)
LL_I2C_FLAG(BERR, I2C_ISR_BERR)
LL_I2C_FLAG(ARLO, I2C_ISR_ARLO)
LL_I2C_FLAG(OVR, I2C_ISR_OVR)

static inline uint32_t LL_I2C_IsActiveFlag_RXNE(I2C_TypeDef *i2c)
{
	return (i2c->ISR & I2C_ISR_RXNE) != 0U;
}
static inline uint32_t LL_I2C_IsActiveFlag_TXIS(I2C_TypeDef *i2c)
{
	return (i2c->ISR & I2C_ISR_TXIS) != 0U;
}
static inline uint32_t LL_I2C_IsActiveFlag_TC(I2C_TypeDef *i2c)
{
	return (i2c->ISR & I2C_ISR_TC) != 0U;
}
static inline uint32_t LL_I2C_IsActiveFlag_TCR(I2C_TypeDef *i2c)
{
	return (i2c->ISR & I2C_ISR_TCR) != 0U;
}
/* TXE is set by writing 1 to it, which is what "flush the transmit register" means here. */
static inline void LL_I2C_ClearFlag_TXE(I2C_TypeDef *i2c) { i2c->ISR |= I2C_ISR_TXE; }

static inline uint32_t LL_I2C_IsActiveSMBusFlag_ALERT(I2C_TypeDef *i2c)
{
	return (i2c->ISR & I2C_ISR_ALERT) != 0U;
}
static inline void LL_I2C_ClearSMBusFlag_ALERT(I2C_TypeDef *i2c)
{
	i2c->ICR = I2C_ICR_ALERTCF;
	i2c->ISR &= ~I2C_ISR_ALERT;
}

static inline uint32_t LL_I2C_GetTransferDirection(I2C_TypeDef *i2c)
{
	return (i2c->ISR & I2C_ISR_DIR) != 0U;
}
static inline uint32_t LL_I2C_GetAddressMatchCode(I2C_TypeDef *i2c)
{
	return (i2c->ISR >> 17) & 0x7FU;
}
static inline void LL_I2C_SetOwnAddress1(I2C_TypeDef *i2c, uint32_t a, uint32_t m)
{
	i2c->OAR1 = a | m;
}
static inline void LL_I2C_EnableOwnAddress1(I2C_TypeDef *i2c) { i2c->OAR1 |= (1UL << 15); }
static inline void LL_I2C_DisableOwnAddress1(I2C_TypeDef *i2c) { i2c->OAR1 &= ~(1UL << 15); }
static inline void LL_I2C_SetOwnAddress2(I2C_TypeDef *i2c, uint32_t a, uint32_t m)
{
	i2c->OAR2 = a | m;
}
static inline void LL_I2C_EnableOwnAddress2(I2C_TypeDef *i2c) { i2c->OAR2 |= (1UL << 15); }
static inline void LL_I2C_DisableOwnAddress2(I2C_TypeDef *i2c) { i2c->OAR2 &= ~(1UL << 15); }

#endif /* LEXX_TEST_SHIM_STM32_LL_I2C_H_ */
