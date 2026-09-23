/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The STM32 I2C v2 interrupt handler, on the host.
 *
 * WHY THESE EXIST. Two long runs on DS20001 ended with the board wedged at interrupt level: RXNE
 * asserted, the driver's byte count already 0, RXIE still enabled and no completion flag to act
 * on, the handler re-entered about 890,000 times a second until the watchdog reset the board. The
 * second run caught the entry that starts it -- one interrupt carrying only TCR, counted by the
 * old handler as a byte received. From then on software is one byte ahead of the hardware.
 *
 * WHAT IS UNDER TEST is the production driver source, compiled as it ships, with the production
 * struct i2c_stm32_data. shim/ supplies the register file and the LL side effects only. Each test
 * plays the peripheral: it asserts on what the driver programmed, then raises the flags the
 * hardware would raise, and calls the real handler. There is no second copy of the state machine
 * anywhere in this file, which is the point -- a test that reimplements the thing it checks would
 * have agreed with the old driver too.
 */

#include <zephyr/ztest.h>
#ifdef CONFIG_ZTEST_ASSERT_HOOK
#include <zephyr/ztest_error_hook.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#include <soc.h>
#include "i2c_ll_stm32.h"

/* The device under test: production structures, a register file instead of a peripheral. */
static I2C_TypeDef regs;
static struct i2c_stm32_data drv_data;
static const struct i2c_stm32_config drv_cfg = {
	.i2c = &regs,
	.bitrate = 400000,
};
static struct device_state dev_state;
static const struct device dut = {
	.name = "i2c_test",
	.config = &drv_cfg,
	.data = &drv_data,
	.state = &dev_state,
};

/* The transfer under test runs in its own thread, because the driver blocks in it until the
 * handler says the transfer is done -- exactly as it does on the board. */
#define XFER_STACK 2048
static K_THREAD_STACK_DEFINE(xfer_stack, XFER_STACK);
static struct k_thread xfer_thread;
static struct k_sem xfer_started;
static struct k_sem xfer_done;
static volatile int xfer_result;
/* Whether the transfer has returned, kept apart from the semaphore. A test that waits for the
 * result consumes the semaphore, and the fixture must not then wait for a signal that has already
 * been taken -- which is what made every such test cost two seconds of nothing. */
static volatile bool xfer_finished;
static bool xfer_live;
static struct i2c_msg xfer_msg;
static uint8_t *xfer_next_flags;

/* The driver writes CR1 last, immediately before it waits, so these bits being set is the
 * observable fact that it has handed the transfer to the interrupt. Bounded, and it fails with
 * what it actually saw rather than hanging. */
static void wait_until_programmed(void)
{
	const uint32_t want = I2C_CR1_ERRIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_NACKIE |
			      I2C_CR1_TXIE | I2C_CR1_RXIE;

	for (int i = 0; i < 100; i++) {
		if ((regs.CR1 & want) == want) {
			return;
		}
		k_msleep(1);
	}
	zassert_unreachable("the driver never enabled the transfer interrupts; CR1 = 0x%08x",
			    regs.CR1);
}

static void xfer_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	k_sem_give(&xfer_started);
	xfer_result = stm32_i2c_transaction(&dut, xfer_msg, xfer_next_flags, 0x2b);
	xfer_finished = true;
	k_sem_give(&xfer_done);
}

static void start_xfer(struct i2c_msg msg, uint8_t *next_flags)
{
	xfer_msg = msg;
	xfer_next_flags = next_flags;
	xfer_result = 0x7fffffff;
	xfer_finished = false;
	k_sem_init(&xfer_started, 0, 1);
	k_sem_init(&xfer_done, 0, 1);
	/* Cooperative and higher priority than the test thread: once created it runs without being
	 * preempted until it blocks waiting for the handler, so by the time k_thread_create
	 * returns the peripheral is programmed and the driver is at its wait. No sleeping, and
	 * nothing that depends on how a CI machine schedules.
	 */
	k_thread_create(&xfer_thread, xfer_stack, XFER_STACK, xfer_entry, NULL, NULL, NULL,
			K_PRIO_COOP(1), 0, K_NO_WAIT);
	xfer_live = true;
	/* The ztest thread is itself cooperative, so a higher priority alone does not take the
	 * CPU: hand it over explicitly. The transfer thread then runs, uninterrupted, until it
	 * blocks waiting for the handler, and control comes back here with the peripheral
	 * programmed. */
	k_yield();
	zassert_ok(k_sem_take(&xfer_started, K_NO_WAIT), "transfer thread did not run");
	wait_until_programmed();
}

unsigned int lexx_test_isr_reads;

/* Which CR1 bit lets each status flag reach the NVIC. A peripheral with the enable clear, or
 * switched off entirely, raises nothing -- which is the whole point of disabling them on a
 * timeout, so the tests have to model it rather than calling the handler regardless. */
static bool interrupt_would_fire(uint32_t flags)
{
	static const struct { uint32_t flag, enable; } gate[] = {
		{ I2C_ISR_TXIS, I2C_CR1_TXIE },   { I2C_ISR_RXNE, I2C_CR1_RXIE },
		{ I2C_ISR_TC, I2C_CR1_TCIE },     { I2C_ISR_TCR, I2C_CR1_TCIE },
		{ I2C_ISR_STOPF, I2C_CR1_STOPIE }, { I2C_ISR_NACKF, I2C_CR1_NACKIE },
		{ I2C_ISR_ADDR, I2C_CR1_ADDRIE },
	};

	if ((regs.CR1 & I2C_CR1_PE) == 0U) {
		return false;
	}
	for (size_t i = 0; i < ARRAY_SIZE(gate); i++) {
		if ((flags & gate[i].flag) != 0U && (regs.CR1 & gate[i].enable) != 0U) {
			return true;
		}
	}
	return false;
}

/* The production interrupt entry point -- the one the vector table calls -- not an internal
 * helper, so the test enters the driver exactly where the hardware does. Returns whether the
 * interrupt actually reached it. */
static bool raise(uint32_t flags)
{
	regs.ISR |= flags;
	if (!interrupt_would_fire(flags)) {
		return false;
	}
	stm32_i2c_event_isr((void *)&dut);
	return true;
}

/* Same, for the flags the error line carries. */
static bool raise_err(uint32_t flags)
{
	regs.ISR |= flags;
	if ((regs.CR1 & (I2C_CR1_PE | I2C_CR1_ERRIE)) != (I2C_CR1_PE | I2C_CR1_ERRIE)) {
		return false;
	}
	stm32_i2c_error_isr((void *)&dut);
	return true;
}

/* Wait for the transfer thread to leave the driver and report. */
static int xfer_wait(k_timeout_t t)
{
	if (k_sem_take(&xfer_done, t) != 0) {
		return 0x7fffffff;
	}
	return xfer_result;
}

/* Asking must not consume the completion, which is why this reads the flag and not the
 * semaphore: a test that checks whether the driver is still waiting would otherwise take the
 * signal it was only asking about. */
static bool xfer_still_waiting(void)
{
	return !xfer_finished;
}

static uint32_t nbytes(void) { return (regs.CR2 & I2C_CR2_NBYTES_Msk) >> I2C_CR2_NBYTES_Pos; }
static bool reload_set(void) { return (regs.CR2 & I2C_CR2_RELOAD) != 0U; }

/* Every test leaves the transfer in whatever state it was studying, so the fixture ends it the way
 * the bus would -- with a STOP -- and waits for the thread to leave the driver before the next
 * test touches the shared structures. */
static void end_xfer(void *unused)
{
	ARG_UNUSED(unused);
	if (!xfer_live) {
		return;
	}
	if (!xfer_finished) {
		regs.ISR |= I2C_ISR_STOPF;
		if ((regs.CR1 & I2C_CR1_PE) != 0U) {
			stm32_i2c_event_isr((void *)&dut);
		}
		(void)k_sem_take(&xfer_done, K_MSEC(2000));
	}
	(void)k_thread_join(&xfer_thread, K_MSEC(2000));
	xfer_live = false;
}

static void reset_dut(void *unused)
{
	ARG_UNUSED(unused);
	lexx_test_isr_reads = 0;
	memset(&regs, 0, sizeof regs);
	memset(&drv_data, 0, sizeof drv_data);
	k_sem_init(&drv_data.device_sync_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&drv_data.bus_mutex, 1, 1);
	drv_data.is_configured = true;
}

/* ---------------------------------------------------------------------------------------------
 * The defect that wedged the board.
 */

ZTEST(i2c_stm32_v2_isr, test_tcr_without_data_must_not_move_the_buffer)
{
	uint8_t buf[4] = {0};
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);

	const unsigned int len_before = drv_data.current.len;
	const uint8_t *buf_before = drv_data.current.buf;

	zassert_true(len_before != 0U, "the transfer should be waiting for data");

	/* An interrupt carrying TCR and BUSY and nothing else: no byte moved on the wire. The old
	 * handler advanced over it anyway, which is how the byte count got ahead of the hardware.
	 */
	raise(I2C_ISR_TCR | I2C_ISR_BUSY);

	zassert_equal(drv_data.current.len, len_before, "TCR without RXNE/TXIS consumed a byte");
	zassert_equal(drv_data.current.buf, buf_before, "TCR without RXNE/TXIS moved the buffer");
}

ZTEST(i2c_stm32_v2_isr, test_rxne_moves_exactly_one_byte)
{
	uint8_t buf[4] = {0};
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);

	const unsigned int len_before = drv_data.current.len;

	regs.RXDR = 0xA5;
	raise(I2C_ISR_RXNE | I2C_ISR_BUSY);

	zassert_equal(drv_data.current.len, len_before - 1U, "RXNE moved %u bytes",
		      len_before - drv_data.current.len);
	zassert_equal(buf[0], 0xA5, "the byte did not reach the caller's buffer");
	zassert_false((regs.ISR & I2C_ISR_RXNE) != 0U, "RXNE was not cleared by reading RXDR");
}


/* ---------------------------------------------------------------------------------------------
 * Segmentation: the 255-byte boundary, and which way a transfer ends.
 */

ZTEST(i2c_stm32_v2_isr, test_320_byte_read_segments_and_ends_in_tc)
{
	static uint8_t buf[320];
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	memset(buf, 0, sizeof buf);
	start_xfer(msg, NULL);

	/* The controller cannot count past 255, so the first segment is 255 with reload on. */
	zassert_equal(nbytes(), 255U, "first segment NBYTES is %u", nbytes());
	zassert_true(reload_set(), "the first segment of a split transfer must use reload");
	zassert_true((regs.CR2 & I2C_CR2_RD_WRN) != 0U, "a read must set RD_WRN");

	for (int i = 0; i < 255; i++) {
		regs.RXDR = (uint8_t)i;
		zassert_true(raise(I2C_ISR_RXNE | I2C_ISR_BUSY), "RXNE did not reach the handler");
	}
	zassert_equal(drv_data.current.len, 65U, "after 255 bytes, %u remain",
		      drv_data.current.len);
	zassert_equal(buf[254], 254, "the 255th byte is wrong");

	/* The hardware now asks for the next length. THIS is where the board wedged: the old
	 * driver re-armed NBYTES and left RELOAD set, so the last segment ended in TCR again and
	 * a byte arrived with nothing left to put it in. */
	zassert_true(raise(I2C_ISR_TCR | I2C_ISR_BUSY), "TCR did not reach the handler");
	zassert_equal(nbytes(), 65U, "final segment NBYTES is %u, expected 65", nbytes());
	zassert_false(reload_set(), "RELOAD is still set on the FINAL segment");

	for (int i = 0; i < 65; i++) {
		regs.RXDR = (uint8_t)(0x40 + i);
		zassert_true(raise(I2C_ISR_RXNE | I2C_ISR_BUSY), "RXNE did not reach the handler");
	}
	zassert_equal(drv_data.current.len, 0U, "the message is not finished");
	zassert_equal(buf[319], 0x40 + 64, "the last byte is wrong");

	/* With reload off the peripheral reports TC, and the message asked for a STOP. */
	zassert_true(xfer_still_waiting(), "the transfer ended before TC");
	zassert_true(raise(I2C_ISR_TC | I2C_ISR_BUSY), "TC did not reach the handler");
	zassert_true((regs.CR2 & I2C_CR2_STOP) != 0U, "TC with STOP in the message issued no STOP");
	zassert_true(xfer_still_waiting(), "the transfer ended before the STOP was seen");

	regs.ISR &= ~(I2C_ISR_TC | I2C_ISR_BUSY);
	zassert_true(raise(I2C_ISR_STOPF), "STOPF did not reach the handler");
	zassert_equal(xfer_wait(K_MSEC(200)), 0, "the transfer did not finish cleanly");
}

ZTEST(i2c_stm32_v2_isr, test_final_segment_with_no_next_message_clears_reload)
{
	static uint8_t buf[300];
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);
	zassert_false(drv_data.current.continue_in_next, "nothing follows this message");

	for (int i = 0; i < 255; i++) {
		regs.RXDR = 0;
		(void)raise(I2C_ISR_RXNE | I2C_ISR_BUSY);
	}
	(void)raise(I2C_ISR_TCR | I2C_ISR_BUSY);

	zassert_equal(nbytes(), 45U, "final segment NBYTES is %u", nbytes());
	zassert_false(reload_set(), "the last segment of the last message must end in TC");
}

ZTEST(i2c_stm32_v2_isr, test_same_direction_next_message_keeps_reload)
{
	static uint8_t buf[8];
	uint8_t next_flags = 0; /* no RESTART: the next message continues the same way */
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_WRITE | I2C_MSG_RESTART };

	start_xfer(msg, &next_flags);

	zassert_true(drv_data.current.continue_in_next,
		     "a same-direction continuation must be flagged for the handler");
	zassert_true(reload_set(), "a continued transfer runs in reload mode");

	/* One byte went out with the start as the errata workaround wants, so seven remain. */
	for (int i = 0; i < 7; i++) {
		zassert_true(raise(I2C_ISR_TXIS | I2C_ISR_BUSY), "TXIS did not reach the handler");
	}
	zassert_equal(drv_data.current.len, 0U, "the message is not finished");

	/* Everything of THIS message has moved, and more follows the same way: the handler hands
	 * back to the thread with reload still on, so the next message just writes a length. */
	zassert_true(raise(I2C_ISR_TCR | I2C_ISR_BUSY), "TCR did not reach the handler");
	zassert_equal(xfer_wait(K_MSEC(200)), 0, "the message did not complete");
	zassert_true(reload_set(), "reload was dropped although another message follows");
}

ZTEST(i2c_stm32_v2_isr, test_next_message_with_restart_does_not_use_reload)
{
	static uint8_t buf[8];
	uint8_t next_flags = I2C_MSG_RESTART;
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_WRITE | I2C_MSG_RESTART };

	start_xfer(msg, &next_flags);

	zassert_false(drv_data.current.continue_in_next,
		      "a restart ends this transfer, so it must not be a continuation");
	zassert_false(reload_set(), "a transfer followed by a restart must not use reload");
}

ZTEST(i2c_stm32_v2_isr, test_txis_moves_exactly_one_byte)
{
	static uint8_t buf[4] = { 0x11, 0x22, 0x33, 0x44 };
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_WRITE | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);

	/* The first byte is put in the shift register before the start, per the errata. */
	zassert_equal(regs.TXDR, 0x11, "the first byte was not preloaded");
	zassert_equal(drv_data.current.len, 3U, "preloading moved %u bytes",
		      4U - drv_data.current.len);

	const unsigned int before = drv_data.current.len;

	zassert_true(raise(I2C_ISR_TXIS | I2C_ISR_BUSY), "TXIS did not reach the handler");
	zassert_equal(drv_data.current.len, before - 1U, "TXIS moved more than one byte");
	zassert_equal(regs.TXDR, 0x22, "the wrong byte went out");
	zassert_false((regs.ISR & I2C_ISR_TXIS) != 0U, "TXIS was not cleared by writing TXDR");
}

/* ---------------------------------------------------------------------------------------------
 * A whole small transaction, the shape the board actually issues most often.
 */

ZTEST(i2c_stm32_v2_isr, test_four_byte_indexed_read_write_restart_read)
{
	static uint8_t index[2] = { 0x00, 0x00 };
	static uint8_t data[4];
	uint8_t next_flags = I2C_MSG_RESTART;
	struct i2c_msg wr = { .buf = index, .len = sizeof index,
			      .flags = I2C_MSG_WRITE | I2C_MSG_RESTART };
	struct i2c_msg rd = { .buf = data, .len = sizeof data,
			      .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	/* Phase one: write the two register-index bytes, no STOP, a restart to follow. */
	start_xfer(wr, &next_flags);
	zassert_false(reload_set(), "a restart follows, so this must not be a reload transfer");
	zassert_equal(nbytes(), 2U, "NBYTES is %u for a two-byte index", nbytes());
	zassert_true((regs.CR2 & I2C_CR2_START) != 0U, "no start condition was requested");
	zassert_equal(regs.TXDR, 0x00, "the index byte was not preloaded");

	zassert_true(raise(I2C_ISR_TXIS | I2C_ISR_BUSY), "TXIS did not reach the handler");
	zassert_equal(drv_data.current.len, 0U, "the index was not fully sent");

	/* No STOP in this message: the handler hands back for the restart and leaves TC set. */
	zassert_true(raise(I2C_ISR_TC | I2C_ISR_BUSY), "TC did not reach the handler");
	zassert_false((regs.CR2 & I2C_CR2_STOP) != 0U, "a STOP was issued mid-transaction");
	zassert_equal(xfer_wait(K_MSEC(200)), 0, "the index write did not complete");
	end_xfer(NULL);

	/* Phase two: the read, entered with TC still set, which is the driver's restart path. */
	start_xfer(rd, NULL);
	zassert_true((regs.CR2 & I2C_CR2_RD_WRN) != 0U, "the restart did not turn the bus around");
	zassert_equal(nbytes(), 4U, "NBYTES is %u for a four-byte read", nbytes());

	for (int i = 0; i < 4; i++) {
		regs.RXDR = (uint8_t)(0xA0 + i);
		zassert_true(raise(I2C_ISR_RXNE | I2C_ISR_BUSY), "RXNE did not reach the handler");
	}
	zassert_equal(data[3], 0xA3, "the payload is wrong");

	zassert_true(raise(I2C_ISR_TC | I2C_ISR_BUSY), "TC did not reach the handler");
	zassert_true((regs.CR2 & I2C_CR2_STOP) != 0U, "the last message issued no STOP");
	regs.ISR &= ~(I2C_ISR_TC | I2C_ISR_BUSY);
	zassert_true(raise(I2C_ISR_STOPF), "STOPF did not reach the handler");
	zassert_equal(xfer_wait(K_MSEC(200)), 0, "the read did not complete");
}

/* ---------------------------------------------------------------------------------------------
 * The synchronising read, and what the handler does with an entry it cannot serve.
 */

ZTEST(i2c_stm32_v2_isr, test_handler_reads_the_status_back_before_returning)
{
	static uint8_t buf[4];
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);

	lexx_test_isr_reads = 0;
	regs.RXDR = 0x5A;
	zassert_true(raise(I2C_ISR_RXNE | I2C_ISR_BUSY), "RXNE did not reach the handler");

	/* One read on entry, one before returning. Without the second, the CPU can re-enter this
	 * level-triggered line on a flag the peripheral has not finished clearing. */
	zassert_equal(lexx_test_isr_reads, 2U,
		      "the handler read the status register %u times, expected 2",
		      lexx_test_isr_reads);

	/* The completion path hands over to the thread instead of returning, and upstream does not
	 * synchronise there -- record that, so a change to it is noticed. */
	lexx_test_isr_reads = 0;
	regs.ISR &= ~I2C_ISR_BUSY;
	zassert_true(raise(I2C_ISR_STOPF), "STOPF did not reach the handler");
	zassert_equal(lexx_test_isr_reads, 1U,
		      "the completion path read the status %u times, expected 1",
		      lexx_test_isr_reads);
	zassert_equal(xfer_wait(K_MSEC(200)), 0, "the transfer did not complete");
}

/* An interrupt whose flags are already gone by the time the handler reads the status. The NVIC
 * latched something; the peripheral cleared it in the meantime. This cannot go through raise(),
 * which models the gate, because the point is that nothing is set any more. */
static void raise_spurious(void)
{
	regs.ISR &= ~(I2C_ISR_TXIS | I2C_ISR_RXNE | I2C_ISR_TC | I2C_ISR_TCR | I2C_ISR_STOPF |
		      I2C_ISR_NACKF | I2C_ISR_ADDR);
	stm32_i2c_event_isr((void *)&dut);
}

ZTEST(i2c_stm32_v2_isr, test_entry_with_nothing_to_serve)
{
	static uint8_t buf[4];
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);

	const unsigned int len_before = drv_data.current.len;
	const uint8_t *buf_before = drv_data.current.buf;
	const uint32_t cr2_before = regs.CR2;

#ifdef CONFIG_ASSERT
	/* Built with assertions, this is fatal, as upstream intends. ztest catches it so the rest
	 * of the suite still runs. */
	ztest_set_assert_valid(true);
	raise_spurious();
	ztest_set_assert_valid(false);
#else
	/* The product builds with assertions off, so __ASSERT_NO_MSG compiles to nothing and the
	 * handler must simply change nothing and leave -- including the synchronising read, which
	 * is what stops it being re-entered on the same stale state. */
	lexx_test_isr_reads = 0;
	raise_spurious();

	zassert_equal(drv_data.current.len, len_before, "a flagless entry consumed a byte");
	zassert_equal(drv_data.current.buf, buf_before, "a flagless entry moved the buffer");
	zassert_equal(regs.CR2, cr2_before, "a flagless entry reprogrammed the transfer");
	zassert_equal(lexx_test_isr_reads, 2U,
		      "a flagless entry read the status %u times, expected 2 (entry and the "
		      "synchronising read)", lexx_test_isr_reads);
	zassert_true(xfer_still_waiting(), "a flagless entry ended the transfer");
#endif
}

/* ---------------------------------------------------------------------------------------------
 * Error codes. The chain enumeration depends on telling these apart.
 */

ZTEST(i2c_stm32_v2_isr, test_clean_nack_is_enxio)
{
	static uint8_t buf[2] = { 0, 0 };
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_WRITE | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);

	/* Nobody at this address: the peripheral NACKs and sends a STOP by itself. */
	zassert_true(raise(I2C_ISR_NACKF | I2C_ISR_BUSY), "NACKF did not reach the handler");
	zassert_equal(drv_data.current.is_nack, 1U, "the NACK was not recorded");
	regs.ISR &= ~I2C_ISR_BUSY;
	zassert_true(raise(I2C_ISR_STOPF), "STOPF did not reach the handler");

	zassert_equal(xfer_wait(K_MSEC(200)), -ENXIO,
		      "a clean NACK must be -ENXIO, or the chain scan cannot tell an empty "
		      "address from a broken bus");
}

ZTEST(i2c_stm32_v2_isr, test_bus_error_is_eio)
{
	static uint8_t buf[2];
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);
	zassert_true(raise_err(I2C_ISR_BERR), "the error line did not reach the handler");
	zassert_equal(xfer_wait(K_MSEC(200)), -EIO, "a bus error must be -EIO");
}

ZTEST(i2c_stm32_v2_isr, test_arbitration_loss_is_eio)
{
	static uint8_t buf[2];
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);
	zassert_true(raise_err(I2C_ISR_ARLO), "the error line did not reach the handler");
	zassert_equal(xfer_wait(K_MSEC(200)), -EIO, "arbitration loss must be -EIO");
}

ZTEST(i2c_stm32_v2_isr, test_silence_is_etimedout)
{
	static uint8_t buf[2];
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);
	/* Nothing answers. The driver waits its own timeout and gives up. */
	zassert_equal(xfer_wait(K_MSEC(2000)), -ETIMEDOUT, "silence must be -ETIMEDOUT");
}

/* ---------------------------------------------------------------------------------------------
 * What a timeout must leave behind: nothing.
 */

ZTEST(i2c_stm32_v2_isr, test_timeout_leaves_no_completion_token)
{
	static uint8_t buf[2];
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);
	zassert_equal(xfer_wait(K_MSEC(2000)), -ETIMEDOUT, "the transfer should have timed out");

	zassert_equal(k_sem_count_get(&drv_data.device_sync_sem), 0U,
		      "a completion token survived the timeout; the next transfer would take it");
}

ZTEST(i2c_stm32_v2_isr, test_timeout_silences_the_peripheral)
{
	static uint8_t buf[2];
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	start_xfer(msg, NULL);
	zassert_equal(xfer_wait(K_MSEC(2000)), -ETIMEDOUT, "the transfer should have timed out");

	/* The abandoned transfer's interrupts are off and the peripheral is disabled, so a flag
	 * arriving late cannot reach the handler at all. */
	zassert_false(raise(I2C_ISR_STOPF), "a late STOP still reached the handler");
	zassert_false(raise(I2C_ISR_RXNE), "a late RXNE still reached the handler");
	zassert_equal(k_sem_count_get(&drv_data.device_sync_sem), 0U,
		      "a late interrupt produced a completion after the timeout");
}

ZTEST(i2c_stm32_v2_isr, test_next_transfer_waits_for_its_own_completion)
{
	static uint8_t buf[2];
	struct i2c_msg msg = { .buf = buf, .len = sizeof buf,
			       .flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP };

	/* The worst case the timeout race can produce: a token from a transfer nobody is waiting
	 * for any more. Put one there directly, so the test does not depend on winning a race. */
	k_sem_give(&drv_data.device_sync_sem);
	zassert_equal(k_sem_count_get(&drv_data.device_sync_sem), 1U, "the stray token is not set");

	start_xfer(msg, NULL);
	zassert_true(xfer_still_waiting(),
		     "the transfer took a token it did not earn and reported success before a "
		     "single byte had moved");

	regs.RXDR = 0x01;
	(void)raise(I2C_ISR_RXNE | I2C_ISR_BUSY);
	regs.RXDR = 0x02;
	(void)raise(I2C_ISR_RXNE | I2C_ISR_BUSY);
	(void)raise(I2C_ISR_TC | I2C_ISR_BUSY);
	regs.ISR &= ~(I2C_ISR_TC | I2C_ISR_BUSY);
	(void)raise(I2C_ISR_STOPF);

	zassert_equal(xfer_wait(K_MSEC(200)), 0, "the transfer did not complete on its own IRQ");
}

ZTEST_SUITE(i2c_stm32_v2_isr, NULL, NULL, reset_dut, end_xfer, NULL);
