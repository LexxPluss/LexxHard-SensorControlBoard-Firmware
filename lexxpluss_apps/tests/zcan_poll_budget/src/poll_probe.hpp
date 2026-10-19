/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The substitute transport these suites run the production pollers against.
 *
 * WHAT IS REAL AND WHAT IS NOT. The pollers are the production ones, included from
 * lexxpluss_apps/src and not reimplemented, and the message queues are real Zephyr queues
 * operated by the real k_msgq calls -- that is the whole point, because what is under test is
 * loop termination and queue accounting. What is faked is one function, zcan_bounded_send::send,
 * and the device handle it would need. Nothing here claims anything about a physical CAN bus;
 * the behaviour of the wrapper itself has its own suite in tests/zcan_bounded_send.
 *
 * WHY THE DEVICE IS A NULL POINTER. Each poller's init() resolves a devicetree node that native_sim
 * does not have, so DEVICE_DT_GET is redefined to nullptr and init() is never called. The pollers
 * are constructed and poll()ed directly: poll() passes `dev` to send() and send() does not look at
 * it, so a null device is exactly as much device as these tests need.
 *
 * WHY SEND CAN REFUSE ON DEMAND. The condition the budget exists for is the one where every send
 * is refused for as long as the host stays away -- mailboxes permanently full. `rc` is what send
 * returns, so a suite can hold it at -EAGAIN and ask what the poller does when nothing it sends
 * ever leaves. `refill` is called from inside send, which reproduces the other half of that
 * condition: a producer that keeps putting messages in the queue while the poller is draining it.
 * Before the budget those two together did not terminate.
 */

#pragma once

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/ztest.h>

/* Must precede any production zcan header: their init() paths would otherwise need a can2 node. */
#undef DEVICE_DT_GET
#define DEVICE_DT_GET(node_id) nullptr

namespace probe {

/* What the faked send returns. -EAGAIN is the interesting default: it is what a full mailbox set
 * gives back once kMailboxWait expires. */
extern int rc;

/* How many times a poll() called send, and the frames it passed, so a suite can assert on content
 * and order rather than only on counts. */
extern int sends;
extern can_frame frames[32];

/* Called from inside send, if set: a producer that refills the queue the poller is draining. */
extern void (*refill)();

void reset();

}  // namespace probe
