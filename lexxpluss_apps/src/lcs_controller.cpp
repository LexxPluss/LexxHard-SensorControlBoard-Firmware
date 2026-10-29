/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include "lcs_can_packer.hpp"
#include "lcs_controller.hpp"
#include "lcs_rx.hpp"
#include "lcs_sync_schedule.hpp"
#include "lcs_timing.hpp"

namespace lexxhard::lcs_controller {

LOG_MODULE_REGISTER(lcs);

namespace {

/* One tick of the receive core is kTickUs, so it must be the kernel tick. */
static_assert(CONFIG_SYS_CLOCK_TICKS_PER_SEC == 1000000 / lcs::kTickUs, "kernel tick must be lcs::kTickUs");

constexpr size_t kRxBufferSize{256};
constexpr size_t kChunkSize{32};
constexpr int kChunkQueueDepth{32};
constexpr int kForwardQueueDepth{16};
constexpr uint32_t kTxTimeoutTicks{2000 / lcs::kTickUs};
/* The loop wakes at least this often, so expiry, TX supervision and RX restart keep a 1 ms cadence. */
constexpr uint32_t kMaxWaitTicks{1000 / lcs::kTickUs};

/* The transceiver needs this long to settle after DE rises; the UART must not start before. */
constexpr uint32_t kDeSettleUs{11};

struct RxChunk {
    uint32_t tick;
    uint8_t len;
    bool error;
    uint8_t data[kChunkSize];
};

/* DMA does not snoop the data cache. */
uint8_t rx_buffer[kRxBufferSize] __nocache __aligned(32);
uint8_t tx_buffer[lcs::kDownlinkSize] __nocache __aligned(32);

char __aligned(4) chunk_storage[kChunkQueueDepth * sizeof(RxChunk)];
char __aligned(4) forward_storage[kForwardQueueDepth * sizeof(lcs::msg)];
k_msgq chunk_queue;

/* Receiver, packer, scheduler and counters in one object so that they can be read as members. */
struct Host {
    lcs::UplinkReceiver receiver;
    lcs::CanPacker packer;
    lcs::SyncScheduler scheduler;
    uint32_t rx_queue_overflow;
    uint32_t tx_timeout;
    uint32_t tx_rejected_busy;
    uint32_t rx_restarts;
    uint32_t tx_start_tick;
    atomic_t tx_busy;
    atomic_t rx_restart_pending;
    atomic_t ready;
    const device *uart;
    gpio_dt_spec de;
    gpio_dt_spec re;
};

Host host;

uint32_t now_ticks()
{
    return static_cast<uint32_t>(k_uptime_ticks());
}

void set_direction(const bool transmit)
{
    gpio_pin_set_dt(&host.de, transmit ? 1 : 0);
    gpio_pin_set_dt(&host.re, transmit ? 1 : 0);
}

void put_chunk(const uint32_t tick, const uint8_t *const data, const size_t len, const bool error)
{
    RxChunk chunk{.tick = tick, .len = static_cast<uint8_t>(len), .error = error, .data = {}};
    if (len > 0)
        std::memcpy(chunk.data, data, len);
    if (k_msgq_put(&chunk_queue, &chunk, K_NO_WAIT) != 0)
        ++host.rx_queue_overflow;
}

void uart_callback(const device *, uart_event *event, void *)
{
    switch (event->type) {
    case UART_RX_RDY: {
        const uart_event_rx &rx{event->data.rx};
        const uint32_t tick{now_ticks()};
        for (size_t done{0}; done < rx.len; done += kChunkSize)
            put_chunk(tick, rx.buf + rx.offset + done, std::min(kChunkSize, rx.len - done), false);
        break;
    }
    case UART_RX_STOPPED:
        put_chunk(now_ticks(), nullptr, 0, true);
        break;
    case UART_RX_DISABLED:
        atomic_set(&host.rx_restart_pending, 1);
        break;
    case UART_RX_BUF_REQUEST:
    case UART_RX_BUF_RELEASED:
        /* Cyclic mode: the one buffer is reused, no next buffer is given. */
        break;
    case UART_TX_DONE:
    case UART_TX_ABORTED:
        set_direction(false);
        atomic_set(&host.tx_busy, 0);
        break;
    }
}

bool send_sync(const lcs::DownlinkFrame &frame)
{
    if (!atomic_cas(&host.tx_busy, 0, 1)) {
        ++host.tx_rejected_busy;
        return false;
    }
    std::memcpy(tx_buffer, frame.data(), frame.size());
    host.tx_start_tick = now_ticks();
    set_direction(true);
    k_busy_wait(kDeSettleUs);
    if (uart_tx(host.uart, tx_buffer, frame.size(), SYS_FOREVER_US) != 0) {
        set_direction(false);
        atomic_set(&host.tx_busy, 0);
        return false;
    }
    return true;
}

void guard_tx()
{
    /* Own time stamp: a transmit started in this pass is later than the pass's first one. */
    if (atomic_get(&host.tx_busy) == 0 || now_ticks() - host.tx_start_tick <= kTxTimeoutTicks)
        return;
    ++host.tx_timeout;
    if (uart_tx_abort(host.uart) != 0) {
        /* Nothing was in progress, so no event will come to release the line. */
        set_direction(false);
        atomic_set(&host.tx_busy, 0);
    }
}

void restart_rx()
{
    if (!atomic_cas(&host.rx_restart_pending, 1, 0))
        return;
    ++host.rx_restarts;
    if (uart_rx_enable(host.uart, rx_buffer, sizeof(rx_buffer), 0) != 0)
        atomic_set(&host.rx_restart_pending, 1);
}

void forward(const lcs::Uplink &uplink)
{
    const std::optional<lcs::msg> frame{host.packer.pack(uplink)};
    if (!frame)
        return;
    while (k_msgq_put(&msgq, &*frame, K_NO_WAIT) != 0)
        k_msgq_purge(&msgq);
}

void handle_chunk(const RxChunk &chunk)
{
    if (chunk.error) {
        host.receiver.on_uart_error(chunk.tick);
        return;
    }
    for (size_t i{0}; i < chunk.len; ++i) {
        if (const std::optional<lcs::Uplink> uplink{host.receiver.on_byte(chunk.data[i], chunk.tick)}; uplink)
            forward(*uplink);
    }
}

}

void init()
{
    k_msgq_init(&msgq, forward_storage, sizeof(lcs::msg), kForwardQueueDepth);
    k_msgq_init(&chunk_queue, chunk_storage, sizeof(RxChunk), kChunkQueueDepth);
    host.uart = DEVICE_DT_GET(DT_NODELABEL(uart4));
    host.de = GPIO_DT_SPEC_GET(DT_NODELABEL(pgv_en), gpios);
    host.re = GPIO_DT_SPEC_GET(DT_NODELABEL(pgv_en_n), gpios);
    if (!device_is_ready(host.uart) || !gpio_is_ready_dt(&host.de) || !gpio_is_ready_dt(&host.re)) {
        LOG_ERR("uart4 or the DE/RE pins are not ready");
        return;
    }
    const uart_config config{
        .baudrate{lcs::kBaud},
        .parity{UART_CFG_PARITY_NONE},
        .stop_bits{UART_CFG_STOP_BITS_1},
        .data_bits{UART_CFG_DATA_BITS_8},
        .flow_ctrl{UART_CFG_FLOW_CTRL_NONE}
    };
    if (uart_configure(host.uart, &config) != 0) {
        LOG_ERR("uart4 configure failed");
        return;
    }
    gpio_pin_configure_dt(&host.de, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_pin_configure_dt(&host.re, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    set_direction(false);
    if (uart_callback_set(host.uart, uart_callback, nullptr) != 0 ||
        uart_rx_enable(host.uart, rx_buffer, sizeof(rx_buffer), 0) != 0) {
        LOG_ERR("uart4 async RX start failed");
        return;
    }
    atomic_set(&host.ready, 1);
}

void run(void *p1, void *p2, void *p3)
{
    while (atomic_get(&host.ready) == 0)
        k_msleep(100);

    while (true) {
        RxChunk chunk;
        /* Wait until the next SYNC is due, at most kMaxWaitTicks, as an absolute time: a relative timeout of the
         * kernel adds one tick. A received chunk ends the wait early. */
        const int64_t wait_from{k_uptime_ticks()};
        const uint32_t wait{host.scheduler.ticks_until_due(static_cast<uint32_t>(wait_from), kMaxWaitTicks)};
        const k_timeout_t timeout{wait == 0 ? K_NO_WAIT : K_TIMEOUT_ABS_TICKS(wait_from + wait)};
        if (k_msgq_get(&chunk_queue, &chunk, timeout) == 0) {
            do {
                handle_chunk(chunk);
            } while (k_msgq_get(&chunk_queue, &chunk, K_NO_WAIT) == 0);
        }
        const uint32_t now{now_ticks()};
        host.receiver.expire(now);
        host.scheduler.poll(now, [](const lcs::DownlinkFrame &frame) { return send_sync(frame); });
        restart_rx();
        guard_tx();
    }
}

k_thread thread;
k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
