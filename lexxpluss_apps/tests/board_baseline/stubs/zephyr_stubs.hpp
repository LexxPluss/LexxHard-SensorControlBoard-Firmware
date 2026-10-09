#include <iostream>
#include <cstdlib>
// Zephyr stub headers for host-based testing
#pragma once

#include <cstdint>
#include <chrono>
#include <map>
#include <vector>
#include <functional>
#include <memory>
#include <optional>
#include <array>
#include <queue>
#include <algorithm>
#include <string>

struct k_timer;

namespace zephyr_stubs {

    struct PinWrite {
        int64_t time;
        int pin;
        int level;
    };

    // Virtual time management
    extern int64_t virtual_time_ms;
    extern std::vector<PinWrite> pin_write_log;
    extern std::vector<k_timer*> active_timers;
    extern std::map<int, int> pin_state;

    void advance_virtual_time(int64_t ms);
    void set_pin_input(int pin_id, int level);
    int get_pin_output(int pin_id);
    void reset_pin_log();

} // namespace zephyr_stubs

// k_uptime_get - returns virtual milliseconds since start
inline int64_t k_uptime_get() {
    return zephyr_stubs::virtual_time_ms;
}

// k_msleep - advances virtual time
inline void k_msleep(int64_t ms) {
    zephyr_stubs::advance_virtual_time(ms);
}

// k_usleep - the virtual clock has millisecond resolution, so only the requested time is accumulated
namespace zephyr_stubs {
    inline int64_t usleep_total_us = 0;
}
inline int k_usleep(int32_t us) {
    zephyr_stubs::usleep_total_us += us;
    return 0;
}

// Timer support
struct k_timer {
    void (*callback)(struct k_timer*) = nullptr;
    void* user_data = nullptr;
    int64_t start_time = 0;
    int64_t period_ms = 0;
    bool periodic = false;
    bool active = false;
};

inline void k_timer_init(k_timer* timer, void (*callback)(k_timer*), void* user_data) {
    timer->callback = callback;
    timer->user_data = user_data;
}

inline void k_timer_user_data_set(k_timer* timer, void* user_data) {
    timer->user_data = user_data;
}

inline void k_timer_start(k_timer* timer, int64_t delay_ms, int64_t period_ms) {
    timer->start_time = zephyr_stubs::virtual_time_ms + delay_ms;
    timer->period_ms = period_ms;
    timer->periodic = (period_ms > 0);
    timer->active = true;
    auto& timers = zephyr_stubs::active_timers;
    if (std::find(timers.begin(), timers.end(), timer) == timers.end()) {
        timers.push_back(timer);
    }
}

inline void k_timer_stop(k_timer* timer) {
    timer->active = false;
}

inline void* k_timer_user_data_get(k_timer* timer) {
    return timer->user_data;
}

// Message queue support
struct k_msgq {
    std::queue<std::vector<uint8_t>> queue;
    size_t msg_size = 0;
    size_t max_msgs = 0;
};

inline void k_msgq_init(k_msgq* q, char* buffer, size_t msg_size, size_t max_msgs) {
    q->msg_size = msg_size;
    q->max_msgs = max_msgs;
}

inline int k_msgq_put(k_msgq* q, const void* data, int timeout) {
    if (q->queue.size() >= q->max_msgs) {
        return -1;
    }
    const auto* bytes = static_cast<const uint8_t*>(data);
    q->queue.push(std::vector<uint8_t>(bytes, bytes + q->msg_size));
    return 0;
}

inline int k_msgq_get(k_msgq* q, void* data, int timeout) {
    if (q->queue.empty()) {
        return -1;
    }
    const auto& msg = q->queue.front();
    std::copy(msg.begin(), msg.end(), static_cast<uint8_t*>(data));
    q->queue.pop();
    return 0;
}

inline void k_msgq_purge(k_msgq* q) {
    while (!q->queue.empty()) {
        q->queue.pop();
    }
}

// Thread stub
struct k_thread {};

// Time constants
constexpr int K_MSEC(int ms) { return ms; }
constexpr int K_NO_WAIT = 0;

// GPIO support
struct gpio_dt_spec {
    const void* port = nullptr;
    int pin = 0;
};

inline bool gpio_is_ready_dt(const gpio_dt_spec* spec) {
    return spec != nullptr;
}

inline int gpio_pin_get_dt(const gpio_dt_spec* spec) {
    if (!spec) return 0;
    return zephyr_stubs::pin_state[spec->pin];
}

inline int gpio_pin_set_dt(const gpio_dt_spec* spec, int value) {
    if (!spec) return -1;
    zephyr_stubs::pin_state[spec->pin] = value;
    zephyr_stubs::pin_write_log.push_back({zephyr_stubs::virtual_time_ms, spec->pin, value});
    return 0;
}

// GET_GPIO macro - creates gpio_dt_spec from pin ID
#define GET_GPIO(label) zephyr_stubs::gpio_dt_spec_from_label(#label)

namespace zephyr_stubs {
    inline gpio_dt_spec gpio_dt_spec_from_label(const char* label) {
        static std::map<std::string, int> label_to_pin;
        if (label_to_pin.empty()) {
            // Map all GPIO labels to unique IDs
            label_to_pin["bmu_c_fet"] = 0;
            label_to_pin["bmu_d_fet"] = 1;
            label_to_pin["bmu_p_dsg"] = 2;
            label_to_pin["bp_left"] = 3;
            label_to_pin["bp_reset"] = 4;
            label_to_pin["eo_option_1"] = 5;
            label_to_pin["es_left"] = 6;
            label_to_pin["es_option_1"] = 7;
            label_to_pin["es_option_2"] = 8;
            label_to_pin["es_right"] = 9;
            label_to_pin["fan1"] = 10;
            label_to_pin["fan2"] = 11;
            label_to_pin["fan3"] = 12;
            label_to_pin["fan4"] = 13;
            label_to_pin["key_switch_left"] = 14;
            label_to_pin["key_switch_right"] = 15;
            label_to_pin["mc_din"] = 16;
            label_to_pin["pgood_24v"] = 17;
            label_to_pin["pgood_peripheral"] = 18;
            label_to_pin["pgood_wheel_motor_left"] = 19;
            label_to_pin["pgood_wheel_motor_right"] = 20;
            label_to_pin["ps_led_out"] = 21;
            label_to_pin["ps_sw_in"] = 22;
            label_to_pin["resume_led_out"] = 23;
            label_to_pin["resume_sw_in"] = 24;
            label_to_pin["safety_lidar_ossd1"] = 25;
            label_to_pin["safety_lidar_ossd2"] = 26;
            label_to_pin["v24"] = 27;
            label_to_pin["v_autocharge"] = 28;
            label_to_pin["v_peripheral"] = 29;
            label_to_pin["v_wheel"] = 30;
            label_to_pin["wheel_en"] = 31;
            label_to_pin["comm_mode"] = 32;
        }
        const auto it = label_to_pin.find(label);
        if (it == label_to_pin.end()) {
            std::cerr << "unknown gpio label: " << label << std::endl;
            std::abort();
        }
        gpio_dt_spec spec;
        spec.pin = it->second;
        return spec;
    }
} // namespace zephyr_stubs

// Device support
struct device {};

inline const device* GET_DEV(int id) {
    static device dev;
    return &dev;
}

inline bool device_is_ready(const device* dev) {
    return dev != nullptr;
}

// Watchdog stubs
struct wdt_timeout_cfg {
    int flags = 0;
    struct {
        int min = 0;
        int max = 0;
    } window;
    void (*callback)(const device*, int) = nullptr;
};

constexpr int WDT_FLAG_RESET_SOC = 1;
constexpr int WDT_OPT_PAUSE_HALTED_BY_DBG = 1;
constexpr int WDT_TIMEOUT_MS = 5000;

inline int wdt_install_timeout(const device* dev, wdt_timeout_cfg* cfg) {
    return 0;
}

inline int wdt_setup(const device* dev, int options) {
    return 0;
}

// UART stubs
struct uart_data_callback_user_data {};
inline int uart_callback_set(const device* dev, uart_data_callback_user_data* cb) {
    return 0;
}

// Logging macros - support both 1 and 2 argument versions
#define LOG_MODULE_REGISTER(...)
#define LOG_DBG(fmt, ...)
#define LOG_INF(fmt, ...)
#define LOG_ERR(fmt, ...)
#define LOG_WRN(fmt, ...)

// Shell macros
#define SHELL_CMD_ARG(name, cmds, help, handler, argc, argv)
#define SHELL_CMD(name, cmds, help, handler)
#define SHELL_SUBCMD_SET_END
#define SHELL_SUBCMD_SET(name, ...) ((void)0)
#define SHELL_CMD_REGISTER(...)
#define SHELL_STATIC_SUBCMD_SET_CREATE(...)
