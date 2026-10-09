// Zephyr stub implementations
#include "zephyr_stubs.hpp"
#include <iostream>
#include <algorithm>

namespace zephyr_stubs {

    int64_t virtual_time_ms = 0;
    std::vector<PinWrite> pin_write_log;
    std::map<int, int> pin_state;
    std::vector<k_timer*> active_timers;

    void advance_virtual_time(int64_t ms) {
        virtual_time_ms += ms;

        // Process all timers that should fire at or before this time
        for (std::size_t i = 0; i < active_timers.size(); ++i) {
            k_timer* const timer = active_timers[i];
            if (timer && timer->active && timer->start_time <= virtual_time_ms) {
                if (timer->callback) {
                    timer->callback(timer);
                }
                if (timer->periodic && timer->period_ms > 0) {
                    timer->start_time = virtual_time_ms + timer->period_ms;
                } else {
                    timer->active = false;
                }
            }
        }
    }

    void set_pin_input(int pin_id, int level) {
        pin_state[pin_id] = level;
    }

    int get_pin_output(int pin_id) {
        return pin_state[pin_id];
    }

    void reset_pin_log() {
        pin_write_log.clear();
    }

} // namespace zephyr_stubs
