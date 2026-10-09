// LexxHard stub implementations
#include <zephyr/kernel.h>

namespace lexxhard {
    namespace led_controller {
        // Global msgq instance
        char msgq_buffer[8 * 256];
        k_msgq msgq;
    }

    namespace adc_reader {
        // ADC read stubs
        uint16_t get(int) {
            return 0;
        }

        int32_t stub_adc3_mv = 0;

        int32_t get_adc3(int) {
            return stub_adc3_mv;
        }
    }
}
