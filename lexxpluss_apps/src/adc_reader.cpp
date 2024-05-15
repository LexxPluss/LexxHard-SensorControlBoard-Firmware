/*
 * Copyright (c) 2022, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include "adc_reader.hpp"

namespace lexxhard::adc_reader {

LOG_MODULE_REGISTER(adc);

class {
public:
    int init() {
        dev = DEVICE_DT_GET(DT_NODELABEL(adc1));
        dev_adc3 = DEVICE_DT_GET(DT_NODELABEL(adc3));
        return (device_is_ready(dev) && device_is_ready(dev_adc3)) ? 0 : -1;
    }
    void run() {
        if (!device_is_ready(dev) && !device_is_ready(dev_adc3))
            return;
        while (true) {
            read_all_channels();
            k_msleep(20);
        }
    }
    int32_t get(int index) const {
        int32_t value{buffer[index]};
        if (int32_t ref{adc_ref_internal(dev)}; ref > 0)
            adc_raw_to_millivolts(ref, ADC_GAIN_1, 12, &value);
        return value;
    }

    int32_t get_adc3(int index) const {
        int32_t value_adc3{buffer_adc3[index]};
        if (int32_t ref_adc3{adc_ref_internal(dev_adc3)}; ref_adc3 > 0)
            adc_raw_to_millivolts(ref_adc3, ADC_GAIN_1, 12, &value_adc3);
        return value_adc3;
    }
private:
    void read_all_channels() {
        /*  ADC1 */
        static constexpr uint8_t ch[NUM_CHANNELS]{8, 9, 10, 11, 12, 13, 14, 15};
        for (int i{0}; i < NUM_CHANNELS; ++i) {
            // Only single channel supported
            adc_channel_cfg channel_cfg{
                .gain{ADC_GAIN_1},
                .reference{ADC_REF_INTERNAL},
                .acquisition_time{ADC_ACQ_TIME_DEFAULT},
                .channel_id{ch[i]},
                .differential{0}
            };
            adc_channel_setup(dev, &channel_cfg);
            adc_sequence sequence{
                .options{nullptr},
                .channels{BIT(ch[i])},
                .buffer{&buffer[i]},
                .buffer_size{sizeof buffer[i]},
                .resolution{12},
                .oversampling{0},
                .calibrate{0}
            };
            adc_read(dev, &sequence);
        }
        /* ADC3 */
        static constexpr uint8_t ch_adc3[NUM_CHANNELS_ADC3]{8};
        for (int i{0}; i < NUM_CHANNELS_ADC3; ++i) {
            // Only single channel supported
            adc_channel_cfg channel_cfg_adc3{
                .gain{ADC_GAIN_1},
                .reference{ADC_REF_INTERNAL},
                .acquisition_time{ADC_ACQ_TIME_DEFAULT},
                .channel_id{ch_adc3[i]},
                .differential{0}
            };
            adc_channel_setup(dev_adc3, &channel_cfg_adc3);
            adc_sequence sequence_adc3{
                .options{nullptr},
                .channels{BIT(ch_adc3[i])},
                .buffer{&buffer_adc3[i]},
                .buffer_size{sizeof buffer_adc3[i]},
                .resolution{12},
                .oversampling{0},
                .calibrate{0}
            };
            adc_read(dev_adc3, &sequence_adc3);
        }
    }
    const device *dev{nullptr}, *dev_adc3{nullptr};
    uint16_t buffer[NUM_CHANNELS], buffer_adc3[NUM_CHANNELS_ADC3];
} impl;

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

int32_t get(int index)
{
    return impl.get(index);
}

int32_t get_adc3(int index)
{
    return impl.get_adc3(index);
}

k_thread thread;

}

// vim: set expandtab shiftwidth=4:
