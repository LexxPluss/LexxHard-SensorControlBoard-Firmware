#include <device.h>
#include <drivers/adc.h>
#include <logging/log.h>
#include "adc_reader.hpp"
#include "rosdiagnostic.hpp"

namespace {

LOG_MODULE_REGISTER(adc);

class adc_reader_impl {
public:
    int init() {
        dev = device_get_binding("ADC_1");
        return dev == nullptr ? -1 : 0;
    }
    void run() {
        if (!device_is_ready(dev))
            return;
        while (true) {
            read_all_channels();
            k_msleep(20);
        }
    }
    void run_error() const {
        msg_rosdiag message{msg_rosdiag::ERROR, "adc", "no device"};
        while (true) {
            while (k_msgq_put(&msgq_rosdiag, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq_rosdiag);
            k_msleep(5000);
        }
    }
    int32_t get(int index) const {
        int32_t ref{adc_ref_internal(dev)};
        int32_t value{buffer[index]};
        if (ref > 0)
            adc_raw_to_millivolts(ref, ADC_GAIN_1, 12, &value);
        return value;
    }
private:
    void read_all_channels() {
        static constexpr uint8_t ch[NUM_CHANNELS]{0, 1, 10, 11, 12, 13};
        for (int i{0}; i < NUM_CHANNELS; ++i) {
            // Only single channel supported
            adc_channel_cfg channel_cfg{
                .gain{ADC_GAIN_1},
                .reference{ADC_REF_INTERNAL},
                .acquisition_time{ADC_ACQ_TIME_DEFAULT},
                .channel_id = ch[i],
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
    }
    static constexpr int NUM_CHANNELS{6};
    const device *dev{nullptr};
    uint16_t buffer[NUM_CHANNELS];
} impl;

}

void adc_reader::init()
{
    impl.init();
}

void adc_reader::run(void *p1, void *p2, void *p3)
{
    impl.run();
    impl.run_error();
}

int32_t adc_reader::get(int index)
{
    return impl.get(index);
}

k_thread adc_reader::thread;

// vim: set expandtab shiftwidth=4:
