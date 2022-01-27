#include <device.h>
#include <devicetree.h>
#include <drivers/led_strip.h>
#include <logging/log.h>
#include <shell/shell.h>
#include <cstdlib>
#include "can_controller.hpp"
#include "led_controller.hpp"
#include "rosdiagnostic.hpp"

namespace lexxfirm::led_controller {

LOG_MODULE_REGISTER(led);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];

class led_message_receiver {
public:
    led_message_receiver() {
        message.pattern = msg::SHOWTIME;
        message.interrupt_ms = 0;
    }
    bool get_message(msg &output) {
        bool updated{false};
        if (msg message_new; k_msgq_get(&msgq, &message_new, K_MSEC(DELAY_MS)) == 0) {
            if (message.interrupt_ms > 0) {
                if (message_new.interrupt_ms > 0) {
                    if (is_new_pattern(message_new.pattern)) {
                        message = message_new;
                        updated = true;
                    }
                } else {
                    message_interrupted = message_new;
                }
            } else {
                if (is_new_pattern(message_new.pattern)) {
                    if (message_new.interrupt_ms > 0) {
                        LOG_INF("interrupted pattern %u %ums", message_new.pattern, message_new.interrupt_ms);
                        message_interrupted = message;
                        cycle_interrupted = k_cycle_get_32();
                    }
                    message = message_new;
                    updated = true;
                }
            }
        }
        if (message.interrupt_ms > 0) {
            uint32_t delta_ms{k_cyc_to_ms_near32(k_cycle_get_32() - cycle_interrupted)};
            if (delta_ms > message.interrupt_ms) {
                message = message_interrupted;
                message.interrupt_ms = 0;
                updated = true;
            }
        }
        output = message;
        return updated;
    }
    static constexpr uint32_t DELAY_MS{25};
private:
    bool is_new_pattern(uint32_t pattern) {
        return pattern == msg::RGB || message.pattern != pattern;
    }
    msg message, message_interrupted;
    uint32_t cycle_interrupted{0};
};

class led_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
        dev[LED_LEFT] = device_get_binding("WS2812_0");
        dev[LED_RIGHT] = device_get_binding("WS2812_1");
        if (!device_is_ready(dev[LED_LEFT]) || !device_is_ready(dev[LED_RIGHT]))
            return -1;
        fill(black);
        update();
        return 0;
    }
    void run() {
        if (!device_is_ready(dev[LED_LEFT]) || !device_is_ready(dev[LED_RIGHT]))
            return;
        while (true) {
            msg message;
            if (rec.get_message(message))
                counter = 0;
            poll(message);
        }
    }
    void run_error() const {
        rosdiagnostic::msg message{rosdiagnostic::msg::ERROR, "led", "no device"};
        while (true) {
            while (k_msgq_put(&rosdiagnostic::msgq, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&rosdiagnostic::msgq);
            k_msleep(5000);
        }
    }
private:
    void poll(const msg &message) {
        switch (message.pattern) {
        default:
        case msg::NONE:            fill(black); break;
        case msg::EMERGENCY_STOP:  fill_strobe(emergency_stop, 10, 50, 1000); break;
        case msg::AMR_MODE:        fill(amr_mode); break;
        case msg::AGV_MODE:        fill(agv_mode); break;
        case msg::MISSION_PAUSE:   fill(mission_pause); break;
        case msg::PATH_BLOCKED:    fill(path_blocked); break;
        case msg::MANUAL_DRIVE:    fill(manual_drive); break;
        case msg::CHARGING:        fill_rainbow(); break;
        case msg::WAITING_FOR_JOB: fill_fade(waiting_for_job); break;
        case msg::LEFT_WINKER:     fill_blink_sequence(sequence, LED_LEFT); break;
        case msg::RIGHT_WINKER:    fill_blink_sequence(sequence, LED_RIGHT); break;
        case msg::BOTH_WINKER:     fill_blink_sequence(sequence, LED_BOTH); break;
        case msg::MOVE_ACTUATOR:   fill_strobe(move_actuator, 10, 200, 200); break;
        case msg::CHARGE_LEVEL:    fill_charge_level(); break;
        case msg::SHOWTIME:        fill_knight_industries_two_thousand(); break;
        case msg::RGB:             fill(led_rgb{.r{message.rgb[0]}, .g{message.rgb[1]}, .b{message.rgb[2]}}); break;
        }
        update();
        ++counter;
    }
    void update() {
        led_strip_update_rgb(dev[LED_LEFT], pixeldata[LED_LEFT], PIXELS);
        led_strip_update_rgb(dev[LED_RIGHT], pixeldata[LED_RIGHT], PIXELS);
    }
    void fill(const led_rgb &color, uint32_t select = LED_BOTH) {
        if (select == LED_BOTH) {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = color;
        } else {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[select][i] = color;
        }
    }
    void fill_strobe(const led_rgb &color, uint32_t nstrobe, uint32_t strobedelay, uint32_t endpause) {
        static constexpr auto delay{led_message_receiver::DELAY_MS};
        if (counter < nstrobe * strobedelay / delay) {
            if ((counter % (strobedelay * 2 / delay)) == 0)
                fill(color);
            else if ((counter % (strobedelay * 2 / delay)) == strobedelay / delay)
                fill(black);
        } else if (counter == (nstrobe * strobedelay + endpause) / delay) {
            fill(black);
            counter = 0;
        }
    }
    void fill_rainbow(uint32_t select = LED_BOTH) {
        if (counter % 3 == 0)
            return;
        if (counter > 256 * 3)
            counter = 0;
        if (select == LED_BOTH) {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = wheel(((i * 256 / PIXELS) + counter / 3) & 255);
        } else {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[select][i] = wheel(((i * 256 / PIXELS) + counter / 3) & 255);
        }
    }
    void fill_fade(const led_rgb &color) {
        static constexpr uint32_t thres{130};
        if (counter >= thres * 2)
            counter = 0;
        int percent;
        if (counter < thres)
            percent = counter * 100 / thres;
        else
            percent = (thres * 2 - counter) * 100 / thres;
        fill(fader(color, percent));
    }
    void fill_blink_sequence(const led_rgb &color, uint32_t select = LED_BOTH) {
        uint32_t n{0};
        if (counter >= 8 && counter < 25) {
            n = (counter - 8) * 6;
            if (n > PIXELS)
                n = PIXELS;
        }
        if (select == LED_BOTH) {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = i < n ? color : black;
        } else {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[select][i] = i < n ? color : black;
            fill(black, select == LED_LEFT ? LED_RIGHT : LED_LEFT);
        }
        if (counter > 25)
            counter = 0;
    }
    void fill_toggle(const led_rgb &color) {
        static constexpr uint32_t thres{5};
        if (counter >= thres * 2)
            counter = 0;
        led_rgb c0, c1;
        if (counter < thres)
            c0 = color, c1 = black;
        else
            c0 = black, c1 = color;
        for (uint32_t i{0}; i < PIXELS; ++i) {
            if (i % 2 == 0)
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = c0;
            else
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = c1;
        }
    }
    void fill_charge_level() {
        static constexpr uint32_t thres{40};
        if (counter >= thres * 2)
            counter = 0;
        uint32_t head;
        if (counter >= thres)
            head = 0;
        else
            head = PIXELS - (PIXELS * counter / thres);
        static constexpr led_rgb color{.r{0xff}, .g{0x20}, .b{0x00}};
        uint32_t rsoc{can_controller::get_rsoc()};
        uint32_t n;
        if (rsoc < 100) {
            n = PIXELS - (PIXELS * rsoc / 100U);
            if (n < head)
                n = head;
        } else {
            n = head;
        }
        for (uint32_t i{0}; i < PIXELS; ++i)
            pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = i < n ? black : color;
    }
    void fill_knight_industries_two_thousand() {
        static constexpr int32_t width{20};
        if (counter >= (PIXELS + width) * 2)
            counter = 0;
        bool back{counter >= PIXELS + width};
        int32_t pos;
        if (back)
            pos = (PIXELS + width) * 2 - counter - width;
        else
            pos = counter;
        for (int32_t i{0}, end{PIXELS}; i < end; ++i) {
            bool no_color;
            if (back)
                no_color = i < pos || i > pos + width;
            else
                no_color = i < pos - width || i > pos;
            if (no_color) {
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = black;
            } else {
                static constexpr led_rgb color{.r{0x80}, .g{0x00}, .b{0x00}};
                int gain{(width - abs(pos - i)) * 100 / width};
                gain = gain * gain * gain / 100 / 100;
                led_rgb dimmed{fader(color, gain)};
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = dimmed;
            }
        }
    }
    led_rgb fader(const led_rgb &color, int percent) const {
        led_rgb color_;
        color_.r = color.r * percent / 100;
        color_.g = color.g * percent / 100;
        color_.b = color.b * percent / 100;
        return color_;
    }
    led_rgb wheel(uint32_t wheelpos) const {
        static constexpr uint32_t thres{256 / 3};
        led_rgb color;
        if (wheelpos < thres) {
            color.r = wheelpos * 3;
            color.g = 255 - wheelpos * 3;
            color.b = 0;
        } else if (wheelpos < thres * 2) {
            wheelpos -= thres;
            color.r = 255 - wheelpos * 3;
            color.g = 0;
            color.b = wheelpos * 3;
        } else {
            wheelpos -= thres * 2;
            color.r = 0;
            color.g = wheelpos * 3;
            color.b = 255 - wheelpos * 3;
        }
        return color;
    }
    led_message_receiver rec;
    static constexpr uint32_t PIXELS{DT_PROP(DT_NODELABEL(led_strip0), chain_length)};
    static constexpr uint32_t LED_LEFT{0}, LED_RIGHT{1}, LED_BOTH{2}, LED_NUM{2};
    const device *dev[LED_NUM]{nullptr, nullptr};
    led_rgb pixeldata[LED_NUM][PIXELS];
    uint32_t counter{0};
    static const led_rgb emergency_stop, amr_mode, agv_mode, mission_pause, path_blocked, manual_drive;
    static const led_rgb dock_mode, waiting_for_job, orange, sequence, move_actuator, black;
} impl;
const led_rgb led_controller_impl::emergency_stop {.r{0x80}, .g{0x00}, .b{0x00}};
const led_rgb led_controller_impl::amr_mode       {.r{0x00}, .g{0x80}, .b{0x80}};
const led_rgb led_controller_impl::agv_mode       {.r{0x45}, .g{0xff}, .b{0x00}};
const led_rgb led_controller_impl::mission_pause  {.r{0xff}, .g{0xff}, .b{0x00}};
const led_rgb led_controller_impl::path_blocked   {.r{0xe6}, .g{0x08}, .b{0xff}};
const led_rgb led_controller_impl::manual_drive   {.r{0xfe}, .g{0xf4}, .b{0xff}};
const led_rgb led_controller_impl::dock_mode      {.r{0x00}, .g{0x00}, .b{0xff}};
const led_rgb led_controller_impl::waiting_for_job{.r{0xff}, .g{0xff}, .b{0x00}};
const led_rgb led_controller_impl::orange         {.r{0xff}, .g{0xa5}, .b{0x00}};
const led_rgb led_controller_impl::sequence       {.r{0x90}, .g{0x20}, .b{0x00}};
const led_rgb led_controller_impl::move_actuator  {.r{0x45}, .g{0xff}, .b{0x00}};
const led_rgb led_controller_impl::black          {.r{0x00}, .g{0x00}, .b{0x00}};

int pattern(const shell *shell, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(shell, "Usage: %s %s <pattern>\n", argv[-1], argv[0]);
        return 1;
    }
    msg message{argv[1]};
    while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
        k_msgq_purge(&msgq);
    return 0;
}

int color(const shell *shell, size_t argc, char **argv)
{
    if (argc != 4) {
        shell_error(shell, "Usage: %s %s <r> <g> <b>\n", argv[-1], argv[0]);
        return 1;
    }
    msg message{msg::RGB, 0};
    message.rgb[0] = atoi(argv[1]);
    message.rgb[1] = atoi(argv[2]);
    message.rgb[2] = atoi(argv[3]);
    while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
        k_msgq_purge(&msgq);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub,
    SHELL_CMD(pattern, NULL, "LED pattern command", pattern),
    SHELL_CMD(color, NULL, "LED color command", color),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(led, &sub, "LED commands", NULL);

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
    impl.run_error();
}

k_thread thread;
k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
