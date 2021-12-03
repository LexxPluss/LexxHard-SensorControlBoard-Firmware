#pragma once

#include <zephyr.h>
#include <cstring>

struct msg_ros2led {
    msg_ros2led() : pattern(NONE), interrupt_ms(0) {}
    msg_ros2led(uint32_t pattern, uint32_t interrupt_ms) :
        pattern(pattern), interrupt_ms(interrupt_ms) {}
    msg_ros2led(const char *str) {
        if      (strcmp(str, "emergency_stop")  == 0) pattern = EMERGENCY_STOP;
        else if (strcmp(str, "amr_mode")        == 0) pattern = AMR_MODE;
        else if (strcmp(str, "agv_mode")        == 0) pattern = AGV_MODE;
        else if (strcmp(str, "mission_pause")   == 0) pattern = MISSION_PAUSE;
        else if (strcmp(str, "path_blocked")    == 0) pattern = PATH_BLOCKED;
        else if (strcmp(str, "manual_drive")    == 0) pattern = MANUAL_DRIVE;
        else if (strcmp(str, "charging")        == 0) pattern = CHARGING;
        else if (strcmp(str, "waiting_for_job") == 0) pattern = WAITING_FOR_JOB;
        else if (strcmp(str, "left_winker")     == 0) pattern = LEFT_WINKER;
        else if (strcmp(str, "right_winker")    == 0) pattern = RIGHT_WINKER;
        else if (strcmp(str, "both_winker")     == 0) pattern = BOTH_WINKER;
        else if (strcmp(str, "move_actuator")   == 0) pattern = MOVE_ACTUATOR;
        else if (strcmp(str, "charge_level")    == 0) pattern = CHARGE_LEVEL;
        else if (strcmp(str, "showtime")        == 0) pattern = SHOWTIME;
        else                                          pattern = NONE;
        interrupt_ms = 0;
    }
    uint32_t pattern{NONE}, interrupt_ms{0};
    uint8_t rgb[3]{0, 0, 0};
    static constexpr uint32_t NONE{0};
    static constexpr uint32_t EMERGENCY_STOP{1};
    static constexpr uint32_t AMR_MODE{2};
    static constexpr uint32_t AGV_MODE{3};
    static constexpr uint32_t MISSION_PAUSE{4};
    static constexpr uint32_t PATH_BLOCKED{5};
    static constexpr uint32_t MANUAL_DRIVE{6};
    static constexpr uint32_t CHARGING{10};
    static constexpr uint32_t WAITING_FOR_JOB{11};
    static constexpr uint32_t LEFT_WINKER{12};
    static constexpr uint32_t RIGHT_WINKER{13};
    static constexpr uint32_t BOTH_WINKER{14};
    static constexpr uint32_t MOVE_ACTUATOR{15};
    static constexpr uint32_t CHARGE_LEVEL{16};
    static constexpr uint32_t SHOWTIME{10000};
    static constexpr uint32_t RGB{10001};
} __attribute__((aligned(4)));

struct led_controller {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static k_thread thread;
};

extern k_msgq msgq_ros2led;

// vim: set expandtab shiftwidth=4:
