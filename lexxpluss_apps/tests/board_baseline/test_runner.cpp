/*
 * Board Controller Baseline Test Harness
 *
 * Compiles the pre-Push-Mode board_controller.cpp (commit 8c94ed6, state_controller `impl`) and bmu_lipy041_decode.cpp
 * against Zephyr stubs and a virtual clock,
 * and drives it by calling impl.poll() while advancing virtual time. Expected values come from the old source
 * (line numbers in comments refer to build/old_src/board_controller.cpp).
 *
 * Every scenario runs in its own process (the binary re-executes itself with the scenario index). The old code keeps
 * state that init() does not reset (function-local `static int prev` in raw_switch::poll, switch debounce counters,
 * bmu/auto_charger data), so a fresh process is the only way to get an identical power-on condition per scenario.
 */

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <deque>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cstdlib>
#include <csignal>

#include <sys/wait.h>
#include <unistd.h>

// Setup stubs before including board_controller
#include "stubs/zephyr_stubs.hpp"
#include "stubs/lexxhard_stubs.hpp"

#define __aligned(x)
#define CLAMP(val, lo, hi) (((val) < (lo)) ? (lo) : (((val) > (hi)) ? (hi) : (val)))

// Forward declare types
struct uart_config {
    uint32_t baudrate;
    uint32_t parity;
    uint32_t stop_bits;
    uint32_t data_bits;
    uint32_t flow_ctrl;
};
struct shell {};

// UART simulation: bytes written by the charger heartbeat are captured, replies are injected via the registered
// IRQ callback (auto_charger::static_serial_read_callback).
namespace uart_sim {
    using Callback = void (*)(const device*, void*);
    inline Callback callback = nullptr;
    inline void* callback_user_data = nullptr;
    inline const device* registered_dev = nullptr;
    inline std::vector<uint8_t> tx_bytes;
    inline std::deque<uint8_t> rx_fifo;
}

inline int uart_configure(const device*, uart_config*) { return 0; }
inline int uart_irq_callback_user_data_set(const device* dev, uart_sim::Callback cb, void* user_data) {
    uart_sim::registered_dev = dev;
    uart_sim::callback = cb;
    uart_sim::callback_user_data = user_data;
    return 0;
}
inline void uart_irq_rx_enable(const device*) {}
inline int uart_irq_update(const device*) { return 1; }
inline int uart_irq_rx_ready(const device*) { return uart_sim::rx_fifo.empty() ? 0 : 1; }
inline int uart_fifo_read(const device*, uint8_t* buf, int len) {
    int count = 0;
    while (count < len && !uart_sim::rx_fifo.empty()) {
        buf[count++] = uart_sim::rx_fifo.front();
        uart_sim::rx_fifo.pop_front();
    }
    return count;
}
inline void uart_poll_out(const device*, uint8_t byte) { uart_sim::tx_bytes.push_back(byte); }
inline void shell_print(const shell*, const char*, ...) {}

// CAN frame struct
struct can_frame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};

// WDT stub with varargs
inline void wdt_feed(...) {}

// Override Zephyr macros that common.hpp expects
#define DT_NODELABEL(label) #label
#define GPIO_DT_SPEC_GET(node_id, gpios) zephyr_stubs::gpio_dt_spec_from_label(node_id)
// Every device resolves to one ready stub device (uart, iwdg); nullptr would make device_is_ready() fail and
// state_controller::init() return early (1293-1296).
inline const device* stubDeviceFromLabel(const char*) {
    static const device stub_device{};
    return &stub_device;
}
#define DEVICE_DT_GET(node_id) stubDeviceFromLabel(node_id)

// Add UART config constants
#define UART_CFG_PARITY_NONE 0
#define UART_CFG_STOP_BITS_1 0
#define UART_CFG_DATA_BITS_8 0
#define UART_CFG_FLOW_CTRL_NONE 0

// Make private members accessible for testing
#define private public

// Include the .cpp which will pull in all headers
#ifndef BC_SOURCE
#define BC_SOURCE "build/old_src/board_controller.cpp"
#endif
#include BC_SOURCE

#undef private

namespace {
    namespace bc = lexxhard::board_controller;

    // Set before setupInputs(): OSSD1/OSSD2 low, as on a machine without a lidar.
    bool lidar_absent = false;

    // Pin ids (see zephyr_stubs.hpp label map)
    constexpr int PIN_BP_LEFT = 3;
    constexpr int PIN_ES_LEFT = 6;
    constexpr int PIN_ES_OPTION_1 = 7;
    constexpr int PIN_ES_OPTION_2 = 8;
    constexpr int PIN_ES_RIGHT = 9;
    constexpr int PIN_KEY_SWITCH_LEFT = 14;
    constexpr int PIN_KEY_SWITCH_RIGHT = 15;
    constexpr int PIN_MC_DIN = 16;
    constexpr int PIN_PGOOD_24V = 17;
    constexpr int PIN_PGOOD_PERIPHERAL = 18;
    constexpr int PIN_PGOOD_WHEEL_LEFT = 19;
    constexpr int PIN_PGOOD_WHEEL_RIGHT = 20;
    constexpr int PIN_PS_SW_IN = 22;
    constexpr int PIN_RESUME_SW_IN = 24;
    constexpr int PIN_OSSD1 = 25;
    constexpr int PIN_OSSD2 = 26;
    constexpr int PIN_V_WHEEL = 30;

    constexpr int STEP_MS = 20;  // matches state_controller::run() k_msleep(20)

    struct Transition {
        int64_t poll_time;  // virtual time when the poll that caused the transition started
        POWER_STATE from;
        POWER_STATE to;
    };

    std::vector<Transition> transitions;
    uint8_t bmu_rsoc = 50;
    bool ros_wheel_power_off = false;
    bool ros_power_off = false;

    std::string stateName(const POWER_STATE s) {
        switch (s) {
            case POWER_STATE::OFF: return "OFF";
            case POWER_STATE::WAIT_SW: return "WAIT_SW";
            case POWER_STATE::POST: return "POST";
            case POWER_STATE::STANDBY: return "STANDBY";
            case POWER_STATE::NORMAL: return "NORMAL";
            case POWER_STATE::AUTO_CHARGE: return "AUTO_CHARGE";
            case POWER_STATE::MANUAL_CHARGE: return "MANUAL_CHARGE";
            case POWER_STATE::LOCKDOWN: return "LOCKDOWN";
            case POWER_STATE::TIMEROFF: return "TIMEROFF";
            case POWER_STATE::SUSPEND: return "SUSPEND";
            case POWER_STATE::RESUME_WAIT: return "RESUME_WAIT";
            case POWER_STATE::OFF_WAIT: return "OFF_WAIT";
            default: return "UNKNOWN";
        }
    }

    // All inputs at their inactive level, key switch in RUNNING. Polarities per the old source:
    //   key switch RUNNING = left 1 / right 0 (304-307, 319), pgood 0 = OK (1029-1032), emergency switch asserted
    //   when pin == 1 (411), bumper asserted when bp_left == 0 (378), manual charger plugged when mc_din == 0 (541),
    //   safety lidar asserted when both ossd == 0 (1229-1243), power switch pressed == 0 (118, 163).
    void setupInputs() {
        zephyr_stubs::virtual_time_ms = 0;
        zephyr_stubs::pin_write_log.clear();
        zephyr_stubs::pin_state.clear();
        ros_wheel_power_off = false;
        ros_power_off = false;

        // The LED command queue is written with a retry loop (1744-1745); it needs capacity or it never terminates.
        static char led_msgq_buffer[8 * sizeof(lexxhard::led_controller::msg)];
        k_msgq_init(&lexxhard::led_controller::msgq, led_msgq_buffer, sizeof(lexxhard::led_controller::msg), 8);

        zephyr_stubs::set_pin_input(PIN_KEY_SWITCH_LEFT, 1);
        zephyr_stubs::set_pin_input(PIN_KEY_SWITCH_RIGHT, 0);
        zephyr_stubs::set_pin_input(PIN_PGOOD_24V, 0);
        zephyr_stubs::set_pin_input(PIN_PGOOD_PERIPHERAL, 0);
        zephyr_stubs::set_pin_input(PIN_PGOOD_WHEEL_LEFT, 0);
        zephyr_stubs::set_pin_input(PIN_PGOOD_WHEEL_RIGHT, 0);
        zephyr_stubs::set_pin_input(PIN_ES_LEFT, 0);
        zephyr_stubs::set_pin_input(PIN_ES_RIGHT, 0);
        zephyr_stubs::set_pin_input(PIN_ES_OPTION_1, 0);
        zephyr_stubs::set_pin_input(PIN_ES_OPTION_2, 0);
        zephyr_stubs::set_pin_input(PIN_BP_LEFT, 1);
        zephyr_stubs::set_pin_input(PIN_MC_DIN, 1);
        zephyr_stubs::set_pin_input(PIN_RESUME_SW_IN, 1);
        zephyr_stubs::set_pin_input(PIN_OSSD1, lidar_absent ? 0 : 1);
        zephyr_stubs::set_pin_input(PIN_OSSD2, lidar_absent ? 0 : 1);
        zephyr_stubs::set_pin_input(PIN_PS_SW_IN, 1);
    }

    // mainboard_driver input (1150-1151, 1180-1204): not emergency, not power off, heartbeat alive.
    void feedHeartbeat() {
        bc::msg_rcv_pb msg = {};
        msg.ros_emergency_stop = false;
        msg.ros_power_off = ros_power_off;
        msg.ros_heartbeat_timeout = false;
        msg.ros_wheel_power_off = ros_wheel_power_off;
        msg.ros_lockdown = false;
        k_msgq_put(&bc::msgq_board_pb_rx, &msg, K_NO_WAIT);  // a full queue (-1) is fine: older messages are equal
    }

    // Bit values injected into the BMU frames; all zero is the healthy state (bmu_lipy041_decode.cpp is_ok, 186-196).
    struct BmuFault {
        uint8_t fail_status1 = 0;   // 0x100 data[0]
        uint8_t fail_status2 = 0;   // 0x101 data[6]
        uint8_t leader_alarm1 = 0;  // 0x113 data[4]
        uint8_t leader_alarm2 = 0;  // 0x113 data[5]
        uint8_t fail_status3 = 0;   // 0x113 data[6]
        uint8_t dlc100 = 8;         // decode_0x100 rejects dlc != 8 (bmu_lipy041_decode.cpp 36)
    };
    BmuFault bmu_fault;

    // bmu_controller input (900-901, 946-950): handle_can() -> decode_frame_power_sequence(). All frames need dlc 8
    // (decode.cpp 36, 49, 104). is_ok() (904-913) is the AND of five masked fields, true for all-zero fields;
    // is_chargable() (936-938) needs !full_charge (fail_status1 bit 6, decode.cpp 199) and rsoc_min < 95
    // (decode.cpp 207).
    void feedBmuFrames() {
        can_frame frame100{};
        frame100.id = 0x100;
        frame100.dlc = bmu_fault.dlc100;
        frame100.data[0] = bmu_fault.fail_status1;
        frame100.data[2] = 0x50;  // asoc_min
        frame100.data[3] = bmu_rsoc;
        k_msgq_put(&bc::msgq_can_bmu_pb, &frame100, K_NO_WAIT);

        can_frame frame101{};
        frame101.id = 0x101;
        frame101.dlc = 8;
        frame101.data[6] = bmu_fault.fail_status2;
        k_msgq_put(&bc::msgq_can_bmu_pb, &frame101, K_NO_WAIT);

        can_frame frame113{};
        frame113.id = 0x113;
        frame113.dlc = 8;
        frame113.data[4] = bmu_fault.leader_alarm1;
        frame113.data[5] = bmu_fault.leader_alarm2;
        frame113.data[6] = bmu_fault.fail_status3;
        k_msgq_put(&bc::msgq_can_bmu_pb, &frame113, K_NO_WAIT);
    }

    // Charger side of the IrDA link: echo every heartbeat frame the SCB sent (816-829) back with the same counter,
    // which makes auto_charger::serial_read() refresh start_time (852-854) and keeps is_docked() true (702).
    void serviceCharger() {
        static bc::serial_message decoder;
        const std::vector<uint8_t> tx = uart_sim::tx_bytes;
        uart_sim::tx_bytes.clear();
        for (const uint8_t byte : tx) {
            if (!decoder.decode(byte)) {
                continue;
            }
            uint8_t param[3];
            if (decoder.get_command(param) != bc::serial_message::HEARTBEAT) {
                continue;
            }
            uint8_t reply[8];
            bc::serial_message::compose(reply, bc::serial_message::HEARTBEAT, param);
            for (const uint8_t reply_byte : reply) {
                uart_sim::rx_fifo.push_back(reply_byte);
            }
            if (uart_sim::callback != nullptr) {
                uart_sim::callback(uart_sim::registered_dev, uart_sim::callback_user_data);
            }
        }
    }

    // One iteration of state_controller::run(): inputs, poll(), sleep. Records state transitions.
    void stepOnce() {
        feedHeartbeat();
        feedBmuFrames();
        serviceCharger();
        const POWER_STATE before = bc::impl.state;
        const int64_t poll_time = zephyr_stubs::virtual_time_ms;
        bc::impl.poll();
        if (bc::impl.state != before) {
            transitions.push_back({poll_time, before, bc::impl.state});
        }
        k_msleep(STEP_MS);
    }

    void stepUntil(const int64_t target_ms) {
        while (zephyr_stubs::virtual_time_ms < target_ms) {
            stepOnce();
        }
    }

    // Press the power switch while in WAIT_SW (needs > PWS_PUSHED_MS = 3000 ms after the pin went low, 169),
    // release it afterwards (POST -> STANDBY needs RELEASED, 1459).
    void driveBootSwitch() {
        zephyr_stubs::set_pin_input(PIN_PS_SW_IN, bc::impl.state == POWER_STATE::WAIT_SW ? 0 : 1);
    }

    bool bootToNormal() {
        bc::impl.init();
        constexpr int64_t BOOT_TIMEOUT_MS = 60000;
        while (bc::impl.state != POWER_STATE::NORMAL && zephyr_stubs::virtual_time_ms < BOOT_TIMEOUT_MS) {
            driveBootSwitch();
            stepOnce();
        }
        return bc::impl.state == POWER_STATE::NORMAL;
    }

    int64_t transitionTime(const POWER_STATE to) {
        for (const Transition& t : transitions) {
            if (t.to == to) {
                return t.poll_time;
            }
        }
        return -1;
    }

    std::vector<zephyr_stubs::PinWrite> pinWrites(const int pin, const int64_t from_time = 0) {
        std::vector<zephyr_stubs::PinWrite> result;
        for (const auto& w : zephyr_stubs::pin_write_log) {
            if (w.pin == pin && w.time >= from_time) {
                result.push_back(w);
            }
        }
        return result;
    }

    void dumpDiagnostics() {
        auto& impl = bc::impl;
        std::cout << "  DIAG t=" << zephyr_stubs::virtual_time_ms << " state=" << stateName(impl.state)
                  << " psw=" << static_cast<int>(impl.psw.get_state())
                  << " raw_sw=" << static_cast<int>(impl.psw.raw_sw.get_state())
                  << " ksw=" << static_cast<int>(impl.ksw.get_state()) << " ksw.is_off=" << impl.ksw.is_off()
                  << " ksw.is_running=" << impl.ksw.is_running() << " should_turn_off=" << impl.should_turn_off()
                  << " bmu.is_ok=" << impl.bmu.is_ok() << " mbd.is_ready=" << impl.mbd.is_ready()
                  << " esw=" << impl.esw.is_asserted() << " sl=" << impl.sl.is_asserted() << std::endl;
        std::cout << "  DIAG ac: connected=" << impl.ac.is_connected() << " overheat=" << impl.ac.is_overheat()
                  << " temp0=" << impl.ac.connector_temp[0] << " check_count=" << impl.ac.connect_check_count
                  << " hb_start=" << impl.ac.start_time << " hb_counter=" << static_cast<int>(impl.ac.heartbeat_counter)
                  << " docked=" << impl.ac.is_docked() << std::endl;
        std::cout << "  DIAG pins:";
        for (const auto& [pin, level] : zephyr_stubs::pin_state) {
            std::cout << " " << pin << "=" << level;
        }
        std::cout << std::endl;
        for (const Transition& t : transitions) {
            std::cout << "  DIAG transition t=" << t.poll_time << " " << stateName(t.from) << " -> "
                      << stateName(t.to) << std::endl;
        }
    }

    // Exit status of a scenario child in a new-source build: observed behavior differs from the baseline expectation.
    constexpr int EXIT_DIFF = 10;
    // Exit status of a Push specification scenario that failed.
    constexpr int EXIT_SPEC_FAIL = 11;
    constexpr int LAST_BASELINE_SCENARIO = 13;
    constexpr int FIRST_PUSH_SCENARIO = 14;

    std::string transitionList() {
        std::ostringstream os;
        for (const Transition& t : transitions) {
            os << " [t=" << t.poll_time << " " << stateName(t.from) << "->" << stateName(t.to) << "]";
        }
        return transitions.empty() ? " (none)" : os.str();
    }

    std::string vWheelList() {
        std::ostringstream os;
        for (const auto& w : pinWrites(PIN_V_WHEEL)) {
            os << " [t=" << w.time << " level=" << w.level << "]";
        }
        return os.str().empty() ? " (none)" : os.str();
    }

    class Checker {
    public:
        void expect(const bool condition, const std::string& message) {
            if (!condition) {
                failures.push_back(message);
            }
        }
        void expectEq(const int64_t actual, const int64_t expected, const std::string& what) {
            if (actual != expected) {
                std::ostringstream os;
                os << what << ": expected " << expected << ", got " << actual;
                failures.push_back(os.str());
            }
        }
        // Baseline expectation as text (transition list, v_wheel writes), printed on a DIFF in new-source builds.
        void setExpected(const std::string& text) { expected_text = text; }
        bool finish(const std::string& name) {
#ifdef BC_NEW_BUILD
            if (failures.empty()) {
                std::cout << "SAME - " << name << std::endl;
                return true;
            }
            std::cout << "DIFF - " << name << std::endl;
            for (const std::string& f : failures) {
                std::cout << "  - " << f << std::endl;
            }
            std::cout << "  observed transitions:" << transitionList() << std::endl;
            std::cout << "  observed v_wheel writes:" << vWheelList() << std::endl;
            std::cout << "  expected: " << expected_text << std::endl;
            return false;
#else
            if (failures.empty()) {
                std::cout << "PASS - " << name << std::endl;
                return true;
            }
            std::cout << "FAIL - " << name << std::endl;
            for (const std::string& f : failures) {
                std::cout << "  - " << f << std::endl;
            }
            dumpDiagnostics();
            return false;
#endif
        }
        bool finishSpec(const std::string& name) {
            if (failures.empty()) {
                std::cout << "SPEC PASS - " << name << std::endl;
                return true;
            }
            std::cout << "SPEC FAIL - " << name << std::endl;
            for (const std::string& f : failures) {
                std::cout << "  - " << f << std::endl;
            }
            std::cout << "  observed transitions:" << transitionList() << std::endl;
            std::cout << "  observed v_wheel writes:" << vWheelList() << std::endl;
            std::cout << "  expected: " << expected_text << std::endl;
            return false;
        }
    private:
        std::vector<std::string> failures;
        std::string expected_text = "(see scenario source)";
    };

    // Last v_wheel level written at or before t, -1 if none.
    int levelAt(const int64_t t) {
        int level = -1;
        for (const auto& w : zephyr_stubs::pin_write_log) {
            if (w.pin == PIN_V_WHEEL && w.time <= t) {
                level = w.level;
            }
        }
        return level;
    }

    std::vector<int> vWheelWritesSince(const size_t log_index) {
        std::vector<int> levels;
        for (size_t i = log_index; i < zephyr_stubs::pin_write_log.size(); ++i) {
            if (zephyr_stubs::pin_write_log[i].pin == PIN_V_WHEEL) {
                levels.push_back(zephyr_stubs::pin_write_log[i].level);
            }
        }
        return levels;
    }

    // Debounce takes about 7 polls either way; callers wait with stepUntilState*.
    void pressEsw(const bool asserted) { zephyr_stubs::set_pin_input(PIN_ES_LEFT, asserted ? 1 : 0); }

    bool stepUntilState(const POWER_STATE target, const int max_polls) {
        for (int i = 0; i < max_polls && bc::impl.state != target; ++i) {
            stepOnce();
        }
        return bc::impl.state == target;
    }

    bool stepUntilStateLeaves(const POWER_STATE source, const int max_polls) {
        for (int i = 0; i < max_polls && bc::impl.state == source; ++i) {
            stepOnce();
        }
        return bc::impl.state != source;
    }

    void stepPolls(const int polls) {
        for (int i = 0; i < polls; ++i) {
            stepOnce();
        }
    }

    std::string levelsText(const std::vector<int>& levels) {
        std::ostringstream os;
        os << "[";
        for (size_t i = 0; i < levels.size(); ++i) {
            os << (i == 0 ? "" : ",") << levels[i];
        }
        os << "]";
        return os.str();
    }

    void expectLevels(Checker& c, const std::vector<int>& actual, const std::vector<int>& expected,
                      const std::string& what) {
        c.expect(actual == expected, what + ": expected " + levelsText(expected) + ", got " + levelsText(actual));
    }

    void setMaintenanceKey() {
        zephyr_stubs::set_pin_input(PIN_KEY_SWITCH_LEFT, 1);
        zephyr_stubs::set_pin_input(PIN_KEY_SWITCH_RIGHT, 1);
    }

    // NORMAL reached and 500 ms elapsed.
    bool bootAndSettle(Checker& c) {
        if (!bootToNormal()) {
            c.expect(false, "NORMAL not reached");
            return false;
        }
        stepUntil(zephyr_stubs::virtual_time_ms + 500);
        return true;
    }

    // wp=1 held in NORMAL, then the emergency switch until SUSPEND, with 5 polls of settling after each step.
    bool holdWpThenSuspend(Checker& c) {
        if (!bootAndSettle(c)) {
            return false;
        }
        ros_wheel_power_off = true;
        stepPolls(5);
        pressEsw(true);
        c.expect(stepUntilState(POWER_STATE::SUSPEND, 40), "SUSPEND not reached");
        stepPolls(5);
        return true;
    }

    // Scenario 1: OFF -> WAIT_SW -> POST -> STANDBY -> NORMAL and the v_wheel writes made on the way.
    bool scenario1() {
        Checker c;
        c.setExpected("transitions OFF->WAIT_SW->POST->STANDBY->NORMAL; v_wheel writes level 0 @POST, 1 @STANDBY, 1 "
                      "@STANDBY+6000, 1 @NORMAL (4 writes)");
        setupInputs();
        bc::impl.init();
        c.expect(bc::impl.state == POWER_STATE::OFF, "state after init() must be OFF (1962)");
        const bool reached = bootToNormal();
        c.expect(reached, "NORMAL not reached");

        // OFF->WAIT_SW (1440-1441), WAIT_SW->POST (1451-1453), POST->STANDBY (1459-1461), STANDBY->NORMAL (1485-1487)
        const std::vector<POWER_STATE> expected_to{POWER_STATE::WAIT_SW, POWER_STATE::POST, POWER_STATE::STANDBY,
                                                   POWER_STATE::NORMAL};
        std::vector<POWER_STATE> actual_to;
        for (const Transition& t : transitions) {
            actual_to.push_back(t.to);
        }
        c.expect(actual_to == expected_to, "power state sequence differs from OFF,WAIT_SW,POST,STANDBY,NORMAL");

        // v_wheel writes: POST enter 0 (1762); STANDBY enter dcdc.set_enable(true) writes 1 first (969), then after
        // 2 x k_msleep(3000) (970, 976) writes bat_out_state = ksw.is_running() = 1 (1730, 1780); NORMAL enter writes
        // bat_out_state = 1 (1795). wheel_relay_control() writes nothing (last_wheel_poweroff == false, running).
        const int64_t t_post = transitionTime(POWER_STATE::POST);
        const int64_t t_standby = transitionTime(POWER_STATE::STANDBY);
        const int64_t t_normal = transitionTime(POWER_STATE::NORMAL);
        const auto writes = pinWrites(PIN_V_WHEEL);
        c.expectEq(static_cast<int64_t>(writes.size()), 4, "number of v_wheel writes");
        if (writes.size() == 4) {
            c.expectEq(writes[0].level, 0, "v_wheel[0] level (POST enter, 1762)");
            c.expectEq(writes[0].time, t_post, "v_wheel[0] time");
            c.expectEq(writes[1].level, 1, "v_wheel[1] level (dcdc enable, 969)");
            c.expectEq(writes[1].time, t_standby, "v_wheel[1] time");
            c.expectEq(writes[2].level, 1, "v_wheel[2] level (STANDBY enter, 1780)");
            c.expectEq(writes[2].time, t_standby + 6000, "v_wheel[2] time (two 3000 ms sleeps, 961/967)");
            c.expectEq(writes[3].level, 1, "v_wheel[3] level (NORMAL enter, 1795)");
            c.expectEq(writes[3].time, t_normal, "v_wheel[3] time");
        }
        return c.finish("Boot POST -> STANDBY -> NORMAL");
    }

    // Scenario 2: NORMAL -> SUSPEND by safety lidar (1516-1518) and the v_wheel write on entering SUSPEND.
    bool scenario2() {
        Checker c;
        c.setExpected("last transition NORMAL->SUSPEND one poll after lidar assert; exactly 1 v_wheel write level 1 at "
                      "the assert time");
        setupInputs();
        if (!bootToNormal()) {
            c.expect(false, "NORMAL not reached");
            return c.finish("NORMAL -> SUSPEND by safety lidar");
        }
        stepUntil(zephyr_stubs::virtual_time_ms + 500);
        c.expect(bc::impl.state == POWER_STATE::NORMAL, "must stay NORMAL while lidar is not asserted");

        const size_t log_before = zephyr_stubs::pin_write_log.size();
        const int64_t t_assert = zephyr_stubs::virtual_time_ms;
        zephyr_stubs::set_pin_input(PIN_OSSD1, 0);  // asserted when both are low (1243)
        zephyr_stubs::set_pin_input(PIN_OSSD2, 0);
        stepOnce();
        c.expect(bc::impl.state == POWER_STATE::SUSPEND, "state must be SUSPEND after one poll (1516-1518)");
        c.expect(!transitions.empty() && transitions.back().from == POWER_STATE::NORMAL &&
                 transitions.back().to == POWER_STATE::SUSPEND, "last transition must be NORMAL -> SUSPEND");

        // SUSPEND enter writes bat_out_state = mbd.is_wheel_poweroff() || ksw.is_running() = 0 || 1 = 1 (1730, 1809)
        std::vector<zephyr_stubs::PinWrite> writes;
        for (size_t i = log_before; i < zephyr_stubs::pin_write_log.size(); ++i) {
            if (zephyr_stubs::pin_write_log[i].pin == PIN_V_WHEEL) {
                writes.push_back(zephyr_stubs::pin_write_log[i]);
            }
        }
        c.expectEq(static_cast<int64_t>(writes.size()), 1, "number of v_wheel writes on SUSPEND entry");
        if (writes.size() == 1) {
            c.expectEq(writes[0].level, 1, "v_wheel level on SUSPEND entry (1809)");
            c.expectEq(writes[0].time, t_assert, "v_wheel write time");
        }
        return c.finish("NORMAL -> SUSPEND by safety lidar");
    }

    // Scenario 3: charge_guard (10 s one-shot started on entering NORMAL, 1797-1799) blocks NORMAL -> AUTO_CHARGE.
    bool scenario3() {
        Checker c;
        c.setExpected("NORMAL until NORMAL+10000 ms (charge_guard), then NORMAL->AUTO_CHARGE within 100 ms");
        setupInputs();
        // Charger connector: connector_v > CONNECT_THRES_VOLTAGE (744, 887) for 100 polls (745, 885) gives
        // is_connected(); connector_v > 0.9 * CHARGING_VOLTAGE (718, 886) gives is_charger_ready().
        lexxhard::adc_reader::stub_adc3_mv = 2900;
        bmu_rsoc = 50;  // is_chargable(): rsoc < 95 (936-938, 1526)
        if (!bootToNormal()) {
            c.expect(false, "NORMAL not reached");
            return c.finish("charge_guard blocks AUTO_CHARGE for 10 s");
        }
        const int64_t t_normal = transitionTime(POWER_STATE::NORMAL);
        c.expectEq(bc::impl.charge_guard_timeout.start_time, t_normal + 10000, "charge_guard expiry (1799)");

        stepUntil(t_normal + 9900);
        stepOnce();  // poll once at ~9.9 s
        c.expect(bc::impl.state == POWER_STATE::NORMAL, "must still be NORMAL at 9.9 s");
        c.expect(bc::impl.charge_guard_asserted, "charge_guard_asserted must still be true at 9.9 s (1797)");
        c.expect(bc::impl.ac.is_docked(), "ac.is_docked() must be true (condition input, 1526)");
        c.expect(bc::impl.bmu.is_chargable(), "bmu.is_chargable() must be true (condition input, 1526)");
        c.expect(bc::impl.ac.is_charger_ready(), "ac.is_charger_ready() must be true (condition input, 1527)");

        stepUntil(t_normal + 10100);
        c.expect(bc::impl.state == POWER_STATE::AUTO_CHARGE, "must be AUTO_CHARGE after 10.1 s (1526-1529)");
        const int64_t t_auto = transitionTime(POWER_STATE::AUTO_CHARGE);
        c.expect(t_auto >= t_normal + 10000 && t_auto <= t_normal + 10100,
                 "AUTO_CHARGE must be entered between 10.0 s and 10.1 s after NORMAL entry");
        return c.finish("charge_guard blocks AUTO_CHARGE for 10 s");
    }

    // Scenario 4: OFF_WAIT -> OFF after 60 s (timer_shutdown reset on entering OFF_WAIT 1859, check 1672-1673).
    bool scenario4() {
        Checker c;
        c.setExpected("NORMAL->OFF_WAIT after key switch off, OFF_WAIT->OFF in (60000, 60100] ms after OFF_WAIT entry");
        setupInputs();
        if (!bootToNormal()) {
            c.expect(false, "NORMAL not reached");
            return c.finish("OFF_WAIT -> OFF after 60 s");
        }
        stepUntil(zephyr_stubs::virtual_time_ms + 200);
        // Key switch to LEFT (left 0 / right 1, 306-311): is_off() = !running && !maintenance (328-330) makes
        // should_turn_off() true, NORMAL -> OFF_WAIT (1493-1494).
        zephyr_stubs::set_pin_input(PIN_KEY_SWITCH_LEFT, 0);
        zephyr_stubs::set_pin_input(PIN_KEY_SWITCH_RIGHT, 1);
        while (bc::impl.state == POWER_STATE::NORMAL && zephyr_stubs::virtual_time_ms < 100000) {
            stepOnce();
        }
        c.expect(bc::impl.state == POWER_STATE::OFF_WAIT, "NORMAL -> OFF_WAIT expected after key switch off");
        const int64_t t_wait = transitionTime(POWER_STATE::OFF_WAIT);
        c.expectEq(bc::impl.timer_shutdown, t_wait, "timer_shutdown reset on OFF_WAIT entry (1859)");

        stepUntil(t_wait + 59900);
        stepOnce();  // poll once at ~59.9 s
        c.expect(bc::impl.state == POWER_STATE::OFF_WAIT, "must still be OFF_WAIT at 59.9 s");

        while (bc::impl.state == POWER_STATE::OFF_WAIT && zephyr_stubs::virtual_time_ms < t_wait + 70000) {
            stepOnce();
        }
        c.expect(bc::impl.state == POWER_STATE::OFF, "must be OFF after 60 s (1672-1673)");
        const int64_t t_off = transitionTime(POWER_STATE::OFF);
        // condition is strictly > 60000 (1672); the dcdc power-off sleeps inside that poll are after poll_time
        c.expect(t_off > t_wait + 60000 && t_off <= t_wait + 60100,
                 "OFF_WAIT -> OFF poll must start in (60.0 s, 60.1 s] after entering OFF_WAIT");
        return c.finish("OFF_WAIT -> OFF after 60 s");
    }

    // Scenario 5: BMU health rule of 8c94ed6. bmu.is_ok() (904-913) is an AND over fail_status1/2/3 and leader_alarm1/2
    // (decode.cpp 186-196); the 19ddd3b rule was an OR (one abnormal field stayed healthy). POST goes to
    // STANDBY when is_ok() (1459) and to OFF once is_ok() is false for more than 3000 ms after POST entry (1462-1464,
    // timer_post set at 1763). A frame with dlc != 8 is not decoded (decode.cpp 36, board_controller.cpp 946-950), so
    // the struct keeps its default fail_status1 = 0xff (decode.hpp 50) and is_ok() stays false.
    // Variant masks (decode.hpp 33-37): fail_status1 0b10111111 (bit 6 excluded), the others as written below.
    struct Scenario5Variant {
        const char* name;
        BmuFault fault;
        bool expect_standby;
    };

    std::vector<Scenario5Variant> scenario5Variants() {
        std::vector<Scenario5Variant> v;
        BmuFault f;
        f = {}; f.fail_status1 = 0x01; v.push_back({"fail_status1 bit0 only", f, false});
        f = {}; f.fail_status2 = 0x01; v.push_back({"fail_status2 bit0 only", f, false});
        f = {}; f.leader_alarm1 = 0x01; v.push_back({"leader_alarm1 bit0 only", f, false});
        f = {}; f.leader_alarm2 = 0x01; v.push_back({"leader_alarm2 bit0 only", f, false});
        f = {}; f.fail_status3 = 0x01; v.push_back({"fail_status3 bit0 only", f, false});
        f = {}; f.dlc100 = 7; v.push_back({"0x100 dlc=7", f, false});
        // bit 6 of fail_status1 is outside FAIL_STATUS1_ABNORMAL_MASK, so it must stay healthy
        f = {}; f.fail_status1 = 0x40; v.push_back({"fail_status1 bit6 only (masked)", f, true});
        // reserved leader_alarm1 bits 3-7 are outside LEADER_ALARM1_ABNORMAL_MASK (0b00000111)
        f = {}; f.leader_alarm1 = 0xf8; v.push_back({"leader_alarm1 reserved bits only (masked)", f, true});
        return v;
    }

    bool scenario5Variant(const size_t variant_index) {
        const Scenario5Variant variant = scenario5Variants().at(variant_index);
        Checker c;
        c.setExpected(variant.expect_standby
                          ? "POST->STANDBY"
                          : "POST->OFF in (3000, 3000 + 20] ms after POST entry, STANDBY never reached");
        setupInputs();
        bmu_fault = variant.fault;
        bc::impl.init();
        constexpr int64_t TIMEOUT_MS = 20000;
        // run until STANDBY, or until POST has fallen back to OFF
        while (zephyr_stubs::virtual_time_ms < TIMEOUT_MS && bc::impl.state != POWER_STATE::STANDBY &&
               !(bc::impl.state == POWER_STATE::OFF && transitionTime(POWER_STATE::POST) >= 0)) {
            driveBootSwitch();
            stepOnce();
        }
        const int64_t t_post = transitionTime(POWER_STATE::POST);
        c.expect(t_post >= 0, "POST must be reached (1451-1453)");
        if (variant.expect_standby) {
            c.expect(bc::impl.state == POWER_STATE::STANDBY, "must reach STANDBY (1459-1461)");
        } else {
            c.expect(bc::impl.state == POWER_STATE::OFF, "must end in OFF (1462-1464)");
            c.expect(transitionTime(POWER_STATE::STANDBY) < 0, "must never reach STANDBY (1459)");
            const int64_t t_off = transitionTime(POWER_STATE::OFF);
            // first poll with k_uptime_get() - timer_post > 3000 (1462), polls are STEP_MS apart
            c.expect(t_post >= 0 && t_off > t_post + 3000 && t_off <= t_post + 3000 + STEP_MS,
                     "POST -> OFF poll must start in (3.0 s, 3.0 s + one step] after POST entry");
        }
        return c.finish(std::string("BMU rule (AND): ") + variant.name);
    }

    // Regression guards S6-S13: behavior of the old code (8c94ed6), defects included.

    // S6 (D1a): wp=1 at boot, running.
    bool scenario6() {
        Checker c;
        c.setExpected("v_wheel levels [0,1,1,0,1]; no further write in 20 polls after NORMAL; final level 1");
        setupInputs();
        ros_wheel_power_off = true;
        c.expect(bootToNormal(), "NORMAL not reached");
        expectLevels(c, vWheelWritesSince(0), {0, 1, 1, 0, 1}, "v_wheel levels at boot");
        const size_t mark = zephyr_stubs::pin_write_log.size();
        stepPolls(20);
        c.expectEq(static_cast<int64_t>(vWheelWritesSince(mark).size()), 0, "v_wheel writes after NORMAL");
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 1, "final v_wheel level");
        return c.finish("D1a boot with wp=1, running");
    }

    // S7 (D1b): NORMAL -> SUSPEND by ros_power_off while wp=1.
    bool scenario7() {
        Checker c;
        c.setExpected("NORMAL->SUSPEND; exactly one v_wheel write, level 1, after ros_power_off");
        setupInputs();
        if (!bootAndSettle(c)) {
            return c.finish("D1b NORMAL -> SUSPEND with wp=1 by ros_power_off");
        }
        ros_wheel_power_off = true;
        stepPolls(5);
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 0, "level before ros_power_off");
        const size_t mark = zephyr_stubs::pin_write_log.size();
        ros_power_off = true;
        stepOnce();
        c.expect(bc::impl.state == POWER_STATE::SUSPEND, "state must be SUSPEND");
        c.expect(!transitions.empty() && transitions.back().from == POWER_STATE::NORMAL &&
                 transitions.back().to == POWER_STATE::SUSPEND, "last transition must be NORMAL -> SUSPEND");
        expectLevels(c, vWheelWritesSince(mark), {1}, "v_wheel writes on SUSPEND entry");
        return c.finish("D1b NORMAL -> SUSPEND with wp=1 by ros_power_off");
    }

    // S8 (D2): maintenance boot, dcdc write 1 lasts until bat_out_state=0 at STANDBY+6000.
    bool scenario8() {
        Checker c;
        c.setExpected("levelAt(STANDBY+1)=1, levelAt(STANDBY+5990)=1, levelAt(STANDBY+6000)=0");
        setupInputs();
        setMaintenanceKey();
        c.expect(bootToNormal(), "NORMAL not reached");
        const int64_t t_st = transitionTime(POWER_STATE::STANDBY);
        c.expectEq(levelAt(t_st + 1), 1, "level at STANDBY+1");
        c.expectEq(levelAt(t_st + 5990), 1, "level at STANDBY+5990");
        c.expectEq(levelAt(t_st + 6000), 0, "level at STANDBY+6000");
        return c.finish("D2 maintenance boot");
    }

    // S9 (D3): wp changes to 1 while SUSPEND by ESW.
    bool scenario9() {
        Checker c;
        c.setExpected("after wp=1 in SUSPEND: exactly one v_wheel write, level 0; state stays SUSPEND");
        setupInputs();
        if (!bootAndSettle(c)) {
            return c.finish("D3 wp=1 in SUSPEND");
        }
        pressEsw(true);
        c.expect(stepUntilState(POWER_STATE::SUSPEND, 40), "SUSPEND not reached");
        stepPolls(5);
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 1, "level in SUSPEND before wp");
        const size_t mark = zephyr_stubs::pin_write_log.size();
        ros_wheel_power_off = true;
        stepOnce();
        expectLevels(c, vWheelWritesSince(mark), {0}, "v_wheel writes after wp=1");
        c.expect(bc::impl.state == POWER_STATE::SUSPEND, "state must stay SUSPEND");
        return c.finish("D3 wp=1 in SUSPEND");
    }

    // S10 (D4): wp=1 held across ESW assert/release.
    bool scenario10() {
        Checker c;
        c.setExpected("after leaving SUSPEND: 0 v_wheel writes in 15 polls, level stays 1");
        setupInputs();
        if (!holdWpThenSuspend(c)) {
            return c.finish("D4 wp=1 held across SUSPEND exit");
        }
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 1, "level in SUSPEND");
        pressEsw(false);
        c.expect(stepUntilStateLeaves(POWER_STATE::SUSPEND, 40), "SUSPEND not left");
        const size_t mark = zephyr_stubs::pin_write_log.size();
        stepPolls(15);
        c.expectEq(static_cast<int64_t>(vWheelWritesSince(mark).size()), 0, "v_wheel writes after SUSPEND exit");
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 1, "final level");
        return c.finish("D4 wp=1 held across SUSPEND exit");
    }

    // S11 (D5): maintenance, wp released in RESUME_WAIT.
    bool scenario11() {
        Checker c;
        c.setExpected("RESUME_WAIT stays; v_wheel writes [1,0] at the same time after wp=0");
        setupInputs();
        setMaintenanceKey();
        if (!bootAndSettle(c)) {
            return c.finish("D5 maintenance wp release in RESUME_WAIT");
        }
        ros_wheel_power_off = true;
        stepPolls(5);
        pressEsw(true);
        c.expect(stepUntilState(POWER_STATE::SUSPEND, 40), "SUSPEND not reached");
        pressEsw(false);
        c.expect(stepUntilState(POWER_STATE::RESUME_WAIT, 40), "RESUME_WAIT not reached");
        stepPolls(3);
        ros_wheel_power_off = false;
        const size_t mark = zephyr_stubs::pin_write_log.size();
        stepOnce();
        c.expect(bc::impl.state == POWER_STATE::RESUME_WAIT, "state must stay RESUME_WAIT");
        expectLevels(c, vWheelWritesSince(mark), {1, 0}, "v_wheel writes after wp=0");
        int64_t first_time = -1, second_time = -2;
        int found = 0;
        for (size_t i = mark; i < zephyr_stubs::pin_write_log.size(); ++i) {
            if (zephyr_stubs::pin_write_log[i].pin == PIN_V_WHEEL) {
                (found++ == 0 ? first_time : second_time) = zephyr_stubs::pin_write_log[i].time;
            }
        }
        c.expectEq(second_time, first_time, "time of the two writes");
        return c.finish("D5 maintenance wp release in RESUME_WAIT");
    }

    // S12 (D6): AUTO_CHARGE -> NORMAL with wp released during charging.
    bool scenario12() {
        Checker c;
        c.setExpected("exactly one level-1 v_wheel write at or after the AUTO_CHARGE -> NORMAL poll");
        setupInputs();
        setMaintenanceKey();
        lexxhard::adc_reader::stub_adc3_mv = 2900;
        bmu_rsoc = 50;
        if (!bootToNormal()) {
            c.expect(false, "NORMAL not reached");
            return c.finish("D6 AUTO_CHARGE -> NORMAL with wp released");
        }
        const int64_t t_normal = transitionTime(POWER_STATE::NORMAL);
        stepUntil(t_normal + 1000);
        ros_wheel_power_off = true;
        c.expect(stepUntilState(POWER_STATE::AUTO_CHARGE, 600), "AUTO_CHARGE not reached");
        ros_wheel_power_off = false;
        stepPolls(10);
        bmu_fault.fail_status1 = 0x40;
        c.expect(stepUntilState(POWER_STATE::NORMAL, 40), "NORMAL not reached after full charge");
        const int64_t t_back = transitions.back().poll_time;
        stepPolls(5);
        int ones = 0;
        for (const auto& w : pinWrites(PIN_V_WHEEL, t_back)) {
            ones += w.level == 1 ? 1 : 0;
        }
        c.expectEq(ones, 1, "level-1 v_wheel writes from AUTO_CHARGE exit");
        return c.finish("D6 AUTO_CHARGE -> NORMAL with wp released");
    }

    // S13 (D7): v_wheel level around NORMAL -> SUSPEND by ESW with wp=1 held.
    bool scenario13() {
        Checker c;
        c.setExpected("level 0 before the NORMAL->SUSPEND poll; that poll writes exactly [1]");
        setupInputs();
        if (!bootAndSettle(c)) {
            return c.finish("D7 SUSPEND entry with wp=1 held");
        }
        ros_wheel_power_off = true;
        stepPolls(5);
        pressEsw(true);
        bool found = false;
        for (int i = 0; i < 40 && !found; ++i) {
            const size_t mark = zephyr_stubs::pin_write_log.size();
            const int level_before = levelAt(zephyr_stubs::virtual_time_ms);
            stepOnce();
            if (!transitions.empty() && transitions.back().from == POWER_STATE::NORMAL &&
                transitions.back().to == POWER_STATE::SUSPEND) {
                found = true;
                c.expectEq(level_before, 0, "level before the SUSPEND-entry poll");
                expectLevels(c, vWheelWritesSince(mark), {1}, "v_wheel writes of the SUSPEND-entry poll");
            }
        }
        c.expect(found, "NORMAL -> SUSPEND not observed");
        return c.finish("D7 SUSPEND entry with wp=1 held");
    }

#ifdef ENABLE_PUSH_MODE
    // Push-mode specification scenarios (expectations are the spec, not the old code).

    // P1 (R5): running, wp=1 during ESW SUSPEND keeps v_wheel on; standard decision returns after release.
    bool scenarioP1(const std::string& title) {
        Checker c;
        c.setExpected("wp ignored in SUSPEND (level 1, no 0 write); level 0 five polls after ESW release");
        setupInputs();
        if (!bootAndSettle(c)) {
            return c.finishSpec(title);
        }
        pressEsw(true);
        c.expect(stepUntilState(POWER_STATE::SUSPEND, 40), "SUSPEND not reached");
        stepPolls(5);
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 1, "level in SUSPEND");
        const size_t mark = zephyr_stubs::pin_write_log.size();
        ros_wheel_power_off = true;
        stepPolls(3);
        const std::vector<int> levels = vWheelWritesSince(mark);
        c.expectEq(std::count(levels.begin(), levels.end(), 0), 0, "level-0 writes while wp=1 in SUSPEND");
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 1, "level while wp=1 in SUSPEND");
        pressEsw(false);
        c.expect(stepUntilStateLeaves(POWER_STATE::SUSPEND, 40), "SUSPEND not left");
        stepPolls(5);
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 0, "level after ESW release");
        return c.finishSpec(title);
    }

    // P2 (Q2/C-1): maintenance, ESW SUSPEND turns v_wheel on; release returns to the maintenance cut.
    bool scenarioP2(const std::string& title) {
        Checker c;
        c.setExpected("level 0 in NORMAL; 1 in ESW SUSPEND without 0 writes; 0 five polls after release");
        setupInputs();
        setMaintenanceKey();
        if (!bootAndSettle(c)) {
            return c.finishSpec(title);
        }
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 0, "level in maintenance NORMAL");
        pressEsw(true);
        c.expect(stepUntilState(POWER_STATE::SUSPEND, 40), "SUSPEND not reached");
        stepPolls(5);
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 1, "level in SUSPEND");
        const size_t mark = zephyr_stubs::pin_write_log.size();
        stepPolls(10);
        const std::vector<int> levels = vWheelWritesSince(mark);
        c.expectEq(std::count(levels.begin(), levels.end(), 0), 0, "level-0 writes in SUSPEND");
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 1, "level after 10 more polls");
        pressEsw(false);
        c.expect(stepUntilStateLeaves(POWER_STATE::SUSPEND, 40), "SUSPEND not left");
        stepPolls(5);
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 0, "level after ESW release");
        return c.finishSpec(title);
    }

    // P3: wp held, then ESW assert/release.
    bool scenarioP3(const std::string& title) {
        Checker c;
        c.setExpected("level 0 with wp=1; 1 in ESW SUSPEND; 0 five polls after release");
        setupInputs();
        if (!bootAndSettle(c)) {
            return c.finishSpec(title);
        }
        ros_wheel_power_off = true;
        stepPolls(5);
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 0, "level with wp=1");
        pressEsw(true);
        c.expect(stepUntilState(POWER_STATE::SUSPEND, 40), "SUSPEND not reached");
        stepPolls(5);
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 1, "level in SUSPEND");
        pressEsw(false);
        c.expect(stepUntilStateLeaves(POWER_STATE::SUSPEND, 40), "SUSPEND not left");
        stepPolls(5);
        c.expectEq(levelAt(zephyr_stubs::virtual_time_ms), 0, "level after ESW release");
        return c.finishSpec(title);
    }

    // P1-P3 on a machine without a lidar (bypass builds).
    bool scenarioP1LidarAbsent() {
        lidar_absent = true;
        return scenarioP1("P1 running: wp ignored in ESW SUSPEND (lidar absent)");
    }
    bool scenarioP2LidarAbsent() {
        lidar_absent = true;
        return scenarioP2("P2 maintenance: ESW SUSPEND (lidar absent)");
    }
    bool scenarioP3LidarAbsent() {
        lidar_absent = true;
        return scenarioP3("P3 wp held then ESW (lidar absent)");
    }
#endif

#ifdef BC_NEW_BUILD
    // B1: power-on of a machine without a lidar (OSSD1/OSSD2 both low).
    bool scenarioB1() {
        Checker c;
#ifdef BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST
        const std::string title = "B1 lidar absent: boots to NORMAL (bypass)";
        c.setExpected("NORMAL reached and held for 2 s, sl not asserted, no NORMAL -> SUSPEND");
#else
        const std::string title = "B1 lidar absent: stalls in STANDBY (no bypass, reproduces the field issue)";
        c.setExpected("NORMAL not reached; stays in STANDBY with sl asserted");
#endif
        lidar_absent = true;
        setupInputs();
        const bool reached = bootToNormal();
#ifdef BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST
        c.expect(reached, "NORMAL not reached");
        if (reached) {
            stepUntil(zephyr_stubs::virtual_time_ms + 2000);
            c.expect(bc::impl.state == POWER_STATE::NORMAL, "not in NORMAL 2 s after reaching it");
            c.expect(!bc::impl.sl.is_asserted(), "sl asserted");
            for (const Transition& t : transitions) {
                c.expect(!(t.from == POWER_STATE::NORMAL && t.to == POWER_STATE::SUSPEND), "NORMAL->SUSPEND observed");
            }
        }
#else
        c.expect(!reached, "NORMAL reached");
        c.expect(bc::impl.state == POWER_STATE::STANDBY, "not stalled in STANDBY");
        c.expect(bc::impl.sl.is_asserted(), "sl not asserted");
#endif
        return c.finishSpec(title);
    }
#endif

    bool runScenario(const int index, const int variant) {
        if (index == 5) {
            return scenario5Variant(static_cast<size_t>(variant));
        }
        switch (index) {
            case 1: return scenario1();
            case 2: return scenario2();
            case 3: return scenario3();
            case 4: return scenario4();
            case 6: return scenario6();
            case 7: return scenario7();
            case 8: return scenario8();
            case 9: return scenario9();
            case 10: return scenario10();
            case 11: return scenario11();
            case 12: return scenario12();
            case 13: return scenario13();
#ifdef ENABLE_PUSH_MODE
            case 14: return scenarioP1("P1 running: wp ignored in ESW SUSPEND");
            case 15: return scenarioP2("P2 maintenance: ESW SUSPEND");
            case 16: return scenarioP3("P3 wp held then ESW");
#else
            case 14:
            case 15:
            case 16:
                std::cout << "SKIP - Push scenario P" << index - 13 << " (ENABLE_PUSH_MODE not set)" << std::endl;
                return true;
#endif
#ifdef BC_NEW_BUILD
            case 17: return scenarioB1();
#endif
#if defined(ENABLE_PUSH_MODE) && defined(BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST)
            case 18: return scenarioP1LidarAbsent();
            case 19: return scenarioP2LidarAbsent();
            case 20: return scenarioP3LidarAbsent();
#endif
            default: return false;
        }
    }

    // Re-executes this binary for one scenario. Returns the child's exit status, or -1 when it did not exit normally
    // (crash, abort, timeout via SIGALRM).
    int runInChild(const char* self, const int index, const int variant = 0) {
        std::cout << std::flush;
        const pid_t pid = fork();
        if (pid == 0) {
            const std::string arg = std::to_string(index);
            const std::string variant_arg = std::to_string(variant);
            execl(self, self, arg.c_str(), variant_arg.c_str(), static_cast<char*>(nullptr));
            _exit(128);
        }
        int status = 0;
        if (pid < 0 || waitpid(pid, &status, 0) < 0) {
            return -1;
        }
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    }
}

int main(int argc, char** argv) {
    if (argc == 3) {
        alarm(60);  // a hang in the code under test ends the child with SIGALRM (reported as a harness error)
#ifdef BC_NEW_BUILD
        const int index = std::atoi(argv[1]);
        return runScenario(index, std::atoi(argv[2])) ? 0 : (index >= FIRST_PUSH_SCENARIO ? EXIT_SPEC_FAIL : EXIT_DIFF);
#else
        return runScenario(std::atoi(argv[1]), std::atoi(argv[2])) ? 0 : 1;
#endif
    }

#ifdef BC_NEW_BUILD
    std::cout << "=== Board Controller Baseline Test Suite (new source, " << BC_NEW_BUILD_NAME << ") ===" << std::endl;
    int same = 0, diff = 0, spec_pass = 0, spec_fail = 0, harness_errors = 0;
#if defined(ENABLE_PUSH_MODE) && defined(BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST)
    constexpr int LAST_SCENARIO = 20;
#else
    constexpr int LAST_SCENARIO = 17;
#endif
    for (int index = 1; index <= LAST_SCENARIO; ++index) {
#ifndef ENABLE_PUSH_MODE
        if (index >= 14 && index <= 16) {
            continue;
        }
#endif
        const int variants = index == 5 ? static_cast<int>(scenario5Variants().size()) : 1;
        for (int variant = 0; variant < variants; ++variant) {
            const int status = runInChild(argv[0], index, variant);
            if (status == 0) {
                ++(index >= FIRST_PUSH_SCENARIO ? spec_pass : same);
            } else if (status == EXIT_DIFF) {
                ++diff;
            } else if (status == EXIT_SPEC_FAIL) {
                ++spec_fail;
            } else {
                ++harness_errors;
                std::cout << "HARNESS ERROR - scenario " << index << " variant " << variant << " status " << status
                          << std::endl;
            }
        }
    }
    std::cout << std::endl << "RESULT: SAME=" << same << " DIFF=" << diff;
    std::cout << " SPEC_PASS=" << spec_pass << " SPEC_FAIL=" << spec_fail;
    std::cout << std::endl;
    return harness_errors == 0 ? 0 : 2;
#else
    std::cout << "=== Board Controller Baseline Test Suite ===" << std::endl;
    std::cout << "Source commit: 8c94ed67dfa3e68e0c2aa007839b23dca4be1262" << std::endl;

    int passed = 0, failed = 0;
    for (int index = 1; index <= LAST_BASELINE_SCENARIO; ++index) {
        // scenario 5 runs one child per BMU variant and passes only when all variants pass
        const int variants = index == 5 ? static_cast<int>(scenario5Variants().size()) : 1;
        bool ok = true;
        for (int variant = 0; variant < variants; ++variant) {
            ok = runInChild(argv[0], index, variant) == 0 && ok;
        }
        if (ok) {
            ++passed;
        } else {
            ++failed;
        }
    }

    std::cout << std::endl << "RESULT: " << passed << " passed, " << failed << " failed" << std::endl;
    return failed == 0 ? 0 : 1;
#endif
}
