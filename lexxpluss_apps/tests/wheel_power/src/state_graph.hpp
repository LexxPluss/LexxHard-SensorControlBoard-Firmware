/*
 * Copyright (c) 2026, LexxPluss Inc.
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

// Test-only model of the board power state graph. Conditions are not modeled: an edge exists when the
// board_controller can make that transition under some input. Line numbers refer to board_controller.cpp with the
// v_wheel owner wired in (the working tree of this branch).

#pragma once

#include <cstdint>

namespace wheel_power_test {

enum class power_state : std::uint8_t {
    OFF,
    TIMEROFF,
    WAIT_SW,
    POST,
    STANDBY,
    NORMAL,
    SUSPEND,
    RESUME_WAIT,
    AUTO_CHARGE,
    MANUAL_CHARGE,
    LOCKDOWN,
    OFF_WAIT
};

constexpr int power_state_count = 12;

constexpr std::uint16_t bit(power_state s) { return static_cast<std::uint16_t>(1U << static_cast<unsigned>(s)); }

template <typename... States>
constexpr std::uint16_t maskOf(States... states)
{
    return static_cast<std::uint16_t>((0U | ... | bit(states)));
}

using ps = power_state;

// Successors of each state, indexed by power_state. The edge set is the same for the standard and the Push build:
// ENABLE_PUSH_MODE only removes conditions of NORMAL -> SUSPEND (psw 1477-1479, sl 1495-1497, emergency stop from ROS
// 1498-1501, is_dead 1502-1504); NORMAL -> SUSPEND stays through 1481-1493.
constexpr std::uint16_t edge_masks[power_state_count] = {
    // OFF: WAIT_SW (1416-1417)
    maskOf(ps::WAIT_SW),
    // TIMEROFF: OFF (1421-1422)
    maskOf(ps::OFF),
    // WAIT_SW: OFF (1425-1426), POST (1427-1429)
    maskOf(ps::OFF, ps::POST),
    // POST: OFF (1433-1434, 1438-1440), STANDBY (1435-1437)
    maskOf(ps::OFF, ps::STANDBY),
    // STANDBY: OFF (1446-1447), LOCKDOWN (1448-1449), OFF_WAIT (1450-1451, 1452-1454), MANUAL_CHARGE (1455-1457),
    //          SUSPEND (1458-1460), NORMAL (1461-1463)
    maskOf(ps::OFF, ps::LOCKDOWN, ps::OFF_WAIT, ps::MANUAL_CHARGE, ps::SUSPEND, ps::NORMAL),
    // NORMAL: OFF_WAIT (1469-1470, 1471-1473), LOCKDOWN (1474-1475), SUSPEND (1477-1504), AUTO_CHARGE (1506-1510),
    //         MANUAL_CHARGE (1511-1513)
    maskOf(ps::OFF_WAIT, ps::LOCKDOWN, ps::SUSPEND, ps::AUTO_CHARGE, ps::MANUAL_CHARGE),
    // SUSPEND: OFF (1519-1520), LOCKDOWN (1521-1522), OFF_WAIT (1523-1525, 1526-1527), RESUME_WAIT (1528-1530),
    //          MANUAL_CHARGE (1531-1533)
    maskOf(ps::OFF, ps::LOCKDOWN, ps::OFF_WAIT, ps::RESUME_WAIT, ps::MANUAL_CHARGE),
    // RESUME_WAIT: OFF_WAIT (1539-1540, 1541-1543), SUSPEND (1544-1567), NORMAL (1568-1573), STANDBY (1574-1577),
    //              MANUAL_CHARGE (1578-1580)
    maskOf(ps::OFF_WAIT, ps::SUSPEND, ps::NORMAL, ps::STANDBY, ps::MANUAL_CHARGE),
    // AUTO_CHARGE: OFF_WAIT (1585-1586, 1587-1589), STANDBY (1590-1601), SUSPEND (1602-1615), NORMAL (1616-1627)
    maskOf(ps::OFF_WAIT, ps::STANDBY, ps::SUSPEND, ps::NORMAL),
    // MANUAL_CHARGE: OFF_WAIT (1631-1633, 1634-1636)
    maskOf(ps::OFF_WAIT),
    // LOCKDOWN: OFF (1640-1642)
    maskOf(ps::OFF),
    // OFF_WAIT: OFF (1646-1647, 1652-1655), TIMEROFF (1649-1650)
    maskOf(ps::OFF, ps::TIMEROFF)};

constexpr bool hasEdge(power_state from, power_state to)
{
    return (edge_masks[static_cast<int>(from)] & bit(to)) != 0;
}

// The states whose poll() calls v_wheel.poll(): STANDBY (1444), NORMAL (1468), SUSPEND (1517), RESUME_WAIT (1538).
constexpr bool pollsWheelRelay(power_state s)
{
    return s == ps::STANDBY || s == ps::NORMAL || s == ps::SUSPEND || s == ps::RESUME_WAIT;
}

// In the cycle where the key switch turns to running (is_transition_to_running), the only transitions that can happen.
// An empty mask means the cycle is not constrained.
constexpr std::uint16_t kswRiseMask(power_state s)
{
    switch (s) {
    case ps::STANDBY:
        // OFF (1446-1447), LOCKDOWN (1448-1449) and OFF_WAIT (1452-1454, certain unless an earlier branch fires)
        return maskOf(ps::OFF, ps::LOCKDOWN, ps::OFF_WAIT);
    case ps::NORMAL:
        // OFF_WAIT (1471-1473); should_turn_off() cannot be true while running
        return maskOf(ps::OFF_WAIT);
    case ps::SUSPEND:
        // OFF (1519-1520), LOCKDOWN (1521-1522), OFF_WAIT (1523-1525)
        return maskOf(ps::OFF, ps::LOCKDOWN, ps::OFF_WAIT);
    case ps::RESUME_WAIT:
        // OFF_WAIT (1541-1543)
        return maskOf(ps::OFF_WAIT);
    case ps::AUTO_CHARGE:
        // OFF_WAIT (1587-1589); AUTO_CHARGE does not call v_wheel.poll()
        return maskOf(ps::OFF_WAIT);
    default:
        return 0;
    }
}

// Events: 0..7 poll(esw = (e >> 2) & 1, wp = (e >> 1) & 1, ksw = e & 1), 8..19 enter(enter_event_states[e - 8]).
constexpr int graph_event_count = 20;
constexpr int first_enter_event = 8;

constexpr power_state enter_event_states[12] = {ps::POST,        ps::STANDBY,   ps::NORMAL,   ps::SUSPEND,
                                                ps::RESUME_WAIT, ps::AUTO_CHARGE, ps::MANUAL_CHARGE, ps::LOCKDOWN,
                                                ps::OFF_WAIT,    ps::OFF,       ps::TIMEROFF, ps::WAIT_SW};

// Tracks the board state along an event sequence and tells which events are possible next.
struct graph_tracker {
    power_state state{ps::POST};
    bool prev_ksw{false};          // ksw of the previous poll event; the ksw starts as not running
    std::uint16_t pending_mask{0};  // after a ksw rise, the only allowed next transitions
    bool last_poll_rise{false};    // the last poll event was a ksw rise cycle

    constexpr bool allowed(int e) const
    {
        if (e < first_enter_event) {
            return pending_mask == 0;
        }
        const power_state target = enter_event_states[e - first_enter_event];
        return hasEdge(state, target) && (pending_mask == 0 || (pending_mask & bit(target)) != 0);
    }

    constexpr void apply(int e)
    {
        last_poll_rise = false;
        if (e < first_enter_event) {
            const bool ksw = (e & 1) != 0;
            last_poll_rise = ksw && !prev_ksw;
            prev_ksw = ksw;
            pending_mask = last_poll_rise ? kswRiseMask(state) : 0;
            return;
        }
        state = enter_event_states[e - first_enter_event];
        pending_mask = 0;
    }
};

}  // namespace wheel_power_test
