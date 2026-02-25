/*
 * Copyright (c) 2024, LexxPluss Inc.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <array>
#include <cstdint>

namespace lexxhard::board_controller {

/**
 * @brief Per-signal PGOOD debouncer state machine.
 *
 * Tracks 4 independent power-good signals (PG_24V, PG_Peripheral,
 * PG_MTR_L, PG_MTR_R). A signal must read NG for NG_CONFIRM_COUNT
 * consecutive polls before a shutdown is triggered. Once confirmed NG,
 * no further transitions occur until reset() is called.
 *
 * Polling period is 20ms (board_controller main loop). With NG_CONFIRM_COUNT=3
 * the confirmation window is 60ms, longer than typical load transients.
 *
 * @tparam ShutdownInMaintenance
 *   true  (default): confirmed NG triggers shutdown even in maintenance mode.
 *   false           : confirmed NG during maintenance mode is suppressed
 *                     (update() returns false). The caller is responsible
 *                     for logging the suppressed fault.
 */
template<bool ShutdownInMaintenance = true>
class PgoodDebouncerT {
public:
    static constexpr uint8_t NG_CONFIRM_COUNT{3};
    static constexpr uint8_t SIGNAL_COUNT{4};

    enum class SignalIndex : uint8_t {
        V24        = 0,
        PERIPHERAL = 1,
        MTR_L      = 2,
        MTR_R      = 3,
    };

    enum class State : uint8_t {
        OK,
        PENDING_NG,
        NG_CONFIRMED,
    };

    struct SignalState {
        State   state{State::OK};
        uint8_t ng_count{0};
    };

    /**
     * @brief Feed one sample into the debouncer and return shutdown intent.
     *
     * @param ng_24v        true = PG_24V GPIO reads NG (HIGH = fault, active-low signal).
     * @param ng_peripheral true = PG_Peripheral GPIO reads NG.
     * @param ng_mtr_l      true = PG_MTR_L GPIO reads NG.
     * @param ng_mtr_r      true = PG_MTR_R GPIO reads NG.
     * @param is_maintenance true = robot is in maintenance or transition-to-running mode.
     *                       When true, ng_mtr_l and ng_mtr_r are masked (treated as OK),
     *                       preserving the existing maintenance bypass behaviour.
     * @return true if shutdown should be triggered this cycle.
     */
    bool update(bool ng_24v, bool ng_peripheral,
                bool ng_mtr_l, bool ng_mtr_r,
                bool is_maintenance) noexcept {
        // Existing maintenance bypass: treat MTR_L/MTR_R as OK during maintenance.
        bool const inputs[SIGNAL_COUNT] = {
            ng_24v,
            ng_peripheral,
            (is_maintenance ? false : ng_mtr_l),
            (is_maintenance ? false : ng_mtr_r),
        };

        for (uint8_t i{0}; i < SIGNAL_COUNT; ++i) {
            update_signal(signals_[i], inputs[i]);
        }

        if (!is_ng_confirmed()) {
            return false;
        }

        // When ShutdownInMaintenance == false and the robot is in maintenance,
        // suppress the shutdown. The caller logs this condition.
        if constexpr (!ShutdownInMaintenance) {
            if (is_maintenance) {
                return false;
            }
        }

        return true;
    }

    /** @brief Reset all signals to OK state. Call on power-on and power-off. */
    void reset() noexcept {
        for (auto& s : signals_) {
            s.state    = State::OK;
            s.ng_count = 0;
        }
    }

    /** @brief True if any signal has reached NG_CONFIRMED, regardless of maintenance mode. */
    bool is_ng_confirmed() const noexcept {
        for (const auto& s : signals_) {
            if (s.state == State::NG_CONFIRMED) {
                return true;
            }
        }
        return false;
    }

    /** @brief Inspect per-signal state (used for logging and test assertions). */
    const SignalState& get_signal(SignalIndex idx) const noexcept {
        return signals_[static_cast<uint8_t>(idx)];
    }

private:
    static void update_signal(SignalState& sig, bool is_ng) noexcept {
        switch (sig.state) {
        case State::OK:
            if (is_ng) {
                sig.ng_count++;
                sig.state = (sig.ng_count >= NG_CONFIRM_COUNT)
                                ? State::NG_CONFIRMED
                                : State::PENDING_NG;
            } else {
                sig.ng_count = 0;
                // stay OK
            }
            break;

        case State::PENDING_NG:
            if (is_ng) {
                sig.ng_count++;
                if (sig.ng_count >= NG_CONFIRM_COUNT) {
                    sig.state = State::NG_CONFIRMED;
                }
            } else {
                sig.ng_count = 0;
                sig.state    = State::OK;
            }
            break;

        case State::NG_CONFIRMED:
            // Sticky: no transitions until reset() is called.
            break;
        }
    }

    std::array<SignalState, SIGNAL_COUNT> signals_{};
};

/** Default alias: shutdown is active in maintenance mode (production default). */
using PgoodDebouncer = PgoodDebouncerT<true>;

}  // namespace lexxhard::board_controller
