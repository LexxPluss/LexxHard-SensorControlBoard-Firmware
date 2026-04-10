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
#include <atomic>
#include <cstdint>

namespace lexxhard::board_controller {

/**
 * @brief Per-signal sampling configuration.
 *
 * @param sampling_period_ms
 *   Interval between samples in milliseconds. tick() must be called at this
 *   rate or faster (ideally at the GCD of all signals' periods).
 *   The prescaler uses >= comparison so skipped ticks do not break sampling.
 * @param ng_count
 *   Number of consecutive NG samples required to confirm a fault.
 *   Must be >= 2 (enforced by static_assert). ng_count=1 would bypass
 *   debouncing entirely, which contradicts the purpose of this class.
 *   Confirmation window = sampling_period_ms * ng_count (ms).
 */
struct SignalConfig {
    uint32_t sampling_period_ms{20};
    uint8_t  ng_count{3};
};

/**
 * @brief Full PGOOD debouncer configuration (C++20 NTTP struct).
 *
 * Defines per-signal sampling period and confirmation count for each of the
 * four PGOOD signals, plus the maintenance-mode shutdown policy.
 *
 * Example — motor lines sampled at 1 ms, 50-sample confirmation (50 ms window);
 * control lines sampled at 20 ms, 3-sample confirmation (60 ms window):
 *
 * @code
 * constexpr PgoodConfig kCfg {
 *     .v24        = {.sampling_period_ms = 20, .ng_count =  3},
 *     .peripheral = {.sampling_period_ms = 20, .ng_count =  3},
 *     .mtr_l      = {.sampling_period_ms =  1, .ng_count = 50},
 *     .mtr_r      = {.sampling_period_ms =  1, .ng_count = 50},
 *     .shutdown_in_maintenance = false,
 * };
 * @endcode
 *
 * @param shutdown_in_maintenance
 *   false (intended default): confirmed NG in maintenance mode is suppressed
 *                     (tick() returns false). Shutdown is deferred until the
 *                     robot leaves maintenance mode.
 *   true              : confirmed NG triggers shutdown even in maintenance mode.
 */
struct PgoodConfig {
    SignalConfig v24        {.sampling_period_ms = 20, .ng_count =  3};
    SignalConfig peripheral {.sampling_period_ms = 20, .ng_count =  3};
    SignalConfig mtr_l      {.sampling_period_ms =  1, .ng_count = 50};
    SignalConfig mtr_r      {.sampling_period_ms =  1, .ng_count = 50};
    bool shutdown_in_maintenance{false};
};

/**
 * @brief Per-signal PGOOD debouncer with independent sampling periods.
 *
 * Tracks 4 independent power-good signals (PG_24V, PG_Peripheral,
 * PG_MTR_L, PG_MTR_R). Each signal has its own sampling period and
 * consecutive-NG confirmation count, configured via PgoodConfig.
 *
 * tick() must be called from a single execution context (e.g. a Zephyr
 * k_work item) at the GCD of all configured sampling periods. Do NOT
 * call tick() and get_signal()/is_ng_confirmed() from different contexts
 * simultaneously — this class contains no synchronisation primitives.
 *
 * State machine per signal:
 *
 *   [reset()] --> OK
 *   OK         --[NG, ++ng_observed < ng_count]--> PENDING_NG
 *   OK         --[NG, ++ng_observed >= ng_count]--> NG_CONFIRMED
 *   PENDING_NG --[OK]--> OK  (ng_observed = 0)
 *   PENDING_NG --[NG, ++ng_observed < ng_count]--> PENDING_NG
 *   PENDING_NG --[NG, ++ng_observed >= ng_count]--> NG_CONFIRMED
 *   NG_CONFIRMED --[any]--> NG_CONFIRMED  (sticky until reset())
 *   NG_CONFIRMED --[reset()]--> OK
 *
 * Transitions occur only when the per-signal prescaler reaches
 * sampling_period_ms (>= comparison; resets to 0 after each sample).
 *
 * @tparam Config  Compile-time PgoodConfig value (C++20 NTTP).
 */
template<PgoodConfig Config = PgoodConfig{}>
class PgoodDebouncerT {
    static_assert(Config.v24.ng_count        >= 2, "ng_count must be >= 2");
    static_assert(Config.peripheral.ng_count >= 2, "ng_count must be >= 2");
    static_assert(Config.mtr_l.ng_count      >= 2, "ng_count must be >= 2");
    static_assert(Config.mtr_r.ng_count      >= 2, "ng_count must be >= 2");
public:
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
        State    state{State::OK};
        uint8_t  ng_observed{0};  ///< consecutive NG sample count (cf. SignalConfig::ng_count)
        uint32_t prescaler{0};    ///< counts up each tick(); samples when >= sampling_period_ms
    };

    /**
     * @brief Advance the debouncer by one base tick and return shutdown intent.
     *
     * Call this function every base-timer period (e.g. every 1 ms).
     * Each signal samples its GPIO input only when its prescaler reaches
     * sampling_period_ms; otherwise the call is a no-op for that signal.
     *
     * GPIO values must be read by the caller before invoking tick():
     * @code
     *   bool ng_24v = gpio_pin_get_dt(&gpio_pgood_24v) == 1;  // 1 = NG
     *   bool shutdown = debouncer_.tick(ng_24v, ng_periph, ng_mtr_l, ng_mtr_r,
     *                                   is_maintenance);
     * @endcode
     *
     * @param ng_24v        true = PG_24V reads NG (HIGH = fault, active-low signal).
     * @param ng_peripheral true = PG_Peripheral reads NG.
     * @param ng_mtr_l      true = PG_MTR_L reads NG.
     * @param ng_mtr_r      true = PG_MTR_R reads NG.
     * @param is_maintenance true = robot is in maintenance or transition-to-running mode.
     *                       MTR_L and MTR_R inputs are masked (treated as OK).
     * @return true if shutdown should be triggered this cycle.
     *         Remains true on every subsequent call until reset() is called.
     */
    bool tick(bool ng_24v, bool ng_peripheral,
              bool ng_mtr_l, bool ng_mtr_r,
              bool is_maintenance) noexcept {
        // Maintenance bypass: mask MTR signals.
        std::array<bool, SIGNAL_COUNT> const inputs = {{
            ng_24v,
            ng_peripheral,
            (is_maintenance ? false : ng_mtr_l),
            (is_maintenance ? false : ng_mtr_r),
        }};

        bool any_confirmed{false};
        for (uint8_t i{0}; i < SIGNAL_COUNT; ++i) {
            tick_signal(signals_[i], kConfigs[i], inputs[i]);
            if (signals_[i].state == State::NG_CONFIRMED) {
                any_confirmed = true;
            }
        }

        // Publish confirmed state atomically so is_ng_confirmed() is safe
        // to call from any interrupt context without a lock.
        ng_confirmed_.store(any_confirmed, std::memory_order_release);

        if (!any_confirmed) {
            return false;
        }

        // When shutdown_in_maintenance == false and in maintenance, suppress.
        if (!Config.shutdown_in_maintenance && is_maintenance) {
            return false;
        }

        return true;
    }

    /** @brief Reset all signals to OK. Call on power-on and power-off. */
    void reset() noexcept {
        for (auto& s : signals_) {
            s.state       = State::OK;
            s.ng_observed = 0;
            s.prescaler   = 0;
        }
        ng_confirmed_.store(false, std::memory_order_release);
    }

    /**
     * @brief True if any signal is NG_CONFIRMED, regardless of maintenance mode.
     *
     * Safe to call from any interrupt context concurrently with tick().
     * The flag is updated atomically at the end of each tick() call.
     */
    bool is_ng_confirmed() const noexcept {
        return ng_confirmed_.load(std::memory_order_acquire);
    }

    /** @brief Inspect per-signal state (for logging and test assertions). */
    const SignalState& get_signal(SignalIndex idx) const noexcept {
        return signals_[static_cast<uint8_t>(idx)];
    }

private:
    static constexpr uint8_t SIGNAL_COUNT{4};

    // Flatten config fields into arrays for uniform iteration.
    static constexpr std::array<SignalConfig, SIGNAL_COUNT> kConfigs = {{
        Config.v24,
        Config.peripheral,
        Config.mtr_l,
        Config.mtr_r,
    }};

    static void tick_signal(SignalState& sig,
                            const SignalConfig& cfg,
                            bool is_ng) noexcept {
        // Advance prescaler; sample only when period is reached.
        if (++sig.prescaler < cfg.sampling_period_ms) {
            return;
        }
        sig.prescaler = 0;

        switch (sig.state) {
        case State::OK:
            if (is_ng) {
                ++sig.ng_observed;
                sig.state = (sig.ng_observed >= cfg.ng_count)
                                ? State::NG_CONFIRMED
                                : State::PENDING_NG;
            } else {
                sig.ng_observed = 0;
            }
            break;

        case State::PENDING_NG:
            if (is_ng) {
                ++sig.ng_observed;
                if (sig.ng_observed >= cfg.ng_count) {
                    sig.state = State::NG_CONFIRMED;
                }
            } else {
                sig.ng_observed = 0;
                sig.state       = State::OK;
            }
            break;

        case State::NG_CONFIRMED:
            // Sticky: no transitions until reset().
            break;
        }
    }

    std::array<SignalState, SIGNAL_COUNT> signals_{};
    std::atomic<bool> ng_confirmed_{false};
};

/** Default alias: default PgoodConfig (production settings). */
using PgoodDebouncer = PgoodDebouncerT<>;

}  // namespace lexxhard::board_controller
