/*
 * Wrapper to compile and access the old board_controller.cpp in test context
 */
#pragma once

#include "zephyr_stubs.hpp"
#include "lexxhard_stubs.hpp"
#include "../build/old_src/power_state.hpp"
#include "../build/old_src/adc_reader.hpp"
#include "../build/old_src/can_controller.hpp"
#include "../build/old_src/led_controller.hpp"
#include "../build/old_src/board_controller.hpp"
#include "../build/old_src/common.hpp"

// We need access to the private state_controller class, so we'll include
// the .cpp file after defining all stubs, and make private public
namespace lexxhard::board_controller {
    // Forward declare the impl object and make it accessible
    extern class state_controller {
    public:
        POWER_STATE state;
        // Add other public accessors as needed
    } impl;
}

#endif // BOARD_CONTROLLER_WRAPPER_HPP
