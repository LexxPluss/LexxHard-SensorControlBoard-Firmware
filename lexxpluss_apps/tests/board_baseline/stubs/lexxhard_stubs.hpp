// LexxHard namespace stubs for host testing
#pragma once

// This file provides minimal stub implementations for LexxHard components
// Actual struct definitions come from the old headers

#include <cstdint>

namespace lexxhard::adc_reader {
// Value returned by get_adc3(); tests set it to simulate the charger connector voltage.
extern int32_t stub_adc3_mv;
}
