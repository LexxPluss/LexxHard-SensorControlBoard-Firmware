/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_l7_runtime.hpp"
#include "tof_l7_sensor.hpp"

namespace l7 = lexxhard::tof_l7;

namespace {

uint8_t firmware_byte{0xA5};
const uint8_t *runtime_firmware{&firmware_byte};
size_t runtime_firmware_size{VL53L7CX_FIRMWARE_DOWNLOAD_SIZE};

uint8_t init_status{VL53L7CX_STATUS_OK};
uint8_t resolution_status{VL53L7CX_STATUS_OK};
uint8_t frequency_status{VL53L7CX_STATUS_OK};
uint8_t start_status{VL53L7CX_STATUS_OK};
uint8_t stop_status{VL53L7CX_STATUS_OK};
uint8_t ready_status{VL53L7CX_STATUS_OK};
uint8_t fetch_status{VL53L7CX_STATUS_OK};
uint8_t ready_value{0};
int injected_port_error{0};
int fetch_port_error{0};
int sticky_port_error{0};

int init_calls{0};
int resolution_calls{0};
int frequency_calls{0};
int start_calls{0};
int stop_calls{0};
int ready_calls{0};
int fetch_calls{0};
uint8_t configured_resolution{0};
uint8_t configured_frequency{0};
VL53L7CX_Configuration *last_device{nullptr};
VL53L7CX_ResultsData fake_results{};

l7::sensor sensor_under_test{};
l7::scratch work{};
l7::sample output{};
l7::operation_status operation{};

void take_port_error() {
  sticky_port_error = injected_port_error;
  injected_port_error = 0;
}

void good_results() {
  memset(&fake_results, 0, sizeof(fake_results));
  fake_results.silicon_temp_degc = 23;
  for (size_t zone{0}; zone < l7::kZoneCount; ++zone) {
    fake_results.nb_target_detected[zone] = 1;
    fake_results.distance_mm[zone] = static_cast<int16_t>(100 + zone);
    fake_results.target_status[zone] = 5;
  }
}

void reset_fakes() {
  runtime_firmware = &firmware_byte;
  runtime_firmware_size = VL53L7CX_FIRMWARE_DOWNLOAD_SIZE;
  init_status = resolution_status = frequency_status = start_status =
      stop_status = ready_status = fetch_status = VL53L7CX_STATUS_OK;
  ready_value = 0;
  injected_port_error = 0;
  fetch_port_error = 0;
  sticky_port_error = 0;
  init_calls = resolution_calls = frequency_calls = start_calls = stop_calls =
      ready_calls = fetch_calls = 0;
  configured_resolution = configured_frequency = 0;
  last_device = nullptr;
  sensor_under_test = l7::sensor{};
  work = l7::scratch{};
  output = l7::sample{};
  operation = l7::operation_status{};
  good_results();
}

void before(void *) { reset_fakes(); }

void open_configure_start() {
  zassert_ok(l7::open(&sensor_under_test, 0x30, &operation));
  zassert_ok(l7::configure(&sensor_under_test, 10, &operation));
  zassert_ok(l7::start(&sensor_under_test, &operation));
}

} // namespace

namespace lexxhard::tof_l7_runtime {

const uint8_t *firmware_data() { return runtime_firmware; }

size_t firmware_size() { return runtime_firmware_size; }

} // namespace lexxhard::tof_l7_runtime

extern "C" {

void vl53l7cx_port_clear_error(void) { sticky_port_error = 0; }

int vl53l7cx_port_error(void) { return sticky_port_error; }

uint8_t vl53l7cx_init(VL53L7CX_Configuration *p_dev) {
  ++init_calls;
  last_device = p_dev;
  take_port_error();
  return init_status;
}

uint8_t vl53l7cx_set_resolution(VL53L7CX_Configuration *p_dev,
                                uint8_t resolution) {
  ++resolution_calls;
  last_device = p_dev;
  configured_resolution = resolution;
  take_port_error();
  return resolution_status;
}

uint8_t vl53l7cx_set_ranging_frequency_hz(VL53L7CX_Configuration *p_dev,
                                          uint8_t frequency_hz) {
  ++frequency_calls;
  last_device = p_dev;
  configured_frequency = frequency_hz;
  take_port_error();
  return frequency_status;
}

uint8_t vl53l7cx_start_ranging(VL53L7CX_Configuration *p_dev) {
  ++start_calls;
  last_device = p_dev;
  take_port_error();
  return start_status;
}

uint8_t vl53l7cx_stop_ranging(VL53L7CX_Configuration *p_dev) {
  ++stop_calls;
  last_device = p_dev;
  take_port_error();
  return stop_status;
}

uint8_t vl53l7cx_check_data_ready(VL53L7CX_Configuration *p_dev,
                                  uint8_t *is_ready) {
  ++ready_calls;
  last_device = p_dev;
  *is_ready = ready_value;
  take_port_error();
  return ready_status;
}

uint8_t vl53l7cx_get_ranging_data(VL53L7CX_Configuration *p_dev,
                                  VL53L7CX_ResultsData *results) {
  ++fetch_calls;
  last_device = p_dev;
  *results = fake_results;
  take_port_error();
  sticky_port_error = fetch_port_error;
  return fetch_status;
}

} // extern "C"

ZTEST_SUITE(tof_l7_sensor, nullptr, nullptr, before, nullptr, nullptr);

ZTEST(tof_l7_sensor,
      test_open_binds_the_verified_runtime_payload_and_converts_address_once) {
  zassert_ok(l7::open(&sensor_under_test, 0x29, &operation));
  zassert_equal(init_calls, 1);
  zassert_equal(last_device, &sensor_under_test.uld);
  zassert_equal(sensor_under_test.uld.platform.address, 0x52);
  zassert_equal(sensor_under_test.uld.platform.firmware, runtime_firmware);
  zassert_equal(sensor_under_test.uld.platform.firmware_size,
                runtime_firmware_size);
  zassert_equal(sensor_under_test.current, l7::lifecycle::opened);
}

ZTEST(tof_l7_sensor,
      test_invalid_addresses_are_rejected_before_the_runtime_or_uld) {
  static constexpr uint8_t invalid[]{0x00, 0x07, 0x78, 0x7F, 0xFF};
  for (const uint8_t address : invalid) {
    zassert_equal(l7::open(&sensor_under_test, address, &operation), -EINVAL);
    zassert_equal(operation.failed_stage, l7::stage::address);
    zassert_equal(init_calls, 0);
  }
  zassert_ok(l7::open(&sensor_under_test, 0x08, &operation));
  reset_fakes();
  zassert_ok(l7::open(&sensor_under_test, 0x77, &operation));
}

ZTEST(tof_l7_sensor, test_missing_or_short_firmware_is_rejected_before_init) {
  runtime_firmware = nullptr;
  zassert_equal(l7::open(&sensor_under_test, 0x29, &operation), -ENODATA);
  zassert_equal(operation.failed_stage, l7::stage::firmware);
  zassert_equal(init_calls, 0);

  runtime_firmware = &firmware_byte;
  runtime_firmware_size = VL53L7CX_FIRMWARE_DOWNLOAD_SIZE - 1U;
  zassert_equal(l7::open(&sensor_under_test, 0x29, &operation), -EPROTO);
  zassert_equal(operation.failed_stage, l7::stage::firmware);
  zassert_equal(init_calls, 0);
}

ZTEST(tof_l7_sensor, test_transport_errno_wins_even_when_init_claims_success) {
  injected_port_error = -ETIMEDOUT;
  zassert_equal(l7::open(&sensor_under_test, 0x29, &operation), -ETIMEDOUT);
  zassert_equal(operation.failed_stage, l7::stage::initialise);
  zassert_equal(operation.port_errno, -ETIMEDOUT);
  zassert_equal(operation.uld_status, VL53L7CX_STATUS_OK);
  zassert_equal(sensor_under_test.current, l7::lifecycle::empty);
  zassert_is_null(sensor_under_test.uld.platform.firmware);
}

ZTEST(tof_l7_sensor,
      test_raw_uld_failures_keep_stage_and_use_stable_errno_mapping) {
  struct case_ {
    uint8_t uld;
    int expected;
  } cases[]{
      {VL53L7CX_STATUS_TIMEOUT_ERROR, -ETIMEDOUT},
      {VL53L7CX_STATUS_CORRUPTED_FRAME, -EBADMSG},
      {VL53L7CX_STATUS_INVALID_PARAM, -EINVAL},
      {VL53L7CX_STATUS_XTALK_FAILED, -EIO},
      {VL53L7CX_STATUS_ERROR, -EIO},
  };

  for (const auto &test : cases) {
    init_status = test.uld;
    zassert_equal(l7::open(&sensor_under_test, 0x29, &operation),
                  test.expected);
    zassert_equal(operation.failed_stage, l7::stage::initialise);
    zassert_equal(operation.uld_status, test.uld);
    reset_fakes();
  }
}

ZTEST(tof_l7_sensor,
      test_configuration_is_8x8_and_frequency_has_no_hidden_default) {
  zassert_ok(l7::open(&sensor_under_test, 0x29, &operation));
  zassert_equal(l7::configure(&sensor_under_test, 0, &operation), -EINVAL);
  zassert_equal(l7::configure(&sensor_under_test, 16, &operation), -EINVAL);
  zassert_equal(resolution_calls, 0);
  zassert_equal(frequency_calls, 0);

  zassert_ok(l7::configure(&sensor_under_test, 15, &operation));
  zassert_equal(configured_resolution, VL53L7CX_RESOLUTION_8X8);
  zassert_equal(configured_frequency, 15);
  zassert_equal(resolution_calls, 1);
  zassert_equal(frequency_calls, 1);
  zassert_equal(sensor_under_test.current, l7::lifecycle::configured);
}

ZTEST(tof_l7_sensor, test_configuration_stops_at_the_first_failed_stage) {
  zassert_ok(l7::open(&sensor_under_test, 0x29, &operation));
  resolution_status = VL53L7CX_STATUS_ERROR;
  zassert_equal(l7::configure(&sensor_under_test, 10, &operation), -EIO);
  zassert_equal(operation.failed_stage, l7::stage::resolution);
  zassert_equal(frequency_calls, 0);
  zassert_equal(sensor_under_test.current, l7::lifecycle::opened);

  resolution_status = VL53L7CX_STATUS_OK;
  frequency_status = VL53L7CX_STATUS_INVALID_PARAM;
  zassert_equal(l7::configure(&sensor_under_test, 10, &operation), -EINVAL);
  zassert_equal(operation.failed_stage, l7::stage::frequency);
  zassert_equal(sensor_under_test.current, l7::lifecycle::opened);
}

ZTEST(tof_l7_sensor,
      test_lifecycle_order_is_enforced_and_failed_stop_stays_running) {
  zassert_equal(l7::start(&sensor_under_test, &operation), -EPERM);
  zassert_equal(operation.failed_stage, l7::stage::state);
  zassert_ok(l7::open(&sensor_under_test, 0x29, &operation));
  zassert_equal(l7::open(&sensor_under_test, 0x29, &operation), -EPERM);
  zassert_ok(l7::configure(&sensor_under_test, 10, &operation));
  zassert_ok(l7::start(&sensor_under_test, &operation));
  zassert_equal(sensor_under_test.current, l7::lifecycle::running);

  stop_status = VL53L7CX_STATUS_ERROR;
  zassert_equal(l7::stop(&sensor_under_test, &operation), -EIO);
  zassert_equal(sensor_under_test.current, l7::lifecycle::running);
  stop_status = VL53L7CX_STATUS_OK;
  zassert_ok(l7::stop(&sensor_under_test, &operation));
  zassert_equal(sensor_under_test.current, l7::lifecycle::configured);
}

ZTEST(tof_l7_sensor, test_not_ready_is_a_clean_nonblocking_no_sample) {
  open_configure_start();
  output.fresh = true;
  output.distance_mm[0] = 0xA5A5;
  ready_value = 0;
  zassert_ok(l7::read_once(&sensor_under_test, &work, &output, &operation));
  zassert_equal(ready_calls, 1);
  zassert_equal(fetch_calls, 0);
  zassert_false(output.fresh);
  zassert_false(operation.sample_present);
  zassert_equal(operation.failed_stage, l7::stage::none);
}

ZTEST(tof_l7_sensor, test_impossible_ready_values_never_fetch) {
  open_configure_start();
  static constexpr uint8_t invalid[]{2, 3, 4, 255};
  for (const uint8_t ready : invalid) {
    ready_value = ready;
    zassert_equal(l7::read_once(&sensor_under_test, &work, &output, &operation),
                  -EPROTO);
    zassert_equal(operation.failed_stage, l7::stage::ready_check);
    zassert_false(output.fresh);
    zassert_equal(fetch_calls, 0);
  }
}

ZTEST(tof_l7_sensor, test_ready_transport_error_wins_and_does_not_fetch) {
  open_configure_start();
  ready_value = 1;
  injected_port_error = -EIO;
  zassert_equal(l7::read_once(&sensor_under_test, &work, &output, &operation),
                -EIO);
  zassert_equal(operation.failed_stage, l7::stage::ready_check);
  zassert_equal(operation.uld_status, VL53L7CX_STATUS_OK);
  zassert_equal(fetch_calls, 0);
}

ZTEST(tof_l7_sensor,
      test_complete_grid_is_copied_without_policy_or_metadata_claims) {
  open_configure_start();
  ready_value = 1;
  fake_results.nb_target_detected[7] = 0;
  fake_results.distance_mm[7] = 8191;
  fake_results.target_status[7] = 255;

  zassert_ok(l7::read_once(&sensor_under_test, &work, &output, &operation));
  zassert_true(output.fresh);
  zassert_true(operation.sample_present);
  zassert_equal(output.silicon_temperature_degc, 23);
  zassert_equal(output.target_count[0], 1);
  zassert_equal(output.distance_mm[0], 100);
  zassert_equal(output.target_status[0], 5);
  zassert_equal(output.target_count[7], 0);
  zassert_equal(output.distance_mm[7], 8191);
  zassert_equal(output.target_status[7], 255);
}

ZTEST(tof_l7_sensor,
      test_fetch_transport_error_cannot_publish_bytes_the_uld_wrote) {
  open_configure_start();
  ready_value = 1;
  fetch_port_error = -ETIMEDOUT;
  const int rc{l7::read_once(&sensor_under_test, &work, &output, &operation)};
  zassert_equal(rc, -ETIMEDOUT, "rc=%d stage=%u port=%d fetch_calls=%d", rc,
                static_cast<unsigned>(operation.failed_stage),
                operation.port_errno, fetch_calls);
  zassert_equal(operation.failed_stage, l7::stage::fetch);
  zassert_false(output.fresh);
  zassert_equal(output.distance_mm[0], 0);
  zassert_false(operation.sample_present);
}

ZTEST(tof_l7_sensor, test_every_impossible_zone_shape_rejects_the_whole_grid) {
  open_configure_start();
  ready_value = 1;

  fake_results.nb_target_detected[3] = 2;
  zassert_equal(l7::read_once(&sensor_under_test, &work, &output, &operation),
                -EPROTO);
  zassert_equal(operation.failed_stage, l7::stage::copy);
  zassert_false(output.fresh);

  good_results();
  fake_results.nb_target_detected[3] = 0;
  fake_results.target_status[3] = 5;
  zassert_equal(l7::read_once(&sensor_under_test, &work, &output, &operation),
                -EPROTO);

  good_results();
  fake_results.target_status[3] = 255;
  zassert_equal(l7::read_once(&sensor_under_test, &work, &output, &operation),
                -EPROTO);

  good_results();
  fake_results.distance_mm[3] = -1;
  zassert_equal(l7::read_once(&sensor_under_test, &work, &output, &operation),
                -EPROTO);
  zassert_false(output.fresh);
}

ZTEST(tof_l7_sensor,
      test_copy_boundary_cannot_bypass_validation_or_leak_an_old_sample) {
  zassert_equal(l7::copy_raw(nullptr, &output), -EINVAL);
  zassert_false(output.fresh);
  zassert_equal(l7::copy_raw(&fake_results, nullptr), -EINVAL);

  zassert_ok(l7::copy_raw(&fake_results, &output));
  zassert_true(output.fresh);
  fake_results.nb_target_detected[63] = 9;
  zassert_equal(l7::copy_raw(&fake_results, &output), -EPROTO);
  zassert_false(output.fresh);
  zassert_equal(output.distance_mm[0], 0);
}

ZTEST(tof_l7_sensor, test_stage_names_are_pairwise_distinct) {
  constexpr l7::stage stages[]{
      l7::stage::none,       l7::stage::arguments,   l7::stage::state,
      l7::stage::address,    l7::stage::firmware,    l7::stage::initialise,
      l7::stage::resolution, l7::stage::frequency,   l7::stage::start,
      l7::stage::stop,       l7::stage::ready_check, l7::stage::fetch,
      l7::stage::copy,
  };
  for (size_t i{0}; i < ARRAY_SIZE(stages); ++i) {
    zassert_not_equal(strcmp(l7::stage_name(stages[i]), "unknown"), 0);
    for (size_t j{i + 1}; j < ARRAY_SIZE(stages); ++j)
      zassert_not_equal(
          strcmp(l7::stage_name(stages[i]), l7::stage_name(stages[j])), 0);
  }
}
