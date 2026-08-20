/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_l7_sensor.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_L7_ULD)

#include <errno.h>
#include <string.h>

extern "C" {
#include "vl53l7cx_port.h"
}

#include "tof_l7_runtime.hpp"

namespace lexxhard::tof_l7 {

namespace {

void reset(operation_status &status) {
  status.failed_stage = stage::none;
  status.port_errno = 0;
  status.uld_status = VL53L7CX_STATUS_OK;
  status.sample_present = false;
}

void clear_sample(sample &out) {
  out.fresh = false;
  out.silicon_temperature_degc = 0;
  memset(out.target_count, 0, sizeof(out.target_count));
  memset(out.distance_mm, 0, sizeof(out.distance_mm));
  memset(out.target_status, 0, sizeof(out.target_status));
}

int mapped_error(uint8_t uld_status) {
  switch (uld_status) {
  case VL53L7CX_STATUS_OK:
    return 0;
  case VL53L7CX_STATUS_TIMEOUT_ERROR:
    return -ETIMEDOUT;
  case VL53L7CX_STATUS_CORRUPTED_FRAME:
    return -EBADMSG;
  case VL53L7CX_STATUS_INVALID_PARAM:
    return -EINVAL;
  default:
    return -EIO;
  }
}

int finish(operation_status &status, stage failed_stage, uint8_t uld_status) {
  status.uld_status = uld_status;
  status.port_errno = vl53l7cx_port_error();
  if (status.port_errno != 0) {
    status.failed_stage = failed_stage;
    return status.port_errno;
  }

  const int rc{mapped_error(uld_status)};
  if (rc != 0)
    status.failed_stage = failed_stage;
  return rc;
}

int state_refusal(operation_status &status) {
  status.failed_stage = stage::state;
  return -EPERM;
}

} // namespace

const char *stage_name(stage value) {
  switch (value) {
  case stage::none:
    return "none";
  case stage::arguments:
    return "arguments";
  case stage::state:
    return "state";
  case stage::address:
    return "address";
  case stage::firmware:
    return "firmware";
  case stage::initialise:
    return "initialise";
  case stage::resolution:
    return "resolution";
  case stage::frequency:
    return "frequency";
  case stage::start:
    return "start";
  case stage::stop:
    return "stop";
  case stage::ready_check:
    return "ready_check";
  case stage::fetch:
    return "fetch";
  case stage::copy:
    return "copy";
  }
  return "unknown";
}

int open(sensor *device, uint8_t address_7bit, operation_status *status) {
  if (status == nullptr)
    return -EINVAL;
  reset(*status);
  if (device == nullptr) {
    status->failed_stage = stage::arguments;
    return -EINVAL;
  }
  if (device->current != lifecycle::empty)
    return state_refusal(*status);
  if (address_7bit < 0x08U || address_7bit > 0x77U) {
    status->failed_stage = stage::address;
    return -EINVAL;
  }

  const uint8_t *const firmware{tof_l7_runtime::firmware_data()};
  const size_t firmware_size{tof_l7_runtime::firmware_size()};
  if (firmware == nullptr) {
    status->failed_stage = stage::firmware;
    return -ENODATA;
  }
  if (firmware_size < VL53L7CX_FIRMWARE_DOWNLOAD_SIZE) {
    status->failed_stage = stage::firmware;
    return -EPROTO;
  }

  memset(&device->uld, 0, sizeof(device->uld));
  device->uld.platform.address = static_cast<uint16_t>(address_7bit) << 1;
  /* This is the single production assignment of an L7 ULD object's firmware
   * pointer. Tests replace the runtime symbols at link time; there is no
   * alternate adapter API accepting a raw pointer that production code could
   * accidentally call. */
  device->uld.platform.firmware = firmware;
  device->uld.platform.firmware_size = firmware_size;

  vl53l7cx_port_clear_error();
  const int rc{finish(*status, stage::initialise, vl53l7cx_init(&device->uld))};
  if (rc != 0) {
    memset(&device->uld, 0, sizeof(device->uld));
    return rc;
  }

  device->current = lifecycle::opened;
  return 0;
}

int configure(sensor *device, uint8_t frequency_hz, operation_status *status) {
  if (status == nullptr)
    return -EINVAL;
  reset(*status);
  if (device == nullptr) {
    status->failed_stage = stage::arguments;
    return -EINVAL;
  }
  if (device->current != lifecycle::opened)
    return state_refusal(*status);
  if (frequency_hz < 1U || frequency_hz > 15U) {
    status->failed_stage = stage::frequency;
    return -EINVAL;
  }

  vl53l7cx_port_clear_error();
  int rc{
      finish(*status, stage::resolution,
             vl53l7cx_set_resolution(&device->uld, VL53L7CX_RESOLUTION_8X8))};
  if (rc != 0)
    return rc;

  vl53l7cx_port_clear_error();
  rc = finish(*status, stage::frequency,
              vl53l7cx_set_ranging_frequency_hz(&device->uld, frequency_hz));
  if (rc != 0)
    return rc;

  device->current = lifecycle::configured;
  return 0;
}

int start(sensor *device, operation_status *status) {
  if (status == nullptr)
    return -EINVAL;
  reset(*status);
  if (device == nullptr) {
    status->failed_stage = stage::arguments;
    return -EINVAL;
  }
  if (device->current != lifecycle::configured)
    return state_refusal(*status);

  vl53l7cx_port_clear_error();
  const int rc{
      finish(*status, stage::start, vl53l7cx_start_ranging(&device->uld))};
  if (rc == 0)
    device->current = lifecycle::running;
  return rc;
}

int stop(sensor *device, operation_status *status) {
  if (status == nullptr)
    return -EINVAL;
  reset(*status);
  if (device == nullptr) {
    status->failed_stage = stage::arguments;
    return -EINVAL;
  }
  if (device->current != lifecycle::running)
    return state_refusal(*status);

  vl53l7cx_port_clear_error();
  const int rc{
      finish(*status, stage::stop, vl53l7cx_stop_ranging(&device->uld))};
  if (rc == 0)
    device->current = lifecycle::configured;
  return rc;
}

int copy_raw(const VL53L7CX_ResultsData *input, sample *out) {
  if (out == nullptr)
    return -EINVAL;
  clear_sample(*out);
  if (input == nullptr)
    return -EINVAL;

  for (size_t zone{0}; zone < kZoneCount; ++zone) {
    const uint8_t count{input->nb_target_detected[zone]};
    const uint8_t raw_status{input->target_status[zone]};
    const int16_t raw_distance{input->distance_mm[zone]};

    if (count > 1U || ((count == 0U) != (raw_status == 255U)) ||
        raw_distance < 0)
      return -EPROTO;
  }

  out->silicon_temperature_degc = input->silicon_temp_degc;
  for (size_t zone{0}; zone < kZoneCount; ++zone) {
    out->target_count[zone] = input->nb_target_detected[zone];
    out->distance_mm[zone] = static_cast<uint16_t>(input->distance_mm[zone]);
    out->target_status[zone] = input->target_status[zone];
  }
  out->fresh = true;
  return 0;
}

int read_once(sensor *device, scratch *work, sample *out,
              operation_status *status) {
  if (status == nullptr)
    return -EINVAL;
  reset(*status);
  if (out != nullptr)
    clear_sample(*out);
  if (device == nullptr || work == nullptr || out == nullptr) {
    status->failed_stage = stage::arguments;
    return -EINVAL;
  }
  if (device->current != lifecycle::running)
    return state_refusal(*status);

  uint8_t ready{0};
  vl53l7cx_port_clear_error();
  int rc{finish(*status, stage::ready_check,
                vl53l7cx_check_data_ready(&device->uld, &ready))};
  if (rc != 0)
    return rc;
  if (ready > 1U) {
    status->failed_stage = stage::ready_check;
    return -EPROTO;
  }
  if (ready == 0U)
    return 0;

  memset(&work->results, 0, sizeof(work->results));
  vl53l7cx_port_clear_error();
  rc = finish(*status, stage::fetch,
              vl53l7cx_get_ranging_data(&device->uld, &work->results));
  if (rc != 0)
    return rc;

  rc = copy_raw(&work->results, out);
  if (rc != 0) {
    status->failed_stage = stage::copy;
    clear_sample(*out);
    return rc;
  }

  status->sample_present = true;
  return 0;
}

} // namespace lexxhard::tof_l7

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_L7_ULD
