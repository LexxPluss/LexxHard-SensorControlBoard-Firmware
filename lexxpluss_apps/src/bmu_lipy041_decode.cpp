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
#include "bmu_lipy041_decode.hpp"

namespace lexxhard::bmu_lipy041 {

// Every LIPY041 List# frame is a fixed 8 bytes (datasheet section 2); dlc != 8
// means the frame is short (or corrupted) and must not be decoded.
namespace {
constexpr uint8_t EXPECTED_DLC{8};
}

bool decode_0x100(const uint8_t (&data)[8], uint8_t dlc, msg_0x100 &msg) {
    if (dlc != EXPECTED_DLC) {
        return false;
    }
    msg.fail_status1 = data[0];
    msg.leader_battery_status = data[1];
    msg.asoc_min = data[2];
    msg.rsoc_min = data[3];
    msg.soh_min = data[4];
    msg.max_fet_temp = static_cast<int16_t>((data[5] << 8) | data[6]);
    return true;
}

bool decode_0x101(const uint8_t (&data)[8], uint8_t dlc, msg_0x101 &msg) {
    if (dlc != EXPECTED_DLC) {
        return false;
    }
    msg.average_current = static_cast<int16_t>((data[0] << 8) | data[1]);
    msg.max_charging_current = static_cast<uint16_t>((data[2] << 8) | data[3]);
    msg.bm_voltage_max = static_cast<uint16_t>((data[4] << 8) | data[5]);
    msg.fail_status2 = data[6];
    return true;
}

bool decode_0x103(const uint8_t (&data)[8], uint8_t dlc, msg_0x103 &msg) {
    if (dlc != EXPECTED_DLC) {
        return false;
    }
    msg.design_capacity = static_cast<uint16_t>((data[0] << 8) | data[1]);
    msg.fcc_min = static_cast<uint16_t>((data[2] << 8) | data[3]);
    msg.rc_min = static_cast<uint16_t>((data[4] << 8) | data[5]);
    msg.fet_status = data[6];
    return true;
}

bool decode_0x110(const uint8_t (&data)[8], uint8_t dlc, msg_0x110 &msg) {
    if (dlc != EXPECTED_DLC) {
        return false;
    }
    msg.max_voltage.value = static_cast<uint16_t>((data[0] << 8) | data[1]);
    msg.max_voltage.id = data[2];
    msg.min_voltage.value = static_cast<uint16_t>((data[4] << 8) | data[5]);
    msg.min_voltage.id = data[6];
    return true;
}

bool decode_0x111(const uint8_t (&data)[8], uint8_t dlc, msg_0x111 &msg) {
    if (dlc != EXPECTED_DLC) {
        return false;
    }
    msg.max_temp.value = static_cast<int16_t>((data[0] << 8) | data[1]);
    msg.max_temp.id = data[2];
    msg.min_temp.value = static_cast<int16_t>((data[4] << 8) | data[5]);
    msg.min_temp.id = data[6];
    return true;
}

bool decode_0x112(const uint8_t (&data)[8], uint8_t dlc, msg_0x112 &msg) {
    if (dlc != EXPECTED_DLC) {
        return false;
    }
    msg.max_current.value = static_cast<int16_t>((data[0] << 8) | data[1]);
    msg.max_current.id = data[2];
    msg.min_current.value = static_cast<int16_t>((data[4] << 8) | data[5]);
    msg.min_current.id = data[6];
    return true;
}

bool decode_0x113(const uint8_t (&data)[8], uint8_t dlc, msg_0x113 &msg) {
    if (dlc != EXPECTED_DLC) {
        return false;
    }
    msg.fw_ver = data[0];
    msg.data_ver = data[1];
    // data[2] is Reserved
    msg.connected_bm_count = data[3];
    msg.leader_alarm1 = data[4];
    msg.leader_alarm2 = data[5];
    msg.fail_status3 = data[6];
    return true;
}

bool decode_0x120(const uint8_t (&data)[8], uint8_t dlc, msg_0x120 &msg) {
    if (dlc != EXPECTED_DLC) {
        return false;
    }
    msg.max_cell_voltage.value = static_cast<uint16_t>((data[0] << 8) | data[1]);
    msg.max_cell_voltage.id = data[2];
    msg.min_cell_voltage.value = static_cast<uint16_t>((data[4] << 8) | data[5]);
    msg.min_cell_voltage.id = data[6];
    return true;
}

bool decode_0x130(const uint8_t (&data)[8], uint8_t dlc, msg_0x130 &msg) {
    if (dlc != EXPECTED_DLC) {
        return false;
    }
    msg.manufacturing = static_cast<uint16_t>((data[0] << 8) | data[1]);
    msg.inspection = static_cast<uint16_t>((data[2] << 8) | data[3]);
    msg.serial = static_cast<uint16_t>((data[4] << 8) | data[5]);
    return true;
}

bool decode_0x131(const uint8_t (&data)[8], uint8_t dlc, msg_0x131 &msg) {
    if (dlc != EXPECTED_DLC) {
        return false;
    }
    msg.accumulated_capacity = (static_cast<uint32_t>(data[0]) << 24) |
                                (static_cast<uint32_t>(data[1]) << 16) |
                                (static_cast<uint32_t>(data[2]) << 8) |
                                static_cast<uint32_t>(data[3]);
    return true;
}

bool decode_frame_bmu_info(uint32_t id, const uint8_t (&data)[8], uint8_t dlc, msg_bmu &msg) {
    if (id == 0x100) {
        return decode_0x100(data, dlc, msg.f100);
    } else if (id == 0x101) {
        return decode_0x101(data, dlc, msg.f101);
    } else if (id == 0x103) {
        return decode_0x103(data, dlc, msg.f103);
    } else if (id == 0x110) {
        return decode_0x110(data, dlc, msg.f110);
    } else if (id == 0x111) {
        return decode_0x111(data, dlc, msg.f111);
    } else if (id == 0x112) {
        return decode_0x112(data, dlc, msg.f112);
    } else if (id == 0x113) {
        return decode_0x113(data, dlc, msg.f113);
    } else if (id == 0x120) {
        return decode_0x120(data, dlc, msg.f120);
    } else if (id == 0x130) {
        return decode_0x130(data, dlc, msg.f130);
    } else if (id == 0x131) {
        return decode_0x131(data, dlc, msg.f131);
    }
    return true;  // unrecognized id: nothing to decode, not an error
}

bool decode_frame_power_sequence(uint32_t id, const uint8_t (&data)[8], uint8_t dlc,
                                  msg_0x100 &f100, msg_0x101 &f101, msg_0x113 &f113) {
    if (id == 0x100) {
        return decode_0x100(data, dlc, f100);
    } else if (id == 0x101) {
        return decode_0x101(data, dlc, f101);
    } else if (id == 0x113) {
        return decode_0x113(data, dlc, f113);
    }
    return true;  // unrecognized id: nothing to decode, not an error
}

bool is_ok(const msg_0x100 &f100, const msg_0x101 &f101, const msg_0x113 &f113) {
    // AND: true only if no field has an abnormal bit. The original LIA1020
    // implementation (board_controller.cpp:902-905) used OR, which is only false
    // when all four fields are abnormal simultaneously -- inconsistent with
    // !is_ok() being used as the power-shutdown trigger at 7 call sites. Fixed to AND.
    return ((f100.fail_status1 & FAIL_STATUS1_ABNORMAL_MASK) == 0 &&
            (f101.fail_status2 & FAIL_STATUS2_ABNORMAL_MASK) == 0 &&
            (f113.fail_status3 & FAIL_STATUS3_ABNORMAL_MASK) == 0 &&
            (f113.leader_alarm1 & LEADER_ALARM1_ABNORMAL_MASK) == 0 &&
            (f113.leader_alarm2 & LEADER_ALARM2_ABNORMAL_MASK) == 0);
}

bool is_full_charge(const msg_0x100 &f100) {
    return (f100.fail_status1 & 0b01000000) != 0;
}

bool is_chargable(const msg_0x100 &f100, const msg_0x101 &f101) {
    // f101 (e.g. fail_status2 charge-overcurrent/overtemp/overcharge bits) is not
    // consulted here: callers already gate on is_ok() before reaching is_chargable(),
    // so those conditions are caught upstream. Signature kept for interface stability.
    (void)f101;
    return !is_full_charge(f100) && f100.rsoc_min < 95;
}

bool is_charging(const msg_0x101 &f101) {
    return f101.average_current > 0;
}

post_result decide_post_transition(const msg_0x100 &f100, const msg_0x101 &f101, const msg_0x113 &f113,
                                    bool switch_released, int64_t elapsed_ms) {
    if (is_ok(f100, f101, f113) && switch_released) {
        return post_result::standby;
    }
    if (!is_ok(f100, f101, f113) && elapsed_ms > POST_TIMEOUT_MS) {
        return post_result::off;
    }
    return post_result::wait;
}

namespace {
constexpr uint8_t NOT_RECEIVED_SENTINEL{0xff};
}

field_health describe_fail_status1(const msg_0x100 &f100) {
    if (f100.fail_status1 == NOT_RECEIVED_SENTINEL) {
        return field_health::not_received;
    }
    return (f100.fail_status1 & FAIL_STATUS1_ABNORMAL_MASK) == 0 ? field_health::ok : field_health::abnormal;
}

field_health describe_fail_status2(const msg_0x101 &f101) {
    if (f101.fail_status2 == NOT_RECEIVED_SENTINEL) {
        return field_health::not_received;
    }
    return (f101.fail_status2 & FAIL_STATUS2_ABNORMAL_MASK) == 0 ? field_health::ok : field_health::abnormal;
}

field_health describe_fail_status3(const msg_0x113 &f113) {
    if (f113.fail_status3 == NOT_RECEIVED_SENTINEL) {
        return field_health::not_received;
    }
    return (f113.fail_status3 & FAIL_STATUS3_ABNORMAL_MASK) == 0 ? field_health::ok : field_health::abnormal;
}

field_health describe_leader_alarm1(const msg_0x113 &f113) {
    if (f113.leader_alarm1 == NOT_RECEIVED_SENTINEL) {
        return field_health::not_received;
    }
    return (f113.leader_alarm1 & LEADER_ALARM1_ABNORMAL_MASK) == 0 ? field_health::ok : field_health::abnormal;
}

field_health describe_leader_alarm2(const msg_0x113 &f113) {
    if (f113.leader_alarm2 == NOT_RECEIVED_SENTINEL) {
        return field_health::not_received;
    }
    return (f113.leader_alarm2 & LEADER_ALARM2_ABNORMAL_MASK) == 0 ? field_health::ok : field_health::abnormal;
}

}

// vim: set expandtab shiftwidth=4:
