#include <zephyr.h>
#include <storage/flash_map.h>
#include <devicetree.h>
#include <sys/reboot.h>
#include <stdint.h>
#include "firmware_updater.hpp"
#include "rosserial_hardware_zephyr.hpp"

namespace lexxhard::firmware_updater {

char __aligned(4) msgq_data_buffer[2 * sizeof (packet_array)];
char __aligned(4) msgq_response_buffer[2 * sizeof (response_array)];

class {
public:
  void init() {
      k_msgq_init(&msgq_data, msgq_data_buffer, sizeof (packet_array), 2);
      k_msgq_init(&msgq_response, msgq_response_buffer, sizeof (response_array), 2);
  }
  void cmd() {
    if (k_msgq_get(&msgq_data, packet.data, K_MSEC(1000)) == 0) {
        switch (static_cast<CMD>(packet.data[2])) {
        case CMD::START:
            cmd_start(packet.data);
            break;
        case CMD::DATA:
            cmd_data(packet.data);
            break;
        case CMD::RESET:
            cmd_reset(reboot_option_enabled);
            break;
        case CMD::RESET_NO_REBOOT:
            cmd_reset(!reboot_option_enabled);
            break;
        }
    }
  }
  void failsafe() {
    uint32_t current_cycle{k_cycle_get_32()};
    if (fa != nullptr && dfu_start_cycle != 0) {
        uint32_t dfu_ms_elapsed{k_cyc_to_ms_near32(current_cycle - dfu_start_cycle)};
        if (dfu_ms_elapsed > 90000) {
            flash_area_reset();
            respond(RESP::R_6);
        } 
    }
  }
  void reboot() {
    uint32_t current_cycle{k_cycle_get_32()};
    if (ready_to_reboot && k_cyc_to_ms_near32(current_cycle - dfu_completion_cycle) > 2000) {
        ready_to_reboot = false;
        sys_reboot(SYS_REBOOT_COLD);
    }
  }
private:
  enum class RESP {
      R_0 = 0,
      R_1 = 1,
      R_2 = 2,
      R_3 = 3,
      R_4 = 4,
      R_5 = 5,
      R_6 = 6,
      R_7 = 7,
      R_8 = 8,
  };
  enum class CMD {
      START           = 0,
      DATA            = 1,
      RESET           = 2,
      RESET_NO_REBOOT = 3,
  };
  void cmd_start(const uint8_t *data) {
      if (fa != nullptr)
          flash_area_reset();
      dfu_start_cycle = k_cycle_get_32();
      if (!DT_NODE_EXISTS(DT_NODELABEL(image_0_secondary_partition))) {
          respond(RESP::R_2);
          return;
      }
      if (flash_area_open(FLASH_AREA_ID(image_1), &fa) != 0) {
          respond(RESP::R_3);
          return;
      }
      static constexpr uint32_t LENGTH{0x00040000};
      for (uint32_t current_offset{0}; current_offset < LENGTH; current_offset += 4) {
          uint32_t buf{0};
          flash_area_read(fa, current_offset, &buf, sizeof buf);
          if (buf != 0xffffffff) {
              if (flash_area_erase(fa, 0, LENGTH) != 0) {
                  flash_area_reset();
                  respond(RESP::R_4);
                  return;
              }
          }
      }
      cmd_data(data);
  }
  void cmd_data(const uint8_t *data) {
      if (fa == nullptr) {
          flash_area_reset();
          respond(RESP::R_7);
          return;
      }
      uint8_t checksum{0x0};
      for (int i = 3; i < 259; i++)
          checksum += data[i];
      if (checksum != data[259]) {
          flash_area_reset();
          respond(RESP::R_5);
          return;
      }
      response.data[1] = data[0] + (data[1] << 8);
      uint32_t current_offset{static_cast<uint32_t>(response.data[1]) * 256U};
      if (flash_area_write(fa, current_offset, &data[3], 256) != 0) {
          flash_area_reset();
          respond(RESP::R_6);
          return;
      }
      respond(RESP::R_0);
  }
  void cmd_reset(bool reboot_option_enabled) {
      if (fa == nullptr) {
          respond(RESP::R_7);
          return;
      }
      flash_area_reset();
      dfu_completion_cycle = k_cycle_get_32();
      if (reboot_option_enabled) {
          respond(RESP::R_8);
          ready_to_reboot = true;
          return;
      }
      else {
          respond(RESP::R_1);
          ready_to_reboot = false;
          return;
      }
  }
  void flash_area_reset() {
      if (fa != nullptr) {
          flash_area_close(fa);
          fa = nullptr;
      }
  }
  void respond(RESP resp) {
      response.data[0] = static_cast<uint16_t>(resp);
      k_msgq_put(&msgq_response, response.data, K_MSEC(2000));
  }
  response_array response;
  packet_array packet;
  const struct flash_area *fa{nullptr};
  uint32_t dfu_start_cycle{0};
  uint32_t dfu_completion_cycle{0};
  bool ready_to_reboot{false};
  const bool reboot_option_enabled{true};
} impl;

void init() {
    impl.init();
}

void run(void *p1, void *p2, void *p3) {
    while (true) {
        impl.cmd();
        impl.failsafe();
        impl.reboot();
    }
}

k_thread thread;
k_msgq msgq_data, msgq_response;

}

// vim: set expandtab shiftwidth=4:
