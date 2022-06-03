#pragma once

#include <zephyr.h>
#include <stdint.h>

namespace lexxhard::firmware_updater {

struct packet_array {
  uint8_t data[256 + 4];
};

struct response_array {
  uint16_t data[2];
};

void init();
void run(void *p1, void *p2, void *p3);
extern k_thread thread;
extern k_msgq msgq_data, msgq_response;

}
