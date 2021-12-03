#include <zephyr.h>
#include "rosdiagnostic.hpp"

k_msgq msgq_rosdiag;

namespace {

char __aligned(4) msgq_rosdiag_buffer[8 * sizeof (msg_rosdiag)];

}

void rosdiagnostic::init()
{
    k_msgq_init(&msgq_rosdiag, msgq_rosdiag_buffer, sizeof (msg_rosdiag), 8);
}

// vim: set expandtab shiftwidth=4:
