#include <zephyr.h>
#include "rosdiagnostic.hpp"


namespace lexxfirm::rosdiagnostic {

char __aligned(4) msgq_buffer[8 * sizeof (msg)];

void init()
{
    k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
}

k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
