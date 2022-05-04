#include <zephyr.h>
#include <logging/log_backend.h>
#include <logging/log_backend_std.h>
#include <shell/shell.h>

namespace lexxhard::log_controller {

class overwrap_ring_buf {
public:
    void init() {
        index = 0;
        buf[index]    = 0x00;
        buf[SIZE - 1] = 0xff; // overwrap detector
        buf[SIZE]     = 0x00; // null terminator for print to end of buffer
    }
    void put(const uint8_t *data, uint32_t length) {
        while (length--) {
            buf[index] = *data++;
            index = next(index);
        }
        buf[index] = 0x00;
    }
    void show(const shell *shell) const {
        if (buf[SIZE - 1] == 0xff) {
            shell_fprintf(shell, SHELL_NORMAL, "%s", buf);
        } else {
            uint32_t last{skip_to_newline()};
            shell_fprintf(shell, SHELL_NORMAL, "%s", &buf[last]);
            if (last != 0)
                shell_fprintf(shell, SHELL_NORMAL, "%s", buf);
        }
    }
private:
    uint32_t skip_to_newline() const {
        uint32_t last{next(index)};
        for (uint32_t i{last}; i != index; i = next(i)) {
            if (buf[i] == '\n')
                return next(i);
        }
        return last;
    }
    uint32_t next(uint32_t i) const {
         return (i + 1) & (SIZE - 1);
    }
    static constexpr uint32_t SIZE{64 * 1024};
    uint32_t index{0};
    uint8_t buf[SIZE + 1];
} ringbuf;

int write_log_to_mem(uint8_t *data, size_t length, void *ctx)
{
    ringbuf.put(data, length);
    return length;
}

uint8_t __aligned(4) log_output_buf[4 * 1024];
LOG_OUTPUT_DEFINE(log_output, write_log_to_mem, log_output_buf, sizeof log_output_buf);

void logapi_init(const log_backend *const backend)
{
    ringbuf.init();
}

void logapi_put(const log_backend *const backend, log_msg *msg)
{
    log_backend_std_put(&log_output, 0, msg);
}

void logapi_panic(log_backend const *const backend)
{
    log_backend_deactivate(backend);
}

void logapi_dropped(const log_backend *const backend, uint32_t cnt)
{
    log_backend_std_dropped(&log_output, cnt);
}

const log_backend_api log_backend_mem_api{
    .put = logapi_put,
    .dropped = logapi_dropped,
    .panic = logapi_panic,
    .init = logapi_init,
};

LOG_BACKEND_DEFINE(log_backend_mem, log_backend_mem_api, true);

int cmd_show(const shell *shell, size_t argc, char **argv)
{
    ringbuf.show(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub,
    SHELL_CMD(show, NULL, "Show memory log", cmd_show),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(mlog, &sub, "Memory log commands", NULL);

}

// vim: set expandtab shiftwidth=4:
