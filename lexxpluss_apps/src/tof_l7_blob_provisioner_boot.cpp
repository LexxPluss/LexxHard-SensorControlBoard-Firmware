/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_l7_blob_provisioner_boot.hpp"

#if defined(ENABLE_L7_BLOB_PROVISIONER)

#include <errno.h>
#include <stdlib.h>

#include <soc.h>
#include <zephyr/devicetree.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/storage/flash_map.h>

#include "tof_l7_blob_provisioner.hpp"
#include "vl53l7cx_blob_expectation.hpp"

LOG_MODULE_REGISTER(l7_blob_provisioner, LOG_LEVEL_INF);

namespace lexxhard::tof_l7_blob_provisioner_boot {

namespace {

namespace prov = lexxhard::tof_l7_blob_provisioner;
namespace blob = lexxhard::tof_l7_blob;

#define PROV_XSTR(x) #x
#define PROV_STR(x) PROV_XSTR(x)
#ifndef VERSION
#define VERSION v0.0.0-local-build
#endif

/* The uncommitted record: the L7 blob-record generator's `pack` output for the vendored
 * VL53L7CX_FIRMWARE, produced offline by the Makefile before the build and handed to CMake as
 * L7_BLOB_RECORD_IMG -- the same tool and inputs as the archived record.uncommitted.img. The commit
 * marker is deliberately not in it. */
const uint8_t kRecord[] = {
#include "l7_blob_record.inc"
};

constexpr size_t kPartitionOffset{FIXED_PARTITION_OFFSET(storage_partition)};
constexpr size_t kPartitionSize{FIXED_PARTITION_SIZE(storage_partition)};
constexpr uintptr_t kFlashBase{DT_REG_ADDR(DT_NODELABEL(flash0))};

/* THE LAYOUT, PINNED. These are the numbers the reader, the SWD runbook and the capacity budget were
 * all written against. A devicetree that moved the partition, or a record of another size, must fail
 * the build here rather than write somewhere nobody reviewed. */
static_assert(kPartitionOffset == 0x20000, "storage_partition is not at 0x20000");
static_assert(kPartitionSize == 0x20000, "storage_partition is not 128 KiB");
static_assert(kFlashBase + kPartitionOffset == 0x08020000, "storage_partition is not at 0x08020000");
static_assert(sizeof(kRecord) == 86080, "the embedded record is not 64 + 86,016 bytes");
static_assert(sizeof(kRecord) + blob::kCommitMarkerSize <= kPartitionSize,
              "the record and its marker do not fit the partition");

struct area {
    const struct flash_area *fa{nullptr};
};

int f_read(void *ctx, size_t offset, void *dst, size_t len)
{
    return flash_area_read(static_cast<area *>(ctx)->fa, static_cast<off_t>(offset), dst, len);
}

int f_erase(void *ctx, size_t offset, size_t len)
{
    return flash_area_erase(static_cast<area *>(ctx)->fa, static_cast<off_t>(offset), len);
}

/* flash_area_write never erases: this is what makes the marker a program-only operation. */
int f_write(void *ctx, size_t offset, const void *src, size_t len)
{
    return flash_area_write(static_cast<area *>(ctx)->fa, static_cast<off_t>(offset), src, len);
}

prov::config make_config()
{
    prov::config c{};
    c.region_size = kPartitionSize;
    c.record = kRecord;
    c.record_len = sizeof kRecord;
    c.accepted = blob::kAcceptedPayloadList;
    c.mapped_base = reinterpret_cast<const uint8_t *>(kFlashBase + kPartitionOffset);
    return c;
}

/* Where this boot's run is. `waiting` lasts the whole start delay, during which the worker neither
 * reads nor writes the partition and the image is not confirmed. (A manual `l7 provision inspect`
 * still reads it; it never writes.) */
enum class phase : uint8_t { waiting, running, done };

/* What happened to the MCUboot confirm. Only `confirmed` makes this image survive the next reset;
 * every other value leaves it a test image that MCUboot reverts at the next reset. */
enum class confirm_state : uint8_t {
    not_yet,        // the run has not finished
    confirmed,      // verified success, and boot_write_img_confirmed() returned 0
    confirm_failed, // verified success, but the confirm itself failed: NOT a success
    withheld,       // any other ending: deliberately not confirmed
};

/* Two locks. flash_lock serialises partition access, so `inspect` can never read the partition while
 * the run is halfway through programming it. state_lock guards only the small report below, so
 * `status` answers at once even while an erase is in progress. */
K_MUTEX_DEFINE(flash_lock);
K_MUTEX_DEFINE(state_lock);
phase run_phase{phase::waiting};
prov::report last_run{};
confirm_state confirm{confirm_state::not_yet};
int confirm_rc{0};
bool confirmed_at_boot{false};
/* RCC_CSR as found at boot, before this image cleared it. Bits 31..25: LPWR WWDG IWDG SFT POR PIN
 * BOR (RM0410 5.3.21). */
uint32_t reset_flags{0};

/* Opens the partition and checks, at run time, that what flash_area actually opened is what was
 * compiled. The static_asserts pin the devicetree; this pins the flash map it produced. */
int open_checked(area &a)
{
    if (flash_area_open(FIXED_PARTITION_ID(storage_partition), &a.fa) != 0)
        return -ENODEV;
    if (static_cast<size_t>(a.fa->fa_off) != kPartitionOffset || a.fa->fa_size != kPartitionSize) {
        flash_area_close(a.fa);
        a.fa = nullptr;
        return -EFAULT;
    }
    return 0;
}

void worker(void *, void *, void *)
{
    k_mutex_lock(&state_lock, K_FOREVER);
    run_phase = phase::running;
    k_mutex_unlock(&state_lock);

    k_mutex_lock(&flash_lock, K_FOREVER);
    area a{};
    prov::report r{};
    if (const int rc{open_checked(a)}; rc != 0) {
        r.result = prov::outcome::bad_config;
        r.last_errno = rc;
    } else {
        const prov::flash_ops ops{f_read, f_erase, f_write, &a};
        r = prov::run(ops, make_config());
        flash_area_close(a.fa);
    }
    k_mutex_unlock(&flash_lock);

    /* THE CONFIRM, and the only one in this image. Only a run whose last step -- the production
     * reader, blob::verify with the production accept-list, reading the partition through
     * flash_area and checking every payload byte against the memory-mapped address the L7 image
     * will hand the sensor -- accepted the partition makes this image permanent. A refusal changed
     * nothing, so there is nothing to keep; a failure may have left a partial record, and the
     * previous image, which never reads the partition, is the right thing to run while somebody
     * looks at it. Neither reboots: the reset that reverts is left to whatever comes next. */
    confirm_state cs{confirm_state::withheld};
    int crc{0};
    if (r.ok() && r.final_status == blob::status::ok) {
        crc = boot_write_img_confirmed();
        cs = crc == 0 ? confirm_state::confirmed : confirm_state::confirm_failed;
    }

    k_mutex_lock(&state_lock, K_FOREVER);
    last_run = r;
    confirm = cs;
    confirm_rc = crc;
    run_phase = phase::done;
    k_mutex_unlock(&state_lock);

    LOG_INF("%s, found %s, reader %s, confirm %d rc %d", prov::outcome_name(r.result),
            prov::found_name(r.initial), blob::status_name(r.final_status), static_cast<int>(cs), crc);
}

K_THREAD_STACK_DEFINE(worker_stack, 4096);
struct k_thread worker_thread;

/* Ten minutes after main() gets here. The delay is not for the SCB's own controllers, CAN or the
 * firmware updater, which are running within seconds. It is for three things outside the board:
 *   - A reset caused by the update itself -- the machine power-cycles after a CAN DFU, and when a
 *     second SCB reset follows, if one does, has not been measured -- lands while the worker has not
 *     touched the partition and the image is unconfirmed, so MCUboot reverts to the previous image
 *     before a byte of storage has changed.
 *   - The robot PC, which takes about three minutes to come back after that power cycle, is up and
 *     logging, and somebody can be watching the shell, before the one risky window opens.
 *   - If this image must not run -- inspect found something unexpected, or the unconfirmed image
 *     did not revert as it should -- there is time to put the previous image back by CAN DFU before
 *     the worker starts: about three minutes for the PC to return plus about 2 min 45 s for the DFU
 *     and its power cycle is already more than five minutes, so five would have left the rollback
 *     racing the first erase.
 * It does nothing for a reset that arrives once the erase has started; that is what the marker-last
 * ordering and the two stated unrecoverable windows in tof_l7_blob_provisioner.hpp are for.
 *
 * An erase stalls the flash bank for about a second, well inside the 10 s IWDG timeout, and the
 * existing DFU already erases a larger 256 KiB sector on this board on every update. */
constexpr int kStartDelayMs{600'000};
/* Below every existing thread: the run is the least urgent thing on the board. */
constexpr int kPriority{9};

void print_digest(const struct shell *sh, const uint8_t *d)
{
    char hex[2 * blob::kDigestSize + 1];
    for (size_t i{0}; i < blob::kDigestSize; ++i)
        snprintk(hex + 2 * i, 3, "%02x", d[i]);
    shell_print(sh, "digest %s", hex);
}

/* Terse on purpose: every string here is flash in an image with no room to spare. The enum values
 * are the order they are declared in. */
int cmd_status(const struct shell *sh, size_t, char **)
{
    k_mutex_lock(&state_lock, K_FOREVER);
    const phase ph{run_phase};
    const prov::report r{last_run};
    const confirm_state cs{confirm};
    const int crc{confirm_rc};
    k_mutex_unlock(&state_lock);

    shell_print(sh, "%s DEV L7-BLOB-PROVISIONER NOT-FOR-RELEASE", PROV_STR(VERSION));
    shell_print(sh, "storage 0x%08x+0x%x record %u", static_cast<unsigned>(kFlashBase + kPartitionOffset),
                static_cast<unsigned>(kPartitionSize), static_cast<unsigned>(sizeof kRecord));
    print_digest(sh, kRecord + 12);
    shell_print(sh, "reset 0x%08x up %us confirmed_at_boot %d", static_cast<unsigned>(reset_flags),
                static_cast<unsigned>(k_uptime_get() / 1000), confirmed_at_boot);
    shell_print(sh, "phase %d (0 waiting until %ds, 1 running, 2 done)", static_cast<int>(ph),
                kStartDelayMs / 1000);
    if (ph != phase::done)
        return 0;
    shell_print(sh, "found %s outcome %s reader %s", prov::found_name(r.initial),
                prov::outcome_name(r.result), blob::status_name(r.final_status));
    shell_print(sh, "erase_attempts %u write_attempts %u readbacks %u flash_op_attempted %d",
                static_cast<unsigned>(r.erase_attempts), static_cast<unsigned>(r.write_attempts),
                static_cast<unsigned>(r.readbacks), r.flash_operation_attempted);
    shell_print(sh, "errno %d offset 0x%x", r.last_errno, static_cast<unsigned>(r.failed_offset));
    shell_print(sh, "confirm %d rc %d (1 confirmed, 2 FAILED, 3 withheld)", static_cast<int>(cs), crc);
    return 0;
}

/* Read-only, now: classify the partition again and ask the production reader. Never writes. */
int cmd_inspect(const struct shell *sh, size_t, char **)
{
    k_mutex_lock(&flash_lock, K_FOREVER);
    area a{};
    int rc{open_checked(a)};
    prov::found f{prov::found::unreadable};
    blob::status st{blob::status::unreadable};
    blob::header_info hdr{};
    if (rc == 0) {
        const prov::flash_ops ops{f_read, nullptr, nullptr, &a};
        const prov::config c{make_config()};
        f = prov::classify(ops, c, rc);
        blob::reader rd{};
        rd.read = f_read;
        rd.ctx = &a;
        blob::blob_view view{};
        st = blob::verify(rd, kPartitionSize, c.accepted, view, c.mapped_base);
        (void)blob::read_header(rd, kPartitionSize, hdr);
        flash_area_close(a.fa);
    }
    k_mutex_unlock(&flash_lock);

    shell_print(sh, "found %s (errno %d) reader %s", prov::found_name(f), rc, blob::status_name(st));
    if (hdr.parsed) {
        shell_print(sh, "stored length %u", static_cast<unsigned>(hdr.payload_len));
        print_digest(sh, hdr.payload_digest);
    }
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_provision,
    SHELL_CMD(status, NULL, "this boot's run", cmd_status),
    SHELL_CMD(inspect, NULL, "read-only classify", cmd_inspect),
    SHELL_SUBCMD_SET_END);
SHELL_STATIC_SUBCMD_SET_CREATE(sub_l7, SHELL_CMD(provision, &sub_provision, NULL, NULL),
                               SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(l7, &sub_l7, "L7 blob provisioner (DEV)", NULL);

}  // namespace

void start()
{
    /* The reset cause, captured before anything else can clear it, and then cleared so the next
     * boot reports its own cause rather than the accumulation since power-on. A register, not a
     * persistent write. */
    reset_flags = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
    confirmed_at_boot = boot_is_img_confirmed();

    k_thread_create(&worker_thread, worker_stack, K_THREAD_STACK_SIZEOF(worker_stack), worker,
                    nullptr, nullptr, nullptr, K_PRIO_PREEMPT(kPriority), 0,
                    K_MSEC(kStartDelayMs));
    k_thread_name_set(&worker_thread, "l7_provision");
}

}  // namespace lexxhard::tof_l7_blob_provisioner_boot

#endif  // ENABLE_L7_BLOB_PROVISIONER
