/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * READ-ONLY readout of storage_partition, for deciding whether its current contents may be erased.
 *
 * WHY THIS EXISTS. dasher1's storage_partition is not blank: the production reader refused the L7
 * blob with `bad_magic`, which by its own definition means "something is there and it is not one of
 * these records". Nothing may be erased until that something is identified, and identifying it means
 * byte-matching the whole region against known images -- not looking at a few sampled words and
 * guessing from pointer shapes, which is how the first attempt went wrong.
 *
 * The existing provisioner already refuses this case (it classifies the region and leaves `foreign`
 * untouched: zero erases, zero writes), so the blocker is not a missing guard. It is that nobody has
 * the bytes.
 *
 * WHY NOT `devmem dump`. It works exactly ONCE per boot. zephyr/subsys/shell/shell.c calls
 * getopt_init() only under CONFIG_SHELL_GETOPT, which this project does not set, so the global
 * optind is never reset and the second invocation reports "'-a <address>' is mandatory". The bench
 * overlay now sets that option, which fixes devmem too -- but this command is still the better tool
 * for the job: it reads through the flash API rather than a hand-computed memory-mapped address, it
 * yields between chunks instead of holding the shell thread for the whole region, and it can hash
 * the region on the device so a host-side reassembly can be checked against it.
 *
 * WHAT IT CANNOT DO. There is no erase and no write here, of storage or of anything else. Note the
 * precise claim: the IMAGE still contains flash write code, because firmware_updater.cpp writes
 * slot1 for the CAN DFU. What this file guarantees is that no command it registers modifies
 * storage_partition.
 */

#if defined(ENABLE_TOF_PACKAGE_BENCH)

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/storage/flash_map.h>

#include <tinycrypt/constants.h>
#include <tinycrypt/sha256.h>

namespace lexxhard::tof_storage_readout {
namespace {

constexpr size_t kPartitionSize{DT_REG_SIZE(DT_NODELABEL(storage_partition))};
constexpr off_t kPartitionOffset{DT_REG_ADDR(DT_NODELABEL(storage_partition))};

/* One flash read per line of output, and the line is 16 bytes, so this is the whole working set.
 * Deliberately small and on the stack: see the stack accounting in overlays/tof_package_bench.conf,
 * which is why nothing here allocates a kilobyte. */
constexpr size_t kLineBytes{16};

/* PACING, AND WHY IT IS STILL HERE AFTER THE REASON FOR IT TURNED OUT TO BE WRONG.
 *
 * The honest history, because the wrong version of it is already in the commit that added these
 * arguments. A dump paced at 16 lines per 1 ms pause looked like it lost exactly the last 6 lines of
 * every 16-line group, identically on repeated runs, and that was read as the host-side socat/UDP
 * multiplexer overrunning. It was not. The host-side PARSER was wrong twice: shell_hexdump_line
 * prints addresses in UPPERCASE hex and the parser matched [0-9a-f], so every line whose address
 * contained A-F was discarded -- which is exactly the last six of each group -- and the two groups of
 * eight bytes are separated by TWO spaces, which the parser's single optional space did not accept.
 * Nothing was ever lost on the wire. A re-read of the same capture with a fixed parser gave all
 * 2,048 lines of a 32 KiB chunk with no gaps.
 *
 * So the pause is only ever about the CPU, which is reason enough on its own: 8,192 lines printed in
 * one uninterruptible loop would hold the shell thread for the better part of a minute, and
 * board_controller feeds the independent watchdog on this board, where a reset takes the whole power
 * rail with it.
 *
 * The defaults stay conservative and both numbers stay arguments, because the console path is shared
 * with a multiplexer this repository does not control, and if it ever does drop output the fix must
 * not need another flash. `dump <off> <len> 16 1` restores the fast pacing that was measured to be
 * lossless. */
constexpr size_t kDefaultGroupLines{8};
constexpr int32_t kDefaultPauseMs{60};
constexpr size_t kMaxGroupLines{64};
constexpr int32_t kMaxPauseMs{500};

/* The census and the hash print a few lines in total, so their pacing is only about the CPU. */
constexpr size_t kYieldLines{16};

int read_at(const struct flash_area *fa, size_t off, uint8_t *dst, size_t len)
{
    return flash_area_read(fa, static_cast<off_t>(off), dst, len);
}

/* One place opens and closes, for the reason tof_l7_blob_provider.cpp gives: an early return that
 * skipped flash_area_close() would leak the handle on exactly the failure paths that matter. */
template <typename Fn>
int with_area(const struct shell *sh, Fn &&fn)
{
    const struct flash_area *fa{nullptr};

    if (flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa) != 0) {
        shell_error(sh, "storage partition would not open");
        return -EIO;
    }
    const int rc{fn(fa)};
    flash_area_close(fa);
    return rc;
}

int cmd_info(const struct shell *sh, size_t, char **)
{
    shell_print(sh, "storage_partition: flash offset 0x%lx, size 0x%zx (%zu B), mapped at 0x%lx",
                static_cast<unsigned long>(kPartitionOffset), kPartitionSize, kPartitionSize,
                static_cast<unsigned long>(CONFIG_FLASH_BASE_ADDRESS + kPartitionOffset));

    /* A BLOCK CENSUS, which is the cheap half of the question. "Is the whole region leftovers, or is
     * part of it live?" does not need 128 KiB of hex: it needs to know which 4 KiB blocks are fully
     * erased and which are not. Reported per block so the pattern is visible rather than summarised
     * away -- a single non-erased block in an otherwise blank region is a very different finding
     * from a region that is non-erased throughout. */
    return with_area(sh, [&](const struct flash_area *fa) {
        constexpr size_t kBlock{4096};
        uint8_t buf[kLineBytes]{};
        size_t blank_blocks{0};
        char map[(kPartitionSize / kBlock) + 1]{};

        for (size_t b{0}; b < kPartitionSize / kBlock; ++b) {
            bool blank{true};
            for (size_t off{b * kBlock}; off < (b + 1) * kBlock; off += kLineBytes) {
                if (read_at(fa, off, buf, sizeof(buf)) != 0) {
                    shell_error(sh, "read failed at 0x%zx", off);
                    return -EIO;
                }
                for (size_t i{0}; i < sizeof(buf); ++i) {
                    if (buf[i] != 0xffU) {
                        blank = false;
                        break;
                    }
                }
                if (!blank)
                    break;
            }
            map[b] = blank ? '.' : 'X';
            if (blank)
                ++blank_blocks;
            if ((b % kYieldLines) == (kYieldLines - 1))
                k_msleep(1);
        }
        shell_print(sh, "4 KiB blocks, '.' = all 0xff, 'X' = has data:");
        shell_print(sh, "  %s", map);
        shell_print(sh, "%zu of %zu blocks are fully erased", blank_blocks, kPartitionSize / kBlock);
        return 0;
    });
}

int cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
    /* Positional rather than getopt, and not because getopt is broken here -- the bench overlay now
     * sets CONFIG_SHELL_GETOPT -- but because this command must keep working in an image that does
     * not. A read-only forensic tool that stops working on its second invocation is worse than one
     * with a clumsy signature. */
    const size_t off{argc > 1 ? strtoul(argv[1], nullptr, 16) : 0U};
    const size_t len{argc > 2 ? strtoul(argv[2], nullptr, 16) : 0x1000U};
    const size_t group{argc > 3 ? strtoul(argv[3], nullptr, 10) : kDefaultGroupLines};
    const int32_t pause{argc > 4 ? static_cast<int32_t>(strtol(argv[4], nullptr, 10))
                                 : kDefaultPauseMs};

    if (off >= kPartitionSize || len == 0U || len > 0x8000U || off + len > kPartitionSize) {
        shell_error(sh, "usage: pkg_storage dump [hex offset] [hex len <= 0x8000] [lines/pause] "
                        "[pause ms], region is 0x%zx",
                    kPartitionSize);
        return -EINVAL;
    }
    if (group == 0U || group > kMaxGroupLines || pause < 0 || pause > kMaxPauseMs) {
        shell_error(sh, "pacing out of range: lines 1-%zu, pause 0-%d ms", kMaxGroupLines,
                    kMaxPauseMs);
        return -EINVAL;
    }

    return with_area(sh, [&](const struct flash_area *fa) {
        uint8_t buf[kLineBytes]{};
        size_t lines{0};

        for (size_t p{off}; p < off + len; p += kLineBytes) {
            const size_t n{(off + len - p) < kLineBytes ? (off + len - p) : kLineBytes};
            if (read_at(fa, p, buf, n) != 0) {
                shell_error(sh, "read failed at 0x%zx", p);
                return -EIO;
            }
            /* The SAME formatting `devmem dump` produces, deliberately: a host-side parser written
             * for one must read the other without a second code path. The offset printed is the
             * MAPPED address, so a dump can be compared with a memory-mapped read of the same
             * bytes. */
            shell_hexdump_line(sh, static_cast<unsigned int>(CONFIG_FLASH_BASE_ADDRESS +
                                                            kPartitionOffset + p),
                               buf, n);
            if ((++lines % group) == 0U)
                k_msleep(pause);
        }
        return 0;
    });
}

int cmd_sha(const struct shell *sh, size_t, char **)
{
    /* THE INDEPENDENT CHECK on a host-side reassembly. The dump goes out over a UART behind a socat
     * mux that broadcasts over UDP, and losing a line there is silent. Hashing on the device gives
     * the host something to prove its reassembled file against, rather than trusting that no line
     * went missing. tinycrypt is already in every L7 image for the blob digest. */
    return with_area(sh, [&](const struct flash_area *fa) {
        struct tc_sha256_state_struct st{};
        uint8_t buf[kLineBytes]{};
        uint8_t digest[32]{};

        if (tc_sha256_init(&st) != TC_CRYPTO_SUCCESS) {
            shell_error(sh, "sha init failed");
            return -EIO;
        }
        for (size_t off{0}; off < kPartitionSize; off += kLineBytes) {
            if (read_at(fa, off, buf, sizeof(buf)) != 0) {
                shell_error(sh, "read failed at 0x%zx", off);
                return -EIO;
            }
            if (tc_sha256_update(&st, buf, sizeof(buf)) != TC_CRYPTO_SUCCESS) {
                shell_error(sh, "sha update failed at 0x%zx", off);
                return -EIO;
            }
            if (((off / kLineBytes) % kYieldLines) == (kYieldLines - 1))
                k_msleep(1);
        }
        if (tc_sha256_final(digest, &st) != TC_CRYPTO_SUCCESS) {
            shell_error(sh, "sha final failed");
            return -EIO;
        }
        shell_fprintf(sh, SHELL_NORMAL, "sha256(storage_partition, 0x%zx B) = ", kPartitionSize);
        for (size_t i{0}; i < sizeof(digest); ++i)
            shell_fprintf(sh, SHELL_NORMAL, "%02x", digest[i]);
        shell_fprintf(sh, SHELL_NORMAL, "\n");
        return 0;
    });
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pkg_storage,
    SHELL_CMD(info, NULL, "partition geometry and a per-4-KiB-block erased/not census", cmd_info),
    SHELL_CMD_ARG(dump, NULL,
                  "dump [hex off] [hex len<=0x8000] [lines/pause=8] [pause ms=60] - read-only hexdump",
                  cmd_dump, 1, 4),
    SHELL_CMD(sha, NULL, "sha256 of the whole region, to check a host-side reassembly", cmd_sha),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(pkg_storage, &sub_pkg_storage,
                   "storage_partition READ-ONLY readout (BENCH BUILD ONLY; never erases or writes)",
                   NULL);

}  // namespace
}  // namespace lexxhard::tof_storage_readout

#endif  // ENABLE_TOF_PACKAGE_BENCH
