/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_l7_blob_provider.hpp"

#if defined(ENABLE_TOF_CHAIN)

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

namespace lexxhard::tof_l7_blob {

LOG_MODULE_REGISTER(tof_l7_blob);

namespace {

/* The partition the blob lives in, and the address it is visible at.
 *
 * The base comes from the devicetree rather than from a constant, because a wrong base here does not
 * fail: it reads whatever is at some other address, and 84 KiB of the wrong flash would be pushed
 * into a sensor as device firmware. The offset comes from the same partition label the reads use, so
 * the two cannot describe different regions.
 *
 * Why this is sound at all: internal flash on this part is memory-mapped and readable while code
 * executes from it. Only erase and program stall the bus, and nothing in this path does either. */
constexpr size_t kPartitionOffset{FIXED_PARTITION_OFFSET(storage_partition)};
constexpr size_t kPartitionSize{FIXED_PARTITION_SIZE(storage_partition)};
constexpr uintptr_t kFlashBase{DT_REG_ADDR(DT_NODELABEL(flash0))};

const uint8_t *mapped_base()
{
    return reinterpret_cast<const uint8_t *>(kFlashBase + kPartitionOffset);
}

struct area_ctx {
    const struct flash_area *fa{nullptr};
};

int read_area(void *ctx, size_t offset, void *dst, size_t len)
{
    auto *a{static_cast<area_ctx *>(ctx)};

    if (a == nullptr || a->fa == nullptr)
        return -EINVAL;
    /* flash_area_read rather than the mapped pointer, deliberately. The verification must not depend
     * on the mapping being right: reading through the API that owns the partition is what makes a
     * wrong base address show up as a digest mismatch here instead of as a sensor that will not
     * range. The mapping is used only after the bytes have been proven, and for the payload only. */
    return flash_area_read(a->fa, offset, dst, len);
}

/* One place opens, checks and closes, because the two entry points differ only in what they do with
 * the reader -- and an early return that skipped flash_area_close() would leak a partition handle on
 * exactly the failure paths that matter. */
template <typename Fn>
report with_area(Fn &&fn)
{
    report rep{};
    area_ctx ctx{};

    rep.region_size = kPartitionSize;
    if (flash_area_open(FIXED_PARTITION_ID(storage_partition), &ctx.fa) != 0) {
        LOG_ERR("storage partition would not open");
        rep.st = status::unreadable;
        return rep;
    }

    reader r{};

    r.read = read_area;
    r.ctx = &ctx;
    fn(r, rep);
    flash_area_close(ctx.fa);
    return rep;
}

}  // namespace

report verify_stored(const expectation &want, blob_view &out)
{
    return with_area([&](const reader &r, report &rep) {
        /* The header first and separately, so that `stored` is populated even when the verdict is a
         * refusal. "wanted 86,016 with digest ab.., found 84,992 with digest cd.." is the sentence
         * somebody needs; the status alone cannot say it. */
        (void)read_header(r, kPartitionSize, rep.stored);
        rep.st = verify(r, kPartitionSize, want, out, mapped_base());
        if (rep.st != status::ok)
            LOG_ERR("stored L7 blob refused: %s", status_name(rep.st));
    });
}

report stored_header()
{
    return with_area([&](const reader &r, report &rep) {
        rep.st = read_header(r, kPartitionSize, rep.stored);
    });
}

}  // namespace lexxhard::tof_l7_blob

#endif  // ENABLE_TOF_CHAIN
