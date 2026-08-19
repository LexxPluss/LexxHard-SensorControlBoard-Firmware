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
 * The base comes from the devicetree rather than from a constant, and verify() compares the mapped
 * bytes against flash_area_read before returning the pointer. A wrong but in-range base therefore
 * refuses with mapping_mismatch instead of pushing unrelated flash into a sensor. The compile-time
 * bound below also prevents a partition outside the mapped flash region.
 *
 * Why this is sound at all: internal flash on this part is memory-mapped and readable while code
 * executes from it. Only erase and program stall the bus, and nothing in this path does either. */
constexpr size_t kPartitionOffset{FIXED_PARTITION_OFFSET(storage_partition)};
constexpr size_t kPartitionSize{FIXED_PARTITION_SIZE(storage_partition)};
constexpr uintptr_t kFlashBase{DT_REG_ADDR(DT_NODELABEL(flash0))};
static_assert(kPartitionOffset + kPartitionSize <= DT_REG_SIZE(DT_NODELABEL(flash0)),
              "storage_partition falls outside the memory-mapped flash0 region");

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
    /* flash_area_read is one side of a two-path check. verify() compares every payload chunk read
     * here with the memory-mapped bytes it is about to authorise, so a wrong base cannot pass by
     * proving one access path and handing out another. */
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

report verify_stored(const accept_list &accepted, blob_view &out)
{
    return with_area([&](const reader &r, report &rep) {
        /* The header first and separately, so that `stored` is populated even when the verdict is a
         * refusal. "wanted 86,016 with digest ab.., found 84,992 with digest cd.." is the sentence
         * somebody needs; the status alone cannot say it. */
        (void)read_header(r, kPartitionSize, rep.stored);
        rep.st = verify(r, kPartitionSize, accepted, out, mapped_base());
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
