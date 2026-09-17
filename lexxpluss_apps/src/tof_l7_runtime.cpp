/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_l7_runtime.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_L7_ULD)

#include <errno.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "vl53l7cx_blob_expectation.hpp"

namespace lexxhard::tof_l7_runtime {

LOG_MODULE_REGISTER(tof_l7_runtime);

namespace {

atomic_t stage_{static_cast<atomic_val_t>(stage::not_started)};
tof_l7_blob::report report_{};
const uint8_t *firmware_{nullptr};
size_t firmware_size_{0};

int refusal_errno(tof_l7_blob::status st)
{
    using tof_l7_blob::status;

    switch (st) {
    case status::ok:                  return 0;
    case status::absent:              return -ENOENT;
    case status::uncommitted:         return -EAGAIN;
    case status::bad_magic:
    case status::bad_header:
    case status::digest_mismatch:     return -EBADMSG;
    case status::unsupported_format:
    case status::length_out_of_range:
    case status::length_mismatch:
    case status::version_mismatch:
    case status::no_expectation:      return -EPROTO;
    case status::mapping_mismatch:    return -EFAULT;
    case status::unreadable:          return -EIO;
    }
    return -EIO;
}

int bootstrap_impl(verifier fn, const tof_l7_blob::accept_list &accepted)
{
    if (!atomic_cas(&stage_, static_cast<atomic_val_t>(stage::not_started),
                    static_cast<atomic_val_t>(stage::verifying)))
        return -EALREADY;

    /* Zephyr atomic operations are a full barrier. Write all ordinary state before publishing the
     * terminal stage; shell and future L7 threads read the atomic stage first and never touch these
     * fields while it says `verifying`, so there is one publication point rather than a lock around
     * immutable state. */
    report_ = tof_l7_blob::report{};
    firmware_ = nullptr;
    firmware_size_ = 0;

    tof_l7_blob::blob_view view{};
    report_ = fn(accepted, view);

    if (report_.st == tof_l7_blob::status::ok && !view.valid()) {
        /* The production provider is mapped. An `ok` without a pointer would prove one access path
         * but leave the ULD nothing proved to consume, so it is a refusal rather than a degraded
         * success. */
        report_.st = tof_l7_blob::status::mapping_mismatch;
    }

    if (report_.st != tof_l7_blob::status::ok) {
        atomic_set(&stage_, static_cast<atomic_val_t>(stage::refused));
        LOG_ERR("L7 device firmware unavailable: %s", tof_l7_blob::status_name(report_.st));
        return refusal_errno(report_.st);
    }

    firmware_ = view.data();
    firmware_size_ = view.size();
    atomic_set(&stage_, static_cast<atomic_val_t>(stage::available));
    LOG_INF("verified L7 device firmware: %zu bytes", firmware_size_);
    return 0;
}

}  // namespace

int bootstrap()
{
    return bootstrap_impl(tof_l7_blob::verify_stored, tof_l7_blob::kAcceptedPayloadList);
}

snapshot current()
{
    snapshot out{};

    out.current_stage = static_cast<stage>(atomic_get(&stage_));
    if (out.current_stage == stage::available || out.current_stage == stage::refused) {
        out.verification = report_;
        out.firmware_available = out.current_stage == stage::available;
        out.firmware_size = out.firmware_available ? firmware_size_ : 0;
    }
    return out;
}

const char *stage_name(stage s)
{
    switch (s) {
    case stage::not_started: return "not_started";
    case stage::verifying:   return "verifying";
    case stage::available:   return "available";
    case stage::refused:     return "refused";
    }
    return "?";
}

const uint8_t *firmware_data()
{
    return static_cast<stage>(atomic_get(&stage_)) == stage::available ? firmware_ : nullptr;
}

size_t firmware_size()
{
    return static_cast<stage>(atomic_get(&stage_)) == stage::available ? firmware_size_ : 0;
}

#ifdef CONFIG_ZTEST
int bootstrap_for_test(verifier fn, const tof_l7_blob::accept_list &accepted)
{
    if (fn == nullptr)
        return -EINVAL;
    return bootstrap_impl(fn, accepted);
}

void reset_for_test()
{
    /* Tests call this only from their fixture, before any worker exists. It is deliberately absent
     * from production: a second verification after L7 objects have retained the first pointer would
     * revoke a value behind their backs. */
    report_ = tof_l7_blob::report{};
    firmware_ = nullptr;
    firmware_size_ = 0;
    atomic_set(&stage_, static_cast<atomic_val_t>(stage::not_started));
}
#endif

}  // namespace lexxhard::tof_l7_runtime

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_L7_ULD
