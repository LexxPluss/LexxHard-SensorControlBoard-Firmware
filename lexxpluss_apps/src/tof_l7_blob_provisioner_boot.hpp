/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The DEV provisioner image's one job, wired to the board: DEV ONLY.
 *
 * The rest of the image is the ordinary product runtime -- power sequencing, board controller, CAN,
 * firmware updater, shell -- because the one thing a remotely flashed provisioner must not do is lose
 * the robot's power or its CAN DFU. Only the ToF chain is absent, and in its place this module runs
 * the provisioner once per boot, in its own low-priority thread, five minutes after boot.
 *
 * THE IMAGE IS NOT CONFIRMED BY main(). It is flashed as an unconfirmed test image and confirms
 * itself only after the run ends in already_provisioned or provisioned_and_verified AND the
 * production reader accepts the partition. Any other ending -- a refusal, a failure, a reset in the
 * middle -- leaves it unconfirmed, and the next reset of any kind reverts to the previous image.
 * A fault in the provisioner can therefore not become a confirmed image that resets forever.
 */

#pragma once

#if defined(ENABLE_L7_BLOB_PROVISIONER)

namespace lexxhard::tof_l7_blob_provisioner_boot {

/* Records the reset cause, creates the provisioning thread with its start delay, returns at once. */
void start();

}  // namespace lexxhard::tof_l7_blob_provisioner_boot

#endif  // ENABLE_L7_BLOB_PROVISIONER
