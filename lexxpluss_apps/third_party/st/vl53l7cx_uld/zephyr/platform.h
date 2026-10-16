/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Platform surface consumed by the ST VL53L7CX ULD. This is intentionally smaller than the
 * upstream callback example: the verified device-firmware pointer is per configuration object
 * rather than a process-global array, and the bus is named by the caller rather than compiled in.
 *
 * ST passes this structure to every transfer callback, so everything that belongs to one device
 * belongs here. An earlier revision put the bus and the transport evidence in file-scope statics
 * and justified it with "the ST platform callback carries no context" -- which is true only of
 * VL53L7CX_SwapBuffer(). It made a second L7 on a second bus impossible to add without the two
 * sharing an errno.
 */

#ifndef LEXXPLUSS_VL53L7CX_PLATFORM_H_
#define LEXXPLUSS_VL53L7CX_PLATFORM_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* The frozen grid contract consumes exactly these ULD outputs:
 *
 *   nb_target_detected, distance_mm, target_status.
 *
 * Do not ask the sensor to send fields that are discarded immediately. Besides shrinking
 * VL53L7CX_ResultsData by about 1.1 KiB, the current selection makes
 * VL53L7CX_MAX_RESULTS_SIZE 328 bytes: one transaction within the independently hardware-proven
 * port limit instead of five segmented transactions for the upstream 1,452-byte default. The
 * compile-time check in platform.c deliberately states a <= relationship; the selected fields and
 * the measured bus limit have different owners and must not be made equal by definition. These
 * defines live in platform.h because vl53l7cx_api.h includes it before declaring either the result
 * structure or the output block list; every consumer therefore sees one ABI rather than relying on
 * a matching build-system definition. */
#define VL53L7CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L7CX_DISABLE_NB_SPADS_ENABLED
#define VL53L7CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L7CX_DISABLE_RANGE_SIGMA_MM
#define VL53L7CX_DISABLE_REFLECTANCE_PERCENT
#define VL53L7CX_DISABLE_MOTION_INDICATOR

/* Per-device port state: which controller this device is on, and the transport evidence gathered
 * while talking to it. Defined in vl53l7cx_port.h, which the vendor sources never include, so this
 * header stays free of Zephyr types. The caller owns the object and its lifetime must cover every
 * ULD call made through this platform. */
struct vl53l7cx_port_context;

/* WHAT THE STOP HANDSHAKE SAW WHILE IT WAITED.
 *
 * vl53l7cx_stop_ranging() provokes the MCU stop and then polls GO2 status 0 for bit 7, up to 500
 * times with a 10 ms wait. On 2026-10-12 that poll spent its whole budget on dasher1 with every
 * transfer succeeding, and the only thing kept about it was that it timed out: the bytes the device
 * actually returned were read, tested, and dropped. Those bytes are the one observation of device
 * state this driver already pays for, so they are kept here instead.
 *
 * NO EXTRA TRANSFERS, NO CHANGED WAIT, NO CHANGED BRANCH. Every field below comes from a read the
 * unmodified sequence already performs; the patch records what those reads returned and alters
 * nothing about when they happen or what the function decides. The structure is filled as the call
 * runs and read afterwards, because printing inside a 5-second poll would change its timing.
 *
 * WHAT IT IS NOT. It is an observation of GO2 status, not a verdict about the device. In
 * particular it is not a cold-start detector: _vl53l7cx_poll_for_mcu_boot() leaves the same bit 0
 * set as an exit condition AFTER the firmware download and MCU reset, so a given value here is
 * consistent with more than one history and says nothing about whether the part was ever powered
 * down. Name any later conclusion after the evidence that supports it, not after this.
 *
 * VALIDITY TRAVELS WITH EVERY VALUE. The Zephyr port hands the caller's buffer straight to
 * i2c_transfer() and leaves it untouched when the transfer fails -- the same property patches 0003
 * and 0004 had to deal with -- so a byte that was never read would otherwise be indistinguishable
 * from one that was. A field whose `_valid` companion is zero was not read and its value is
 * meaningless, not stale-but-close. */
enum {
	/* The auto-stop path skipped the poll entirely: nothing was asked and nothing is recorded. */
	VL53L7CX_GO2_EXIT_NOT_POLLED = 0,
	/* The loop's own condition saw bit 7, which is the MCU stop this call waits for. */
	VL53L7CX_GO2_EXIT_CONFIRMED,
	/* The 500 iterations ran out. This is the exit that patch 0002 made report a timeout. */
	VL53L7CX_GO2_EXIT_EXHAUSTED,
};

struct vl53l7cx_go2_observation {
	/* GO2 status 0 from the FIRST and LAST successful reads inside the poll. Both, because a
	 * value that never changes and one that changes once are different observations and the
	 * count alone cannot tell them apart. */
	uint8_t first;
	uint8_t last;
	/* GO2 status 0 from the read the sequence performs AFTER the loop, on the way to the status
	 * 1 check. Separate from `last`: it is taken once the budget is already spent, so it is the
	 * device's answer at a different moment, not another sample of the same one. */
	uint8_t after_loop;
	/* GO2 status 1, which the sequence reads only when bit 7 is set in `after_loop`. */
	uint8_t status1;
	/* One of the VL53L7CX_GO2_EXIT_* values above. */
	uint8_t exit_reason;
	uint8_t first_valid;
	uint8_t last_valid;
	uint8_t after_loop_valid;
	uint8_t status1_valid;
	/* Bit 7 observed in any successfully read sample. Kept separately from `exit_reason`
	 * because the two answer different questions: one is what the device said, the other is how
	 * this particular loop came to an end. */
	uint8_t saw_mcu_stop;
	/* Read attempts inside the poll, and how many of them succeeded. A gap between the two is a
	 * transport fault during the wait, which is not the same event as a device that answered
	 * every time without ever confirming. */
	uint16_t polls;
	uint16_t valid_polls;
};

typedef struct {
    /* ST stores the 8-bit wire address here (default 0x52). The Zephyr port validates it and
     * converts exactly once to the 7-bit controller address. */
    uint16_t address;
    /* Filled only from a verified tof_l7_blob::blob_view. The upstream patch refuses init when
     * this is null and uses it for the three firmware download ranges. */
    const uint8_t *firmware;
    /* Kept with the pointer so the patched ULD can prove that all three fixed ranges fit before the
     * first I2C operation. A valid pointer without its bound is not an authorised buffer. */
    size_t firmware_size;
    /* The bus this device is on, and where its errno and completed-transfer count accumulate.
     * A null context is a programming error rather than a recoverable condition: without it the
     * port has no controller to address, so every transfer is refused with -ENODEV. */
    struct vl53l7cx_port_context *port;
    /* Where the patched stop sequence records what GO2 status said while it waited. Optional: a
     * null pointer means the sequence runs exactly as before and observes nothing, so production
     * images that do not ask for it carry neither the storage nor a behaviour difference. */
    struct vl53l7cx_go2_observation *go2;
} VL53L7CX_Platform;

enum { VL53L7CX_FIRMWARE_DOWNLOAD_SIZE = 0x15000U };

#ifndef VL53L7CX_NB_TARGET_PER_ZONE
#define VL53L7CX_NB_TARGET_PER_ZONE (1U)
#endif

/* Keep ULD conversion enabled. The three fields used by the frozen contract remain present. */
#define PROCESSOR_LITTLE_ENDIAN
#define SWAP_UINT16(x) (x)
#define SWAP_UINT32(x) (x)

uint8_t VL53L7CX_RdByte(VL53L7CX_Platform *p_platform, uint16_t register_address,
                        uint8_t *value);
uint8_t VL53L7CX_WrByte(VL53L7CX_Platform *p_platform, uint16_t register_address,
                        uint8_t value);
uint8_t VL53L7CX_RdMulti(VL53L7CX_Platform *p_platform, uint16_t register_address,
                         uint8_t *values, uint32_t size);
uint8_t VL53L7CX_WrMulti(VL53L7CX_Platform *p_platform, uint16_t register_address,
                         uint8_t *values, uint32_t size);
void VL53L7CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L7CX_WaitMs(VL53L7CX_Platform *p_platform, uint32_t time_ms);

#endif
