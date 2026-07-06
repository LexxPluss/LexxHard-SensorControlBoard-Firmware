/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * CHANGELOG:
 * 2024-04-02: created IIM42652 driver based on ICM42605 driver by Takuro Tsujikawa (takuro.tsujikawa@lexxpluss.com)
 * 	- changed definition from ICM42605 to IIM42652
 * 	- changed filename from icm42605_setup.c to iim42652_setup.c
 *  - added REG_INT_CONFIG setting for INT porality and drive type
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "iim42652.h"
#include "iim42652_reg.h"
#include "iim42652_spi.h"

LOG_MODULE_DECLARE(IIM42652, CONFIG_SENSOR_LOG_LEVEL);

int iim42652_set_fs(const struct device *dev, uint16_t a_sf, uint16_t g_sf)
{
	const struct iim42652_config *cfg = dev->config;
	uint8_t databuf;
	int result;

	/* Validate FS_SEL indices against the datasheet code counts
	 * (4 codes for accel, 8 for gyro; see iim42652_reg.h). */
	if (a_sf >= IIM42652_ACCEL_FS_COUNT ||
	    g_sf >= IIM42652_GYRO_FS_COUNT) {
		LOG_ERR("Invalid FS_SEL a=%u g=%u", a_sf, g_sf);
		return -EINVAL;
	}
	__ASSERT_NO_MSG(a_sf < IIM42652_ACCEL_FS_COUNT &&
			g_sf < IIM42652_GYRO_FS_COUNT);

	result = inv_spi_read(&cfg->spi, REG_ACCEL_CONFIG0, &databuf, 1);
	if (result) {
		return result;
	}
	databuf &= ~BIT_ACCEL_FSR;
	databuf |= ((uint8_t)a_sf << SHIFT_ACCEL_FS_SEL) & BIT_ACCEL_FSR;

	result = inv_spi_single_write(&cfg->spi, REG_ACCEL_CONFIG0, &databuf);
	if (result) {
		return result;
	}
	LOG_DBG("ACCEL_CONFIG0 written = 0x%02X (FS_SEL=%u)", databuf, a_sf);

	result = inv_spi_read(&cfg->spi, REG_GYRO_CONFIG0, &databuf, 1);

	if (result) {
		return result;
	}

	databuf &= ~BIT_GYRO_FSR;
	databuf |= ((uint8_t)g_sf << SHIFT_GYRO_FS_SEL) & BIT_GYRO_FSR;

	result = inv_spi_single_write(&cfg->spi, REG_GYRO_CONFIG0, &databuf);

	if (result) {
		return result;
	}
	LOG_DBG("GYRO_CONFIG0 written = 0x%02X (FS_SEL=%u)", databuf, g_sf);

	return 0;
}

int iim42652_set_odr(const struct device *dev, uint16_t a_rate, uint16_t g_rate)
{
	const struct iim42652_config *cfg = dev->config;
	uint8_t databuf;
	int result;

	if (a_rate > 8000 || g_rate > 8000 ||
	    a_rate < 1 || g_rate < 12) {
		LOG_ERR("Not supported frequency");
		return -ENOTSUP;
	}

	result = inv_spi_read(&cfg->spi, REG_ACCEL_CONFIG0, &databuf, 1);

	if (result) {
		return result;
	}

	databuf &= ~BIT_ACCEL_ODR;

	if (a_rate > 4000) {
		databuf |= BIT_ACCEL_ODR_8000;
	} else if (a_rate > 2000) {
		databuf |= BIT_ACCEL_ODR_4000;
	} else if (a_rate > 1000) {
		databuf |= BIT_ACCEL_ODR_2000;
	} else if (a_rate > 500) {
		databuf |= BIT_ACCEL_ODR_1000;
	} else if (a_rate > 200) {
		databuf |= BIT_ACCEL_ODR_500;
	} else if (a_rate > 100) {
		databuf |= BIT_ACCEL_ODR_200;
	} else if (a_rate > 50) {
		databuf |= BIT_ACCEL_ODR_100;
	} else if (a_rate > 25) {
		databuf |= BIT_ACCEL_ODR_50;
	} else if (a_rate > 12) {
		databuf |= BIT_ACCEL_ODR_25;
	} else if (a_rate > 6) {
		databuf |= BIT_ACCEL_ODR_12;
	} else if (a_rate > 3) {
		databuf |= BIT_ACCEL_ODR_6;
	} else if (a_rate > 1) {
		databuf |= BIT_ACCEL_ODR_3;
	} else {
		databuf |= BIT_ACCEL_ODR_1;
	}

	result = inv_spi_single_write(&cfg->spi, REG_ACCEL_CONFIG0, &databuf);

	if (result) {
		return result;
	}

	LOG_DBG("Write Accel ODR 0x%X", databuf);

	result = inv_spi_read(&cfg->spi, REG_GYRO_CONFIG0, &databuf, 1);

	if (result) {
		return result;
	}

	databuf &= ~BIT_GYRO_ODR;

	if (g_rate > 4000) {
		databuf |= BIT_GYRO_ODR_8000;
	} else if (g_rate > 2000) {
		databuf |= BIT_GYRO_ODR_4000;
	} else if (g_rate > 1000) {
		databuf |= BIT_GYRO_ODR_2000;
	} else if (g_rate > 500) {
		databuf |= BIT_GYRO_ODR_1000;
	} else if (g_rate > 200) {
		databuf |= BIT_GYRO_ODR_500;
	} else if (g_rate > 100) {
		databuf |= BIT_GYRO_ODR_200;
	} else if (g_rate > 50) {
		databuf |= BIT_GYRO_ODR_100;
	} else if (g_rate > 25) {
		databuf |= BIT_GYRO_ODR_50;
	} else if (g_rate > 12) {
		databuf |= BIT_GYRO_ODR_25;
	} else {
		databuf |= BIT_GYRO_ODR_12;
	}

	LOG_DBG("Write GYRO ODR 0x%X", databuf);

	result = inv_spi_single_write(&cfg->spi, REG_GYRO_CONFIG0, &databuf);
	if (result) {
		return result;
	}

	return result;
}

int iim42652_sensor_init(const struct device *dev)
{
	const struct iim42652_config *cfg = dev->config;
	int result = 0;
	uint8_t v;

	result = inv_spi_read(&cfg->spi, REG_WHO_AM_I, &v, 1);

	if (result) {
		return result;
	}

	LOG_DBG("WHO AM I : 0x%X", v);

	result = inv_spi_read(&cfg->spi, REG_DEVICE_CONFIG, &v, 1);

	if (result) {
		LOG_DBG("read REG_DEVICE_CONFIG_REG failed");
		return result;
	}

	v |= BIT_SOFT_RESET;

	result = inv_spi_single_write(&cfg->spi, REG_DEVICE_CONFIG, &v);

	if (result) {
		LOG_ERR("write REG_DEVICE_CONFIG failed");
		return result;
	}

	/* Need at least 10ms after soft reset */
	k_msleep(10);

	/* INTF_CONFIG1 RMW: preserve reserved bits[7:4] (DS §14.29). The previous
	 * unconditional write of (BIT_GYRO_AFSR_MODE_HFS | BIT_ACCEL_AFSR_MODE_HFS |
	 * BIT_CLK_SEL_PLL) = 0x51 clobbered reserved bits 7 and 4. The AFSR_MODE
	 * macros were carried over from ICM-426xx and are not applicable to
	 * IIM-42652 — writing them is undefined behavior on this part.
	 */
	result = inv_spi_read(&cfg->spi, REG_INTF_CONFIG1, &v, 1);
	if (result) {
		LOG_ERR("read REG_INTF_CONFIG1 failed");
		return result;
	}
	LOG_INF("INTF_CONFIG1 reset value = 0x%02X", v);

	v &= ~0x0F;             /* clear ACCEL_LP_CLK_SEL, RTC_MODE, CLKSEL */
	v |= BIT_CLK_SEL_PLL;   /* set CLKSEL = PLL */

	result = inv_spi_single_write(&cfg->spi, REG_INTF_CONFIG1, &v);

	if (result) {
		LOG_ERR("write REG_INTF_CONFIG1 failed");
		return result;
	}
	LOG_INF("INTF_CONFIG1 after RMW   = 0x%02X", v);

	v = BIT_EN_DREG_FIFO_D2A |
	    BIT_TMST_TO_REGS_EN |
	    BIT_TMST_EN;

	result = inv_spi_single_write(&cfg->spi, REG_TMST_CONFIG, &v);

	if (result) {
		LOG_ERR("Write REG_TMST_CONFIG failed");
		return result;
	}

	result = inv_spi_read(&cfg->spi, REG_INTF_CONFIG0, &v, 1);

	if (result) {
		LOG_ERR("Read REG_INTF_CONFIG0 failed");
		return result;
	}

	LOG_DBG("Read REG_INTF_CONFIG0 0x%X", v);

	v |= BIT_UI_SIFS_DISABLE_I2C;

	result = inv_spi_single_write(&cfg->spi, REG_INTF_CONFIG0, &v);

	if (result) {
		LOG_ERR("Write REG_INTF_CONFIG failed");
		return result;
	}

	v = 0;
	result = inv_spi_single_write(&cfg->spi, REG_INT_CONFIG1, &v);

	if (result) {
		return result;
	}

	result = inv_spi_single_write(&cfg->spi, REG_PWR_MGMT0, &v);

	if (result) {
		return result;
	}

	v = 0x03;

	result = inv_spi_single_write(&cfg->spi, REG_INT_CONFIG, &v);

	if (result) {
		LOG_ERR("Write REG_INT_CONFIG failed");
		return result;
	}

	return 0;
}

int iim42652_turn_on_fifo(const struct device *dev)
{
	const struct iim42652_data *drv_data = dev->data;
	const struct iim42652_config *cfg = dev->config;

	uint8_t int0_en = BIT_INT_UI_DRDY_INT1_EN;
	uint8_t fifo_en = BIT_FIFO_ACCEL_EN | BIT_FIFO_GYRO_EN | BIT_FIFO_WM_TH;
	uint8_t burst_read[3];
	int result;
	uint8_t v = 0;

	v = BIT_FIFO_MODE_BYPASS;
	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG, &v);
	if (result) {
		return result;
	}

	v = 0;
	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG1, &v);
	if (result) {
		return result;
	}

	result = inv_spi_read(&cfg->spi, REG_FIFO_COUNTH, burst_read, 2);
	if (result) {
		return result;
	}

	result = inv_spi_read(&cfg->spi, REG_FIFO_DATA, burst_read, 3);
	if (result) {
		return result;
	}

	v = BIT_FIFO_MODE_STREAM;
	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG, &v);
	if (result) {
		return result;
	}

	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG1, &fifo_en);
	if (result) {
		return result;
	}

	result = inv_spi_single_write(&cfg->spi, REG_INT_SOURCE0, &int0_en);
	if (result) {
		return result;
	}

	if (drv_data->tap_en) {
		v = BIT_TAP_ENABLE;
		result = inv_spi_single_write(&cfg->spi, REG_APEX_CONFIG0, &v);
		if (result) {
			return result;
		}

		v = BIT_DMP_INIT_EN;
		result = inv_spi_single_write(&cfg->spi, REG_SIGNAL_PATH_RESET, &v);
		if (result) {
			return result;
		}

		v = BIT_BANK_SEL_4;
		result = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &v);
		if (result) {
			return result;
		}

		v = BIT_INT_STATUS_TAP_DET;
		result = inv_spi_single_write(&cfg->spi, REG_INT_SOURCE6, &v);
		if (result) {
			return result;
		}

		v = BIT_BANK_SEL_0;
		result = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &v);
		if (result) {
			return result;
		}
	}

	LOG_DBG("turn on fifo done");
	return 0;
}

int iim42652_turn_off_fifo(const struct device *dev)
{
	const struct iim42652_data *drv_data = dev->data;
	const struct iim42652_config *cfg = dev->config;
	uint8_t int0_en = 0;
	uint8_t burst_read[3];
	int result;
	uint8_t v = 0;

	v = BIT_FIFO_MODE_BYPASS;
	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG, &v);
	if (result) {
		return result;
	}

	v = 0;
	result = inv_spi_single_write(&cfg->spi, REG_FIFO_CONFIG1, &v);
	if (result) {
		return result;
	}

	result = inv_spi_read(&cfg->spi, REG_FIFO_COUNTH, burst_read, 2);
	if (result) {
		return result;
	}

	result = inv_spi_read(&cfg->spi, REG_FIFO_DATA, burst_read, 3);
	if (result) {
		return result;
	}

	result = inv_spi_single_write(&cfg->spi, REG_INT_SOURCE0, &int0_en);
	if (result) {
		return result;
	}

	if (drv_data->tap_en) {
		v = 0;
		result = inv_spi_single_write(&cfg->spi, REG_APEX_CONFIG0, &v);
		if (result) {
			return result;
		}

		result = inv_spi_single_write(&cfg->spi, REG_SIGNAL_PATH_RESET, &v);
		if (result) {
			return result;
		}

		v = BIT_BANK_SEL_4;
		result = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &v);
		if (result) {
			return result;
		}

		v = 0;
		result = inv_spi_single_write(&cfg->spi, REG_INT_SOURCE6, &v);
		if (result) {
			return result;
		}

		v = BIT_BANK_SEL_0;
		result = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &v);
		if (result) {
			return result;
		}
	}

	return 0;
}

int iim42652_turn_on_sensor(const struct device *dev)
{
	struct iim42652_data *drv_data = dev->data;
	const struct iim42652_config *cfg = dev->config;
	uint8_t v = 0;
	int result = 0;


	if (drv_data->sensor_started) {
		LOG_ERR("Sensor already started");
		return -EALREADY;
	}

	iim42652_set_fs(dev, drv_data->accel_sf, drv_data->gyro_sf);

	iim42652_set_odr(dev, drv_data->accel_hz, drv_data->gyro_hz);

	v |= BIT_ACCEL_MODE_LNM;
	v |= BIT_GYRO_MODE_LNM;

	result = inv_spi_single_write(&cfg->spi, REG_PWR_MGMT0, &v);
	if (result) {
		return result;
	}

	/* Accelerometer sensor need at least 10ms startup time
	 * Gyroscope sensor need at least 30ms startup time
	 */
	k_msleep(100);

	iim42652_turn_on_fifo(dev);

	drv_data->sensor_started = true;

	return 0;
}

int iim42652_turn_off_sensor(const struct device *dev)
{
	const struct iim42652_config *cfg = dev->config;
	uint8_t v = 0;
	int result = 0;

	result = inv_spi_read(&cfg->spi, REG_PWR_MGMT0, &v, 1);

	v ^= BIT_ACCEL_MODE_LNM;
	v ^= BIT_GYRO_MODE_LNM;

	result = inv_spi_single_write(&cfg->spi, REG_PWR_MGMT0, &v);
	if (result) {
		return result;
	}

	/* Accelerometer sensor need at least 10ms startup time
	 * Gyroscope sensor need at least 30ms startup time
	 */
	k_msleep(100);

	iim42652_turn_off_fifo(dev);

	return 0;
}

/* Write all six OFFSET_USER channels in one operation (DS §18).
 *
 *   gyro  step unit  = 1/32 dps      (range ±64 dps,    12-bit signed)
 *   accel step unit  = 0.5 mG        (range ±1 g,        12-bit signed)
 *
 * Per DS §12.9 the register must be written with accel + gyro powered down.
 * We only touch PWR_MGMT0 (not FIFO), so the trigger callback resumes
 * seamlessly after restore. All 9 OFFSET_USER bytes get rewritten, so the
 * shared-nibble registers (USER1 / USER4 / USER7) do NOT need RMW — every
 * bit's source is one of the six caller-supplied step values.
 *
 * Sign convention (verified on PACO v71es007):
 *   sensor_output = sensor_raw + OFFUSER
 *   - ACCEL_Z: 2026-05-14, plan §3.1 (+40/-56 step experiments)
 *   - GYRO X/Y/Z: 2026-05-15 (+320 step per axis → ~+320 LSB sensor-frame
 *     shift on the matching GYRO_DATA register; all three axes same sign
 *     as accel)
 * So automated calibration should write OFFUSER = -bias_observed.
 */
int iim42652_set_offset_user(const struct device *dev,
			     int16_t gx_step, int16_t gy_step, int16_t gz_step,
			     int16_t ax_step, int16_t ay_step, int16_t az_step)
{
	struct iim42652_data *drv_data = dev->data;
	const struct iim42652_config *cfg = dev->config;
	uint8_t saved_pwr, off_val, bank;
	int rc, rc2;
	const int16_t steps[6] = {
		gx_step, gy_step, gz_step,
		ax_step, ay_step, az_step,
	};
	for (int i = 0; i < 6; i++) {
		if (steps[i] < -2048 || steps[i] > 2047) {
			return -EINVAL;
		}
	}

	/* Build the 9-byte image of OFFSET_USER0..8 from the six 12-bit signed
	 * fields. Shared-nibble layout from DS §18:
	 *   USER0 = GX[7:0]
	 *   USER1 = GY[11:8] | GX[11:8]
	 *   USER2 = GY[7:0]
	 *   USER3 = GZ[7:0]
	 *   USER4 = AX[11:8] | GZ[11:8]
	 *   USER5 = AX[7:0]
	 *   USER6 = AY[7:0]
	 *   USER7 = AZ[11:8] | AY[11:8]
	 *   USER8 = AZ[7:0]
	 * Cast through uint16_t before the >>8 to avoid signed-shift ambiguity.
	 */
	const uint16_t gx = (uint16_t)gx_step;
	const uint16_t gy = (uint16_t)gy_step;
	const uint16_t gz = (uint16_t)gz_step;
	const uint16_t ax = (uint16_t)ax_step;
	const uint16_t ay = (uint16_t)ay_step;
	const uint16_t az = (uint16_t)az_step;
	uint8_t off[9];
	off[0] =  gx        & 0xFF;
	off[1] = ((gy >> 8) & 0x0F) << 4 | ((gx >> 8) & 0x0F);
	off[2] =  gy        & 0xFF;
	off[3] =  gz        & 0xFF;
	off[4] = ((ax >> 8) & 0x0F) << 4 | ((gz >> 8) & 0x0F);
	off[5] =  ax        & 0xFF;
	off[6] =  ay        & 0xFF;
	off[7] = ((az >> 8) & 0x0F) << 4 | ((ay >> 8) & 0x0F);
	off[8] =  az        & 0xFF;

	k_mutex_lock(&drv_data->bus_lock, K_FOREVER);

	rc = inv_spi_read(&cfg->spi, REG_PWR_MGMT0, &saved_pwr, 1);
	if (rc) {
		goto out_unlock;
	}

	off_val = saved_pwr & ~0x0F;
	rc = inv_spi_single_write(&cfg->spi, REG_PWR_MGMT0, &off_val);
	if (rc) {
		goto restore_pwr;
	}

	k_msleep(2);

	bank = BIT_BANK_SEL_4;
	rc = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &bank);
	if (rc) {
		goto restore_bank;
	}

	for (int i = 0; i < 9; i++) {
		rc = inv_spi_single_write(&cfg->spi,
					  REG_OFFSET_USER0 + i,
					  &off[i]);
		if (rc) {
			/* Best-effort recovery: scrub all 9 bytes back to 0 so
			 * the chip is left in a known (zero-offset) state rather
			 * than a partial mix of new + old values. We ignore the
			 * return code of the scrub writes — if the bus is dead
			 * there's nothing more we can do, but the caller will
			 * see the original failure code and can decide. */
			LOG_WRN("OFFSET_USER write failed at byte %d (rc=%d), scrubbing all to 0",
				i, rc);
			uint8_t zero = 0;
			for (int j = 0; j < 9; j++) {
				(void)inv_spi_single_write(&cfg->spi,
							   REG_OFFSET_USER0 + j,
							   &zero);
			}
			break;
		}
	}

restore_bank:
	/* Always attempt to restore Bank 0, even if selecting Bank 4 reported
	 * an error. Everything below this point assumes Bank 0:
	 *   SIGNAL_PATH_RESET (0x4B) — Bank 4 0x4B is ACCEL_WOM_Y_THR
	 *   PWR_MGMT0         (0x4E) — Bank 4 0x4E is INT_SOURCE7
	 * If bank0 select fails we must not write those addresses blindly, or
	 * we would corrupt the wrong-bank register. Sensors stay powered down. */
	bank = BIT_BANK_SEL_0;
	rc2 = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &bank);
	const bool bank0_restored_after_off = (rc2 == 0);
	if (rc == 0) {
		rc = rc2;
	}

	if (!bank0_restored_after_off) {
		LOG_ERR("Bank 0 restore failed (rc=%d); skipping FIFO flush and PWR_MGMT0 restore; sensors left OFF",
			rc2);
		goto out_unlock;
	}

	/* Bank confirmed 0. Flush FIFO best-effort so the first packet after
	 * we restore PWR_MGMT0 reflects the new OFFSET. */
	{
		uint8_t flush = BIT_FIFO_FLUSH;
		int rc3 = inv_spi_single_write(&cfg->spi,
					       REG_SIGNAL_PATH_RESET,
					       &flush);
		if (rc3) {
			LOG_WRN("FIFO flush after OFFSET write failed: %d", rc3);
		}
	}

restore_pwr:
	/* Reached either via fall-through from restore_bank (bank0_restored is
	 * true) or via the early `goto restore_pwr` taken when the PWR_MGMT0
	 * OFF write itself failed — in that case bank was never switched to
	 * Bank 4, so PWR_MGMT0 still resolves to the Bank 0 register. */
	rc2 = inv_spi_single_write(&cfg->spi, REG_PWR_MGMT0, &saved_pwr);
	if (rc == 0) {
		rc = rc2;
	}

	k_msleep(100);

out_unlock:
	k_mutex_unlock(&drv_data->bus_lock);
	return rc;
}

int iim42652_diag_read_regs(const struct device *dev, uint8_t bank, uint8_t addr,
			    uint8_t *buf, size_t len)
{
	struct iim42652_data *drv_data = dev->data;
	const struct iim42652_config *cfg = dev->config;
	uint8_t bank_val;
	int rc;

	if (buf == NULL || len == 0) {
		return -EINVAL;
	}

	/* Serialize with sample_fetch() — without this, a 50 Hz fetch could
	 * land between our bank-select and bank-restore and read INT_STATUS /
	 * FIFO_COUNT from the wrong bank. */
	k_mutex_lock(&drv_data->bus_lock, K_FOREVER);

	bank_val = bank;
	rc = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &bank_val);
	if (rc) {
		/* Bank-select write failed: usually bank is unchanged, but we
		 * cannot be sure. Fall through to the restore path so Bank 0
		 * is asserted regardless. */
		goto restore_bank;
	}

	rc = inv_spi_read(&cfg->spi, addr, buf, len);

restore_bank:
	if (bank != BIT_BANK_SEL_0) {
		uint8_t b0 = BIT_BANK_SEL_0;
		int rc2 = inv_spi_single_write(&cfg->spi, REG_BANK_SEL, &b0);
		if (rc == 0) {
			rc = rc2;
		}
		/* If both the original op and the restore failed, the original
		 * error wins — that's what the caller actually cares about. */
	}

	k_mutex_unlock(&drv_data->bus_lock);
	return rc;
}

int iim42652_diag_read_reg(const struct device *dev, uint8_t bank, uint8_t addr,
			   uint8_t *val)
{
	return iim42652_diag_read_regs(dev, bank, addr, val, 1);
}
