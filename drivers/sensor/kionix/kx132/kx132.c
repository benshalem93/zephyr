/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kionix_kx132

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

#include "kx132.h"

LOG_MODULE_REGISTER(kx132, CONFIG_SENSOR_LOG_LEVEL);

static uint16_t kx132_range_to_gain(uint8_t range)
{
	switch (range) {
	case 2:
		return KX132_GAIN_2G;
	case 4:
		return KX132_GAIN_4G;
	case 8:
		return KX132_GAIN_8G;
	case 16:
		return KX132_GAIN_16G;
	default:
		return KX132_GAIN_2G;
	}
}

static uint8_t kx132_range_to_gsel(uint8_t range)
{
	switch (range) {
	case 2:
		return KX132_GSEL_2G;
	case 4:
		return KX132_GSEL_4G;
	case 8:
		return KX132_GSEL_8G;
	case 16:
		return KX132_GSEL_16G;
	default:
		return KX132_GSEL_2G;
	}
}

static uint8_t kx132_odr_to_osa(uint16_t odr)
{
	if (odr >= 25600) {
		return KX132_OSA_25600HZ;
	} else if (odr >= 12800) {
		return KX132_OSA_12800HZ;
	} else if (odr >= 6400) {
		return KX132_OSA_6400HZ;
	} else if (odr >= 3200) {
		return KX132_OSA_3200HZ;
	} else if (odr >= 1600) {
		return KX132_OSA_1600HZ;
	} else if (odr >= 800) {
		return KX132_OSA_800HZ;
	} else if (odr >= 400) {
		return KX132_OSA_400HZ;
	} else if (odr >= 200) {
		return KX132_OSA_200HZ;
	} else if (odr >= 100) {
		return KX132_OSA_100HZ;
	} else if (odr >= 50) {
		return KX132_OSA_50HZ;
	} else if (odr >= 25) {
		return KX132_OSA_25HZ;
	} else if (odr >= 12) {
		return KX132_OSA_12_5HZ;
	} else if (odr >= 6) {
		return KX132_OSA_6_25HZ;
	} else if (odr >= 3) {
		return KX132_OSA_3_125HZ;
	} else if (odr >= 1) {
		return KX132_OSA_1_563HZ;
	}

	return KX132_OSA_0_781HZ;
}

static uint8_t kx132_odr_to_owuf(uint16_t odr)
{
	if (odr >= 100) {
		return KX132_OWUF_100HZ;
	} else if (odr >= 50) {
		return KX132_OWUF_50HZ;
	} else if (odr >= 25) {
		return KX132_OWUF_25HZ;
	} else if (odr >= 12) {
		return KX132_OWUF_12_5HZ;
	} else if (odr >= 6) {
		return KX132_OWUF_6_25HZ;
	} else if (odr >= 3) {
		return KX132_OWUF_3_125HZ;
	} else if (odr >= 1) {
		return KX132_OWUF_1_563HZ;
	}

	return KX132_OWUF_0_781HZ;
}

static int kx132_set_standby(const struct device *dev, bool standby)
{
	const struct kx132_config *cfg = dev->config;
	uint8_t val;
	int ret;

	ret = i2c_reg_read_byte_dt(&cfg->i2c, KX132_REG_CNTL1, &val);
	if (ret < 0) {
		return ret;
	}

	if (standby) {
		val &= ~KX132_CNTL1_PC1;
	} else {
		val |= KX132_CNTL1_PC1;
	}

	return i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_CNTL1, val);
}

static int kx132_sample_fetch(const struct device *dev,
			      enum sensor_channel chan)
{
	const struct kx132_config *cfg = dev->config;
	struct kx132_data *data = dev->data;
	uint8_t buf[6];
	int ret;

	if (chan != SENSOR_CHAN_ALL &&
	    chan != SENSOR_CHAN_ACCEL_XYZ &&
	    chan != SENSOR_CHAN_ACCEL_X &&
	    chan != SENSOR_CHAN_ACCEL_Y &&
	    chan != SENSOR_CHAN_ACCEL_Z) {
		return -ENOTSUP;
	}

	ret = i2c_burst_read_dt(&cfg->i2c, KX132_REG_XOUT_L, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to read accel data: %d", ret);
		return ret;
	}

	data->acc[0] = (int16_t)((buf[1] << 8) | buf[0]);
	data->acc[1] = (int16_t)((buf[3] << 8) | buf[2]);
	data->acc[2] = (int16_t)((buf[5] << 8) | buf[4]);

	return 0;
}

static void kx132_convert(struct sensor_value *val, int16_t raw, uint16_t gain)
{
	int64_t micro_ms2;

	/* raw * gain_ug gives ug, multiply by 9.80665 to get u(m/s^2) */
	micro_ms2 = (int64_t)raw * gain * 980665LL / 100000LL;

	val->val1 = (int32_t)(micro_ms2 / 1000000LL);
	val->val2 = (int32_t)(micro_ms2 % 1000000LL);
}

static int kx132_channel_get(const struct device *dev,
			     enum sensor_channel chan,
			     struct sensor_value *val)
{
	struct kx132_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		kx132_convert(val, data->acc[0], data->gain);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		kx132_convert(val, data->acc[1], data->gain);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		kx132_convert(val, data->acc[2], data->gain);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		for (int i = 0; i < 3; i++) {
			kx132_convert(&val[i], data->acc[i], data->gain);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int kx132_attr_set(const struct device *dev,
			  enum sensor_channel chan,
			  enum sensor_attribute attr,
			  const struct sensor_value *val)
{
	const struct kx132_config *cfg = dev->config;
	struct kx132_data *data = dev->data;
	int ret;

	/* OTF (on-the-fly) attributes — no standby required */
	switch (attr) {
	case SENSOR_ATTR_SLOPE_TH: {
		/* Convert mg to counts: counts = mg * 256 / 1000 */
		uint16_t counts = (uint16_t)(val->val1 * 256 / 1000);

		ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_WUFTH,
					    counts & 0xFF);
		if (ret < 0) {
			return ret;
		}
		return i2c_reg_update_byte_dt(&cfg->i2c, KX132_REG_BTSWUFTH,
					      0x07, (counts >> 8) & 0x07);
	}
	case SENSOR_ATTR_SLOPE_DUR:
		return i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_WUFC,
					     (uint8_t)val->val1);
	case SENSOR_ATTR_HYSTERESIS: {
		uint8_t owuf = kx132_odr_to_owuf((uint16_t)val->val1);

		return i2c_reg_update_byte_dt(&cfg->i2c, KX132_REG_CNTL3,
					      KX132_CNTL3_OWUF_MASK, owuf);
	}
	default:
		break;
	}

	/* Attributes that require standby mode */
	ret = kx132_set_standby(dev, true);
	if (ret < 0) {
		return ret;
	}

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE: {
		uint8_t range = (uint8_t)val->val1;
		uint8_t gsel = kx132_range_to_gsel(range);
		uint8_t reg;

		ret = i2c_reg_read_byte_dt(&cfg->i2c, KX132_REG_CNTL1, &reg);
		if (ret < 0) {
			break;
		}
		reg &= ~KX132_CNTL1_GSEL_MASK;
		reg |= (gsel << KX132_CNTL1_GSEL_SHIFT);
		ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_CNTL1, reg);
		if (ret == 0) {
			data->gain = kx132_range_to_gain(range);
		}
		break;
	}
	case SENSOR_ATTR_SAMPLING_FREQUENCY: {
		uint8_t osa = kx132_odr_to_osa((uint16_t)val->val1);

		ret = i2c_reg_update_byte_dt(&cfg->i2c, KX132_REG_ODCNTL,
					     KX132_ODCNTL_OSA_MASK, osa);
		break;
	}
	default:
		ret = -ENOTSUP;
	}

	if (ret < 0) {
		return ret;
	}

	return kx132_set_standby(dev, false);
}

static int kx132_power_up(const struct device *dev)
{
	const struct kx132_config *cfg = dev->config;
	struct kx132_data *data = dev->data;
	uint8_t val;
	int ret;

	/* Set standby mode (PC1=0) - required before configuration */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_CNTL1, 0x00);
	if (ret < 0) {
		return ret;
	}

	/* Configure ODR */
	uint8_t osa = kx132_odr_to_osa(cfg->odr);

	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_ODCNTL, osa);
	if (ret < 0) {
		return ret;
	}

	/* Set gain based on configured range */
	data->gain = kx132_range_to_gain(cfg->range);

#ifdef CONFIG_KX132_TRIGGER
	ret = kx132_init_interrupt(dev);
	if (ret < 0) {
		LOG_ERR("Failed to init interrupts: %d", ret);
		return ret;
	}
#endif

	/* Enable sensor: PC1=1, low-power mode (RES=0), set range */
	data->cntl1_val = KX132_CNTL1_PC1 |
			  (kx132_range_to_gsel(cfg->range) << KX132_CNTL1_GSEL_SHIFT);

	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_CNTL1, data->cntl1_val);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_KX132_TRIGGER
	/* Force initial sleep state for wake-up engine */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_CNTL5,
				    KX132_CNTL5_MAN_SLEEP);
	if (ret < 0) {
		return ret;
	}
#endif

	/* Clear any pending interrupts */
	ret = i2c_reg_read_byte_dt(&cfg->i2c, KX132_REG_INT_REL, &val);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("KX132-1211 initialized (ODR=%u Hz, range=+/-%ug)",
		cfg->odr, cfg->range);

	return 0;
}

static int kx132_pm_action(const struct device *dev,
			   enum pm_device_action action)
{
	const struct kx132_config *cfg = dev->config;
	struct kx132_data *data = dev->data;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/* Enter standby mode (PC1=0) for minimum power consumption */
		return i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_CNTL1,
					     data->cntl1_val & ~KX132_CNTL1_PC1);
	case PM_DEVICE_ACTION_RESUME:
		return kx132_power_up(dev);
	case PM_DEVICE_ACTION_TURN_ON:
		return kx132_power_up(dev);
	case PM_DEVICE_ACTION_TURN_OFF:
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int kx132_init(const struct device *dev)
{
	const struct kx132_config *cfg = dev->config;
	uint8_t val;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Verify WHO_AM_I */
	ret = i2c_reg_read_byte_dt(&cfg->i2c, KX132_REG_WHO_AM_I, &val);
	if (ret < 0) {
		LOG_ERR("Failed to read WHO_AM_I: %d", ret);
		return ret;
	}

	if (val != KX132_WHO_AM_I_VAL) {
		LOG_ERR("Invalid WHO_AM_I: 0x%02x (expected 0x%02x)",
			val, KX132_WHO_AM_I_VAL);
		return -ENODEV;
	}

	/* Software reset */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_CNTL2, KX132_CNTL2_SRST);
	if (ret < 0) {
		LOG_ERR("Failed to reset: %d", ret);
		return ret;
	}

	/* Wait for reset to complete */
	k_msleep(2);

	return pm_device_driver_init(dev, kx132_pm_action);
}

static DEVICE_API(sensor, kx132_driver_api) = {
	.sample_fetch = kx132_sample_fetch,
	.channel_get = kx132_channel_get,
	.attr_set = kx132_attr_set,
#ifdef CONFIG_KX132_TRIGGER
	.trigger_set = kx132_trigger_set,
#endif
};

#define KX132_TRIGGER_CFG(inst)						\
	IF_ENABLED(CONFIG_KX132_TRIGGER,				\
		(.gpio_int = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, {0}), \
		 .wakeup_threshold = DT_INST_PROP_OR(inst, wakeup_threshold, 112), \
		 .wakeup_debounce = DT_INST_PROP_OR(inst, wakeup_debounce, 10),))

#define KX132_DEFINE(inst)						\
	static struct kx132_data kx132_data_##inst;			\
	static const struct kx132_config kx132_config_##inst = {	\
		.i2c = I2C_DT_SPEC_INST_GET(inst),			\
		.odr = DT_INST_PROP_OR(inst, odr, 50),			\
		.range = DT_INST_PROP_OR(inst, range, 2),		\
		KX132_TRIGGER_CFG(inst)					\
	};								\
	PM_DEVICE_DT_INST_DEFINE(inst, kx132_pm_action);		\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, kx132_init,			\
				     PM_DEVICE_DT_INST_GET(inst),	\
				     &kx132_data_##inst,		\
				     &kx132_config_##inst,		\
				     POST_KERNEL,			\
				     CONFIG_SENSOR_INIT_PRIORITY,	\
				     &kx132_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KX132_DEFINE)
