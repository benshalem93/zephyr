/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kionix_kx132

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "kx132.h"

LOG_MODULE_DECLARE(kx132, CONFIG_SENSOR_LOG_LEVEL);

static void kx132_handle_interrupt(const struct device *dev)
{
	const struct kx132_config *cfg = dev->config;
	struct kx132_data *data = dev->data;
	uint8_t ins2 = 0;
	uint8_t ins3 = 0;
	uint8_t dummy;

	/* Read interrupt source registers */
	if (i2c_reg_read_byte_dt(&cfg->i2c, KX132_REG_INS2, &ins2) < 0) {
		LOG_ERR("Failed to read INS2");
	}

	if (i2c_reg_read_byte_dt(&cfg->i2c, KX132_REG_INS3, &ins3) < 0) {
		LOG_ERR("Failed to read INS3");
	}

	/* Handle wake-up (motion) interrupt */
	if ((ins3 & KX132_INS3_WUFS) && data->motion_handler) {
		data->motion_handler(dev, data->motion_trig);
	}

	/* Handle data ready interrupt */
	if ((ins2 & KX132_INS2_DRDY) && data->drdy_handler) {
		data->drdy_handler(dev, data->drdy_trig);
	}

	/* Read INT_REL to clear latched interrupts and release INT pin */
	i2c_reg_read_byte_dt(&cfg->i2c, KX132_REG_INT_REL, &dummy);

	/* Force sensor back to sleep state so the wake-up engine can
	 * trigger again on the next motion event.
	 */
	if (ins3 & KX132_INS3_WUFS) {
		i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_CNTL5,
				      KX132_CNTL5_ADPE | KX132_CNTL5_MAN_SLEEP);
	}

	/* Re-enable GPIO interrupt */
	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);
}

static void kx132_gpio_callback(const struct device *port,
				struct gpio_callback *cb, uint32_t pins)
{
	struct kx132_data *data = CONTAINER_OF(cb, struct kx132_data, gpio_cb);
	const struct kx132_config *cfg = data->dev->config;

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	/* Disable GPIO interrupt until handled */
	gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_DISABLE);

#ifdef CONFIG_KX132_TRIGGER_OWN_THREAD
	k_sem_give(&data->gpio_sem);
#elif defined(CONFIG_KX132_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
}

#ifdef CONFIG_KX132_TRIGGER_OWN_THREAD
static void kx132_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct kx132_data *data = p1;

	while (1) {
		k_sem_take(&data->gpio_sem, K_FOREVER);
		kx132_handle_interrupt(data->dev);
	}
}
#elif defined(CONFIG_KX132_TRIGGER_GLOBAL_THREAD)
static void kx132_work_cb(struct k_work *work)
{
	struct kx132_data *data = CONTAINER_OF(work, struct kx132_data, work);

	kx132_handle_interrupt(data->dev);
}
#endif

int kx132_trigger_set(const struct device *dev,
		      const struct sensor_trigger *trig,
		      sensor_trigger_handler_t handler)
{
	const struct kx132_config *cfg = dev->config;
	struct kx132_data *data = dev->data;
	uint8_t inc4;
	int ret;

	if (!cfg->gpio_int.port) {
		return -ENOTSUP;
	}

	/* Read current INC4 value */
	ret = i2c_reg_read_byte_dt(&cfg->i2c, KX132_REG_INC4, &inc4);
	if (ret < 0) {
		return ret;
	}

	switch (trig->type) {
	case SENSOR_TRIG_MOTION:
		data->motion_handler = handler;
		data->motion_trig = trig;

		if (handler) {
			/* --- Arm WUF + ADP engine --- */

			/* Enter standby: CNTL4 and ADP_CNTL1/2 require PC1=0 to latch */
			ret = kx132_set_standby(dev, true);
			if (ret < 0) {
				return ret;
			}

			/* Enable all axes for wake-up detection */
			ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_INC2,
						    KX132_INC2_ALL_AXES);
			if (ret < 0) {
				kx132_set_standby(dev, false);
				return ret;
			}

			/* Enable wake-up engine, relative threshold mode */
			ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_CNTL4,
						    KX132_CNTL4_WUFE | KX132_CNTL4_TH_MODE);
			if (ret < 0) {
				kx132_set_standby(dev, false);
				return ret;
			}

			/* Set OWUF to match the main ODR. CNTL3 OWUF controls the WUF
			 * debounce counter clock even when ADP_WB_ISEL=1. If left at
			 * its reset default (50 Hz) the counter ticks every 20 ms,
			 * causing periodic current spikes regardless of OADP.
			 */
			/* OWUF and OSA share the same encoding for frequencies up to
			 * 100 Hz, so the OSA value read from ODCNTL is directly usable
			 * as the OWUF value (both are 3-bit fields, same table).
			 * We read ODCNTL first so we can reuse odcntl below.
			 */
			uint8_t odcntl_early;

			ret = i2c_reg_read_byte_dt(&cfg->i2c, KX132_REG_ODCNTL, &odcntl_early);
			if (ret < 0) {
				kx132_set_standby(dev, false);
				return ret;
			}

			/* OSA[3:0] → OWUF[2:0]: clamp to OWUF max (100 Hz = 0x07) */
			uint8_t owuf = MIN(odcntl_early & KX132_ODCNTL_OSA_MASK,
					   KX132_OWUF_100HZ);

			ret = i2c_reg_update_byte_dt(&cfg->i2c, KX132_REG_CNTL3,
						     KX132_CNTL3_OWUF_MASK, owuf);
			if (ret < 0) {
				kx132_set_standby(dev, false);
				return ret;
			}

			/* Configure ADP ODR to match main ODR (in standby so OADP latches) */
			uint8_t oadp = odcntl_early & KX132_ODCNTL_OSA_MASK;
			uint8_t adp_cntl1 =
				(data->rms_avc << KX132_ADP_CNTL1_RMS_AVC_SHIFT) | oadp;

			ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_ADP_CNTL1, adp_cntl1);
			if (ret < 0) {
				kx132_set_standby(dev, false);
				return ret;
			}

			/* Route RMS output to WUF engine; bypass both filters */
			uint8_t adp_cntl2 = KX132_ADP_CNTL2_ADP_WB_ISEL  |
					     KX132_ADP_CNTL2_RMS_WB_OSEL  |
					     KX132_ADP_CNTL2_ADP_FLT1_BYP |
					     KX132_ADP_CNTL2_ADP_FLT2_BYP |
					     KX132_ADP_CNTL2_ADP_RMS_OSEL;

			ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_ADP_CNTL2, adp_cntl2);
			if (ret < 0) {
				kx132_set_standby(dev, false);
				return ret;
			}

			ret = kx132_set_standby(dev, false);
			if (ret < 0) {
				return ret;
			}

			/* Set threshold and debounce */
			ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_WUFTH,
						    cfg->wakeup_threshold & 0xFF);
			if (ret < 0) {
				return ret;
			}

			uint8_t btswufth = (cfg->wakeup_threshold >> 8) & 0x07;

			ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_BTSWUFTH, btswufth);
			if (ret < 0) {
				return ret;
			}

			ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_WUFC,
						    cfg->wakeup_debounce);
			if (ret < 0) {
				return ret;
			}

			/* Enable ADP engine and arm WUF state machine */
			ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_CNTL5,
						    KX132_CNTL5_ADPE | KX132_CNTL5_MAN_SLEEP);
			if (ret < 0) {
				return ret;
			}

			inc4 |= KX132_INC4_WUFI1;
		} else {
			/* --- Disarm WUF + ADP engine --- */

			ret = kx132_set_standby(dev, true);
			if (ret < 0) {
				return ret;
			}

			/* Disable wake-up engine */
			ret = i2c_reg_update_byte_dt(&cfg->i2c, KX132_REG_CNTL4,
						     KX132_CNTL4_WUFE, 0);
			if (ret < 0) {
				kx132_set_standby(dev, false);
				return ret;
			}

			ret = kx132_set_standby(dev, false);
			if (ret < 0) {
				return ret;
			}

			/* Disable ADP engine */
			ret = i2c_reg_update_byte_dt(&cfg->i2c, KX132_REG_CNTL5,
						     KX132_CNTL5_ADPE, 0);
			if (ret < 0) {
				return ret;
			}

			inc4 &= ~KX132_INC4_WUFI1;
		}
		break;

	case SENSOR_TRIG_DATA_READY:
		data->drdy_handler = handler;
		data->drdy_trig = trig;
		if (handler) {
			inc4 |= KX132_INC4_DRDYI1;
		} else {
			inc4 &= ~KX132_INC4_DRDYI1;
		}
		break;
	default:
		return -ENOTSUP;
	}

	return i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_INC4, inc4);
}

int kx132_init_interrupt(const struct device *dev)
{
	const struct kx132_config *cfg = dev->config;
	struct kx132_data *data = dev->data;
	int ret;

	if (!cfg->gpio_int.port) {
		LOG_INF("No interrupt GPIO configured");
		return 0;
	}

	if (!gpio_is_ready_dt(&cfg->gpio_int)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	data->dev = dev;

	ret = gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure GPIO: %d", ret);
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, kx132_gpio_callback,
			   BIT(cfg->gpio_int.pin));

	ret = gpio_add_callback(cfg->gpio_int.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add GPIO callback: %d", ret);
		return ret;
	}

#ifdef CONFIG_KX132_TRIGGER_OWN_THREAD
	k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);
	k_thread_create(&data->thread, data->thread_stack,
			CONFIG_KX132_THREAD_STACK_SIZE,
			kx132_thread, data, NULL, NULL,
			K_PRIO_COOP(CONFIG_KX132_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_KX132_TRIGGER_GLOBAL_THREAD)
	k_work_init(&data->work, kx132_work_cb);
#endif

	/* Configure INT1 pin: enabled, active high, latched until INT_REL read */
	uint8_t inc1 = KX132_INC1_IEN1 | KX132_INC1_IEA1 | KX132_INC1_IEL1;

	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_INC1, inc1);
	if (ret < 0) {
		return ret;
	}

	/* Enable GPIO interrupt */
	ret = gpio_pin_interrupt_configure_dt(&cfg->gpio_int,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure GPIO interrupt: %d", ret);
		return ret;
	}

	return 0;
}
