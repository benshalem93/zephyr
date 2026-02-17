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
				      KX132_CNTL5_MAN_SLEEP);
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
			inc4 |= KX132_INC4_WUFI1;
		} else {
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

	/* Configure INT1 pin: enabled, active high, latched */
	uint8_t inc1 = KX132_INC1_IEN1 | KX132_INC1_IEA1;

	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_INC1, inc1);
	if (ret < 0) {
		return ret;
	}

	/* Enable all axes for wake-up detection */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_INC2,
				    KX132_INC2_ALL_AXES);
	if (ret < 0) {
		return ret;
	}

	/* Enable wake-up engine with relative threshold mode */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_CNTL4,
				    KX132_CNTL4_WUFE | KX132_CNTL4_TH_MODE);
	if (ret < 0) {
		return ret;
	}

	/* Set wake-up engine ODR to 12.5Hz (80ms per debounce count) */
	ret = i2c_reg_update_byte_dt(&cfg->i2c, KX132_REG_CNTL3,
				     KX132_CNTL3_OWUF_MASK, KX132_OWUF_12_5HZ);
	if (ret < 0) {
		return ret;
	}

	/* Set wake-up threshold (WUFTH register = lower 8 bits) */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_WUFTH,
				    cfg->wakeup_threshold & 0xFF);
	if (ret < 0) {
		return ret;
	}

	/* Set BTSWUFTH (upper bits of thresholds) */
	uint8_t btswufth = (cfg->wakeup_threshold >> 8) & 0x07;

	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_BTSWUFTH, btswufth);
	if (ret < 0) {
		return ret;
	}

	/* Set wake-up debounce counter */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, KX132_REG_WUFC,
				    cfg->wakeup_debounce);
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
