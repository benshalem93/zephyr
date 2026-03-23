/*
 * Copyright (c) 2024 NXP
 * Copyright (c) 2018 Phytec Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

K_SEM_DEFINE(sem_irq,      0, 1);
K_SEM_DEFINE(sem_periodic, 0, 1);

static void print_uptime(void)
{
	int64_t ms  = k_uptime_get();
	int     hh  = (int)(ms / 3600000);
	int     mm  = (int)((ms % 3600000) / 60000);
	int     ss  = (int)((ms % 60000) / 1000);
	int     fms = (int)(ms % 1000);

	printf("%02d:%02d:%02d.%03d  ", hh, mm, ss, fms);
}

static void motion_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trigger)
{
	ARG_UNUSED(trigger);

	if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ) < 0) {
		printf("ERROR: SENSOR_CHAN_ACCEL_XYZ fetch failed\n");
	}

	k_sem_give(&sem_irq);
}

static void periodic_expiry(struct k_timer *t)
{
	ARG_UNUSED(t);
	k_sem_give(&sem_periodic);
}

K_TIMER_DEFINE(periodic_timer, periodic_expiry, NULL);

int main(void)
{
	struct sensor_value data[3];
	struct sensor_value val_th, val_dur;
	const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(accel0));

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return 0;
	}

	/*
	 * Arm the WUF engine.  trigger_set writes to the sensor:
	 *   - CNTL4:  C_MODE=1, TH_MODE=1, WUFE=1, PR_MODE=1
	 *   - WUFTH:  initial threshold from devicetree (overridden by attr_set below)
	 *   - WUFC:   initial debounce  from devicetree (overridden by attr_set below)
	 *   - CNTL5:  MAN_SLEEP=1 — transitions the WUF state machine to the sleep
	 *             state, making it ready to detect the next motion event
	 *
	 * ADP is disabled — raw accelerometer output feeds the WUF engine
	 * directly (no IIR filter, no RMS path).
	 */
	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_MOTION,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	if (sensor_trigger_set(dev, &trig, motion_trigger_handler) < 0) {
		printf("Could not set motion trigger\n");
		return 0;
	}

	/*
	 * SENSOR_ATTR_SLOPE_TH — wake-up threshold in milli-g
	 *   (1 mg = g/1000 = 9.80665e-3 m/s²)
	 *
	 * The driver converts to 11-bit WUFTH counts: counts = mg * 512 / (range_g * 1000)
	 *
	 * 500 mg → 128 counts at ±2 g — strong knock or deliberate shake
	 */
	val_th.val1 = 500; /* mg */
	val_th.val2 = 0;
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &val_th);

	/*
	 * SENSOR_ATTR_SLOPE_DUR — debounce window in milliseconds
	 *
	 * The WUF engine requires the threshold to be exceeded continuously
	 * for this duration before firing the interrupt.  The driver converts
	 * to WUFC counts using the active OWUF ODR (1.563 Hz → 1 count ≈ 640 ms).
	 *
	 * 5000 ms → 7 counts at 1.563 Hz ≈ 4.5 s of sustained motion required.
	 */
	val_dur.val1 = 5000; /* ms */
	val_dur.val2 = 0;
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &val_dur);

	printf("Waiting for motion...\n");
	printf("  Threshold = %d mg  (raw accel delta to wake)\n", val_th.val1);
	printf("  Debounce  = %d ms (sustained motion required)\n\n", val_dur.val1);

	/* Start 12.5 Hz periodic timer (80 ms period) */
	k_timer_start(&periodic_timer, K_MSEC(80), K_MSEC(80));

	struct k_poll_event events[2] = {
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY, &sem_irq, 0),
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY, &sem_periodic, 0),
	};

	while (1) {
		k_poll(events, 2, K_FOREVER);

		if (events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
			events[0].state = K_POLL_STATE_NOT_READY;
			k_sem_take(&sem_irq, K_NO_WAIT);
			sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, data);
			printf("\x1b[1;33m");
			print_uptime();
			printf("*** MOTION  X=%8.4f  Y=%8.4f  Z=%8.4f ***\x1b[0m\n",
			       sensor_value_to_double(&data[0]),
			       sensor_value_to_double(&data[1]),
			       sensor_value_to_double(&data[2]));
		}

		if (events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
			events[1].state = K_POLL_STATE_NOT_READY;
			k_sem_take(&sem_periodic, K_NO_WAIT);
			if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ) == 0) {
				sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, data);
				printf("\x1b[36m");
				print_uptime();
				printf("Sample      X=%8.4f  Y=%8.4f  Z=%8.4f\x1b[0m\n",
				       sensor_value_to_double(&data[0]),
				       sensor_value_to_double(&data[1]),
				       sensor_value_to_double(&data[2]));
			}
		}
	}
}
