/*
 * Copyright (c) 2024 NXP
 * Copyright (c) 2018 Phytec Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

K_SEM_DEFINE(sem, 0, 1);

static void motion_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trigger)
{
	ARG_UNUSED(trigger);

	if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ) < 0) {
		printf("ERROR: SENSOR_CHAN_ACCEL_XYZ fetch failed\n");
	}

	k_sem_give(&sem);
}

int main(void)
{
	struct sensor_value data[3];
	struct sensor_value val;
	const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(accel0));

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return 0;
	}

	/*
	 * Arm the WUF engine.  trigger_set writes to the sensor:
	 *   - CNTL4:  C_MODE=1, TH_MODE=1, WUFE=1, PR_MODE=1
	 *   - WUFTH:  wakeup-threshold from devicetree (see overlay)
	 *   - WUFC:   wakeup-debounce  from devicetree (see overlay)
	 *   - CNTL5:  MAN_SLEEP=1 (arm WUF state machine)
	 *
	 * ADP is disabled — raw accelerometer output feeds the WUF engine
	 * directly (no filters, no RMS path).
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
	 * The driver converts to 11-bit WUFTH counts: counts = mg * 256 / 1000
	 * (1 count ≈ 3.9 mg, fixed and range-independent).
	 *
	 * 100 mg →  25 counts — light tap
	 * 500 mg → 128 counts — strong knock or deliberate shake  ← chosen
	 */
	val.val1 = 500; /* mg */
	val.val2 = 0;
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &val);

	/*
	 * SENSOR_ATTR_SLOPE_DUR — debounce window in milliseconds
	 *
	 * The WUF engine requires the threshold to be exceeded continuously
	 * for this duration before firing the interrupt.  The driver converts
	 * to WUFC counts using the active OWUF ODR (1.563 Hz → 1 count ≈ 640 ms).
	 *
	 * 3000 ms → 4 counts at 1.563 Hz ≈ 2.56 s of sustained motion required.
	 */
	val.val1 = 3000; /* ms */
	val.val2 = 0;
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &val);

	printf("Waiting for motion...\n");
	printf("  Sample rate = 1.5 Hz  (accel sampling rate)\n");
	printf("  Threshold = 500 mg  (raw accel delta to wake)\n");
	printf("  Debounce  = 3000 ms (sustained motion required)\n\n");

	while (1) {
		/* CPU sleeps here (tickless WFI). Wakes only when KX132 pulls
		 * the INT1 GPIO line, which happens when delta-RMS > threshold.
		 */
		k_sem_take(&sem, K_FOREVER);

		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, data);

		printf("Motion Detected: X=%8.4f  Y=%8.4f  Z=%8.4f\n",
		       sensor_value_to_double(&data[0]),
		       sensor_value_to_double(&data[1]),
		       sensor_value_to_double(&data[2]));
		k_msleep(500);
	}
}
