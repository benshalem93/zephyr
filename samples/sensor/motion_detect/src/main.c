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
	 * Arm the WUF + ADP engine.  trigger_set writes to the sensor:
	 *   - INC2:      all-axis wake-up enabled
	 *   - CNTL4:     WUFE=1 (WUF on), TH_MODE=1 (relative / delta-RMS mode)
	 *   - ADP_CNTL1: OADP = same ODR as main accel; RMS_AVC from attr above
	 *   - ADP_CNTL2: both filters bypassed (raw → RMS), RMS routed to WUF
	 *   - WUFTH:     wakeup-threshold from devicetree (see overlay)
	 *   - WUFC:      wakeup-debounce  from devicetree (see overlay)
	 *   - CNTL5:     ADPE=1 (enable ADP), MAN_SLEEP=1 (arm WUF state machine)
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
	 * SENSOR_ATTR_SLOPE_TH — wake-up threshold (overrides devicetree value)
	 *
	 * In TH_MODE=1 (relative), the WUF engine computes:
	 *   |RMS_new - RMS_old| > WUFTH  →  wake
	 *
	 * The register stores counts in the accelerometer's native scale.
	 * attr_set converts: counts = val.val1 [mg] * 256 / 1000.
	 *
	 * At 2 g range, full-scale = 2000 mg = 32768 counts, so 1 count ≈ 0.06 mg.
	 *
	 * 50 mg  → 12 counts  → very sensitive, catches a breath on the desk
	 * 100 mg → 25 counts  → light tap on the table         ← chosen
	 * 500 mg → 128 counts → strong knock or deliberate shake
	 *
	 * Setting 100 mg here: a very gentle tap on the PCB will wake the MCU,
	 * but normal office background vibration will not.
	 */
	val.val1 = 100; /* mg */
	val.val2 = 0;
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &val);

	/*
	 * SENSOR_ATTR_SLOPE_DUR — debounce counter (WUFC register)
	 *
	 * The WUF engine requires the threshold to be exceeded for WUFC
	 * consecutive RMS windows before firing the interrupt.
	 *
	 * WUFC=1: fire on the first window that exceeds threshold.
	 *         Best for brief events (a single tap ≈ 1–2 windows long).
	 * WUFC=3: require 3 consecutive windows → 3 × 333 ms ≈ 1 s of motion.
	 *         Good for filtering out single vibration spikes.
	 *
	 * For small motion detection we use WUFC=1: we do not want a short
	 * tap to go undetected just because it didn't last long enough.
	 */
	val.val1 = 1;
	val.val2 = 0;
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &val);

	printf("Waiting for motion...\n");
	printf("  ODR       = 12 Hz  (accel sampling rate)\n");
	printf("  RMS window= 4 smp  (WUF checks every ~333 ms)\n");
	printf("  Threshold = 100 mg (delta-RMS to wake)\n");
	printf("  Debounce  = 1 win  (single window sufficient)\n\n");

	while (1) {
		/* CPU sleeps here (tickless WFI). Wakes only when KX132 pulls
		 * the INT1 GPIO line, which happens when delta-RMS > threshold.
		 */
		k_sem_take(&sem, K_FOREVER);

		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, data);

		printf("Motion! [m/s^2] X=%8.4f  Y=%8.4f  Z=%8.4f\n",
		       sensor_value_to_double(&data[0]),
		       sensor_value_to_double(&data[1]),
		       sensor_value_to_double(&data[2]));
	}
}
