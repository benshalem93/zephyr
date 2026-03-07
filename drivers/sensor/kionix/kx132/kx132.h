/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_KX132_H_
#define ZEPHYR_DRIVERS_SENSOR_KX132_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

/* Device identification */
#define KX132_WHO_AM_I_VAL	0x3D

/* Register addresses */
#define KX132_REG_MAN_ID	0x00
#define KX132_REG_PART_ID	0x01
#define KX132_REG_XOUT_L	0x08
#define KX132_REG_XOUT_H	0x09
#define KX132_REG_YOUT_L	0x0A
#define KX132_REG_YOUT_H	0x0B
#define KX132_REG_ZOUT_L	0x0C
#define KX132_REG_ZOUT_H	0x0D
#define KX132_REG_COTR		0x12
#define KX132_REG_WHO_AM_I	0x13
#define KX132_REG_INS1		0x16
#define KX132_REG_INS2		0x17
#define KX132_REG_INS3		0x18
#define KX132_REG_STATUS_REG	0x19
#define KX132_REG_INT_REL	0x1A
#define KX132_REG_CNTL1	0x1B
#define KX132_REG_CNTL2	0x1C
#define KX132_REG_CNTL3	0x1D
#define KX132_REG_CNTL4	0x1E
#define KX132_REG_CNTL5	0x1F
#define KX132_REG_CNTL6	0x20
#define KX132_REG_ODCNTL	0x21
#define KX132_REG_INC1		0x22
#define KX132_REG_INC2		0x23
#define KX132_REG_INC3		0x24
#define KX132_REG_INC4		0x25
#define KX132_REG_INC5		0x26
#define KX132_REG_INC6		0x27
#define KX132_REG_WUFTH	0x49
#define KX132_REG_BTSWUFTH	0x4A
#define KX132_REG_BTSTH	0x4B
#define KX132_REG_BTSC		0x4C
#define KX132_REG_WUFC		0x4D

/* CNTL1 bits */
#define KX132_CNTL1_PC1	BIT(7)
#define KX132_CNTL1_RES	BIT(6)
#define KX132_CNTL1_DRDYE	BIT(5)
#define KX132_CNTL1_GSEL_MASK	(BIT(4) | BIT(3))
#define KX132_CNTL1_GSEL_SHIFT	3
#define KX132_CNTL1_TDTE	BIT(2)
#define KX132_CNTL1_TPE	BIT(0)

/* CNTL1 GSEL values */
#define KX132_GSEL_2G		0
#define KX132_GSEL_4G		1
#define KX132_GSEL_8G		2
#define KX132_GSEL_16G		3

/* CNTL2 bits */
#define KX132_CNTL2_SRST	BIT(7)
#define KX132_CNTL2_COTC	BIT(6)

/* CNTL3 bits - OWUF field */
#define KX132_CNTL3_OWUF_MASK	(BIT(2) | BIT(1) | BIT(0))
#define KX132_OWUF_0_781HZ	0x00
#define KX132_OWUF_1_563HZ	0x01
#define KX132_OWUF_3_125HZ	0x02
#define KX132_OWUF_6_25HZ	0x03
#define KX132_OWUF_12_5HZ	0x04
#define KX132_OWUF_25HZ	0x05
#define KX132_OWUF_50HZ	0x06
#define KX132_OWUF_100HZ	0x07

/* CNTL4 bits */
#define KX132_CNTL4_C_MODE	BIT(7)
#define KX132_CNTL4_TH_MODE	BIT(6)
#define KX132_CNTL4_WUFE	BIT(5)
#define KX132_CNTL4_BTSE	BIT(4)
#define KX132_CNTL4_PR_MODE	BIT(3)

/* CNTL5 bits */
#define KX132_CNTL5_ADPE	BIT(4)
#define KX132_CNTL5_MAN_WAKE	BIT(1)
#define KX132_CNTL5_MAN_SLEEP	BIT(0)

/* ODCNTL bits */
#define KX132_ODCNTL_IIR_BYPASS	BIT(7)
#define KX132_ODCNTL_LPRO	BIT(6)
#define KX132_ODCNTL_FSTUP	BIT(5)
#define KX132_ODCNTL_OSA_MASK	0x0F

/* Advanced Data Path Control Registers */
#define KX132_REG_ADP_CNTL1	0x64
#define KX132_REG_ADP_CNTL2	0x65

/* ADP_CNTL1 bits */
#define KX132_ADP_CNTL1_RMS_AVC_MASK	(BIT(6) | BIT(5) | BIT(4))
#define KX132_ADP_CNTL1_RMS_AVC_SHIFT	4
#define KX132_ADP_CNTL1_OADP_MASK	0x0F

/* RMS_AVC values: number of samples averaged for RMS output */
#define KX132_RMS_AVC_2		0x00
#define KX132_RMS_AVC_4		0x01
#define KX132_RMS_AVC_8		0x02
#define KX132_RMS_AVC_16	0x03
#define KX132_RMS_AVC_32	0x04
#define KX132_RMS_AVC_64	0x05
#define KX132_RMS_AVC_128	0x06
#define KX132_RMS_AVC_256	0x07

/* ADP_CNTL2 bits (TRM p.46) */
#define KX132_ADP_CNTL2_ADP_BUF_SEL	BIT(7) /* Route ADP to sample buffer */
#define KX132_ADP_CNTL2_ADP_WB_ISEL	BIT(6) /* ADP data to WUF/BTS engines */
#define KX132_ADP_CNTL2_RMS_WB_OSEL	BIT(5) /* RMS (not filtered) to WUF/BTS */
#define KX132_ADP_CNTL2_ADP_FLT2_BYP	BIT(4) /* Bypass filter-2 */
#define KX132_ADP_CNTL2_ADP_FLT1_BYP	BIT(3) /* Bypass filter-1 */
/* Bit 2: reserved */
#define KX132_ADP_CNTL2_ADP_RMS_OSEL	BIT(1) /* RMS to XADP/YADP/ZADP regs; required if RMS_WB_OSEL=1 */
#define KX132_ADP_CNTL2_ADP_F2_HP	BIT(0) /* Filter-2 as high-pass */

/* INC1 bits */
#define KX132_INC1_PW1_MASK	(BIT(7) | BIT(6))
#define KX132_INC1_IEN1	BIT(5)
#define KX132_INC1_IEA1	BIT(4)
#define KX132_INC1_IEL1	BIT(3)

/* INC2 bits - WUF/BTS axis mask */
#define KX132_INC2_AOI		BIT(6)
#define KX132_INC2_XNWUE	BIT(5)
#define KX132_INC2_XPWUE	BIT(4)
#define KX132_INC2_YNWUE	BIT(3)
#define KX132_INC2_YPWUE	BIT(2)
#define KX132_INC2_ZNWUE	BIT(1)
#define KX132_INC2_ZPWUE	BIT(0)
#define KX132_INC2_ALL_AXES	0x3F

/* INC4 bits - interrupt routing to INT1 */
#define KX132_INC4_FFI1	BIT(7)
#define KX132_INC4_BFI1	BIT(6)
#define KX132_INC4_WMI1	BIT(5)
#define KX132_INC4_DRDYI1	BIT(4)
#define KX132_INC4_BTSI1	BIT(3)
#define KX132_INC4_TDTI1	BIT(2)
#define KX132_INC4_WUFI1	BIT(1)
#define KX132_INC4_TPI1	BIT(0)

/* INS2 bits */
#define KX132_INS2_FFS		BIT(7)
#define KX132_INS2_BFI		BIT(6)
#define KX132_INS2_WMI		BIT(5)
#define KX132_INS2_DRDY	BIT(4)
#define KX132_INS2_TDTS_MASK	(BIT(3) | BIT(2))
#define KX132_INS2_TPS		BIT(0)

/* INS3 bits */
#define KX132_INS3_WUFS	BIT(7)
#define KX132_INS3_BTS		BIT(6)

/* STATUS_REG bits */
#define KX132_STATUS_INT	BIT(4)
#define KX132_STATUS_WAKE	BIT(0)

/* Sensitivity in ug/LSB for 16-bit output */
#define KX132_GAIN_2G		61
#define KX132_GAIN_4G		122
#define KX132_GAIN_8G		244
#define KX132_GAIN_16G		488

/* OSA (Output Data Rate) register values */
#define KX132_OSA_0_781HZ	0x00
#define KX132_OSA_1_563HZ	0x01
#define KX132_OSA_3_125HZ	0x02
#define KX132_OSA_6_25HZ	0x03
#define KX132_OSA_12_5HZ	0x04
#define KX132_OSA_25HZ		0x05
#define KX132_OSA_50HZ		0x06
#define KX132_OSA_100HZ	0x07
#define KX132_OSA_200HZ	0x08
#define KX132_OSA_400HZ	0x09
#define KX132_OSA_800HZ	0x0A
#define KX132_OSA_1600HZ	0x0B
#define KX132_OSA_3200HZ	0x0C
#define KX132_OSA_6400HZ	0x0D
#define KX132_OSA_12800HZ	0x0E
#define KX132_OSA_25600HZ	0x0F

struct kx132_config {
	struct i2c_dt_spec i2c;
	uint16_t odr;
	uint8_t range;
#ifdef CONFIG_KX132_TRIGGER
	struct gpio_dt_spec gpio_int;
	uint16_t wakeup_threshold;
	uint8_t wakeup_debounce;
#endif
};

struct kx132_data {
	int16_t acc[3];
	uint16_t gain;
	uint8_t cntl1_val;

#ifdef CONFIG_KX132_TRIGGER
	uint8_t rms_avc;

	const struct device *dev;
	struct gpio_callback gpio_cb;

	sensor_trigger_handler_t motion_handler;
	const struct sensor_trigger *motion_trig;
	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trig;

#ifdef CONFIG_KX132_TRIGGER_OWN_THREAD
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_KX132_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_KX132_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_KX132_TRIGGER */
};

#ifdef CONFIG_KX132_TRIGGER
int kx132_set_standby(const struct device *dev, bool standby);

int kx132_trigger_set(const struct device *dev,
		      const struct sensor_trigger *trig,
		      sensor_trigger_handler_t handler);

int kx132_init_interrupt(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_KX132_H_ */
