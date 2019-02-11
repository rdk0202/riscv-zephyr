/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_LSM303C_H__
#define __SENSOR_LSM303C_H__

#include <zephyr/types.h>
#include <i2c.h>
#include <sensor.h>
#include <misc/util.h>

#define LOG_LEVEL CONFIG_SYS_LOG_SENSOR_LEVEL
#include <logging/log.h>

/* Accelerometer Registers */

#define LSM303C_REG_WHO_AM_I_A	0x0F
#define LSM303C_VAL_WHO_AM_I_A	0x41

#define LSM303C_REG_ACT_THS_A   0x1E
#define LSM303C_REG_ACT_DUR_A   0x1F

#define LSM303C_REG_CTRL_REG1_A			0x20
#define LSM303C_REG_CTRL_REG1_A_HR		BIT(7)
#define LSM303C_REG_CTRL_REG1_A_ODR_MASK	(BIT(4) | BIT(5) | BIT(6))
#define LSM303C_REG_CTRL_REG1_A_ODR_SHIFT	4
#define LSM303C_REG_CTRL_REG1_A_BDU		BIT(3)
#define LSM303C_REG_CTRL_REG1_A_ZEN		BIT(2)
#define LSM303C_REG_CTRL_REG1_A_YEN		BIT(1)
#define LSM303C_REG_CTRL_REG1_A_XEN		BIT(0)

#define LSM303C_REG_CTRL_REG2_A			0x21
#define LSM303C_REG_CTRL_REG2_A_DFC_SHIFT	5
#define LSM303C_REG_CTRL_REG2_A_DFC_MASK	(BIT(5) | BIT(6))
#define LSM303C_REG_CTRL_REG2_A_HPM_SHIFT	3
#define LSM303C_REG_CTRL_REG2_A_HPM_MASK	(BIT(3) | BIT(4))
#define LSM303C_REG_CTRL_REG2_A_FDS		BIT(2)
#define LSM303C_REG_CTRL_REG2_A_HPI2		BIT(0)
#define LSM303C_REG_CTRL_REG2_A_HPI1		BIT(0)

#define LSM303C_REG_CTRL_REG3_A			0x22
#define LSM303C_REG_CTRL_REG3_A_FIFO_EN		BIT(7)
#define LSM303C_REG_CTRL_REG3_A_STOP_FTH	BIT(6)
#define LSM303C_REG_CTRL_REG3_A_INT_XL_INACT	BIT(5)
#define LSM303C_REG_CTRL_REG3_A_INT_XL_IG2	BIT(4)
#define LSM303C_REG_CTRL_REG3_A_INT_XL_IG1	BIT(3)
#define LSM303C_REG_CTRL_REG3_A_INT_XL_OVR	BIT(2)
#define LSM303C_REG_CTRL_REG3_A_INT_XL_FTH	BIT(1)
#define LSM303C_REG_CTRL_REG3_A_INT_XL_DRD	BIT(0)

#define LSM303C_REG_CTRL_REG4_A			0x23
#define LSM303C_REG_CTRL_REG4_A_BW_SHIFT	6
#define LSM303C_REG_CTRL_REG4_A_BW_MASK		(BIT(6) | BIT(7))
#define LSM303C_REG_CTRL_REG4_A_FS_SHIFT	4
#define LSM303C_REG_CTRL_REG4_A_FS_MASK		(BIT(4) | BIT(5))
#define LSM303C_REG_CTRL_REG4_A_SCALE_ODR	BIT(3)
#define LSM303C_REG_CTRL_REG4_A_IF_ADD_INC	BIT(2)
#define LSM303C_REG_CTRL_REG4_A_I2C_DISABLE	BIT(1)
#define LSM303C_REG_CTRL_REG4_A_SIM		BIT(0)

#define LSM303C_REG_CTRL_REG5_A			0x24
#define LSM303C_REG_CTRL_REG5_A_DEBUG		BIT(7)
#define LSM303C_REG_CTRL_REG5_A_SOFT_RESET	BIT(6)
#define LSM303C_REG_CTRL_REG5_A_DEC_SHIFT	4
#define LSM303C_REG_CTRL_REG5_A_DEC_MASK	(BIT(4) | BIT(5))
#define LSM303C_REG_CTRL_REG5_A_ST_SHIFT	2
#define LSM303C_REG_CTRL_REG5_A_ST_MASK		(BIT(2) | BIT(3))
#define LSM303C_REG_CTRL_REG5_A_H_LACTIVE	BIT(1)
#define LSM303C_REG_CTRL_REG5_A_PP_OD		BIT(0)

#define LSM303C_REG_CTRL_REG6_A			0x25
#define LSM303C_REG_CTRL_REG6_A_BOOT		BIT(7)

#define LSM303C_REG_CTRL_REG7_A			0x26
#define LSM303C_REG_CTRL_REG7_A_DCRM_SHIFT	4
#define LSM303C_REG_CTRL_REG7_A_DCRM_MASK	(BIT(4) | BIT(5))
#define LSM303C_REG_CTRL_REG7_A_LIR_SHIFT	2
#define LSM303C_REG_CTRL_REG7_A_LIR_MASK	(BIT(2) | BIT(3))
#define LSM303C_REG_CTRL_REG7_A_4D_IG_SHIFT	0
#define LSM303C_REG_CTRL_REG7_A_4D_IG_MASK	(BIT(0) | BIT(1))

#define LSM303C_REG_STATUS_A			0x27
#define LSM303C_REG_STATUS_A_ZYXOR		BIT(7)
#define LSM303C_REG_STATUS_A_ZOR		BIT(6)
#define LSM303C_REG_STATUS_A_YOR		BIT(5)
#define LSM303C_REG_STATUS_A_XOR		BIT(4)
#define LSM303C_REG_STATUS_A_ZYXDA		BIT(3)
#define LSM303C_REG_STATUS_A_ZDA		BIT(2)
#define LSM303C_REG_STATUS_A_YDA		BIT(1)
#define LSM303C_REG_STATUS_A_XDA		BIT(0)

#define LSM303C_REG_OUT_X_L_A                   0x28
#define LSM303C_REG_OUT_X_H_A                   0x29
#define LSM303C_REG_OUT_Y_L_A                   0x2A
#define LSM303C_REG_OUT_Y_H_A                   0x2B
#define LSM303C_REG_OUT_Z_L_A                   0x2C
#define LSM303C_REG_OUT_Z_H_A                   0x2D

#define LSM303C_REG_FIFO_CTRL                   0x2E
#define LSM303C_REG_FIFO_CTRL_FMODE_SHIFT	5
#define LSM303C_REG_FIFO_CTRL_FMODE_MASK	(BIT(7) | BIT(6) | BIT(5))
#define LSM303C_REG_FIFO_CTRL_FTH_SHIFT		0
#define LSM303C_REG_FIFO_CTRL_FTH_MASK		(BIT(4) | BIT(3) | BIT(2) | \
						 BIT(1) | BIT(0))

#define LSM303C_REG_FIFO_SRC                    0x2F
#define LSM303C_REG_FIFO_SRC_FTH		BIT(7)
#define LSM303C_REG_FIFO_SRC_OVR		BIT(6)
#define LSM303C_REG_FIFO_SRC_EMPTY		BIT(5)
#define LSM303C_REG_FIFO_SRC_FSS4_SHIFT		0
#define LSM303C_REG_FIFO_SRC_FSS4_MASK		(BIT(4) | BIT(3) | BIT(2) | \
						 BIT(1) | BIT(0))

#define LSM303C_REG_INT_GEN_CFG_A               0x30

#define LSM303C_REG_INT_GEN_THS_X_A             0x32
#define LSM303C_REG_INT_GEN_THS_Y_A             0x33
#define LSM303C_REG_INT_GEN_THS_Z_A             0x34
#define LSM303C_REG_INT_GEN_DUR_A               0x35

#define LSM303C_REG_XL_REFERENCE_A		0x3A
#define LSM303C_REG_XH_REFERENCE_A		0x3B
#define LSM303C_REG_YL_REFERENCE_A		0x3C
#define LSM303C_REG_YH_REFERENCE_A		0x3D
#define LSM303C_REG_ZL_REFERENCE_A		0x3E
#define LSM303C_REG_ZH_REFERENCE_A		0x3F

/* Magnetometer and Thermometer Registers */

#define LSM303C_REG_WHO_AM_I_M			0x0F
#define LSM303C_VAL_WHO_AM_I_M			0x3D

#define LSM303C_REG_CTRL_REG1_M			0x20
#define LSM303C_REG_CTRL_REG1_M_TEMP_EN		BIT(7)
#define LSM303C_REG_CTRL_REG1_M_OM_SHIFT	5
#define LSM303C_REG_CTRL_REG1_M_OM_MASK		(BIT(5) | BIT(6))
#define LSM303C_REG_CTRL_REG1_M_OM_ULTRAHIGH	3
#define LSM303C_REG_CTRL_REG1_M_DO_SHIFT	2
#define LSM303C_REG_CTRL_REG1_M_DO_MASK		(BIT(2) | BIT(3) | BIT(4))
#define LSM303C_REG_CTRL_REG1_M_ST		BIT(0)

#define LSM303C_REG_CTRL_REG2_M			0x21
#define LSM303C_REG_CTRL_REG2_M_FS_SHIFT	5
#define LSM303C_REG_CTRL_REG2_M_FS_MASK		(BIT(5) | BIT(6))
#define LSM303C_REG_CTRL_REG2_M_REBOOT		BIT(3)
#define LSM303C_REG_CTRL_REG2_M_SOFT_RST	BIT(2)

#define LSM303C_REG_CTRL_REG3_M			0x22
#define LSM303C_REG_CTRL_REG3_M_I2C_DISABLE	BIT(7)
#define LSM303C_REG_CTRL_REG3_M_LP		BIT(5)
#define LSM303C_REG_CTRL_REG3_M_SIM		BIT(2)
#define LSM303C_REG_CTRL_REG3_M_MD_SHIFT	0
#define LSM303C_REG_CTRL_REG3_M_MD_MASK		(BIT(0) | BIT(1))
#define LSM303C_REG_CTRL_REG3_M_MD_CONTINUOUS	0

#define LSM303C_REG_CTRL_REG4_M			0x23
#define LSM303C_REG_CTRL_REG4_M_OMZ_SHIFT	2
#define LSM303C_REG_CTRL_REG4_M_OMZ_MASK	(BIT(2) | BIT(3))
#define LSM303C_REG_CTRL_REG4_M_OMZ_ULTRAHIGH	3
#define LSM303C_REG_CTRL_REG4_M_BLE		BIT(1)

#define LSM303C_REG_CTRL_REG5_M			0x24
#define LSM303C_REG_CTRL_REG5_M_BDU		BIT(6)

#define LSM303C_REG_STATUS_M			0x27
#define LSM303C_REG_STATUS_M_ZYXOR		BIT(7)
#define LSM303C_REG_STATUS_M_ZOR		BIT(6)
#define LSM303C_REG_STATUS_M_YOR		BIT(5)
#define LSM303C_REG_STATUS_M_XOR		BIT(4)
#define LSM303C_REG_STATUS_M_ZYXDA		BIT(3)
#define LSM303C_REG_STATUS_M_ZDA		BIT(2)
#define LSM303C_REG_STATUS_M_YDA		BIT(1)
#define LSM303C_REG_STATUS_M_XDA		BIT(0)

#define LSM303C_REG_OUT_X_L_M                   0x28
#define LSM303C_REG_OUT_X_H_M                   0x29
#define LSM303C_REG_OUT_Y_L_M                   0x2A
#define LSM303C_REG_OUT_Y_H_M                   0x2B
#define LSM303C_REG_OUT_Z_L_M                   0x2C
#define LSM303C_REG_OUT_Z_H_M                   0x2D
#define LSM303C_REG_TEMP_L_M			0x2E
#define LSM303C_REG_TEMP_H_M			0x2F

#define LSM303C_REG_INT_CFG_M			0x30
#define LSM303C_REG_INT_SRC_M			0x31
#define LSM303C_REG_INT_THS_L_M			0x32
#define LSM303C_REG_INT_THS_H_M			0x33

/* Conversion Constants */

#ifdef CONFIG_FLOAT
#define SENSOR_G_DOUBLE		(SENSOR_G / 1000000.0)
#endif /* CONFIG_FLOAT */

/* Channel Enables */

#if defined(CONFIG_LSM303C_ACCEL_ENABLE_X_AXIS)
	#define LSM303C_ACCEL_ENABLE_X_AXIS	1
#else
	#define LSM303C_ACCEL_ENABLE_X_AXIS	0
#endif

#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Y_AXIS)
	#define LSM303C_ACCEL_ENABLE_Y_AXIS	1
#else
	#define LSM303C_ACCEL_ENABLE_Y_AXIS	0
#endif

#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Z_AXIS)
	#define LSM303C_ACCEL_ENABLE_Z_AXIS	1
#else
	#define LSM303C_ACCEL_ENABLE_Z_AXIS	0
#endif

#if defined(CONFIG_LSM303C_MAGN_ENABLE_X_AXIS)
	#define LSM303C_MAGN_ENABLE_X_AXIS	1
#else
	#define LSM303C_MAGN_ENABLE_X_AXIS	0
#endif

#if defined(CONFIG_LSM303C_MAGN_ENABLE_Y_AXIS)
	#define LSM303C_MAGN_ENABLE_Y_AXIS	1
#else
	#define LSM303C_MAGN_ENABLE_Y_AXIS	0
#endif

#if defined(CONFIG_LSM303C_MAGN_ENABLE_Z_AXIS)
	#define LSM303C_MAGN_ENABLE_Z_AXIS	1
#else
	#define LSM303C_MAGN_ENABLE_Z_AXIS	0
#endif

#if defined(CONFIG_LSM303C_ENABLE_TEMP)
	#define LSM303C_ENABLE_TEMP		1
#else
	#define LSM303C_ENABLE_TEMP		0
#endif

/* Scaling Values */

#if CONFIG_LSM303C_ACCEL_FULLSCALE == 2
	#define LSM303C_ACCEL_FULLSCALE_2G
#elif CONFIG_LSM303C_ACCEL_FULLSCALE == 4
	#define LSM303C_ACCEL_FULLSCALE_4G
#elif CONFIG_LSM303C_ACCEL_FULLSCALE == 8
	#define LSM303C_ACCEL_FULLSCALE_8G
#endif

#if defined(LSM303C_ACCEL_FULLSCALE_2G)
	#define LSM303C_DEFAULT_ACCEL_FULLSCALE		0
	#define LSM303C_DEFAULT_ACCEL_FULLSCALE_FACTOR	2
#elif defined(LSM303C_ACCEL_FULLSCALE_4G)
	#define LSM303C_DEFAULT_ACCEL_FULLSCALE		2
	#define LSM303C_DEFAULT_ACCEL_FULLSCALE_FACTOR	4
#elif defined(LSM303C_ACCEL_FULLSCALE_8G)
	#define LSM303C_DEFAULT_ACCEL_FULLSCALE		3
	#define LSM303C_DEFAULT_ACCEL_FULLSCALE_FACTOR	8
#endif

/* Accelerometer sample rate values */

#if CONFIG_LSM303C_ACCEL_SAMPLING_RATE == 0
	#define LSM303C_ACCEL_SAMPLING_RATE_0
#elif CONFIG_LSM303C_ACCEL_SAMPLING_RATE == 10
	#define LSM303C_ACCEL_SAMPLING_RATE_10
#elif CONFIG_LSM303C_ACCEL_SAMPLING_RATE == 50
	#define LSM303C_ACCEL_SAMPLING_RATE_50
#elif CONFIG_LSM303C_ACCEL_SAMPLING_RATE == 100
	#define LSM303C_ACCEL_SAMPLING_RATE_100
#elif CONFIG_LSM303C_ACCEL_SAMPLING_RATE == 200
	#define LSM303C_ACCEL_SAMPLING_RATE_200
#elif CONFIG_LSM303C_ACCEL_SAMPLING_RATE == 400
	#define LSM303C_ACCEL_SAMPLING_RATE_400
#elif CONFIG_LSM303C_ACCEL_SAMPLING_RATE == 800
	#define LSM303C_ACCEL_SAMPLING_RATE_800
#endif

#if defined(LSM303C_ACCEL_SAMPLING_RATE_0)
	#define LSM303C_DEFAULT_ACCEL_SAMPLING_RATE	0
#elif defined(LSM303C_ACCEL_SAMPLING_RATE_10)
	#define LSM303C_DEFAULT_ACCEL_SAMPLING_RATE	1
#elif defined(LSM303C_ACCEL_SAMPLING_RATE_50)
	#define LSM303C_DEFAULT_ACCEL_SAMPLING_RATE	2
#elif defined(LSM303C_ACCEL_SAMPLING_RATE_100)
	#define LSM303C_DEFAULT_ACCEL_SAMPLING_RATE	3
#elif defined(LSM303C_ACCEL_SAMPLING_RATE_200)
	#define LSM303C_DEFAULT_ACCEL_SAMPLING_RATE	4
#elif defined(LSM303C_ACCEL_SAMPLING_RATE_400)
	#define LSM303C_DEFAULT_ACCEL_SAMPLING_RATE	5
#elif defined(LSM303C_ACCEL_SAMPLING_RATE_800)
	#define LSM303C_DEFAULT_ACCEL_SAMPLING_RATE	6
#endif

/* Magnetometer full scale values */

#define LSM303C_DEFAULT_MAGN_FULLSCALE		16
#define LSM303C_DEFAULT_MAGN_FULLSCALE_FACTOR	16

/* Magnetometer sample rate values */

#if CONFIG_LSM303C_MAGN_SAMPLING_RATE == 5
	#define LSM303C_MAGN_SAMPLING_RATE_5
#elif CONFIG_LSM303C_MAGN_SAMPLING_RATE == 10
	#define LSM303C_MAGN_SAMPLING_RATE_10
#elif CONFIG_LSM303C_MAGN_SAMPLING_RATE == 20
	#define LSM303C_MAGN_SAMPLING_RATE_20
#elif CONFIG_LSM303C_MAGN_SAMPLING_RATE == 40
	#define LSM303C_MAGN_SAMPLING_RATE_40
#elif CONFIG_LSM303C_MAGN_SAMPLING_RATE == 80
	#define LSM303C_MAGN_SAMPLING_RATE_80
#endif

#if defined(LSM303C_MAGN_SAMPLING_RATE_5)
	#define LSM303C_DEFAULT_MAGN_SAMPLING_RATE	3
#elif defined(LSM303C_MAGN_SAMPLING_RATE_10)
	#define LSM303C_DEFAULT_MAGN_SAMPLING_RATE	4
#elif defined(LSM303C_MAGN_SAMPLING_RATE_20)
	#define LSM303C_DEFAULT_MAGN_SAMPLING_RATE	5
#elif defined(LSM303C_MAGN_SAMPLING_RATE_40)
	#define LSM303C_DEFAULT_MAGN_SAMPLING_RATE	6
#elif defined(LSM303C_MAGN_SAMPLING_RATE_80)
	#define LSM303C_DEFAULT_MAGN_SAMPLING_RATE	7
#endif

/* Typedefs */

struct lsm303c_config {
	char *i2c_master_dev_name;
	u16_t i2c_slave_addr;
	u16_t i2c_m_slave_addr;
};

struct lsm303c_data {
	struct device *i2c_master;

#if defined(CONFIG_LSM303C_ACCEL_ENABLE_X_AXIS)
	s16_t accel_sample_x;
#endif
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Y_AXIS)
	s16_t accel_sample_y;
#endif
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Z_AXIS)
	s16_t accel_sample_z;
#endif

#if defined(CONFIG_LSM303C_MAGN_ENABLE_X_AXIS)
	s16_t magn_sample_x;
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Y_AXIS)
	s16_t magn_sample_y;
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Z_AXIS)
	s16_t magn_sample_z;
#endif

#if defined(CONFIG_LSM303C_ENABLE_TEMP)
	s16_t temp_sample;
#endif
};

extern struct lsm303c_data lsm303c_data;

/* Function declarations */

int lsm303c_accel_reboot(struct device *dev);
int lsm303c_accel_init(struct device *dev);

int lsm303c_accel_axis_ctrl(struct device *dev, int x_en,
			    int y_en,
			    int z_en);
int lsm303c_accel_set_fs_raw(struct device *dev, u8_t fs);
int lsm303c_accel_set_odr_raw(struct device *dev, u8_t odr);

int lsm303c_sample_fetch_accel(struct device *dev);

void lsm303c_accel_convert(struct sensor_value *val,
			   s16_t raw_val,
			   s64_t scale);
int lsm303c_accel_get_channel(enum sensor_channel chan,
			      struct sensor_value *val,
			      struct lsm303c_data *data,
			      s64_t scale);
int lsm303c_accel_channel_get(enum sensor_channel chan,
			      struct sensor_value *val,
			      struct lsm303c_data *data);

int lsm303c_magn_reboot(struct device *dev);
int lsm303c_magn_init(struct device *dev);

int lsm303c_magn_axis_ctrl(struct device *dev,
			   int x_en,
			   int y_en,
			   int z_en);
int lsm303c_temp_enable(struct device *dev, int t_en);
int lsm303c_magn_set_fs_raw(struct device *dev, u8_t fs);
int lsm303c_magn_set_odr_raw(struct device *dev, u8_t odr);

int lsm303c_sample_fetch_magn(struct device *dev);
#if defined(CONFIG_LSM303C_ENABLE_TEMP)
int lsm303c_sample_fetch_temp(struct device *dev);
#endif
void lsm303c_magn_convert(struct sensor_value *val,
			  int raw_val,
			  s32_t scale);
int lsm303c_magn_get_channel(enum sensor_channel chan,
			     struct sensor_value *val,
			     struct lsm303c_data *data,
			     s32_t scale);
int lsm303c_magn_channel_get(enum sensor_channel chan,
			     struct sensor_value *val,
			     struct lsm303c_data *data);
void lsm303c_magn_channel_get_temp(struct sensor_value *val,
				   struct lsm303c_data *data);

int lsm303c_selftest(struct device *dev);

#endif /* __SENSOR_LSM303C_H__ */
