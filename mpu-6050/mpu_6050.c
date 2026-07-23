// SPDX-License-Identifier: GPL-2.0
/*
 * mpu6050.c - MPU6050 accelerometer/gyroscope acquisition driver
 *
 * reads accel + gyro data from the mpu6050 chip and sends it to
 * isfdas whenever a new sample is ready.
 *
 * runs at a fixed setting: 100 Hz, DLPF 42 Hz, gyro +-250 dps,
 * accel +-2g. not changeable at runtime, keeps things simple.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/bitops.h>

#include "isfdas.h"

/* register addresses */
#define MPU6050_REG_SMPLRT_DIV		0x19
#define MPU6050_REG_CONFIG		0x1A
#define MPU6050_REG_GYRO_CONFIG		0x1B
#define MPU6050_REG_ACCEL_CONFIG	0x1C
#define MPU6050_REG_INT_PIN_CFG		0x37
#define MPU6050_REG_INT_ENABLE		0x38
#define MPU6050_REG_INT_STATUS		0x3A
#define MPU6050_REG_ACCEL_XOUT_H	0x3B
#define MPU6050_REG_PWR_MGMT_1		0x6B
#define MPU6050_REG_WHO_AM_I		0x75

/* bit values */
#define MPU6050_PWR_RESET		BIT(7)
#define MPU6050_PWR_SLEEP		BIT(6)
#define MPU6050_PWR_CLK_PLL_XGYRO	0x01

#define MPU6050_INT_RD_CLEAR		BIT(4)
#define MPU6050_INT_DATA_RDY_EN		BIT(0)

/* fixed settings we always use */
#define MPU6050_DLPF_42HZ		0x03
#define MPU6050_SMPLRT_DIV_100HZ	0x09	/* 1000/(1+9) = 100 Hz */
#define MPU6050_GYRO_FS_250		(0x00 << 3)
#define MPU6050_ACCEL_FS_2G		(0x00 << 3)

#define MPU6050_WHO_AM_I_VAL		0x68

// accel(6) + temp(2) + gyro(6), starting at ACCEL_XOUT_H
#define MPU6050_SAMPLE_LEN		14

struct mpu6050_dev {
	struct i2c_client *client;
	struct regmap *regmap;
	int irq;
};

static const struct regmap_config mpu6050_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MPU6050_REG_WHO_AM_I,
};

// checks the chip's ID register to make sure it's really an mpu6050
static int mpu6050_chip_id_check(struct mpu6050_dev *mpu)
{
	unsigned int who_am_i;
	int ret;

	ret = regmap_read(mpu->regmap, MPU6050_REG_WHO_AM_I, &who_am_i);
	if (ret)
		return dev_err_probe(&mpu->client->dev, ret,
				     "failed to read WHO_AM_I\n");

	if (who_am_i != MPU6050_WHO_AM_I_VAL)
		return dev_err_probe(&mpu->client->dev, -ENODEV,
				     "unexpected WHO_AM_I 0x%02x\n", who_am_i);

	return 0;
}

// sets up the chip with our fixed settings and turns on the data
// ready interrupt
static int mpu6050_hw_init(struct mpu6050_dev *mpu)
{
	struct device *dev = &mpu->client->dev;
	int ret;

	ret = regmap_write(mpu->regmap, MPU6050_REG_PWR_MGMT_1,
			    MPU6050_PWR_RESET);
	if (ret)
		return dev_err_probe(dev, ret, "chip reset failed\n");

	msleep(100);

	ret = regmap_write(mpu->regmap, MPU6050_REG_PWR_MGMT_1,
			    MPU6050_PWR_CLK_PLL_XGYRO);
	if (ret)
		return dev_err_probe(dev, ret, "clock select failed\n");

	msleep(10);

	ret = regmap_write(mpu->regmap, MPU6050_REG_CONFIG,
			    MPU6050_DLPF_42HZ);
	if (ret)
		return dev_err_probe(dev, ret, "DLPF config failed\n");

	ret = regmap_write(mpu->regmap, MPU6050_REG_SMPLRT_DIV,
			    MPU6050_SMPLRT_DIV_100HZ);
	if (ret)
		return dev_err_probe(dev, ret, "sample rate config failed\n");

	ret = regmap_write(mpu->regmap, MPU6050_REG_GYRO_CONFIG,
			    MPU6050_GYRO_FS_250);
	if (ret)
		return dev_err_probe(dev, ret, "gyro config failed\n");

	ret = regmap_write(mpu->regmap, MPU6050_REG_ACCEL_CONFIG,
			    MPU6050_ACCEL_FS_2G);
	if (ret)
		return dev_err_probe(dev, ret, "accel config failed\n");

	ret = regmap_write(mpu->regmap, MPU6050_REG_INT_PIN_CFG,
			    MPU6050_INT_RD_CLEAR);
	if (ret)
		return dev_err_probe(dev, ret, "INT pin config failed\n");

	ret = regmap_write(mpu->regmap, MPU6050_REG_INT_ENABLE,
			    MPU6050_INT_DATA_RDY_EN);
	if (ret)
		return dev_err_probe(dev, ret, "INT enable failed\n");

	dev_info(dev, "MPU6050 initialized: 100Hz, ±250dps, ±2g\n");
	return 0;
}

// this runs every time the DATA_RDY interrupt fires. reads the
// sample bytes, clears the interrupt, fills in a record and sends
// it off to isfdas
static irqreturn_t mpu6050_irq_thread(int irq, void *dev_id)
{
	struct mpu6050_dev *mpu = dev_id;
	struct device *dev = &mpu->client->dev;
	u8 buf[MPU6050_SAMPLE_LEN];
	unsigned int status;
    struct sensor_record rec = { };
	int ret;

	/* reading INT_STATUS clears it since INT_RD_CLEAR was set at init */
	ret = regmap_read(mpu->regmap, MPU6050_REG_INT_STATUS, &status);
	if (ret) {
		dev_err_ratelimited(dev, "INT_STATUS read failed: %d\n", ret);
		return IRQ_HANDLED;
	}

	ret = regmap_bulk_read(mpu->regmap, MPU6050_REG_ACCEL_XOUT_H,
				buf, MPU6050_SAMPLE_LEN);
	if (ret) {
		dev_err_ratelimited(dev, "sample read failed: %d\n", ret);
		return IRQ_HANDLED;
	}

	rec.timestamp_ns = ktime_get_ns();

	rec.accel_x = (s16)((buf[0]  << 8) | buf[1]);
	rec.accel_y = (s16)((buf[2]  << 8) | buf[3]);
	rec.accel_z = (s16)((buf[4]  << 8) | buf[5]);
	rec.temp_raw = (s16)((buf[6] << 8) | buf[7]);
	rec.gyro_x  = (s16)((buf[8]  << 8) | buf[9]);
	rec.gyro_y  = (s16)((buf[10] << 8) | buf[11]);
	rec.gyro_z  = (s16)((buf[12] << 8) | buf[13]);

	isfdas_submit_record(&rec);

	return IRQ_HANDLED;
}

static int mpu6050_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mpu6050_dev *mpu;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -ENODEV,
				     "adapter lacks I2C_FUNC_I2C\n");

	mpu = devm_kzalloc(dev, sizeof(*mpu), GFP_KERNEL);
	if (!mpu)
		return -ENOMEM;

	mpu->client = client;
	i2c_set_clientdata(client, mpu);

	mpu->regmap = devm_regmap_init_i2c(client, &mpu6050_regmap_config);
	if (IS_ERR(mpu->regmap))
		return dev_err_probe(dev, PTR_ERR(mpu->regmap),
				     "regmap init failed\n");

	ret = mpu6050_chip_id_check(mpu);
	if (ret)
		return ret;

	ret = mpu6050_hw_init(mpu);
	if (ret)
		return ret;

	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "no valid IRQ\n");
	mpu->irq = client->irq;

	// top half is NULL, kernel gives us a default handler that just
	// wakes up our thread since we need i2c (sleeping) access
	ret = devm_request_threaded_irq(dev, mpu->irq, NULL,
					 mpu6050_irq_thread,
					 IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					 "mpu6050", mpu);
	if (ret)
		return dev_err_probe(dev, ret, "IRQ request failed\n");

	dev_info(dev, "MPU6050 acquisition driver ready (IRQ %d)\n",
		 mpu->irq);
	return 0;
}

static void mpu6050_remove(struct i2c_client *client)
{
	struct mpu6050_dev *mpu = i2c_get_clientdata(client);

	// turn off interrupts before putting the chip to sleep
	regmap_write(mpu->regmap, MPU6050_REG_INT_ENABLE, 0);
	regmap_write(mpu->regmap, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_SLEEP);
}

static const struct of_device_id mpu6050_of_match[] = {
	{ .compatible = "invensense,mpu6050" },
	{ }
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

static const struct i2c_device_id mpu6050_id[] = {
	{ "mpu6050", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mpu6050_id);

static struct i2c_driver mpu6050_driver = {
	.driver = {
		.name = "mpu6050",
		.of_match_table = mpu6050_of_match,
	},
	.probe = mpu6050_probe,
	.remove = mpu6050_remove,
	.id_table = mpu6050_id,
};
module_i2c_driver(mpu6050_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Manjunatha V");
MODULE_DESCRIPTION("MPU6050 acquisition driver for ISFDAS");
MODULE_VERSION("1.0");