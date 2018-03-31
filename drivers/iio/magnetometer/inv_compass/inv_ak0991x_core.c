/*
* Copyright (C) 2013 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include "inv_ak0991x_iio.h"
#ifdef INV_KERNEL_3_10
#include <linux/iio/sysfs.h>
#else
#include "sysfs.h"
#endif
#include "inv_test/inv_counters.h"
#ifdef CONFIG_DTS_INV_MPU_IIO
#include "inv_mpu_dts.h"
#endif

#include <linux/ktime.h>
extern ktime_t alarm_get_elapsed_realtime(void);

static s64 get_time_ns(void)
{
    struct timespec ts;

	get_monotonic_boottime(&ts);
    //ts = ktime_to_timespec(alarm_get_elapsed_realtime());
    //uktime_get_ts(&ts);
    //get_monotonic_boottime(&ts);

    return timespec_to_ns(&ts);
}

/**
 *  inv_serial_read() - Read one or more bytes from the device registers.
 *  @st:     Device driver instance.
 *  @reg:    First device register to be read from.
 *  @length: Number of bytes to read.
 *  @data:   Data read from device.
 *  NOTE:    The slave register will not increment when reading from the FIFO.
 */
int inv_serial_read(struct inv_ak0991x_state_s *st, u8 reg, u16 length, u8 *data)
{
	int result;
	INV_I2C_INC_COMPASSWRITE(3);
	INV_I2C_INC_COMPASSREAD(length);
	result = i2c_smbus_read_i2c_block_data(st->i2c, reg, length, data);
	if (result != length) {
		if (result < 0)
			return result;
		else
			return -EINVAL;
	} else {
		return 0;
	}
}

/**
 *  inv_serial_single_write() - Write a byte to a device register.
 *  @st:	Device driver instance.
 *  @reg:	Device register to be written to.
 *  @data:	Byte to write to device.
 */
int inv_serial_single_write(struct inv_ak0991x_state_s *st, u8 reg, u8 data)
{
	u8 d[1];
	d[0] = data;
	INV_I2C_INC_COMPASSWRITE(3);

	return i2c_smbus_write_i2c_block_data(st->i2c, reg, 1, d);
}

static int ak0991x_init(struct inv_ak0991x_state_s *st)
{
	int result = 0;
	unsigned char serial_data[3];

	result = inv_serial_single_write(st, AK0991x_REG_CNTL,
					 AK0991x_CNTL_MODE_POWER_DOWN);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
		return result;
	}
	/* Wait at least 100us */
	udelay(100);

    if (st->compass_id == COMPASS_ID_AK09911)
    {
    	result = inv_serial_single_write(st, AK0991x_REG_CNTL,
    					 AK09911_CNTL_MODE_FUSE_ACCESS);
    	if (result) {
    		pr_err("%s, line=%d\n", __func__, __LINE__);
    		return result;
    	}

    	/* Wait at least 200us */
    	udelay(200);

    	result = inv_serial_read(st, AK09911_FUSE_ASAX, 3, serial_data);
    	if (result) {
    		pr_err("%s, line=%d\n", __func__, __LINE__);
    		return result;
    	}

    	st->asa[0] = serial_data[0];
    	st->asa[1] = serial_data[1];
    	st->asa[2] = serial_data[2];

    	result = inv_serial_single_write(st, AK0991x_REG_CNTL,
    					 AK0991x_CNTL_MODE_POWER_DOWN);
    	if (result) {
    		pr_err("%s, line=%d\n", __func__, __LINE__);
    		return result;
    	}
    	udelay(100);
    }
    else if (st->compass_id == COMPASS_ID_AK09916)
    {
        //ak09916 has no Fuse ROM
    	st->asa[0] = 0;
    	st->asa[1] = 0;
    	st->asa[2] = 0;
    }
    else 
    {
    }
    
	return result;
}

int ak0991x_read(struct inv_ak0991x_state_s *st, short rawfixed[3])
{
	unsigned char regs[9];
	unsigned char *stat = &regs[0];
	unsigned char *stat2 = &regs[8];
	int result = 0;
	int status = 0;

	result = inv_serial_read(st, AK0991x_REG_ST1, 9, regs);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
	return result;
	}

	rawfixed[0] = (s16)__le16_to_cpup((__le16 *)&regs[1]);
	rawfixed[1] = (s16)__le16_to_cpup((__le16 *)&regs[3]);
	rawfixed[2] = (s16)__le16_to_cpup((__le16 *)&regs[5]);

	/*
	 * ST : data ready -
	 * Measurement has been completed and data is ready to be read.
	 */
	if (*stat & 0x01)
		status = 0;
	/*
	 * ST2 : overflow -
	 * the sum of the absolute values of all axis |X|+|Y|+|Z| < 2400uT.
	 * This is likely to happen in presence of an external magnetic
	 * disturbance; it indicates, the sensor data is incorrect and should
	 * be ignored.
	 * An error is returned.
	 * HOFL bit clears when a new measurement starts.
	 */
	if (*stat2 & 0x08)
		status = 0x08;
	/*
	 * ST : overrun -
	 * the previous sample was not fetched and lost.
	 * Valid in continuous measurement mode only.
	 * In single measurement mode this error should not occour and we
	 * don't consider this condition an error.
	 * DOR bit is self-clearing when ST2 or any meas. data register is
	 * read.
	 */
	if (*stat & 0x02) {
		/* status = INV_ERROR_COMPASS_DATA_UNDERFLOW; */
		status = 0;
	}

	/*
	 * trigger next measurement if:
	 *	- stat is non zero;
	 *	- if stat is zero and stat2 is non zero.
	 * Won't trigger if data is not ready and there was no error.
	 */
	result = inv_serial_single_write(st, AK0991x_REG_CNTL,
				AK0991x_CNTL_MODE_SNG_MEASURE);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
		return result;
	}

	if (status)
		pr_err("%s, line=%d, status=%d\n", __func__, __LINE__, status);

#if 0
	return status;
#else
    return 0;
#endif
}

/**
 *  ak0991x_read_raw() - read raw method.
 */
static int ak0991x_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask) {
	struct inv_ak0991x_state_s  *st = iio_priv(indio_dev);
	int scale = 0;

	switch (mask) {
	case 0:
		if (!(iio_buffer_enabled(indio_dev)))
			return -EINVAL;
		if (chan->type == IIO_MAGN) {
			*val = st->compass_data[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}

		return -EINVAL;
	case IIO_CHAN_INFO_SCALE:
        if (st->compass_id == COMPASS_ID_AK09911)
            scale = DATA_AK09911_SCALE;
        else if (st->compass_id == COMPASS_ID_AK09916)
            scale = DATA_AK09916_SCALE;
		*val = scale;
			return IIO_VAL_INT;
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

/*
 * ak0991x_self_test() - self_test w/ chip's self_test mode
 **/
static int ak0991x_self_test(struct inv_ak0991x_state_s *st)
{
	unsigned char status = 0;
	__le16 val[3];
	s16 x, y, z;
	int const max_retry = 50; /* retry 500 times within 500ms */
	int result, retry = max_retry;

	result = inv_serial_single_write(st, AK0991x_REG_CNTL,
			AK0991x_CNTL_MODE_POWER_DOWN);
	mdelay(2);
	result = inv_serial_single_write(st, AK0991x_REG_CNTL,
			AK0991x_CNTL_MODE_SELF_TEST);
	while(!result && retry-- &&
			!(status & 0x01 /* 0x1 for DRDY */)) {
		mdelay(1);
		result = inv_serial_read(st, AK0991x_REG_ST1,
				sizeof(status), &status);
	}
	if (result) {
		pr_err("%s: read/write register fail", __func__);
		return result;
	} else if (retry == 0) {
		pr_err("%s: data not ready", __func__);
		return -EAGAIN;
	}

	result = inv_serial_read(st, AK0991x_REG_HXL, sizeof(val), (u8 *)val);
	if (result) {
		pr_err("%s: read/write register fail", __func__);
		return result;
	}
	x = (s16)__le16_to_cpu(val[0]);
	y = (s16)__le16_to_cpu(val[1]);
	z = (s16)__le16_to_cpu(val[2]);
	if (x < -200 || 200 < x ||
		y < -200 || 200 < y ||
		z < -1000 || -200 < z) {
		pr_err("%s: response (%d, %d, %d) out of range\n",
				__func__, x, y, z);
		return 0; /* fail */
	} else {
		pr_info("%s: response ok (%d, %d, %d)\n", __func__, x, y, z);
	}
	return 1; /* ok */
}

static ssize_t ak0991x_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_ak0991x_state_s *st = iio_priv(indio_dev);
	short c[3];

	mutex_lock(&indio_dev->mlock);
	c[0] = st->compass_data[0];
	c[1] = st->compass_data[1];
	c[2] = st->compass_data[2];
	mutex_unlock(&indio_dev->mlock);
	return sprintf(buf, "%d, %d, %d\n", c[0], c[1], c[2]);
}

static ssize_t ak0991x_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_ak0991x_state_s *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->status);
}

static ssize_t ak0991x_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_ak0991x_state_s *st = iio_priv(indio_dev);
	/* transform delay in ms to rate */
	return sprintf(buf, "%d\n", (1000 / st->delay));
}

static ssize_t ak0991x_self_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_ak0991x_state_s *st = iio_priv(indio_dev);
	ssize_t ret = -EBUSY;
	/* transform delay in ms to rate */

	mutex_lock(&st->enable_lock);
	if (!st->enable) {
		ret = ak0991x_self_test(st);
		if (ret >= 0) /* no error */
			ret = sprintf(buf, "%zd\n", ret);
	}
	mutex_unlock(&st->enable_lock);
	return ret;
}

/**
 * ak0991x_matrix_show() - show orientation matrix
 */
static ssize_t ak0991x_matrix_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	signed char *m;
	struct inv_ak0991x_state_s *st = iio_priv(indio_dev);
	m = st->plat_data.orientation;
	return sprintf(buf,
		"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}

void set_ak0991x_enable(struct iio_dev *indio_dev, bool enable)
{
	struct inv_ak0991x_state_s *st = iio_priv(indio_dev);
	int result = 0;

	mutex_lock(&st->enable_lock);
	if (enable != st->enable) {
		if (enable) {
			result = inv_serial_single_write(st, AK0991x_REG_CNTL,
						AK0991x_CNTL_MODE_SNG_MEASURE);
			if (result)
				pr_err("%s, line=%d\n", __func__, __LINE__);
			schedule_delayed_work(&st->work,
				msecs_to_jiffies(st->delay));
		} else {
			cancel_delayed_work_sync(&st->work);
			result = inv_serial_single_write(st, AK0991x_REG_CNTL,
						AK0991x_CNTL_MODE_POWER_DOWN);
			if (result)
				pr_err("%s, line=%d\n", __func__, __LINE__);
			mdelay(1);	/* wait at least 100us */
		}
		st->enable = enable;
	}
	mutex_unlock(&st->enable_lock);
}

static ssize_t ak0991x_rate_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_ak0991x_state_s *st = iio_priv(indio_dev);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	/* transform rate to delay in ms */
	data = 1000 / data;
	if (data > AK0991x_MAX_DELAY)
		data = AK0991x_MAX_DELAY;
	if (data < AK0991x_MIN_DELAY)
		data = AK0991x_MIN_DELAY;
	st->delay = (unsigned int) data;
	return count;
}

static void ak0991x_work_func(struct work_struct *work)
{
	struct inv_ak0991x_state_s *st =
		container_of((struct delayed_work *)work,
			struct inv_ak0991x_state_s, work);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	unsigned long delay = msecs_to_jiffies(st->delay);

//	mutex_lock(&indio_dev->mlock);
//	if (!(iio_buffer_enabled(indio_dev)))
//		goto error_ret;

	st->timestamp = get_time_ns();
	schedule_delayed_work(&st->work, delay);
	inv_read_ak0991x_fifo(indio_dev);
	INV_I2C_INC_COMPASSIRQ();
//printk("ak09911 work, %d, %ld, %lld\n", st->delay, delay, st->timestamp);

//error_ret:
//	mutex_unlock(&indio_dev->mlock);
}

static const struct iio_chan_spec compass_channels[] = {
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		//.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = INV_AK0991x_SCAN_MAGN_X,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0
		}
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		//.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = INV_AK0991x_SCAN_MAGN_Y,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0
		}
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		//.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = INV_AK0991x_SCAN_MAGN_Z,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0
		}
	},
	IIO_CHAN_SOFT_TIMESTAMP(INV_AK0991x_SCAN_TIMESTAMP)
};

static DEVICE_ATTR(value, S_IRUGO, ak0991x_value_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, ak0991x_status_show, NULL);
static DEVICE_ATTR(self_test, S_IRUGO, ak0991x_self_test_show, NULL);
static DEVICE_ATTR(sampling_frequency, S_IRUGO | S_IWUSR | S_IWGRP,
		ak0991x_rate_show, ak0991x_rate_store);
static DEVICE_ATTR(compass_matrix, S_IRUGO, ak0991x_matrix_show, NULL);

static struct attribute *inv_ak0991x_attributes[] = {
	&dev_attr_value.attr,
	&dev_attr_status.attr,
	&dev_attr_self_test.attr,
	&dev_attr_sampling_frequency.attr,
	&dev_attr_compass_matrix.attr,
	NULL,
};

static const struct attribute_group inv_attribute_group = {
	.name = "ak0991x",
	.attrs = inv_ak0991x_attributes
};

static const struct iio_info ak0991x_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &ak0991x_read_raw,
	.attrs = &inv_attribute_group,
};

/*constant IIO attribute */
/**
 *  inv_ak0991x_probe() - probe function.
 */
static int inv_ak0991x_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct inv_ak0991x_state_s *st;
	struct iio_dev *indio_dev;
	int result;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}
#ifdef INV_KERNEL_3_10
	indio_dev = iio_device_alloc(sizeof(*st));
#else
	indio_dev = iio_allocate_device(sizeof(*st));
#endif
	if (indio_dev == NULL) {
		result =  -ENOMEM;
		goto out_no_free;
	}
	st = iio_priv(indio_dev);
	st->i2c = client;
	st->sl_handle = client->adapter;
	mutex_init(&st->enable_lock);

#ifdef CONFIG_DTS_INV_MPU_IIO
	result = inv_parse_orientation_matrix(&client->dev,
			st->plat_data.orientation);
	dev_dbg(&client->dev,
		"%s orientation [%d %d %d] [%d %d %d] [%d %d %d]\n", __func__,
		st->plat_data.orientation[0], st->plat_data.orientation[1],
		st->plat_data.orientation[2], st->plat_data.orientation[3],
		st->plat_data.orientation[4], st->plat_data.orientation[5],
		st->plat_data.orientation[6], st->plat_data.orientation[7],
		st->plat_data.orientation[8]);
#else
	st->plat_data =
		*(struct mpu_platform_data *)dev_get_platdata(&client->dev);
#endif
	st->i2c_addr = client->addr;
	st->delay = AK0991x_DEFAULT_DELAY;
	st->compass_id = id->driver_data;

	i2c_set_clientdata(client, indio_dev);
	result = ak0991x_init(st);
	if (result)
		goto out_free;

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = id->name;
	indio_dev->channels = compass_channels;
	indio_dev->num_channels = ARRAY_SIZE(compass_channels);
	indio_dev->info = &ak0991x_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;

	result = inv_ak0991x_configure_ring(indio_dev);
	if (result)
		goto out_free;
	result = iio_buffer_register(indio_dev, indio_dev->channels,
					indio_dev->num_channels);
	if (result)
		goto out_unreg_ring;
	result = inv_ak0991x_probe_trigger(indio_dev);
	if (result)
		goto out_remove_ring;

	result = iio_device_register(indio_dev);
	if (result)
		goto out_remove_trigger;
	INIT_DELAYED_WORK(&st->work, ak0991x_work_func);
	st->status = 1;
	pr_info("%s: Probe name %s\n", __func__, id->name);
	printk("%s, probe ok\n", __func__);
	return 0;
out_remove_trigger:
	if (indio_dev->modes & INDIO_BUFFER_TRIGGERED)
		inv_ak0991x_remove_trigger(indio_dev);
out_remove_ring:
	iio_buffer_unregister(indio_dev);
out_unreg_ring:
	inv_ak0991x_unconfigure_ring(indio_dev);
out_free:
	mutex_destroy(&st->enable_lock);
#ifdef INV_KERNEL_3_10
	iio_device_free(indio_dev);
#else
	iio_free_device(indio_dev);
#endif
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return -EIO;
}

/**
 *  inv_ak0991x_remove() - remove function.
 */
static int inv_ak0991x_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct inv_ak0991x_state_s *st = iio_priv(indio_dev);
	cancel_delayed_work_sync(&st->work);
	iio_device_unregister(indio_dev);
	inv_ak0991x_remove_trigger(indio_dev);
	iio_buffer_unregister(indio_dev);
	inv_ak0991x_unconfigure_ring(indio_dev);
	mutex_destroy(&st->enable_lock);
#ifdef INV_KERNEL_3_10
	iio_device_free(indio_dev);
#else
	iio_free_device(indio_dev);
#endif
	dev_info(&client->adapter->dev, "inv-ak0991x-iio module removed.\n");
	return 0;
}

static const unsigned short normal_i2c[] = { I2C_CLIENT_END };

/* device id table is used to identify what device can be
 * supported by this driver
 */
static const struct i2c_device_id inv_ak0991x_id[] = {
	{"ak09911", COMPASS_ID_AK09911},
    {"ak09916", COMPASS_ID_AK09916},
	{}
};

MODULE_DEVICE_TABLE(i2c, inv_ak0991x_id);

static struct i2c_driver inv_ak0991x_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		=	inv_ak0991x_probe,
	.remove		=	inv_ak0991x_remove,
	.id_table	=	inv_ak0991x_id,
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	"inv-ak0991x-iio",
	},
	.address_list = normal_i2c,
};

static int __init inv_ak0991x_init(void)
{
	int result = i2c_add_driver(&inv_ak0991x_driver);
	if (result) {
		pr_err("%s failed\n", __func__);
		return result;
	}
	return 0;
}

static void __exit inv_ak0991x_exit(void)
{
	i2c_del_driver(&inv_ak0991x_driver);
}

module_init(inv_ak0991x_init);
module_exit(inv_ak0991x_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("inv-ak0991x-iio");

