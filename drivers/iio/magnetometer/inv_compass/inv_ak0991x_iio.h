/*
* Copyright (C) 2012 Invensense, Inc.
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
#ifndef _INV_AK0991x_IIO_H_
#define _INV_AK0991x_IIO_H_

#include <linux/i2c.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/spinlock.h>
#include <linux/mpu.h>

#include "iio.h"
#include "buffer.h"
#include "trigger.h"

/**
 *  struct inv_ak09911_state_s - Driver state variables.
 *  @plat_data:     board file platform data.
 *  @i2c:           i2c client handle.
 *  @trig:          not used. for compatibility.
 *  @work:          work data structure.
 *  @delay:         delay between each scheduled work.
 *  @compass_id:    id of compass.
 *  @compass_data:  compass data.
 *  @asa:           fuse data to adjust final data.
 *  @timestamp:     timestamp of each data.
 *  @i2c_addr:      i2c address
 *  @sl_handle:		Handle to I2C port.
 */
struct inv_ak0991x_state_s {
	struct mpu_platform_data plat_data;
	struct i2c_client *i2c;
	struct iio_trigger  *trig;
	struct delayed_work work;
	int delay;                 /* msec */
	bool enable;
	struct mutex enable_lock;
	unsigned char compass_id;
	short compass_data[3];
	u8 asa[3];	           /* axis sensitivity adjustment */
	s64 timestamp;
	short i2c_addr;
	void *sl_handle;
	int status;
};

/* scan element definition */
enum inv_mpu_scan {
	INV_AK0991x_SCAN_MAGN_X,
	INV_AK0991x_SCAN_MAGN_Y,
	INV_AK0991x_SCAN_MAGN_Z,
	INV_AK0991x_SCAN_TIMESTAMP,
};

/*! \name ak09911 constant definition
 \anchor ak09911_Def
 Constant definitions of the ak09911.*/
#define AK0991x_MEASUREMENT_TIME_US	10000

/*! \name ak09911 operation mode
 \anchor AK09911_Mode
 Defines an operation mode of the ak09911.*/
/*! @{*/
#define	AK0991x_CNTL_MODE_POWER_DOWN     0x00
#define AK0991x_CNTL_MODE_SNG_MEASURE    0x01
#define	AK0991x_CNTL_MODE_SELF_TEST      0x10

#define	AK09911_CNTL_MODE_FUSE_ACCESS    0x1F
/*! @}*/

/*! \name ak09911 register address
\anchor AK09911_REG
Defines a register address of the ak09911.*/
/*! @{*/
#define AK0991x_REG_WIA		0x00
#define AK0991x_REG_INFO	0x01
#define AK0991x_REG_ST1		0x10
#define AK0991x_REG_HXL		0x11
#define AK0991x_REG_ST2		0x18
#define AK0991x_REG_CNTL	0x31
/*! @}*/

/*! \name ak09911 fuse-rom address
\anchor AK09911_FUSE
Defines a read-only address of the fuse ROM of the ak09911.*/
/*! @{*/
#define AK09911_FUSE_ASAX	0x60
/*! @}*/

#define AK0991x_MAX_DELAY        (200)
#define AK0991x_MIN_DELAY        (10)
#define AK0991x_DEFAULT_DELAY    (100)

#define INV_ERROR_COMPASS_DATA_OVERFLOW  (-1)
#define INV_ERROR_COMPASS_DATA_NOT_READY (-2)

#define DATA_AK09911_SCALE       (19661 * (1L << 15))
#define DATA_AK09916_SCALE       (4915 * (1L << 15))

int inv_ak0991x_configure_ring(struct iio_dev *indio_dev);
void inv_ak0991x_unconfigure_ring(struct iio_dev *indio_dev);
int inv_ak0991x_probe_trigger(struct iio_dev *indio_dev);
void inv_ak0991x_remove_trigger(struct iio_dev *indio_dev);
void set_ak0991x_enable(struct iio_dev *indio_dev, bool enable);
int ak0991x_read_raw_data(struct inv_ak0991x_state_s *st,
				short dat[3]);
void inv_read_ak0991x_fifo(struct iio_dev *indio_dev);
int ak0991x_read(struct inv_ak0991x_state_s *st, short rawfixed[3]);

#endif

