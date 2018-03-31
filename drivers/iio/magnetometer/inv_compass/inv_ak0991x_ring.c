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

#include "iio.h"
#include "kfifo_buf.h"
#include "trigger_consumer.h"
#include "sysfs.h"

#include "inv_ak0991x_iio.h"

static int put_scan_to_buf(struct iio_dev *indio_dev, unsigned char *d,
				short *s, int scan_index)
{
	struct iio_buffer *ring = indio_dev->buffer;
	int st;
	int i, d_ind;

	d_ind = 0;
	for (i = 0; i < 3; i++) {
		st = iio_scan_mask_query(indio_dev, ring, scan_index + i);
		if (st) {
			memcpy(&d[d_ind], &s[i], sizeof(s[i]));
			d_ind += sizeof(s[i]);
		}
	}

	return d_ind;
}

/**
 *  inv_read_ak0991x_fifo() - Transfer data from FIFO to ring buffer.
 */
void inv_read_ak0991x_fifo(struct iio_dev *indio_dev)
{
	struct inv_ak0991x_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	int d_ind, i;
	s8 *tmp;
	s64 tmp_buf[2];

	if (!ak0991x_read(st, st->compass_data)) {
        if (st->compass_id == COMPASS_ID_AK09911) {
    		for (i = 0; i < 3; i++) {
    			st->compass_data[i] = (short)(((int)st->compass_data[i]
    						* (st->asa[i] + 128)) >> 7);
		    }
        } else if (st->compass_id == COMPASS_ID_AK09916){
            //ak09916 needs no sensitivity adjustment
        }
		tmp = (u8 *)tmp_buf;
		d_ind = put_scan_to_buf(indio_dev, tmp, st->compass_data,
						INV_AK0991x_SCAN_MAGN_X);
		if (ring->scan_timestamp)
			tmp_buf[(d_ind + 7)/8] = st->timestamp;
		//ring->access->store_to(indio_dev->buffer, tmp, st->timestamp);
		ring->access->store_to(indio_dev->buffer, tmp);
//printk("ak0991x, %lld\n", st->timestamp);
	}
    else {
        printk("%s: ak0991x, error\n", __func__);
    }
}

void inv_ak0991x_unconfigure_ring(struct iio_dev *indio_dev)
{
	iio_kfifo_free(indio_dev->buffer);
};

static int inv_ak0991x_postenable(struct iio_dev *indio_dev)
{
	struct inv_ak0991x_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;

	/* when all the outputs are disabled, even though buffer/enable is on,
	   do nothing */
	if (!(iio_scan_mask_query(indio_dev, ring, INV_AK0991x_SCAN_MAGN_X) ||
		iio_scan_mask_query(indio_dev, ring, INV_AK0991x_SCAN_MAGN_Y) ||
		iio_scan_mask_query(indio_dev, ring, INV_AK0991x_SCAN_MAGN_Z)))
		return 0;

	set_ak0991x_enable(indio_dev, true);
	schedule_delayed_work(&st->work, msecs_to_jiffies(st->delay));

	return 0;
}

static int inv_ak0991x_predisable(struct iio_dev *indio_dev)
{
	struct iio_buffer *ring = indio_dev->buffer;
	struct inv_ak0991x_state_s *st = iio_priv(indio_dev);

	cancel_delayed_work_sync(&st->work);
	clear_bit(INV_AK0991x_SCAN_MAGN_X, ring->scan_mask);
	clear_bit(INV_AK0991x_SCAN_MAGN_Y, ring->scan_mask);
	clear_bit(INV_AK0991x_SCAN_MAGN_Z, ring->scan_mask);
	set_ak0991x_enable(indio_dev, false);

	return 0;
}

static const struct iio_buffer_setup_ops inv_ak0991x_ring_setup_ops = {
	.postenable = &inv_ak0991x_postenable,
	.predisable = &inv_ak0991x_predisable,
};

int inv_ak0991x_configure_ring(struct iio_dev *indio_dev)
{
	int ret = 0;
	struct iio_buffer *ring;

	ring = iio_kfifo_allocate(indio_dev);
	if (!ring) {
		ret = -ENOMEM;
		return ret;
	}
	indio_dev->buffer = ring;
	/* setup ring buffer */
	ring->scan_timestamp = true;
	indio_dev->setup_ops = &inv_ak0991x_ring_setup_ops;

	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	return 0;
}

