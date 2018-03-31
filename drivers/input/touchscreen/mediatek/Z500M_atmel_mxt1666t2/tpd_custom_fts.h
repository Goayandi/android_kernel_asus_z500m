/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_ctl.c
*
* Author:
*
* Created: 2015-01-01
*
* Abstract: declare for IC info, Read/Write, reset
*
************************************************************************/
#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/jiffies.h>
#include <linux/version.h>

/* IC info */
/* Pre-defined definition */
#define TINNO_TOUCH_TRACK_IDS		10
#define CFG_MAX_TOUCH_POINTS		10
#define TPD_RES_X			1536
#define TPD_RES_Y			2048
#define IICReadWriteRetryTime		3

#endif /* TOUCHPANEL_H__ */
