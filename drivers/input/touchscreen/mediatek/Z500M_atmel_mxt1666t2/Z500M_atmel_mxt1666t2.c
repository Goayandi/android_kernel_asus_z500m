/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2014 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/platform_data/atmel_mxt_ts.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <../../../../../include/uapi/linux/input.h>
#include "tpd.h"
#include "tpd_custom_fts.h"

#include "../../../misc/mediatek/include/mt-plat/mt_boot_common.h"

/* Configuration file */
#define MXT_CFG_MAGIC		"OBP_RAW V1"

/* Registers */
#define MXT_OBJECT_START	0x07
#define MXT_OBJECT_SIZE		6
#define MXT_INFO_CHECKSUM_SIZE	3
#define MXT_MAX_BLOCK_WRITE	255

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_GEN_DATASOURCE_T53		53
#define MXT_TOUCH_MULTI_T9		9
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_TOUCH_PROXKEY_T52		52
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_GRIP_T40		40
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_CTECONFIG_T28		28
#define MXT_SPT_USERDATA_T38		38
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46
#define MXT_PROCI_ACTIVE_STYLUS_T63	63
#define MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71 71
#define MXT_PROCI_GLOVEDETECTION_T78	78
#define MXT_PROCI_SYMBOLGESTUREPROCESSOR_T92	92
#define MXT_PROCI_TOUCHSEQUENCELOGGER_T93	93
//#define MXT_PROCI_SYMBOLGESTUREPROCESSOR	92
//#define MXT_PROCI_TOUCHSEQUENCELOGGER	93
#define MXT_TOUCH_MULTITOUCHSCREEN_T100 100
#define MXT_SPT_AUXTOUCHCONFIG_T104	104
#define MXT_PROCI_ACTIVESTYLUS_T107	107

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	BIT(7)
#define MXT_T6_STATUS_OFL	BIT(6)
#define MXT_T6_STATUS_SIGERR	BIT(5)
#define MXT_T6_STATUS_CAL	BIT(4)
#define MXT_T6_STATUS_CFGERR	BIT(3)
#define MXT_T6_STATUS_COMSERR	BIT(2)

/* MXT_GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

#define MXT_POWER_CFG_RUN		0
#define MXT_POWER_CFG_DEEPSLEEP		1
#define MXT_POWER_CFG_IDLE		2

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_CTRL		0
#define MXT_T9_ORIENT		9
#define MXT_T9_RANGE		18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		BIT(0)
#define MXT_T9_SUPPRESS		BIT(1)
#define MXT_T9_AMP		BIT(2)
#define MXT_T9_VECTOR		BIT(3)
#define MXT_T9_MOVE		BIT(4)
#define MXT_T9_RELEASE		BIT(5)
#define MXT_T9_PRESS		BIT(6)
#define MXT_T9_DETECT		BIT(7)

struct t9_range {
	u16 x;
	u16 y;
} __packed;

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH	BIT(0)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1
#define MXT_COMMS_RETRIGEN      BIT(6)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_RESET_VALUE		0x01
#define MXT_BACKUP_VALUE	0x55

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	BIT(0)

/* T63 Stylus */
#define MXT_T63_STYLUS_PRESS	(1 << 0)
#define MXT_T63_STYLUS_RELEASE	(1 << 1)
#define MXT_T63_STYLUS_MOVE		(1 << 2)
#define MXT_T63_STYLUS_SUPPRESS	(1 << 3)

#define MXT_T63_STYLUS_DETECT	(1 << 4)
#define MXT_T63_STYLUS_TIP		(1 << 5)
#define MXT_T63_STYLUS_ERASER	(1 << 6)
#define MXT_T63_STYLUS_BARREL	(1 << 7)

#define MXT_T63_STYLUS_PRESSURE_MASK	0x3F

#define MXT_T15_CTRL		0

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL		0
#define MXT_T100_CFG1		1
#define MXT_T100_TCHAUX		3
#define MXT_T100_XRANGE		13
#define MXT_T100_YRANGE		24

#define MXT_T100_CFG_SWITCHXY	BIT(5)

#define MXT_T100_TCHAUX_VECT	BIT(0)
#define MXT_T100_TCHAUX_AMPL	BIT(1)
#define MXT_T100_TCHAUX_AREA	BIT(2)

#define MXT_T100_DETECT		BIT(7)
#define MXT_T100_TYPE_MASK	0x70

enum t100_type {
	MXT_T100_TYPE_FINGER		= 1,
	MXT_T100_TYPE_PASSIVE_STYLUS	= 2,
	MXT_T100_TYPE_ACTIVE_STYLUS	= 3,
	MXT_T100_TYPE_HOVERING_FINGER	= 4,
	MXT_T100_TYPE_GLOVE		= 5,
	MXT_T100_TYPE_LARGE_TOUCH	= 6,
};

#define MXT_DISTANCE_ACTIVE_TOUCH	0
#define MXT_DISTANCE_HOVERING		1

#define MXT_TOUCH_MAJOR_DEFAULT		1
#define MXT_PRESSURE_DEFAULT		1

/* Gen2 Active Stylus */
#define MXT_T107_CTRL			0
//#define MXT_T107_FRAME_SYNC		97
//#define MXT_T107_STYLUS_STYAUX		42
#define MXT_T107_STYLUS_STYAUX		37
#define MXT_T107_STYLUS_STYAUX_PRESSURE	(1 << 0)
#define MXT_T107_STYLUS_STYAUX_PEAK	(1 << 4)

#define MXT_T107_STYLUS_HOVER		(1 << 0)
#define MXT_T107_STYLUS_TIPSWITCH	(1 << 1)
#define MXT_T107_STYLUS_BUTTON0		(1 << 2)
#define MXT_T107_STYLUS_BUTTON1		(1 << 3)
#define MXT_T107_CTRL_ENABLE		(1 << 0)

/* Delay times */
#define MXT_BACKUP_TIME		50	/* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_TIMEOUT	3000	/* msec */
#define MXT_CRC_TIMEOUT		1000	/* msec */
#define MXT_FW_RESET_TIME	3000	/* msec */
#define MXT_FW_CHG_TIMEOUT	300	/* msec */
#define MXT_WAKEUP_TIME		25	/* msec */
#define MXT_REGULATOR_DELAY	150	/* msec */
#define MXT_CHG_DELAY	        100	/* msec */
#define MXT_POWERON_DELAY	150	/* msec */
#define MXT_BOOTLOADER_WAIT	36E5	/* 1 minute */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f
#define MXT_BOOT_EXTENDED_ID	BIT(5)
#define MXT_BOOT_ID_MASK	0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_PIXELS_PER_MM	20

#define DEBUG_MSG_MAX		200

#define PRJ_ID_ARIEL		'0'
#define PRJ_ID_Z500M		'1'

#define MXT_CONFIG_NAME_1666T2 "ASUS_Z500M_mXT1666T2_FW23AB_CFG34_0xA13BF3.raw"
#define MXT_FIRMWARE_NAME_1666T2 "ASUS_Z500M_mXT1666T2_v23AB.fw"

/* Gesture function report code */
#define MXT_T92_REPORT_CODE_W	119
#define MXT_T92_REPORT_CODE_S	115
#define MXT_T92_REPORT_CODE_E	101
#define MXT_T92_REPORT_CODE_C	67
#define MXT_T92_REPORT_CODE_Z	122
#define MXT_T92_REPORT_CODE_V	118
#define ASUS_GESTURE_COUNT	6
#define ASUS_GESTURE_MASK_W	5
#define ASUS_GESTURE_MASK_S	4
#define ASUS_GESTURE_MASK_E	3
#define ASUS_GESTURE_MASK_C	2
#define ASUS_GESTURE_MASK_Z	1
#define ASUS_GESTURE_MASK_V	0
#define APP_SWITCH		139

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

static struct workqueue_struct *maxtouch_wq;

/* Firmware frame structure */
struct mxt_fw_frame {
	__be16 size;
	u8 data[];
};

/* Firmware update context */
struct mxt_flash {
	struct mxt_data *data;
	const struct firmware *fw;
	struct mxt_fw_frame *frame;
	loff_t pos;
	size_t frame_size;
	unsigned int count;
	unsigned int retry;
	u8 previous;
	struct completion flash_completion;
	struct delayed_work work;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct switch_dev touch_sdev;
	char phys[64];		/* device physical location */
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info *info;
	void *raw_info_block;
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	bool in_bootloader;
	u16 mem_size;
	u8 t100_aux_ampl;
	u8 t100_aux_area;
	u8 t100_aux_vect;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	bool debug_v2_enabled;
	u8 *debug_msg_data;
	u16 debug_msg_count;
	struct bin_attribute debug_msg_attr;
	struct mutex debug_msg_lock;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
	u8 bootloader_addr;
	u8 *msg_buf;
	u8 t6_status;
	bool update_input;
	u8 last_message_count;
	u8 num_touchids;
	u8 multitouch;
	struct t7_config t7_cfg;
	u8 num_stylusids;
	unsigned long t15_keystatus;
	u8 stylus_aux_pressure;
	u8 stylus_aux_peak;
	bool use_retrigen_workaround;
	struct regulator *reg_vdd;
	struct regulator *reg_avdd;
	char *fw_name;
	char *cfg_name;
	struct mxt_flash *flash;

	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u16 T7_address;
	u16 T8_address;
	u16 T71_address;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T18_address;
	u8 T19_reportid;
	u8 T42_reportid_min;
	u8 T42_reportid_max;
	u16 T44_address;
	u8 T48_reportid;
	u8 T63_reportid_min;
	u8 T63_reportid_max;
	u16 T78_address;
	u16 T92_address;
	u8 T92_reportid;
	u8 T92_reportid_min;
	u8 T92_reportid_max;
	u16 T93_address;
	u8 T93_reportid;
	u8 T93_reportid_min;
	u8 T93_reportid_max;
	u16 T100_address;
	u8 T100_reportid_min;
	u8 T100_reportid_max;
	u16 T104_address;
	u16 T107_address;
	u16 T38_address;

	/* for reset handling */
	struct completion reset_completion;

	/* for config update handling */
	struct completion crc_completion;

	/* for power up handling */
	struct completion chg_completion;

	/* Indicates whether device is in suspend */
	bool suspended;

	/* Indicates whether device is updating configuration */
	bool updating_config;
	/* Indicates whether irq is disabled by sysfs*/
	bool sysfs_enable_irq;

	u8 t78_ctrl_enable;
	u8 t78_ctrl_disable;
	bool t78_master_ctrl;
	u8 t92_ctrl_enable;
	u8 t92_ctrl_disable;
	u8 t92_gesture_sel;
	u8 t93_ctrl_enable;
	u8 t93_ctrl_disable;
	bool t93_master_ctrl;
	/*gesture_wakeup master control ZenMotion*/
	bool gesture_wakeup;
	/*t100 default enabled*/
	u8 t100_ctrl;
	u8 t100_ctrl_disable;
	/*T8 cfg backup for byte 10 and 11*/
	u8 t8_backup_cfg;

	/*workaround for pen put in smart cover,
	  aim to saving pen power*/
	bool lid_closed;

	//++++++ add by angel ++++++
	/* for firmware and config file recovery and upgrade */
	struct work_struct work_upgrade;
	struct delayed_work report_delay_work;
	struct workqueue_struct *mxt_wq;
	struct wake_lock wake_lock;
	bool force_upgrade;
};

struct mxt_data *g_data_p;
static u8 global_fw_version;
static u8 global_fw_build;
static u8 global_cfg_version;
static u8 global_cfg_build;
static char pcbid_prj_id;

static bool global_pen_present;
static u8 global_pen_capacity;
static u8 global_pen_capacity_keep;
static bool global_zstylus_enable;
static bool global_zstylus_activation;
static struct mxt_data *global_mxt_data_pt;

static bool global_zstylus_only_mode;
static bool global_zstylus_only_pen_exist;
static int global_zstylus_only_mode_delay;

static int mxt_get_cfg_version(struct mxt_data *data);
static void mxt_hw_reset(struct device *dev);
static void mxt_force_updating(struct work_struct *work);

//static void mxt_t107_frame_sync_disable(struct mxt_data *data, bool suspend);
static void mxt_t8_pen_scan_disable(struct mxt_data *data, bool suspend);
static int  mxt_read_t8_backup_config(struct mxt_data *data);
static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep);
static void mxt_report_delay_work(struct work_struct *);

void mxt_lid_callback_T7_disable(bool lid_closed)
{
	if (lid_closed) {
		global_mxt_data_pt->lid_closed = true;
		if (global_mxt_data_pt->suspended) {
			pr_info("[ATMEL TOUCH] lid close, T7 DEEPSLEEP\n");
			mxt_set_t7_power_cfg(global_mxt_data_pt,
					MXT_POWER_CFG_DEEPSLEEP);
		}
	} else {
		global_mxt_data_pt->lid_closed = false;
		if (global_mxt_data_pt->suspended) {
			if (global_mxt_data_pt->gesture_wakeup) {
				pr_info("[ATMEL TOUCH] lid open, T7 IDLE\n");
				mxt_set_t7_power_cfg(global_mxt_data_pt,
							MXT_POWER_CFG_IDLE);
			}
		} else {
			pr_info("[ATMEL TOUCH] lid open, T7 RUN\n");
			mxt_set_t7_power_cfg(global_mxt_data_pt,
					MXT_POWER_CFG_RUN);
		}
	}
}
EXPORT_SYMBOL(mxt_lid_callback_T7_disable);

static enum power_supply_property pen_bat_properties[] = {
        POWER_SUPPLY_PROP_PRESENT,
        POWER_SUPPLY_PROP_CAPACITY_LEVEL,
        POWER_SUPPLY_PROP_PEN_ONLY,
};

static char *pen_bat_supplied_to[] = {
	"pen_bat",
};

static int pen_bat_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int ret=0;
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (global_pen_present)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (global_pen_capacity == 0)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		else if (global_pen_capacity == 1)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (global_pen_capacity == 2)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_MIDDLE;
		else if (global_pen_capacity == 3)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_PEN_ONLY:
		if (global_zstylus_only_pen_exist)
			val->strval = "1";
		else
			val->strval = "0";
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static struct power_supply pen_bat_supply[] = {
	{
		.name           = "pen_bat",
		.type           = POWER_SUPPLY_TYPE_PEN_BAT,
		.supplied_to    = pen_bat_supplied_to,
		.num_supplicants = ARRAY_SIZE(pen_bat_supplied_to),
		.properties     = pen_bat_properties,
		.num_properties = ARRAY_SIZE(pen_bat_properties),
		.get_property   = pen_bat_get_property,
	},
};

static size_t mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static size_t mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_GEN_DATASOURCE_T53:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	dev_dbg(&data->client->dev, "MXT MSG: %*ph\n",
		       data->T5_msg_size, message);
  
	if (data->debug_enabled)
		pr_info("[ATMEL TOUCH] MXT MSG: %*ph\n",
			data->T5_msg_size, message);
}

static void mxt_debug_msg_enable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (data->debug_v2_enabled)
		return;

	mutex_lock(&data->debug_msg_lock);

	data->debug_msg_data = kcalloc(DEBUG_MSG_MAX,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->debug_msg_data)
		return;

	data->debug_v2_enabled = true;
	mutex_unlock(&data->debug_msg_lock);

	dev_dbg(dev, "Enabled message output\n");
}

static void mxt_debug_msg_disable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (!data->debug_v2_enabled)
		return;

	dev_info(dev, "disabling message output\n");
	data->debug_v2_enabled = false;

	mutex_lock(&data->debug_msg_lock);
	kfree(data->debug_msg_data);
	data->debug_msg_data = NULL;
	data->debug_msg_count = 0;
	mutex_unlock(&data->debug_msg_lock);
	dev_dbg(dev, "Disabled message output\n");
}

static void mxt_debug_msg_add(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;

	mutex_lock(&data->debug_msg_lock);

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return;
	}

	if (data->debug_msg_count < DEBUG_MSG_MAX) {
		memcpy(data->debug_msg_data +
		       data->debug_msg_count * data->T5_msg_size,
		       msg,
		       data->T5_msg_size);
		data->debug_msg_count++;
	} else {
		dev_dbg(dev, "Discarding %u messages\n", data->debug_msg_count);
		data->debug_msg_count = 0;
	}

	mutex_unlock(&data->debug_msg_lock);

	sysfs_notify(&data->client->dev.kobj, NULL, "debug_notify");
}

static ssize_t mxt_debug_msg_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	return -EIO;
}

static ssize_t mxt_debug_msg_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t bytes)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int count;
	size_t bytes_read;

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return 0;
	}

	count = bytes / data->T5_msg_size;

	if (count > DEBUG_MSG_MAX)
		count = DEBUG_MSG_MAX;

	mutex_lock(&data->debug_msg_lock);

	if (count > data->debug_msg_count)
		count = data->debug_msg_count;

	bytes_read = count * data->T5_msg_size;

	memcpy(buf, data->debug_msg_data, bytes_read);
	data->debug_msg_count = 0;

	mutex_unlock(&data->debug_msg_lock);

	return bytes_read;
}

static int mxt_debug_msg_init(struct mxt_data *data)
{
	sysfs_bin_attr_init(&data->debug_msg_attr);
	data->debug_msg_attr.attr.name = "debug_msg";
	data->debug_msg_attr.attr.mode = 0666;
	data->debug_msg_attr.read = mxt_debug_msg_read;
	data->debug_msg_attr.write = mxt_debug_msg_write;
	data->debug_msg_attr.size = data->T5_msg_size * DEBUG_MSG_MAX;

	if (sysfs_create_bin_file(&data->client->dev.kobj,
				  &data->debug_msg_attr) < 0) {
		dev_err(&data->client->dev, "Failed to create %s\n",
			data->debug_msg_attr.attr.name);
		return -EINVAL;
	}

	return 0;
}

static void mxt_debug_msg_remove(struct mxt_data *data)
{
	if (data->debug_msg_attr.attr.name)
		sysfs_remove_bin_file(&data->client->dev.kobj,
				      &data->debug_msg_attr);
}

static int mxt_wait_for_completion(struct mxt_data *data,
				   struct completion *comp,
				   unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		return ret;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int mxt_bootloader_read(struct mxt_data *data,
			       u8 *val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(&data->client->dev, "%s: i2c recv failed (%d)\n",
			__func__, ret);
	}

	return ret;
}

static int mxt_bootloader_write(struct mxt_data *data,
				const u8 * const val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (u8 *)val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(&data->client->dev, "%s: i2c send failed (%d)\n",
			__func__, ret);
	}

	return ret;
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, bool retry)
{
	u8 appmode = data->client->addr;
	u8 bootloader;
	u8 family_id = data->info ? data->info->family_id : 0;

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* Chips after 1664S use different scheme */
		if (retry || family_id >= 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;

	default:
		dev_err(&data->client->dev,
			"Appmode i2c address 0x%02x not found\n",
			appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;
	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, bool alt_address)
{
	struct device *dev = &data->client->dev;
	int error;
	u8 buf[3];
	bool crc_failure, extended_id;

	error = mxt_lookup_bootloader_address(data, alt_address);
	if (error)
		return error;

	/* Check bootloader status and version information */
	error = mxt_bootloader_read(data, buf, sizeof(buf));
	if (error)
		return error;

	crc_failure = (buf[0] & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;
	extended_id = buf[0] & MXT_BOOT_EXTENDED_ID;

	dev_info(dev, "Found bootloader addr:%02x ID:%u%s%u%s\n",
		 data->bootloader_addr,
		 extended_id ? (buf[1] & MXT_BOOT_ID_MASK) : buf[0],
		 extended_id ? " version:" : "",
		 extended_id ? buf[2] : 0,
		 crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock);

static int mxt_write_firmware_frame(struct mxt_data *data, struct mxt_flash *f)
{
	f->frame = (struct mxt_fw_frame *)(f->fw->data + f->pos);

	/* Take account of CRC bytes */
	f->frame_size = __be16_to_cpu(f->frame->size) + 2U;

	/* Write one frame to device */
	return mxt_bootloader_write(data, f->fw->data + f->pos,
				   f->frame_size);
}

static int mxt_check_bootloader(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_flash *f = data->flash;
	u8 state;
	int ret;

	/* Handle interrupt after download/flash process */
	if (f->pos >= f->fw->size) {
		complete(&f->flash_completion);
		return 0;
	}

	ret = mxt_bootloader_read(data, &state, 1);
	if (ret)
		return ret;

	/* Remove don't care bits */
	if (state & ~MXT_BOOT_STATUS_MASK)
		state &= ~MXT_BOOT_STATUS_MASK;

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
		dev_info(dev, "Unlocking bootloader\n");
		ret = mxt_send_bootloader_cmd(data, true);
		if (ret)
			return ret;

		break;

	case MXT_WAITING_FRAME_DATA:
		if ((f->previous != MXT_WAITING_BOOTLOAD_CMD)
		    && (f->previous != MXT_FRAME_CRC_PASS)
		    && (f->previous != MXT_FRAME_CRC_FAIL))
			goto unexpected;

		ret = mxt_write_firmware_frame(data, f);
		if (ret)
			return ret;

		break;

	case MXT_FRAME_CRC_CHECK:
		if (f->previous != MXT_WAITING_FRAME_DATA)
			goto unexpected;
		break;

	case MXT_FRAME_CRC_PASS:
		if (f->previous != MXT_FRAME_CRC_CHECK)
			goto unexpected;

		/* Next frame */
		f->retry = 0;
		f->pos += f->frame_size;
		f->count++;

		if (f->pos >= f->fw->size)
			dev_info(dev, "Sent %u frames, %zu bytes\n",
				f->count, f->fw->size);
		else if (f->count % 50 == 0)
			dev_info(dev, "Sent %u frames, %lld/%zu bytes\n",
				f->count, f->pos, f->fw->size);

		break;

	case MXT_FRAME_CRC_FAIL:
		if (f->retry > 20) {
			dev_err(dev, "Retry count exceeded\n");
			return -EIO;
		}

		/* Back off by 20ms per retry */
		dev_dbg(dev, "Bootloader frame CRC failure\n");
		f->retry++;
		msleep(f->retry * 20);
		break;

	default:
		return -EINVAL;
	}

	f->previous = state;

	/* Poll after 0.1s if no interrupt received */
	schedule_delayed_work(&f->work, HZ / 10);

	return 0;

unexpected:
	dev_err(dev, "Unexpected state transition\n");
	return -EINVAL;
}

int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int ret;
	bool retry = false;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

retry_read:
	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_read;
		} else {
			dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
				__func__, ret);
			return -EIO;
		}
	}

	return 0;
}

#define MIN(a, b)       (a > b ? b : a)
static int mxt_read_blks(struct mxt_data *data, u16 start, u16 count, u8 *buf)
{
	u16 offset = 0;
	int error;
	u16 size;

	while (offset < count) {
		size = MIN(MXT_MAX_BLOCK_WRITE, count - offset);

		error = __mxt_read_reg(data->client,
				       start + offset,
				       size, buf + offset);
		if (error)
			return error;

		offset += size;
	}

	return 0;
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
			   const void *val)
{
	u8 *buf;
	size_t count;
	int ret;
	bool retry = false;

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);

retry_write:
	ret = i2c_master_send(client, buf, count);
	if (ret != count) {
		if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_write;
		} else {
			dev_err(&client->dev, "%s: i2c send failed (%d)\n",
				__func__, ret);
			ret = -EIO;
		}
	} else {
		ret = 0;
	}

	kfree(buf);
	return ret;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_warn(&data->client->dev, "Invalid object type T%u\n", type);
	return NULL;
}

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	if (crc != data->config_crc) {
		data->config_crc = crc;
		dev_dbg(dev, "T6 Config Checksum: 0x%06X\n", crc);
	}

	complete(&data->crc_completion);

	/* Detect reset */
	if (status & MXT_T6_STATUS_RESET)
		complete(&data->reset_completion);

	/* Output debug if status has changed */
	if (status != data->t6_status)
		dev_dbg(dev, "T6 Status 0x%02X%s%s%s%s%s%s%s\n",
			status,
			status == 0 ? " OK" : "",
			status & MXT_T6_STATUS_RESET ? " RESET" : "",
			status & MXT_T6_STATUS_OFL ? " OFL" : "",
			status & MXT_T6_STATUS_SIGERR ? " SIGERR" : "",
			status & MXT_T6_STATUS_CAL ? " CAL" : "",
			status & MXT_T6_STATUS_CFGERR ? " CFGERR" : "",
			status & MXT_T6_STATUS_COMSERR ? " COMSERR" : "");

	/* Save current status */
	data->t6_status = status;
}

static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object || offset >= mxt_obj_size(object))
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_reg(data->client, reg + offset, val);
}

static void mxt_input_button(struct mxt_data *data, u8 *message)
{
	struct input_dev *input = data->input_dev;
	const struct mxt_platform_data *pdata = data->pdata;
	int i;

	for (i = 0; i < pdata->t19_num_keys; i++) {
		if (pdata->t19_keymap[i] == KEY_RESERVED)
			continue;

		/* Active-low switch */
		input_report_key(input, pdata->t19_keymap[i],
				 !(message[1] & BIT(i)));
	}
}

static void mxt_input_sync(struct mxt_data *data)
{
	if (data->input_dev) {
		input_mt_report_pointer_emulation(data->input_dev,
				data->pdata->t19_num_keys);
		input_sync(data->input_dev);
	}
}

static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;
	int tool;

	id = message[0] - data->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	/* Handle 10/12 bit switching */
	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	area = message[5];

	amplitude = message[6];
	vector = message[7];

	dev_dbg(dev,
		"[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: %02X\n",
		id,
		(status & MXT_T9_DETECT) ? 'D' : '.',
		(status & MXT_T9_PRESS) ? 'P' : '.',
		(status & MXT_T9_RELEASE) ? 'R' : '.',
		(status & MXT_T9_MOVE) ? 'M' : '.',
		(status & MXT_T9_VECTOR) ? 'V' : '.',
		(status & MXT_T9_AMP) ? 'A' : '.',
		(status & MXT_T9_SUPPRESS) ? 'S' : '.',
		(status & MXT_T9_UNGRIP) ? 'U' : '.',
		x, y, area, amplitude, vector);

	input_mt_slot(input_dev, id);

	if (status & MXT_T9_DETECT) {
		/*
		 * Multiple bits may be set if the host is slow to read
		 * the status messages, indicating all the events that
		 * have happened.
		 */
		if (status & MXT_T9_RELEASE) {
			input_mt_report_slot_state(input_dev,
						   MT_TOOL_FINGER, 0);
			mxt_input_sync(data);
		}

		/* A size of zero indicates touch is from a linked T47 Stylus */
		if (area == 0) {
			area = MXT_TOUCH_MAJOR_DEFAULT;
			tool = MT_TOOL_PEN;
		} else {
			tool = MT_TOOL_FINGER;
		}

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	data->update_input = true;
}

static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	u8 type = 0;
	u16 x;
	u16 y;
	int tool = 0;
	u8 major = 0;
	u16 pressure = 0;
	u8 prs_lo = 0;
	u8 prs_hi = 0;
	u8 orientation = 0;
	bool active = false;
	bool hover = false;
	bool eraser = false;
	bool barrel = false;
	u8 pen_capacity = 0;

	id = message[0] - data->T100_reportid_min - 2;

	/* ignore SCRSTATUS events */
	if (id < 0)
		return;

	status = message[1];
	x = (message[3] << 8) | message[2];
	y = (message[5] << 8) | message[4];

	if (status & MXT_T100_DETECT) {
		type = (status & MXT_T100_TYPE_MASK) >> 4;

		switch (type) {
		case MXT_T100_TYPE_HOVERING_FINGER:
			hover = true;
			/* fall through */
		case MXT_T100_TYPE_FINGER:
		case MXT_T100_TYPE_GLOVE:
			if (global_zstylus_only_mode &&
					global_zstylus_only_pen_exist) {
				dev_dbg(dev,
				"[ATMEL] zstylus only, skip finger touch\n");
				break;
			}

			active = true;
			tool = MT_TOOL_FINGER;

			if (data->t100_aux_area)
				major = message[data->t100_aux_area];
			if (data->t100_aux_ampl)
				pressure = message[data->t100_aux_ampl] << 2;
			if (data->t100_aux_vect)
				orientation = message[data->t100_aux_vect];

			break;

		case MXT_T100_TYPE_PASSIVE_STYLUS:
			tool = MT_TOOL_PEN;
			active = true;

			/* Passive stylus is reported with size zero so
			 * hardcode */
			major = MXT_TOUCH_MAJOR_DEFAULT;

			if (data->t100_aux_ampl)
				pressure = message[data->t100_aux_ampl] << 2;

			break;

		case MXT_T100_TYPE_ACTIVE_STYLUS:
			/* stylus in range, but position unavailable */
			if (!(message[6] & MXT_T107_STYLUS_HOVER))
				break;

			active = true;
			tool = MT_TOOL_PEN;
			major = MXT_TOUCH_MAJOR_DEFAULT;
			eraser = message[6] & MXT_T107_STYLUS_BUTTON1;
			barrel = message[6] & MXT_T107_STYLUS_BUTTON0;
			pen_capacity = message[8] & 0x03;
			global_zstylus_activation = true;

			if (!(message[6] & MXT_T107_STYLUS_TIPSWITCH))
				hover = true;
			else if (data->stylus_aux_pressure){
				prs_lo = message[data->stylus_aux_pressure - 1] & 0xC0;
				prs_hi = message[data->stylus_aux_pressure];
				pressure  = ( prs_hi << 2 ) | ( prs_lo >> 6 );
			}

			if (!global_pen_present) {
				if (global_zstylus_only_mode) {
					global_zstylus_only_pen_exist = true;
					cancel_delayed_work(
						&data->report_delay_work);
				}

				global_pen_present = true;
				global_pen_capacity = global_pen_capacity_keep;
				power_supply_changed(pen_bat_supply);
			} else {
				if (pen_capacity &&
					(global_pen_capacity != pen_capacity)) {
					global_pen_capacity = pen_capacity;
					power_supply_changed(pen_bat_supply);
				}
			}

			break;

		case MXT_T100_TYPE_LARGE_TOUCH:
			/* Ignore suppressed touch */
			break;

		default:
			dev_dbg(dev, "Unexpected T100 type\n");
			return;
		}
	}

	if (hover) {
		pressure = 0;
		major = 0;
	} else if (active) {
		/*
		 * Values reported should be non-zero if tool is touching the
		 * device
		 */
		if (pressure == 0)
			pressure = MXT_PRESSURE_DEFAULT;

		if (major == 0)
			major = MXT_TOUCH_MAJOR_DEFAULT;
	}

	input_mt_slot(input_dev, id);

	if (active) {
		dev_dbg(dev, "[%u] type:%u x:%u y:%u a:%02X p:%02X v:%02X\n",
			id, type, x, y, major, pressure, orientation);
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, major);
		input_report_abs(input_dev, ABS_MT_PRESSURE, (pressure * 100) / 1023);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, orientation);

		input_report_key(input_dev, BTN_STYLUS, eraser);
		input_report_key(input_dev, BTN_STYLUS2, barrel);
	} else {
		dev_dbg(dev, "[%u] release\n", id);

		if (global_pen_present) {
			global_pen_present = false;
			global_pen_capacity_keep = global_pen_capacity;
			global_pen_capacity = 0;
			power_supply_changed(pen_bat_supply);

			if (global_zstylus_only_mode) {
				queue_delayed_work(data->mxt_wq,
					&data->report_delay_work,
					global_zstylus_only_mode_delay * HZ);
			}
		}

		/* close out slot */
		input_mt_report_slot_state(input_dev, 0, 0);
	}

	data->update_input = true;
}

static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;
	struct device *dev = &data->client->dev;
	int key;
	bool curr_state, new_state;
	bool sync = false;
	unsigned long keystates = le32_to_cpu(msg[2]);

	for (key = 0; key < 2; key++) {
		curr_state = test_bit(key, &data->t15_keystatus);
		new_state = test_bit(key, &keystates);
		if (!curr_state && new_state) {
			dev_dbg(dev, "[ATMEL] T15 key press: %u\n", key);
			__set_bit(key, &data->t15_keystatus);
			if (key == 0) {
				pr_info("[ATMEL] T15 KEY_BACK press\n");
				input_report_key(input_dev, KEY_BACK, 1);
			}
			if (key == 1) {
				pr_info("[ATMEL] T15 APP_SWITCH press\n");
				input_report_key(input_dev, APP_SWITCH, 1);
			}
			sync = true;
		} else if (curr_state && !new_state) {
			dev_dbg(dev, "[ATMEL] T15 key release: %u\n", key);
			__clear_bit(key, &data->t15_keystatus);
			if (key == 0) {
				pr_info("[ATMEL] T15 KEY_BACK release\n");
				input_report_key(input_dev, KEY_BACK, 0);
			}
			if (key == 1) {
				pr_info("[ATMEL] T15 APP_SWITCH release\n");
				input_report_key(input_dev, APP_SWITCH, 0);
			}
			sync = true;
		}
	}

	if (sync)
		input_sync(input_dev);
}

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP)
		dev_info(dev, "T42 suppress\n");
	else
		dev_info(dev, "T42 normal\n");
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status, state;

	status = msg[1];
	state  = msg[4];

	dev_dbg(dev, "T48 state %d status %02X %s%s%s%s%s\n", state, status,
		status & 0x01 ? "FREQCHG " : "",
		status & 0x02 ? "APXCHG " : "",
		status & 0x04 ? "ALGOERR " : "",
		status & 0x10 ? "STATCHG " : "",
		status & 0x20 ? "NLVLCHG " : "");

	return 0;
}

static void mxt_proc_t63_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	u8 id;
	u16 x, y;
	u8 pressure;

	/* stylus slots come after touch slots */
	id = data->num_touchids + (msg[0] - data->T63_reportid_min);

	if (id < 0 || id > (data->num_touchids + data->num_stylusids)) {
		dev_err(dev, "invalid stylus id %d, max slot is %d\n",
			id, data->num_stylusids);
		return;
	}

	x = msg[3] | (msg[4] << 8);
	y = msg[5] | (msg[6] << 8);
	pressure = msg[7] & MXT_T63_STYLUS_PRESSURE_MASK;

	dev_dbg(dev,
		"[%d] %c%c%c%c x: %d y: %d pressure: %d stylus:%c%c%c%c\n",
		id,
		msg[1] & MXT_T63_STYLUS_SUPPRESS ? 'S' : '.',
		msg[1] & MXT_T63_STYLUS_MOVE     ? 'M' : '.',
		msg[1] & MXT_T63_STYLUS_RELEASE  ? 'R' : '.',
		msg[1] & MXT_T63_STYLUS_PRESS    ? 'P' : '.',
		x, y, pressure,
		msg[2] & MXT_T63_STYLUS_BARREL   ? 'B' : '.',
		msg[2] & MXT_T63_STYLUS_ERASER   ? 'E' : '.',
		msg[2] & MXT_T63_STYLUS_TIP      ? 'T' : '.',
		msg[2] & MXT_T63_STYLUS_DETECT   ? 'D' : '.');

	input_mt_slot(input_dev, id);

	if (msg[2] & MXT_T63_STYLUS_DETECT) {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
	} else {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 0);
	}

	input_report_key(input_dev, BTN_STYLUS,
			 (msg[2] & MXT_T63_STYLUS_ERASER));
	input_report_key(input_dev, BTN_STYLUS2,
			 (msg[2] & MXT_T63_STYLUS_BARREL));

	mxt_input_sync(data);
}

static void mxt_proc_t92_messages(struct mxt_data *data, u8 *message)
{
	struct input_dev *input_dev = data->input_dev;
	u8 report_code;

	report_code = message[1] & 0x7F;

	switch (report_code) {
	case MXT_T92_REPORT_CODE_W:
		if(data->t92_gesture_sel & (0x1 << ASUS_GESTURE_MASK_W)) {
			pr_info("[ATMEL TOUCH] Resume by gesture(W)\n");
			input_report_key(input_dev, KEY_GESTURE_W, 1);
			input_sync(data->input_dev);
			input_report_key(input_dev, KEY_GESTURE_W, 0);
			input_sync(data->input_dev);
		}
		break;
	case MXT_T92_REPORT_CODE_S:
		if(data->t92_gesture_sel & (0x1 << ASUS_GESTURE_MASK_S)) {
			pr_info("[ATMEL TOUCH] Resume by gesture(S)\n");
			input_report_key(input_dev, KEY_GESTURE_S, 1);
			input_sync(data->input_dev);
			input_report_key(input_dev, KEY_GESTURE_S, 0);
			input_sync(data->input_dev);
		}
		break;
	case MXT_T92_REPORT_CODE_E:
		if(data->t92_gesture_sel & (0x1 << ASUS_GESTURE_MASK_E)) {
			pr_info("[ATMEL TOUCH] Resume by gesture(E)\n");
			input_report_key(input_dev, KEY_GESTURE_E, 1);
			input_sync(data->input_dev);
			input_report_key(input_dev, KEY_GESTURE_E, 0);
			input_sync(data->input_dev);
		}
		break;
	case MXT_T92_REPORT_CODE_C:
		if(data->t92_gesture_sel & (0x1 << ASUS_GESTURE_MASK_C)) {
			pr_info("[ATMEL TOUCH] Resume by gesture(C)\n");
			input_report_key(input_dev, KEY_GESTURE_C, 1);
			input_sync(data->input_dev);
			input_report_key(input_dev, KEY_GESTURE_C, 0);
			input_sync(data->input_dev);
		}
		break;
	case MXT_T92_REPORT_CODE_Z:
		if(data->t92_gesture_sel & (0x1 << ASUS_GESTURE_MASK_Z)) {
			pr_info("[ATMEL TOUCH] Resume by gesture(Z)\n");
			input_report_key(input_dev, KEY_GESTURE_Z, 1);
			input_sync(data->input_dev);
			input_report_key(input_dev, KEY_GESTURE_Z, 0);
			input_sync(data->input_dev);
		}
		break;
	case MXT_T92_REPORT_CODE_V:
		if(data->t92_gesture_sel & (0x1 << ASUS_GESTURE_MASK_V)) {
			pr_info("[ATMEL TOUCH] Resume by gesture(V)\n");
			input_report_key(input_dev, KEY_GESTURE_V, 1);
			input_sync(data->input_dev);
			input_report_key(input_dev, KEY_GESTURE_V, 0);
			input_sync(data->input_dev);
		}
		break;
	default:
		pr_info("[ATMEL TOUCH] Unknown gesture report code.\n");
		break;
	}
}

static void mxt_proc_t93_messages(struct mxt_data *data, u8 *message)
{
	struct input_dev *input_dev = data->input_dev;

	if (message[1] & 0x02) {
		pr_info("[ATMEL TOUCH] Resume by double click\n");
		input_report_key(input_dev, KEY_GESTURE_DOUBLE_CLICK, 1);
		input_sync(data->input_dev);
		input_report_key(input_dev, KEY_GESTURE_DOUBLE_CLICK, 0);
		input_sync(data->input_dev);
	}
}

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool dump = data->debug_enabled;

	//pr_info("[ATMEL] enter %s - report_id=%d\n", __func__, report_id);

	if (report_id == MXT_RPTID_NOMSG)
		return 0;

	if (report_id == data->T6_reportid) {
		mxt_proc_t6_messages(data, message);
	} else if (report_id >= data->T42_reportid_min
		   && report_id <= data->T42_reportid_max) {
		mxt_proc_t42_messages(data, message);
	} else if (report_id == data->T48_reportid) {
		mxt_proc_t48_messages(data, message);
	} else if (!data->input_dev) {
		/*
		 * Do not report events if input device is not
		 * yet registered or returning from suspend
		 */
		mxt_dump_message(data, message);
	} else if (report_id >= data->T9_reportid_min
	    && report_id <= data->T9_reportid_max) {
		mxt_proc_t9_message(data, message);
	} else if (report_id >= data->T100_reportid_min
	    && report_id <= data->T100_reportid_max) {
		mxt_proc_t100_message(data, message);
	} else if (report_id == data->T19_reportid) {
		mxt_input_button(data, message);
		data->update_input = true;
	} else if (report_id >= data->T63_reportid_min
		   && report_id <= data->T63_reportid_max) {
		mxt_proc_t63_messages(data, message);
	} else if (report_id >= data->T15_reportid_min
		   && report_id <= data->T15_reportid_max) {
		mxt_proc_t15_messages(data, message);
	} else if (report_id >= data->T92_reportid_min
		   && report_id <= data->T92_reportid_max) {
		mxt_proc_t92_messages(data, message);
	} else if (report_id >= data->T93_reportid_min
		   && report_id <= data->T93_reportid_max) {
		mxt_proc_t93_messages(data, message);
	} else {
		dump = true;
	}

	if (dump)
		mxt_dump_message(data, message);

	if (data->debug_v2_enabled)
		mxt_debug_msg_add(data, message);

	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg(data->client, data->T5_address,
				data->T5_msg_size * count, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(data,
			data->msg_buf + data->T5_msg_size * i);

		if (ret == 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 count, num_left;

	//pr_info("[ATMEL] mxt_process_messages_t44 begin!!!\n");
	wake_lock(&data->wake_lock);
	/* Read T44 and T5 together */
	ret = __mxt_read_reg(data->client, data->T44_address,
		data->T5_msg_size + 1, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read T44 and T5 (%d)\n", ret);
		wake_unlock(&data->wake_lock);
		return IRQ_NONE;
	}

	count = data->msg_buf[0];

	if (count == 0) {
		/*
		 * This condition is caused by the CHG line being configured
		 * in Mode 0. It results in unnecessary I2C operations but it
		 * is benign.
		 */
		dev_dbg(dev, "Interrupt triggered but zero messages\n");
		wake_unlock(&data->wake_lock);
		return IRQ_NONE;
	} else if (count > data->max_reportid) {
		dev_err(dev, "T44 count %d exceeded max report id\n", count);
		count = data->max_reportid;
	}

	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		dev_warn(dev, "Unexpected invalid message\n");
		wake_unlock(&data->wake_lock);
		return IRQ_NONE;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		//pr_info("[ATMEL] mxt_process_messages_t44 - mxt_read_and_process_messages!!!\n");
		ret = mxt_read_and_process_messages(data, num_left);
		if (ret < 0)
			goto end;
		else if (ret != num_left)
			dev_warn(dev, "Unexpected invalid message\n");
	}

end:
	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	wake_unlock(&data->wake_lock);
	return IRQ_HANDLED;
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int count, read;
	u8 tries = 2;

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mxt_read_and_process_messages(data, count);
		if (read < count)
			return 0;
	} while (--tries);

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	dev_err(dev, "CHG pin isn't cleared\n");
	return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	int total_handled, num_handled;
	u8 count = data->last_message_count;

	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count + 1);
	if (total_handled < 0)
		return IRQ_NONE;
	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* keep reading two msgs until one is invalid or reportid limit */
	do {
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_NONE;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;

	complete(&data->chg_completion);

	if (data->in_bootloader) {
		if (data->flash && &data->flash->work)
			cancel_delayed_work_sync(&data->flash->work);

		return IRQ_RETVAL(mxt_check_bootloader(data));
	}

	if (!data->object_table)
		return IRQ_HANDLED;

	if (data->T44_address) {
		return mxt_process_messages_t44(data);
	} else {
		return mxt_process_messages(data);
	}
}

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
			  u8 value, bool wait)
{
	u16 reg;
	u8 command_register;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while (command_register != 0 && timeout_counter++ <= 100);

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	dev_info(dev, "Resetting device\n");

	disable_irq(data->irq);

	reinit_completion(&data->reset_completion);

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

	/* Ignore CHG line for 100ms after reset */
	msleep(100);

	enable_irq(data->irq);

	ret = mxt_wait_for_completion(data, &data->reset_completion,
				      MXT_RESET_TIMEOUT);
	if (ret)
		return ret;

	return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	/*
	 * On failure, CRC is set to 0 and config will always be
	 * downloaded.
	 */
	data->config_crc = 0;
	reinit_completion(&data->crc_completion);

	mxt_t6_command(data, cmd, value, true);

	/*
	 * Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded.
	 */
	mxt_wait_for_completion(data, &data->crc_completion, MXT_CRC_TIMEOUT);
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static int mxt_check_retrigen(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	int val;

	if (irq_get_trigger_type(data->irq) & IRQF_TRIGGER_LOW)
		return 0;

	if (data->T18_address) {
		error = __mxt_read_reg(client,
				       data->T18_address + MXT_COMMS_CTRL,
				       1, &val);
		if (error)
			return error;

		if (val & MXT_COMMS_RETRIGEN)
			return 0;
	}

	dev_warn(&client->dev, "Enabling RETRIGEN workaround\n");
	data->use_retrigen_workaround = true;
	return 0;
}

static int mxt_prepare_cfg_mem(struct mxt_data *data,
			       const struct firmware *cfg,
			       unsigned int data_pos,
			       unsigned int cfg_start_ofs,
			       u8 *config_mem,
			       size_t config_mem_size)
{
	struct device *dev = &data->client->dev;
	struct mxt_object *object;
	unsigned int type, instance, size, byte_offset;
	int offset;
	int ret;
	int i;
	u16 reg;
	u8 val;

	while (data_pos < (cfg->size - 2)) {
		/* Read type, instance, length */
		ret = sscanf(cfg->data + data_pos, "%x %x %x%n",
			     &type, &instance, &size, &offset);
		if (ret == 0) {
			/* EOF */
			break;
		} else if (ret != 3) {
			dev_err(dev, "Bad format: failed to parse object\n");
			return -EINVAL;
		}
		data_pos += offset;

		object = mxt_get_object(data, type);
		if (!object) {
			/* Skip object */
			for (i = 0; i < size; i++) {
				ret = sscanf(cfg->data + data_pos, "%hhx%n",
					     &val, &offset);
				if (ret != 1) {
					dev_err(dev, "Bad format in T%d at %d\n",
						type, i);
					return -EINVAL;
				}
				data_pos += offset;
			}
			continue;
		}

		if (size > mxt_obj_size(object)) {
			/*
			 * Either we are in fallback mode due to wrong
			 * config or config from a later fw version,
			 * or the file is corrupt or hand-edited.
			 */
			dev_warn(dev, "Discarding %zu byte(s) in T%u\n",
				 size - mxt_obj_size(object), type);
		} else if (mxt_obj_size(object) > size) {
			/*
			 * If firmware is upgraded, new bytes may be added to
			 * end of objects. It is generally forward compatible
			 * to zero these bytes - previous behaviour will be
			 * retained. However this does invalidate the CRC and
			 * will force fallback mode until the configuration is
			 * updated. We warn here but do nothing else - the
			 * malloc has zeroed the entire configuration.
			 */
			dev_warn(dev, "Zeroing %zu byte(s) in T%d\n",
				 mxt_obj_size(object) - size, type);
		}

		if (instance >= mxt_obj_instances(object)) {
			dev_err(dev, "Object instances exceeded!\n");
			return -EINVAL;
		}

		reg = object->start_address + mxt_obj_size(object) * instance;

		for (i = 0; i < size; i++) {
			ret = sscanf(cfg->data + data_pos, "%hhx%n",
				     &val,
				     &offset);
			if (ret != 1) {
				dev_err(dev, "Bad format in T%d at %d\n",
					type, i);
				return -EINVAL;
			}
			data_pos += offset;

			if (i > mxt_obj_size(object))
				continue;

			byte_offset = reg + i - cfg_start_ofs;

			if (byte_offset >= 0 && byte_offset < config_mem_size) {
				*(config_mem + byte_offset) = val;
			} else {
				dev_err(dev, "Bad object: reg:%d, T%d, ofs=%d\n",
					reg, object->type, byte_offset);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int mxt_upload_cfg_mem(struct mxt_data *data, unsigned int cfg_start,
			      u8 *config_mem, size_t config_mem_size)
{
	unsigned int byte_offset = 0;
	int error;

	/* Write configuration as blocks */
	while (byte_offset < config_mem_size) {
		unsigned int size = config_mem_size - byte_offset;

		if (size > MXT_MAX_BLOCK_WRITE)
			size = MXT_MAX_BLOCK_WRITE;

		error = __mxt_write_reg(data->client,
					cfg_start + byte_offset,
					size, config_mem + byte_offset);
		if (error) {
			dev_err(&data->client->dev,
				"Config write error, ret=%d\n", error);
			return error;
		}

		byte_offset += size;
	}

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data);

/*
 * mxt_update_cfg - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mxt_update_cfg(struct mxt_data *data, const struct firmware *cfg)
{
	struct device *dev = &data->client->dev;
	struct mxt_info cfg_info;
	int ret;
	int offset;
	int data_pos;
	int i;
	int cfg_start_ofs;
	u32 info_crc, config_crc, calculated_crc;
	u8 *config_mem;
	size_t config_mem_size;
	u16 crc_start = 0;

	mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	if (strncmp(cfg->data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC))) {
		dev_err(dev, "Unrecognised config file\n");
		return -EINVAL;
	}

	data_pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++) {
		ret = sscanf(cfg->data + data_pos, "%hhx%n",
			     (unsigned char *)&cfg_info + i,
			     &offset);
		if (ret != 1) {
			dev_err(dev, "Bad format\n");
			return -EINVAL;
		}

		data_pos += offset;
	}

	if (cfg_info.family_id != data->info->family_id) {
		dev_err(dev, "Family ID mismatch!\n");
		return -EINVAL;
	}

	if (cfg_info.variant_id != data->info->variant_id) {
		dev_err(dev, "Variant ID mismatch!\n");
		return -EINVAL;
	}

	/* Read CRCs */
	ret = sscanf(cfg->data + data_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Info CRC\n");
		return -EINVAL;
	}
	data_pos += offset;

	ret = sscanf(cfg->data + data_pos, "%x%n", &config_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Config CRC\n");
		return -EINVAL;
	}
	data_pos += offset;

	/*
	 * The Info Block CRC is calculated over mxt_info and the object
	 * table. If it does not match then we are trying to load the
	 * configuration from a different chip or firmware version, so
	 * the configuration CRC is invalid anyway.
	 */
	if (info_crc == data->info_crc) {
		if (config_crc == 0 || data->config_crc == 0) {
			dev_info(dev, "CRC zero, attempting to apply config\n");
		} else if (config_crc == data->config_crc) {
			dev_dbg(dev, "Config CRC 0x%06X: OK\n",
				 data->config_crc);
			return 0;
		} else {
			dev_info(dev, "Config CRC 0x%06X: does not match file 0x%06X\n",
				 data->config_crc, config_crc);
		}
	} else {
		dev_warn(dev,
			 "Warning: Info CRC error - device=0x%06X file=0x%06X\n",
			 data->info_crc, info_crc);
	}

	/* Malloc memory to store configuration */
	cfg_start_ofs = MXT_OBJECT_START +
			data->info->object_num * sizeof(struct mxt_object) +
			MXT_INFO_CHECKSUM_SIZE;
	config_mem_size = data->mem_size - cfg_start_ofs;
	config_mem = kzalloc(config_mem_size, GFP_KERNEL);
	if (!config_mem)
		return -ENOMEM;

	ret = mxt_prepare_cfg_mem(data, cfg, data_pos, cfg_start_ofs,
				  config_mem, config_mem_size);
	if (ret)
		goto release_mem;

	/* Calculate crc of the received configs (not the raw config file) */
	if (data->T71_address)
		crc_start = data->T71_address;
	else if (data->T7_address)
		crc_start = data->T7_address;
	else
		dev_warn(dev, "Could not find CRC start\n");

	if (crc_start > cfg_start_ofs) {
		calculated_crc = mxt_calculate_crc(config_mem,
						   crc_start - cfg_start_ofs,
						   config_mem_size);

		if (config_crc > 0 && config_crc != calculated_crc)
			dev_warn(dev, "Config CRC in file inconsistent, calculated=%06X, file=%06X\n",
				 calculated_crc, config_crc);
	}

	ret = mxt_upload_cfg_mem(data, cfg_start_ofs,
				 config_mem, config_mem_size);
	if (ret)
		goto release_mem;

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);

	ret = mxt_check_retrigen(data);
	if (ret)
		goto release_mem;

	ret = mxt_soft_reset(data);
	if (ret)
		goto release_mem;

	dev_info(dev, "Config successfully updated\n");

	/* T7 config may have changed */
	mxt_init_t7_power_cfg(data);

release_mem:
	kfree(config_mem);
	return ret;
}

static int mxt_acquire_irq(struct mxt_data *data)
{
	int error;

	if (!data->irq) {
		error = request_threaded_irq(data->client->irq, NULL,
				mxt_interrupt,
				data->pdata->irqflags | IRQF_ONESHOT,
				data->client->name, data);
		if (error) {
			dev_err(&data->client->dev, "Error requesting irq\n");
			return error;
		}

		/* Presence of data->irq means IRQ initialised */
		data->irq = data->client->irq;
	} else {
		enable_irq(data->irq);
	}

	if (data->object_table && data->use_retrigen_workaround) {
		error = mxt_process_messages_until_invalid(data);
		if (error)
			return error;
	}

	return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void mxt_free_object_table(struct mxt_data *data)
{
	mxt_debug_msg_remove(data);

	data->object_table = NULL;
	data->info = NULL;
	kfree(data->raw_info_block);
	data->raw_info_block = NULL;
	kfree(data->msg_buf);
	data->msg_buf = NULL;
	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T71_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->T18_address = 0;
	data->T19_reportid = 0;
	data->T42_reportid_min = 0;
	data->T42_reportid_max = 0;
	data->T44_address = 0;
	data->T48_reportid = 0;
	data->T63_reportid_min = 0;
	data->T63_reportid_max = 0;
	data->T78_address = 0;
	data->T92_reportid = 0;
	data->T92_address = 0;
	data->T93_reportid = 0;
	data->T93_address = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data,
				  struct mxt_object *object_table)
{
	struct i2c_client *client = data->client;
	int i;
	u8 reportid;
	u16 end_address;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		dev_dbg(&data->client->dev,
			"T%u Start:%u Size:%zu Instances:%zu Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			if (data->info->family_id == 0x80 &&
			    data->info->version < 0x20) {
				/*
				 * On mXT224 firmware versions prior to V2.0
				 * read and discard unused CRC byte otherwise
				 * DMA reads are misaligned.
				 */
				data->T5_msg_size = mxt_obj_size(object);
			} else {
				/* CRC not enabled, so skip last byte */
				data->T5_msg_size = mxt_obj_size(object) - 1;
			}
			data->T5_address = object->start_address;
			break;
		case MXT_GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			data->T7_address = object->start_address;
			break;
		case MXT_GEN_ACQUIRE_T8:
			data->T8_address = object->start_address;
			break;
		case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
			data->T71_address = object->start_address;
			break;
		case MXT_TOUCH_MULTI_T9:
			data->multitouch = MXT_TOUCH_MULTI_T9;
			/* Only handle messages from first T9 instance */
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = min_id +
						object->num_report_ids - 1;
			data->num_touchids = object->num_report_ids;
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			data->T15_reportid_min = min_id;
			data->T15_reportid_max = max_id;
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			data->T18_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_reportid_min = min_id;
			data->T42_reportid_max = max_id;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_SPT_GPIOPWM_T19:
			data->T19_reportid = min_id;
			break;
		case MXT_PROCG_NOISESUPPRESSION_T48:
			data->T48_reportid = min_id;
			break;
		case MXT_PROCI_ACTIVE_STYLUS_T63:
			/* Only handle messages from first T63 instance */
			data->T63_reportid_min = min_id;
			data->T63_reportid_max = min_id;
			data->num_stylusids = 1;
			break;
		/*case MXT_PROCI_SYMBOLGESTUREPROCESSOR:
			data->T92_reportid = min_id;
			data->T92_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSEQUENCELOGGER:
			data->T93_reportid = min_id;
			data->T93_address = object->start_address;
			break;*/
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			data->multitouch = MXT_TOUCH_MULTITOUCHSCREEN_T100;
			data->T100_reportid_min = min_id;
			data->T100_reportid_max = max_id;
			/* first two report IDs reserved */
			data->num_touchids = object->num_report_ids - 2;
			break;
		case MXT_SPT_AUXTOUCHCONFIG_T104:
			data->T104_address = object->start_address;
			break;
		case MXT_PROCI_ACTIVESTYLUS_T107:
			data->T107_address = object->start_address;
			break;
		case MXT_SPT_USERDATA_T38:
			data->T38_address = object->start_address;
                        break;
		case MXT_PROCI_GLOVEDETECTION_T78:
			data->T78_address = object->start_address;
			break;
		case MXT_PROCI_SYMBOLGESTUREPROCESSOR_T92:
			data->T92_address = object->start_address;
			data->T92_reportid_min = min_id;
			data->T92_reportid_max = max_id;
                        break;
		case MXT_PROCI_TOUCHSEQUENCELOGGER_T93:
			data->T93_address = object->start_address;
			data->T93_reportid_min = min_id;
			data->T93_reportid_max = max_id;
                        break;
		}

		end_address = object->start_address
			+ mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(&client->dev, "Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf)
		return -ENOMEM;

	return 0;
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size;
	void *id_buf, *buf;
	uint8_t num_objects;
	u32 calculated_crc;
	u8 *crc_ptr;

	global_fw_version = 0;
	global_fw_build = 0;

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	id_buf = kzalloc(size, GFP_KERNEL);
	if (!id_buf)
		return -ENOMEM;

	error = __mxt_read_reg(client, 0, size, id_buf);
	if (error) {
		kfree(id_buf);
		return error;
	}

	/* Resize buffer to give space for rest of info block */
	num_objects = ((struct mxt_info *)id_buf)->object_num;
	size += (num_objects * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = krealloc(id_buf, size, GFP_KERNEL);
	if (!buf) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Read rest of info block */
	error = mxt_read_blks(data, MXT_OBJECT_START,
			      size - MXT_OBJECT_START,
			      buf + MXT_OBJECT_START);
	if (error)
		goto err_free_mem;

	/* Extract & calculate checksum */
	crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mxt_calculate_crc(buf, 0,
					   size - MXT_INFO_CHECKSUM_SIZE);

	/*
	 * CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol (eg i2c-hid)
	 */
	if ((data->info_crc == 0) || (data->info_crc != calculated_crc)) {
		dev_err(&client->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			calculated_crc, data->info_crc);
		error = -EIO;
		goto err_free_mem;
	}

	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;

	dev_info(&client->dev,
		 "Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id,
		 data->info->version >> 4, data->info->version & 0xf,
		 data->info->build, data->info->object_num);

	global_fw_version = data->info->version;
	global_fw_build = data->info->build;

	/* Parse object table information */
	error = mxt_parse_object_table(data, buf + MXT_OBJECT_START);
	if (error) {
		dev_err(&client->dev, "Error %d parsing object table\n", error);
		mxt_free_object_table(data);
		return error;
	}

	data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);

	return 0;

err_free_mem:
	kfree(buf);
	return error;
}

static void mxt_regulator_enable(struct mxt_data *data)
{
	int error;

	if (!data->reg_vdd || !data->reg_avdd)
		return;

	gpio_set_value(data->pdata->gpio_reset, 0);

	error = regulator_enable(data->reg_vdd);
	if (error)
		return;

	error = regulator_enable(data->reg_avdd);
	if (error)
		return;

	/*
	 * According to maXTouch power sequencing specification, RESET line
	 * must be kept low until some time after regulators come up to
	 * voltage
	 */
	msleep(MXT_REGULATOR_DELAY);
	gpio_set_value(data->pdata->gpio_reset, 1);
	msleep(MXT_CHG_DELAY);

retry_wait:
	reinit_completion(&data->chg_completion);
	data->in_bootloader = true;
	error = mxt_wait_for_completion(data, &data->chg_completion,
					MXT_POWERON_DELAY);
	if (error == -EINTR)
		goto retry_wait;

	data->in_bootloader = false;
}

static void mxt_regulator_disable(struct mxt_data *data)
{
	if (!data->reg_vdd || !data->reg_avdd)
		return;

	regulator_disable(data->reg_vdd);
	regulator_disable(data->reg_avdd);
}

static int mxt_probe_regulators(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	/* Must have reset GPIO to use regulator support */
	if (!gpio_is_valid(data->pdata->gpio_reset)) {
		error = -EINVAL;
		goto fail;
	}

	data->reg_vdd = regulator_get(dev, "vdd");
	if (IS_ERR(data->reg_vdd)) {
		error = PTR_ERR(data->reg_vdd);
		dev_err(dev, "Error %d getting vdd regulator\n", error);
		goto fail;
	}

	data->reg_avdd = regulator_get(dev, "avdd");
	if (IS_ERR(data->reg_avdd)) {
		error = PTR_ERR(data->reg_avdd);
		dev_err(dev, "Error %d getting avdd regulator\n", error);
		goto fail_release;
	}

	mxt_regulator_enable(data);

	dev_dbg(dev, "Initialised regulators\n");
	return 0;

fail_release:
	regulator_put(data->reg_vdd);
fail:
	data->reg_vdd = NULL;
	data->reg_avdd = NULL;
	return error;
}

static int mxt_read_t9_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct t9_range range;
	unsigned char orient;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T9_RANGE,
			       sizeof(range), &range);
	if (error)
		return error;

	le16_to_cpus(&range.x);
	le16_to_cpus(&range.y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T9_ORIENT,
				1, &orient);
	if (error)
		return error;

	/* Handle default values */
	if (range.x == 0)
		range.x = 1023;

	if (range.y == 0)
		range.y = 1023;

	if (orient & MXT_T9_ORIENT_SWITCH) {
		data->max_x = range.y;
		data->max_y = range.x;
	} else {
		data->max_x = range.x;
		data->max_y = range.y;
	}

	dev_dbg(&client->dev,
		"Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

/*static int mxt_set_up_active_stylus(struct input_dev *input_dev,
				    struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u8 styaux;
	int aux;
	u8 ctrl;

	object = mxt_get_object(data, MXT_PROCI_ACTIVESTYLUS_T107);
	if (!object)
		return 0;

	error = __mxt_read_reg(client, object->start_address, 1, &ctrl);
	if (error)
		return error;

	//Check enable bit
	if (!(ctrl & 0x01))
		return 0;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T107_STYLUS_STYAUX,
			       1, &styaux);
	if (error)
		return error;

	//map aux bits
	aux = 7;

	if (styaux & MXT_T107_STYLUS_STYAUX_PRESSURE)
		data->stylus_aux_pressure = aux++;

	if (styaux & MXT_T107_STYLUS_STYAUX_PEAK)
		data->stylus_aux_peak = aux++;

	input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
	input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);

	dev_dbg(&client->dev,
		"T107 active stylus, aux map pressure:%u peak:%u\n",
		data->stylus_aux_pressure, data->stylus_aux_peak);

	return 0;
}*/

static int mxt_read_t100_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u16 range_x, range_y;
	u8 cfg, tchaux;
	u8 aux;

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_XRANGE,
			       sizeof(range_x), &range_x);
	if (error)
		return error;

	le16_to_cpus(&range_x);

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_YRANGE,
			       sizeof(range_y), &range_y);
	if (error)
		return error;

	le16_to_cpus(&range_y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_CFG1,
				1, &cfg);
	if (error)
		return error;

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_TCHAUX,
				1, &tchaux);
	if (error)
		return error;

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_CTRL,
				sizeof(data->t100_ctrl), &data->t100_ctrl);
	if (error)
		return error;

	data->t100_ctrl_disable = data->t100_ctrl & 0xFD;
	data->t100_ctrl = data->t100_ctrl | 0x02;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	if (range_y == 0)
		range_y = 1023;

	if (cfg & MXT_T100_CFG_SWITCHXY) {
		data->max_x = range_y;
		data->max_y = range_x;
	} else {
		data->max_x = range_x;
		data->max_y = range_y;
	}

	/* allocate aux bytes */
	aux = 6;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		data->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		data->t100_aux_ampl = aux++;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		data->t100_aux_area = aux++;

	dev_dbg(&client->dev,
		"T100 aux mappings vect:%u ampl:%u area:%u\n",
		data->t100_aux_vect, data->t100_aux_ampl, data->t100_aux_area);

	dev_info(&client->dev,
		 "T100 Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static void mxt_set_up_as_touchpad(struct input_dev *input_dev,
				   struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	int i;

	input_dev->name = "Atmel maXTouch Touchpad";

	__set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

	input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_MT_POSITION_X,
			  MXT_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
			  MXT_PIXELS_PER_MM);

	for (i = 0; i < pdata->t19_num_keys; i++)
		if (pdata->t19_keymap[i] != KEY_RESERVED)
			input_set_capability(input_dev, EV_KEY,
					     pdata->t19_keymap[i]);
}

static int mxt_initialize_t9_input_device(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	const struct mxt_platform_data *pdata = data->pdata;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;
	unsigned int mt_flags = 0;
	int i;

	pr_info("[ATMEL] enter %s\n", __func__);
	error = mxt_read_t9_resolution(data);
	if (error)
		dev_warn(dev, "Failed to initialize T9 resolution\n");

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	input_dev->name = "Atmel maXTouch Touchscreen";
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	if (pdata->t19_num_keys) {
		__set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

		for (i = 0; i < pdata->t19_num_keys; i++)
			if (pdata->t19_keymap[i] != KEY_RESERVED)
				input_set_capability(input_dev, EV_KEY,
						     pdata->t19_keymap[i]);

		mt_flags |= INPUT_MT_POINTER;

		input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_X,
				  MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
				  MXT_PIXELS_PER_MM);

		input_dev->name = "Atmel maXTouch Touchpad";
	} else {
		mt_flags |= INPUT_MT_DIRECT;
	}

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			     0, 255, 0, 0);

	/* For multi touch */
	num_mt_slots = data->num_touchids + data->num_stylusids;
	error = input_mt_init_slots(input_dev, num_mt_slots, mt_flags);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			     0, 255, 0, 0);

	/* For T63 active stylus */
	if (data->T63_reportid_min) {
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
			0, MT_TOOL_MAX, 0, 0);
	}

	/* For T15 key array */
	if (data->T15_reportid_min) {
		data->t15_keystatus = 0;

		for (i = 0; i < data->pdata->t15_num_keys; i++)
			input_set_capability(input_dev, EV_KEY,
					     data->pdata->t15_keymap[i]);
	}

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_read_t78_glove_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_PROCI_GLOVEDETECTION_T78);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client, object->start_address, 1,
					&data->t78_ctrl_disable);
	if (error)
		return error;

	/*enable ctrl by bit-OR 00000011, disable by bit-OR 11111100*/
	data->t78_ctrl_enable = data->t78_ctrl_disable | 0x03;
	data->t78_ctrl_disable = data->t78_ctrl_disable & 0xFC;

	return 0;
}

static int mxt_read_t92_gesture_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_PROCI_SYMBOLGESTUREPROCESSOR_T92);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client, object->start_address, 1,
					&data->t92_ctrl_disable);
	if (error)
		return error;

	/*enable ctrl by bit-OR 00000011, disable by bit-AND 11111100*/
	data->t92_ctrl_enable = data->t92_ctrl_disable | 0x03;
	data->t92_ctrl_disable = data->t92_ctrl_disable & 0xFC;

	return 0;
}

static int mxt_read_t93_dclick_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_PROCI_TOUCHSEQUENCELOGGER_T93);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client, object->start_address, 1,
					&data->t93_ctrl_disable);
	if (error)
		return error;

	/*enable ctrl by bit-OR 00000011, disable by bit-OR 11111100*/
	data->t93_ctrl_enable = data->t93_ctrl_disable | 0x03;
	data->t93_ctrl_disable = data->t93_ctrl_disable & 0xFC;

	return 0;
}

static int mxt_read_t107_stylus_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u8 styaux;
	int aux;

	object = mxt_get_object(data, MXT_PROCI_ACTIVESTYLUS_T107);
	if (!object)
		return 0;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T107_STYLUS_STYAUX,
			       1, &styaux);
	if (error)
		return error;

	/* map aux bits */
	aux = 7;

	if (styaux & MXT_T107_STYLUS_STYAUX_PRESSURE)
		data->stylus_aux_pressure = aux++;

	if (styaux & MXT_T107_STYLUS_STYAUX_PEAK)
		data->stylus_aux_peak = aux++;

	dev_dbg(&client->dev,
		"Enabling T107 active stylus, aux map pressure:%u peak:%u\n",
		data->stylus_aux_pressure, data->stylus_aux_peak);

	return 0;
}

static int mxt_initialize_t100_input_device(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev;
	unsigned int mt_flags = 0;
	int error;

	pr_info("[ATMEL] enter %s\n", __func__);
	error = mxt_read_t100_config(data);
	if (error) {
		dev_err(dev, "Failed to read T100 config\n");
		return error;
	}

	error = mxt_read_t107_stylus_config(data);
	if (error)
		dev_err(dev, "Failed to read T107 config\n");

	input_dev = input_allocate_device();
	if (!data || !input_dev)
		return -ENOMEM;

	if (data->pdata->input_name)
		input_dev->name = data->pdata->input_name;
	else
		input_dev->name = "atmel_mxt_ts_T100_touchscreen";

	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &data->client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	/* For single touch */

	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);

	if (data->t100_aux_ampl)
		input_set_abs_params(input_dev, ABS_PRESSURE,
				     0, 1023, 0, 0);

	/* If device has buttons we assume it is a touchpad */
	if (pdata->t19_num_keys) {
		mxt_set_up_as_touchpad(input_dev, data);
		mt_flags |= INPUT_MT_POINTER;
	} else {
		mt_flags |= INPUT_MT_DIRECT;
	}

	/* For multi touch */
	error = input_mt_init_slots(input_dev, data->num_touchids,
				    INPUT_MT_DIRECT);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_FINGER, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);

	if (data->T107_address) {
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
	}

	if (data->t100_aux_area)
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
				     0, MXT_MAX_AREA, 0, 0);

	if (data->t100_aux_ampl | data->stylus_aux_pressure)
		input_set_abs_params(input_dev, ABS_MT_PRESSURE,
				     0, 1023, 0, 0);

	if (data->t100_aux_vect)
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
				     0, 255, 0, 0);

	/* for gesture function */
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOUBLE_CLICK);

	/* t100 For T15 key array */
	data->t15_keystatus = 0;
	input_set_capability(input_dev, EV_KEY, KEY_BACK);
	input_set_capability(input_dev, EV_KEY, APP_SWITCH);

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_sysfs_init(struct mxt_data *data);
static void mxt_sysfs_remove(struct mxt_data *data);

static int mxt_configure_objects(struct mxt_data *data,
				 const struct firmware *cfg);

static void mxt_config_cb(const struct firmware *cfg, void *ctx)
{
	mxt_configure_objects(ctx, cfg);
	release_firmware(cfg);
}

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int recovery_attempts = 0;
	int error;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	while (1) {
		error = mxt_read_info_block(data);
		if (!error) {
			pr_info("[ATMEL TOUCH] %s - read info block done.\n", __func__);
			break;
		}

		/* Check bootloader state */
		error = mxt_probe_bootloader(data, false);
		if (error) {
			dev_info(&client->dev, "Trying alternate bootloader address\n");
			error = mxt_probe_bootloader(data, true);
			if (error) {
				/* Chip is not in appmode or bootloader mode */
				return error;
			}
		}

		/* OK, we are in bootloader, see if we can recover */
		if (++recovery_attempts > 1) {
			dev_err(&client->dev, "Could not recover from bootloader mode\n");
			/*
			 * We can reflash from this state, so do not
			 * abort initialization.
			 */
			data->in_bootloader = true;
			return 2;
		}

		/* Attempt to exit bootloader into app mode */
		mxt_send_bootloader_cmd(data, false);
		msleep(MXT_FW_RESET_TIME);
	}

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	error = mxt_check_retrigen(data);
	if (error)
		goto err_free_object_table;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	error = mxt_acquire_irq(data);
	if (error)
		goto err_free_object_table;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	error = mxt_sysfs_init(data);
	if (error)
		goto err_free_object_table;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	error = mxt_debug_msg_init(data);
	if (error)
		goto err_free_object_table;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	if (data->cfg_name) {
		pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
		error = request_firmware_nowait(THIS_MODULE, true,
					data->cfg_name, &data->client->dev,
					GFP_KERNEL, data, mxt_config_cb);
		if (error) {
			dev_err(&client->dev, "Failed to invoke firmware loader: %d\n",
				error);
			goto err_free_object_table;
		}
	} else {
		pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
		error = mxt_configure_objects(data, NULL);
		if (error)
			goto err_free_object_table;
	}
	pr_info("[ATMEL TOUCH] %s - mxt initialize done.\n", __func__);

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
	struct device *dev = &data->client->dev;
	int error;
	struct t7_config *new_config;
	struct t7_config deepsleep = { .active = 0, .idle = 0 };
	struct t7_config gesture_idle = { .active = 15, .idle = 50 };

	if (sleep == MXT_POWER_CFG_DEEPSLEEP)
		new_config = &deepsleep;
	else if (sleep == MXT_POWER_CFG_IDLE)
		new_config = &gesture_idle;
	else
		new_config = &data->t7_cfg;

	error = __mxt_write_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), new_config);
	if (error)
		return error;

	dev_dbg(dev, "Set T7 ACTV:%d IDLE:%d\n",
		new_config->active, new_config->idle);

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;
	bool retry = false;

recheck:
	error = __mxt_read_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), &data->t7_cfg);
	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			dev_dbg(dev, "T7 cfg zero, resetting\n");
			mxt_soft_reset(data);
			retry = true;
			goto recheck;
		} else {
			dev_dbg(dev, "T7 cfg zero after reset, overriding\n");
			data->t7_cfg.active = 20;
			data->t7_cfg.idle = 100;
			return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		}
	}

	dev_dbg(dev, "Initialized power cfg: ACTV %d, IDLE %d\n",
		data->t7_cfg.active, data->t7_cfg.idle);
	return 0;
}

static int mxt_configure_objects(struct mxt_data *data,
				 const struct firmware *cfg)
{
	struct device *dev = &data->client->dev;
	int error;

	error = mxt_soft_reset(data);
	if (error)
		goto err_free_object_table;

	error = mxt_init_t7_power_cfg(data);
	if (error) {
		dev_err(dev, "Failed to initialize power cfg\n");
		goto err_free_object_table;
	}

	if (cfg) {
		error = mxt_update_cfg(data, cfg);
		if (error)
			dev_warn(dev, "Error %d updating config\n", error);
	}

	error = mxt_read_t78_glove_config(data);
	if (error)
		dev_err(dev, "Failed to read T78 config\n");

	error = mxt_read_t92_gesture_config(data);
	if (error)
		dev_err(dev, "Failed to read T92 config\n");

	error = mxt_read_t93_dclick_config(data);
	if (error)
		dev_err(dev, "Failed to read T93 config\n");

	error = mxt_read_t8_backup_config(data);
	if (error)
		dev_err(dev, "Failed to read T8 config\n");

	/*set zstylus according to user setting*/
	if (global_zstylus_enable) {
		pr_info("[ATMEL TOUCH] %s: zStylus enabled\n", __func__);
		mxt_t8_pen_scan_disable(data, false);
	} else {
		pr_info("[ATMEL TOUCH] %s: zStylus disabled\n", __func__);
		mxt_t8_pen_scan_disable(data, true);
	}

	if (data->T9_reportid_min) {
		error = mxt_initialize_t9_input_device(data);
		if (error)
			goto err_free_object_table;
	} else if (data->T100_reportid_min) {
		error = mxt_initialize_t100_input_device(data);
		if (error)
			goto err_free_object_table;
	} else {
		dev_warn(dev, "No touch object detected\n");
	}

	if (mxt_get_cfg_version(data))
		pr_info("[ATMEL TOUCH] read touch cfg ver. failed\n");

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

/* Configuration crc check sum is returned as hex xxxxxx */
static ssize_t mxt_config_crc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%06x\n", data->config_crc);
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
			 data->info->version >> 4, data->info->version & 0xf,
			 data->info->build);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			data->info->family_id, data->info->variant_id);
}

static ssize_t mxt_show_instance(char *buf, int count,
				 struct mxt_object *object, int instance,
				 const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 *obuf;

	if (!data->object_table)
		return -EINVAL;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = 0;
	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = mxt_read_blks(data, addr, size, obuf);
			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}

done:
	kfree(obuf);
	return error ?: count;
}

static int mxt_check_firmware_format(struct device *dev,
				     const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/*
	 * To convert file try:
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw
	 */
	dev_err(dev, "Aborting: firmware file must be in binary format\n");

	return -EINVAL;
}

static int mxt_enter_bootloader(struct mxt_data *data)
{
	int ret;

	if (data->suspended) {
		if (data->pdata->suspend_mode == MXT_SUSPEND_REGULATOR)
			mxt_regulator_enable(data);

		data->suspended = false;
	}

	if (!data->in_bootloader) {
		disable_irq(data->irq);

		/* Change to the bootloader mode */
		ret = mxt_t6_command(data, MXT_COMMAND_RESET,
				     MXT_BOOT_VALUE, false);
		if (ret)
			return ret;

		msleep(MXT_RESET_TIME);

		/* Do not need to scan since we know family ID */
		ret = mxt_probe_bootloader(data, 0);
		if (ret)
			return ret;

		data->in_bootloader = true;
		mxt_sysfs_remove(data);
		mxt_free_input_device(data);
		mxt_free_object_table(data);
	}

	dev_dbg(&data->client->dev, "Entered bootloader\n");

	return 0;
}

static void mxt_fw_work(struct work_struct *work)
{
	struct mxt_flash *f =
		container_of(work, struct mxt_flash, work.work);

	mxt_check_bootloader(f->data);
}

static int mxt_load_fw(struct device *dev)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret;

	data->flash = devm_kzalloc(dev, sizeof(struct mxt_flash), GFP_KERNEL);
	if (!data->flash)
		return -ENOMEM;

	data->flash->data = data;

	ret = request_firmware(&data->flash->fw, data->fw_name, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", data->fw_name);
		goto free;
	}

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, data->flash->fw);
	if (ret)
		goto release_firmware;

	init_completion(&data->flash->flash_completion);
	INIT_DELAYED_WORK(&data->flash->work, mxt_fw_work);
	reinit_completion(&data->flash->flash_completion);

	if (!data->in_bootloader) {
		ret = mxt_enter_bootloader(data);
		if (ret)
			goto release_firmware;
	}

	ret = mxt_acquire_irq(data);
	if (ret)
		goto release_firmware;

	/* Poll after 0.1s if no interrupt received */
	schedule_delayed_work(&data->flash->work, HZ / 10);

	/* Wait for flash. */
	ret = mxt_wait_for_completion(data, &data->flash->flash_completion,
				      MXT_BOOTLOADER_WAIT);

	disable_irq(data->irq);
	cancel_delayed_work_sync(&data->flash->work);
	data->in_bootloader = false;
release_firmware:
	release_firmware(data->flash->fw);
free:
	devm_kfree(dev, data->flash);
	return ret;
}

static int mxt_update_file_name(struct device *dev, char **file_name,
				const char *buf, size_t count)
{
	char *file_name_tmp;

	/* Simple sanity check */
	if (count > 64) {
		dev_warn(dev, "File name too long\n");
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp)
		return -ENOMEM;

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	error = mxt_update_file_name(dev, &data->fw_name, buf, count);
	if (error)
		return error;

	error = mxt_load_fw(dev);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");

		data->suspended = false;

		mxt_hw_reset(dev);

		error = mxt_initialize(data);
		if (error)
			return error;
	}

	return count;
}

static ssize_t mxt_update_cfg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct mxt_platform_data *pdata = data->pdata;
	const struct firmware *cfg;
	int ret;

	ret = mxt_update_file_name(dev, &data->cfg_name, buf, count);
	if (ret)
		return ret;

	ret = request_firmware(&cfg, data->cfg_name, dev);
	if (ret < 0) {
		dev_err(dev, "Failure to request config file %s\n",
			data->cfg_name);
		ret = -ENOENT;
		goto out;
	}

	data->updating_config = true;

	mxt_free_input_device(data);

	if (data->suspended) {
		if (pdata->suspend_mode == MXT_SUSPEND_REGULATOR) {
			enable_irq(data->irq);
			mxt_regulator_enable(data);
		} else if (pdata->suspend_mode == MXT_SUSPEND_DEEP_SLEEP) {
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
			mxt_acquire_irq(data);
		}

		data->suspended = false;
	}

	ret = mxt_configure_objects(data, cfg);
	if (ret)
		goto release;

	ret = count;

release:
	release_firmware(cfg);
out:
	data->updating_config = false;
	return ret;
}

//++++++ add by angel ++++++
static int mxt_self_update_cfg(struct mxt_data *data){

	struct device *dev = &data->client->dev;
	const struct mxt_platform_data *pdata = data->pdata;
	const struct firmware *cfg;
	int ret = 0;

	pr_info("[ATMEL] mxt_self_update_cfg begin!!!\n");
	if (data->in_bootloader) {
		dev_err(dev, "Not in appmode\n");
		return -EINVAL;
	}

	ret = request_firmware(&cfg, MXT_CONFIG_NAME_1666T2, dev);
	if (ret < 0) {
		dev_err(dev, "Failure to request config file %s\n",
			MXT_CONFIG_NAME_1666T2);
		ret = -ENOENT;
		goto out;
	}
	data->updating_config = true;

	mxt_free_input_device(data);

	if (data->suspended) {
		if (pdata->suspend_mode == MXT_SUSPEND_REGULATOR) {
			enable_irq(data->irq);
			mxt_regulator_enable(data);
		} else if (pdata->suspend_mode == MXT_SUSPEND_DEEP_SLEEP) {
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
			mxt_acquire_irq(data);
		}

		data->suspended = false;
	}

	ret = mxt_configure_objects(data, cfg);
	pr_info("[ATMEL] mxt_configure_objects begin!!!\n");
	if (ret) {
		pr_info("[ATMEL] self_update_cfg OK!!!\n");
		goto release;
	}
	//ret = count;

release:
	release_firmware(cfg);
out:
	data->updating_config = false;
	return ret;
}

static void mxt_force_updating(struct work_struct *work)
{
	struct mxt_data *data =
			container_of(work,
				struct mxt_data, work_upgrade);
	u8 fw_info;
	int error;

	pr_info("[ATMEL] enter mxt_force_updating function \n");
	msleep(20);
	if (data->info != NULL)
		fw_info = data->info->build;


	data->force_upgrade = 1;

	data->fw_name = MXT_FIRMWARE_NAME_1666T2;
	data->cfg_name = MXT_CONFIG_NAME_1666T2;
	printk("[Atmel] %s ++ test \n",__func__);
	if (data->force_upgrade) {
		error = mxt_load_fw(&data->client->dev);
		if (error) {
			dev_err(&data->client->dev, "The firmware update failed(%d)\n", error);
		} else {
			dev_info(&data->client->dev, "The firmware update succeeded\n");
			msleep(MXT_RESET_TIME);
			pr_info("[ATMEL] firmware force update succeeded, goto mxt_initialize\n");
			mxt_initialize(data);
		}
	}

	disable_irq(data->irq);
	pr_info("[ATMEL] firmware force update done, goto mxt_self_update_cfg\n");
	mxt_self_update_cfg(data);
	enable_irq(data->irq);
}
//++++++ add by angel ++++++

static ssize_t mxt_debug_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char c;

	c = data->debug_enabled ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_debug_notify_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t mxt_debug_v2_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		if (i == 1)
			mxt_debug_msg_enable(data);
		else
			mxt_debug_msg_disable(data);

		ret = count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t mxt_debug_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		data->debug_enabled = (i == 1);

		dev_dbg(dev, "%s\n", i ? "debug enabled" : "debug disabled");
		ret = count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
				       size_t *count)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > MXT_MAX_BLOCK_WRITE)
		*count = MXT_MAX_BLOCK_WRITE;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_read_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_write_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_touch_status_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *t38_object;
	int err_r, err_w, i, touch_status;
	u8 *obuf;
	u16 addr;

	t38_object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
	if (!t38_object)
                return -EINVAL;

	obuf = kmalloc(1, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	addr = (t38_object->start_address) + 63;
	for (i = 0; i < 3; i++) {
		err_w = mxt_write_reg(data->client, addr, 0x66);
		msleep(10);
		err_r = __mxt_read_reg(data->client, addr, 1, obuf);
		msleep(10);
		err_w = mxt_write_reg(data->client, addr, 0x0);
		if (!err_w && !err_r)
			break;
	}

	pr_info("[ATMEL] object T%u last bytes is %02x\n",
					t38_object->type, obuf[0]);
	if ( i >= 3 || obuf[0] != 0x66)
		touch_status = 0;
	else
		touch_status = 1;

	kfree(obuf);
	return scnprintf(buf, PAGE_SIZE, "%d\n", touch_status);
}

static ssize_t mxt_touch_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	sscanf(buf, "%u", &i);
	switch (i) {
	case 0:
		if (data->sysfs_enable_irq) {
			pr_info("[ATMEL] disable irq.\n");
			disable_irq(data->irq);
			data->sysfs_enable_irq = false;
		}
		else
			pr_info("[ATMEL] irq already disable.\n");
		break;
	case 1:
		if (!(data->sysfs_enable_irq)) {
			pr_info("[ATMEL] enable irq.\n");
			enable_irq(data->irq);
			data->sysfs_enable_irq = true;
		}
		else
			pr_info("[ATMEL] irq already enable.\n");
		break;
	default:
		break;
	}
	return count;
}

static ssize_t mxt_touch_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", data->sysfs_enable_irq ? 1 : 0);
}

static ssize_t atmel_touch_switch_name(struct switch_dev *sdev, char *buf)
{
	struct mxt_data *data = container_of(sdev, struct mxt_data, touch_sdev);

	return scnprintf(buf, PAGE_SIZE, "fw:%u.%u.%02X cfg:%u.%u csum:%06x\n",
				global_fw_version >> 4,
				global_fw_version & 0xf,
				global_fw_build,
				global_cfg_version,
				global_cfg_build,
				data->config_crc);
}

static ssize_t mxt_tp_fw_info_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "fw:%u.%u.%02X cfg:%u.%u csum:%06x\n",
				global_fw_version >> 4,
				global_fw_version & 0xf,
				global_fw_build,
				global_cfg_version,
				global_cfg_build,
				data->config_crc);
}

static ssize_t config_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	if(mxt_get_cfg_version(data))
		return scnprintf(buf, PAGE_SIZE, "0\n");
	else
		return scnprintf(buf, PAGE_SIZE, "%u%u\n",
					global_cfg_version,
					global_cfg_build);
}

static ssize_t mxt_T78_glove_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "Glove mode = %d\n",
					data->t78_master_ctrl);
}

static ssize_t mxt_T78_glove_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	if (buf[0] == '1') {
		data->t78_master_ctrl = true;
		mxt_write_reg(data->client, data->T78_address, data->t78_ctrl_enable);
	}
	else{	/*disabled if input unknown*/
		data->t78_master_ctrl = false;
		mxt_write_reg(data->client, data->T78_address, data->t78_ctrl_disable);
	}

	pr_info("[ATMEL TOUCH] %s: set glove mode = %d\n",
			__func__, data->t78_master_ctrl);
	return count;
}

static ssize_t mxt_T93_dclick_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "Double click = %d\n",
					data->t93_master_ctrl);
}

static ssize_t mxt_T93_dclick_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	if (buf[0] == '1')
		data->t93_master_ctrl = true;
	else	/*disabled if input unknown*/
		data->t93_master_ctrl = false;

	pr_info("[ATMEL TOUCH] %s: set dclick = %d\n",
			__func__, data->t93_master_ctrl);
	return count;
}

static ssize_t mxt_T92_gesture_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "Gesture setting = 0x%x\n",
					data->t92_gesture_sel);
}

static ssize_t mxt_T92_gesture_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;
	u8 ges_sel = 0;

	for (i = 0; i <= ASUS_GESTURE_COUNT; i++){
		if (buf[i] == '1')
			ges_sel = ges_sel | (0x1 << (ASUS_GESTURE_COUNT - i));
		else	/*disabled if input unknown*/
			ges_sel = ges_sel & ~(0x1 << (ASUS_GESTURE_COUNT - i));
	}

	data->t92_gesture_sel = ges_sel;
	pr_info("[ATMEL TOUCH] %s: set gesture sel = 0x%x\n",
			__func__, data->t92_gesture_sel);
	return count;
}

/*update cfg name*/
static ssize_t mxt_update_cfg_name_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret;

	ret = mxt_update_file_name(dev, &data->cfg_name, buf, count);
	if(ret)
		pr_info("[ATMEL]%s: update file name failed!\n", __func__);

	return count;
}

/*read cfg name*/
static ssize_t mxt_cfg_name_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
        struct mxt_data *data = dev_get_drvdata(dev);
        if(data->cfg_name == NULL)
                return scnprintf(buf, PAGE_SIZE, "NULL\n");
        else
	        return scnprintf(buf, PAGE_SIZE, "%s\n", data->cfg_name);
}

static DEVICE_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);

static struct attribute *mxt_fw_attrs[] = {
	&dev_attr_update_fw.attr,
	NULL
};

static ssize_t mxt_zstylus_only_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	if (buf[0] == '0') {
		global_zstylus_only_mode = false;
		global_zstylus_only_mode_delay = 1;
	} else {
		global_zstylus_only_mode = true;
		global_zstylus_only_mode_delay = buf[2] - '0';
	}

	pr_info("[ATMEL TOUCH] zstylus only %s, delay = %ds\n",
		(global_zstylus_only_mode)? "enabled" : "disabled",
		global_zstylus_only_mode_delay);

	return count;
}

static ssize_t mxt_zstylus_only_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "zstylus only = %s, delay = %d\n",
		(global_zstylus_only_mode)? "enabled" : "disabled",
		global_zstylus_only_mode_delay);
}

static ssize_t mxt_zstylus_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	if (buf[0] == '1')
		global_zstylus_enable = true;
	else
		global_zstylus_enable = false;

	if (data->in_bootloader || data->updating_config) {
		pr_info("[ATMEL TOUCH] %s: updating fw, abort zstylus store\n",
								__func__);
		goto zstylus_enable_end;
	}

	if (global_zstylus_enable) {
		pr_info("[ATMEL TOUCH] %s: zStylus enabled\n", __func__);
		mxt_t8_pen_scan_disable(data, false);
	} else {
		pr_info("[ATMEL TOUCH] %s: zStylus disabled\n", __func__);
		mxt_t8_pen_scan_disable(data, true);
	}

zstylus_enable_end:
	return count;
}

static ssize_t mxt_zstylus_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "zStylus = %d\n",
					global_zstylus_enable);
}

static ssize_t mxt_soft_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	if (mxt_soft_reset(data))
		pr_info("[ATMEL TOUCH] soft reset failed\n");

	return count;
}

static ssize_t mxt_zstylus_activation_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n",
					global_zstylus_activation);
}

static const struct attribute_group mxt_fw_attr_group = {
	.attrs = mxt_fw_attrs,
};

static DEVICE_ATTR(fw_version, S_IRUGO, mxt_fw_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static DEVICE_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static DEVICE_ATTR(update_cfg, S_IWUSR, NULL, mxt_update_cfg_store);
static DEVICE_ATTR(config_crc, S_IRUGO, mxt_config_crc_show, NULL);
static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show,
		   mxt_debug_enable_store);
static DEVICE_ATTR(debug_v2_enable, S_IWUSR | S_IRUSR, NULL,
		   mxt_debug_v2_enable_store);
static DEVICE_ATTR(debug_notify, S_IRUGO, mxt_debug_notify_show, NULL);
static DEVICE_ATTR(touch_status, S_IRUGO, mxt_touch_status_show, NULL);
static DEVICE_ATTR(touch_enable, S_IWUSR | S_IRUSR, mxt_touch_enable_show,
			mxt_touch_enable_store);
static DEVICE_ATTR(tp_fw_info, S_IRUGO, mxt_tp_fw_info_show, NULL);
static DEVICE_ATTR(config_version, S_IRUGO, config_version_show, NULL);
static DEVICE_ATTR(glove_mode, S_IWUSR | S_IRUGO, mxt_T78_glove_show,
					mxt_T78_glove_store);
static DEVICE_ATTR(dclick, S_IWUSR | S_IRUGO, mxt_T93_dclick_show,
					mxt_T93_dclick_store);
static DEVICE_ATTR(gesture, S_IWUSR | S_IRUGO, mxt_T92_gesture_show,
					mxt_T92_gesture_store);
static DEVICE_ATTR(cfg_name_update, S_IWUSR | S_IRUSR,
			mxt_cfg_name_show, mxt_update_cfg_name_store);
static DEVICE_ATTR(zstylus_enable, S_IWUSR | S_IRUSR,
			mxt_zstylus_enable_show, mxt_zstylus_enable_store);
static DEVICE_ATTR(touch_reset, S_IWUSR, NULL, mxt_soft_reset_store);
static DEVICE_ATTR(zstylus_activation, S_IRUGO,
			mxt_zstylus_activation_show, NULL);
static DEVICE_ATTR(zstylus_only_enable, S_IWUSR | S_IRUSR,
			mxt_zstylus_only_mode_show,
			mxt_zstylus_only_mode_store);

static struct attribute *mxt_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_object.attr,
	&dev_attr_update_cfg.attr,
	&dev_attr_config_crc.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_debug_v2_enable.attr,
	&dev_attr_debug_notify.attr,
	&dev_attr_touch_status.attr,
	&dev_attr_touch_enable.attr,
	&dev_attr_tp_fw_info.attr,
	&dev_attr_config_version.attr,
	&dev_attr_glove_mode.attr,
	&dev_attr_dclick.attr,
	&dev_attr_gesture.attr,
	&dev_attr_cfg_name_update.attr,
	&dev_attr_zstylus_enable.attr,
	&dev_attr_touch_reset.attr,
	&dev_attr_zstylus_activation.attr,
	&dev_attr_zstylus_only_enable.attr,
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

static int mxt_sysfs_init(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error) {
		dev_err(&client->dev, "Failure %d creating sysfs group\n",
			error);
		return error;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	error = sysfs_create_bin_file(&client->dev.kobj,
				  &data->mem_access_attr);
	if (error) {
		dev_err(&client->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_remove_sysfs_group;
	}

	return 0;

err_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	return error;
}

static void mxt_sysfs_remove(struct mxt_data *data)
{
	struct i2c_client *client = data->client;

	if (data->mem_access_attr.attr.name)
		sysfs_remove_bin_file(&client->dev.kobj,
				      &data->mem_access_attr);

	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
}

static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	unsigned int num_mt_slots;
	int id;

	if (!input_dev)
		return;

	num_mt_slots = data->num_touchids + data->num_stylusids;

	for (id = 0; id < num_mt_slots; id++) {
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	mxt_input_sync(data);
}

static int mxt_start(struct mxt_data *data)
{
	int ret;

	if (!data->suspended || data->in_bootloader)
		return 0;

	ret = mxt_write_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100, MXT_T100_CTRL, 0x83);
	if (ret != 0)
			pr_info("[ATMEL TOUCH]%s:T100 cfg write error, ret=%d\n",
								__func__, ret);

	ret = mxt_write_object(data, MXT_TOUCH_KEYARRAY_T15, MXT_T15_CTRL, 0x03);
	if (ret != 0)
			pr_info("[ATMEL TOUCH]%s:T15 cfg write error, ret=%d\n",
								__func__, ret);

	ret = mxt_write_reg(data->client, data->T78_address,
					data->t78_ctrl_disable);
	if (ret != 0)
		pr_info("[ATMEL TOUCH]%s: T78 Config write error, ret=%d\n",
							__func__, ret);

	ret = mxt_write_reg(data->client, data->T92_address,
					data->t92_ctrl_disable);
	if (ret != 0)
		pr_info("[ATMEL TOUCH]%s: T92 Config write error, ret=%d\n",
							__func__, ret);

	ret = mxt_write_reg(data->client, data->T93_address,
					data->t93_ctrl_disable);
	if (ret != 0)
		pr_info("[ATMEL TOUCH]%s: T93 Config write error, ret=%d\n",
							__func__, ret);

	switch (data->pdata->suspend_mode) {
	case MXT_SUSPEND_T9_CTRL:
		pr_info("[ATMEL] %s-%d, suspend_mode:MXT_SUSPEND_T9_CTRL \n", __func__, __LINE__);
		mxt_soft_reset(data);

		/* Touch enable */
		/* 0x83 = SCANEN | RPTEN | ENABLE */
		mxt_write_object(data,
				MXT_TOUCH_MULTI_T9, MXT_T9_CTRL, 0x83);
		break;

	case MXT_SUSPEND_REGULATOR:
		pr_info("[ATMEL] %s-%d, suspend_mode:MXT_SUSPEND_REGULATOR \n", __func__, __LINE__);
		enable_irq(data->irq);
		mxt_regulator_enable(data);
		break;

	case MXT_SUSPEND_DEEP_SLEEP:
		pr_info("[ATMEL] %s-%d, suspend_mode:MXT_SUSPEND_DEEP_SLEEP \n", __func__, __LINE__);
	default:
		pr_info("[ATMEL] %s-%d, suspend_mode:default \n", __func__, __LINE__);
		/*
		 * Discard any touch messages still in message buffer
		 * from before chip went to sleep
		 */
		mxt_process_messages_until_invalid(data);

		ret = mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);

		if(global_zstylus_enable)
			mxt_t8_pen_scan_disable(data, false);
		else
			mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);

		/*enable T107 vsync to enable pen touch*/
		//mxt_t107_frame_sync_disable(data, false);

		pr_info("[ATMEL TOUCH] TS_CALIBRATE.\n");

		if (!data->gesture_wakeup)
			mxt_acquire_irq(data);
		else
			disable_irq_wake(data->irq);
		break;
	}

	data->suspended = false;

	return 0;
}

static int mxt_stop(struct mxt_data *data)
{
	int ret;

	if (data->suspended || data->in_bootloader || data->updating_config)
		return 0;

	ret = mxt_write_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100, MXT_T100_CTRL, 0x81);
	if (ret != 0)
			pr_info("[ATMEL TOUCH]%s:T100 cfg write error, ret=%d\n",
								__func__, ret);

	ret = mxt_write_object(data, MXT_TOUCH_KEYARRAY_T15, MXT_T15_CTRL, 0);
	if (ret != 0)
			pr_info("[ATMEL TOUCH]%s:T15 cfg write error, ret=%d\n",
								__func__, ret);

	switch (data->pdata->suspend_mode) {
	case MXT_SUSPEND_T9_CTRL:
		/* Touch disable */
		ret = mxt_write_object(data,
				MXT_TOUCH_MULTI_T9, MXT_T9_CTRL, 0);
		if (ret)
			return ret;

		break;

	case MXT_SUSPEND_REGULATOR:
		disable_irq(data->irq);
		mxt_regulator_disable(data);
		mxt_reset_slots(data);
		break;

	case MXT_SUSPEND_DEEP_SLEEP:
	default:
		data->gesture_wakeup = false;
		pr_info("[ATMEL TOUCH]%s: gesture enabled? = 0x%x\n",
						__func__, data->t92_gesture_sel);

		if (data->t92_gesture_sel & (0x1 << ASUS_GESTURE_COUNT)) {
			data->gesture_wakeup = true;
			ret = mxt_write_reg(data->client, data->T92_address,
							data->t92_ctrl_enable);
			if (ret != 0)
				pr_info("[ATMEL TOUCH]%s:T92 cfg write error, ret=%d\n",
									__func__, ret);
		}

		pr_info("[ATMEL TOUCH]%s: double click enabled? = %d\n",
						__func__, data->t93_master_ctrl);
		if (data->t93_master_ctrl) {
			data->gesture_wakeup = true;
			ret = mxt_write_reg(data->client, data->T93_address,
							data->t93_ctrl_enable);
			if (ret != 0)
				pr_info("[ATMEL TOUCH]%s:T93 cfg write error, ret=%d\n",
									__func__, ret);
		}

		if (data->gesture_wakeup) {
			/* Modify t8 to disable pen scan and trigger calibration
			if zstylus is disabled, use t6 to trigger calibration
			*/
			enable_irq_wake(data->irq);

			if (global_zstylus_enable)
				mxt_t8_pen_scan_disable(data, true);
			else
				mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);

			/*disable T107 vsync to avoid finger touch fail*/
			//mxt_t107_frame_sync_disable(data, true);

			if (data->lid_closed) {
				mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
				pr_info("[ATMEL]:Touch DEEPSLEEP ...\n");
			}
			else {
				mxt_set_t7_power_cfg(data, MXT_POWER_CFG_IDLE);
				pr_info("[ATMEL]:Touch GESTURE ...\n");
			}

		} else {
			mxt_t8_pen_scan_disable(data, true);

			/*disable T107 vsync to avoid finger touch fail*/
			//mxt_t107_frame_sync_disable(data, true);

			disable_irq(data->irq);
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
		}

		mxt_reset_slots(data);
		break;
	}

	data->suspended = true;
	return 0;
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int ret;

	ret = mxt_start(data);

	if (ret)
		dev_err(&data->client->dev, "%s failed rc=%d\n", __func__, ret);

	return ret;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int ret;

	ret = mxt_stop(data);

	if (ret)
		dev_err(&data->client->dev, "%s failed rc=%d\n", __func__, ret);
}

/*static void mxt_t107_frame_sync_disable(struct mxt_data *data, bool suspend)
{
	struct i2c_client *client = data->client;
	int ret;

	if (suspend) {
		ret = mxt_write_reg(client,
				data->T107_address + MXT_T107_FRAME_SYNC, 0);
		if (ret)
			pr_info("[ATMEL TOUCH] %d-%s: write T107 cfg failed\n",
					__LINE__ ,__func__);
	} else {
		ret = mxt_write_reg(client,
				data->T107_address + MXT_T107_FRAME_SYNC, 1);
		if (ret)
			pr_info("[ATMEL TOUCH] %d-%s: write T107 cfg failed\n",
					__LINE__ ,__func__);
	}
}*/

static int mxt_read_t8_backup_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int ret = 0;

	ret = __mxt_read_reg(client, data->T8_address + 10,
					1, &data->t8_backup_cfg);
	if (ret)
		pr_info("[ATMEL TOUCH] %d-%s: read T8 cfg failed\n",
					__LINE__ ,__func__);
	return ret;
}

static void mxt_t8_pen_scan_disable(struct mxt_data *data, bool suspend)
{
	struct i2c_client *client = data->client;
	int ret;

	if (suspend) {
		/*write 3 to byte 10 for disable pen scan, trigger cal*/
		ret = mxt_write_reg(client, data->T8_address + 10, 3);
		if (ret)
			pr_info("[ATMEL TOUCH] %d-%s: write T8 cfg failed\n",
					__LINE__ ,__func__);
	} else {
		ret = mxt_write_reg(client, data->T8_address + 10,
					data->t8_backup_cfg);
		if (ret)
			pr_info("[ATMEL TOUCH] %d-%s: write T8 cfg failed\n",
					__LINE__ ,__func__);
	}
}

static void mxt_report_delay_work(struct work_struct *work)
{
	global_zstylus_only_pen_exist = false;
	power_supply_changed(pen_bat_supply);
}

static int mxt_get_cfg_version(struct mxt_data *data)
{
	struct mxt_object *t38_object;
        int error;
        u16 addr;
        u8 *obuf;

	global_cfg_version = 0;
	global_cfg_build = 0;

        if(data->in_bootloader){
		pr_info("[ATMEL TOUCH]: Not in appmode\n");
		return -EINVAL;
	}

	obuf = kmalloc(2, GFP_KERNEL);
	if (!obuf)
                return -ENOMEM;

	t38_object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
	if (!t38_object)
                goto read_cfg_version_fail;

        addr = t38_object->start_address;
        error = __mxt_read_reg(data->client, addr, 2, obuf);
        if (error)
                goto read_cfg_version_fail;

	global_cfg_version = obuf[0];
	global_cfg_build = obuf[1];
        kfree(obuf);
	return 0;

read_cfg_version_fail:
        kfree(obuf);
	return -EINVAL;
}

static int mxt_gpio_init(struct mxt_data *data)
{
	int ret;
	if (!gpio_is_valid(data->pdata->gpio_reset) ||
			/*!gpio_is_valid(data->pdata->gpio_power) ||*/
			!gpio_is_valid(data->pdata->gpio_int)) {
		pr_info("[ATMEL] touch gpio is invalid\n");
		return -EINVAL;
	}

	ret = gpio_request(data->pdata->gpio_reset, "atmel_mxt_rst");
	if (ret) {
		pr_info("[ATMEL] gpio-reset:%d request fail.\n",
					(int)data->pdata->gpio_reset);
		return ret;
	}
	/*ret = gpio_request(data->pdata->gpio_power, "atmel_mxt_3v3");
	if (ret) {
		pr_info("[ATMEL] gpio-reset:%d request fail.\n",
					(int)data->pdata->gpio_reset);
		return ret;
	}*/
	ret = gpio_request(data->pdata->gpio_int, "atmel_mxt_int");
	if (ret) {
		pr_info("[ATMEL] gpio-int:%d request fail.\n",
					(int)data->pdata->gpio_int);
		return ret;
	}

	gpio_direction_input(data->pdata->gpio_int);
	gpio_direction_output(data->pdata->gpio_reset, 0);
	//gpio_direction_output(data->pdata->gpio_power, 0);
	msleep(MXT_RESET_TIME);
	return 0;
}

static int mxt_power_on(struct mxt_data *data)
{
	int ret;

	pr_info("[ATMEL] %s: rst = %d, power = %d, int = %d\n",
			__func__,
			gpio_get_value(data->pdata->gpio_reset),
			gpio_get_value(data->pdata->gpio_power),
			gpio_get_value(data->pdata->gpio_int));

	if (/*!tpd->reg || */!tpd->io_reg)
		return -EINVAL;

	//ret = regulator_enable(tpd->reg);
	//if (ret != 0) {
	//	pr_info("[ATMEL] Failed to enable reg-vgp6: %d\n", ret);
	//	return -EINVAL;
	//}
	ret = regulator_enable(tpd->io_reg);
	if (ret != 0) {
		pr_info("[ATMEL] Failed to enable reg-vgp4: %d\n", ret);
		return -EINVAL;
	}
	gpio_set_value(data->pdata->gpio_power, 1);
	pr_info("[ATMEL] %s: rst = %d, power = %d, int = %d\n",
			__func__,
			gpio_get_value(data->pdata->gpio_reset),
			gpio_get_value(data->pdata->gpio_power),
			gpio_get_value(data->pdata->gpio_int));
	msleep(MXT_REGULATOR_DELAY);
	gpio_set_value(data->pdata->gpio_reset, 1);
	pr_info("[ATMEL] %s: rst = %d, power = %d, int = %d\n",
			__func__,
			gpio_get_value(data->pdata->gpio_reset),
			gpio_get_value(data->pdata->gpio_power),
			gpio_get_value(data->pdata->gpio_int));
	msleep(MXT_CHG_DELAY);
	return 0;
}

static void mxt_hw_reset(struct device *dev)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	pr_info("[ATMEL] start HW reset\n");

	gpio_set_value(data->pdata->gpio_power, 0);
	gpio_set_value(data->pdata->gpio_reset, 0);
	msleep(MXT_RESET_TIME);
	dev_dbg(dev, "[ATMEL] %s: rst = %d, power = %d, int = %d\n",
			__func__,
			gpio_get_value(data->pdata->gpio_reset),
			gpio_get_value(data->pdata->gpio_power),
			gpio_get_value(data->pdata->gpio_int));

	gpio_set_value(data->pdata->gpio_power, 1);
	dev_dbg(dev, "[ATMEL] %s: rst = %d, power = %d, int = %d\n",
			__func__,
			gpio_get_value(data->pdata->gpio_reset),
			gpio_get_value(data->pdata->gpio_power),
			gpio_get_value(data->pdata->gpio_int));
	msleep(MXT_REGULATOR_DELAY);

	gpio_set_value(data->pdata->gpio_reset, 1);
	dev_dbg(dev, "[ATMEL] %s: rst = %d, power = %d, int = %d\n",
			__func__,
			gpio_get_value(data->pdata->gpio_reset),
			gpio_get_value(data->pdata->gpio_power),
			gpio_get_value(data->pdata->gpio_int));
	msleep(MXT_CHG_DELAY);
	pr_info("[ATMEL] HW reset done\n");
}

static int of_get_pcbid_data(void)
{
	struct device_node *np;
	const char *pcbid_str;

	np = of_find_compatible_node(NULL, NULL, "android,firmware");
	if (np == NULL) {
		pcbid_prj_id = PRJ_ID_Z500M;
		return 1;
	}

	pcbid_str = of_get_property(np, "project_id", NULL);
	if (pcbid_str == NULL) {
		pr_err("[ATMEL] get project_id failed, use default Z500M\n");
		pcbid_prj_id = PRJ_ID_Z500M;
	} else
		pcbid_prj_id = *pcbid_str;

	return 0;
}

#ifdef CONFIG_OF
static const struct mxt_platform_data *mxt_parse_dt(struct i2c_client *client)
{
	struct mxt_platform_data *pdata;
	struct device_node *np = client->dev.of_node;
	u32 *keymap;
	int proplen, ret;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	if (!np)
		return ERR_PTR(-ENOENT);

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	pdata->gpio_reset = of_get_named_gpio(np, "rst-gpio", 0);
	pdata->gpio_int = of_get_named_gpio(np, "int-gpio", 0);
	pdata->gpio_power = of_get_named_gpio(np, "power-gpio", 0);
	pdata->touch_irq = irq_of_parse_and_map(np, 0);

	of_property_read_string(np, "atmel,cfg_name", &pdata->cfg_name);

	of_property_read_string(np, "atmel,input_name", &pdata->input_name);

	if (of_find_property(np, "linux,gpio-keymap", &proplen)) {
		pdata->t19_num_keys = proplen / sizeof(u32);

		keymap = devm_kzalloc(&client->dev,
				pdata->t19_num_keys * sizeof(keymap[0]),
				GFP_KERNEL);
		if (!keymap)
			return ERR_PTR(-ENOMEM);

		ret = of_property_read_u32_array(np, "linux,gpio-keymap",
						 keymap, pdata->t19_num_keys);
		if (ret)
			dev_warn(&client->dev,
				 "Couldn't read linux,gpio-keymap: %d\n", ret);

		pdata->t19_keymap = keymap;
	}

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	//of_property_read_u32(np, "atmel,suspend-mode", &pdata->suspend_mode);
	pdata->suspend_mode = MXT_SUSPEND_DEEP_SLEEP;

	return pdata;
}
#else
static const struct mxt_platform_data *mxt_parse_dt(struct i2c_client *client)
{
	return ERR_PTR(-ENOENT);
}
#endif

#ifdef CONFIG_ACPI

struct mxt_acpi_platform_data {
	const char *hid;
	struct mxt_platform_data pdata;
};

static unsigned int samus_touchpad_buttons[] = {
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	BTN_LEFT
};

static struct mxt_acpi_platform_data samus_platform_data[] = {
	{
		/* Touchpad */
		.hid	= "ATML0000",
		.pdata	= {
			.t19_num_keys	= ARRAY_SIZE(samus_touchpad_buttons),
			.t19_keymap	= samus_touchpad_buttons,
		},
	},
	{
		/* Touchscreen */
		.hid	= "ATML0001",
	},
	{ }
};

static const struct dmi_system_id mxt_dmi_table[] = {
	{
		/* 2015 Google Pixel */
		.ident = "Chromebook Pixel 2",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "GOOGLE"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Samus"),
		},
		.driver_data = samus_platform_data,
	},
	{ }
};

static const struct mxt_platform_data *mxt_parse_acpi(struct i2c_client *client)
{
	struct acpi_device *adev;
	const struct dmi_system_id *system_id;
	const struct mxt_acpi_platform_data *acpi_pdata;

	/*
	 * Ignore ACPI devices representing bootloader mode.
	 *
	 * This is a bit of a hack: Google Chromebook BIOS creates ACPI
	 * devices for both application and bootloader modes, but we are
	 * interested in application mode only (if device is in bootloader
	 * mode we'll end up switching into application anyway). So far
	 * application mode addresses were all above 0x40, so we'll use it
	 * as a threshold.
	 */
	if (client->addr < 0x40)
		return ERR_PTR(-ENXIO);

	adev = ACPI_COMPANION(&client->dev);
	if (!adev)
		return ERR_PTR(-ENOENT);

	system_id = dmi_first_match(mxt_dmi_table);
	if (!system_id)
		return ERR_PTR(-ENOENT);

	acpi_pdata = system_id->driver_data;
	if (!acpi_pdata)
		return ERR_PTR(-ENOENT);

	while (acpi_pdata->hid) {
		if (!strcmp(acpi_device_hid(adev), acpi_pdata->hid))
			return &acpi_pdata->pdata;

		acpi_pdata++;
	}

	return ERR_PTR(-ENOENT);
}
#else
static const struct mxt_platform_data *mxt_parse_acpi(struct i2c_client *client)
{
	return ERR_PTR(-ENOENT);
}
#endif

static struct mxt_platform_data *mxt_default_pdata(struct i2c_client *client)
{
	struct mxt_platform_data *pdata;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	/* Set default parameters */
	pdata->irqflags = IRQF_TRIGGER_FALLING;

	return pdata;
}

static const struct mxt_platform_data *
mxt_get_platform_data(struct i2c_client *client)
{
	const struct mxt_platform_data *pdata;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	pdata = dev_get_platdata(&client->dev);
	if (pdata)
		return pdata;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	pdata = mxt_parse_dt(client);
	if (!IS_ERR(pdata) || PTR_ERR(pdata) != -ENOENT)
		return pdata;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	pdata = mxt_parse_acpi(client);
	if (!IS_ERR(pdata) || PTR_ERR(pdata) != -ENOENT)
		return pdata;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	pdata = mxt_default_pdata(client);
	if (!IS_ERR(pdata))
		return pdata;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	dev_err(&client->dev, "No platform data specified\n");
	return ERR_PTR(-EINVAL);
}

static int mxt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mxt_data *data;
	const struct mxt_platform_data *pdata;
	int error;

	tpd_load_status = 0;
	global_fw_version = 0;
	global_fw_build = 0;
	global_cfg_version = 0;
	global_cfg_build = 0;

	global_pen_present = false;
	global_pen_capacity = 0;
	global_pen_capacity_keep = 2;
	global_zstylus_enable = true;
	global_zstylus_activation = true;
	global_zstylus_only_mode = false;
	global_zstylus_only_pen_exist = false;
	global_zstylus_only_mode_delay = 1;

	pr_info("[ATMEL] enter Z500M driver function %s\n", __func__);
	pr_info("[ATMEL] enter %s\n", __func__);

	maxtouch_wq = create_singlethread_workqueue("maxtouch_wq");
	if (!maxtouch_wq) {
		dev_err(&client->dev, "create_singlethread_workqueue error\n");
		return -ENOMEM;dev_err(&client->dev, "create_singlethread_workqueue error\n");
		return -ENOMEM;
	}

	error = of_get_pcbid_data();
	if (error)
		pr_err("[ATMEL] get pcbid failed, use default setting\n");

	/*if (pcbid_prj_id != PRJ_ID_Z500M) {
		pr_info("[ATMEL] It's not Z500M, driver probe aborted\n");
		return 0;
	}*/

	pdata = mxt_get_platform_data(client);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);

	/*get tpd regulator*/
	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	//tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	//error = regulator_set_voltage(tpd->reg, 3300000, 3300000);
	//if (error != 0) {
	//	pr_info("Failed to set reg-vgp6 voltage: %d\n", error);
	//	return -1;
	//}

	tpd->io_reg = regulator_get(tpd->tpd_dev, "vtouchio");
	error = regulator_set_voltage(tpd->io_reg, 1800000, 1800000);
	if (error != 0) {
		pr_info("Failed to set reg-vgp4 voltage: %d\n", error);
		return -1;
	}

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);

	data->client = client;
	data->pdata = pdata;
	i2c_set_clientdata(client, data);
	global_mxt_data_pt = data;

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	if (data->pdata->cfg_name)
		mxt_update_file_name(&data->client->dev,
				     &data->cfg_name,
				     data->pdata->cfg_name,
				     strlen(data->pdata->cfg_name));

	init_completion(&data->chg_completion);
	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);
	mutex_init(&data->debug_msg_lock);
	//++++++ add by angel ++++++
	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "atmel_wake_lock");
	INIT_WORK(&data->work_upgrade, mxt_force_updating);

	pr_info("[ATMEL] rst_gpio = %d, int_gpio = %d, power_gpio = %d\n",
			(int)data->pdata->gpio_reset,
			(int)data->pdata->gpio_int,
			(int)data->pdata->gpio_power);
	pr_info("[ATMEL] parse irq = %d, client irq = %d, slave addr = 0x%x\n",
			data->pdata->touch_irq, data->client->irq,
			client->addr);

	pr_info("[ATMEL] %s-%d, suspend_mode = %d\n",
			__func__, __LINE__, pdata->suspend_mode);
	if (pdata->suspend_mode == MXT_SUSPEND_REGULATOR) {
		pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
		error = mxt_acquire_irq(data);
		if (error)
			goto err_free_mem;

		error = mxt_probe_regulators(data);
		if (error)
			goto err_free_irq;

		disable_irq(data->irq);
	}

	error = mxt_gpio_init(data);
	if (error) {
		pr_info("[ATMEL] gpio init failed\n");
		goto err_free_mem;
	}

	error = mxt_power_on(data);
	if (error) {
		pr_info("[ATMEL] power on failed\n");
		goto err_free_mem;
	}

	pr_info("[ATMEL] %s-%d\n", __func__, __LINE__);
	error = mxt_initialize(data);
	//++++++ add by angel ++++++
	if (error == 2)
		queue_work(maxtouch_wq, &data->work_upgrade);

	error = sysfs_create_group(&client->dev.kobj, &mxt_fw_attr_group);
	if (error) {
		dev_err(&client->dev, "Failure %d creating fw sysfs group\n",
			error);
		return error;
	}

	data->sysfs_enable_irq = true;

	/* set default GloveMode ctrl*/
	data->t78_master_ctrl = false;
	error = mxt_write_reg(data->client, data->T78_address, data->t78_ctrl_disable);
	/* set default ZenMotion ctrl*/
	data->t93_master_ctrl = true;
	data->gesture_wakeup = true;
	data->t92_gesture_sel = 0x7F;
	pr_info("[ATMEL] T92, T93 probe close\n");
	error = mxt_write_reg(data->client, data->T92_address,
					data->t92_ctrl_disable);
	if (error != 0)
		pr_info("[ATMEL TOUCH]%s: T92 Config write error, error=%d\n",
							__func__, error);

	error = mxt_write_reg(data->client, data->T93_address,
					data->t93_ctrl_disable);
	if (error != 0)
		pr_info("[ATMEL TOUCH]%s: T93 Config write error, error=%d\n",
							__func__, error);
	/* set default lid*/
	data->lid_closed = false;

	error = power_supply_register(&client->dev, pen_bat_supply);
	if (error)
		dev_info(&client->dev, "pen power supply register failed!\n");

	g_data_p = data;
	tpd_load_status = 1;
	data->mxt_wq = create_singlethread_workqueue("atmel_mxt_wq");
	if (!data->mxt_wq) {
		dev_err(&client->dev, "create workqueue failed!\n");
		return -ENOMEM;
	}

	data->touch_sdev.name = "touch";
	data->touch_sdev.print_name = atmel_touch_switch_name;
	if(switch_dev_register(&data->touch_sdev) < 0)
		dev_info(&client->dev,  "switch_dev_register for touch failed!\n");

	INIT_DELAYED_WORK(&data->report_delay_work, mxt_report_delay_work);
	destroy_workqueue(maxtouch_wq);
	maxtouch_wq = NULL;

#if defined(CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		data->t93_master_ctrl = false;
		data->gesture_wakeup = false;
		data->t92_gesture_sel = 0x00;
		//pr_info("[ATMEL] It's on COS mode, driver probe aborted\n");
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
		pr_info("[ATMEL]:Touch DEEPSLEEP ...\n");
		//return 0;
	}
#endif

	pr_info("[ATMEL] probe function done\n");
	return 0;

err_free_irq:
	if (data->irq)
		free_irq(data->irq, data);
	destroy_workqueue(maxtouch_wq);
	maxtouch_wq = NULL;
err_free_mem:
	kfree(data);
	return error;
}

static int mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &mxt_fw_attr_group);
	mxt_sysfs_remove(data);

	if (data->irq)
		free_irq(data->irq, data);

	regulator_put(data->reg_avdd);
	regulator_put(data->reg_vdd);
	mxt_free_input_device(data);
	mxt_free_object_table(data);
	kfree(data);

	return 0;
}

static void mxt_suspend(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct mxt_data *data = i2c_get_clientdata(client);
	struct mxt_data *data = g_data_p;
	struct input_dev *input_dev = data->input_dev;

	pr_info("[ATMEL] enter suspend\n");
	if (!input_dev)
		return;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_stop(data);

	mutex_unlock(&input_dev->mutex);

	pr_info("[ATMEL] suspend done\n");
	return;
}

static void mxt_resume(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct mxt_data *data = i2c_get_clientdata(client);
	struct mxt_data *data = g_data_p;
	struct input_dev *input_dev = data->input_dev;

	pr_info("[ATMEL] enter resume\n");
	if (!input_dev)
		return;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_start(data);

	mutex_unlock(&input_dev->mutex);

	pr_info("[ATMEL] resume done\n");
	return;
}

//static SIMPLE_DEV_PM_OPS(mxt_pm_ops, mxt_suspend, mxt_resume);

static const struct of_device_id mxt_of_match[] = {
	{ .compatible = "mtk_tpd,atmel_mxt_ts", },
	{},
};
MODULE_DEVICE_TABLE(of, mxt_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id mxt_acpi_id[] = {
	{ "ATML0000", 0 },	/* Touchpad */
	{ "ATML0001", 0 },	/* Touchscreen */
	{ }
};
MODULE_DEVICE_TABLE(acpi, mxt_acpi_id);
#endif

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "atmel_mxt_tp", 0 },
	{ "mXT224", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mxt_of_match),
		.acpi_match_table = ACPI_PTR(mxt_acpi_id),
		//.pm	= &mxt_pm_ops,
	},
	.probe		= mxt_probe,
	.remove		= mxt_remove,
	.id_table	= mxt_id,
};

static int tpd_local_init(void)
{
	pr_info("[ATMEL] I2C Driver init.\n");

	if (i2c_add_driver(&mxt_driver) != 0) {
		pr_info("[ATMEL] unable to add i2c driver.\n");
		return -1;
	}

	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID,
				0, (TINNO_TOUCH_TRACK_IDS-1), 0, 0);

	tpd_type_cap = 1;

	pr_info("[ATMEL] I2C driver init finished\n");
	return 0;
}

static struct tpd_driver_t tpd_device_driver =
{
	.tpd_device_name = "atmel_mxt_ts",
	.tpd_local_init = tpd_local_init,
	.suspend = mxt_suspend,
	.resume = mxt_resume,
	.tpd_have_button = 0,
};

static int __init tpd_driver_init(void)
{
	pr_info("[ATMEL] MediaTek atmel touch driver init\n");

	if (tpd_driver_add(&tpd_device_driver) < 0)
		pr_info("[Focal] MediaTek add atmel driver failed\n");

	return 0;
}

static void __exit tpd_driver_exit(void)
{
	pr_info("[ATMEL] MediaTek atmel touch driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
//module_i2c_driver(mxt_driver);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
