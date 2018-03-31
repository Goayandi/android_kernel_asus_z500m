#define pr_fmt(fmt) "asustek_battery_id %s: " fmt, __func__
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "mt_auxadc.h"

#define AUX_IN2_NTC (12)

static DEFINE_MUTEX(ACCESS_lock);

static int get_hw_battery_id_pin(void)
{

	int ret = 0, data[4], i, ret_value = 0, ret_id = 0, output;
	int times = 1, Channel = AUX_IN2_NTC;	/* (AUX_IN2_NTC) */
	static int valid_id;

	mutex_lock(&ACCESS_lock);
	if (IMM_IsAdcInitReady() == 0) {
		pr_err("AUXADC is not ready\n");
		mutex_unlock(&ACCESS_lock);
		return 0;
	}

	i = times;
	while (i--) {
		ret_value = IMM_GetOneChannelValue(Channel, data, &ret_id);
		if (ret_value == -1) {/* AUXADC is busy */
			ret_id = valid_id;
		} else {
			valid_id = ret_id;
		}
		ret += ret_id;
		pr_debug("ret_id=%d\n", ret_id);
	}

	/* Mt_auxadc_hal.c */
	/* #define VOLTAGE_FULL_RANGE  1500 // VA voltage */
	/* #define AUXADC_PRECISE      4096 // 12 bits */
	ret = ret * 1500 / 4096;
	/* ret = ret*1800/4096;//82's ADC power */
	pr_info("APtery output mV = %d\n", ret);
	if (ret < 380)
		output = 0;
	else if (ret >= 380 && ret < 750)
		output = 1;
	else
		output = 2;
	pr_info("Battery ID: %d\n", output);
	mutex_unlock(&ACCESS_lock);
	return output;
}

int asustek_get_battery_id(unsigned int *t)
{
	*t = get_hw_battery_id_pin();
	return 0;
}
EXPORT_SYMBOL(asustek_get_battery_id);

static int __init asustek_battery_id_init(void)
{
	pr_notice("\n");
	return 0;
}

static void __exit asustek_battery_id_exit(void)
{
	pr_notice("\n");
}

module_init(asustek_battery_id_init);
module_exit(asustek_battery_id_exit);
