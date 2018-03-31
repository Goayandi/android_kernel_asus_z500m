#define pr_fmt(fmt) "mtkts_charger %s: " fmt, __func__
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "mach/mt_thermal.h"

static int g_RAP_pull_up_R = 39000;	/* 39K,pull up resister */
static int g_TAP_over_critical_low =  199970;	/* base on 10K NTC temp default value -40 deg */
static int g_RAP_pull_up_voltage = 1800;	/* 1.8V ,pull up voltage */
#define AUX_IN1_NTC (1)

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);
extern int IMM_IsAdcInitReady(void);

typedef struct {
	int CHARGER_Temp;
	int TemperatureR;
} CHARGER_TEMPERATURE;

/* NCP15WF104F03RC(10K) */
CHARGER_TEMPERATURE CHARGER_Temperature_Table[] = {
	{-40, 199970},
	{-35, 151840},
	{-30, 116250},
	{-25, 89611},
	{-20, 69593},
	{-15, 54493},
	{-10, 43029},
	{-5, 34254},
	{0, 27473},
	{5, 22184},
	{10, 18025},
	{15, 14731},
	{20, 12105},
	{25, 10000},		/* 10K */
	{30, 8305},
	{35, 6934},
	{40, 5819},
	{45, 4908},
	{50, 4160},
	{55, 3543},
	{60, 3031},
	{65, 2604},
	{70, 2246},
	{75, 1944},
	{80, 1689},
	{85, 1472},
	{90, 1287},
	{95, 1128},
	{100, 993},
	{105, 876},
	{110, 776},
	{115, 689},
	{120, 614},
	{125, 550}
};

/* convert register to temperature  */
static int mtkts_charger_thermistor_conver_temp(int Res)
{
	int i = 0;
	int asize = 0;
	int RES1 = 0, RES2 = 0;
	int TAP_Value = -200, TMP1 = 0, TMP2 = 0;

	asize = (sizeof(CHARGER_Temperature_Table) / sizeof(CHARGER_TEMPERATURE));
	if (Res >= CHARGER_Temperature_Table[0].TemperatureR) {
		TAP_Value = -40;	/* min */
	} else if (Res <= CHARGER_Temperature_Table[asize - 1].TemperatureR) {
		TAP_Value = 125;	/* max */
	} else {
		RES1 = CHARGER_Temperature_Table[0].TemperatureR;
		TMP1 = CHARGER_Temperature_Table[0].CHARGER_Temp;

		for (i = 0; i < asize; i++) {
			if (Res >= CHARGER_Temperature_Table[i].TemperatureR) {
				RES2 = CHARGER_Temperature_Table[i].TemperatureR;
				TMP2 = CHARGER_Temperature_Table[i].CHARGER_Temp;
				break;
			}
			RES1 = CHARGER_Temperature_Table[i].TemperatureR;
			TMP1 = CHARGER_Temperature_Table[i].CHARGER_Temp;
		}

		TAP_Value = (((Res - RES2) * TMP1) + ((RES1 - Res) * TMP2)) / (RES1 - RES2);
	}
	return TAP_Value;
}

/* convert ADC_temp_volt to register */
/*Volt to Temp formula same with 6589*/
static int mtk_ts_charger_volt_to_temp(unsigned int dwVolt)
{
	int TRes;
	int dwVCriAP = 0;
	int CHARGER_TMP = -100;

	/* SW workaround----------------------------------------------------- */
	/* dwVCriAP = (TAP_OVER_CRITICAL_LOW * 1800) / (TAP_OVER_CRITICAL_LOW + 39000); */
	/* dwVCriAP = (TAP_OVER_CRITICAL_LOW * RAP_PULL_UP_VOLT) / (TAP_OVER_CRITICAL_LOW + RAP_PULL_UP_R); */
	dwVCriAP =
	    (g_TAP_over_critical_low * g_RAP_pull_up_voltage) / (g_TAP_over_critical_low +
								 g_RAP_pull_up_R);

	if (dwVolt > dwVCriAP) {
		TRes = g_TAP_over_critical_low;
	} else {
		/* TRes = (39000*dwVolt) / (1800-dwVolt); */
		/* TRes = (RAP_PULL_UP_R*dwVolt) / (RAP_PULL_UP_VOLT-dwVolt); */
		TRes = (g_RAP_pull_up_R * dwVolt) / (g_RAP_pull_up_voltage - dwVolt);
	}
	/* ------------------------------------------------------------------ */

	/* convert register to temperature */
	CHARGER_TMP = mtkts_charger_thermistor_conver_temp(TRes);

	return CHARGER_TMP;
}

static int get_hw_charger_temp(void)
{

	int ret = 0, data[4], i, ret_value = 0, ret_temp = 0, output;
	int times = 1, Channel = AUX_IN1_NTC;	/* (AUX_IN1_NTC) */
	static int valid_temp;

	if (IMM_IsAdcInitReady() == 0) {
		pr_debug("AUXADC is not ready\n");
		return 0;
	}

	i = times;
	while (i--) {
		ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
		if (ret_value == -1) {/* AUXADC is busy */
			ret_temp = valid_temp;
		} else {
			valid_temp = ret_temp;
		}
		ret += ret_temp;
		pr_warn("ret_temp=%d\n", ret_temp);
	}

	/* Mt_auxadc_hal.c */
	/* #define VOLTAGE_FULL_RANGE  1500 // VA voltage */
	/* #define AUXADC_PRECISE      4096 // 12 bits */
	ret = ret * 1500 / 4096;
	/* ret = ret*1800/4096;//82's ADC power */
	pr_warn("APtery output mV = %d\n", ret);
	output = mtk_ts_charger_volt_to_temp(ret);
	pr_warn("CHARGER nearby temperature = %d\n", output);
	return output;
}

static DEFINE_MUTEX(CHARGER_lock);
int mtkts_charger_get_hw_temp(void)
{
	int t_ret = 0;
	mutex_lock(&CHARGER_lock);

	t_ret = get_hw_charger_temp();
	mutex_unlock(&CHARGER_lock);

	if (t_ret > 40)	/* abnormal high temp */
		pr_debug("temperature=%d\n", t_ret);

	pr_warn("temperature %d\n", t_ret);
	return t_ret;
}

int mtkts_charger_get_temp(unsigned long *t)
{
	*t = mtkts_charger_get_hw_temp();
	return 0;
}
EXPORT_SYMBOL(mtkts_charger_get_temp);

static int __init mtkts_charger_init(void)
{
	pr_warn("\n");
	return 0;
}

static void __exit mtkts_charger_exit(void)
{
	pr_warn("\n");
}

module_init(mtkts_charger_init);
module_exit(mtkts_charger_exit);
