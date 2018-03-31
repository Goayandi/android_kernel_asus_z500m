
#include "anx_ohio_public_interface.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include "mt_charging.h"
#include "mt_battery_common.h"


union doDataObject_t {
	u32 object;
	u16 word[2];
	u8 byte[4];
	struct {
		unsigned:30;
		unsigned SupplyType:2;
	} PDO;
	struct {
		unsigned MaxCurrent:10;
		unsigned Voltage:10;
		unsigned PeakCurrent:2;
		unsigned:3;
		unsigned DataRoleSwap:1;
		unsigned USBCommCapable:1;
		unsigned ExternallyPowered:1;
		unsigned USBSuspendSupport:1;
		unsigned DualRolePower:1;
		unsigned SupplyType:2;
	} FPDOSupply;
	struct {
		unsigned OperationalCurrent:10;
		unsigned Voltage:10;
		unsigned:5;
		unsigned DataRoleSwap:1;
		unsigned USBCommCapable:1;
		unsigned ExternallyPowered:1;
		unsigned HigherCapability:1;
		unsigned DualRolePower:1;
		unsigned SupplyType:2;
	} FPDOSink;
	struct {
		unsigned MaxCurrent:10;
		unsigned MinVoltage:10;
		unsigned MaxVoltage:10;
		unsigned SupplyType:2;
	} VPDO;
	struct {
		unsigned MaxPower:10;
		unsigned MinVoltage:10;
		unsigned MaxVoltage:10;
		unsigned SupplyType:2;
	} BPDO;
	struct {
		unsigned MinMaxCurrent:10;
		unsigned OpCurrent:10;
		unsigned:4;
		unsigned NoUSBSuspend:1;
		unsigned USBCommCapable:1;
		unsigned CapabilityMismatch:1;
		unsigned GiveBack:1;
		unsigned ObjectPosition:3;
		unsigned:1;
	} FVRDO;
	struct {
		unsigned MinMaxPower:10;
		unsigned OpPower:10;
		unsigned:4;
		unsigned NoUSBSuspend:1;
		unsigned USBCommCapable:1;
		unsigned CapabilityMismatch:1;
		unsigned GiveBack:1;
		unsigned ObjectPosition:3;
		unsigned:1;
	} BRDO;
	struct {
		unsigned VendorDefined:15;
		unsigned VDMType:1;
		unsigned VendorID:16;
	} UVDM;
	struct {
		unsigned Command:5;
		unsigned:1;
		unsigned CommandType:2;
		unsigned ObjectPosition:3;
		unsigned:2;
		unsigned Version:2;
		unsigned VDMType:1;
		unsigned VendorID:16;
	} SVDM;
};

struct mt_pd_policy {
	u32 SinkRequestMaxVoltage;
	u32 SinkRequestMaxPower;
	u32 SinkRequestOpPower;
	bool SinkGotoMinCompatible;
	bool SinkUSBSuspendOperation;
	bool SinkUSBCommCapable;
	u8 CapSinkNum;
	u8 CapSourceNum;
	union doDataObject_t CapsSink[7];
	union doDataObject_t CapsSource[7];
	union doDataObject_t SinkRequest;
};

static struct mt_pd_policy device_policy;
static pd_callback_t pd_callback_array[256] = { 0 };

/*
	0: sink
	1: source
	2: swap standy 0 to 1
	3: swap standy 1 to 0
*/
static u8 pd_power_role;

static void dump_fixed_supply_pdo(union doDataObject_t *pdo)
{
	pr_debug("Fixed Supply: max current(%d) peak current(%d) voltage(%d)\n",
		 pdo->FPDOSupply.MaxCurrent * 10, pdo->FPDOSupply.PeakCurrent,
		 pdo->FPDOSupply.Voltage * 50);
}

static void dump_variable_supply_pdo(union doDataObject_t *pdo)
{
	pr_debug("Variable Supply: max current(%d) max vol(%d) min vol(%d)\n",
		 pdo->VPDO.MaxCurrent * 10, pdo->VPDO.MaxVoltage * 50, pdo->VPDO.MinVoltage * 50);
}

static void dump_battery_supply_pdo(union doDataObject_t *pdo)
{
	pr_debug("Battery Supply: max vol(%d) min vol(%d) max power(%d)\n",
		 pdo->BPDO.MaxVoltage * 50, pdo->BPDO.MinVoltage * 50, pdo->BPDO.MaxPower * 250);
}

static u8 recv_pd_source_caps_callback(void *para, u8 para_len)
{
	u32 i;
	u32 pdo_count = para_len / 4;
	union doDataObject_t *pdo;
	u32 reqPos = 0, SelVoltage = 0, ReqCurrent = 0, MaxPower = 0;
	u32 objVoltage, objCurrent, objPower;

	pdo = (union doDataObject_t *)para;

	for (i = 0; i < pdo_count; i++) {

		switch (pdo->PDO.SupplyType) {
		case 0x0:
			dump_fixed_supply_pdo(pdo);
			objVoltage = pdo->FPDOSupply.Voltage;
			if (objVoltage > device_policy.SinkRequestMaxVoltage)
				objPower = 0;
			else {
				objCurrent = pdo->FPDOSupply.MaxCurrent;
				objPower = objVoltage * objCurrent;
			}
			break;
		case 0x1:
			dump_battery_supply_pdo(pdo);
			objPower = 0;
			break;
		case 0x2:
			dump_variable_supply_pdo(pdo);
			objVoltage = pdo->VPDO.MaxVoltage;
			if (objVoltage > device_policy.SinkRequestMaxVoltage)
				objPower = 0;
			else {
				objVoltage = pdo->VPDO.MinVoltage;
				objCurrent = pdo->VPDO.MaxCurrent;
				objPower = objVoltage * objCurrent;
			}
			break;
		default:
			objPower = 0;
			pr_warn("%s:unknown supply type!\n", __func__);
			break;
		}

		if (objPower > MaxPower) {
			MaxPower = objPower;
			SelVoltage = objVoltage;
			reqPos = i + 1;
		}
		pdo++;
	}

	/* pr_debug("reqPos=%d, selVoltage=%d\n", reqPos, SelVoltage); */
	if ((reqPos > 0) && (SelVoltage > 0)) {
		device_policy.SinkRequest.FVRDO.ObjectPosition = reqPos & 0x07;
		device_policy.SinkRequest.FVRDO.GiveBack = device_policy.SinkGotoMinCompatible;
		device_policy.SinkRequest.FVRDO.NoUSBSuspend =
		    device_policy.SinkUSBSuspendOperation;
		device_policy.SinkRequest.FVRDO.USBCommCapable = device_policy.SinkUSBCommCapable;
		ReqCurrent = MaxPower / SelVoltage;
		device_policy.SinkRequest.FVRDO.OpCurrent = (ReqCurrent & 0x3FF);
		device_policy.SinkRequest.FVRDO.MinMaxCurrent = (ReqCurrent & 0x3FF);

		if (device_policy.SinkGotoMinCompatible)
			device_policy.SinkRequest.FVRDO.CapabilityMismatch = false;
		else {
			if (MaxPower / 2 /*mW */  < device_policy.SinkRequestMaxPower)
				device_policy.SinkRequest.FVRDO.CapabilityMismatch = true;
			else
				device_policy.SinkRequest.FVRDO.CapabilityMismatch = false;
		}

		send_pd_msg(TYPE_PWR_OBJ_REQ, (const char *)&device_policy.SinkRequest, 4);

		/* TODO: anx7418 should callback ACCEPT to let AP know high voltage charging supported */
		if (SelVoltage == 100)
			bat_update_charger_type(TYPEC_PD_5V_CHARGER);

		if (SelVoltage >= 240)
			bat_update_charger_type(TYPEC_PD_9V_CHARGER);

	} else
		pr_err("No available PDO from source caps!\n");

	return 1;
}

/* unit: mA */
int mt_usb_pd_get_current(void)
{
	return device_policy.SinkRequest.FVRDO.OpCurrent * 10;
}

static void init_pd_callback(void)
{
	u32 i;

	pd_callback_array[TYPE_PWR_SRC_CAP] = recv_pd_source_caps_callback;

	for (i = 0; i < ARRAY_SIZE(pd_callback_array); i++) {
		if (pd_callback_array[i])
			register_pd_msg_callback_func(i, pd_callback_array[i]);
	}
}

void mt_init_pd_policy(void)
{
	init_pd_callback();

	device_policy.SinkRequestMaxVoltage = 240;	/* 12V. unit:50mV */
	device_policy.SinkRequestMaxPower = 25000;	/* 25000mW */
	device_policy.SinkRequestOpPower = 25000;	/* 25000mW */
	device_policy.SinkGotoMinCompatible = false;
	device_policy.SinkUSBCommCapable = false;
	device_policy.SinkUSBSuspendOperation = false;

	device_policy.CapSinkNum = 2;
	device_policy.CapsSink[0].FPDOSink.Voltage = 100;	/* 5V */
	device_policy.CapsSink[0].FPDOSink.OperationalCurrent = 100;	/* 1A */
	device_policy.CapsSink[0].FPDOSink.DataRoleSwap = 0;
	device_policy.CapsSink[0].FPDOSink.USBCommCapable = 1;
	device_policy.CapsSink[0].FPDOSink.ExternallyPowered = 0;
	device_policy.CapsSink[0].FPDOSink.HigherCapability = 0;
	device_policy.CapsSink[0].FPDOSink.DualRolePower = 1;

	device_policy.CapsSink[1].FPDOSink.Voltage = 240;	/* 12V */
	device_policy.CapsSink[1].FPDOSink.OperationalCurrent = 100;	/* 1A */
	device_policy.CapsSink[1].FPDOSink.DataRoleSwap = 0;
	device_policy.CapsSink[1].FPDOSink.USBCommCapable = 1;
	device_policy.CapsSink[1].FPDOSink.ExternallyPowered = 0;
	device_policy.CapsSink[1].FPDOSink.HigherCapability = 0;
	device_policy.CapsSink[1].FPDOSink.DualRolePower = 1;

	device_policy.CapSourceNum = 2;

	device_policy.CapsSource[0].FPDOSupply.Voltage = 100;	/* 5V */
	device_policy.CapsSource[0].FPDOSupply.MaxCurrent = 100;	/* 1A */
	device_policy.CapsSource[0].FPDOSupply.PeakCurrent = 0;
	device_policy.CapsSource[0].FPDOSupply.DataRoleSwap = 0;
	device_policy.CapsSource[0].FPDOSupply.USBCommCapable = 1;
	device_policy.CapsSource[0].FPDOSupply.ExternallyPowered = 0;
	device_policy.CapsSource[0].FPDOSupply.USBSuspendSupport = 0;
	device_policy.CapsSource[0].FPDOSupply.DualRolePower = 1;
	device_policy.CapsSource[0].FPDOSupply.SupplyType = 0;

	device_policy.CapsSource[1].FPDOSupply.Voltage = 100;	/* 5V */
	device_policy.CapsSource[1].FPDOSupply.MaxCurrent = 150;	/* 1.5A */
	device_policy.CapsSource[1].FPDOSupply.PeakCurrent = 0;
	device_policy.CapsSource[1].FPDOSupply.DataRoleSwap = 0;
	device_policy.CapsSource[1].FPDOSupply.USBCommCapable = 1;
	device_policy.CapsSource[1].FPDOSupply.ExternallyPowered = 0;
	device_policy.CapsSource[1].FPDOSupply.USBSuspendSupport = 0;
	device_policy.CapsSource[1].FPDOSupply.DualRolePower = 1;
	device_policy.CapsSource[1].FPDOSupply.SupplyType = 0;

}

void mt_init_sink_caps(u8 *caps_num, u8 *caps_buf)
{
	u8 i;
	union doDataObject_t *pdo = (union doDataObject_t *)caps_buf;

	for (i = 0; i < device_policy.CapSinkNum; i++, pdo++)
		*pdo = device_policy.CapsSink[i];
	*caps_num = device_policy.CapSinkNum;
}

void mt_init_src_caps(u8 *caps_num, u8 *caps_buf)
{
	u8 i;
	union doDataObject_t *pdo = (union doDataObject_t *)caps_buf;

	for (i = 0; i < device_policy.CapSourceNum; i++, pdo++)
		*pdo = device_policy.CapsSource[i];
	*caps_num = device_policy.CapSourceNum;
}

bool mt_usb_pd_support(void)
{
	return true;
}

bool mt_is_power_sink(void)
{
	if (pd_power_role == 0)
		return true;
	else
		return false;
}

void mt_update_power_role(u8 role)
{
	pd_power_role = role;
}

/*
	0: sink
	1: source
	2: swap standy 0 to 1
	3: swap standy 1 to 0
*/
void mt_do_power_role_swap(void)
{
	if (pd_power_role == 0) {

		pd_power_role = 2;
		wake_up_bat();
	} else if (pd_power_role == 1) {

		pd_power_role = 3;
		bat_charger_boost_enable(false);
	} else {
		pr_warn("%s: unexpected pd flow!\n", __func__);
	}
}

void mt_power_role_swap_ready(void)
{
	if (pd_power_role == 2) {

		/* from sink to source */
		pd_power_role = 1;
		bat_charger_boost_enable(true);

	} else if (pd_power_role == 3) {

		/* from source to sink */
		pd_power_role = 0;
		wake_up_bat();

	} else {
		pr_warn("%s: unexpected pd flow!\n", __func__);
	}
}
