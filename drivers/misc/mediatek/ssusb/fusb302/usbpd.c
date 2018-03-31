 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  *
  * Software License Agreement:
  *
  * The software supplied herewith by Fairchild Semiconductor (the Company)
  * is supplied to you, the Company's customer, for exclusive use with its
  * USB Type C / USB PD products.  The software is owned by the Company and/or
  * its supplier, and is protected under applicable copyright laws.
  * All rights are reserved. Any use in violation of the foregoing restrictions
  * may subject the user to criminal sanctions under applicable laws, as well
  * as to civil liability for the breach of the terms and conditions of this
  * license.
  *
  * THIS SOFTWARE IS PROVIDED IN AN AS IS CONDITION. NO WARRANTIES,
  * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
  * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
  * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
  * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
  *
  *****************************************************************************/
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
/* #include <mach/mt_pm_ldo.h> */
#include <linux/interrupt.h>
#include <linux/time.h>
/* #include <cust_eint.h> */
/* #include <mach/eint.h> */
/* #include <cust_eint.h> */
#include <linux/kthread.h>
/* #include <mach/mt_gpio.h> */

#include "usbpd.h"
#include "fusb302.h"

/* for mtk battery driver */
#include "../../../power/mt81xx/mt_charging.h"

#define FUSB302_DEBUG
#define PD_CHARGING_DRIVER_SUPPORT

#ifdef FUSB302_DEBUG
#define FUSB_LOG(fmt, args...)  pr_info("[fusbpd]" fmt, ##args)
#else
#define FUSB_LOG(fmt, args...)
#endif

#define FUSB_MS_TO_NS(x) (x * 1000 * 1000)
#define Delay10us(x) udelay(x*10)

struct fusb_pd_ctx *fusb_pd_ctx;

#ifndef PD_CHARGING_DRIVER_SUPPORT
void smb1351_suspend_enable(bool enable) {};
bool smb1351_is_battery_low(void) {return 0; };
#endif

void set_policy_state(enum PolicyState_t st)
{
	fusb_pd_ctx->PolicyState = st;
	fusb_i2c_data->state_changed = true;
	FUSB_LOG("set PolicyState=%d\n", st);
}

void set_protocol_state(enum ProtocolState_t st)
{
	fusb_pd_ctx->ProtocolState = st;
	fusb_i2c_data->state_changed = true;
	FUSB_LOG("set fusb_pd_ctx->ProtocolState =%d\n", st);
}

void set_pdtx_state(enum PDTxStatus_t st)
{
	fusb_pd_ctx->PDTxStatus = st;
	fusb_i2c_data->state_changed = true;
	FUSB_LOG("set PDTxStatus=%d\n", st);
}

void set_policy_subindex(u8 index)
{
	fusb_pd_ctx->PolicySubIndex = index;
	fusb_i2c_data->state_changed = true;
}

void increase_policy_subindex(void)
{
	fusb_pd_ctx->PolicySubIndex++;
	fusb_i2c_data->state_changed = true;
}

void usbpd_start_timer(struct hrtimer *timer, int ms)
{
	ktime_t ktime;

	ktime = ktime_set(0, FUSB_MS_TO_NS(ms));
	hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
}

enum hrtimer_restart pd_func_hrtimer(struct hrtimer *timer)
{
	if (timer == &fusb_pd_ctx->protocol_hrtimer)
		fusb_pd_ctx->protocol_time_count = 0;
	else if (timer == &fusb_pd_ctx->policystate_hrtimer)
		fusb_pd_ctx->policystate_time_count = 0;
	else if (timer == &fusb_pd_ctx->noresponse_hrtimer)
		fusb_pd_ctx->noresponse_time_count = 0;
	else if (timer == &fusb_pd_ctx->prswap_hrtimer)
		fusb_i2c_data->prswap_time_count = 0;

	FUSB_LOG("%s, protocol_time_count=%d, policystate_time_count=%d\n",
	     __func__, fusb_pd_ctx->protocol_time_count,
	     fusb_pd_ctx->policystate_time_count);
	FUSB_LOG("noresponse_time_count=%d, PRSwapTimer=%d\n",
	     fusb_pd_ctx->noresponse_time_count,
	     fusb_i2c_data->prswap_time_count);

	wake_up_statemachine();

	return HRTIMER_NORESTART;
}

int FUSB300WriteFIFO(unsigned char length, unsigned char *data)
{
	unsigned char llen = length;
	unsigned char i = 0;
	int ret = 0;
	char log[200];

	ret = sprintf(log, ">>>>");
	for (i = 0; i < length; i++)
		ret += sprintf(log + ret, " %2x", data[i]);
	FUSB_LOG("%s\n", log);

	ret = 0;
	i = 0;

	while (llen > 4) {
		ret = FUSB300Write(regFIFO, 4, &data[i << 2]);
		i++;
		llen -= 4;
	}
	if (llen > 0)
		ret = FUSB300Write(regFIFO, llen, &data[i << 2]);
	return ret;
}

int FUSB300ReadFIFO(unsigned char length, unsigned char *data)
{
	unsigned char llen = length;
	unsigned char i = 0;
	int ret = 0;
	char log[200];

	while (llen > 8) {
		ret = FUSB300Read(regFIFO, 8, &data[i << 3]);
		i++;
		llen -= 8;
	}
	if (llen > 0)
		ret = FUSB300Read(regFIFO, llen, &data[i << 3]);

	ret = sprintf(log, "<<<<<");
	for (i = 0; i < length; i++)
		ret += sprintf(log + ret, " %2x", data[i]);
	FUSB_LOG("%s\n", log);

	return ret;
}

int InitializeUSBPDVariables(void)
{
	fusb_pd_ctx = kzalloc(sizeof(struct fusb_pd_ctx), GFP_KERNEL);
	if (!fusb_pd_ctx) {
		FUSB_LOG("FUSB_PD_CTX data alloc fail\n");
		return -ENOMEM;
	}

	fusb_pd_ctx->USBPDBufStart = 0;
	fusb_pd_ctx->USBPDBufEnd = 0;
	fusb_pd_ctx->USBPDBufOverflow = false;
	fusb_pd_ctx->SinkRequestMaxVoltage = 240;
	fusb_pd_ctx->SinkRequestMaxCurrent = 300;
	fusb_pd_ctx->SinkRequestMaxPower = 48000;
	fusb_pd_ctx->SinkRequestOpPower = 48000;
	fusb_pd_ctx->SinkGotoMinCompatible = false;
	fusb_pd_ctx->SinkUSBSuspendOperation = false;
	fusb_pd_ctx->SinkUSBCommCapable = false;
	fusb_pd_ctx->SourceCapsUpdated = false;
	fusb_pd_ctx->CapsHeaderSource.NumDataObjects = 2;
	fusb_pd_ctx->CapsHeaderSource.PortDataRole = 0;
	fusb_pd_ctx->CapsHeaderSource.PortPowerRole = 1;
	fusb_pd_ctx->CapsHeaderSource.SpecRevision = 1;
	fusb_pd_ctx->CapsSource[0].FPDOSupply.Voltage = 100;
	fusb_pd_ctx->CapsSource[0].FPDOSupply.MaxCurrent = 100;
	fusb_pd_ctx->CapsSource[0].FPDOSupply.PeakCurrent = 0;
	fusb_pd_ctx->CapsSource[0].FPDOSupply.DataRoleSwap = true;
	fusb_pd_ctx->CapsSource[0].FPDOSupply.USBCommCapable = false;
	fusb_pd_ctx->CapsSource[0].FPDOSupply.ExternallyPowered = true;
	fusb_pd_ctx->CapsSource[0].FPDOSupply.USBSuspendSupport = false;
	fusb_pd_ctx->CapsSource[0].FPDOSupply.DualRolePower = true;
	fusb_pd_ctx->CapsSource[0].FPDOSupply.SupplyType = 0;

	fusb_pd_ctx->CapsSource[1].FPDOSupply.Voltage = 240;
	fusb_pd_ctx->CapsSource[1].FPDOSupply.MaxCurrent = 150;
	fusb_pd_ctx->CapsSource[1].FPDOSupply.PeakCurrent = 0;
	fusb_pd_ctx->CapsSource[1].FPDOSupply.DataRoleSwap = 0;
	fusb_pd_ctx->CapsSource[1].FPDOSupply.USBCommCapable = 0;
	fusb_pd_ctx->CapsSource[1].FPDOSupply.ExternallyPowered = 0;
	fusb_pd_ctx->CapsSource[1].FPDOSupply.USBSuspendSupport = 0;
	fusb_pd_ctx->CapsSource[1].FPDOSupply.DualRolePower = 0;
	fusb_pd_ctx->CapsSource[1].FPDOSupply.SupplyType = 0;

	fusb_pd_ctx->CapsHeaderSink.NumDataObjects = 2;
	fusb_pd_ctx->CapsHeaderSink.PortDataRole = 0;
	fusb_pd_ctx->CapsHeaderSink.PortPowerRole = 0;
	fusb_pd_ctx->CapsHeaderSink.SpecRevision = 1;
	fusb_pd_ctx->CapsSink[0].FPDOSink.Voltage = 100;
	fusb_pd_ctx->CapsSink[0].FPDOSink.OperationalCurrent = 10;
	fusb_pd_ctx->CapsSink[0].FPDOSink.DataRoleSwap = 0;
	fusb_pd_ctx->CapsSink[0].FPDOSink.USBCommCapable = 0;
	fusb_pd_ctx->CapsSink[0].FPDOSink.ExternallyPowered = 0;
	fusb_pd_ctx->CapsSink[0].FPDOSink.HigherCapability = false;
	fusb_pd_ctx->CapsSink[0].FPDOSink.DualRolePower = 0;

	fusb_pd_ctx->CapsSink[1].FPDOSink.Voltage = 240;
	fusb_pd_ctx->CapsSink[1].FPDOSink.OperationalCurrent = 10;
	fusb_pd_ctx->CapsSink[1].FPDOSink.DataRoleSwap = 0;
	fusb_pd_ctx->CapsSink[1].FPDOSink.USBCommCapable = 0;
	fusb_pd_ctx->CapsSink[1].FPDOSink.ExternallyPowered = 0;
	fusb_pd_ctx->CapsSink[1].FPDOSink.HigherCapability = 0;
	fusb_pd_ctx->CapsSink[1].FPDOSink.DualRolePower = 0;

	fusb_pd_ctx->pdpower_record.usbpdselVoltage = 0;
	fusb_pd_ctx->pdpower_record.usbpdselCurrent = 0;
	fusb_pd_ctx->pdpower_record.usbpdselObjectPosition = 0;
	fusb_pd_ctx->pdpower_record.selVoltage_lowbattery = 0;
	fusb_pd_ctx->pdpower_record.selCurrent_lowbattery = 0;
	fusb_pd_ctx->pdpower_record.ObjectPosition_lowbattery = 0;

	hrtimer_init(&fusb_pd_ctx->protocol_hrtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	fusb_pd_ctx->protocol_hrtimer.function = pd_func_hrtimer;

	hrtimer_init(&fusb_pd_ctx->policystate_hrtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	fusb_pd_ctx->policystate_hrtimer.function = pd_func_hrtimer;

	hrtimer_init(&fusb_pd_ctx->noresponse_hrtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	fusb_pd_ctx->noresponse_hrtimer.function = pd_func_hrtimer;

	hrtimer_init(&fusb_pd_ctx->prswap_hrtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	fusb_pd_ctx->prswap_hrtimer.function = pd_func_hrtimer;

	return 0;
}

void USBPDEnable(bool FUSB300Update, bool TypeCDFP)
{
	u8 data[5];

	if (fusb_i2c_data->USBPDEnabled == true) {
		if (fusb_i2c_data->blnCCPinIsCC1)
			fusb_i2c_data->Registers.Switches.TXCC1 = true;
		else if (fusb_i2c_data->blnCCPinIsCC2)
			fusb_i2c_data->Registers.Switches.TXCC2 = true;
		if (fusb_i2c_data->blnCCPinIsCC1
		    || fusb_i2c_data->blnCCPinIsCC2) {
			fusb_i2c_data->USBPDActive = true;
			ResetProtocolLayer(false);
			fusb_pd_ctx->noresponse_time_count = USHRT_MAX;
			fusb_pd_ctx->PolicyIsSource = TypeCDFP;
			fusb_pd_ctx->PolicyIsDFP = TypeCDFP;
			if (fusb_pd_ctx->PolicyIsSource) {
				set_policy_state(peSourceStartup);
				fusb_i2c_data->Registers.Switches.POWERROLE = 1;
				fusb_i2c_data->Registers.Switches.DATAROLE = 1;
			} else {

				set_policy_state(peSinkStartup);
				fusb_i2c_data->Registers.Switches.POWERROLE = 0;
				fusb_i2c_data->Registers.Switches.DATAROLE = 0;
			}
			fusb_i2c_data->Registers.Switches.AUTO_CRC = 1;
			fusb_i2c_data->Registers.Power.PWR |= 0x08;
			fusb_i2c_data->Registers.Control.AUTO_PRE = 0;
			fusb_i2c_data->Registers.Control.N_RETRIES = 2;
			fusb_i2c_data->Registers.Control.AUTO_RETRY = 1;
			fusb_i2c_data->Registers.Slice.SDAC = SDAC_DEFAULT;
			data[0] = fusb_i2c_data->Registers.Slice.byte;
			data[1] =
			    fusb_i2c_data->Registers.Control.byte[0] | 0x40;
			data[2] =
			    fusb_i2c_data->Registers.Control.byte[1] | 0x04;
			data[3] = fusb_i2c_data->Registers.Control.byte[2];
			data[4] = fusb_i2c_data->Registers.Control.byte[3];
			FUSB300Write(regSlice, 5, &data[0]);
			if (FUSB300Update) {
				FUSB300Write(regPower, 1,
					     &fusb_i2c_data->Registers.
					     Power.byte);
				FUSB300Write(regSwitches1, 1,
					     &fusb_i2c_data->Registers.
					     Switches.byte[1]);
			}
			StoreUSBPDToken(true, pdtAttach);

		}
	}
}

void USBPDDisable(bool FUSB300Update)
{
	if (fusb_i2c_data->USBPDActive == true)
		StoreUSBPDToken(true, pdtDetach);
	fusb_i2c_data->USBPDActive = false;
	set_protocol_state(PRLDisabled);
	set_policy_state(peDisabled);
	set_pdtx_state(txIdle);
	fusb_pd_ctx->PolicyIsSource = false;
	fusb_pd_ctx->PolicyHasContract = false;
	fusb_pd_ctx->SourceCapsUpdated = true;
	if (FUSB300Update) {
		fusb_i2c_data->Registers.Switches.byte[1] = 0;
		fusb_i2c_data->Registers.Power.PWR &= 0x07;
		FUSB300Write(regPower, 1, &fusb_i2c_data->Registers.Power.byte);
		FUSB300Write(regSwitches1, 1,
			     &fusb_i2c_data->Registers.Switches.byte[1]);
	}
}

void USBPDPolicyEngine(void)
{

	switch (fusb_pd_ctx->PolicyState) {
	case peErrorRecovery:
		PolicyErrorRecovery();
		break;
	case peSourceSendHardReset:
		PolicySourceSendHardReset();
		break;
	case peSourceSendSoftReset:
		PolicySourceSendSoftReset();
		break;
	case peSourceSoftReset:
		PolicySourceSoftReset();
		break;
	case peSourceStartup:
		PolicySourceStartup();
		break;
	case peSourceDiscovery:
		PolicySourceDiscovery();
		break;
	case peSourceSendCaps:
		PolicySourceSendCaps();
		break;
	case peSourceDisabled:
		PolicySourceDisabled();
		break;
	case peSourceTransitionDefault:
		PolicySourceTransitionDefault();
		break;
	case peSourceNegotiateCap:
		PolicySourceNegotiateCap();
		break;
	case peSourceCapabilityResponse:
		PolicySourceCapabilityResponse();
		break;
	case peSourceTransitionSupply:
		PolicySourceTransitionSupply();
		break;
	case peSourceReady:
		PolicySourceReady();
		break;
	case peSourceGiveSourceCaps:
		PolicySourceGiveSourceCap();
		break;
	case peSourceGetSinkCaps:
		PolicySourceGetSinkCap();
		break;
	case peSourceSendPing:
		PolicySourceSendPing();
		break;
	case peSourceGotoMin:
		PolicySourceGotoMin();
		break;
	case peSourceGiveSinkCaps:
		PolicySourceGiveSinkCap();
		break;
	case peSourceGetSourceCaps:
		PolicySourceGetSourceCap();
		break;
	case peSourceSendDRSwap:
		PolicySourceSendDRSwap();
		break;
	case peSourceEvaluateDRSwap:
		PolicySourceEvaluateDRSwap();
		break;
	case peSourceSendVCONNSwap:
		PolicySourceSendVCONNSwap();
		break;
	case peSourceSendPRSwap:
		PolicySourceSendPRSwap();
		break;
	case peSourceEvaluatePRSwap:
		PolicySourceEvaluatePRSwap();
		break;
	case peSinkStartup:
		PolicySinkStartup();
		break;
	case peSinkSendHardReset:
		PolicySinkSendHardReset();
		break;
	case peSinkSoftReset:
		PolicySinkSoftReset();
		break;
	case peSinkSendSoftReset:
		PolicySinkSendSoftReset();
		break;
	case peSinkTransitionDefault:
		PolicySinkTransitionDefault();
		break;
	case peSinkDiscovery:
		PolicySinkDiscovery();
		break;
	case peSinkWaitCaps:
		PolicySinkWaitCaps();
		break;
	case peSinkEvaluateCaps:
		PolicySinkEvaluateCaps();
		break;
	case peSinkSelectCapability:
		PolicySinkSelectCapability();
		break;
	case peSinkTransitionSink:
		PolicySinkTransitionSink();
		break;
	case peSinkReady:
		PolicySinkReady();
		break;
	case peSinkGiveSinkCap:
		PolicySinkGiveSinkCap();
		break;
	case peSinkGetSourceCap:
		PolicySinkGetSourceCap();
		break;
	case peSinkGetSinkCap:
		PolicySinkGetSinkCap();
		break;
	case peSinkGiveSourceCap:
		PolicySinkGiveSourceCap();
		break;
	case peSinkSendDRSwap:
		PolicySinkSendDRSwap();
		break;
	case peSinkEvaluateDRSwap:
		PolicySinkEvaluateDRSwap();
		break;
	case peSinkEvaluateVCONNSwap:
		PolicySinkEvaluateVCONNSwap();
		break;
	case peSinkSendPRSwap:
		PolicySinkSendPRSwap();
		break;
	case peSinkEvaluatePRSwap:
		PolicySinkEvaluatePRSwap();
		break;
	default:
		break;
	}
	fusb_pd_ctx->USBPDTxFlag = false;
}

void PolicyErrorRecovery(void)
{
	SetStateErrorRecovery();
}

void PolicySourceSendHardReset(void)
{
	PolicySendHardReset(peSourceTransitionDefault, tPSHardReset);
}

void PolicySourceSoftReset(void)
{
	PolicySendCommand(CMTAccept, peSourceSendCaps, 0);
}

void PolicySourceSendSoftReset(void)
{
	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(CMTSoftReset, peSourceSendSoftReset, 1) ==
		    STAT_SUCCESS) {
			fusb_pd_ctx->policystate_time_count = tSenderResponse;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
		}
		break;
	default:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if ((fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) &&
			    (fusb_pd_ctx->PolicyRxHeader.MessageType ==
			     CMTAccept)) {
				/* And it was the Accept... */
				set_policy_state(peSourceSendCaps);
			} else
				set_policy_state(peSourceSendHardReset);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peSourceSendHardReset);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	}
}

void PolicySourceStartup(void)
{
	fusb_pd_ctx->PolicyIsSource = true;
	ResetProtocolLayer(true);
	fusb_i2c_data->prswap_time_count = 0;
	fusb_pd_ctx->CapsCounter = 0;
	fusb_pd_ctx->CollisionCounter = 0;
	fusb_pd_ctx->policystate_time_count = 0;
	set_policy_state(peSourceSendCaps);
	set_policy_subindex(0);
}

void PolicySourceDiscovery(void)
{
	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		fusb_pd_ctx->policystate_time_count = tTypeCSendSourceCap;
		usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
				  fusb_pd_ctx->policystate_time_count);
		increase_policy_subindex();
		break;
	default:
		if ((fusb_pd_ctx->HardResetCounter > nHardResetCount)
		    && (fusb_pd_ctx->noresponse_time_count == 0)) {
			if (fusb_pd_ctx->PolicyHasContract)
				set_policy_state(peErrorRecovery);
			else
				set_policy_state(peSourceDisabled);
			set_policy_subindex(0);
		} else if (fusb_pd_ctx->policystate_time_count == 0) {
			if (fusb_pd_ctx->CapsCounter > nCapsCount)
				set_policy_state(peSourceDisabled);
			else
				set_policy_state(peSourceSendCaps);
			set_policy_subindex(0);
		}
		break;
	}
}

void PolicySourceSendCaps(void)
{
	if ((fusb_pd_ctx->HardResetCounter > nHardResetCount)
	    && (fusb_pd_ctx->noresponse_time_count == 0)) {
		if (fusb_pd_ctx->PolicyHasContract)
			set_policy_state(peErrorRecovery);
		else
			set_policy_state(peSourceDisabled);
	} else {

		switch (fusb_pd_ctx->PolicySubIndex) {
		case 0:
			if (PolicySendData
			    (DMTSourceCapabilities,
			     fusb_pd_ctx->CapsHeaderSource.NumDataObjects,
			     &fusb_pd_ctx->CapsSource[0], peSourceSendCaps,
			     1) == STAT_SUCCESS) {
				fusb_pd_ctx->HardResetCounter = 0;
				fusb_pd_ctx->CapsCounter = 0;
				fusb_pd_ctx->noresponse_time_count = USHRT_MAX;
				fusb_pd_ctx->policystate_time_count =
				    tSenderResponse;
				usbpd_start_timer
				    (&fusb_pd_ctx->policystate_hrtimer,
				     fusb_pd_ctx->policystate_time_count);
			}
			break;
		default:
			if (fusb_pd_ctx->ProtocolMsgRx) {
				fusb_pd_ctx->ProtocolMsgRx = false;
				if ((fusb_pd_ctx->
				     PolicyRxHeader.NumDataObjects == 1)
				    && (fusb_pd_ctx->
					PolicyRxHeader.MessageType ==
					DMTRequest))
					set_policy_state(peSourceNegotiateCap);
				else
					set_policy_state(peSourceSendHardReset);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
			} else if (!fusb_pd_ctx->policystate_time_count) {
				fusb_pd_ctx->ProtocolMsgRx = false;
				set_policy_state(peSourceSendHardReset);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
			}
			break;
		}
	}
}

void PolicySourceDisabled(void)
{
	fusb_pd_ctx->USBPDContract.object = 0;

}

void PolicySourceTransitionDefault(void)
{
	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		fusb_i2c_data->VBUS_5V_EN = 0;
		fusb_i2c_data->VBUS_12V_EN = 0;
		fusb_pd_ctx->USBPDContract.object = 0;
		fusb_pd_ctx->policystate_time_count = tPSSourceOffNom;
		usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
				  fusb_pd_ctx->policystate_time_count);
		increase_policy_subindex();
		break;
	case 1:
		if (fusb_pd_ctx->policystate_time_count == 0) {
			fusb_i2c_data->VBUS_5V_EN = 1;
			fusb_pd_ctx->policystate_time_count =
			    tTypeCSendSourceCap;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
			increase_policy_subindex();
		}
		break;
	default:
		if (fusb_pd_ctx->policystate_time_count == 0) {
			fusb_pd_ctx->noresponse_time_count = tNoResponse;
			usbpd_start_timer(&fusb_pd_ctx->noresponse_hrtimer,
					  fusb_pd_ctx->noresponse_time_count);
			set_policy_state(peSourceStartup);
			set_policy_subindex(0);
		}
		break;
	}
}

void PolicySourceNegotiateCap(void)
{

	bool reqAccept = false;
	u8 objPosition;

	objPosition = fusb_pd_ctx->PolicyRxDataObj[0].FVRDO.ObjectPosition;
	if ((objPosition > 0)
	    && (objPosition <= fusb_pd_ctx->CapsHeaderSource.NumDataObjects)) {
		if (fusb_pd_ctx->PolicyRxDataObj[0].FVRDO.OpCurrent <=
		    fusb_pd_ctx->CapsSource[objPosition -
					    1].FPDOSupply.MaxCurrent)
			reqAccept = true;
	}
	if (reqAccept)
		set_policy_state(peSourceTransitionSupply);
	else
		set_policy_state(peSourceCapabilityResponse);
}

void PolicySourceTransitionSupply(void)
{
	u8 objPosition;

	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		PolicySendCommand(CMTAccept, peSourceTransitionSupply, 1);
		break;
	case 1:
		fusb_pd_ctx->policystate_time_count = tSnkTransition;
		usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
				  fusb_pd_ctx->policystate_time_count);
		increase_policy_subindex();
		break;
	case 2:
		if (!fusb_pd_ctx->policystate_time_count)
			increase_policy_subindex();
		break;
	case 3:
		fusb_pd_ctx->PolicyHasContract = true;
		fusb_pd_ctx->USBPDContract.object =
		    fusb_pd_ctx->PolicyRxDataObj[0].object;
		objPosition = fusb_pd_ctx->USBPDContract.FVRDO.ObjectPosition;
		if ((fusb_pd_ctx->CapsSource[objPosition - 1].
		     FPDOSupply.SupplyType == 0)
		    && (fusb_pd_ctx->CapsSource[objPosition - 1].
			FPDOSupply.Voltage == 240)) {
			if (fusb_i2c_data->VBUS_12V_EN) {
				set_policy_subindex(5);
			} else {

				fusb_i2c_data->VBUS_5V_EN = 1;
				fusb_i2c_data->VBUS_12V_EN = 1;
				fusb_pd_ctx->policystate_time_count =
				    tFPF2498Transition;
				usbpd_start_timer
				    (&fusb_pd_ctx->policystate_hrtimer,
				     fusb_pd_ctx->policystate_time_count);
				increase_policy_subindex();
			}
		} else {
			if (fusb_i2c_data->VBUS_12V_EN) {
				fusb_i2c_data->VBUS_5V_EN = 1;
				fusb_i2c_data->VBUS_12V_EN = 0;
				fusb_pd_ctx->policystate_time_count =
				    tFPF2498Transition;
				usbpd_start_timer
				    (&fusb_pd_ctx->policystate_hrtimer,
				     fusb_pd_ctx->policystate_time_count);
				increase_policy_subindex();
			} else {
				set_policy_subindex(5);
			}
		}
		break;
	case 4:

		if (fusb_pd_ctx->policystate_time_count == 0)
			increase_policy_subindex();
		break;
	default:
		PolicySendCommand(CMTPS_RDY, peSourceReady, 0);
		break;
	}
}

void PolicySourceCapabilityResponse(void)
{
	if (fusb_pd_ctx->PolicyHasContract)
		PolicySendCommand(CMTReject, peSourceReady, 0);
	else
		PolicySendCommand(CMTReject, peSourceSendHardReset, 0);
}

void PolicySourceReady(void)
{
	if (fusb_pd_ctx->ProtocolMsgRx) {
		fusb_pd_ctx->ProtocolMsgRx = false;
		if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
			switch (fusb_pd_ctx->PolicyRxHeader.MessageType) {
			case CMTGetSourceCap:
				set_policy_state(peSourceGiveSourceCaps);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTGetSinkCap:
				set_policy_state(peSourceGiveSinkCaps);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTDR_Swap:
				set_policy_state(peSourceEvaluateDRSwap);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTPR_Swap:
				set_policy_state(peSourceEvaluatePRSwap);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTSoftReset:
				set_policy_state(peSourceSoftReset);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			default:
				break;
			}
		} else {

			switch (fusb_pd_ctx->PolicyRxHeader.MessageType) {
			case DMTRequest:
				set_policy_state(peSourceNegotiateCap);
				break;
			case DMTVenderDefined:
				break;
			default:
				break;
			}
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
	} else if (fusb_pd_ctx->USBPDTxFlag) {
		if (fusb_pd_ctx->PDTransmitHeader.NumDataObjects == 0) {
			switch (fusb_pd_ctx->PDTransmitHeader.MessageType) {
			case CMTGetSinkCap:
				set_policy_state(peSourceGetSinkCaps);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTGetSourceCap:
				set_policy_state(peSourceGetSourceCaps);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTPing:
				set_policy_state(peSourceSendPing);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTGotoMin:
				set_policy_state(peSourceGotoMin);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTPR_Swap:
				if (fusb_i2c_data->PortType == USBTypeC_DRP) {
					set_policy_state(peSourceSendPRSwap);
					set_policy_subindex(0);
					set_pdtx_state(txIdle);
				}
				break;
			case CMTDR_Swap:
				if (fusb_i2c_data->PortType == USBTypeC_DRP) {
					set_policy_state(peSourceSendDRSwap);
					set_policy_subindex(0);
					set_pdtx_state(txIdle);
				}
				break;
			case CMTVCONN_Swap:
				set_policy_state(peSourceSendVCONNSwap);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTSoftReset:
				set_policy_state(peSourceSendSoftReset);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			default:
				break;
			}
		} else {
			switch (fusb_pd_ctx->PDTransmitHeader.MessageType) {
			case DMTSourceCapabilities:
				set_policy_state(peSourceSendCaps);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case DMTVenderDefined:
				set_policy_subindex(0);
				break;
			default:
				break;
			}
		}
	}
}

void PolicySourceGiveSourceCap(void)
{
	PolicySendData(DMTSourceCapabilities,
		       fusb_pd_ctx->CapsHeaderSource.NumDataObjects,
		       &fusb_pd_ctx->CapsSource[0], peSourceReady, 0);
}

void PolicySourceGetSourceCap(void)
{
	PolicySendCommand(CMTGetSourceCap, peSourceReady, 0);
}

void PolicySourceGetSinkCap(void)
{
	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(CMTGetSinkCap, peSourceGetSinkCaps, 1) ==
		    STAT_SUCCESS) {
			fusb_pd_ctx->policystate_time_count = tSenderResponse;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
		}
		break;
	default:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if ((fusb_pd_ctx->PolicyRxHeader.NumDataObjects > 0)
			    && (fusb_pd_ctx->PolicyRxHeader.MessageType ==
				DMTSinkCapabilities)) {
				UpdateCapabilitiesRx(false);
				set_policy_state(peSourceReady);
			} else {

				set_policy_state(peSourceSendHardReset);
			}
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peSourceSendHardReset);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	}
}

void PolicySourceGiveSinkCap(void)
{
	if (fusb_i2c_data->PortType == USBTypeC_DRP)
		PolicySendData(DMTSinkCapabilities,
			       fusb_pd_ctx->CapsHeaderSink.NumDataObjects,
			       &fusb_pd_ctx->CapsSink[0], peSourceReady, 0);
	else
		PolicySendCommand(CMTReject, peSourceReady, 0);
}

void PolicySourceSendPing(void)
{
	PolicySendCommand(CMTPing, peSourceReady, 0);
}

void PolicySourceGotoMin(void)
{
	if (fusb_pd_ctx->ProtocolMsgRx) {
		fusb_pd_ctx->ProtocolMsgRx = false;
		if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
			switch (fusb_pd_ctx->PolicyRxHeader.MessageType) {
			case CMTSoftReset:
				set_policy_state(peSourceSoftReset);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			default:
				break;
			}
		}
	} else {
		switch (fusb_pd_ctx->PolicySubIndex) {
		case 0:
			PolicySendCommand(CMTGotoMin, peSourceGotoMin, 1);
			break;
		case 1:
			fusb_pd_ctx->policystate_time_count = tSnkTransition;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
			increase_policy_subindex();
			break;
		case 2:
			if (!fusb_pd_ctx->policystate_time_count)
				increase_policy_subindex();
			break;
		case 3:
			increase_policy_subindex();
			break;
		case 4:
			increase_policy_subindex();
			break;
		default:
			PolicySendCommand(CMTPS_RDY, peSourceReady, 0);
			break;
		}
	}
}

void PolicySourceSendDRSwap(void)
{
	u8 Status;

	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		Status =
		    PolicySendCommandNoReset(CMTDR_Swap, peSourceSendDRSwap, 1);
		if (Status == STAT_SUCCESS) {
			fusb_pd_ctx->policystate_time_count = tSenderResponse;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
		} else if (Status == STAT_ERROR)
			set_policy_state(peErrorRecovery);
		break;
	default:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
				switch (fusb_pd_ctx->
					PolicyRxHeader.MessageType) {
				case CMTAccept:
					fusb_pd_ctx->PolicyIsDFP =
					    !fusb_pd_ctx->PolicyIsDFP;
					fusb_i2c_data->Registers.
					    Switches.DATAROLE =
					    fusb_pd_ctx->PolicyIsDFP;
					FUSB300Write(regSwitches1, 1,
						     &fusb_i2c_data->
						     Registers.Switches.
						     byte[1]);
					set_policy_state(peSourceReady);
					break;
				case CMTSoftReset:
					set_policy_state(peSourceSoftReset);
					break;
				default:
					set_policy_state(peSourceReady);
					break;
				}
			} else {

				set_policy_state(peSourceReady);
			}
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		} else if (fusb_pd_ctx->policystate_time_count == 0) {
			set_policy_state(peSourceReady);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	}
}

void PolicySourceEvaluateDRSwap(void)
{
	u8 Status;

	if (fusb_i2c_data->PortType != USBTypeC_DRP) {
		PolicySendCommand(CMTReject, peSourceReady, 0);
	} else {

		Status = PolicySendCommandNoReset(CMTAccept, peSourceReady, 0);
		if (Status == STAT_SUCCESS) {
			fusb_pd_ctx->PolicyIsDFP = !fusb_pd_ctx->PolicyIsDFP;
			fusb_i2c_data->Registers.Switches.DATAROLE =
			    fusb_pd_ctx->PolicyIsDFP;
			FUSB300Write(regSwitches1, 1,
				     &fusb_i2c_data->Registers.
				     Switches.byte[1]);
		} else if (Status == STAT_ERROR) {
			set_policy_state(peErrorRecovery);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
	}
}

void PolicySourceSendVCONNSwap(void)
{
	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(CMTVCONN_Swap, peSourceSendVCONNSwap, 1)
		    == STAT_SUCCESS) {
			fusb_pd_ctx->policystate_time_count = tSenderResponse;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
		}
		break;
	case 1:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
				switch (fusb_pd_ctx->
					PolicyRxHeader.MessageType) {
				case CMTAccept:
					increase_policy_subindex();
					break;
				case CMTWait:
				case CMTReject:
					set_policy_state(peSourceReady);
					set_policy_subindex(0);
					set_pdtx_state(txIdle);
					break;
				default:
					break;
				}
			}
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peSourceReady);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	case 2:
		if (fusb_i2c_data->Registers.Switches.VCONN_CC1
		    || fusb_i2c_data->Registers.Switches.VCONN_CC2) {
			fusb_pd_ctx->policystate_time_count = tVCONNSourceOn;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
			increase_policy_subindex();
		} else {

			if (fusb_i2c_data->blnCCPinIsCC1)
				fusb_i2c_data->Registers.Switches.VCONN_CC2 = 1;
			else
				fusb_i2c_data->Registers.Switches.VCONN_CC1 = 1;
			FUSB300Write(regSwitches0, 1,
				     &fusb_i2c_data->Registers.
				     Switches.byte[0]);
			fusb_pd_ctx->policystate_time_count =
			    tFPF2498Transition;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
			set_policy_subindex(4);
		}
		break;
	case 3:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
				switch (fusb_pd_ctx->
					PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					fusb_i2c_data->Registers.
					    Switches.VCONN_CC1 = 0;
					fusb_i2c_data->Registers.
					    Switches.VCONN_CC2 = 0;
					FUSB300Write(regSwitches0, 1,
						     &fusb_i2c_data->
						     Registers.Switches.
						     byte[0]);
					set_policy_state(peSourceReady);
					set_policy_subindex(0);
					set_pdtx_state(txIdle);
					break;
				default:
					break;
				}
			}
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peSourceSendHardReset);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	default:
		if (!fusb_pd_ctx->policystate_time_count)
			PolicySendCommand(CMTPS_RDY, peSourceReady, 0);
		break;
	}
}

void PolicySourceSendPRSwap(void)
{
	u8 Status;

	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(CMTPR_Swap, peSourceSendPRSwap, 1) ==
		    STAT_SUCCESS) {
			fusb_pd_ctx->policystate_time_count = tSenderResponse;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
		}
		break;
	case 1:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
				switch (fusb_pd_ctx->
					PolicyRxHeader.MessageType) {
				case CMTAccept:
					fusb_i2c_data->prswap_time_count =
					    tPRSwapBailout;
					usbpd_start_timer
					    (&fusb_pd_ctx->prswap_hrtimer,
					     fusb_i2c_data->prswap_time_count);
					fusb_pd_ctx->policystate_time_count =
					    tSnkTransition;
					usbpd_start_timer
					    (&fusb_pd_ctx->policystate_hrtimer,
					     fusb_pd_ctx->
						policystate_time_count);
					increase_policy_subindex();
					break;
				case CMTWait:
				case CMTReject:
					set_policy_state(peSourceReady);
					set_policy_subindex(0);
					set_pdtx_state(txIdle);
					break;
				default:
					break;
				}
			}
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peSourceReady);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	case 2:
		if (!fusb_pd_ctx->policystate_time_count) {
			RoleSwapToAttachedSink();
			fusb_pd_ctx->policystate_time_count = tPSSourceOffNom;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
			increase_policy_subindex();
		}
		break;
	case 3:
		if (!fusb_pd_ctx->policystate_time_count) {
			Status =
			    PolicySendCommandNoReset(CMTPS_RDY,
						     peSourceSendPRSwap, 4);
			if (Status == STAT_SUCCESS) {
				fusb_pd_ctx->policystate_time_count =
				    tPSSourceOnMax;
				usbpd_start_timer
				    (&fusb_pd_ctx->policystate_hrtimer,
				     fusb_pd_ctx->policystate_time_count);
			} else if (Status == STAT_ERROR)
				set_policy_state(peErrorRecovery);
		}
		break;
	default:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
				switch (fusb_pd_ctx->
					PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					set_policy_state(peSinkStartup);
					set_policy_subindex(0);
					fusb_pd_ctx->policystate_time_count = 0;
					break;
				default:
					break;
				}
			}
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peErrorRecovery);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	}
}

void PolicySourceEvaluatePRSwap(void)
{
	u8 Status;

	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		if (fusb_i2c_data->PortType != USBTypeC_DRP) {
			PolicySendCommand(CMTReject, peSourceReady, 0);
		} else {
			if (PolicySendCommand
			    (CMTAccept, peSourceEvaluatePRSwap,
			     1) == STAT_SUCCESS) {
				fusb_i2c_data->prswap_time_count =
				    tPRSwapBailout;
				usbpd_start_timer(&fusb_pd_ctx->prswap_hrtimer,
						  fusb_i2c_data->
							prswap_time_count);
				RoleSwapToAttachedSink();
				fusb_pd_ctx->policystate_time_count =
				    tPSSourceOffNom;
				usbpd_start_timer
				    (&fusb_pd_ctx->policystate_hrtimer,
				     fusb_pd_ctx->policystate_time_count);
			}
		}
		break;
	case 1:
		if (!fusb_pd_ctx->policystate_time_count) {
			Status =
			    PolicySendCommandNoReset(CMTPS_RDY,
						     peSourceEvaluatePRSwap, 2);
			if (Status == STAT_SUCCESS) {
				fusb_pd_ctx->policystate_time_count =
				    tPSSourceOnMax;
				usbpd_start_timer
				    (&fusb_pd_ctx->policystate_hrtimer,
				     fusb_pd_ctx->policystate_time_count);
			} else if (Status == STAT_ERROR)
				set_policy_state(peErrorRecovery);
		}
		break;
	default:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
				switch (fusb_pd_ctx->
					PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					set_policy_state(peSinkStartup);
					set_policy_subindex(0);
					fusb_pd_ctx->policystate_time_count = 0;
					break;
				default:
					break;
				}
			}
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peErrorRecovery);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	}
}

void PolicySinkSendHardReset(void)
{
	PolicySendHardReset(peSinkTransitionDefault, 0);
}

void PolicySinkSoftReset(void)
{
	if (PolicySendCommand(CMTAccept, peSinkWaitCaps, 0) == STAT_SUCCESS) {
		fusb_pd_ctx->policystate_time_count = tSinkWaitCap;
		usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
				  fusb_pd_ctx->policystate_time_count);
	}
}

void PolicySinkSendSoftReset(void)
{
	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(CMTSoftReset, peSinkSendSoftReset, 1) ==
		    STAT_SUCCESS) {
			fusb_pd_ctx->policystate_time_count = tSenderResponse;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
		}
		break;
	default:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if ((fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0)
			    && (fusb_pd_ctx->PolicyRxHeader.MessageType ==
				CMTAccept)) {
				set_policy_state(peSinkWaitCaps);
				fusb_pd_ctx->policystate_time_count =
				    tSinkWaitCap;
				usbpd_start_timer
				    (&fusb_pd_ctx->policystate_hrtimer,
				     fusb_pd_ctx->policystate_time_count);
			} else
				set_policy_state(peSinkSendHardReset);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peSinkSendHardReset);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	}
}

void PolicySinkTransitionDefault(void)
{
	fusb_pd_ctx->USBPDContract.object = 0;
	fusb_pd_ctx->noresponse_time_count = tNoResponse;
	usbpd_start_timer(&fusb_pd_ctx->noresponse_hrtimer,
			  fusb_pd_ctx->noresponse_time_count);
	fusb_i2c_data->prswap_time_count = tNoResponse;
	usbpd_start_timer(&fusb_pd_ctx->prswap_hrtimer,
			  fusb_i2c_data->prswap_time_count);
	set_policy_state(peSinkStartup);
	set_policy_subindex(0);
	set_pdtx_state(txIdle);
	ResetProtocolLayer(true);
}

void PolicySinkStartup(void)
{
	fusb_pd_ctx->PolicyIsSource = false;
	ResetProtocolLayer(true);
	fusb_pd_ctx->CapsCounter = 0;
	fusb_pd_ctx->CollisionCounter = 0;
	fusb_pd_ctx->policystate_time_count = 0;
	set_policy_state(peSinkDiscovery);
	set_policy_subindex(0);
}

void PolicySinkDiscovery(void)
{
	if (fusb_i2c_data->Registers.Status.VBUSOK) {
		fusb_i2c_data->prswap_time_count = 0;
		set_policy_state(peSinkWaitCaps);
		set_policy_subindex(0);
		fusb_pd_ctx->policystate_time_count = tSinkWaitCap;
	} else if (fusb_pd_ctx->noresponse_time_count == 0) {
		fusb_i2c_data->prswap_time_count = 0;
		set_policy_state(peErrorRecovery);
		set_policy_subindex(0);
	}
}

void PolicySinkWaitCaps(void)
{
	if (fusb_pd_ctx->ProtocolMsgRx) {
		fusb_pd_ctx->ProtocolMsgRx = false;
		if ((fusb_pd_ctx->PolicyRxHeader.NumDataObjects > 0)
		    && (fusb_pd_ctx->PolicyRxHeader.MessageType ==
			DMTSourceCapabilities)) {
			UpdateCapabilitiesRx(true);
			set_policy_state(peSinkEvaluateCaps);
		} else if ((fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0)
			   && (fusb_pd_ctx->PolicyRxHeader.MessageType ==
			       CMTSoftReset)) {
			set_policy_state(peSinkSoftReset);
		}
		set_policy_subindex(0);
	} else if ((fusb_pd_ctx->noresponse_time_count == 0)
		   && (fusb_pd_ctx->HardResetCounter > nHardResetCount)) {
		set_policy_state(peErrorRecovery);
		set_policy_subindex(0);
	} else if (fusb_pd_ctx->policystate_time_count == 0) {
		set_policy_state(peSinkSendHardReset);
		set_policy_subindex(0);
	} else {
		if (fusb_i2c_data->g_peSinkWaitCaps_retry <= 3) {
			pr_info("%s:g_peSinkWaitCaps_retry=%d\n",
			  __func__, fusb_i2c_data->g_peSinkWaitCaps_retry);
			fusb_pd_ctx->PolicyState = peSinkWaitCaps;
			fusb_pd_ctx->policystate_time_count = tSinkWaitCap;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					fusb_pd_ctx->policystate_time_count);
			fusb_i2c_data->g_peSinkWaitCaps_retry += 1;
	}
}
}

int usbpd_getvoltagecurrent(int select)
{
	int ret = 0;

	switch (select) {
	case USBPD_VOLTAGE: /* mV */
		if (smb1351_is_battery_low() == true)
			ret =
		    (fusb_pd_ctx->pdpower_record.selVoltage_lowbattery*1000)/20;
		else
			ret =
		    (fusb_pd_ctx->pdpower_record.usbpdselVoltage * 1000)/20;

		pr_info("%s: USBPD_VOLTAGE = %d mV\n", __func__, ret);
		return ret;
	case USBPD_CURRENT: /* mA */
		if (smb1351_is_battery_low() == true)
			ret =
		  (fusb_pd_ctx->pdpower_record.selCurrent_lowbattery*1000)/100;
		else
			ret =
		    (fusb_pd_ctx->pdpower_record.usbpdselCurrent*1000)/100;

		pr_info("%s: USBPD_CURRENT = %d mA\n", __func__, ret);
		return ret;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(usbpd_getvoltagecurrent);

void usbpd_jump_voltage(void)
{
	set_policy_state(peSinkEvaluateCaps);
	wake_up_statemachine();
}
EXPORT_SYMBOL(usbpd_jump_voltage);

void PolicySinkEvaluateCaps(void)
{
	int i, reqPos;
	int select_done = 0;
	unsigned int objVoltage, objCurrent, objPower, MaxPower, SelVoltage,
	    ReqCurrent;
	unsigned int tempCurrent;

	fusb_pd_ctx->noresponse_time_count = USHRT_MAX;
	fusb_pd_ctx->HardResetCounter = 0;
	SelVoltage = 0;
	MaxPower = 0;
	reqPos = 0;
	ReqCurrent = 0;
	objCurrent = 0;
	objVoltage = 0;
	tempCurrent = 0;
	for (i = 0; i < fusb_pd_ctx->CapsHeaderReceived.NumDataObjects; i++) {
		pr_info("%s: NumDataObjects=%d\n",
		  __func__, fusb_pd_ctx->CapsHeaderReceived.NumDataObjects);
		switch (fusb_pd_ctx->CapsReceived[i].PDO.SupplyType) {
		case pdoTypeFixed:
			objVoltage =
			    fusb_pd_ctx->CapsReceived[i].FPDOSupply.Voltage;
			if (objVoltage > fusb_pd_ctx->SinkRequestMaxVoltage)
				objPower = 0;
			else {

				objCurrent =
				    fusb_pd_ctx->CapsReceived[i].
				    FPDOSupply.MaxCurrent;
				objPower = objVoltage * objCurrent;
				pr_info("%s: objCurrent=%d\n",
					__func__, objCurrent);
			}
			pr_info("%s: objVoltage= %d, objPower=%d\n",
				__func__, objVoltage, objPower);
			break;
		case pdoTypeVariable:
			objVoltage =
			    fusb_pd_ctx->CapsReceived[i].VPDO.MaxVoltage;
			if (objVoltage > fusb_pd_ctx->SinkRequestMaxVoltage)
				objPower = 0;
			else {

				objVoltage =
				    fusb_pd_ctx->CapsReceived[i].
				    VPDO.MinVoltage;
				if (objVoltage >
					fusb_pd_ctx->SinkRequestMaxVoltage)
					objPower = 0;
				else {
				objCurrent =
				   fusb_pd_ctx->CapsReceived[i].VPDO.MaxCurrent;
					if (objCurrent >
					  fusb_pd_ctx->SinkRequestMaxCurrent)
						objCurrent =
					    fusb_pd_ctx->SinkRequestMaxCurrent;
				objPower = objVoltage * objCurrent;
			}
			}
			break;
		case pdoTypeBattery:
		default:
			pr_info("%s: pdoTypeBattery\n", __func__);
			objPower = 0;
			break;
		}
		if (objVoltage == 100
		  && (fusb_pd_ctx->pdpower_record.selCurrent_lowbattery <
			objCurrent)) {
			fusb_pd_ctx->pdpower_record.selVoltage_lowbattery
								= objVoltage;
			fusb_pd_ctx->pdpower_record.selCurrent_lowbattery
								= objCurrent;
			fusb_pd_ctx->pdpower_record.ObjectPosition_lowbattery
								= i + 1;
		}
		if (objPower > MaxPower) {
			/* 8V,3A v.s. 12V,3A 8V,4A v.s 12V,4A*/
			if (objPower > fusb_pd_ctx->SinkRequestMaxPower) {
				if (MaxPower == 0) {
					select_done = 1;
				/* 8V,4A v.s 12V,4A */
				} else if (MaxPower >=
					fusb_pd_ctx->SinkRequestMaxPower) {
					if (objVoltage <
				   fusb_pd_ctx->pdpower_record.usbpdselVoltage)
						select_done = 1;
				} else {
					select_done = 1;
				}
			} else {
				select_done = 1;
			}
		/* 10V 1A v.s 5v,2A    6V,4A v.s 12v 2A */
		} else if (objPower == MaxPower) {
			pr_info("%s: objPower == MaxPower : objCurrent=%d\n",
				__func__, objCurrent);
			if (objCurrent > fusb_pd_ctx->SinkRequestMaxCurrent) {
				if (MaxPower <
				  fusb_pd_ctx->SinkRequestMaxCurrent*objVoltage)
					select_done = 1;
			} else if (fusb_pd_ctx->pdpower_record.usbpdselVoltage >
					objVoltage)
				select_done = 1;
			else {
			  if (ReqCurrent > fusb_pd_ctx->SinkRequestMaxCurrent)
				if (objPower >
				  fusb_pd_ctx->SinkRequestMaxCurrent*SelVoltage)
					select_done = 1;
			}
		}
		if (select_done == 1) {
			MaxPower = objPower;
			SelVoltage = objVoltage;
			ReqCurrent = objCurrent;
			reqPos = i + 1;
			fusb_pd_ctx->pdpower_record.usbpdselVoltage
				= SelVoltage;
			fusb_pd_ctx->pdpower_record.usbpdselCurrent
				= ReqCurrent;
			fusb_pd_ctx->pdpower_record.usbpdselObjectPosition
				= reqPos;
			select_done = 0;
		}
	}

	if (smb1351_is_battery_low() == true) {
		MaxPower = fusb_pd_ctx->pdpower_record.selVoltage_lowbattery
			* fusb_pd_ctx->pdpower_record.selCurrent_lowbattery;
		SelVoltage = fusb_pd_ctx->pdpower_record.selVoltage_lowbattery;
		ReqCurrent = fusb_pd_ctx->pdpower_record.selCurrent_lowbattery;
		reqPos = fusb_pd_ctx->pdpower_record.ObjectPosition_lowbattery;
	} else {
		MaxPower = fusb_pd_ctx->pdpower_record.usbpdselVoltage
			* fusb_pd_ctx->pdpower_record.usbpdselCurrent;
		SelVoltage = fusb_pd_ctx->pdpower_record.usbpdselVoltage;
		ReqCurrent = fusb_pd_ctx->pdpower_record.usbpdselCurrent;
		reqPos = fusb_pd_ctx->pdpower_record.usbpdselObjectPosition;
	}

	pr_info("low_battery: reqPos=%d, selVoltage=%d, ReqCurrent =%d\n",
			fusb_pd_ctx->pdpower_record.ObjectPosition_lowbattery,
			fusb_pd_ctx->pdpower_record.selVoltage_lowbattery,
			fusb_pd_ctx->pdpower_record.selCurrent_lowbattery);

	pr_info("normal: reqPos=%d, selVoltage=%d, ReqCurrent =%d\n",
			fusb_pd_ctx->pdpower_record.usbpdselObjectPosition,
			fusb_pd_ctx->pdpower_record.usbpdselVoltage,
			fusb_pd_ctx->pdpower_record.usbpdselCurrent);


	pr_info("reqPos=%d, selVoltage=%d, ReqCurrent =%d\n",
			reqPos, SelVoltage, ReqCurrent);
	 tempCurrent = ReqCurrent;

	if ((reqPos > 0) && (SelVoltage > 0)) {
		fusb_pd_ctx->SinkRequest.FVRDO.ObjectPosition = reqPos & 0x07;
		fusb_pd_ctx->SinkRequest.FVRDO.GiveBack =
		    fusb_pd_ctx->SinkGotoMinCompatible;
		fusb_pd_ctx->SinkRequest.FVRDO.NoUSBSuspend =
		    fusb_pd_ctx->SinkUSBSuspendOperation;
		fusb_pd_ctx->SinkRequest.FVRDO.USBCommCapable =
		    fusb_pd_ctx->SinkUSBCommCapable;
		ReqCurrent = fusb_pd_ctx->SinkRequestOpPower / SelVoltage;
		if (tempCurrent > fusb_pd_ctx->SinkRequestMaxCurrent)
			ReqCurrent = fusb_pd_ctx->SinkRequestMaxCurrent;
		else if (ReqCurrent > tempCurrent)
			ReqCurrent = tempCurrent;

		fusb_pd_ctx->SinkRequest.FVRDO.OpCurrent = (ReqCurrent & 0x3FF);
		ReqCurrent = fusb_pd_ctx->SinkRequestOpPower / SelVoltage;
		if (tempCurrent > fusb_pd_ctx->SinkRequestMaxCurrent)
			ReqCurrent = fusb_pd_ctx->SinkRequestMaxCurrent;
		else if (ReqCurrent > tempCurrent)
			ReqCurrent = tempCurrent;

		fusb_pd_ctx->SinkRequest.FVRDO.MinMaxCurrent =
		    (ReqCurrent & 0x3FF);
		if (fusb_pd_ctx->SinkGotoMinCompatible)
			fusb_pd_ctx->SinkRequest.FVRDO.CapabilityMismatch =
			    false;
		else {

			if (MaxPower < fusb_pd_ctx->SinkRequestMaxPower)
				fusb_pd_ctx->SinkRequest.
				    FVRDO.CapabilityMismatch = true;
			else
				fusb_pd_ctx->SinkRequest.
				    FVRDO.CapabilityMismatch = false;
		}
		fusb_pd_ctx->pdpower_record.usbpdselCurrent = ReqCurrent;
		pr_info("Re request: reqPos=%d, selVoltage=%d, ReqCurrent =%d\n",
			reqPos, SelVoltage, ReqCurrent);

		pr_info("fusb_pd_ctx->SinkRequest.FVRDO.CapabilityMismatch =%d\n",
			fusb_pd_ctx->SinkRequest.FVRDO.CapabilityMismatch);
		bat_update_TypeC_charger_type(TYPEC_PD_NO_READY_CHARGER);
		if (smb1351_is_battery_low() == false)
			smb1351_suspend_enable(1);

		set_policy_state(peSinkSelectCapability);
		set_policy_subindex(0);
		fusb_pd_ctx->policystate_time_count = tSenderResponse;
		usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
				  fusb_pd_ctx->policystate_time_count);
	} else {
		set_policy_state(peSinkWaitCaps);
		fusb_pd_ctx->policystate_time_count = tSinkWaitCap;
		usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
				  fusb_pd_ctx->policystate_time_count);
	}
}

void PolicySinkSelectCapability(void)
{
	unsigned char oldPolicySubIndex;

	do {
		oldPolicySubIndex = fusb_pd_ctx->PolicySubIndex;
		switch (fusb_pd_ctx->PolicySubIndex) {
		case 0:
			if (PolicySendData
			    (DMTRequest, 1, &fusb_pd_ctx->SinkRequest,
			     peSinkSelectCapability, 1) == STAT_SUCCESS) {
				fusb_pd_ctx->policystate_time_count =
				    tSenderResponse;
				usbpd_start_timer
				    (&fusb_pd_ctx->policystate_hrtimer,
				     fusb_pd_ctx->policystate_time_count);
			}
			break;
		default:
			if (fusb_pd_ctx->ProtocolMsgRx) {
				fusb_pd_ctx->ProtocolMsgRx = false;
				if (fusb_pd_ctx->
				    PolicyRxHeader.NumDataObjects == 0) {
					switch (fusb_pd_ctx->
						PolicyRxHeader.MessageType) {
					case CMTAccept:
						fusb_pd_ctx->PolicyHasContract =
						    true;
						fusb_pd_ctx->
						    USBPDContract.object =
						    fusb_pd_ctx->
						    SinkRequest.object;
						fusb_pd_ctx->
						    policystate_time_count =
						    tPSTransition;
						usbpd_start_timer
						    (&fusb_pd_ctx->
							policystate_hrtimer,
						     fusb_pd_ctx->
							policystate_time_count);
						set_policy_state
						    (peSinkTransitionSink);
						break;
					case CMTWait:
					case CMTReject:
						set_policy_state(peSinkReady);
						break;
					case CMTSoftReset:
						set_policy_state
						    (peSinkSoftReset);
						break;
					default:
						set_policy_state
						    (peSinkSendSoftReset);
						break;
					}
				} else {

					switch (fusb_pd_ctx->
						PolicyRxHeader.MessageType) {
					case DMTSourceCapabilities:
						UpdateCapabilitiesRx(true);
						set_policy_state
						    (peSinkEvaluateCaps);
						break;
					default:
						set_policy_state
						    (peSinkSendSoftReset);
						break;
					}
				}
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
			} else if (fusb_pd_ctx->policystate_time_count == 0) {
				set_policy_state(peSinkSendHardReset);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
			}
			break;
		}
	} while ((fusb_pd_ctx->PolicyState == peSinkSelectCapability)
		 && (oldPolicySubIndex < fusb_pd_ctx->PolicySubIndex));
}

void PolicySinkTransitionSink(void)
{
	if (fusb_pd_ctx->ProtocolMsgRx) {
		fusb_pd_ctx->ProtocolMsgRx = false;
		if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
			switch (fusb_pd_ctx->PolicyRxHeader.MessageType) {
			case CMTPS_RDY:
				set_policy_state(peSinkReady);
				/* update PD charger type to battery driver */
			  if (smb1351_is_battery_low())
				bat_update_TypeC_charger_type(TYPEC_PD_5V_CHARGER);
			  else
				bat_update_TypeC_charger_type(TYPEC_PD_9V_CHARGER);
			  pr_info("%s: CMTPS_RDY\n", __func__);
				break;
			case CMTSoftReset:
				set_policy_state(peSinkSoftReset);
				break;
			default:
				break;
			}
		} else {

			switch (fusb_pd_ctx->PolicyRxHeader.MessageType) {
			case DMTSourceCapabilities:
				UpdateCapabilitiesRx(true);
				set_policy_state(peSinkEvaluateCaps);
				break;
			default:
				set_policy_state(peSinkSendSoftReset);
				break;
			}
		}
		set_policy_subindex(0);
		set_pdtx_state(txIdle);
	} else if (fusb_pd_ctx->policystate_time_count == 0) {
		set_policy_state(peSinkSendHardReset);
		set_policy_subindex(0);
		set_pdtx_state(txIdle);
	}
}

void PolicySinkReady(void)
{
	if (fusb_pd_ctx->ProtocolMsgRx) {
		fusb_pd_ctx->ProtocolMsgRx = false;
		if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
			switch (fusb_pd_ctx->PolicyRxHeader.MessageType) {
			case CMTGotoMin:
				set_policy_state(peSinkTransitionSink);
				fusb_pd_ctx->policystate_time_count =
				    tPSTransition;
				usbpd_start_timer
				    (&fusb_pd_ctx->policystate_hrtimer,
				     fusb_pd_ctx->policystate_time_count);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTGetSinkCap:
				set_policy_state(peSinkGiveSinkCap);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTGetSourceCap:
				set_policy_state(peSinkGiveSourceCap);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTDR_Swap:
				set_policy_state(peSinkEvaluateDRSwap);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTPR_Swap:
				set_policy_state(peSinkEvaluatePRSwap);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTVCONN_Swap:
				set_policy_state(peSinkEvaluateVCONNSwap);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTSoftReset:
				set_policy_state(peSinkSoftReset);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			default:
				break;
			}
		} else {
			switch (fusb_pd_ctx->PolicyRxHeader.MessageType) {
			case DMTSourceCapabilities:
				UpdateCapabilitiesRx(true);
				set_policy_state(peSinkEvaluateCaps);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case DMTVenderDefined:
				break;
			default:
				break;
			}
		}
		set_policy_subindex(0);
		set_pdtx_state(txIdle);
	} else if (fusb_pd_ctx->USBPDTxFlag) {
		if (fusb_pd_ctx->PDTransmitHeader.NumDataObjects == 0) {
			switch (fusb_pd_ctx->PDTransmitHeader.MessageType) {
			case CMTGetSourceCap:
				set_policy_state(peSinkGetSourceCap);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTGetSinkCap:
				set_policy_state(peSinkGetSinkCap);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			case CMTDR_Swap:
				if (fusb_i2c_data->PortType == USBTypeC_DRP) {
					set_policy_state(peSinkSendDRSwap);
					set_policy_subindex(0);
					set_pdtx_state(txIdle);
				}
				break;
			case CMTPR_Swap:
				if (fusb_i2c_data->PortType == USBTypeC_DRP) {
					set_policy_state(peSinkSendPRSwap);
					set_policy_subindex(0);
					set_pdtx_state(txIdle);
				}
				break;
			case CMTSoftReset:
				set_policy_state(peSinkSendSoftReset);
				set_policy_subindex(0);
				set_pdtx_state(txIdle);
				break;
			default:
				break;
			}
		} else {
			switch (fusb_pd_ctx->PDTransmitHeader.MessageType) {
			case DMTRequest:
				fusb_pd_ctx->SinkRequest.object =
				    fusb_pd_ctx->PDTransmitObjects[0].object;
				set_policy_state(peSinkSelectCapability);
				set_policy_subindex(0);
				fusb_pd_ctx->policystate_time_count =
				    tSenderResponse;
				usbpd_start_timer
				    (&fusb_pd_ctx->policystate_hrtimer,
				     fusb_pd_ctx->policystate_time_count);
				break;
			case DMTVenderDefined:
				set_policy_subindex(0);
				break;
			default:
				break;
			}
		}
	}
}

void PolicySinkGiveSinkCap(void)
{
	PolicySendData(DMTSinkCapabilities,
		       fusb_pd_ctx->CapsHeaderSink.NumDataObjects,
		       &fusb_pd_ctx->CapsSink[0], peSinkReady, 0);
}

void PolicySinkGetSinkCap(void)
{
	PolicySendCommand(CMTGetSinkCap, peSinkReady, 0);
}

void PolicySinkGiveSourceCap(void)
{
	if (fusb_i2c_data->PortType == USBTypeC_DRP)
		PolicySendData(DMTSourceCapabilities,
			       fusb_pd_ctx->CapsHeaderSource.NumDataObjects,
			       &fusb_pd_ctx->CapsSource[0], peSinkReady, 0);
	else
		PolicySendCommand(CMTReject, peSinkReady, 0);
}

void PolicySinkGetSourceCap(void)
{
	PolicySendCommand(CMTGetSourceCap, peSinkReady, 0);
}

void PolicySinkSendDRSwap(void)
{
	u8 Status;

	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		Status =
		    PolicySendCommandNoReset(CMTDR_Swap, peSinkSendDRSwap, 1);
		if (Status == STAT_SUCCESS) {
			fusb_pd_ctx->policystate_time_count = tSenderResponse;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
		} else if (Status == STAT_ERROR)
			set_policy_state(peErrorRecovery);
		break;
	default:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
				switch (fusb_pd_ctx->
					PolicyRxHeader.MessageType) {
				case CMTAccept:
					fusb_pd_ctx->PolicyIsDFP =
					    !fusb_pd_ctx->PolicyIsDFP;
					fusb_i2c_data->Registers.
					    Switches.DATAROLE =
					    fusb_pd_ctx->PolicyIsDFP;
					FUSB300Write(regSwitches1, 1,
						     &fusb_i2c_data->
						     Registers.Switches.
						     byte[1]);
					set_policy_state(peSinkReady);
					break;
				case CMTSoftReset:
					set_policy_state(peSinkSoftReset);
					break;
				default:
					set_policy_state(peSinkReady);
					break;
				}
			} else {

				set_policy_state(peSinkReady);
			}
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		} else if (fusb_pd_ctx->policystate_time_count == 0) {
			set_policy_state(peSinkReady);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	}
}

void PolicySinkEvaluateDRSwap(void)
{
	u8 Status;

	if (fusb_i2c_data->PortType != USBTypeC_DRP) {
		PolicySendCommand(CMTReject, peSinkReady, 0);
	} else {

		Status = PolicySendCommandNoReset(CMTAccept, peSinkReady, 0);
		if (Status == STAT_SUCCESS) {
			fusb_pd_ctx->PolicyIsDFP = !fusb_pd_ctx->PolicyIsDFP;
			fusb_i2c_data->Registers.Switches.DATAROLE =
			    fusb_pd_ctx->PolicyIsDFP;
			FUSB300Write(regSwitches1, 1,
				     &fusb_i2c_data->Registers.
				     Switches.byte[1]);
		} else if (Status == STAT_ERROR) {
			set_policy_state(peErrorRecovery);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
	}
}

void PolicySinkEvaluateVCONNSwap(void)
{
	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		PolicySendCommand(CMTAccept, peSinkEvaluateVCONNSwap, 1);
		break;
	case 1:
		if (fusb_i2c_data->Registers.Switches.VCONN_CC1
		    || fusb_i2c_data->Registers.Switches.VCONN_CC2) {
			fusb_pd_ctx->policystate_time_count = tVCONNSourceOn;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
			increase_policy_subindex();
		} else {

			if (fusb_i2c_data->blnCCPinIsCC1) {
				fusb_i2c_data->Registers.Switches.VCONN_CC2 = 1;
				fusb_i2c_data->Registers.Switches.PDWN2 = 0;
			} else {

				fusb_i2c_data->Registers.Switches.VCONN_CC1 = 1;
				fusb_i2c_data->Registers.Switches.PDWN1 = 0;
			}
			FUSB300Write(regSwitches0, 1,
				     &fusb_i2c_data->Registers.
				     Switches.byte[0]);
			fusb_pd_ctx->policystate_time_count =
			    tFPF2498Transition;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
			set_policy_subindex(3);
		}
		break;
	case 2:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
				switch (fusb_pd_ctx->
					PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					fusb_i2c_data->Registers.
					    Switches.VCONN_CC1 = 0;
					fusb_i2c_data->Registers.
					    Switches.VCONN_CC2 = 0;
					fusb_i2c_data->Registers.
					    Switches.PDWN1 = 1;
					fusb_i2c_data->Registers.
					    Switches.PDWN2 = 1;
					FUSB300Write(regSwitches0, 1,
						     &fusb_i2c_data->
						     Registers.Switches.
						     byte[0]);
					set_policy_state(peSinkReady);
					set_policy_subindex(0);
					set_pdtx_state(txIdle);
					break;
				default:
					break;
				}
			}
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peSourceSendHardReset);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	default:
		if (!fusb_pd_ctx->
			policystate_time_count) {
			PolicySendCommand(CMTPS_RDY, peSinkReady, 0);
		}
		break;
	}
}

void PolicySinkSendPRSwap(void)
{
	u8 Status;

	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		if (PolicySendCommand(CMTPR_Swap, peSinkSendPRSwap, 1) ==
		    STAT_SUCCESS) {
			fusb_pd_ctx->policystate_time_count = tSenderResponse;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
		}
		break;
	case 1:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
				switch (fusb_pd_ctx->
					PolicyRxHeader.MessageType) {
				case CMTAccept:
					fusb_i2c_data->prswap_time_count =
					    tPRSwapBailout;
					usbpd_start_timer
					    (&fusb_pd_ctx->prswap_hrtimer,
					     fusb_i2c_data->prswap_time_count);
					fusb_pd_ctx->policystate_time_count =
					    tPSSourceOffMax;
					usbpd_start_timer
					    (&fusb_pd_ctx->policystate_hrtimer,
					     fusb_pd_ctx->
						policystate_time_count);
					increase_policy_subindex();
					break;
				case CMTWait:
				case CMTReject:
					set_policy_state(peSinkReady);
					set_policy_subindex(0);
					set_pdtx_state(txIdle);
					break;
				default:
					break;
				}
			}
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peSinkReady);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	case 2:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
				switch (fusb_pd_ctx->
					PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					RoleSwapToAttachedSource();
					fusb_pd_ctx->policystate_time_count =
					    tPSSourceOnNom;
					usbpd_start_timer
					    (&fusb_pd_ctx->policystate_hrtimer,
					     fusb_pd_ctx->
						policystate_time_count);
					increase_policy_subindex();
					break;
				default:
					break;
				}
			}
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peErrorRecovery);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	default:
		if (!fusb_pd_ctx->policystate_time_count) {
			Status =
			    PolicySendCommandNoReset(CMTPS_RDY, peSourceStartup,
						     0);
			if (Status == STAT_ERROR)
				set_policy_state(peErrorRecovery);
		}
		break;
	}
}

void PolicySinkEvaluatePRSwap(void)
{
	u8 Status;

	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		if (fusb_i2c_data->PortType != USBTypeC_DRP) {
			PolicySendCommand(CMTReject, peSinkReady, 0);
		} else {
			if (PolicySendCommand
			    (CMTAccept, peSinkEvaluatePRSwap,
			     1) == STAT_SUCCESS) {
				fusb_i2c_data->prswap_time_count =
				    tPRSwapBailout;
				usbpd_start_timer(&fusb_pd_ctx->prswap_hrtimer,
						  fusb_i2c_data->
						  prswap_time_count);
				fusb_pd_ctx->policystate_time_count =
				    tPSSourceOffMax;
				usbpd_start_timer
				    (&fusb_pd_ctx->policystate_hrtimer,
				     fusb_pd_ctx->policystate_time_count);
			}
		}
		break;
	case 1:
		if (fusb_pd_ctx->ProtocolMsgRx) {
			fusb_pd_ctx->ProtocolMsgRx = false;
			if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0) {
				switch (fusb_pd_ctx->
					PolicyRxHeader.MessageType) {
				case CMTPS_RDY:
					RoleSwapToAttachedSource();
					fusb_pd_ctx->policystate_time_count =
					    tPSSourceOnNom;
					usbpd_start_timer
					    (&fusb_pd_ctx->policystate_hrtimer,
					     fusb_pd_ctx->
						policystate_time_count);
					increase_policy_subindex();
					break;
				default:
					break;
				}
			}
		} else if (!fusb_pd_ctx->policystate_time_count) {
			set_policy_state(peErrorRecovery);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	default:
		if (!fusb_pd_ctx->policystate_time_count) {
			Status =
			    PolicySendCommandNoReset(CMTPS_RDY, peSourceStartup,
						     0);
			if (Status == STAT_ERROR)
				set_policy_state(peErrorRecovery);
		}
		break;
	}
}

bool PolicySendHardReset(enum PolicyState_t nextState, u32 delay)
{
	bool Success = false;

	switch (fusb_pd_ctx->PolicySubIndex) {
	case 0:
		switch (fusb_pd_ctx->PDTxStatus) {
		case txReset:
		case txWait:
			break;
		case txSuccess:
			fusb_pd_ctx->policystate_time_count = delay;
			usbpd_start_timer(&fusb_pd_ctx->policystate_hrtimer,
					  fusb_pd_ctx->policystate_time_count);
			increase_policy_subindex();
			Success = true;
			break;
		default:
			set_pdtx_state(txReset);
			break;
		}
		break;
	default:
		if (fusb_pd_ctx->policystate_time_count == 0) {
			fusb_pd_ctx->HardResetCounter++;
			set_policy_state(nextState);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
		}
		break;
	}
	return Success;
}

u8 PolicySendCommand(u8 Command, enum PolicyState_t nextState, u8 subIndex)
{
	u8 Status = STAT_BUSY;

	if (fusb_pd_ctx->ProtocolMsgTx == true) {
		set_policy_state(nextState);
		set_policy_subindex(subIndex);
		set_pdtx_state(txIdle);
		Status = STAT_SUCCESS;
		fusb_pd_ctx->ProtocolMsgTx = false;
	} else {
		switch (fusb_pd_ctx->PDTxStatus) {
		case txIdle:
			fusb_pd_ctx->PolicyTxHeader.word = 0;
			fusb_pd_ctx->PolicyTxHeader.NumDataObjects = 0;
			fusb_pd_ctx->PolicyTxHeader.MessageType =
			    Command & 0x0F;
			fusb_pd_ctx->PolicyTxHeader.PortDataRole =
			    fusb_pd_ctx->PolicyIsDFP;
			fusb_pd_ctx->PolicyTxHeader.PortPowerRole =
			    fusb_pd_ctx->PolicyIsSource;
			fusb_pd_ctx->PolicyTxHeader.SpecRevision = USBPDSPECREV;
			set_pdtx_state(txSend);
			break;
		case txSend:
		case txBusy:
		case txWait:
			break;
		case txSuccess:
			set_policy_state(nextState);
			set_policy_subindex(subIndex);
			set_pdtx_state(txIdle);
			Status = STAT_SUCCESS;
			break;
		case txError:
			if (fusb_pd_ctx->PolicyState == peSourceSendSoftReset)
				set_policy_state(peSourceSendHardReset);
			else if (fusb_pd_ctx->PolicyState ==
				 peSinkSendSoftReset)
				set_policy_state(peSinkSendHardReset);
			else if (fusb_pd_ctx->PolicyIsSource)
				set_policy_state(peSourceSendSoftReset);
			else
				set_policy_state(peSinkSendSoftReset);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
			Status = STAT_ERROR;
			break;
		case txCollision:
			fusb_pd_ctx->CollisionCounter++;
			if (fusb_pd_ctx->CollisionCounter > nRetryCount) {
				if (fusb_pd_ctx->PolicyIsSource)
					set_policy_state(peSourceSendHardReset);
				else
					set_policy_state(peSinkSendHardReset);
				set_policy_subindex(0);
				set_pdtx_state(txReset);
				Status = STAT_ERROR;
			} else
				set_pdtx_state(txIdle);
			break;
		default:
			if (fusb_pd_ctx->PolicyIsSource)
				set_policy_state(peSourceSendHardReset);
			else
				set_policy_state(peSinkSendHardReset);
			set_policy_subindex(0);
			set_pdtx_state(txReset);
			Status = STAT_ERROR;
			break;
		}
	}
	return Status;
}

u8 PolicySendCommandNoReset(u8 Command, enum PolicyState_t nextState,
			    u8 subIndex)
{
	u8 Status = STAT_BUSY;

	switch (fusb_pd_ctx->PDTxStatus) {
	case txIdle:
		fusb_pd_ctx->PolicyTxHeader.word = 0;
		fusb_pd_ctx->PolicyTxHeader.NumDataObjects = 0;
		fusb_pd_ctx->PolicyTxHeader.MessageType = Command & 0x0F;
		fusb_pd_ctx->PolicyTxHeader.PortDataRole =
		    fusb_pd_ctx->PolicyIsDFP;
		fusb_pd_ctx->PolicyTxHeader.PortPowerRole =
		    fusb_pd_ctx->PolicyIsSource;
		fusb_pd_ctx->PolicyTxHeader.SpecRevision = USBPDSPECREV;
		set_pdtx_state(txSend);
		break;
	case txSend:
	case txBusy:
	case txWait:
		break;
	case txSuccess:
		set_policy_state(nextState);
		set_policy_subindex(subIndex);
		set_pdtx_state(txIdle);
		Status = STAT_SUCCESS;
		break;
	default:
		set_policy_state(peErrorRecovery);
		set_policy_subindex(0);
		set_pdtx_state(txReset);
		Status = STAT_ERROR;
		break;
	}
	return Status;
}

u8 PolicySendData(u8 MessageType, u8 NumDataObjects,
		  union doDataObject_t *DataObjects,
		  enum PolicyState_t nextState, u8 subIndex)
{
	u8 Status = STAT_BUSY;
	int i;

	if (fusb_pd_ctx->ProtocolMsgTx == true) {
		set_policy_state(nextState);
		set_policy_subindex(subIndex);
		set_pdtx_state(txIdle);
		Status = STAT_SUCCESS;
		fusb_pd_ctx->ProtocolMsgTx = false;
	} else {
		switch (fusb_pd_ctx->PDTxStatus) {
		case txIdle:
			if (NumDataObjects > 7)
				NumDataObjects = 7;
			fusb_pd_ctx->PolicyTxHeader.word = 0x0000;
			fusb_pd_ctx->PolicyTxHeader.NumDataObjects =
			    NumDataObjects;
			fusb_pd_ctx->PolicyTxHeader.MessageType =
			    MessageType & 0x0F;
			fusb_pd_ctx->PolicyTxHeader.PortDataRole =
			    fusb_pd_ctx->PolicyIsDFP;
			fusb_pd_ctx->PolicyTxHeader.PortPowerRole =
			    fusb_pd_ctx->PolicyIsSource;
			fusb_pd_ctx->PolicyTxHeader.SpecRevision = USBPDSPECREV;
			for (i = 0; i < NumDataObjects; i++)
				fusb_pd_ctx->PolicyTxDataObj[i].object =
				    DataObjects[i].object;
			if (fusb_pd_ctx->PolicyState == peSourceSendCaps)
				fusb_pd_ctx->CapsCounter++;
			set_pdtx_state(txSend);
			break;
		case txSend:
		case txBusy:
		case txWait:
			break;
		case txSuccess:
			set_policy_state(nextState);
			set_policy_subindex(subIndex);
			set_pdtx_state(txIdle);
			Status = STAT_SUCCESS;
			break;
		case txError:
			if (fusb_pd_ctx->PolicyState == peSourceSendCaps)
				set_policy_state(peSourceDiscovery);
			else if (fusb_pd_ctx->PolicyIsSource)
				set_policy_state(peSourceSendSoftReset);
			else
				set_policy_state(peSinkSendSoftReset);
			set_policy_subindex(0);
			set_pdtx_state(txIdle);
			Status = STAT_ERROR;
			break;
		case txCollision:
			fusb_pd_ctx->CollisionCounter++;
			if (fusb_pd_ctx->CollisionCounter > nRetryCount) {
				if (fusb_pd_ctx->PolicyIsSource)
					set_policy_state(peSourceSendHardReset);
				else
					set_policy_state(peSinkSendHardReset);
				set_pdtx_state(txReset);
				Status = STAT_ERROR;
			} else
				set_pdtx_state(txIdle);
			break;
		default:
			if (fusb_pd_ctx->PolicyIsSource)
				set_policy_state(peSourceSendHardReset);
			else
				set_policy_state(peSinkSendHardReset);
			set_policy_subindex(0);
			set_pdtx_state(txReset);
			Status = STAT_ERROR;
			break;
		}
	}
	return Status;
}

u8 PolicySendDataNoReset(u8 MessageType, u8 NumDataObjects,
			 union doDataObject_t *DataObjects,
			 enum PolicyState_t nextState, u8 subIndex)
{
	u8 Status = STAT_BUSY;
	int i;

	switch (fusb_pd_ctx->PDTxStatus) {
	case txIdle:
		if (NumDataObjects > 7)
			NumDataObjects = 7;
		fusb_pd_ctx->PolicyTxHeader.word = 0x0000;
		fusb_pd_ctx->PolicyTxHeader.NumDataObjects = NumDataObjects;
		fusb_pd_ctx->PolicyTxHeader.MessageType = MessageType & 0x0F;
		fusb_pd_ctx->PolicyTxHeader.PortDataRole =
		    fusb_pd_ctx->PolicyIsDFP;
		fusb_pd_ctx->PolicyTxHeader.PortPowerRole =
		    fusb_pd_ctx->PolicyIsSource;
		fusb_pd_ctx->PolicyTxHeader.SpecRevision = USBPDSPECREV;
		for (i = 0; i < NumDataObjects; i++)
			fusb_pd_ctx->PolicyTxDataObj[i].object =
			    DataObjects[i].object;
		if (fusb_pd_ctx->PolicyState == peSourceSendCaps)
			fusb_pd_ctx->CapsCounter++;
		set_pdtx_state(txSend);
		break;
	case txSend:
	case txBusy:
	case txWait:
		break;
	case txSuccess:
		set_policy_state(nextState);
		set_policy_subindex(subIndex);
		set_pdtx_state(txIdle);
		Status = STAT_SUCCESS;
		break;
	default:
		set_policy_state(peErrorRecovery);
		set_policy_subindex(0);
		set_pdtx_state(txReset);
		Status = STAT_ERROR;
		break;
	}
	return Status;
}

void UpdateCapabilitiesRx(bool IsSourceCaps)
{
	int i;

	fusb_pd_ctx->SourceCapsUpdated = IsSourceCaps;
	fusb_pd_ctx->CapsHeaderReceived.word = fusb_pd_ctx->PolicyRxHeader.word;

	for (i = 0; i < fusb_pd_ctx->CapsHeaderReceived.NumDataObjects; i++) {
		fusb_pd_ctx->CapsReceived[i].object =
		    fusb_pd_ctx->PolicyRxDataObj[i].object;
	}
	for (i = fusb_pd_ctx->CapsHeaderReceived.NumDataObjects; i < 7; i++)
		fusb_pd_ctx->CapsReceived[i].object = 0;
}

void USBPDProtocol(void)
{
	if (fusb_i2c_data->Registers.Status.I_HARDRST) {
		ResetProtocolLayer(true);
		if (fusb_pd_ctx->PolicyIsSource)
			set_policy_state(peSourceTransitionDefault);
		else
			set_policy_state(peSinkTransitionDefault);
		set_policy_subindex(0);
		StoreUSBPDToken(false, pdtHardReset);
	} else {
		switch (fusb_pd_ctx->ProtocolState) {
		case PRLReset:
			ProtocolSendHardReset();
			set_pdtx_state(txWait);
			set_protocol_state(PRLResetWait);
			fusb_pd_ctx->protocol_time_count = tBMCTimeout;
			usbpd_start_timer(&fusb_pd_ctx->protocol_hrtimer,
					  fusb_pd_ctx->protocol_time_count);
			break;
		case PRLResetWait:
			ProtocolResetWait();
			break;
		case PRLTxSendingMessage:
			ProtocolSendingMessage();
			break;
		case PRLIdle:
			ProtocolIdle();
			break;
		case PRLTxVerifyGoodCRC:
			ProtocolVerifyGoodCRC();
			break;
		default:
			break;
		}
	}
}

void ProtocolIdle(void)
{
	if (fusb_pd_ctx->PDTxStatus == txReset)
		set_protocol_state(PRLReset);
	else if (fusb_i2c_data->Registers.Status.I_GCRCSENT
		 || (fusb_i2c_data->Registers.Status.RX_EMPTY == 0)) {
		ProtocolGetRxPacket();
		set_pdtx_state(txIdle);
	} else if (fusb_pd_ctx->PDTxStatus == txSend) {
		ProtocolTransmitMessage();
	}
}

void ProtocolResetWait(void)
{
	if (fusb_i2c_data->Registers.Status.I_HARDSENT) {
		set_protocol_state(PRLIdle);
		set_pdtx_state(txSuccess);
	} else if (fusb_pd_ctx->protocol_time_count == 0) {
		set_protocol_state(PRLIdle);
		set_pdtx_state(txSuccess);
	}
}

void ProtocolGetRxPacket(void)
{
	int i, j;
	u8 data[3];

	FUSB300Read(regInterruptb, 1, &fusb_i2c_data->Registers.Status.byte[3]);
	FUSB300ReadFIFO(3, &data[0]);
	fusb_pd_ctx->PolicyRxHeader.byte[0] = data[1];
	fusb_pd_ctx->PolicyRxHeader.byte[1] = data[2];

#if 0
	PolicyTxHeader.word = 0;
	PolicyTxHeader.NumDataObjects = 0;
	PolicyTxHeader.MessageType = CMTGoodCRC;
	PolicyTxHeader.PortDataRole = fusb_pd_ctx->PolicyIsDFP;
	PolicyTxHeader.PortPowerRole = fusb_pd_ctx->PolicyIsSource;
	PolicyTxHeader.SpecRevision = USBPDSPECREV;
	PolicyTxHeader.MessageID = fusb_pd_ctx->PolicyRxHeader.MessageID;
#endif
	if ((data[0] & 0xE0) == 0xE0) {
		if ((fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0)
		    && (fusb_pd_ctx->PolicyRxHeader.MessageType ==
			CMTSoftReset)) {
			fusb_pd_ctx->MessageIDCounter = 0;
			fusb_pd_ctx->MessageID = 0xFF;
			fusb_pd_ctx->ProtocolMsgRx = true;
			fusb_pd_ctx->SourceCapsUpdated = true;
		} else if (fusb_pd_ctx->PolicyRxHeader.MessageID !=
			   fusb_pd_ctx->MessageID) {
			fusb_pd_ctx->MessageID =
			    fusb_pd_ctx->PolicyRxHeader.MessageID;
			fusb_pd_ctx->ProtocolMsgRx = true;
		}
	}
	if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects > 0) {
		FUSB300ReadFIFO((fusb_pd_ctx->
				 PolicyRxHeader.NumDataObjects << 2),
				&fusb_pd_ctx->ProtocolRxBuffer[0]);
		for (i = 0;
		     i < fusb_pd_ctx->PolicyRxHeader.NumDataObjects;
		     i++) {
			for (j = 0; j < 4; j++) {
				fusb_pd_ctx->PolicyRxDataObj[i].byte[j] =
				    fusb_pd_ctx->ProtocolRxBuffer[j + (i << 2)];
			}
		}
	}
	FUSB300ReadFIFO(4, &fusb_pd_ctx->ProtocolCRC[0]);
	FUSB300Read(regStatus0, 2, &fusb_i2c_data->Registers.Status.byte[4]);
	FUSB_LOG("--------****---3fH=%2x,40H=%2x, 41H=%2x\n",
		 fusb_i2c_data->Registers.Status.byte[3],
		 fusb_i2c_data->Registers.Status.byte[4],
		 fusb_i2c_data->Registers.Status.byte[5]);

}

void ProtocolTransmitMessage(void)
{
	int i, j;

	ProtocolFlushTxFIFO();
	ProtocolLoadSOP();
	if ((fusb_pd_ctx->PolicyTxHeader.NumDataObjects == 0)
	    && (fusb_pd_ctx->PolicyTxHeader.MessageType == CMTSoftReset)) {
		fusb_pd_ctx->MessageIDCounter = 0;
		fusb_pd_ctx->MessageID = 0xFF;
		fusb_pd_ctx->SourceCapsUpdated = true;
	}
	fusb_pd_ctx->PolicyTxHeader.MessageID = fusb_pd_ctx->MessageIDCounter;
	fusb_pd_ctx->ProtocolTxBuffer[fusb_pd_ctx->ProtocolTxBytes++] =
	    PACKSYM | (2 + (fusb_pd_ctx->PolicyTxHeader.NumDataObjects << 2));
	fusb_pd_ctx->ProtocolTxBuffer[fusb_pd_ctx->ProtocolTxBytes++] =
	    fusb_pd_ctx->PolicyTxHeader.byte[0];
	fusb_pd_ctx->ProtocolTxBuffer[fusb_pd_ctx->ProtocolTxBytes++] =
	    fusb_pd_ctx->PolicyTxHeader.byte[1];
	if (fusb_pd_ctx->PolicyTxHeader.NumDataObjects > 0) {
		for (i = 0;
		     i < fusb_pd_ctx->PolicyTxHeader.NumDataObjects;
		     i++) {
			for (j = 0; j < 4; j++)
				fusb_pd_ctx->ProtocolTxBuffer
				    [fusb_pd_ctx->ProtocolTxBytes++]
				    = fusb_pd_ctx->PolicyTxDataObj[i].byte[j];
		}
	}
	ProtocolLoadEOP();
	FUSB300WriteFIFO(fusb_pd_ctx->ProtocolTxBytes,
			 &fusb_pd_ctx->ProtocolTxBuffer[0]);
	fusb_i2c_data->Registers.Control.TX_START = 1;
	FUSB300Write(regControl0, 1, &fusb_i2c_data->Registers.Control.byte[0]);
	fusb_i2c_data->Registers.Control.TX_START = 0;
	set_pdtx_state(txBusy);
	set_protocol_state(PRLTxSendingMessage);
	fusb_pd_ctx->protocol_time_count = tBMCTimeout;
	usbpd_start_timer(&fusb_pd_ctx->protocol_hrtimer,
			  fusb_pd_ctx->protocol_time_count);
	StoreUSBPDMessage(fusb_pd_ctx->PolicyTxHeader,
			  &fusb_pd_ctx->PolicyTxDataObj[0], true, 0xE0);
}

void ProtocolSendingMessage(void)
{
	FUSB_LOG("enter %s\n", __func__);
	if (fusb_i2c_data->Registers.Status.I_TXSENT) {
		ProtocolFlushTxFIFO();
		ProtocolVerifyGoodCRC();
		ProtocolIdle();
	} else if (fusb_i2c_data->Registers.Status.I_COLLISION) {
		ProtocolFlushTxFIFO();
		set_pdtx_state(txCollision);
		fusb_pd_ctx->protocol_time_count = tBMCTimeout;
		usbpd_start_timer(&fusb_pd_ctx->protocol_hrtimer,
				  fusb_pd_ctx->protocol_time_count);
		set_protocol_state(PRLRxWait);
	} else if (fusb_pd_ctx->protocol_time_count == 0) {
		ProtocolFlushTxFIFO();
		ProtocolFlushRxFIFO();
		set_pdtx_state(txError);
		set_protocol_state(PRLIdle);
	}
}

void ProtocolVerifyGoodCRC(void)
{
	int i, j;
	u8 data[3];

	FUSB300ReadFIFO(3, &data[0]);
	fusb_pd_ctx->PolicyRxHeader.byte[0] = data[1];
	fusb_pd_ctx->PolicyRxHeader.byte[1] = data[2];
	if ((fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0)
	    && (fusb_pd_ctx->PolicyRxHeader.MessageType == CMTGoodCRC)) {
		FUSB300ReadFIFO(4, &fusb_pd_ctx->ProtocolCRC[0]);
		if (fusb_pd_ctx->PolicyRxHeader.MessageID !=
		    fusb_pd_ctx->MessageIDCounter) {
			StoreUSBPDToken(false, pdtBadMessageID);
			set_pdtx_state(txError);
			set_protocol_state(PRLIdle);
		} else {

			fusb_pd_ctx->MessageIDCounter++;
			fusb_pd_ctx->MessageIDCounter &= 0x07;
			set_protocol_state(PRLIdle);
			set_pdtx_state(txSuccess);
			fusb_pd_ctx->ProtocolMsgTx = true;
			StoreUSBPDMessage(fusb_pd_ctx->PolicyRxHeader,
					  &fusb_pd_ctx->PolicyRxDataObj[0],
					  false, data[0]);
		}
	} else {
		set_protocol_state(PRLIdle);
		set_pdtx_state(txError);
		if ((fusb_pd_ctx->PolicyRxHeader.NumDataObjects == 0)
		    && (fusb_pd_ctx->PolicyRxHeader.MessageType ==
			CMTSoftReset)) {
			FUSB300ReadFIFO(4, &fusb_pd_ctx->ProtocolCRC[0]);
			fusb_pd_ctx->MessageIDCounter = 0;
			fusb_pd_ctx->MessageID = 0xFF;
			fusb_pd_ctx->ProtocolMsgRx = true;
			fusb_pd_ctx->SourceCapsUpdated = true;
		} else if (fusb_pd_ctx->PolicyRxHeader.MessageID !=
			   fusb_pd_ctx->MessageID) {
			FUSB300ReadFIFO(4, &fusb_pd_ctx->ProtocolCRC[0]);
			fusb_pd_ctx->MessageID =
			    fusb_pd_ctx->PolicyRxHeader.MessageID;
			fusb_pd_ctx->ProtocolMsgRx = true;
		}
		if (fusb_pd_ctx->PolicyRxHeader.NumDataObjects > 0) {
			FUSB300ReadFIFO(fusb_pd_ctx->
					PolicyRxHeader.NumDataObjects << 2,
					&fusb_pd_ctx->ProtocolRxBuffer[0]);
			for (i = 0;
			     i < fusb_pd_ctx->PolicyRxHeader.NumDataObjects;
			     i++) {
				for (j = 0; j < 4; j++)
					fusb_pd_ctx->
					    PolicyRxDataObj[i].byte[j] =
					    fusb_pd_ctx->ProtocolRxBuffer[j +
									  (i <<
									   2)];
			}
		}
	}
	FUSB300Read(regInterruptb, 3, &fusb_i2c_data->Registers.Status.byte[3]);
	FUSB_LOG("--------***---------40H=%2x, 41H=%2x\n",
		 fusb_i2c_data->Registers.Status.byte[4],
		 fusb_i2c_data->Registers.Status.byte[5]);
}

void ProtocolLoadSOP(void)
{
	fusb_pd_ctx->ProtocolTxBytes = 0;
	fusb_pd_ctx->ProtocolTxBuffer[fusb_pd_ctx->ProtocolTxBytes++] = SOP1;
	fusb_pd_ctx->ProtocolTxBuffer[fusb_pd_ctx->ProtocolTxBytes++] = SOP1;
	fusb_pd_ctx->ProtocolTxBuffer[fusb_pd_ctx->ProtocolTxBytes++] = SOP1;
	fusb_pd_ctx->ProtocolTxBuffer[fusb_pd_ctx->ProtocolTxBytes++] = SOP2;
}

void ProtocolLoadEOP(void)
{
	fusb_pd_ctx->ProtocolTxBuffer[fusb_pd_ctx->ProtocolTxBytes++] = JAM_CRC;
	fusb_pd_ctx->ProtocolTxBuffer[fusb_pd_ctx->ProtocolTxBytes++] = EOP;
	fusb_pd_ctx->ProtocolTxBuffer[fusb_pd_ctx->ProtocolTxBytes++] = TXOFF;
}

void ProtocolSendHardReset(void)
{
	u8 data;

	data = fusb_i2c_data->Registers.Control.byte[3] | 0x40;
	FUSB300Write(regControl3, 1, &data);
	StoreUSBPDToken(true, pdtHardReset);
}

void ProtocolFlushRxFIFO(void)
{
	u8 data;

	data = fusb_i2c_data->Registers.Control.byte[1];
	data |= 0x04;
	FUSB300Write(regControl1, 1, &data);
}

void ProtocolFlushTxFIFO(void)
{
	u8 data;

	data = fusb_i2c_data->Registers.Control.byte[0];
	data |= 0x40;
	FUSB300Write(regControl0, 1, &data);
}

void ResetProtocolLayer(bool ResetPDLogic)
{
	int i;
	u8 data = 0x02;

	if (ResetPDLogic)
		FUSB300Write(regReset, 1, &data);
	ProtocolFlushRxFIFO();
	ProtocolFlushTxFIFO();
	set_protocol_state(PRLIdle);
	set_pdtx_state(txIdle);
	fusb_pd_ctx->protocol_time_count = 0;
	fusb_pd_ctx->ProtocolTxBytes = 0;
	fusb_pd_ctx->MessageIDCounter = 0;
	fusb_pd_ctx->MessageID = 0xFF;
	fusb_pd_ctx->ProtocolMsgRx = false;
	fusb_pd_ctx->ProtocolMsgTx = false;
	fusb_pd_ctx->USBPDTxFlag = false;
	fusb_pd_ctx->PolicyHasContract = false;
	fusb_pd_ctx->USBPDContract.object = 0;
	fusb_pd_ctx->SourceCapsUpdated = true;
	fusb_pd_ctx->CapsHeaderReceived.word = 0;
	for (i = 0; i < 7; i++)
		fusb_pd_ctx->CapsReceived[i].object = 0;
}

bool StoreUSBPDToken(bool transmitter, enum USBPD_BufferTokens_t token)
{
	u8 header1 = 1;

	if (ClaimBufferSpace(2) == false)
		return false;
	if (transmitter)
		header1 |= 0x40;
	fusb_pd_ctx->USBPDBuf[fusb_pd_ctx->USBPDBufEnd++] = header1;
	fusb_pd_ctx->USBPDBufEnd %= PDBUFSIZE;
	token &= 0x0F;
	fusb_pd_ctx->USBPDBuf[fusb_pd_ctx->USBPDBufEnd++] = token;
	fusb_pd_ctx->USBPDBufEnd %= PDBUFSIZE;
	return true;
}

bool StoreUSBPDMessage(union sopMainHeader_t Header,
		       union doDataObject_t *DataObject, bool transmitter,
		       u8 SOPToken)
{
	int i, j, required;
	u8 header1;

	required = Header.NumDataObjects * 4 + 2 + 2;
	if (ClaimBufferSpace(required) == false)
		return false;
	header1 = (0x1F & (required - 1)) | 0x80;
	if (transmitter)
		header1 |= 0x40;
	fusb_pd_ctx->USBPDBuf[fusb_pd_ctx->USBPDBufEnd++] = header1;
	fusb_pd_ctx->USBPDBufEnd %= PDBUFSIZE;
	SOPToken &= 0xE0;
	SOPToken >>= 5;
	fusb_pd_ctx->USBPDBuf[fusb_pd_ctx->USBPDBufEnd++] = SOPToken;
	fusb_pd_ctx->USBPDBufEnd %= PDBUFSIZE;
	fusb_pd_ctx->USBPDBuf[fusb_pd_ctx->USBPDBufEnd++] = Header.byte[0];
	fusb_pd_ctx->USBPDBufEnd %= PDBUFSIZE;
	fusb_pd_ctx->USBPDBuf[fusb_pd_ctx->USBPDBufEnd++] = Header.byte[1];
	fusb_pd_ctx->USBPDBufEnd %= PDBUFSIZE;
	for (i = 0; i < Header.NumDataObjects; i++) {
		for (j = 0; j < 4; j++) {
			fusb_pd_ctx->USBPDBuf[fusb_pd_ctx->USBPDBufEnd++] =
			    DataObject[i].byte[j];
			fusb_pd_ctx->USBPDBufEnd %= PDBUFSIZE;
		}
	}
	return true;
}

u8 GetNextUSBPDMessageSize(void)
{
	u8 numBytes;

	if (fusb_pd_ctx->USBPDBufStart == fusb_pd_ctx->USBPDBufEnd)
		numBytes = 0;
	else
		numBytes =
		    (fusb_pd_ctx->USBPDBuf[fusb_pd_ctx->USBPDBufStart] & 0x1F) +
		    1;
	return numBytes;
}

u8 GetUSBPDBufferNumBytes(void)
{
	u8 bytes;

	if (fusb_pd_ctx->USBPDBufStart == fusb_pd_ctx->USBPDBufEnd)
		bytes = 0;	/* return 0 */
	else if (fusb_pd_ctx->USBPDBufEnd > fusb_pd_ctx->USBPDBufStart)
		bytes = fusb_pd_ctx->USBPDBufEnd - fusb_pd_ctx->USBPDBufStart;
	else			/* Otherwise it has wrapped... */
		bytes =
		    fusb_pd_ctx->USBPDBufEnd + (PDBUFSIZE -
						fusb_pd_ctx->USBPDBufStart);
	return bytes;
}

bool ClaimBufferSpace(int intReqSize)
{
	int available;
	u8 numBytes;

	if (intReqSize >= PDBUFSIZE)
		return false;
	if (fusb_pd_ctx->USBPDBufStart == fusb_pd_ctx->USBPDBufEnd)
		available = PDBUFSIZE;
	else if (fusb_pd_ctx->USBPDBufStart > fusb_pd_ctx->USBPDBufEnd)
		available =
		    fusb_pd_ctx->USBPDBufStart - fusb_pd_ctx->USBPDBufEnd;
	else
		available =
		    PDBUFSIZE - (fusb_pd_ctx->USBPDBufEnd -
				 fusb_pd_ctx->USBPDBufStart);
	do {
		if (intReqSize >= available) {
			fusb_pd_ctx->USBPDBufOverflow = true;
			numBytes = GetNextUSBPDMessageSize();
			if (numBytes == 0)
				return false;
			available += numBytes;
			fusb_pd_ctx->USBPDBufStart += numBytes;
			fusb_pd_ctx->USBPDBufStart %= PDBUFSIZE;
		} else
			break;
	} while (1);
	return true;
}

void GetUSBPDStatus(unsigned char abytData[])
{
	int i, j;
	int intIndex = 0;

	abytData[intIndex++] = GetUSBPDStatusOverview();
	abytData[intIndex++] = GetUSBPDBufferNumBytes();
	abytData[intIndex++] = fusb_pd_ctx->PolicyState;
	abytData[intIndex++] = fusb_pd_ctx->PolicySubIndex;
	abytData[intIndex++] =
	    (fusb_pd_ctx->ProtocolState << 4) | fusb_pd_ctx->PDTxStatus;
	for (i = 0; i < 4; i++)
		abytData[intIndex++] = fusb_pd_ctx->USBPDContract.byte[i];
	if (fusb_pd_ctx->PolicyIsSource) {
		abytData[intIndex++] = fusb_pd_ctx->CapsHeaderSource.byte[0];
		abytData[intIndex++] = fusb_pd_ctx->CapsHeaderSource.byte[1];
		for (i = 0; i < 7; i++) {
			for (j = 0; j < 4; j++)
				abytData[intIndex++] =
				    fusb_pd_ctx->CapsSource[i].byte[j];
		}
	} else {
		abytData[intIndex++] = fusb_pd_ctx->CapsHeaderReceived.byte[0];
		abytData[intIndex++] = fusb_pd_ctx->CapsHeaderReceived.byte[1];
		for (i = 0; i < 7; i++) {
			for (j = 0; j < 4; j++)
				abytData[intIndex++] =
				    fusb_pd_ctx->CapsReceived[i].byte[j];
		}
	}

	intIndex = 44;
	abytData[intIndex++] = fusb_i2c_data->Registers.DeviceID.byte;
	abytData[intIndex++] = fusb_i2c_data->Registers.Switches.byte[0];
	abytData[intIndex++] = fusb_i2c_data->Registers.Switches.byte[1];
	abytData[intIndex++] = fusb_i2c_data->Registers.Measure.byte;
	abytData[intIndex++] = fusb_i2c_data->Registers.Slice.byte;
	abytData[intIndex++] = fusb_i2c_data->Registers.Control.byte[0];
	abytData[intIndex++] = fusb_i2c_data->Registers.Control.byte[1];
	abytData[intIndex++] = fusb_i2c_data->Registers.Mask.byte;
	abytData[intIndex++] = fusb_i2c_data->Registers.Power.byte;
	abytData[intIndex++] = fusb_i2c_data->Registers.Status.byte[4];
	abytData[intIndex++] = fusb_i2c_data->Registers.Status.byte[5];
	abytData[intIndex++] = fusb_i2c_data->Registers.Status.byte[6];
}

u8 GetUSBPDStatusOverview(void)
{
	unsigned char status = 0;

	if (fusb_i2c_data->USBPDEnabled)
		status |= 0x01;
	if (fusb_i2c_data->USBPDActive)
		status |= 0x02;
	if (fusb_pd_ctx->PolicyIsSource)
		status |= 0x04;
	if (fusb_pd_ctx->PolicyIsDFP)
		status |= 0x08;
	if (fusb_pd_ctx->PolicyHasContract)
		status |= 0x10;
	if (fusb_pd_ctx->SourceCapsUpdated)
		status |= 0x20;

	fusb_pd_ctx->SourceCapsUpdated = false;
	if (fusb_pd_ctx->USBPDBufOverflow)
		status |= 0x80;
	return status;
}

u8 ReadUSBPDBuffer(unsigned char *pData, unsigned char bytesAvail)
{
	u8 i, msgSize, bytesRead;

	bytesRead = 0;
	do {
		msgSize = GetNextUSBPDMessageSize();

		if ((msgSize != 0) && (msgSize <= bytesAvail)) {
			for (i = 0; i < msgSize; i++) {
				*pData++ =
				    fusb_pd_ctx->
				    USBPDBuf[fusb_pd_ctx->USBPDBufStart++];
				fusb_pd_ctx->USBPDBufStart %= PDBUFSIZE;
			}
			bytesAvail -= msgSize;
			bytesRead += msgSize;
		} else
			break;
	} while (1);
	return bytesRead;
}

void EnableUSBPD(void)
{
	bool enabled = fusb_i2c_data->blnSMEnabled;

	if (fusb_i2c_data->USBPDEnabled)
		return;

	DisableFUSB300StateMachine();
	fusb_i2c_data->USBPDEnabled = true;
	if (enabled)
		EnableFUSB300StateMachine();
}

void DisableUSBPD(void)
{
	bool enabled = fusb_i2c_data->blnSMEnabled;

	if (!fusb_i2c_data->USBPDEnabled)
		return;

	DisableFUSB300StateMachine();
	fusb_i2c_data->USBPDEnabled = false;
	if (enabled)
		EnableFUSB300StateMachine();
}

void SendUSBPDMessage(unsigned char *abytData)
{
	int i, j;

	fusb_pd_ctx->PDTransmitHeader.byte[0] = *abytData++;
	fusb_pd_ctx->PDTransmitHeader.byte[1] = *abytData++;
	for (i = 0; i < fusb_pd_ctx->PDTransmitHeader.NumDataObjects; i++) {
		for (j = 0; j < 4; j++)
			fusb_pd_ctx->PDTransmitObjects[i].byte[j] = *abytData++;
	}
	fusb_pd_ctx->USBPDTxFlag = true;
}

void WriteSourceCapabilities(unsigned char *abytData)
{
	int i, j;
	union sopMainHeader_t Header;

	Header.byte[0] = *abytData++;
	Header.byte[1] = *abytData++;
	if ((Header.NumDataObjects > 0)
	    && (Header.MessageType == DMTSourceCapabilities)) {
		fusb_pd_ctx->CapsHeaderSource.word = Header.word;
		for (i = 0; i < fusb_pd_ctx->CapsHeaderSource.NumDataObjects;
		     i++) {
			for (j = 0; j < 4; j++)
				fusb_pd_ctx->CapsSource[i].byte[j] =
				    *abytData++;
		}
		if (fusb_pd_ctx->PolicyIsSource) {
			fusb_pd_ctx->PDTransmitHeader.word =
			    fusb_pd_ctx->CapsHeaderSource.word;
			fusb_pd_ctx->USBPDTxFlag = true;
			fusb_pd_ctx->SourceCapsUpdated = true;
		}
	}
}

void ReadSourceCapabilities(unsigned char *abytData)
{
	int i, j;
	*abytData++ = fusb_pd_ctx->CapsHeaderSource.byte[0];
	*abytData++ = fusb_pd_ctx->CapsHeaderSource.byte[1];
	for (i = 0; i < fusb_pd_ctx->CapsHeaderSource.NumDataObjects; i++) {
		for (j = 0; j < 4; j++)
			*abytData++ = fusb_pd_ctx->CapsSource[i].byte[j];
	}
}

void WriteSinkCapabilities(unsigned char *abytData)
{
	int i, j;
	union sopMainHeader_t Header;

	Header.byte[0] = *abytData++;
	Header.byte[1] = *abytData++;
	if ((Header.NumDataObjects > 0)
	    && (Header.MessageType == DMTSinkCapabilities)) {
		fusb_pd_ctx->CapsHeaderSink.word = Header.word;
		for (i = 0; i < fusb_pd_ctx->CapsHeaderSink.NumDataObjects;
		     i++) {
			for (j = 0; j < 4; j++)
				fusb_pd_ctx->CapsSink[i].byte[j] = *abytData++;
		}
	}
}

void ReadSinkCapabilities(unsigned char *abytData)
{
	int i, j;

	*abytData++ = fusb_pd_ctx->CapsHeaderSink.byte[0];
	*abytData++ = fusb_pd_ctx->CapsHeaderSink.byte[1];
	for (i = 0; i < fusb_pd_ctx->CapsHeaderSink.NumDataObjects; i++) {
		for (j = 0; j < 4; j++)
			*abytData++ = fusb_pd_ctx->CapsSink[i].byte[j];
	}
}

void WriteSinkRequestSettings(unsigned char *abytData)
{
	u32 uintPower;

	fusb_pd_ctx->SinkGotoMinCompatible = *abytData & 0x01 ? true : false;
	fusb_pd_ctx->SinkUSBSuspendOperation = *abytData & 0x02 ? true : false;
	fusb_pd_ctx->SinkUSBCommCapable = *abytData++ & 0x04 ? true : false;
	fusb_pd_ctx->SinkRequestMaxVoltage = (u32) (*abytData++);
	fusb_pd_ctx->SinkRequestMaxVoltage |= ((u32) (*abytData++) << 8);
	uintPower = (u32) (*abytData++);
	uintPower |= ((u32) (*abytData++) << 8);
	uintPower |= ((u32) (*abytData++) << 16);
	uintPower |= ((u32) (*abytData++) << 24);
	fusb_pd_ctx->SinkRequestOpPower = uintPower;
	uintPower = (u32) (*abytData++);
	uintPower |= ((u32) (*abytData++) << 8);
	uintPower |= ((u32) (*abytData++) << 16);
	uintPower |= ((u32) (*abytData++) << 24);
	fusb_pd_ctx->SinkRequestMaxPower = uintPower;
}

void ReadSinkRequestSettings(unsigned char *abytData)
{
	*abytData = fusb_pd_ctx->SinkGotoMinCompatible ? 0x01 : 0;
	*abytData |= fusb_pd_ctx->SinkUSBSuspendOperation ? 0x02 : 0;
	*abytData++ |= fusb_pd_ctx->SinkUSBCommCapable ? 0x04 : 0;
	*abytData++ = (u8) (fusb_pd_ctx->SinkRequestMaxVoltage & 0xFF);
	*abytData++ = (u8) ((fusb_pd_ctx->SinkRequestMaxVoltage & 0xFF) >> 8);
	*abytData++ = (u8) (fusb_pd_ctx->SinkRequestOpPower & 0xFF);
	*abytData++ = (u8) ((fusb_pd_ctx->SinkRequestOpPower >> 8) & 0xFF);
	*abytData++ = (u8) ((fusb_pd_ctx->SinkRequestOpPower >> 16) & 0xFF);
	*abytData++ = (u8) ((fusb_pd_ctx->SinkRequestOpPower >> 24) & 0xFF);
	*abytData++ = (u8) (fusb_pd_ctx->SinkRequestMaxPower & 0xFF);
	*abytData++ = (u8) ((fusb_pd_ctx->SinkRequestMaxPower >> 8) & 0xFF);
	*abytData++ = (u8) ((fusb_pd_ctx->SinkRequestMaxPower >> 16) & 0xFF);
	*abytData++ = (u8) ((fusb_pd_ctx->SinkRequestMaxPower >> 24) & 0xFF);
}

void SendUSBPDHardReset(void)
{
	if (fusb_pd_ctx->PolicyIsSource)
		set_policy_state(peSourceSendHardReset);
	else
		set_policy_state(peSinkSendHardReset);
	set_policy_subindex(0);
	set_pdtx_state(txIdle);

}

#if 0
void InitializeVdmManager(void)
{
	initialize(&vdmm);
#if 0
	/* configure callbacks */
	vdmm.req_id_info = sim_RequestIdentityInfo;
	vdmm.req_svid_info = sim_RequestSvidInfo;
	vdmm.req_modes_info = sim_RequestModesInfo;
	vdmm.enter_mode_result = sim_EnterModeResult;
	vdmm.exit_mode_result = sim_ExitModeResult;
	vdmm.inform_id = sim_InformIdentity;
	vdmm.inform_svids = sim_InformSvids;
	vdmm.inform_modes = sim_InformModes;
	vdmm.inform_attention = sim_InformAttention;
	vdmm.req_mode_entry = sim_ModeEntryRequest;
	vdmm.req_mode_exit = sim_ModeExitRequest;
#endif
}

void convertAndProcessVdmMessage(void)
{
	int i, j;
	u32 vdm_arr[7];

	for (i = 0; i < fusb_pd_ctx->PolicyRxHeader.NumDataObjects; i++) {
		vdm_arr[i] = 0;
		for (j = 0; j < 4; j++) {
			vdm_arr[i] <<= 8;
			vdm_arr[i] &= 0xFFFFFF00;
			vdm_arr[i] |= (fusb_pd_ctx->PolicyRxDataObj[i].byte[j]);
		}
	}

	processVdmMessage(&vdmm, SOP, vdm_arr,
			  fusb_pd_ctx->PolicyRxHeader.NumDataObjects);
}

void sendVdmMessage(VdmManager *vdmm, SopType sop, u32 *arr,
		    unsigned int length)
{
	unsigned int i, j;

	fusb_pd_ctx->PolicyTxHeader.MessageType = DMTVenderDefined;
	fusb_pd_ctx->PolicyTxHeader.NumDataObjects = length;

	for (i = 0; i < fusb_pd_ctx->PolicyTxHeader.NumDataObjects; i++) {
		for (j = 0; j < 4; j++) {
			fusb_pd_ctx->PolicyTxDataObj[i].byte[j] =
			    (arr[i] >> (8 * j));
		}
	}

	set_pdtx_state(txSend);
}

void doVdmCommand(void)
{
	u32 svdm_header_bits;
	StructuredVdmHeader svdm_header;
	unsigned int i;
	unsigned int command;
	unsigned int svid;

	svdm_header_bits = 0;

	command = fusb_pd_ctx->PDTransmitObjects[0].byte[0] & 0x1F;
	svid = 0;
	svid |= (fusb_pd_ctx->PDTransmitObjects[0].byte[3] << 8);
	svid |= (fusb_pd_ctx->PDTransmitObjects[0].byte[2] << 0);

	if (command == 1)
		discoverIdentities(&vdmm, SOP);
	else if (command == DISCOVER_SVIDS)
		discoverSvids(&vdmm, SOP);
	else if (command == DISCOVER_MODES)
		discoverModes(&vdmm, SOP, svid);

}
#endif
