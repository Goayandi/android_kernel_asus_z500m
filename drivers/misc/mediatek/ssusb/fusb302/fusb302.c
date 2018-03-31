#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include "../mu3d/ssusb.h"

#include "fusb302.h"

#define PD_SUPPORT
#include "usbpd.h"

#ifdef PD_SUPPORT
#include "usbpd.h"
#endif

/* for mtk battery driver */
#include "../../../power/mt81xx/mt_charging.h"
#include "../../../power/mt81xx/mt_battery_common.h"

#define FUSB302_I2C_NAME "fusb302"

#define PORT_UFP                0
#define PORT_DFP                1
#define PORT_DRP                2

#define HOST_CUR_NO     0
#define HOST_CUR_USB        1
#define HOST_CUR_1500       2
#define HOST_CUR_3000       3

#define CC_UNKNOWN       4
#define CC_RD_3000      3
#define CC_RD_1500      2
#define CC_RD_DEFAULT       1
#define CC_RA           0

/* Configure Definition */
#define FUSB302_I2C_NUM     1
#define FUSB302_PORT_TYPE   USBTypeC_Source
#define FUSB302_HOST_CUR    HOST_CUR_USB
/*#define FUSB302_PORT_TYPE USBTypeC_DRP*/
/*#define FUDB302_PORT_TYPE USBTypeC_Sink*/

#define FUSB_MS_TO_NS(x) (x * 1000 * 1000)
#define Delay10us(x) udelay(x*10)

#define FUSB302_DEBUG

#ifdef FUSB302_DEBUG
#define FUSB_LOG(fmt, args...)  pr_info("[fusb302]" fmt, ##args)
#else
#define FUSB_LOG(fmt, args...)
#endif

struct fusb302_i2c_data *fusb_i2c_data;
static DECLARE_WAIT_QUEUE_HEAD(fusb_thread_wq);
static int fusb302_probe_finish;

int FUSB300Write(u8 regAddr, u8 length, u8 *data)
{
	return i2c_smbus_write_i2c_block_data(fusb_i2c_data->client, regAddr,
					      length, data);
}

int FUSB300Read(u8 regAddr, u8 length, u8 *data)
{
	return i2c_smbus_read_i2c_block_data(fusb_i2c_data->client, regAddr,
					     length, data);
}

static int FUSB300Int_PIN_LVL(void)
{
	int ret = 0; /*mt_get_gpio_in(GPIO_CC_DECODER_PIN); */

	return ret;
}

void FUSB302_start_timer(struct hrtimer *timer, int ms)
{
	ktime_t ktime;

	ktime = ktime_set(0, FUSB_MS_TO_NS(ms));
	hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
}

enum hrtimer_restart func_hrtimer(struct hrtimer *timer)
{
	if (timer == &fusb_i2c_data->state_hrtimer)
		fusb_i2c_data->state_time_count = 0;
	else if (timer == &fusb_i2c_data->debounce_hrtimer1)
		fusb_i2c_data->debounce_time_count1 = 0;
	else if (timer == &fusb_i2c_data->debounce_hrtimer2)
		fusb_i2c_data->debounce_time_count2 = 0;
	else if (timer == &fusb_i2c_data->toggle_hrtimer)
		fusb_i2c_data->toggle_time_count = 0;

	fusb_i2c_data->state_changed = true;
	wake_up(&fusb_thread_wq);

	return HRTIMER_NORESTART;
}

void wake_up_statemachine(void)
{
	fusb_i2c_data->state_changed = true;
	wake_up_interruptible(&fusb_thread_wq);
}

static void dump_reg(void)
{
	char buf[1024];
	int i, len = 0;
	/*u8 byte = 0; */

	for (i = 0; i < 7; i++) {
		len +=
		    sprintf(buf + len, "%02xH:%02x ", i + regStatus0a,
			    fusb_i2c_data->Registers.Status.byte[i]);
	}
	FUSB_LOG("%s\n", buf);
}


void InitializeFUSB300Variables(struct fusb302_i2c_data *fusb)
{
	fusb->blnSMEnabled = true;
	fusb->blnAccSupport = false;
	fusb->blnSrcPreferred = false;
	fusb->PortType = USBTypeC_DRP;
	fusb->ConnState = Disabled;
	fusb->blnCCPinIsCC1 = false;
	fusb->blnCCPinIsCC2 = false;

	fusb->state_time_count = USHRT_MAX;
	fusb->debounce_time_count1 = USHRT_MAX;
	fusb->debounce_time_count2 = USHRT_MAX;
	fusb->toggle_time_count = USHRT_MAX;
	fusb->CC1TermDeb = CCTypeNone;
	fusb->CC2TermDeb = CCTypeNone;
	fusb->CC1TermAct = fusb->CC1TermDeb;
	fusb->CC2TermAct = fusb->CC2TermDeb;
	fusb->SinkCurrent = utccNone;
	fusb->SourceCurrent = utccDefault;
	fusb->Registers.DeviceID.byte = 0x00;
	fusb->Registers.Switches.byte[0] = 0x03;
	fusb->Registers.Switches.byte[1] = 0x00;
	fusb->Registers.Measure.byte = 0x00;
	fusb->Registers.Slice.byte = SDAC_DEFAULT;
	fusb->Registers.Control.byte[0] = 0x20;
	fusb->Registers.Control.byte[1] = 0x00;
	fusb->Registers.Control.byte[2] = 0x02;
	/**/ fusb->Registers.Control.byte[3] = 0x06;
	/**/ fusb->Registers.Mask.byte = 0x00;
	fusb->Registers.Power.byte = 0x07;
	fusb->Registers.Status.Status = 0;
	fusb->Registers.Status.StatusAdv = 0;
	fusb->Registers.Status.Interrupt1 = 0;
	fusb->Registers.Status.InterruptAdv = 0;
	fusb->USBPDActive = false;
	fusb->USBPDEnabled = true;
	fusb->prswap_time_count = 0;

	 /**/ fusb->state_changed = false;
	hrtimer_init(&fusb->state_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	fusb->state_hrtimer.function = func_hrtimer;

	hrtimer_init(&fusb->debounce_hrtimer1, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	fusb->debounce_hrtimer1.function = func_hrtimer;

	hrtimer_init(&fusb->debounce_hrtimer2, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	fusb->debounce_hrtimer2.function = func_hrtimer;

	hrtimer_init(&fusb->toggle_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	fusb->toggle_hrtimer.function = func_hrtimer;
}

void InitializeFUSB300(void)
{
	FUSB_LOG("enter:%s", __func__);
	FUSB300Read(regDeviceID, 2, &fusb_i2c_data->Registers.DeviceID.byte);
	FUSB300Read(regSlice, 1, &fusb_i2c_data->Registers.Slice.byte);
	fusb_i2c_data->Registers.Mask.byte = 0x44;
	FUSB300Write(regMask, 1, &fusb_i2c_data->Registers.Mask.byte);
	fusb_i2c_data->Registers.Control.dword = 0x06220004;
	switch (fusb_i2c_data->PortType) {
	case USBTypeC_Sink:
		fusb_i2c_data->Registers.Control.MODE = 0x2;
		break;
	case USBTypeC_Source:
		fusb_i2c_data->Registers.Control.MODE = 0x3;
		break;
	default:
		fusb_i2c_data->Registers.Control.MODE = 0x1;
		break;
	}
	FUSB300Write(regControl2, 2, &fusb_i2c_data->Registers.Control.byte[2]);
	fusb_i2c_data->Registers.Control4.TOG_USRC_EXIT = 1;
	FUSB300Write(regControl4, 1, &fusb_i2c_data->Registers.Control4.byte);
	fusb_i2c_data->Registers.Power.byte = 0x01;
	FUSB300Read(regStatus0a, 2, &fusb_i2c_data->Registers.Status.byte[0]);
	FUSB300Read(regStatus0, 2, &fusb_i2c_data->Registers.Status.byte[4]);
	SetStateDelayUnattached();
}

void DisableFUSB300StateMachine(void)
{
	fusb_i2c_data->blnSMEnabled = false;
	SetStateDisabled();
}

void EnableFUSB300StateMachine(void)
{
	InitializeFUSB300();
	fusb_i2c_data->blnSMEnabled = true;
}

void StateMachineFUSB300(void)
{
	FUSB_LOG
	    ("connstate=%d, policyState=%d, protocolState=%d, PDTxStatus=%d\n",
	     fusb_i2c_data->ConnState, fusb_pd_ctx->PolicyState,
	     fusb_pd_ctx->ProtocolState, fusb_pd_ctx->PDTxStatus);

	if (!FUSB300Int_PIN_LVL()) {
		FUSB300Read(regStatus0a, 7,
			    &fusb_i2c_data->Registers.Status.byte[0]);
		dump_reg();
		if (fusb_i2c_data->int_disabled) {
			fusb_i2c_data->int_disabled = 0;
			enable_irq(fusb_i2c_data->eint_num);
		}
	}
#ifdef PD_SUPPORT
	if (fusb_i2c_data->USBPDActive) {
		USBPDProtocol();
		USBPDPolicyEngine();
	}
#endif

	switch (fusb_i2c_data->ConnState) {
	case Disabled:
		StateMachineDisabled();
		break;
	case ErrorRecovery:
		StateMachineErrorRecovery();
		break;
	case Unattached:
		StateMachineUnattached();
		break;
	case AttachWaitSink:
		StateMachineAttachWaitSnk();
		break;
	case AttachedSink:
		StateMachineAttachedSink();
		break;
	case AttachWaitSource:
		StateMachineAttachWaitSrc();
		break;
	case AttachedSource:
		StateMachineAttachedSource();
		break;
	case TrySource:
		StateMachineTrySrc();
		break;
	case TryWaitSink:
		StateMachineTryWaitSnk();
		break;
	case AudioAccessory:
		StateMachineAudioAccessory();
		break;
	case DebugAccessory:
		StateMachineDebugAccessory();
		break;
	case AttachWaitAccessory:
		StateMachineAttachWaitAcc();
		break;
	case PoweredAccessory:
		StateMachinePoweredAccessory();
		break;
	case UnsupportedAccessory:
		StateMachineUnsupportedAccessory();
		break;
	case DelayUnattached:
		StateMachineDelayUnattached();
		break;
	default:
		SetStateDelayUnattached();
		break;
	}
	fusb_i2c_data->Registers.Status.Interrupt1 = 0;
	fusb_i2c_data->Registers.Status.InterruptAdv = 0;
}

void StateMachineDisabled(void)
{
}

void StateMachineErrorRecovery(void)
{
	if (fusb_i2c_data->state_time_count == 0)
		SetStateDelayUnattached();
}

void StateMachineDelayUnattached(void)
{
	if (fusb_i2c_data->state_time_count == 0)
		SetStateUnattached();
}

void StateMachineUnattached(void)
{
	if (BMT_status.TypeC_charger_type > 0) {
	bat_update_TypeC_charger_type(CHARGER_UNKNOWN);
		fusb_pd_ctx->pdpower_record.ObjectPosition_lowbattery = 0;
		fusb_pd_ctx->pdpower_record.usbpdselObjectPosition = 0;
		fusb_pd_ctx->pdpower_record.usbpdselVoltage = 0;
		fusb_pd_ctx->pdpower_record.usbpdselCurrent = 0;
		fusb_pd_ctx->pdpower_record.selVoltage_lowbattery = 0;
		fusb_pd_ctx->pdpower_record.selCurrent_lowbattery = 0;
	}

	fusb_i2c_data->g_peSinkWaitCaps_retry = 0;
	if (fusb_i2c_data->Registers.Status.I_TOGDONE) {
		switch (fusb_i2c_data->Registers.Status.TOGSS) {
		case 0b101:
			fusb_i2c_data->blnCCPinIsCC1 = true;
			fusb_i2c_data->blnCCPinIsCC2 = false;
			SetStateAttachWaitSnk();
			break;
		case 0b110:
			fusb_i2c_data->blnCCPinIsCC1 = false;
			fusb_i2c_data->blnCCPinIsCC2 = true;
			SetStateAttachWaitSnk();
			break;
		case 0b001:
			fusb_i2c_data->blnCCPinIsCC1 = true;
			fusb_i2c_data->blnCCPinIsCC2 = false;
			if ((fusb_i2c_data->PortType == USBTypeC_Sink) &&
				(fusb_i2c_data->blnAccSupport))
				SetStateAttachWaitAcc();
			else
				SetStateAttachWaitSrc();
			break;
		case 0b010:
			fusb_i2c_data->blnCCPinIsCC1 = false;
			fusb_i2c_data->blnCCPinIsCC2 = true;
			if ((fusb_i2c_data->PortType == USBTypeC_Sink) &&
				(fusb_i2c_data->blnAccSupport))
				SetStateAttachWaitAcc();
			else
				SetStateAttachWaitSrc();
			break;
		case 0b111:
			fusb_i2c_data->blnCCPinIsCC1 = false;
			fusb_i2c_data->blnCCPinIsCC2 = false;
			if ((fusb_i2c_data->PortType == USBTypeC_Sink) &&
				(fusb_i2c_data->blnAccSupport))
				SetStateAttachWaitAcc();
			else
				SetStateAttachWaitSrc();
			break;
		default:
			fusb_i2c_data->Registers.Control.TOGGLE = 0;
			FUSB300Write(regControl2, 1,
				&fusb_i2c_data->Registers.Control.byte[2]);
			Delay10us(1);

			fusb_i2c_data->Registers.Control.TOGGLE = 1;
			FUSB300Write(regControl2, 1,
				&fusb_i2c_data->Registers.Control.byte[2]);
			break;
		}
	}
}

void StateMachineAttachWaitSnk(void)
{
	enum CCTermType CCValue = DecodeCCTermination();

	if (fusb_i2c_data->Registers.Switches.MEAS_CC1) {
		if (fusb_i2c_data->CC1TermAct != CCValue) {
			fusb_i2c_data->CC1TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		}
	} else {
		if (fusb_i2c_data->CC2TermAct != CCValue) {
			fusb_i2c_data->CC2TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		}
	}
	if (fusb_i2c_data->debounce_time_count1 == 0) {
		fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
		if ((fusb_i2c_data->CC1TermDeb != fusb_i2c_data->CC1TermAct) ||
		    (fusb_i2c_data->CC2TermDeb != fusb_i2c_data->CC2TermAct)) {
			fusb_i2c_data->debounce_time_count2 =
			    tCCDebounceMin - tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer2,
					    fusb_i2c_data->
					    debounce_time_count2);
		}
		fusb_i2c_data->CC1TermDeb = fusb_i2c_data->CC1TermAct;
		fusb_i2c_data->CC2TermDeb = fusb_i2c_data->CC2TermAct;
	}
	if (fusb_i2c_data->toggle_time_count == 0) {
		if (fusb_i2c_data->Registers.Switches.MEAS_CC1)
			ToggleMeasureCC2();
		else
			ToggleMeasureCC1();
		fusb_i2c_data->toggle_time_count = tFUSB302Toggle;
		FUSB302_start_timer(&fusb_i2c_data->toggle_hrtimer,
				    fusb_i2c_data->toggle_time_count);
	}
	if ((fusb_i2c_data->CC1TermDeb == CCTypeRa) &&
		(fusb_i2c_data->CC2TermDeb == CCTypeRa))
		SetStateDelayUnattached();
	else if (fusb_i2c_data->Registers.Status.VBUSOK &&
		(fusb_i2c_data->debounce_time_count2 == 0)) {
		if ((fusb_i2c_data->CC1TermDeb > CCTypeRa) &&
			(fusb_i2c_data->CC2TermDeb == CCTypeRa)) {
			if ((fusb_i2c_data->PortType == USBTypeC_DRP) &&
				fusb_i2c_data->blnSrcPreferred)
				SetStateTrySrc();
			else {
				fusb_i2c_data->blnCCPinIsCC1 = true;
				fusb_i2c_data->blnCCPinIsCC2 = false;
				/**/ SetStateAttachedSink();
			}
		} else if ((fusb_i2c_data->CC1TermDeb == CCTypeRa) &&
				(fusb_i2c_data->CC2TermDeb > CCTypeRa)) {
			if ((fusb_i2c_data->PortType == USBTypeC_DRP) &&
				fusb_i2c_data->blnSrcPreferred)
				SetStateTrySrc();
			else {
				fusb_i2c_data->blnCCPinIsCC1 = false;
			 fusb_i2c_data->blnCCPinIsCC2 = true;
				SetStateAttachedSink();
			}
		}
	}
}

void StateMachineAttachWaitSrc(void)
{
	enum CCTermType CCValue = DecodeCCTermination();

	if (fusb_i2c_data->Registers.Switches.MEAS_CC1) {
		if (fusb_i2c_data->CC1TermAct != CCValue) {
			fusb_i2c_data->CC1TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		}
	} else {
		if (fusb_i2c_data->CC2TermAct != CCValue) {
			fusb_i2c_data->CC2TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		}
	}
	if (fusb_i2c_data->debounce_time_count1 == 0) {
		fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
		if ((fusb_i2c_data->CC1TermDeb != fusb_i2c_data->CC1TermAct)
		    || (fusb_i2c_data->CC2TermDeb !=
			fusb_i2c_data->CC2TermAct)) {
			fusb_i2c_data->debounce_time_count2 = tCCDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer2,
					    fusb_i2c_data->
					    debounce_time_count2);
		}
		fusb_i2c_data->CC1TermDeb = fusb_i2c_data->CC1TermAct;
		fusb_i2c_data->CC2TermDeb = fusb_i2c_data->CC2TermAct;
	}
	if (fusb_i2c_data->toggle_time_count == 0) {
		if (fusb_i2c_data->Registers.Switches.MEAS_CC1)
			ToggleMeasureCC2();
		else
			ToggleMeasureCC1();
		fusb_i2c_data->toggle_time_count = tFUSB302Toggle;
		FUSB302_start_timer(&fusb_i2c_data->toggle_hrtimer,
				    fusb_i2c_data->toggle_time_count);
	}
	if ((fusb_i2c_data->CC1TermDeb == CCTypeNone) &&
		(fusb_i2c_data->CC2TermDeb == CCTypeNone)) {
		SetStateDelayUnattached();
	} else if ((fusb_i2c_data->CC1TermDeb == CCTypeNone) &&
		(fusb_i2c_data->CC2TermDeb == CCTypeRa)) {
		SetStateDelayUnattached();
	} else if ((fusb_i2c_data->CC1TermDeb == CCTypeRa) &&
		(fusb_i2c_data->CC2TermDeb == CCTypeNone)) {
		SetStateDelayUnattached();
	} else if (fusb_i2c_data->debounce_time_count2 == 0) {
		if ((fusb_i2c_data->CC1TermDeb == CCTypeRa) &&
			(fusb_i2c_data->CC2TermDeb == CCTypeRa))
			SetStateAudioAccessory();
		else if ((fusb_i2c_data->CC1TermDeb > CCTypeRa) &&
			(fusb_i2c_data->CC2TermDeb > CCTypeRa))
			SetStateDebugAccessory();
		else if (fusb_i2c_data->CC1TermDeb > CCTypeRa) {
			fusb_i2c_data->blnCCPinIsCC1 = true;
			fusb_i2c_data->blnCCPinIsCC2 = false;
			SetStateAttachedSrc();
		} else if (fusb_i2c_data->CC2TermDeb > CCTypeRa) {
			fusb_i2c_data->blnCCPinIsCC1 = false;
			fusb_i2c_data->blnCCPinIsCC2 = true;
			SetStateAttachedSrc();
		}
	}
}

void StateMachineAttachWaitAcc(void)
{
	enum CCTermType CCValue = DecodeCCTermination();

	if (fusb_i2c_data->Registers.Switches.MEAS_CC1) {
		if (fusb_i2c_data->CC1TermAct != CCValue) {
			fusb_i2c_data->CC1TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tCCDebounceNom;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		}
	} else {
		if (fusb_i2c_data->CC2TermAct != CCValue) {
			fusb_i2c_data->CC2TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tCCDebounceNom;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		}
	}
	if (fusb_i2c_data->toggle_time_count == 0) {
		if (fusb_i2c_data->Registers.Switches.MEAS_CC1)
			ToggleMeasureCC2();
		else
			ToggleMeasureCC1();
		fusb_i2c_data->toggle_time_count = tFUSB302Toggle;
		FUSB302_start_timer(&fusb_i2c_data->toggle_hrtimer,
				    fusb_i2c_data->toggle_time_count);
	}
	if (fusb_i2c_data->debounce_time_count1 == 0) {
		if ((fusb_i2c_data->CC1TermDeb == CCTypeRa) &&
			(fusb_i2c_data->CC2TermDeb == CCTypeRa))
			SetStateAudioAccessory();
		else if ((fusb_i2c_data->CC1TermDeb > CCTypeRa) &&
			(fusb_i2c_data->CC2TermDeb > CCTypeRa))
			SetStateDebugAccessory();
		else if ((fusb_i2c_data->CC1TermDeb == CCTypeNone) ||
			(fusb_i2c_data->CC2TermDeb == CCTypeNone))
			SetStateDelayUnattached();
		else if ((fusb_i2c_data->CC1TermDeb > CCTypeRa) &&
			(fusb_i2c_data->CC2TermDeb == CCTypeRa)) {
			fusb_i2c_data->blnCCPinIsCC1 = true;
			fusb_i2c_data->blnCCPinIsCC2 = false;
			SetStatePoweredAccessory();
		} else if ((fusb_i2c_data->CC1TermDeb == CCTypeRa) &&
			(fusb_i2c_data->CC2TermDeb > CCTypeRa)) {
			fusb_i2c_data->blnCCPinIsCC1 = true;
			fusb_i2c_data->blnCCPinIsCC2 = false;
			SetStatePoweredAccessory();
		}
	}
}

void StateMachineAttachedSink(void)
{
	enum CCTermType CCValue = DecodeCCTermination();

	if ((fusb_i2c_data->Registers.Status.VBUSOK == false) &&
		(!fusb_i2c_data->prswap_time_count))
		SetStateDelayUnattached();
	else {
		if (fusb_i2c_data->Registers.Switches.MEAS_CC1) {
			if (CCValue != fusb_i2c_data->CC1TermAct) {
				fusb_i2c_data->CC1TermAct = CCValue;
				fusb_i2c_data->debounce_time_count1 =
					tPDDebounceMin;
				FUSB302_start_timer(&fusb_i2c_data->
						    debounce_hrtimer1,
						    fusb_i2c_data->
						    debounce_time_count1);
			} else if (fusb_i2c_data->debounce_time_count1 == 0) {
				fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
				fusb_i2c_data->CC1TermDeb =
					fusb_i2c_data->CC1TermAct;
				UpdateSinkCurrent(fusb_i2c_data->CC1TermDeb);
			}
		} else {
			if (CCValue != fusb_i2c_data->CC2TermAct) {
				fusb_i2c_data->CC2TermAct = CCValue;
				fusb_i2c_data->debounce_time_count1 =
					tPDDebounceMin;
				FUSB302_start_timer(&fusb_i2c_data->
						    debounce_hrtimer1,
						    fusb_i2c_data->
						    debounce_time_count1);
			} else if (fusb_i2c_data->debounce_time_count1 == 0) {
				fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
				fusb_i2c_data->CC2TermDeb =
					fusb_i2c_data->CC2TermAct;
				UpdateSinkCurrent(fusb_i2c_data->CC2TermDeb);
			}
		}
	}
}

void StateMachineAttachedSource(void)
{
	enum CCTermType CCValue = DecodeCCTermination();

	if (fusb_i2c_data->Registers.Switches.MEAS_CC1) {
		if (fusb_i2c_data->CC1TermAct != CCValue) {
			fusb_i2c_data->CC1TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		} else if (fusb_i2c_data->debounce_time_count1 == 0) {
			fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
			fusb_i2c_data->CC1TermDeb = fusb_i2c_data->CC1TermAct;
		}
		if ((fusb_i2c_data->CC1TermDeb == CCTypeNone)
		    && (!fusb_i2c_data->prswap_time_count)) {
			if ((fusb_i2c_data->PortType == USBTypeC_DRP) &&
				fusb_i2c_data->blnSrcPreferred)
				SetStateTryWaitSnk();
			else {
				SetStateDelayUnattached();
				ssusb_mode_switch_typec(0);
			}
		}
	} else {
		if (fusb_i2c_data->CC2TermAct != CCValue) {
			fusb_i2c_data->CC2TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		} else if (fusb_i2c_data->debounce_time_count1 == 0) {
			fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
			fusb_i2c_data->CC2TermDeb = fusb_i2c_data->CC2TermAct;
		}

		if ((fusb_i2c_data->CC2TermDeb == CCTypeNone)
		    && (!fusb_i2c_data->prswap_time_count)) {
			if ((fusb_i2c_data->PortType == USBTypeC_DRP) &&
				fusb_i2c_data->blnSrcPreferred)
				SetStateTryWaitSnk();
			else {
				SetStateDelayUnattached();
				ssusb_mode_switch_typec(0);
			}
		}
	}
}

void StateMachineTryWaitSnk(void)
{
	enum CCTermType CCValue = DecodeCCTermination();

	if (fusb_i2c_data->Registers.Switches.MEAS_CC1) {
		if (fusb_i2c_data->CC1TermAct != CCValue) {
			fusb_i2c_data->CC1TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		}
	} else {

		if (fusb_i2c_data->CC2TermAct != CCValue) {
			fusb_i2c_data->CC2TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		}
	}
	if (fusb_i2c_data->debounce_time_count1 == 0) {
		fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
		if ((fusb_i2c_data->CC1TermDeb != fusb_i2c_data->CC1TermAct)
		    || (fusb_i2c_data->CC2TermDeb !=
			fusb_i2c_data->CC2TermAct)) {
			fusb_i2c_data->debounce_time_count2 =
				tCCDebounceMin - tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer2,
					    fusb_i2c_data->
					    debounce_time_count2);
		}
		fusb_i2c_data->CC1TermDeb = fusb_i2c_data->CC1TermAct;
		fusb_i2c_data->CC2TermDeb = fusb_i2c_data->CC2TermAct;
	}
	if (fusb_i2c_data->toggle_time_count == 0) {
		if (fusb_i2c_data->Registers.Switches.MEAS_CC1)
			ToggleMeasureCC2();
		else
			ToggleMeasureCC1();
		fusb_i2c_data->toggle_time_count = tFUSB302Toggle;
		FUSB302_start_timer(&fusb_i2c_data->toggle_hrtimer,
				    fusb_i2c_data->toggle_time_count);
	}
	if ((fusb_i2c_data->state_time_count == 0) &&
			(fusb_i2c_data->CC1TermDeb == CCTypeRa) &&
			(fusb_i2c_data->CC2TermDeb == CCTypeRa))
		SetStateDelayUnattached();
	else if (fusb_i2c_data->Registers.Status.VBUSOK &&
			(fusb_i2c_data->debounce_time_count2 == 0)) {
		if ((fusb_i2c_data->CC1TermDeb > CCTypeRa) &&
			(fusb_i2c_data->CC2TermDeb == CCTypeRa)) {
			fusb_i2c_data->blnCCPinIsCC1 = true;
			fusb_i2c_data->blnCCPinIsCC2 = false;
			/**/ SetStateAttachedSink();
		} else if ((fusb_i2c_data->CC1TermDeb == CCTypeRa) &&
			(fusb_i2c_data->CC2TermDeb > CCTypeRa)) {
			fusb_i2c_data->blnCCPinIsCC1 = false;
			/**/ fusb_i2c_data->blnCCPinIsCC2 = true;
			SetStateAttachedSink();
		}
	}
}

void StateMachineTrySrc(void)
{
	enum CCTermType CCValue = DecodeCCTermination();

	if (fusb_i2c_data->Registers.Switches.MEAS_CC1) {
		if (fusb_i2c_data->CC1TermAct != CCValue) {
			fusb_i2c_data->CC1TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		}
	} else {

		if (fusb_i2c_data->CC2TermAct != CCValue) {
			fusb_i2c_data->CC2TermAct = CCValue;
			fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
			FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
					    fusb_i2c_data->
					    debounce_time_count1);
		}
	}
	if (fusb_i2c_data->debounce_time_count1 == 0) {
		fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
		fusb_i2c_data->CC1TermDeb = fusb_i2c_data->CC1TermAct;
		fusb_i2c_data->CC2TermDeb = fusb_i2c_data->CC2TermAct;
	}
	if (fusb_i2c_data->toggle_time_count == 0) {
		if (fusb_i2c_data->Registers.Switches.MEAS_CC1)
			ToggleMeasureCC2();
		else
			ToggleMeasureCC1();
		fusb_i2c_data->toggle_time_count = tPDDebounceMax;
		FUSB302_start_timer(&fusb_i2c_data->toggle_hrtimer,
				    fusb_i2c_data->toggle_time_count);
	}
	if ((fusb_i2c_data->CC1TermDeb > CCTypeRa) &&
			((fusb_i2c_data->CC2TermDeb == CCTypeNone) ||
			 (fusb_i2c_data->CC2TermDeb == CCTypeRa))) {
		fusb_i2c_data->blnCCPinIsCC1 = true;
		fusb_i2c_data->blnCCPinIsCC2 = false;
		SetStateAttachedSrc();
	} else if ((fusb_i2c_data->CC2TermDeb > CCTypeRa) &&
			((fusb_i2c_data->CC1TermDeb == CCTypeNone) ||
			 (fusb_i2c_data->CC1TermDeb == CCTypeRa))) {
		fusb_i2c_data->blnCCPinIsCC1 = false;
		fusb_i2c_data->blnCCPinIsCC2 = true;
		SetStateAttachedSrc();
	} else if (fusb_i2c_data->state_time_count == 0)
		SetStateTryWaitSnk();
}

void StateMachineDebugAccessory(void)
{
	enum CCTermType CCValue = DecodeCCTermination();

	if (fusb_i2c_data->CC1TermAct != CCValue) {
		fusb_i2c_data->CC1TermAct = CCValue;
		fusb_i2c_data->debounce_time_count1 = tCCDebounceMin;
		FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
				    fusb_i2c_data->debounce_time_count1);
	} else if (fusb_i2c_data->debounce_time_count1 == 0) {
		fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
		fusb_i2c_data->CC1TermDeb = fusb_i2c_data->CC1TermAct;
	}
	if (fusb_i2c_data->CC1TermDeb == CCTypeNone)
		SetStateDelayUnattached();

}

void StateMachineAudioAccessory(void)
{
	enum CCTermType CCValue = DecodeCCTermination();

	if (fusb_i2c_data->CC1TermAct != CCValue) {
		fusb_i2c_data->CC1TermAct = CCValue;
		fusb_i2c_data->debounce_time_count1 = tCCDebounceMin;
		FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
				    fusb_i2c_data->debounce_time_count1);
	} else if (fusb_i2c_data->debounce_time_count1 == 0) {
		fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
		fusb_i2c_data->CC1TermDeb = fusb_i2c_data->CC1TermAct;
	}
	if (fusb_i2c_data->CC1TermDeb == CCTypeNone)
		SetStateDelayUnattached();
}

void StateMachinePoweredAccessory(void)
{
	enum CCTermType CCValue = DecodeCCTermination();

	if (fusb_i2c_data->CC1TermAct != CCValue) {
		fusb_i2c_data->CC1TermAct = CCValue;
		fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
		FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
				    fusb_i2c_data->debounce_time_count1);
	} else if (fusb_i2c_data->debounce_time_count1 == 0) {
		fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
		fusb_i2c_data->CC1TermDeb = fusb_i2c_data->CC1TermAct;
	}
	if (fusb_i2c_data->CC1TermDeb == CCTypeNone)
		SetStateDelayUnattached();
	else if (fusb_i2c_data->state_time_count == 0)
		SetStateUnsupportedAccessory();
}

void StateMachineUnsupportedAccessory(void)
{
	enum CCTermType CCValue = DecodeCCTermination();

	if (fusb_i2c_data->CC1TermAct != CCValue) {
		fusb_i2c_data->CC1TermAct = CCValue;
		fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
		FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
				    fusb_i2c_data->debounce_time_count1);
	} else if (fusb_i2c_data->debounce_time_count1 == 0) {
		fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
		fusb_i2c_data->CC1TermDeb = fusb_i2c_data->CC1TermAct;
	}
	if (fusb_i2c_data->CC1TermDeb == CCTypeNone)
		SetStateDelayUnattached();
}

void SetStateDisabled(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Power.PWR = 0x01;
	fusb_i2c_data->Registers.Control.TOGGLE = 0;
	fusb_i2c_data->Registers.Control.HOST_CUR = 0x00;
	fusb_i2c_data->Registers.Switches.word = 0x0000;
	FUSB300Write(regPower, 1,
		&fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regControl0, 3,
		&fusb_i2c_data->Registers.Control.byte[0]);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
#ifdef PD_SUPPORT
	USBPDDisable(false);
#endif
	fusb_i2c_data->CC1TermDeb = CCTypeNone;
	fusb_i2c_data->CC2TermDeb = CCTypeNone;
	fusb_i2c_data->CC1TermAct = fusb_i2c_data->CC1TermDeb;
	fusb_i2c_data->CC2TermAct = fusb_i2c_data->CC2TermDeb;
	fusb_i2c_data->blnCCPinIsCC1 = false;
	fusb_i2c_data->blnCCPinIsCC2 = false;
	fusb_i2c_data->ConnState = Disabled;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;

}

void SetStateErrorRecovery(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Power.PWR = 0x01;
	fusb_i2c_data->Registers.Control.TOGGLE = 0;
	fusb_i2c_data->Registers.Control.HOST_CUR = 0x00;
	fusb_i2c_data->Registers.Switches.word = 0x0000;
	FUSB300Write(regPower, 1,
		&fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regControl0, 3,
		&fusb_i2c_data->Registers.Control.byte[0]);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
#ifdef PD_SUPPORT
	USBPDDisable(false);
#endif
	fusb_i2c_data->CC1TermDeb = CCTypeNone;
	fusb_i2c_data->CC2TermDeb = CCTypeNone;
	fusb_i2c_data->CC1TermAct = fusb_i2c_data->CC1TermDeb;
	fusb_i2c_data->CC2TermAct = fusb_i2c_data->CC2TermDeb;
	fusb_i2c_data->blnCCPinIsCC1 = false;
	fusb_i2c_data->blnCCPinIsCC2 = false;
	fusb_i2c_data->ConnState = ErrorRecovery;
	fusb_i2c_data->state_time_count = tErrorRecovery;
	FUSB302_start_timer(&fusb_i2c_data->state_hrtimer,
			    fusb_i2c_data->state_time_count);
	fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;
	wake_up_statemachine();
}

void SetStateDelayUnattached(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Power.PWR = 0x01;
	fusb_i2c_data->Registers.Control.TOGGLE = 0;
	fusb_i2c_data->Registers.Control.HOST_CUR = 0x00;
	fusb_i2c_data->Registers.Switches.word = 0x0000;
	FUSB300Write(regPower, 1,
		&fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regControl0, 3,
		&fusb_i2c_data->Registers.Control.byte[0]);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
#ifdef PD_SUPPORT
	USBPDDisable(false);
#endif
	fusb_i2c_data->CC1TermDeb = CCTypeNone;
	fusb_i2c_data->CC2TermDeb = CCTypeNone;
	fusb_i2c_data->CC1TermAct = fusb_i2c_data->CC1TermDeb;
	fusb_i2c_data->CC2TermAct = fusb_i2c_data->CC2TermDeb;
	fusb_i2c_data->blnCCPinIsCC1 = false;
	fusb_i2c_data->blnCCPinIsCC2 = false;
	fusb_i2c_data->ConnState = DelayUnattached;
	fusb_i2c_data->state_time_count = 10;
	FUSB302_start_timer(&fusb_i2c_data->state_hrtimer,
			    fusb_i2c_data->state_time_count);
	fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;
	wake_up_statemachine();
}

void SetStateUnattached(void)
{

	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Control.HOST_CUR = 0x01;
	fusb_i2c_data->Registers.Control.TOGGLE = 1;
	if ((fusb_i2c_data->PortType == USBTypeC_DRP)
		|| (fusb_i2c_data->blnAccSupport))
		fusb_i2c_data->Registers.Control.MODE = 0x1;
	else if (fusb_i2c_data->PortType == USBTypeC_Source)
		fusb_i2c_data->Registers.Control.MODE = 0x3;
	else
		fusb_i2c_data->Registers.Control.MODE = 0x2;
	fusb_i2c_data->Registers.Switches.word = 0x0003;
	fusb_i2c_data->Registers.Power.PWR = 0x07;
	fusb_i2c_data->Registers.Measure.MDAC = MDAC_2P05V;
	FUSB300Write(regPower, 1, &fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regControl0, 3,
		&fusb_i2c_data->Registers.Control.byte[0]);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	FUSB300Write(regMeasure, 1,
		&fusb_i2c_data->Registers.Measure.byte);
#ifdef PD_SUPPORT
	USBPDDisable(false);
#endif
	fusb_i2c_data->ConnState = Unattached;
	fusb_i2c_data->SinkCurrent = utccNone;
	fusb_i2c_data->CC1TermDeb = CCTypeNone;
	fusb_i2c_data->CC2TermDeb = CCTypeNone;
	fusb_i2c_data->CC1TermAct = fusb_i2c_data->CC1TermDeb;
	fusb_i2c_data->CC2TermAct = fusb_i2c_data->CC2TermDeb;
	fusb_i2c_data->blnCCPinIsCC1 = false;
	fusb_i2c_data->blnCCPinIsCC2 = false;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = USHRT_MAX;
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;
	wake_up_statemachine();
}

void SetStateAttachWaitSnk(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Power.PWR = 0x07;
	fusb_i2c_data->Registers.Switches.word = 0x0003;
	if (fusb_i2c_data->blnCCPinIsCC1)
		fusb_i2c_data->Registers.Switches.MEAS_CC1 = 1;
	else
		fusb_i2c_data->Registers.Switches.MEAS_CC2 = 1;
	fusb_i2c_data->Registers.Measure.MDAC = MDAC_2P05V;
	fusb_i2c_data->Registers.Control.HOST_CUR = 0x00;
	fusb_i2c_data->Registers.Control.TOGGLE = 0;
	FUSB300Write(regPower, 1,
		&fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	FUSB300Write(regMeasure, 1,
		&fusb_i2c_data->Registers.Measure.byte);
	FUSB300Write(regControl0, 3,
		&fusb_i2c_data->Registers.Control.byte[0]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2,
		&fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = AttachWaitSink;
	fusb_i2c_data->SinkCurrent = utccNone;
	if (fusb_i2c_data->Registers.Switches.MEAS_CC1) {
		fusb_i2c_data->CC1TermAct = DecodeCCTermination();
		fusb_i2c_data->CC2TermAct = CCTypeNone;
	} else {
		fusb_i2c_data->CC1TermAct = CCTypeNone;
		fusb_i2c_data->CC2TermAct = DecodeCCTermination();
	}
	fusb_i2c_data->CC1TermDeb = CCTypeNone;
	fusb_i2c_data->CC2TermDeb = CCTypeNone;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = tPDDebounceMax;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = tFUSB302Toggle;
	FUSB302_start_timer(&fusb_i2c_data->toggle_hrtimer,
			    fusb_i2c_data->toggle_time_count);
	wake_up_statemachine();
}

void SetStateAttachWaitSrc(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Power.PWR = 0x07;
	fusb_i2c_data->Registers.Switches.word = 0x0000;
	if (fusb_i2c_data->blnCCPinIsCC1)
		fusb_i2c_data->Registers.Switches.word = 0x0044;
	else
		fusb_i2c_data->Registers.Switches.word = 0x0088;
	fusb_i2c_data->SourceCurrent = utccDefault;
	UpdateSourcePowerMode();
	fusb_i2c_data->Registers.Control.TOGGLE = 0;
	FUSB300Write(regPower, 1,
		&fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	FUSB300Write(regMeasure, 1,
		&fusb_i2c_data->Registers.Measure.byte);
	FUSB300Write(regControl2, 1,
		&fusb_i2c_data->Registers.Control.byte[2]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2,
		&fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = AttachWaitSource;
	fusb_i2c_data->SinkCurrent = utccNone;
	if (fusb_i2c_data->Registers.Switches.MEAS_CC1) {
		fusb_i2c_data->CC1TermAct = DecodeCCTermination();
		fusb_i2c_data->CC2TermAct = CCTypeNone;
	} else {
		fusb_i2c_data->CC1TermAct = CCTypeNone;
		fusb_i2c_data->CC2TermAct = DecodeCCTermination();
	}
	fusb_i2c_data->CC1TermDeb = CCTypeRa;
	fusb_i2c_data->CC2TermDeb = CCTypeRa;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = tDRP;
	FUSB302_start_timer(&fusb_i2c_data->toggle_hrtimer,
			    fusb_i2c_data->toggle_time_count);
	wake_up_statemachine();
}

void SetStateAttachWaitAcc(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Power.PWR = 0x07;
	fusb_i2c_data->Registers.Switches.word = 0x0044;
	UpdateSourcePowerMode();
	fusb_i2c_data->Registers.Control.TOGGLE = 0;
	FUSB300Write(regPower, 1,
		&fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	FUSB300Write(regMeasure, 1,
		&fusb_i2c_data->Registers.Measure.byte);
	FUSB300Write(regControl2, 1,
		&fusb_i2c_data->Registers.Control.byte[2]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2,
		&fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = AttachWaitAccessory;
	fusb_i2c_data->SinkCurrent = utccNone;
	fusb_i2c_data->CC1TermAct = DecodeCCTermination();
	fusb_i2c_data->CC2TermAct = CCTypeNone;
	fusb_i2c_data->CC1TermDeb = CCTypeNone;
	fusb_i2c_data->CC2TermDeb = CCTypeNone;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = tCCDebounceNom;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = tFUSB302Toggle;
	FUSB302_start_timer(&fusb_i2c_data->toggle_hrtimer,
			    fusb_i2c_data->toggle_time_count);
	wake_up_statemachine();
}

void SetStateAttachedSrc(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 1;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->SourceCurrent = utccDefault;
	UpdateSourcePowerMode();
	if (fusb_i2c_data->blnCCPinIsCC1 == true)
		fusb_i2c_data->Registers.Switches.word = 0x0064;
	else
		fusb_i2c_data->Registers.Switches.word = 0x0098;
	fusb_i2c_data->Registers.Power.PWR = 0x07;
#ifdef PD_SUPPORT
	 /* disable pd if as src, (device doesn't support pd) */
	/* USBPDEnable(false, true); */
#endif
	FUSB300Write(regPower, 1,
		&fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2,
		&fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = AttachedSource;
	fusb_i2c_data->SinkCurrent = utccNone;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;
	wake_up_statemachine();

	ssusb_mode_switch_typec(1);
}

void SetStateAttachedSink(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Power.PWR = 0x07;
	fusb_i2c_data->Registers.Control.HOST_CUR = 0x00;
	fusb_i2c_data->Registers.Measure.MDAC = MDAC_2P05V;
	fusb_i2c_data->Registers.Switches.word = 0x0003;
	if (fusb_i2c_data->blnCCPinIsCC1)
		fusb_i2c_data->Registers.Switches.MEAS_CC1 = 1;
	else
		fusb_i2c_data->Registers.Switches.MEAS_CC2 = 1;
#ifdef PD_SUPPORT
	USBPDEnable(false, false);
#endif
	FUSB300Write(regPower, 1, &fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regControl0, 1,
		&fusb_i2c_data->Registers.Control.byte[0]);
	FUSB300Write(regMeasure, 1, &fusb_i2c_data->Registers.Measure.byte);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2, &fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = AttachedSink;
	fusb_i2c_data->SinkCurrent = utccDefault;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;
	wake_up_statemachine();
}

void RoleSwapToAttachedSink(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Control.HOST_CUR = 0x00;
	fusb_i2c_data->Registers.Measure.MDAC = MDAC_2P05V;
	if (fusb_i2c_data->blnCCPinIsCC1) {
		fusb_i2c_data->Registers.Switches.PU_EN1 = 0;
		fusb_i2c_data->Registers.Switches.PDWN1 = 1;
		fusb_i2c_data->CC1TermAct = CCTypeRa;
		fusb_i2c_data->CC1TermDeb = CCTypeRa;
	} else {
		fusb_i2c_data->Registers.Switches.PU_EN2 = 0;
		fusb_i2c_data->Registers.Switches.PDWN2 = 1;
		fusb_i2c_data->CC2TermAct = CCTypeRa;
		fusb_i2c_data->CC2TermDeb = CCTypeRa;
	}
	FUSB300Write(regControl0, 1,
		&fusb_i2c_data->Registers.Control.byte[0]);
	FUSB300Write(regMeasure, 1, &fusb_i2c_data->Registers.Measure.byte);
	FUSB300Write(regSwitches0, 1,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2, &fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = AttachedSink;
	fusb_i2c_data->SinkCurrent = utccNone;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;
	wake_up_statemachine();
}

void RoleSwapToAttachedSource(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 1;
	fusb_i2c_data->VBUS_12V_EN = 0;
	UpdateSourcePowerMode();
	if (fusb_i2c_data->blnCCPinIsCC1) {
		fusb_i2c_data->Registers.Switches.PU_EN1 = 1;
		fusb_i2c_data->Registers.Switches.PDWN1 = 0;
		fusb_i2c_data->CC1TermAct = CCTypeNone;
		fusb_i2c_data->CC1TermDeb = CCTypeNone;
	} else {
		fusb_i2c_data->Registers.Switches.PU_EN2 = 1;
		fusb_i2c_data->Registers.Switches.PDWN2 = 0;
		fusb_i2c_data->CC2TermAct = CCTypeNone;
		fusb_i2c_data->CC2TermDeb = CCTypeNone;
	}
	FUSB300Write(regSwitches0, 1,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2, &fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = AttachedSource;
	fusb_i2c_data->SinkCurrent = utccNone;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;
	wake_up_statemachine();
}

void SetStateTryWaitSnk(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Switches.word = 0x0007;
	fusb_i2c_data->Registers.Power.PWR = 0x07;
	fusb_i2c_data->Registers.Measure.MDAC = MDAC_2P05V;
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	FUSB300Write(regPower, 1, &fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regMeasure, 1, &fusb_i2c_data->Registers.Measure.byte);
	Delay10us(25);
	FUSB300Read(regStatus0, 2, &fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = TryWaitSink;
	fusb_i2c_data->SinkCurrent = utccNone;
	if (fusb_i2c_data->Registers.Switches.MEAS_CC1) {
		fusb_i2c_data->CC1TermAct = DecodeCCTermination();
		fusb_i2c_data->CC2TermAct = CCTypeNone;
	} else {
		fusb_i2c_data->CC1TermAct = CCTypeNone;
		fusb_i2c_data->CC2TermAct = DecodeCCTermination();
	}
	fusb_i2c_data->CC1TermDeb = CCTypeNone;
	fusb_i2c_data->CC2TermDeb = CCTypeNone;
	fusb_i2c_data->state_time_count = tDRPTryWait;
	FUSB302_start_timer(&fusb_i2c_data->state_hrtimer,
			    fusb_i2c_data->state_time_count);
	fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = tFUSB302Toggle;
	FUSB302_start_timer(&fusb_i2c_data->toggle_hrtimer,
			    fusb_i2c_data->toggle_time_count);
	wake_up_statemachine();
}

void SetStateTrySrc(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->SourceCurrent = utccDefault;
	fusb_i2c_data->Registers.Power.PWR = 0x07;
	fusb_i2c_data->Registers.Switches.word = 0x0000;
	if (fusb_i2c_data->blnCCPinIsCC1) {
		fusb_i2c_data->Registers.Switches.PU_EN1 = 1;
		fusb_i2c_data->Registers.Switches.MEAS_CC1 = 1;
	} else {
		fusb_i2c_data->Registers.Switches.PU_EN2 = 1;
		fusb_i2c_data->Registers.Switches.MEAS_CC2 = 1;
	}
	UpdateSourcePowerMode();
	FUSB300Write(regPower, 1, &fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2, &fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = TrySource;
	fusb_i2c_data->SinkCurrent = utccNone;
	fusb_i2c_data->blnCCPinIsCC1 = false;
	fusb_i2c_data->blnCCPinIsCC2 = false;
	if (fusb_i2c_data->Registers.Switches.MEAS_CC1) {
		fusb_i2c_data->CC1TermAct = DecodeCCTermination();
		fusb_i2c_data->CC2TermAct = CCTypeNone;
	} else {
		fusb_i2c_data->CC1TermAct = CCTypeNone;
		fusb_i2c_data->CC2TermAct = DecodeCCTermination();
	}
	fusb_i2c_data->CC1TermDeb = CCTypeNone;
	fusb_i2c_data->CC2TermDeb = CCTypeNone;
	fusb_i2c_data->state_time_count = tDRPTry;
	FUSB302_start_timer(&fusb_i2c_data->state_hrtimer,
			    fusb_i2c_data->state_time_count);
	fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = tPDDebounceMax;
	FUSB302_start_timer(&fusb_i2c_data->toggle_hrtimer,
			    fusb_i2c_data->toggle_time_count);
	wake_up_statemachine();
}

void SetStateDebugAccessory(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Power.PWR = 0x07;
	fusb_i2c_data->Registers.Switches.word = 0x0044;
	UpdateSourcePowerMode();
	FUSB300Write(regPower, 1, &fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	FUSB300Write(regMeasure, 1, &fusb_i2c_data->Registers.Measure.byte);
	Delay10us(25);
	FUSB300Read(regStatus0, 2, &fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = DebugAccessory;
	fusb_i2c_data->SinkCurrent = utccNone;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = tCCDebounceNom;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;
	wake_up_statemachine();
}

void SetStateAudioAccessory(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->Registers.Power.PWR = 0x07;
	fusb_i2c_data->Registers.Switches.word = 0x0044;
	UpdateSourcePowerMode();
	FUSB300Write(regPower, 1, &fusb_i2c_data->Registers.Power.byte);
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	FUSB300Write(regMeasure, 1, &fusb_i2c_data->Registers.Measure.byte);
	Delay10us(25);
	FUSB300Read(regStatus0, 2, &fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = AudioAccessory;
	fusb_i2c_data->SinkCurrent = utccNone;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = tCCDebounceNom;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;
	wake_up_statemachine();
}

void SetStatePoweredAccessory(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->SourceCurrent = utcc1p5A;
	UpdateSourcePowerMode();
	if (fusb_i2c_data->blnCCPinIsCC1 == true)
		fusb_i2c_data->Registers.Switches.word = 0x0064;
	else
		fusb_i2c_data->Registers.Switches.word = 0x0098;
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2,
		&fusb_i2c_data->Registers.Status.byte[4]);
		fusb_i2c_data->ConnState = PoweredAccessory;
	fusb_i2c_data->SinkCurrent = utccNone;
	fusb_i2c_data->state_time_count = tAMETimeout;
	FUSB302_start_timer(&fusb_i2c_data->state_hrtimer,
			    fusb_i2c_data->state_time_count);
	fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;
	wake_up_statemachine();
}

void SetStateUnsupportedAccessory(void)
{
	FUSB_LOG("enter:%s\n", __func__);
	fusb_i2c_data->VBUS_5V_EN = 0;
	fusb_i2c_data->VBUS_12V_EN = 0;
	fusb_i2c_data->SourceCurrent = utccDefault;
	UpdateSourcePowerMode();
	fusb_i2c_data->Registers.Switches.VCONN_CC1 = 0;
	fusb_i2c_data->Registers.Switches.VCONN_CC2 = 0;
	FUSB300Write(regSwitches0, 2,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2,
		&fusb_i2c_data->Registers.Status.byte[4]);
	fusb_i2c_data->ConnState = UnsupportedAccessory;
	fusb_i2c_data->SinkCurrent = utccNone;
	fusb_i2c_data->state_time_count = USHRT_MAX;
	fusb_i2c_data->debounce_time_count1 = tPDDebounceMin;
	FUSB302_start_timer(&fusb_i2c_data->debounce_hrtimer1,
			    fusb_i2c_data->debounce_time_count1);
	fusb_i2c_data->debounce_time_count2 = USHRT_MAX;
	fusb_i2c_data->toggle_time_count = USHRT_MAX;
	wake_up_statemachine();
}

void UpdateSourcePowerMode(void)
{
	switch (fusb_i2c_data->SourceCurrent) {
	case utccDefault:
		fusb_i2c_data->Registers.Measure.MDAC = MDAC_1P6V;
		fusb_i2c_data->Registers.Control.HOST_CUR = 0x01;
		break;
	case utcc1p5A:
		fusb_i2c_data->Registers.Measure.MDAC = MDAC_1P6V;
		fusb_i2c_data->Registers.Control.HOST_CUR = 0x02;
		break;
	case utcc3p0A:
		fusb_i2c_data->Registers.Measure.MDAC = MDAC_2P6V;
		fusb_i2c_data->Registers.Control.HOST_CUR = 0x03;
		break;
	default:
		fusb_i2c_data->Registers.Measure.MDAC = MDAC_1P6V;
		fusb_i2c_data->Registers.Control.HOST_CUR = 0x00;
		break;
	}
	FUSB300Write(regMeasure, 1, &fusb_i2c_data->Registers.Measure.byte);
	FUSB300Write(regControl0, 1, &fusb_i2c_data->Registers.Control.byte[0]);
}


void ToggleMeasureCC1(void)
{
	fusb_i2c_data->Registers.Switches.PU_EN1 =
		fusb_i2c_data->Registers.Switches.PU_EN2;
	fusb_i2c_data->Registers.Switches.PU_EN2 = 0;
	fusb_i2c_data->Registers.Switches.MEAS_CC1 = 1;
	fusb_i2c_data->Registers.Switches.MEAS_CC2 = 0;
	FUSB300Write(regSwitches0, 1,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2, &fusb_i2c_data->Registers.Status.byte[4]);
}

void ToggleMeasureCC2(void)
{
	fusb_i2c_data->Registers.Switches.PU_EN2 =
		fusb_i2c_data->Registers.Switches.PU_EN1;
	fusb_i2c_data->Registers.Switches.PU_EN1 = 0;
	fusb_i2c_data->Registers.Switches.MEAS_CC1 = 0;
	fusb_i2c_data->Registers.Switches.MEAS_CC2 = 1;
	FUSB300Write(regSwitches0, 1,
		&fusb_i2c_data->Registers.Switches.byte[0]);
	Delay10us(25);
	FUSB300Read(regStatus0, 2, &fusb_i2c_data->Registers.Status.byte[4]);
}

enum CCTermType DecodeCCTermination(void)
{
	enum CCTermType Termination = CCTypeNone;

	if (fusb_i2c_data->Registers.Status.COMP == 0) {
		switch (fusb_i2c_data->Registers.Status.BC_LVL) {
		case 0b00:
			Termination = CCTypeRa;
			break;
		case 0b01:
			Termination = CCTypeRdUSB;
			break;
		case 0b10:
			Termination = CCTypeRd1p5;
			break;
		default:
			Termination = CCTypeRd3p0;
			break;
		}
	}
	return Termination;
}

void UpdateSinkCurrent(enum CCTermType Termination)
{
	switch (Termination) {
	case CCTypeRdUSB:
		fusb_i2c_data->SinkCurrent = utccDefault;
		wake_up_bat();
		pr_info("%s: CCTypeRdUSB\n", __func__);
		break;
	case CCTypeRa:
		fusb_i2c_data->SinkCurrent = utccDefault;
		pr_info("%s: CCTypeRa\n", __func__);
		break;
	case CCTypeRd1p5:
		fusb_i2c_data->SinkCurrent = utcc1p5A;
		bat_update_TypeC_charger_type(TYPEC_1_5A_CHARGER);
		pr_info("%s: CCTypeRd1p5\n", __func__);
		break;
	case CCTypeRd3p0:
		fusb_i2c_data->SinkCurrent = utcc3p0A;
		bat_update_TypeC_charger_type(TYPEC_3A_CHARGER);
		pr_info("%s: CCTypeRd3p0\n", __func__);
		break;
	default:
		fusb_i2c_data->SinkCurrent = utccNone;
		pr_info("%s: utccNone\n", __func__);
		break;
	}
}

void ConfigurePortType(unsigned char Control)
{
	unsigned char value;

	DisableFUSB300StateMachine();
	value = Control & 0x03;
	switch (value) {
	case 1:
		fusb_i2c_data->PortType = USBTypeC_Source;
		break;
	case 2:
		fusb_i2c_data->PortType = USBTypeC_DRP;
		break;
	default:
		fusb_i2c_data->PortType = USBTypeC_Sink;
		break;
	}
	if (Control & 0x04)
		fusb_i2c_data->blnAccSupport = true;
	else
		fusb_i2c_data->blnAccSupport = false;
	if (Control & 0x08)
		fusb_i2c_data->blnSrcPreferred = true;
	else
		fusb_i2c_data->blnSrcPreferred = false;
	value = (Control & 0x30) >> 4;
	switch (value) {
	case 1:
		fusb_i2c_data->SourceCurrent = utccDefault;
		break;
	case 2:
		fusb_i2c_data->SourceCurrent = utcc1p5A;
		break;
	case 3:
		fusb_i2c_data->SourceCurrent = utcc3p0A;
		break;
	default:
		fusb_i2c_data->SourceCurrent = utccNone;
		break;
	}
	if (Control & 0x80)
		EnableFUSB300StateMachine();
}

void UpdateCurrentAdvert(unsigned char Current)
{
	switch (Current) {
	case 1:
		fusb_i2c_data->SourceCurrent = utccDefault;
		break;
	case 2:
		fusb_i2c_data->SourceCurrent = utcc1p5A;
		break;
	case 3:
		fusb_i2c_data->SourceCurrent = utcc3p0A;
		break;
	default:
		fusb_i2c_data->SourceCurrent = utccNone;
		break;
	}
	if (fusb_i2c_data->ConnState == AttachedSource)
		UpdateSourcePowerMode();
}

void GetFUSB300TypeCStatus(unsigned char abytData[])
{
	int intIndex = 0;

	abytData[intIndex++] = GetTypeCSMControl();
	abytData[intIndex++] = fusb_i2c_data->ConnState & 0xFF;
	abytData[intIndex++] = GetCCTermination();
	abytData[intIndex++] = fusb_i2c_data->SinkCurrent;
}

unsigned char GetTypeCSMControl(void)
{
	unsigned char status = 0;

	status |= (fusb_i2c_data->PortType & 0x03);
	switch (fusb_i2c_data->PortType) {
	case USBTypeC_Source:
		status |= 0x01;
		break;
	case USBTypeC_DRP:
		status |= 0x02;
		break;
	default:
		break;
	}
	if (fusb_i2c_data->blnAccSupport)
		status |= 0x04;
	if (fusb_i2c_data->blnSrcPreferred)
		status |= 0x08;
	status |= (fusb_i2c_data->SourceCurrent << 4);
	if (fusb_i2c_data->blnSMEnabled)
		status |= 0x80;
	return status;
}

unsigned char GetCCTermination(void)
{
	unsigned char status = 0;

	status |= (fusb_i2c_data->CC1TermDeb & 0x07);
	status |= ((fusb_i2c_data->CC2TermDeb & 0x07) << 4);
	return status;
}

static irqreturn_t cc_eint_interrupt_handler(int irq_num, void *fusb)
{
	disable_irq_nosync(fusb_i2c_data->eint_num);
	fusb_i2c_data->int_disabled = 1;
	FUSB_LOG("%s\n", __func__);
	wake_up_statemachine();
	/* mt_eint_unmask(CUST_EINT_CC_DECODER_NUM); */
	return IRQ_HANDLED;
}

static void fusb_eint_work(struct work_struct *work)
{
	FUSB_LOG("%s\n", __func__);
	wake_up_statemachine();
	/* mt_eint_unmask(CUST_EINT_CC_DECODER_NUM); */
}

int fusb302_state_kthread(void *x)
{
	struct sched_param param
		= { .sched_priority = /*RTPM_PRIO_CPU_CALLBACK*/98 };

	FUSB_LOG("*****enter fusb302 state thread!!*****\n");
	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		if (1 /*FUSB300Int_PIN_LVL()*/) {
			set_current_state(TASK_INTERRUPTIBLE);
			wait_event_interruptible(fusb_thread_wq,
						 fusb_i2c_data->state_changed ==
						 true);
			fusb_i2c_data->state_changed = false;
			set_current_state(TASK_RUNNING);
		}
		StateMachineFUSB300();
	}
	return 0;
}

static void fusb302_device_check(void)
{
	FUSB300Read(regDeviceID, 2, &fusb_i2c_data->Registers.DeviceID.byte);
	FUSB_LOG("device id:%2x\n", fusb_i2c_data->Registers.DeviceID.byte);
}

static ssize_t fusb302_reg_dump(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int reg_addr[] = {
		regDeviceID, regSwitches0, regSwitches1,
		regMeasure, regSlice, regControl0, regControl1,
		regControl2, regControl3, regMask, regPower,
		regReset, regOCPreg, regMaska, regMaskb, regControl4,
		regStatus0a, regStatus1a, regInterrupta, regInterruptb,
		regStatus0, regStatus1, regInterrupt
	};

	int i, len = 0;
	u8 byte = 0;

	for (i = 0; i < ARRAY_SIZE(reg_addr); i++) {
		FUSB300Read(reg_addr[i], 1, &byte);
		len += sprintf(buf + len, "R%02xH:%02x ", reg_addr[i], byte);
		if (((i + 1) % 6) == 0)
			len += sprintf(buf + len, "\n");
	}
	return len;
}

static const char *charger_string(int charger_type)
{
	switch (charger_type) {
	case CHARGER_UNKNOWN:
		return "CHARGER_UNKNOWN";
	case STANDARD_HOST:
		return "STANDARD_HOST";
	case CHARGING_HOST:
		return "CHARGING_HOST";
	case NONSTANDARD_CHARGER:
		return "NONSTANDARD_CHARGER";
	case STANDARD_CHARGER:
		return "STANDARD_CHARGER";
	case APPLE_2_1A_CHARGER:
		return "APPLE_2_1A_CHARGER";
	case APPLE_1_0A_CHARGER:
		return "APPLE_1_0A_CHARGER";
	case APPLE_0_5A_CHARGER:
		return "APPLE_0_5A_CHARGER";
	case TYPEC_RdUSB_CHARGER:
		return "TYPEC_RdUSB_CHARGER";
	case TYPEC_1_5A_CHARGER:
		return "TYPEC_1_5A_CHARGER";
	case TYPEC_3A_CHARGER:
		return "TYPEC_3A_CHARGER";
	case TYPEC_PD_NO_READY_CHARGER:
		return "TYPEC_PD_NO_READY_CHARGER";
	case TYPEC_PD_5V_CHARGER:
		return "TYPEC_PD_5V_CHARGER";
	case TYPEC_PD_9V_CHARGER:
		return "TYPEC_PD_9V_CHARGER";
	default:
		return "CHARGER_UNKNOWN";
	}
}

static ssize_t fusb302_reg_set(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t size)
{
	return size;
}

static DEVICE_ATTR(reg_dump, 0664, fusb302_reg_dump, fusb302_reg_set);

static ssize_t fusb302_state(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	char data[5];

	GetFUSB300TypeCStatus(data);
	return sprintf(buf, "SMC=%2x, connState=%2d, cc=%2x, current=%d\n",
		       data[0], data[1], data[2], data[3]);
}

static DEVICE_ATTR(state, 0444, fusb302_state, NULL);

static ssize_t fusb302_configure_mode
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 byte = 0;

	FUSB300Read(regControl2, 1, &byte);
	/* read regControl2 and check mode[2:1] */
	byte = (byte & 0x06) >> 1;

	if (byte == 1 && fusb302_probe_finish == 1)
		return sprintf(buf, "%d\n", 1);
	else
		return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR(configure_mode, 0444, fusb302_configure_mode, NULL);

static ssize_t fusb302_get_cc_pin
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 byte = 0;

	if (fusb_i2c_data->blnCCPinIsCC1)
		byte = byte | 0x01;

	if (fusb_i2c_data->blnCCPinIsCC2)
		byte = byte | 0x02;

	return sprintf(buf, "%d\n", byte);
}
static DEVICE_ATTR(cc_pin, 0444, fusb302_get_cc_pin, NULL);

static ssize_t fusb302_get_pd_voltage
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;

	i = usbpd_getvoltagecurrent(USBPD_VOLTAGE);
	return sprintf(buf, "%d\n", i);
}
static DEVICE_ATTR(pd_voltage, 0444, fusb302_get_pd_voltage, NULL);

static ssize_t fusb302_get_pd_current
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;

	i = usbpd_getvoltagecurrent(USBPD_CURRENT);
	return sprintf(buf, "%d\n", i);
}
static DEVICE_ATTR(pd_current, 0444, fusb302_get_pd_current, NULL);

static ssize_t fusb302_get_pd_type
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", charger_string(BMT_status.TypeC_charger_type));
}
static DEVICE_ATTR(pd_type, 0444, fusb302_get_pd_type, NULL);

static int get_intn_eint_num(struct i2c_client *i2c)
{
	struct device_node *dn;
	u32 irq_num;

	dn = of_find_compatible_node(NULL, NULL, "fcs,type-C-fusb302");
	if (dn == NULL) {
		dev_err(&i2c->dev,
			"%s: type-C-fusb302 node not found\n", __func__);
		return -EINVAL;
	}

	irq_num = irq_of_parse_and_map(dn, 0);
	if (irq_num <= 0) {
		if (i2c->irq > 0) {
			dev_warn(&i2c->dev, "eint number - %d\n", i2c->irq);
			return i2c->irq;
		}
		dev_err(&i2c->dev,
			"invalid eint number - %d\n", irq_num);
		return -EINVAL;
	}
	dev_warn(&i2c->dev, "eint number - %d\n", irq_num);
	return irq_num;
}

static int fusb302_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct fusb302_i2c_data *fusb;
	int ret_device_file = 0;
	unsigned char data;
	int eint_num;
	int err;

	FUSB_LOG("enter probe\n");
	eint_num = get_intn_eint_num(i2c);
	if (eint_num < 0)
		goto exit;

	fusb = kzalloc(sizeof(struct fusb302_i2c_data), GFP_KERNEL);
	if (!fusb)
		goto exit;

	InitializeFUSB300Variables(fusb);
#ifdef PD_SUPPORT
	InitializeUSBPDVariables();
#endif

	fusb->eint_num = eint_num;
	fusb->int_disabled = 0;
	fusb_i2c_data = fusb;
	i2c_set_clientdata(i2c, fusb);
	fusb->client = i2c;

	ret_device_file =
		device_create_file(&(i2c->dev), &dev_attr_reg_dump);
	ret_device_file =
		device_create_file(&(i2c->dev), &dev_attr_state);
	ret_device_file =
		device_create_file(&(i2c->dev), &dev_attr_configure_mode);
	ret_device_file =
		device_create_file(&(i2c->dev), &dev_attr_cc_pin);
	ret_device_file =
		device_create_file(&(i2c->dev), &dev_attr_pd_voltage);
	ret_device_file =
		device_create_file(&(i2c->dev), &dev_attr_pd_current);
	ret_device_file =
		device_create_file(&(i2c->dev), &dev_attr_pd_type);

	fusb302_device_check();
	data = 0x01;
	FUSB300Write(regReset, 1, &data);
	InitializeFUSB300();

	fusb->thread =
	    kthread_run(fusb302_state_kthread, NULL, "fusb302_state_kthread");

#ifdef USE_EARLY_SUSPEND
	fusb->early_drv.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	    fusb->early_drv.suspend = fusb302_early_suspend,
	    fusb->early_drv.resume = fusb302_late_resume,
	    register_early_suspend(&fusb->early_drv);
#endif

	INIT_WORK(&fusb_i2c_data->eint_work, fusb_eint_work);

	err = request_irq(fusb->eint_num,
			cc_eint_interrupt_handler,
			  IRQF_ONESHOT, "type-C-eint", fusb);
	if (err) {
		dev_err(&i2c->dev, "fail to register otg eint iddig isr\n");
		return -EINVAL;
	}

	if (!fusb302_probe_finish)
		fusb302_probe_finish = 1;

	FUSB_LOG("probe successfully!\n");
	return 0;

exit:
	return -1;
}

static int fusb302_remove(struct i2c_client *i2c)
{
	if (fusb302_probe_finish)
		fusb302_probe_finish = 0;
	kfree(i2c_get_clientdata(i2c));
	return 0;
}

#ifndef USE_EARLY_SUSPEND
static int fusb302_suspend(struct i2c_client *client, pm_message_t msg)
{
	return 0;
}

static int fusb302_resume(struct i2c_client *client)
{
	return 0;
}
#else
static void fusb302_early_suspend(struct early_suspend *h)
{
}

static void fusb302_late_resume(struct early_suspend *h)
{
	/* mt_eint_unmask(CUST_EINT_CC_DECODER_NUM); */
}
#endif

static const struct of_device_id fusb302_id[] = {
	{.compatible = "fcs,type-C-fusb302"},
	{},
};
MODULE_DEVICE_TABLE(of, fusb302_id);

static const struct i2c_device_id fusb302_i2c_id[] = {
	{ FUSB302_I2C_NAME, 0 },
	{}
};


static struct i2c_driver fusb302_i2c_driver = {
	.driver = {
		.name = FUSB302_I2C_NAME,
		.of_match_table = of_match_ptr(fusb302_id),
	},
	.probe = fusb302_probe,
	.remove = fusb302_remove,
#if !defined(USE_EARLY_SUSPEND)
	.suspend = fusb302_suspend,
	.resume = fusb302_resume,
#endif
	.id_table = fusb302_i2c_id,
};

static int fusb302_i2c_init(void)
{
	return i2c_add_driver(&fusb302_i2c_driver);
}

static void fusb302_i2c_exit(void)
{
	i2c_del_driver(&fusb302_i2c_driver);
}

module_init(fusb302_i2c_init);
module_exit(fusb302_i2c_exit);
