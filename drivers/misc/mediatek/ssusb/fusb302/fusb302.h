#ifndef FUSB302_H
#define	FUSB302_H

#define SDAC_DEFAULT        0x20
#define MDAC_1P6V           0x26
#define MDAC_2P05V          0x31
#define MDAC_2P6V           0x3E

/* FUSB300 Register Addresses */
#define regDeviceID     0x01
#define regSwitches0    0x02
#define regSwitches1    0x03
#define regMeasure      0x04
#define regSlice        0x05
#define regControl0     0x06
#define regControl1     0x07
#define regControl2     0x08
#define regControl3     0x09
#define regMask         0x0A
#define regPower        0x0B
#define regReset        0x0C
#define regOCPreg       0x0D
#define regMaska        0x0E
#define regMaskb        0x0F
#define regControl4     0x10
#define regStatus0a     0x3C
#define regStatus1a     0x3D
#define regInterrupta   0x3E
#define regInterruptb   0x3F
#define regStatus0      0x40
#define regStatus1      0x41
#define regInterrupt    0x42
#define regFIFO         0x43

/* Type C Timing Parameters */
#define tAMETimeout     1000
#define tCCDebounceMin  100
#define tCCDebounceNom  120
#define tCCDebounceMax  200
#define tPDDebounceMin  10
#define tPDDebounceMax  20
#define tAccDetect      100
#define tDRP            80
#define tDRPAdvert      30
#define tDRPTransition  1
#define tDRPTry         125
#define tDRPTryWait     600
#define tErrorRecovery  25


#define tVBUSOn         275

#define tVBUSOff        650

#define tVConnOn        2

#define tVConnOnPA      100

#define tVConnOff       35

#define tSinkAdj        40


#define tFUSB302Toggle  3

enum USBTypeCPort {
	USBTypeC_Sink = 0,
	USBTypeC_Source,
	USBTypeC_DRP
};

enum ConnectionState {
	Disabled = 0,
	ErrorRecovery,
	Unattached,
	AttachWaitSink,
	AttachedSink,
	AttachWaitSource,
	AttachedSource,
	TrySource,
	TryWaitSink,
	AudioAccessory,
	DebugAccessory,
	AttachWaitAccessory,
	PoweredAccessory,
	UnsupportedAccessory,
	DelayUnattached,
};

enum CCTermType {
	CCTypeNone = 0,
	CCTypeRa,
	CCTypeRdUSB,
	CCTypeRd1p5,
	CCTypeRd3p0
};

enum TypeCPins_t {
	TypeCPin_None = 0,
	TypeCPin_GND1,
	TypeCPin_TXp1,
	TypeCPin_TXn1,
	TypeCPin_VBUS1,
	TypeCPin_CC1,
	TypeCPin_Dp1,
	TypeCPin_Dn1,
	TypeCPin_SBU1,
	TypeCPin_VBUS2,
	TypeCPin_RXn2,
	TypeCPin_RXp2,
	TypeCPin_GND2,
	TypeCPin_GND3,
	TypeCPin_TXp2,
	TypeCPin_TXn2,
	TypeCPin_VBUS3,
	TypeCPin_CC2,
	TypeCPin_Dp2,
	TypeCPin_Dn2,
	TypeCPin_SBU2,
	TypeCPin_VBUS4,
	TypeCPin_RXn1,
	TypeCPin_RXp1,
	TypeCPin_GND4
};

enum USBTypeCCurrent {
	utccNone = 0,
	utccDefault,
	utcc1p5A,
	utcc3p0A
};

union regDeviceID_t {
	u8 byte;
	struct {
		unsigned REVISION:3;
		unsigned VERSION:5;
	};
};

union regSwitches_t {
	u16 word;
	u8 byte[2];
	struct {
		/* Switches0 */
		unsigned PDWN1:1;
		unsigned PDWN2:1;
		unsigned MEAS_CC1:1;
		unsigned MEAS_CC2:1;
		unsigned VCONN_CC1:1;
		unsigned VCONN_CC2:1;
		unsigned PU_EN1:1;
		unsigned PU_EN2:1;
		/* Switches1 */
		unsigned TXCC1:1;
		unsigned TXCC2:1;
		unsigned AUTO_CRC:1;
		unsigned:1;
		unsigned DATAROLE:1;
		unsigned SPECREV:2;
		unsigned POWERROLE:1;
	};
};

union regMeasure_t {
	u8 byte;
	struct {
		unsigned MDAC:6;
		unsigned MEAS_VBUS:1;
		unsigned:1;
	};
};

union regSlice_t {
	u8 byte;
	struct {
		unsigned SDAC:6;
		unsigned:2;
	};
};

union regControl_t {
	u32 dword;
	u8 byte[4];
	struct {
		/* Control0 */
		unsigned TX_START:1;
		unsigned AUTO_PRE:1;
		unsigned HOST_CUR:2;
		unsigned LOOPBACK:1;
		unsigned INT_MASK:1;
		unsigned TX_FLUSH:1;
		unsigned:1;
		/* Control1 */
		unsigned ENSOP1:1;
		unsigned ENSOP2:1;
		unsigned RX_FLUSH:1;
		unsigned:1;
		unsigned BIST_MODE2:1;
		unsigned ENSOP1DP:1;
		unsigned ENSOP2DB:1;
		unsigned:1;
		/* Control2 */
		unsigned TOGGLE:1;
		unsigned MODE:2;
		unsigned WAKE_EN:1;
		unsigned WAKE_SELF:1;
		unsigned TOG_RD_ONLY:1;
		unsigned:2;
		/* Control3 */
		unsigned AUTO_RETRY:1;
		unsigned N_RETRIES:2;
		unsigned AUTO_SOFTRESET:1;
		unsigned AUTO_HARDRESET:1;
		unsigned:1;
		unsigned SEND_HARDRESET:1;
		unsigned:1;
	};
};

union regMask_t {
	u8 byte;
	struct {
		unsigned M_BC_LVL:1;
		unsigned M_COLLISION:1;
		unsigned M_WAKE:1;
		unsigned M_ALERT:1;
		unsigned M_CRC_CHK:1;
		unsigned M_COMP_CHNG:1;
		unsigned M_ACTIVITY:1;
		unsigned M_VBUSOK:1;
	};
};

union regPower_t {
	u8 byte;
	struct {
		unsigned PWR:4;
		unsigned:4;
	};
};

union regReset_t {
	u8 byte;
	struct {
		unsigned SW_RES:1;
		unsigned:7;
	};
};

union regOCPreg_t {
	u8 byte;
	struct {
		unsigned OCP_CUR:3;
		unsigned OCP_RANGE:1;
		unsigned:4;
	};
};

union regMaskAdv_t {
	u16 word;
	u8 byte[2];
	struct {
		/* Maska */
		unsigned M_HARDRST:1;
		unsigned M_SOFTRST:1;
		unsigned M_TXCRCSENT:1;
		unsigned M_HARDSENT:1;
		unsigned M_RETRYFAIL:1;
		unsigned M_SOFTFAIL:1;
		unsigned M_TOGDONE:1;
		unsigned M_OCP_TEMP:1;
		/* Maskb */
		unsigned M_GCRCSENT:1;
		unsigned:7;
	};
};

union regControl4_t {
	u8 byte;
	struct {
		unsigned TOG_USRC_EXIT:1;
		unsigned:7;
	};
};

union regStatus_t {
	u8 byte[7];
	struct {
		u16  StatusAdv;
		u16  InterruptAdv;
		u16  Status;
		u8   Interrupt1;
	};
	struct {
		/* Status0a */
		unsigned HARDRST:1;
		unsigned SOFTRST:1;
		unsigned POWER23:2;
		unsigned RETRYFAIL:1;
		unsigned SOFTFAIL:1;
		unsigned TOGDONE:1;
		unsigned M_OCP_TEMP:1;
		/* Status1a */
		unsigned RXSOP:1;
		unsigned RXSOP1DB:1;
		unsigned RXSOP2DB:1;
		unsigned TOGSS:3;
		unsigned:2;
		/* Interrupta */
		unsigned I_HARDRST:1;
		unsigned I_SOFTRST:1;
		unsigned I_TXSENT:1;
		unsigned I_HARDSENT:1;
		unsigned I_RETRYFAIL:1;
		unsigned I_SOFTFAIL:1;
		unsigned I_TOGDONE:1;
		unsigned I_OCP_TEMP:1;
		/* Interruptb */
		unsigned I_GCRCSENT:1;
		unsigned:7;
		/* Status0 */
		unsigned BC_LVL:2;
		unsigned WAKE:1;
		unsigned ALERT:1;
		unsigned CRC_CHK:1;
		unsigned COMP:1;
		unsigned ACTIVITY:1;
		unsigned VBUSOK:1;
		/* Status1 */
		unsigned OCP:1;
		unsigned OVRTEMP:1;
		unsigned TX_FULL:1;
		unsigned TX_EMPTY:1;
		unsigned RX_FULL:1;
		unsigned RX_EMPTY:1;
		unsigned RXSOP1:1;
		unsigned RXSOP2:1;
		/* Interrupt */
		unsigned I_BC_LVL:1;
		unsigned I_COLLISION:1;
		unsigned I_WAKE:1;
		unsigned I_ALERT:1;
		unsigned I_CRC_CHK:1;
		unsigned I_COMP_CHNG:1;
		unsigned I_ACTIVITY:1;
		unsigned I_VBUSOK:1;
	};
};

struct FUSB300reg_t {
	union regDeviceID_t   DeviceID;
	union regSwitches_t   Switches;
	union regMeasure_t    Measure;
	union regSlice_t      Slice;
	union regControl_t    Control;
	union regMask_t       Mask;
	union regPower_t      Power;
	union regReset_t      Reset;
	union regOCPreg_t     OCPreg;
	union regMaskAdv_t    MaskAdv;
	union regControl4_t   Control4;
	union regStatus_t     Status;
};

struct fusb302_i2c_data {
	struct i2c_client       *client;
	struct task_struct      *thread;
	struct work_struct      eint_work;
	spinlock_t	lock;

	#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
	struct early_suspend	early_drv;
	#endif

	struct FUSB300reg_t	Registers;
	bool	USBPDActive;
	bool	USBPDEnabled;
	u32	prswap_time_count;

	enum USBTypeCPort	PortType;
	bool	blnCCPinIsCC1;
	bool	blnCCPinIsCC2;
	bool	blnSMEnabled;

	struct hrtimer state_hrtimer;
	struct hrtimer debounce_hrtimer1;
	struct hrtimer debounce_hrtimer2;
	struct hrtimer toggle_hrtimer;

	bool	state_changed;
	bool	blnSrcPreferred;
	bool	blnAccSupport;
	u16	state_time_count;
	u16	debounce_time_count1;
	u16	debounce_time_count2;
	u16	toggle_time_count;
	enum CCTermType	CC1TermAct;
	enum CCTermType	CC2TermAct;
	enum CCTermType	CC1TermDeb;
	enum CCTermType	CC2TermDeb;
	enum USBTypeCCurrent	SinkCurrent;
	enum USBTypeCCurrent	SourceCurrent;

	enum ConnectionState	ConnState;
	int VBUS_5V_EN;
	int VBUS_12V_EN;
	int eint_num;
	int int_disabled;
	int g_peSinkWaitCaps_retry;
};

extern struct fusb_pd_ctx *fusb_pd_ctx;

void InitializeFUSB300Variables(struct fusb302_i2c_data *fusb);
void InitializeFUSB300(void);
void DisableFUSB300StateMachine(void);
void EnableFUSB300StateMachine(void);
void StateMachineFUSB300(void);
void StateMachineDisabled(void);
void StateMachineErrorRecovery(void);
void StateMachineDelayUnattached(void);
void StateMachineUnattached(void);
void StateMachineAttachWaitSnk(void);
void StateMachineAttachWaitSrc(void);
void StateMachineAttachWaitAcc(void);
void StateMachineAttachedSink(void);
void StateMachineAttachedSource(void);
void StateMachineTryWaitSnk(void);
void StateMachineTrySrc(void);
void StateMachineDebugAccessory(void);
void StateMachineAudioAccessory(void);
void StateMachinePoweredAccessory(void);
void StateMachineUnsupportedAccessory(void);
void SetStateDisabled(void);
void SetStateErrorRecovery(void);
void SetStateDelayUnattached(void);
void SetStateUnattached(void);
void SetStateAttachWaitSnk(void);
void SetStateAttachWaitSrc(void);
void SetStateAttachWaitAcc(void);
void SetStateAttachedSrc(void);
void SetStateAttachedSink(void);
void RoleSwapToAttachedSink(void);
void RoleSwapToAttachedSource(void);
void SetStateTryWaitSnk(void);
void SetStateTrySrc(void);
void SetStateDebugAccessory(void);
void SetStateAudioAccessory(void);
void SetStatePoweredAccessory(void);
void SetStateUnsupportedAccessory(void);
void UpdateSourcePowerMode(void);
void ToggleMeasureCC1(void);
void ToggleMeasureCC2(void);
enum CCTermType DecodeCCTermination(void);
void UpdateSinkCurrent(enum CCTermType Termination);
void ConfigurePortType(unsigned char Control);
void UpdateCurrentAdvert(unsigned char Current);
void GetFUSB300TypeCStatus(unsigned char abytData[]);
unsigned char GetTypeCSMControl(void);
unsigned char GetCCTermination(void);
int FUSB300Write(u8 regAddr, u8 length, u8 *data);
int FUSB300Read(u8 regAddr, u8 length, u8 *data);

#endif	/* FUSB300_H */

