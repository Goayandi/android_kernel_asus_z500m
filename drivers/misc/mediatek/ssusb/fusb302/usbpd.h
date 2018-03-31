#ifndef USBPD_H
#define	USBPD_H

#define PDBUFSIZE               128
#define USBPDSPECREV            1 /* Revision 2.0*/

#define STAT_BUSY               0
#define STAT_SUCCESS            1
#define STAT_ERROR              2

/* FUSB300 FIFO Token Definitions*/
#define TXON                    0xA1
#define SOP1                    0x12
#define SOP2                    0x13
#define SOP3                    0x1B
#define RESET1                  0x15
#define RESET2                  0x16
#define PACKSYM                 0x80
#define JAM_CRC                 0xFF
#define EOP                     0x14
#define TXOFF                   0xFE

/* USB PD Header Message Defintions*/
#define PDPortRoleSink          0
#define PDPortRoleSource        1
#define PDSpecRev1p0            0
#define PDSpecRev2p0            1
#define PDDataRoleUFP           0
#define PDDataRoleDFP           1
#define PDCablePlugSource       0
#define PDCablePlugPlug         1

/* USB PD Control Message Types*/
#define CMTGoodCRC              0x1 /*(0b0001) */
#define CMTGotoMin              0x2 /*(0b0010) */
#define CMTAccept               0x3 /*(0b0011) */
#define CMTReject               0x4 /*(0b0100) */
#define CMTPing                 0x5 /*(0b0101) */
#define CMTPS_RDY               0x6 /*(0b0110) */
#define CMTGetSourceCap         0x7 /*(0b0111) */
#define CMTGetSinkCap           0x8 /*(0b1000) */
#define CMTDR_Swap              0x9 /*(0b1001) */
#define CMTPR_Swap              0xa /*(0b1010) */
#define CMTVCONN_Swap           0xb /*(0b1011) */
#define CMTWait                 0xc /*(0b1100) */
#define CMTSoftReset            0xd /*(0b1101) */

/* USB PD Data Message Types*/
#define DMTSourceCapabilities   0x1 /*(0b0001) */
#define DMTRequest              0x2 /*(0b0010) */
#define DMTBIST                 0x3 /*(0b0011) */
#define DMTSinkCapabilities     0x4 /*(0b0100) */
#define DMTVenderDefined        0xf /*(0b1111) */

/* USB PD Timing Parameters defined in ms */
#define tNoResponse             (5000 * 40)
#define tSenderResponse         (30 * 40)
#define tTypeCSendSourceCap     (150 * 40)
#define tSinkWaitCap            (2300 * 40)
#define tSnkTransition          (35)
#define tPSHardReset            (15 * 40)
#define tPSTransition           (500 * 40)
#define tPSSourceOffMin         (750 * 40)
#define tPSSourceOffMax         (920 * 40)
#define tPSSourceOffNom         (800)
#define tPSSourceOnMin          (390 * 40)
#define tPSSourceOnMax          (480 * 40)
#define tPSSourceOnNom          (445)
#define tVCONNSourceOn          (100 * 40)
#define tReceive                (200)
#define tBMCTimeout             (200)
#define tFPF2498Transition      (20)
#define tPRSwapBailout          (5000 * 40)

#define nHardResetCount         2
#define nRetryCount             2
#define nCapsCount              50

enum USBPD_BufferTokens_t {
	pdtNone = 0,
	pdtAttach,
	pdtDetach,
	pdtHardReset,
	pdtBadMessageID,
};

enum PolicyState_t {
	peDisabled = 0,
	peErrorRecovery,
	peSourceHardReset,
	peSourceSendHardReset,
	peSourceSoftReset,
	peSourceSendSoftReset,
	peSourceStartup,
	peSourceSendCaps,
	peSourceDiscovery,
	peSourceDisabled,
	peSourceTransitionDefault,
	peSourceNegotiateCap,
	peSourceCapabilityResponse,
	peSourceTransitionSupply,
	peSourceReady,
	peSourceGiveSourceCaps,
	peSourceGetSinkCaps,
	peSourceSendPing,
	peSourceGotoMin,
	peSourceGiveSinkCaps,
	peSourceGetSourceCaps,
	peSourceSendDRSwap,
	peSourceEvaluateDRSwap,
	peSinkHardReset,
	peSinkSendHardReset,
	peSinkSoftReset,
	peSinkSendSoftReset,
	peSinkTransitionDefault,
	peSinkStartup,
	peSinkDiscovery,
	peSinkWaitCaps,
	peSinkEvaluateCaps,
	peSinkSelectCapability,
	peSinkTransitionSink,
	peSinkReady,
	peSinkGiveSinkCap,
	peSinkGetSourceCap,
	peSinkGetSinkCap,
	peSinkGiveSourceCap,
	peSinkSendDRSwap,
	peSinkEvaluateDRSwap,
	peSourceSendVCONNSwap,
	peSinkEvaluateVCONNSwap,
	peSourceSendPRSwap,
	peSourceEvaluatePRSwap,
	peSinkSendPRSwap,
	peSinkEvaluatePRSwap,
};

enum ProtocolState_t {
	PRLDisabled = 0,
	PRLIdle,
	PRLReset,
	PRLResetWait,
	PRLRxWait,
	PRLTxSendingMessage,
	PRLTxWaitForPHYResponse,
	PRLTxVerifyGoodCRC
};

enum PDTxStatus_t {
	txIdle = 0,
	txReset,
	txSend,
	txBusy,
	txWait,
	txSuccess,
	txError,
	txCollision
};

enum pdoSupplyType {
	pdoTypeFixed = 0,
	pdoTypeBattery,
	pdoTypeVariable,
	pdoTypeReserved
};

union sopMainHeader_t {
	u16 word;
	u8 byte[2];
	struct {
		/* Switches0*/
		unsigned MessageType:4;
		unsigned:1;
		unsigned PortDataRole:1;
		unsigned SpecRevision:2;
		unsigned PortPowerRole:1;
		unsigned MessageID:3;
		unsigned NumDataObjects:3;
		unsigned:1;
	};
};

union sopPlugHeader_t {
	u16 word;
	u8 byte[2];
	struct {
		/* Switches0*/
		unsigned MessageType:4;
		unsigned:1;
		unsigned:1;
		unsigned SpecRevision:2;
		unsigned CablePlug:1;
		unsigned MessageID:3;
		unsigned NumDataObjects:3;
		unsigned:1;
	};
};

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

struct usbpd_select_charging {
	/* PD adapter support */
	unsigned usbpdselObjectPosition:3;
	unsigned usbpdselVoltage:10;
	unsigned usbpdselCurrent:10;
	unsigned ObjectPosition_lowbattery:3;
	unsigned selVoltage_lowbattery:10;
	unsigned selCurrent_lowbattery:10;
};


struct fusb_pd_ctx {
	/* Debugging Variables*/
	u8	USBPDBuf[PDBUFSIZE];
	u8	USBPDBufStart;
	u8	USBPDBufEnd;
	bool	USBPDBufOverflow;

	struct hrtimer protocol_hrtimer;
	struct hrtimer policystate_hrtimer;
	struct hrtimer noresponse_hrtimer;
	struct hrtimer prswap_hrtimer;
	struct usbpd_select_charging pdpower_record;

	bool	USBPDTxFlag;

	union sopMainHeader_t PDTransmitHeader;
	union sopMainHeader_t CapsHeaderSink;
	union sopMainHeader_t CapsHeaderSource;
	union sopMainHeader_t CapsHeaderReceived;
	union doDataObject_t	PDTransmitObjects[7];
	union doDataObject_t	CapsSink[7];
	union doDataObject_t	CapsSource[7];
	union doDataObject_t	CapsReceived[7];
	union doDataObject_t	USBPDContract;
	union doDataObject_t	SinkRequest;
	u32 SinkRequestMaxVoltage;
	u32 SinkRequestMaxCurrent;
	u32 SinkRequestMaxPower;
	u32 SinkRequestOpPower;
	u32 SelVoltage;
	bool	SinkGotoMinCompatible;
	bool	SinkUSBSuspendOperation;
	bool	SinkUSBCommCapable;
	bool	SourceCapsUpdated;

	/* Policy Variables*/
	enum PolicyState_t	PolicyState;
	u8	PolicySubIndex;
	bool	PolicyIsSource;
	bool	PolicyIsDFP;
	bool	PolicyHasContract;
	u8	CollisionCounter;
	u8	HardResetCounter;
	u8	CapsCounter;
	u32	policystate_time_count;
	u32	noresponse_time_count;
	union sopMainHeader_t	PolicyRxHeader;
	union sopMainHeader_t	PolicyTxHeader;
	union doDataObject_t	PolicyRxDataObj[7];
	union doDataObject_t	PolicyTxDataObj[7];

	/* Protocol Variables*/
	enum ProtocolState_t ProtocolState;
	enum PDTxStatus_t	PDTxStatus;
	u8	MessageIDCounter;
	u8	MessageID;
	bool	ProtocolMsgRx;
	bool	ProtocolMsgTx;
	u8	ProtocolTxBytes;
	u8	ProtocolTxBuffer[64];
	u8	ProtocolRxBuffer[64];
	u16	protocol_time_count;
	u8	ProtocolCRC[4];
};

extern void wake_up_statemachine(void);
extern struct fusb302_i2c_data *fusb_i2c_data;

/******************************************************************************
 *                            LOCAL PROTOTYPES                                *
 ******************************************************************************/
int InitializeUSBPDVariables(void);
void USBPDEnable(bool FUSB300Update, bool TypeCDFP);
void USBPDDisable(bool FUSB300Update);
void USBPDPolicyEngine(void);

void PolicyErrorRecovery(void);
void PolicySourceSendHardReset(void);
void PolicySourceSoftReset(void);
void PolicySourceSendSoftReset(void);
void PolicySourceStartup(void);
void PolicySourceDiscovery(void);
void PolicySourceSendCaps(void);
void PolicySourceDisabled(void);
void PolicySourceTransitionDefault(void);
void PolicySourceNegotiateCap(void);
void PolicySourceTransitionSupply(void);
void PolicySourceCapabilityResponse(void);
void PolicySourceReady(void);
void PolicySourceGiveSourceCap(void);
void PolicySourceGetSourceCap(void);
void PolicySourceGetSinkCap(void);
void PolicySourceGetSinkCap(void);
void PolicySourceSendPing(void);
void PolicySourceGotoMin(void);
void PolicySourceGiveSinkCap(void);
void PolicySourceSendDRSwap(void);
void PolicySourceEvaluateDRSwap(void);
void PolicySourceSendVCONNSwap(void);
void PolicySourceSendPRSwap(void);
void PolicySourceEvaluatePRSwap(void);
void PolicySinkSendHardReset(void);
void PolicySinkSoftReset(void);
void PolicySinkSendSoftReset(void);
void PolicySinkTransitionDefault(void);
void PolicySinkStartup(void);
void PolicySinkDiscovery(void);
void PolicySinkWaitCaps(void);
void PolicySinkEvaluateCaps(void);
void PolicySinkSelectCapability(void);
void PolicySinkTransitionSink(void);
void PolicySinkReady(void);
void PolicySinkGiveSinkCap(void);
void PolicySinkGetSinkCap(void);
void PolicySinkGiveSourceCap(void);
void PolicySinkGetSourceCap(void);
void PolicySinkSendDRSwap(void);
void PolicySinkEvaluateDRSwap(void);
void PolicySinkEvaluateVCONNSwap(void);
void PolicySinkSendPRSwap(void);
void PolicySinkEvaluatePRSwap(void);
bool PolicySendHardReset(enum PolicyState_t nextState, u32 delay);
u8 PolicySendCommand(u8 Command, enum PolicyState_t nextState, u8 subIndex);
u8 PolicySendCommandNoReset(u8 Command, enum PolicyState_t nextState,
			    u8 subIndex);
u8 PolicySendData(u8 MessageType, u8 NumDataObjects,
		  union doDataObject_t *DataObjects,
		enum PolicyState_t nextState, u8 subIndex);
u8 PolicySendDataNoReset(u8 MessageType, u8 NumDataObjects,
		union doDataObject_t *DataObjects,
		enum PolicyState_t nextState, u8 subIndex);
void UpdateCapabilitiesRx(bool IsSourceCaps);
void USBPDProtocol(void);
void ProtocolIdle(void);
void ProtocolResetWait(void);
void ProtocolRxWait(void);
void ProtocolGetRxPacket(void);
void ProtocolTransmitMessage(void);
void ProtocolSendingMessage(void);
void ProtocolWaitForPHYResponse(void);
void ProtocolVerifyGoodCRC(void);
void ProtocolTxRetryCounter(void);
void ProtocolSendGoodCRC(void);
void ProtocolLoadSOP(void);
void ProtocolLoadEOP(void);
void ProtocolSendHardReset(void);
void ProtocolFlushRxFIFO(void);
void ProtocolFlushTxFIFO(void);
void ResetProtocolLayer(bool ResetPDLogic);
bool StoreUSBPDToken(bool transmitter, enum USBPD_BufferTokens_t token);
bool StoreUSBPDMessage(union sopMainHeader_t Header,
		       union doDataObject_t *DataObject,
		       bool transmitter, u8 SOPType);
u8 GetNextUSBPDMessageSize(void);
u8 GetUSBPDBufferNumBytes(void);
bool ClaimBufferSpace(int intReqSize);
void GetUSBPDStatus(unsigned char abytData[]);
u8 GetUSBPDStatusOverview(void);
u8 ReadUSBPDBuffer(unsigned char *pData, unsigned char bytesAvail);
void SendUSBPDMessage(unsigned char *abytData);
void WriteSourceCapabilities(unsigned char *abytData);
void ReadSourceCapabilities(unsigned char *abytData);
void WriteSinkCapabilities(unsigned char *abytData);
void ReadSinkCapabilities(unsigned char *abytData);
void WriteSinkRequestSettings(unsigned char *abytData);
void ReadSinkRequestSettings(unsigned char *abytData);
void SendUSBPDHardReset(void);

#if 0
/* shim functions for VDM code*/
extern VdmManager vdmm;
void InitializeVdmManager(void);
void convertAndProcessVdmMessage(void);
void sendVdmMessage(VdmManager *vdmm, SopType sop, u32 *arr,
		    unsigned int length);
void doVdmCommand(void);
void doDiscoverIdentity(void);
void doDiscoverSvids(void);
#endif

int usbpd_getvoltagecurrent(int select);

#endif	/* USBPD_H */
