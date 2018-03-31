#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/rwlock_types.h>
#include <linux/completion.h>
#include "anx_ohio_private_interface.h"
#include "anx_ohio_public_interface.h"

/* init setting for TYPE_PWR_SRC_CAP */
static u32 init_src_caps[1] = {
	/*5V, 1.5A, Fixed */
	PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_1500MA, PDO_FIXED_FLAGS)
};

	/* init setting for TYPE_PWR_SNK_CAP */
static u32 init_snk_cap[3] = {
	/*5V, 0.9A, Fixed */
	PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_900MA, PDO_FIXED_FLAGS),
	/*min 5V, max 20V, power 60W, battery */
	PDO_BATT(PD_VOLTAGE_5V, PD_MAX_VOLTAGE_20V, PD_POWER_15W),
	/*min5V, max 5V, current 3A, variable */
	PDO_VAR(PD_VOLTAGE_5V, PD_MAX_VOLTAGE_20V, PD_CURRENT_3A)
};

	/* init setting for TYPE_SVID */
static u8 init_svid[4] = { 0x00, 0x00, 0x01, 0xff };

	/* init setting for TYPE_DP_SNK_IDENTITY */
static u8 init_snk_ident[16] = { 0x00, 0x00, 0x00, 0xec,	/*snk_id_hdr */
	0x00, 0x00, 0x00, 0x00,	/*snk_cert */
	0x00, 0x00, 0x00, 0x00,	/*snk_prd */
	0x39, 0x00, 0x00, 0x51	/*5snk_ama */
};

u8 pd_src_pdo_cnt = 2;
u8 pd_src_pdo[VDO_SIZE] = {
	/*5V 0.9A , 5V 1.5 */
	0x5A, 0x90, 0x01, 0x2A, 0x96, 0x90, 0x01, 0x2A
};

u8 sink_svid_vdo[PD_ONE_DATA_OBJECT_SIZE];
u8 pd_snk_pdo_cnt = 3;
u8 pd_snk_pdo[VDO_SIZE];
u8 pd_rdo[PD_ONE_DATA_OBJECT_SIZE];
u8 DP_caps[PD_ONE_DATA_OBJECT_SIZE];
u8 configure_DP_caps[PD_ONE_DATA_OBJECT_SIZE];
u8 src_dp_caps[PD_ONE_DATA_OBJECT_SIZE];

/* circular buffer driver */
unsigned char InterfaceSendBuf[32];
unsigned char InterfaceRecvBuf[32];

#define MAX_SEND_BUF_SIZE 8
#define MAX_RECV_BUF_SIZE 8

unsigned char pbuf_rx_front = 0;
unsigned char pbuf_tx_rear = 0;

#define TX_BUF_FRONT 0x11
#define TX_BUF_REAR   0x12
#define TX_BUF_START 0x18	/* 0x18-0x1f */

#define RX_BUF_FRONT 0x13
#define RX_BUF_REAR   0x14
#define RX_BUF_START  0x20	/* 0x20-0x27 */

#define RCVDER_ACK_STATUS 0x15
#define SENDER_ACK_STATUS 0x16


#define tx_buf_rear() pbuf_tx_rear
#define rx_buf_front() pbuf_rx_front

#define rx_buf_rear() OhioReadReg(RX_BUF_REAR)
#define tx_buf_front() OhioReadReg(TX_BUF_FRONT)

#define receiver_set_ack_status(val) OhioWriteReg(RCVDER_ACK_STATUS, val)
#define receiver_get_ack_status() OhioReadReg(RCVDER_ACK_STATUS)

#define sender_get_ack_status() ((OhioReadReg(SENDER_ACK_STATUS)) & 0x7F)
#define sender_set_ack_status(val) OhioWriteReg(SENDER_ACK_STATUS, val)


/**
 * @desc:   Interface AP fetch OTP word 1 byte 4,
 *
 *
 * @return:
 *
 */
u8 get_otp_indicator_byte(void)
{
	u8 temp;

	OhioWriteReg(0xe5, 0xa0);	/* disable ocm access otp & wake up otp */
	OhioWriteReg(0xef, 0x7a);	/* OTP access key */
	OhioWriteReg(0xd0, 0x00);	/* high address */
	OhioWriteReg(0xd1, 0x01);	/* low address */
	OhioWriteReg(0xe5, 0xa1);	/* otp read start */
	while (OhioReadReg(0xed) & 0x30)
		;	/* wait for read done */
	temp = OhioReadReg(0xe0);
	OhioWriteReg(0xef, 0x0);	/* OTP access key clear */
	OhioWriteReg(0xe5, 0x0);	/* enable ocm */
	return temp;
}

/**
 * @desc:   The interface AP will get the ohio's data role
 *
 * @param:  none
 *
 * @return:  data role , dfp 1 , ufp 0, other error: -1, not ready
 *
 */
s8 get_data_role(void)
{
	u8 status;

	/*fetch the data role */
	status = OhioReadReg(OHIO_SYSTEM_STSTUS);

	return ((status & DATA_ROLE) != 0);

}

/**
 * @desc:   The interface AP will get the ohio's power role
 *
 * @param:  none
 *
 * @return:  data role , source 1 , sink 0, other error, -1, not ready
 *
 */
s8 get_power_role(void)
{
	u8 status;

	/*fetch the power role */
	status = OhioReadReg(0x40);

	return ((status & 0x08) == 0);
}

/**
 * @desc:   Interface AP fetch the source capability from Ohio
 *
 * @param:  pdo_buf: PDO buffer pointer of source capability in Ohio
 *          src_caps_size: source capability's size
 *
 * @return:  0: success 1: fail
 *
 */
u8 get_src_cap(const u8 *src_caps, u8 src_caps_size)
{

	return 1;
}

/**
 * @desc:   Interface that AP fetch the sink capability from Ohio's downstream device
 *
 * @param:  sink_caps: PDO buffer pointer of sink capability
 *            which will be responsed by Ohio's SINK Capablity Message
 *
 *          snk_caps_len: sink capability max length of the array
 *
 * @return:  sink capability array length>0: success.  0: fail
 *
 */
u8 get_snk_cap(u8 *snk_caps, u8 snk_caps_len)
{

	return 1;
}

/**
 * @desc:   The Interface AP set the source capability to Ohio
 *
 * @param:  pdo_buf: PDO buffer pointer of source capability,
 *                              which can be packed by PDO_FIXED_XXX macro
 *                eg: default5Vsafe src_cap(5V, 0.9A fixed) -->
 *			PDO_FIXED(5000,900, PDO_FIXED_FLAGS)
 *
 *                src_caps_size: source capability's size
 *
 * @return:  0: success 1: fail
 *
 */
u8 send_src_cap(const u8 *src_caps, u8 src_caps_size)
{
	if (NULL == src_caps)
		return CMD_FAIL;
	if ((src_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
	    (src_caps_size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}
	memcpy(pd_src_pdo, src_caps, src_caps_size);
	pd_src_pdo_cnt = src_caps_size / PD_ONE_DATA_OBJECT_SIZE;

	/*send source capabilities message to Ohio really */
	return interface_send_msg_timeout(TYPE_PWR_SRC_CAP, pd_src_pdo,
					  pd_src_pdo_cnt *
					  PD_ONE_DATA_OBJECT_SIZE, INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure) the sink capability to Ohio's downstream device
 *
 * @param:  snk_caps: PDO buffer pointer of sink capability
 *
 *                snk_caps_size: sink capability length
 *
 * @return:  1: success.  0: fail
 *
 */
u8 send_snk_cap(const u8 *snk_caps, u8 snk_caps_size)
{
	memcpy(pd_snk_pdo, snk_caps, snk_caps_size);
	pd_snk_pdo_cnt = snk_caps_size / PD_ONE_DATA_OBJECT_SIZE;

	/*configure sink cap */
	return interface_send_msg_timeout(TYPE_PWR_SNK_CAP, pd_snk_pdo,
					  pd_snk_pdo_cnt * 4, INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure) the DP's sink capability to Ohio's downstream device
 *
 * @param:  dp_snk_caps: PDO buffer pointer of DP sink capability
 *
 *                dp_snk_caps_size: DP sink capability length
 *
 * @return:  1: success.  0: fail
 *
 */
u8 send_dp_snk_cfg(const u8 *dp_snk_caps, u8 dp_snk_caps_size)
{
	memcpy(configure_DP_caps, dp_snk_caps, dp_snk_caps_size);
	interface_send_dp_caps();
	return 1;
}

/**
 * @desc:   Interface that AP initialze the DP's capability of Ohio, as source device
 *
 * @param:  dp_caps: DP's capability  pointer of source
 *
 *                dp_caps_size: source DP capability length
 *
 * @return:  1: success.  0: fail
 *
 */
u8 send_src_dp_cap(const u8 *dp_caps, u8 dp_caps_size)
{
	if (NULL == dp_caps)
		return CMD_FAIL;
	if ((dp_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
	    (dp_caps_size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}

	memcpy(src_dp_caps, dp_caps, dp_caps_size);

	/*configure source DP cap */
	return interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
					  src_dp_caps, dp_caps_size, INTERFACE_TIMEOUT);
}

u8 send_dp_snk_identity(const u8 *snk_ident, u8 snk_ident_size)
{
	return interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
					  (u8 *) snk_ident, snk_ident_size, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The Interface AP set the VDM packet to Ohio
 *
 * @param:  vdm:  object buffer pointer of VDM
 *
 *                size: vdm packet size
 *
 * @return:  0: success 1: fail
 *
 */
u8 send_vdm(const u8 *vdm, u8 size)
{
	u8 tmp[32] = { 0 };

	if (NULL == vdm)
		return CMD_FAIL;
	if (size > 3 && size < 32) {
		memcpy(tmp, vdm, size);
		if (tmp[2] == 0x01 && tmp[3] == 0x00) {
			tmp[3] = 0x40;
			return interface_send_msg_timeout(TYPE_VDM, tmp, size, INTERFACE_TIMEOUT);
		}
	}
	return 1;
}

/**
 * @desc:   The Interface AP set the SVID packet to Ohio
 *
 * @param:  svid:  object buffer pointer of svid
 *
 *                size: svid packet size
 *
 * @return:  0: success 1: fail
 *
 */
u8 send_svid(const u8 *svid, u8 size)
{
	u8 tmp[4] = {
		0
	};
	if (NULL == svid || size != 4)
		return CMD_FAIL;
	memcpy(tmp, svid, size);
	return interface_send_msg_timeout(TYPE_SVID, tmp, size, INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure) the sink capability to Ohio's downstream device
 *
 * @param:  snk_caps: PDO buffer pointer of sink capability
 *
 *                snk_caps_size: sink capability length
 *
 * @return:  1: success.  0: fail
 *
 */
u8 send_rdo(const u8 *rdo, u8 size)
{
	u8 i;

	if (NULL == rdo)
		return CMD_FAIL;
	if ((size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
	    (size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}
	for (i = 0; i < size; i++)
		pd_rdo[i] = *rdo++;

	return interface_send_msg_timeout(TYPE_PWR_OBJ_REQ, pd_rdo, size, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send  PR_Swap command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
u8 send_power_swap(void)
{
	return interface_pr_swap();
}

/**
 * @desc:   The interface AP will send DR_Swap command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
u8 send_data_swap(void)
{
	return interface_dr_swap();
}

/**
 * @desc:   The interface AP will send accpet command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
u8 send_accept(void)
{
	return interface_send_msg_timeout(TYPE_ACCEPT, 0, 0, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send reject command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
u8 send_reject(void)
{
	return interface_send_msg_timeout(TYPE_REJECT, 0, 0, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send soft reset command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
u8 send_soft_reset(void)
{
	return interface_send_soft_rst();
}

/**
 * @desc:   The interface AP will send hard reset command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
u8 send_hard_reset(void)
{
	return interface_send_hard_rst();
}

char *interface_to_str(unsigned char header_type)
{
	return (header_type == TYPE_PWR_SRC_CAP) ? "src cap" :
	    (header_type == TYPE_PWR_SNK_CAP) ? "snk cap" :
	    (header_type == TYPE_PWR_OBJ_REQ) ? "RDO" :
	    (header_type == TYPE_DP_SNK_IDENTITY) ? "snk identity" :
	    (header_type == TYPE_SVID) ? "svid" :
	    (header_type == TYPE_PSWAP_REQ) ? "PR_SWAP" :
	    (header_type == TYPE_DSWAP_REQ) ? "DR_SWAP" :
	    (header_type == TYPE_GOTO_MIN_REQ) ? "GOTO_MIN" :
	    (header_type == TYPE_DP_ALT_ENTER) ? "DPALT_ENTER" :
	    (header_type == TYPE_DP_ALT_EXIT) ? "DPALT_EXIT" :
	    (header_type == TYPE_VCONN_SWAP_REQ) ? "VCONN_SWAP" :
	    (header_type == TYPE_GET_DP_SNK_CAP) ? "GET_SINK_DP_CAP" :
	    (header_type == TYPE_DP_SNK_CFG) ? "dp cap" :
	    (header_type == TYPE_SOFT_RST) ? "Software Reset" :
	    (header_type == TYPE_HARD_RST) ? "Hardware Reset" :
	    (header_type == TYPE_RESTART) ? "Restart" :
	    (header_type == TYPE_PD_STATUS_REQ) ? "PD Status" :
	    (header_type == TYPE_ACCEPT) ? "ACCEPT" :
	    (header_type == TYPE_REJECT) ? "REJECT" :
	    (header_type == TYPE_VDM) ? "VDM" :
	    (header_type == TYPE_RESPONSE_TO_REQ) ? "Response to Request" : "Unknown";
}

inline unsigned char cac_checksum(unsigned char *pSendBuf, unsigned char len)
{
	unsigned char i;
	unsigned char checksum;

	checksum = 0;
	for (i = 0; i < len; i++)
		checksum += *(pSendBuf + i);

	return (u8) (0 - checksum);
}

#ifdef USB_PD_WAIT_LOCK
static DEFINE_RWLOCK(usb_pd_cmd_rwlock);
int usb_pd_cmd_counter = 0;
u8 usb_pd_cmd = 0;

#else
DECLARE_COMPLETION(usb_pd_complete);
DEFINE_SPINLOCK(usb_pd_cmd_lock);
u8 CUR_REQUESTING_PD_CMD = 0xff;

#endif				/*  */
u8 usb_pd_cmd_status = 0xff;

#define DATA_ROLE_IS_DFP    0x2
#define POWER_ROLE_IS_SOURCE    0x1

u8 fetch_data_role_from_pd_result(void)
{
	u8 pd_status = 0x00;
#ifdef USB_PD_WAIT_LOCK
	read_lock(&usb_pd_cmd_rwlock);
	pd_status = usb_pd_cmd_status;
	write_unlock_irq(&usb_pd_cmd_rwlock);
#else
	spin_lock_irq(&usb_pd_cmd_lock);
	pd_status = usb_pd_cmd_status;
	spin_unlock_irq(&usb_pd_cmd_lock);
#endif

	/* DFP 1, UFP 0 */
	return pd_status & DATA_ROLE_IS_DFP;
}

u8 fetch_power_role_from_pd_result(void)
{
	u8 pd_status = 0x00;
#ifdef USB_PD_WAIT_LOCK
	read_lock(&usb_pd_cmd_rwlock);
	pd_status = usb_pd_cmd_status;
	write_unlock_irq(&usb_pd_cmd_rwlock);
#else
	spin_lock_irq(&usb_pd_cmd_lock);
	pd_status = usb_pd_cmd_status;
	spin_unlock_irq(&usb_pd_cmd_lock);
#endif

	/* DFP 1, UFP 0 */
	return pd_status & POWER_ROLE_IS_SOURCE;
}

u8 wait_pd_cmd_timeout(PD_MSG_TYPE pd_cmd, int pd_cmd_timeout)
{
#ifdef USB_PD_WAIT_LOCK
	unsigned long expire;
	u8 cmd_status = 0;

	write_lock_irq(&usb_pd_cmd_rwlock);
	usb_pd_cmd_counter = 1;
	write_unlock_irq(&usb_pd_cmd_rwlock);
	pr_info("wait_pd_cmd_timeout\n");

	/*looply check counter to be changed to 0 in interface interrupt */
	expire = msecs_to_jiffies(pd_cmd_timeout) + jiffies;
	while (1) {
		if (time_before(expire, jiffies)) {
			write_lock_irq(&usb_pd_cmd_rwlock);
			usb_pd_cmd_counter = 0;
			cmd_status = 0;
			write_unlock_irq(&usb_pd_cmd_rwlock);

			return CMD_FAIL;
		}
		read_lock(&usb_pd_cmd_rwlock);
		if (usb_pd_cmd_counter <= 0) {
			cmd_status = usb_pd_cmd_status;
			read_unlock(&usb_pd_cmd_rwlock);

			if (usb_pd_cmd == pd_cmd) {
				read_unlock(&usb_pd_cmd_rwlock);
				return CMD_SUCCESS;
			}
		}
		read_unlock(&usb_pd_cmd_rwlock);
	}
	return CMD_FAIL;

#else				/*  */
	unsigned long left_time = msecs_to_jiffies(pd_cmd_timeout);
	u8 cmd_status = 0xff;

	while (left_time > 0) {
		left_time = wait_for_completion_timeout(&usb_pd_complete, left_time);
		if (0 == left_time) {
			pr_info("Wait for PD timeout\n");
			return CMD_FAIL;
		}
		if (left_time > 0) {
			spin_lock_irq(&usb_pd_cmd_lock);
			if (CUR_REQUESTING_PD_CMD != pd_cmd) {
				pr_info("pd_cmd %x not match rsp %x\n",
					pd_cmd, CUR_REQUESTING_PD_CMD);
				spin_unlock_irq(&usb_pd_cmd_lock);
				continue;
			}
			cmd_status = usb_pd_cmd_status;
			spin_unlock_irq(&usb_pd_cmd_lock);
			return CMD_SUCCESS;
		}
	}

#endif				/*  */
	return CMD_SUCCESS;
}

void printb(const char *buf, size_t size)
{
#ifdef OHIO_DEBUG
	while (size--)
		pr_err("%0x ", *buf++);
	pr_err("\n");
#endif
}

void interface_init(void)
{
	pbuf_rx_front = 0;
	pbuf_tx_rear = 0;

}

void send_initialized_setting(void)
{
	/* OHO-439, in AP side, for the interoperability, */
	/* set the try.UFP period to 0x96*2 = 300ms. */
#ifndef PD_CTS_TEST
	OhioWriteReg(0x6A, 0x96);
#endif

	/* send TYPE_PWR_SRC_CAP init setting */
	send_pd_msg(TYPE_PWR_SRC_CAP, (const char *)init_src_caps, sizeof(init_src_caps));

	/* send TYPE_PWR_SNK_CAP init setting */
	send_pd_msg(TYPE_PWR_SNK_CAP, (const char *)init_snk_cap, sizeof(init_snk_cap));

	/* send TYPE_DP_SNK_IDENTITY init setting */
	send_pd_msg(TYPE_DP_SNK_IDENTITY, init_snk_ident, sizeof(init_snk_ident));

	/* send TYPE_SVID init setting */
	send_pd_msg(TYPE_SVID, init_svid, sizeof(init_svid));
}

inline void receiver_reset_queue(void)
{
	rx_buf_front() = rx_buf_rear();
	OhioWriteReg(RX_BUF_FRONT, rx_buf_front());
}

void handle_intr_vector(void)
{
	u8 intr_vector = OhioReadReg(OHIO_INTERFACE_CHANGE_INT);
	u8 status;

#ifdef OHIO_DEBUG
	pr_info(" intr vector = %x\n", intr_vector);
#endif
	if (intr_vector) {
		OhioWriteReg(OHIO_INTERFACE_CHANGE_INT, intr_vector & (~intr_vector));
		clear_soft_interrupt();
		if (intr_vector & RECEIVED_MSG)
			polling_interface_msg(INTERACE_TIMEOUT_MS);
		if (intr_vector & VBUS_CHANGE) {
			status = OhioReadReg(OHIO_SYSTEM_STSTUS);
			pd_vbus_control_default_func(status & VBUS_STATUS);
		}
		if (intr_vector & VCONN_CHANGE) {
			status = OhioReadReg(OHIO_SYSTEM_STSTUS);
			pd_vconn_control_default_func(status & VCONN_STATUS);
		}
		if (intr_vector & CC_STATUS_CHANGE) {
			status = OhioReadReg(NEW_CC_STATUS);
			pd_cc_status_default_func(status);
		}
		if (intr_vector & DATA_ROLE_CHANGE) {
			status = OhioReadReg(OHIO_SYSTEM_STSTUS);
			pd_drole_change_default_func(status & DATA_ROLE);
		}
	}

}

/* 0, send interface msg timeout
 * 1 successful
 */
u8 interface_send_msg_timeout(u8 type, u8 *pbuf, u8 len, int timeout_ms)
{
	int msg_total_len = 0;
	unsigned long expire = 0;
	u8 tmp_len = 0;
	s8 rear, front;
	u8 buf[32] = { 0 };

	/* full, return 0 */
	buf[0] = len + 1;	/* cmd */
	buf[1] = type;
	memcpy(buf + 2, pbuf, len);
	/* cmd + checksum */
	buf[len + 2] = cac_checksum(buf, len + 1 + 1);
	msg_total_len = len + 3;
	rear = tx_buf_rear();

#ifdef OHIO_DEBUG
	pr_info("snd type=%d len=%d\n", type, msg_total_len);
#endif

	expire = msecs_to_jiffies(timeout_ms) + jiffies;
	while (msg_total_len > 0) {

		front = tx_buf_front();
		if (front == rear) {
			tmp_len = 7;

			if (tmp_len > msg_total_len)
				tmp_len = msg_total_len;

			if ((TX_BUF_START + rear + tmp_len) >
			    (TX_BUF_START + MAX_SEND_BUF_SIZE - 1)) {
				OhioWriteBlockReg(TX_BUF_START + rear, MAX_SEND_BUF_SIZE - rear,
						  buf + len + 3 - msg_total_len);
				if (tmp_len - (MAX_SEND_BUF_SIZE - rear))
					OhioWriteBlockReg(TX_BUF_START,
							  tmp_len - (MAX_SEND_BUF_SIZE - rear),
							  buf + len + 3 - msg_total_len +
							  (MAX_SEND_BUF_SIZE - rear));
			} else
				OhioWriteBlockReg(TX_BUF_START + rear, tmp_len,
						  buf + len + 3 - msg_total_len);

			msg_total_len -= tmp_len;
			/* update rear position */
			rear = (rear + tmp_len) % MAX_SEND_BUF_SIZE;
			OhioWriteReg(TX_BUF_REAR, rear);
#ifdef OHIO_DEBUG
			pr_info("Tx len=%d remainder=%d, front=%d, rear=%d\n", tmp_len,
				msg_total_len, front, rear);
#endif
		}

		if (time_before(expire, jiffies)) {
			pr_info("TX Timeout %d\n", msg_total_len);
			return CMD_FAIL;
		}
	}
	tx_buf_rear() = rear;

	expire = msecs_to_jiffies(timeout_ms) + jiffies;
	while ((msg_total_len = sender_get_ack_status()) == 0) {
		if (time_before(expire, jiffies)) {
			pr_info("TX Timeout %d\n", msg_total_len);
			return CMD_FAIL;
		}
	}
	if (msg_total_len == 0x01) {
		pr_info("succ << %s\n", interface_to_str(buf[1]));
		memcpy(InterfaceSendBuf, buf, msg_total_len);
	} else {
		pr_info("Ack error %d\n", msg_total_len);
		printb(buf, len + 3);
		sender_set_ack_status(0x00);
		return CMD_FAIL;
	}
	return CMD_SUCCESS;
}

/* Desc: polling private interface interrupt request message
 * Args: timeout,  block timeout time, if = 0, is noblock
 * Ret: if return 0, success recv one message
 *  the message pointer, it's statically alloced in the function
 *  if > 0,  Error happen
 * Interface's Format:
 *	1Byte Len + 1Byte Type + Len Bytes Data + 1Byte checksum   */
u8 polling_interface_msg(int timeout_ms)
{
	u8 checksum = 0;
	unsigned long expire = 0;
	u8 front, rear, i, tmp_len, msg_total_len, rec_len, first_len, sec_len;
	u8 buf[32] = { 0 };

	expire = msecs_to_jiffies(timeout_ms) + jiffies;

	/*first block read to get total length */
	rec_len = 0;
	front = rx_buf_front();
	rear = rx_buf_rear();
	if (front != rear) {

		if (rear > front) {
			tmp_len = rear - front;
			OhioReadBlockReg(RX_BUF_START + front, tmp_len, buf + rec_len);
		} else {
			tmp_len = (MAX_SEND_BUF_SIZE - (front - rear)) % MAX_SEND_BUF_SIZE;
			OhioReadBlockReg(RX_BUF_START + front, MAX_SEND_BUF_SIZE - front,
					 buf + rec_len);
			if (rear > 0)
				OhioReadBlockReg(RX_BUF_START, rear,
						 buf + rec_len + MAX_SEND_BUF_SIZE - front);
		}
		rec_len += tmp_len;
		/* update front position */
		front = (front + tmp_len) % MAX_SEND_BUF_SIZE;
		OhioWriteReg(RX_BUF_FRONT, front);
#ifdef OHIO_DEBUG
		pr_info("Rx len=%d Received len=%d, front=%d, rear=%d\n", tmp_len, rec_len, front,
			rear);
#endif
	}

	if (buf[0] == 0 || buf[0] > 31)
		goto err_rcv_len;

	msg_total_len = buf[0] + 2;
#ifdef OHIO_DEBUG
	pr_info("total receive message len=%d\n", msg_total_len);
#endif

	/* remain block read */
	while (msg_total_len > rec_len) {

		rear = rx_buf_rear();

		if (rear > front)
			tmp_len = rear - front;
		else
			tmp_len = (MAX_SEND_BUF_SIZE - front + rear) % MAX_SEND_BUF_SIZE;

		if (front == ((rear + 1) % MAX_SEND_BUF_SIZE)
		    || (rec_len + tmp_len) >= msg_total_len) {

			if (rear > front) {

				if (tmp_len > (msg_total_len - rec_len))
					tmp_len = msg_total_len - rec_len;
				OhioReadBlockReg(RX_BUF_START + front, tmp_len, buf + rec_len);
			} else {
				first_len = MAX_SEND_BUF_SIZE - front;
				sec_len = rear;

				if (tmp_len > (msg_total_len - rec_len)) {
					tmp_len = msg_total_len - rec_len;
					if (first_len > (msg_total_len - rec_len)) {
						first_len = msg_total_len - rec_len;
						sec_len = 0;
					} else
						sec_len = msg_total_len - rec_len - first_len;
				}

				OhioReadBlockReg(RX_BUF_START + front, first_len, buf + rec_len);
				if (sec_len > 0)
					OhioReadBlockReg(RX_BUF_START, sec_len,
							 buf + rec_len + MAX_SEND_BUF_SIZE - front);
			}
			rec_len += tmp_len;
			/* update front position */
			front = (front + tmp_len) % MAX_SEND_BUF_SIZE;
			OhioWriteReg(RX_BUF_FRONT, front);
#ifdef OHIO_DEBUG
			pr_info("Rx len=%d Received len=%d, front=%d, rear=%d\n", tmp_len, rec_len,
				front, rear);
#endif
		}

		if (time_before(expire, jiffies))
			goto err_timeout;
	}
	rx_buf_front() = front;
	/* checksum judgement */
	for (i = 0; i < msg_total_len; i++)
		checksum += buf[i];
	if (checksum == 0) {
		receiver_set_ack_status(0x01);
		pr_info("\n>>%s\n", interface_to_str(buf[1]));
		memcpy(InterfaceRecvBuf, buf, msg_total_len);
		dispatch_rcvd_pd_msg((PD_MSG_TYPE) buf[1], &(buf[2]), buf[0] - 1);
	} else {
		receiver_set_ack_status(0x02);
		pr_info("checksum error: %x\n", (u16) checksum);
		printb(buf, rec_len);
		return CMD_FAIL;
	}
	return CMD_SUCCESS;

err_timeout:
	sender_set_ack_status(0x03);
	receiver_reset_queue();
	pr_info("err: RX Timeout %d\n", rec_len);
	printb(buf, rec_len);
	return CMD_FAIL;
err_rcv_len:
	pr_info("err: rx length error len %d\n", buf[0]);
	receiver_set_ack_status(0x02);
	receiver_reset_queue();
	return CMD_FAIL;
}

void interface_send_dp_caps(void)
{
	memcpy(InterfaceSendBuf + 2, configure_DP_caps, 4);
	memcpy(InterfaceSendBuf + 2 + 4, DP_caps, 4);
	interface_send_msg_timeout(TYPE_DP_SNK_CFG, InterfaceSendBuf + 2, 4 + 4, INTERFACE_TIMEOUT);
}

void interface_send_status(u8 cmd_type, u8 status)
{
	InterfaceSendBuf[2] = cmd_type;
	InterfaceSendBuf[3] = status;
	interface_send_msg_timeout(TYPE_RESPONSE_TO_REQ, InterfaceSendBuf + 2,
				   2, INTERFACE_TIMEOUT);
}

/* define max request current 3A and voltage 5V */
#define MAX_REQUEST_VOLTAGE 5000
#define MAX_REQUEST_CURRENT 900
#define set_rdo_value(v0, v1, v2, v3)	\
	do {				\
		pd_rdo[0] = (v0);	\
		pd_rdo[1] = (v1);	\
		pd_rdo[2] = (v2);	\
		pd_rdo[3] = (v3);	\
	} while (0)

u8 sel_voltage_pdo_index = 0x02;
/* default request max RDO */
u8 build_rdo_from_source_caps(u8 obj_cnt, u8 *buf)
{
	u8 i = 0;
	u16 pdo_h = 0, pdo_l = 0, pdo_h_tmp, pdo_l_tmp;
	u16 max_request_ma;
	u32 pdo_max, pdo_max_tmp = 0;
	u16 return_val = 0;

	pdo_max = 0;
	obj_cnt &= 0x07;

	/* find the max voltage pdo */
	for (i = 0; i < obj_cnt; i++) {
		pdo_l_tmp = buf[i * 4 + 0];
		pdo_l_tmp |= (u16) buf[i * 4 + 1] << 8;
		pdo_h_tmp = buf[i * 4 + 2];
		pdo_h_tmp |= (u16) buf[i * 4 + 3] << 8;

		/* get max voltage now */
		pdo_max_tmp = (u16) (((((pdo_h_tmp & 0xf) << 6) | (pdo_l_tmp >> 10)) & 0x3ff) * 50);
		if (pdo_max_tmp > pdo_max) {
			pdo_max = pdo_max_tmp;
			pdo_l = pdo_l_tmp;
			pdo_h = pdo_h_tmp;
			sel_voltage_pdo_index = i;
		}
	}
#ifdef OHIO_DEBUG
	pr_info("maxV=%d, cnt %d index %d\n", pdo_max_tmp, obj_cnt, sel_voltage_pdo_index);
#endif
	if ((pdo_h & (3 << 14)) != (PDO_TYPE_BATTERY >> 16)) {
		max_request_ma = (u16) ((pdo_l & 0x3ff) * 10);
#ifdef OHIO_DEBUG
		pr_info("maxMa %d\n", max_request_ma);
#endif
		/* less than 900mA */
		if (max_request_ma < MAX_REQUEST_CURRENT) {
			pdo_max =
			    RDO_FIXED(sel_voltage_pdo_index + 1, max_request_ma, max_request_ma, 0);
			pdo_max |= RDO_CAP_MISMATCH;
			set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
				      (pdo_max >> 16) & 0xff, (pdo_max >> 24) & 0xff);
			return_val = 1;
		} else {
			pdo_max =
			    RDO_FIXED(sel_voltage_pdo_index + 1,
				      MAX_REQUEST_CURRENT, MAX_REQUEST_CURRENT, 0);
			set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
				      (pdo_max >> 16) & 0xff, (pdo_max >> 24) & 0xff);

			return_val = 1;
		}
	} else {
		pdo_max =
		    RDO_FIXED(sel_voltage_pdo_index + 1, MAX_REQUEST_CURRENT,
			      MAX_REQUEST_CURRENT, 0);
		set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
			      (pdo_max >> 16) & 0xff, (pdo_max >> 24) & 0xff);
		return_val = 1;
	}

	pr_info("RDO Mismatch !!!\n");
	set_rdo_value(0x0A, 0x28, 0x00, 0x10);

	return return_val;
}

u32 change_bit_order(u8 *pbuf)
{
	return ((u32) pbuf[3] << 24) | ((u32) pbuf[2] << 16)
	    | ((u32) pbuf[1] << 8) | pbuf[0];
}

u8 pd_check_requested_voltage(u32 rdo)
{
	int max_ma = rdo & 0x3FF;
	int op_ma = (rdo >> 10) & 0x3FF;
	int idx = rdo >> 28;

	u32 pdo;
	u32 pdo_max;

	if (!idx || idx > pd_src_pdo_cnt) {
		pr_info("rdo = %x, Requested RDO is %d, Provided RDO number is %d\n", rdo,
			(unsigned int)idx, (unsigned int)pd_src_pdo_cnt);
		return 0;	/* Invalid index */
	}
	/* Update to pass TD.PD.SRC.E12 Reject Request */
	pdo = change_bit_order(pd_src_pdo + ((idx - 1) * 4));
	pdo_max = (pdo & 0x3ff);
#ifdef OHIO_DEBUG
	pr_info("pdo_max = %x\n", pdo_max);
#endif
	/* TRACE3("Requested  %d/~%d mA, idx %d\n",      (u16)op_ma * 10, (u16)max_ma *10, (u16)idx); */
	/* check current ... */
	if (op_ma > pdo_max)	/* Update to pass TD.PD.SRC.E12 Reject Request */
		return 0;	/* too much op current */
	if (max_ma > pdo_max)	/* Update to pass TD.PD.SRC.E12 Reject Request */
		return 0;	/* too much max current */



	return 1;
}

/* Receive Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * return:  0, fail;   1, success
  */
u8 recv_pd_source_caps_default_callback(void *para, u8 para_len)
{
	u8 *pdo = 0;

	pdo = (u8 *) para;
	if (para_len % 4 != 0)
		return 0;
	if (build_rdo_from_source_caps(para_len / 4, para)) {
		interface_send_request();
		pr_info("Snd RDO %x %x %x %x succ\n", pd_rdo[0], pd_rdo[1], pd_rdo[2], pd_rdo[3]);
	}
	return 1;
}

/* Receive Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * return:  0, fail;   1, success
  */
u8 recv_pd_sink_caps_default_callback(void *para, u8 para_len)
{
	u8 *pdo = 0;

	pdo = (u8 *) para;
	if (para_len % 4 != 0)
		return 0;
	if (para_len > VDO_SIZE)
		return 0;
	/*do what you want to do */
	return 1;
}

/* Receive Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * return:  0, fail;   1, success
  */
u8 recv_pd_pwr_object_req_default_callback(void *para, u8 para_len)
{
	u8 *pdo = (u8 *) para;
	u32 rdo = 0;

	if (para_len != 4)
		return 0;

	rdo = pdo[0] | (pdo[1] << 8) | (pdo[2] << 16) | (pdo[3] << 24);
	if (pd_check_requested_voltage(rdo))
		send_accept();
	else
		interface_send_reject();

	return 1;
}

/* Receive accept message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
  * return:  0, fail;   1, success
  */
u8 recv_pd_accept_default_callback(void *para, u8 para_len)
{
	return 1;
}

/* Receive reject message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
  * return:  0, fail;   1, success
  */
u8 recv_pd_reject_default_callback(void *para, u8 para_len)
{
	return 1;
}

/* Receive reject message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
  * return:  0, fail;   1, success
  */
u8 recv_pd_goto_min_default_callback(void *para, u8 para_len)
{

	return 1;
}

void pd_vbus_control_default_func(bool on)
{
	/* to enable or disable VBus in 35ms */
#ifdef OHIO_DEBUG
	pr_info("vbus control %d\n", (int)on);
#endif
}

void pd_vconn_control_default_func(bool on)
{
	/* to enable or disable VConn */

}

void pd_cc_status_default_func(u8 cc_status)
{
	/* cc status */
#ifdef OHIO_DEBUG
	pr_info("cc status %x\n", cc_status);
#endif
}

void pd_drole_change_default_func(bool on)
{
	/* data role changed */

}

/*PD Status command response, default callback function.
  *It can be change by customer for redevelopment
  * Byte0: CC status from ohio
  * Byte1: misc status from ohio
  * Byte2: debug ocm FSM state from ohio
  */
u8 cc_status = 0;
u8 misc_status = 0;
u8 pd_fsm_status = 0;
void interface_get_status_result(void)
{
	cc_status = InterfaceRecvBuf[3];
	pr_info("Ohio CC Status:%x\n", cc_status);
	misc_status = InterfaceRecvBuf[4];
	pr_info("Ohio misc status:%x\n", misc_status);
	pd_fsm_status = InterfaceRecvBuf[5];
	pr_info("Ohio pd_fsm_status:%x\n", pd_fsm_status);
}

u8 recv_pd_cmd_rsp_default_callback(void *para, u8 para_len)
{
	u8 need_notice_pd_cmd = 0;
	u8 pd_cmd = 0;

	para = para;
	para_len = para_len;
	pd_cmd = RESPONSE_REQ_TYPE();
	switch (RESPONSE_REQ_TYPE()) {
	case TYPE_PD_STATUS_REQ:
		interface_get_status_result();
		need_notice_pd_cmd = 1;
		break;
	case TYPE_DSWAP_REQ:
		need_notice_pd_cmd = 1;
		if (RESPONSE_REQ_RESULT() == CMD_SUCCESS)
			pr_info("pd_cmd DRSwap result is successful\n");
		else if (RESPONSE_REQ_RESULT() == CMD_REJECT)
			pr_info("pd_cmd DRSwap result is rejected\n");
		else if (RESPONSE_REQ_RESULT() == CMD_BUSY)
			pr_info("pd_cmd DRSwap result is busy\n");
		else if (RESPONSE_REQ_RESULT() == CMD_FAIL)
			pr_info("pd_cmd DRSwap result is fail\n");
		else
			pr_info("pd_cmd DRSwap result is unknown\n");
		break;
	case TYPE_PSWAP_REQ:
		need_notice_pd_cmd = 1;
		if (RESPONSE_REQ_RESULT() == CMD_SUCCESS) {
			pr_info("pd_cmd PRSwap result is successful\n");
			mt_power_role_swap_ready();
		} else if (RESPONSE_REQ_RESULT() == CMD_REJECT)
			pr_info("pd_cmd PRSwap result is rejected\n");
		else if (RESPONSE_REQ_RESULT() == CMD_BUSY)
			pr_info("pd_cmd PRSwap result is busy\n");
		else if (RESPONSE_REQ_RESULT() == CMD_FAIL)
			pr_info("pd_cmd PRSwap result is fail\n");
		else
			pr_info("pd_cmd PRSwap result is unknown\n");
		break;
	default:
		break;
	}
	if (need_notice_pd_cmd) {

#ifdef USB_PD_WAIT_LOCK
		/* check pd cmd has been locked */
		write_lock_irq(&usb_pd_cmd_rwlock);
		usb_pd_cmd_status = RESPONSE_REQ_RESULT();
		usb_pd_cmd = pd_cmd;
		if (usb_pd_cmd_counter)
			usb_pd_cmd_counter = 0;

		write_unlock_irq(&usb_pd_cmd_rwlock);

#else				/*  */
		spin_lock_irq(&usb_pd_cmd_lock);
		CUR_REQUESTING_PD_CMD = pd_cmd;
		usb_pd_cmd_status = RESPONSE_REQ_RESULT();
		spin_unlock_irq(&usb_pd_cmd_lock);
		complete(&usb_pd_complete);

#endif				/*  */
	}
	return CMD_SUCCESS;
}

u8 recv_pd_hard_rst_default_callback(void *para, u8 para_len)
{
#ifdef OHIO_DEBUG
	pr_info("recv pd hard reset\n");
#endif

	return CMD_SUCCESS;
}

/* Receive Data Role Swap message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through init_pd_msg_callback, it it pd_callback is not 0, using the default
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * return:  0, fail;   1, success
  */
u8 recv_pd_dswap_default_callback(void *para, u8 para_len)
{
	/* dswap just notice AP, do nothing */
	return 1;
}

/* Receive Power Role Swap message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through init_pd_msg_callback, it it pd_callback is not 0, using the default
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * return:  0, fail;   1, success
  */
u8 recv_pd_pswap_default_callback(void *para, u8 para_len)
{
	/* pswap just notice AP, do nothing */
	mt_do_power_role_swap();
	return 1;
}
static pd_callback_t pd_callback_array[256] = { 0 };

pd_callback_t get_pd_callback_fnc(PD_MSG_TYPE type)
{
	pd_callback_t fnc = 0;

	if (type < 256)
		fnc = pd_callback_array[type];
	return fnc;
}

void set_pd_callback_fnc(PD_MSG_TYPE type, pd_callback_t fnc)
{
	pd_callback_array[type] = fnc;
}

void init_pd_msg_callback(void)
{
	u8 i = 0;

	for (i = 0; i < 256; i++)
		pd_callback_array[i] = 0x0;
}
