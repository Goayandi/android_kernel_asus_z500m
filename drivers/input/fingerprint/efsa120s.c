#include <linux/module.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/pm.h>

#include <linux/namei.h>
#include <linux/mount.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/poll.h>
#include "asus_fp_id.h"
#include "asus_navigation.h"
#include <../../misc/mediatek/include/mt-plat/mt_boot_common.h>

#define VERSION_LOG	"ELAN FINGER PRINT V1.5"
#define SPI_MAX_SPEED		10000000	/* chagne by customer */
#define _SIGNAL_MODE_					/* define = Signal Event, no def = Key mode */
#define KEY_FP_INT			KEY_POWER 	/* KEY_WAKEUP / change by customer & framework support */
#define KEY_FP_INT2	        KEY_1       /* KEY_WAKEUP / change by customer & framework support */
#define ELAN_FP '1'

#define DISABLE_FP_FUNC 1

#ifdef ASUS_FTM
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#else
#include <linux/platform_device.h>
#endif

static int sig;							/* added v1.43 */
static pid_t pid;						/* added v1.43 */
static unsigned char bsig_pid = 0;		/* added v1.43 */
static unsigned char bState = 1;		/* 1=on, 0=off */

static DECLARE_WAIT_QUEUE_HEAD(elan_poll_wq);
#ifdef ASUS_FTM
static DECLARE_WAIT_QUEUE_HEAD(image_waiter);
#endif
struct completion cmd_done;
struct completion cmd_done_irq;

#define _ELAN_DEBUG_
#ifdef _ELAN_DEBUG_
#define ELAN_DEBUG(dev, format, args ...) 		printk("%s %s: [ELAN]:%5d: " format, dev_driver_string(dev), dev_name(dev), __LINE__, ##args)
#else
#define ELAN_DEBUG(dev, format, args ...)
#endif
#define _ELAN_DEBUG_DATA_
#ifdef _ELAN_DEBUG_DATA_
#define ELAN_DEBUG_DATA(dev, format, args ...) 	printk("%s %s: [ELAN]:%5d: " format, dev_driver_string(dev), dev_name(dev), __LINE__, ##args)
#else
#define ELAN_DEBUG_DATA(dev, format, args ...)
#endif

#define WRITE_REG_HEAD		0x80
#define READ_REG_HEAD		0x40
#define READ_SERIER_REG_HEAD	0xC0
#define INT_NORMAL_HIGH		0x40
#define START_SCAN			0x01
#define START_READ_IMAGE	0x10
#define ADDR_SIZE_WH		0x01
#define ALIG_4_BYTE						/* change by customer */

struct efsa120s_data {
#ifdef ASUS_FTM
	struct spi_device   *spi;
	struct device       *spi_device;
#else
	struct platform_device 	*spi;
	struct device		*platform_device;
#endif
	struct input_dev 	*input_dev;
	struct mutex 		spi_mutex;
	struct cdev 		spi_cdev;
	struct class 		*spi_class;

	/* File I/O for user-space */
	struct mutex 		sysfs_mutex;
	struct miscdevice 	efsa120_dev;	/* char device for ioctl */

	int intr_gpio;
	int	isr;
	int rst_gpio;
	int	cs_pin;
	int osvcc_pin;
	struct regulator *sovcc;
	struct regulator *vcc;
	struct work_struct  work;
	unsigned char bImageReady;
	spinlock_t irq_lock;
	int irq_is_disable;
	wait_queue_head_t efsa_wait;
};

#define FINGERPRINT_IOCTL 0x80
#define ID_IOCTL_INIT _IOW(FINGERPRINT_IOCTL, 0,  int)				/* To Get Raw Image (14->8)*/
#define ID_IOCTL_READ_REGISTER _IOW(FINGERPRINT_IOCTL, 2,  int)
#define ID_IOCTL_WRITE_REGISTER _IOW(FINGERPRINT_IOCTL, 3,  int)
#define ID_IOCTL_RESET _IOW(FINGERPRINT_IOCTL, 6,  int)
#define ID_IOCTL_GET_RAW_IMAGE _IOW(FINGERPRINT_IOCTL, 10, int) 	/* To Get Raw Image (Original)*/
#define ID_IOCTL_STATUS _IOW(FINGERPRINT_IOCTL, 12, int)
#define ID_IOCTL_SET_AUTO_RAW_IMAGE	_IOW(FINGERPRINT_IOCTL, 13, int)
#define ID_IOCTL_GET_AUTO_RAW_IMAGE	_IOW(FINGERPRINT_IOCTL, 14, int)
#define ID_IOCTL_READ_CMD _IOW(FINGERPRINT_IOCTL, 15, int) 			/* General read cmd */
#define ID_IOCTL_WRITE_CMD _IOW(FINGERPRINT_IOCTL, 16, int) 		/* General write cmd */
#define ID_IOCTL_IOIRQ_STATUS _IOW(FINGERPRINT_IOCTL, 17, int) 		/* Use INT to read buffer */
#define ID_IOCTL_SPI_STATUS	_IOW(FINGERPRINT_IOCTL, 18, int) 		/* UPdate SPI Speed & CS delay */
#define ID_IOCTL_SIG_PID _IOW(FINGERPRINT_IOCTL, 19, int) 			/* WOE signal event to pid */

#define ID_IOCTL_POLL_INIT _IOW(FINGERPRINT_IOCTL, 20, int)
#define ID_IOCTL_INPUT_KEYCODE _IOW(FINGERPRINT_IOCTL, 22, int)
#define ID_IOCTL_NAV_WOE _IOW(FINGERPRINT_IOCTL, 4077, int)

static int major_number = 0;
static int minor_number = 0;
static unsigned int IMG_WIDTH = 120;
static unsigned int IMG_HEIGHT = 120;
static unsigned int IMG_WIDTH_DEFAULT = 120;
static unsigned int IMG_HEIGHT_DEFAULT = 120;
static unsigned char * imagebuffer = NULL;							/* for SPI Transfer and orginal data */
static unsigned char status[5] = {0};								/* added by samuel */
static int auto_raw_image_num = 0; 									/* added by samuel */
static int auto_raw_image_count = 0;								/* added by samuel */
static unsigned char bCMD_REG = 0;									/* CMD = 0, REG= 1 */
static int nWaitImage = 100000;										/* wait for max 1ms */
static int nWaitImage_Count = 0;
static int Image_index = 0;
static unsigned char IOIRQ_STATUS = 0;
static unsigned int spi_speed = SPI_MAX_SPEED;						/* 10MHz */
static unsigned short spi_cs_delay = 5;								/* 5us */
static struct workqueue_struct *efsa120s_wq;

#ifdef ASUS_FTM
static int efsa120s_spi_transfer(struct spi_device *spi, char *txbuf, char *rxbuf, int len)
{
	struct efsa120s_data *fp = spi_get_drvdata(spi);
	struct spi_transfer t;
	struct spi_message m;

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);
	t.tx_buf = txbuf;
	t.rx_buf = rxbuf;
	t.bits_per_word = 8;
	t.len = len;
	t.speed_hz = spi_speed;
	spi_message_add_tail(&t, &m);

	return spi_sync(fp->spi, &m);
}
#endif /*endif !FP_TEE*/

static int efsa120s_send_signal_to_pid(void)
{
	struct siginfo info;

	if (bsig_pid == 0)
		return -1;
	if (bState == 0) 					/* Display off */
		info.si_signo = sig + 1;		/* global para */
	else								/* Display on */
		info.si_signo = sig;			/* global para */
	info.si_errno = 0;
	info.si_code = SI_USER;				/* send by kill, sigsend, raise ? */
	info.si_pid = get_current()->pid;	/* global para */
	info.si_uid = from_kuid_munged(current_user_ns(), current_uid());

	/* printk("[ELAN] send signal(0x%x) to pid(0x%x).\r\n", info.si_signo, pid); */
	return kill_proc_info(info.si_signo, &info, pid);
}

#ifdef ASUS_FTM
static int efsa120s_kmalloc_image(void)
{
#ifdef ALIG_4_BYTE
	imagebuffer = kmalloc(sizeof(unsigned char) * \
			(IMG_WIDTH*2+4)*IMG_HEIGHT, GFP_KERNEL); /* add 4 = 2 + 2 is for aligment */
#else
	imagebuffer = kmalloc(sizeof(unsigned char) * \
			(IMG_WIDTH*2+2)*IMG_HEIGHT, GFP_KERNEL); /* add 2 is cmd & dummy */
#endif
	if (imagebuffer == NULL)
		goto km_imagebuffer_fail;

	return 0;

km_imagebuffer_fail:
	printk("[ELAN]: efsa120s_kmalloc_image fail. Memory alloc Error.\r\n");
	return -ENOMEM;
}

static int efsa120s_receive_image(struct spi_device *spi)
{
	int i;
#ifdef ALIG_4_BYTE
	char txbuf[IMG_WIDTH*2+4];
#else
	char txbuf[IMG_WIDTH*2+2];
#endif
	/* SPI Command (Image_Burst_Read) */
	txbuf[0] = START_READ_IMAGE;

#ifdef ALIG_4_BYTE /* MTK DMA Mode So Used ALIG 4 Bytes */
	for (i = 0; i < IMG_HEIGHT; i++) {
#ifdef DEBUG_ON
		ELAN_DEBUG_DATA(&spi->dev,"H2[%d] ", i);
#endif
		efsa120s_spi_transfer(spi, txbuf, \
			&(imagebuffer[i*(IMG_WIDTH*2+4)]), IMG_WIDTH*2+2);
	}
#else
	for (i = 0; i < IMG_HEIGHT; i++) {
#ifdef DEBUG_ON
		ELAN_DEBUG_DATA(&spi->dev,"H2[%d] ", i);
#endif
		efsa120s_spi_transfer(spi, txbuf, \
			&(imagebuffer[i*(IMG_WIDTH*2+2)]), IMG_WIDTH*2+2);
	}
#endif
	return 0;
}

static int efsa120s_receive_image_1BYte(struct spi_device *spi)
{
	int i;
#ifdef ALIG_4_BYTE
	char txbuf[IMG_WIDTH+4];
#else
	char txbuf[IMG_WIDTH+2];
#endif

	/* SPI Command (Image_Burst_Read) */
	txbuf[0] = START_READ_IMAGE;

#ifdef ALIG_4_BYTE /* MTK DMA Mode So Used ALIG 4 Bytes */
	for (i = 0; i < IMG_HEIGHT; i++) {
#ifdef DEBUG_ON
		ELAN_DEBUG_DATA(&spi->dev,"H1[%d] ", i);
#endif
		efsa120s_spi_transfer(spi, txbuf, \
			&(imagebuffer[i*(IMG_WIDTH+4)]), IMG_WIDTH+2);
	}
#else
	for (i = 0; i < IMG_HEIGHT; i++) {
#ifdef DEBUG_ON
		ELAN_DEBUG_DATA(&spi->dev,"H1[%d] ", i);
#endif
		efsa120s_spi_transfer(spi, txbuf, \
			&(imagebuffer[i*(IMG_WIDTH+2)]), IMG_WIDTH+2);
	}
#endif
	return 0;
}
#endif

#ifdef ASUS_FTM
static int efsa120s_fingerprint_init(struct spi_device *spi, unsigned char data[])
{
	struct efsa120s_data *fp = spi_get_drvdata(spi);
	unsigned char tbl_init_para1[] = { 0x00, 0x5A, 0x08, 0x04, 0x0B, \
									0x71, 0x0C, 0x49, 0x0F, 0x2B, \
									0x11, 0x2B, 0x13, 0x28, 0x15, \
									0x28, 0x18, 0x04, 0x21, 0x20, \
									0x22, 0x36, 0x06, 0xED, 0x05, \
									0x6F };
	unsigned char tbl_init_para2[] = { 0x2A, 0x0F };
	unsigned char tbl_init_para3[] = { 0x2A, 0x0B };
	unsigned char tbl_init_para4[] = { 0x2A, 0x4F };	/* FUNC Enable */
	unsigned char tbl_init_para5[] = { 0x2c, 0x09 };	/* INTERRUPT */
	char txbuf[4], rxbuf[4];
	int i;
#else
static int efsa120s_fingerprint_init(struct platform_device *spi, unsigned char data[])
{
	struct efsa120s_data *fp = platform_get_drvdata(spi);
#endif /*endif !FP_TEE*/

	gpio_direction_output(fp->rst_gpio, 0);
	mdelay(5);
	gpio_direction_output(fp->rst_gpio, 1);
	mdelay(50);

#ifdef ASUS_FTM
	/* SPI Command (Fuse Load) */
	txbuf[0] = 0x04;
	efsa120s_spi_transfer(spi ,txbuf, rxbuf, 1);
#ifdef DEBUG_ON
	ELAN_DEBUG(&spi->dev,"Fuse Load\r\n");
#endif
	mdelay(1);

	for (i=0; i<sizeof(tbl_init_para1); i+=2) {
		txbuf[0] = WRITE_REG_HEAD + tbl_init_para1[i];	/* Regist Address */
		txbuf[1] = tbl_init_para1[i+1];					/* Regist Values */
		efsa120s_spi_transfer(spi, txbuf, rxbuf, 2);
	}
#ifdef DEBUG_ON
	ELAN_DEBUG(&spi->dev,"Write REG Data Value\r\n");
#endif
	mdelay(1);

	for (i=0; i<sizeof(tbl_init_para2); i+=2) {
		txbuf[0] = WRITE_REG_HEAD + tbl_init_para2[i];	/* Regist Address */
		txbuf[1] = tbl_init_para2[i+1];					/* Regist Values */
		efsa120s_spi_transfer(spi, txbuf, rxbuf, 2);
	}
#ifdef DEBUG_ON
	ELAN_DEBUG(&spi->dev,"AFE Regulator turn on\r\n");
#endif
	mdelay(1);

	for (i=0; i<sizeof(tbl_init_para3); i+=2) {
		txbuf[0] = WRITE_REG_HEAD + tbl_init_para3[i];	/* Regist Address */
		txbuf[1] = tbl_init_para3[i+1];					/* Regist Values */
		efsa120s_spi_transfer(spi, txbuf, rxbuf, 2);
	}
#ifdef DEBUG_ON
	ELAN_DEBUG(&spi->dev,"AFE Regulator turn on\r\n");
#endif
	mdelay(1);

	/* FUNC Enable */
	for (i=0; i<sizeof(tbl_init_para4); i+=2) {
		txbuf[0] = WRITE_REG_HEAD + tbl_init_para4[i];	/* Regist Address */
		txbuf[1] = tbl_init_para4[i+1];					/* Regist Values */
		efsa120s_spi_transfer(spi, txbuf, rxbuf, 2);
	}
#ifdef DEBUG_ON
	ELAN_DEBUG(&spi->dev,"AFE Regulator turn on\r\n");
#endif
	mdelay(1);

	tbl_init_para5[1] |= (status[0] & INT_NORMAL_HIGH);
	for (i=0; i<sizeof(tbl_init_para5); i+=2) {
		txbuf[0] = WRITE_REG_HEAD + tbl_init_para5[i];	/* Regist Address */
		txbuf[1] = tbl_init_para5[i+1];					/* Regist Values */
		efsa120s_spi_transfer(spi, txbuf, rxbuf, 2);

	}
#ifdef DEBUG_ON
	ELAN_DEBUG(&spi->dev,"INT mode enable\r\n");
#endif
	mdelay(1);
#endif /*endif !FP_TEE*/

	return 0;
}

#ifdef ASUS_FTM
static int efsa120s_read_register(struct spi_device *spi, unsigned char *RegInfo)
{
	/*[0] = length, [1] = register address, [2~length+1] = data*/
	char *txbuf = NULL;
	char *rxbuf = NULL;
	int i;
	int len = RegInfo[0] + 2;
#ifdef ALIG_4_BYTE
	if((len % 4) != 0)
		len = len / 4 * 4 + 4;
#endif

	txbuf = kmalloc(len, GFP_KERNEL);				/* (+2 = cmd & dummy) */
	rxbuf = kmalloc(len, GFP_KERNEL);				/* (+2 = cmd & dummy) */
	if (txbuf == NULL)
		return -ENOMEM;
	else if (rxbuf == NULL)
		return -ENOMEM;
	if (RegInfo[0] < 2 || bCMD_REG == 0) {			/* read with dummy */
		if (bCMD_REG == 1)							/* 0 == CMD, 1 == REG */
			txbuf[0] = READ_REG_HEAD + RegInfo[1];	/* one byte data read (+1 = cmd) */
		else if (bCMD_REG == 0)						/* 0 == CMD, 1 == REG */
			txbuf[0] = RegInfo[1];					/* one byte data read (+1 = cmd) */
		efsa120s_spi_transfer(spi, txbuf, rxbuf, RegInfo[0] + 1);
		if (RegInfo[0] < 2) {						/* read reg */
			RegInfo[2] = rxbuf[1];
#ifdef DEBUG_ON
			ELAN_DEBUG(&spi->dev, "[ELAN] %s() Read = 0x%02x\r\n", __func__, rxbuf[1]);
#endif
		} else {									/* read cmd over one byte */
			for (i = 0; i < RegInfo[0] - 1; i++) {
				RegInfo[i + 2] = rxbuf[i + 2];
#ifdef DEBUG_ON
				ELAN_DEBUG(&spi->dev, "[ELAN] %s() Read CMD = 0x%02x\r\n", __func__, rxbuf[i + 2]);
#endif
			}
		}
	} else {
		txbuf[0] = READ_SERIER_REG_HEAD + RegInfo[1];	/* mutli-byte read (+2 = cmd & dummy) */
		efsa120s_spi_transfer(spi, txbuf, rxbuf, RegInfo[0] + 2);
		for (i = 0; i < RegInfo[0]; i++)
			RegInfo[i + 2] = rxbuf[i + 2];

#ifdef DEBUG_ON
		ELAN_DEBUG(&spi->dev, "[ELAN] %s() Read = ", __func__);

		for (i = 0; i < RegInfo[0]; i++)
			ELAN_DEBUG(&spi->dev, "0x%02x ", rxbuf[i + 2]);

		ELAN_DEBUG(&spi->dev, "\r\n");
#endif
	}
	kfree(rxbuf);
	kfree(txbuf);
	return 0;
}

static int efsa120s_write_register(struct spi_device *spi, unsigned char *RegInfo)
{
	/*[0] = length, [1] = register address, [2~length+1] = data*/
	char *txbuf = NULL;
	int i;
	int len = RegInfo[0] + 1;
#ifdef ALIG_4_BYTE
	if((len % 4) != 0)
		len = len / 4 * 4 + 4;
#endif

	txbuf = kmalloc(len, GFP_KERNEL);
	if (txbuf == NULL)
		return -ENOMEM;

	if (bCMD_REG == 1)	/* 0 == CMD, 1 == REG */
		txbuf[0] = WRITE_REG_HEAD + RegInfo[1];
	else if (bCMD_REG == 0)
		txbuf[0] = RegInfo[1];

	for (i = 0; i < RegInfo[0]; i++)
		txbuf[i + 1] = RegInfo[i + 2];

	efsa120s_spi_transfer(spi, txbuf, NULL, RegInfo[0] + 1);
#ifdef DEBUG_ON
	ELAN_DEBUG(&spi->dev, "[ELAN] %s() ", __func__);

	for (i = 0; i < RegInfo[0]; i++)
		ELAN_DEBUG(&spi->dev, "0x%02x ", txbuf[i + 1]);

	ELAN_DEBUG(&spi->dev, "\n");
#endif
	kfree(txbuf);
	return 0;
}
#endif /*endif !FP_TEE*/

static ssize_t show_drv_version_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VERSION_LOG);
}
static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);

static ssize_t show_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "elfp\n");
}
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

#ifdef ASUS_FTM
static ssize_t efsa120s_test_node(struct device *dev,
		struct device_attribute *attr, char *txtbuf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct efsa120s_data *fp = spi_get_drvdata(spi);
	int err = 0;
	unsigned char buf[8];
	const unsigned char expected[] = {0x00, 0xbf, 0x00, 0x37};

	memset(buf, 0, sizeof(buf));

	buf[0] = 4;		/* read length */
	buf[1] = 0x01;	/* reg addr */
	bCMD_REG = 1;	/* CMD = 0, REG= 1 */
	err = efsa120s_read_register(fp->spi, buf);

	/*if (err >= 0)
		return sprintf(txtbuf, "%x %x %x %x\n",
				buf[2], buf[3], buf[4], buf[5]);
	else
		return sprintf(txtbuf, "Failed\n");*/

	return sprintf(txtbuf, "%d\n",
			((err >= 0) &&
			 !memcmp(buf + 2, expected, sizeof(expected))));
}
static DEVICE_ATTR(atd_test_node, S_IRUGO, efsa120s_test_node, NULL);
#endif /*endif !FP_TEE*/

static struct attribute *efsa120s_attributes[] = {
	&dev_attr_drv_version.attr,
#ifdef ASUS_FTM
	&dev_attr_atd_test_node.attr,
#endif /*endif !FP_TEE*/
	&dev_attr_name.attr,
	NULL
};

static struct attribute_group efsa120s_attr_group = {
	.attrs = efsa120s_attributes,
};

static void efsa120s_reset(struct efsa120s_data *fp)
{
	gpio_direction_output(fp->rst_gpio, 0);
	mdelay(5);
	gpio_direction_output(fp->rst_gpio, 1);
	mdelay(50);
}

static int efsa120s_open(struct inode *inode, struct file *filp)
{
	struct efsa120s_data *fp;

	fp = container_of(inode->i_cdev, struct efsa120s_data, spi_cdev);
	filp->private_data = fp;

#ifdef DEBUG_ON
	ELAN_DEBUG(&fp->spi->dev, "[ELAN] %s()\n", __func__);
#endif

	return 0;
}

static int efsa120s_close(struct inode *inode, struct file *filp)
{
#ifdef DEBUG_ON
		ELAN_DEBUG(&fp->spi->dev, "[ELAN] %s()\n", __func__);
#endif
	return 0;
}

#ifdef ASUS_FTM
static ssize_t efsa120s_read(struct file *filp, char *user_buf, size_t len, loff_t *off)
{
	struct efsa120s_data *fp = filp->private_data;
	int ret = 0;
	int err = 0;
	int j = 0;
	int i = 0;
	int y = 0;

#ifdef DEBUG_ON
		ELAN_DEBUG(&spi->dev, "%s() len=%d\r\n", __func__, (int) len);
#endif
	printk("[E_LAN]%s() len=%d\r\n", __func__, (int) len);
	/* Switch INT mode or Polling mode */
	if (IOIRQ_STATUS & 0X08) {
		/* Buffer INT Enable */
#if 0
		/* Wait for INT IRQ Read complete. */
		nWaitImage_Count = 0;
		while (nWaitImage_Count < nWaitImage) {
			if (fp->bImageReady == 1)
				break;
			nWaitImage_Count++;
			udelay(1); /* 1us * 1000 = 1ms */
		}
		if (nWaitImage_Count >= nWaitImage)
			return -1;

		fp->bImageReady = 0;
		Image_index = 0;
#else
		/* 2016/01/14 KennyKang */
		/* Wait for INT IRQ Read complete. */
		err = wait_event_interruptible_timeout(image_waiter, fp->bImageReady == 1, HZ/2);
		if ( err == 0 )
			return -1;

		fp->bImageReady = 0;
		Image_index = 0;
#endif
	} else {
		if (status[1] & 0x1) {
			/* Image 1 Byte */
			err = efsa120s_receive_image_1BYte(fp->spi);
			if (err == -1)
				return -1;
			j = 1;
		} else {
			/* Image 2 Byte */
			err = efsa120s_receive_image(fp->spi);
			if (err == -1)
				return -1;
			j = 2;
		}
	}

	for (y = 0; y < IMG_HEIGHT; y++) {
#ifdef ALIG_4_BYTE
		i=(IMG_WIDTH * j + 4) * y + 2;
#else
		i=(IMG_WIDTH * j + 2) * y + 2;
#endif
		err = copy_to_user(user_buf+(y*IMG_WIDTH * j), &imagebuffer[i], sizeof(unsigned char)*IMG_WIDTH * j);
		if (err)
			break;
	}
	return (ret==0) ? len : ret;
}
#endif /*endif !FP_TEE*/

/*******************************************************
Function:
Disable irq function
Input:
file: file
Output:
None.
 *********************************************************/
void efsa120s_irq_disable(void *_fp)
{
	struct efsa120s_data *fp = _fp;
	unsigned long irqflags;

#ifdef DEBUG_ON
	ELAN_DEBUG(&fp->spi->dev, "IRQ Disable = %d.\n", fp->isr);
#endif

	spin_lock_irqsave(&fp->irq_lock, irqflags);
	if (!fp->irq_is_disable) {
		fp->irq_is_disable = 1;
		disable_irq_nosync(fp->isr);
	}
	spin_unlock_irqrestore(&fp->irq_lock, irqflags);
}

/*******************************************************
Function:
Enable irq function
Input:
file: file
Output:
None.
 *********************************************************/
void efsa120s_irq_enable(void *_fp)
{
	struct efsa120s_data *fp = _fp;
	unsigned long irqflags = 0;

#ifdef DEBUG_ON
	ELAN_DEBUG(&fp->spi->dev, "IRQ Enable = %d.\n", fp->isr);
#endif

	spin_lock_irqsave(&fp->irq_lock, irqflags);
	if (fp->irq_is_disable) {
		enable_irq(fp->isr);
		fp->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&fp->irq_lock, irqflags);
}

static ssize_t efsa120s_write(struct file *filp, const char __user *user_buf, size_t len, loff_t *off)
{
	struct efsa120s_data *fp = filp->private_data;
	int ret = 0;
	unsigned char buf[130];
	int cmd = user_buf[0];
#ifdef ASUS_FTM
	struct spi_device *spi = fp->spi;
	unsigned char * txbuf = NULL;
	unsigned char * rxbuf = NULL;
	int i = 0;
	int j = 0;
	int ii = 0;
	uint16_t nMax = 0;
	uint16_t nMin = 0xffff;
	uint16_t nData = 0;
	int len_buf = len;
#endif

#ifdef DEBUG_ON
	ELAN_DEBUG(&spi->dev, "%s() len=%d\r\n", __func__, (int) len);
#endif
	/* for test */
	if (cmd == 0) {
		/* show function list */
		printk("*******************************************************\n");
		printk("0: show function list. %s\n", VERSION_LOG);
		printk("1: efsa120s initialize ...\n");
		printk("2: start scan.\n");
		printk("3: show image min & max.\n");
		printk("4: show image data.\n");
		printk("5/0: spi speed now\n");
		printk("5/1/data: spi speed update. data *100k\n");
		printk("6: hardware reset.\n");
		printk("7/0: interupt disable.\n");
		printk("7/1: interupt enable.\n");
		printk("8/data: write data. head, dummy ... you should set by yourself!!!\n");
		printk("9/cmd/len: read data. head, dummy ... you should count to len!!!\n");
		printk("A/data: IOIRQ_Status update in driver\n");
		printk("C/sig[4]/pid[4]:signal to pid.\n");
		printk("D/State: send power_key, State 0=up, 1=down.\n");
		printk("E/TOUCH/State:State 0=up, 1=down.\n");
		printk("*******************************************************\n");
	} else if (cmd == 1) {
		IOIRQ_STATUS = 0X09;
		efsa120s_fingerprint_init(fp->spi, buf); /* buf do not need */
		printk("[ELAN] %s init ...\n", VERSION_LOG);
#ifdef ASUS_FTM
	} else if (cmd == 2) {
		Image_index = 0;
		fp->bImageReady = 0;
		nWaitImage_Count = 0;

#ifdef ALIG_4_BYTE
		len_buf = 4;
#endif
		/* SPI Command (Start_Scan) */
		rxbuf = kzalloc(len_buf, GFP_KERNEL);
		if (rxbuf == NULL)
			printk("memory error.\r\n");
		txbuf = kzalloc(len_buf, GFP_KERNEL);
		if (txbuf == NULL){
			printk("memory error.\r\n");
			kfree(rxbuf);
		} else {
			txbuf[0] = START_SCAN;
			efsa120s_spi_transfer(spi, txbuf, rxbuf, 1);
			printk("[ELAN] %s scan ...\r\n", VERSION_LOG);
			kfree(rxbuf);
			kfree(txbuf);
		}
	} else if(cmd == 3) {
		/* show max min data */
#ifdef DEBUG_ON
		ELAN_DEBUG(&spi->dev, "log image [%dx%d]\n", IMG_WIDTH, IMG_HEIGHT);
#endif
		for (i = 0; i < IMG_HEIGHT; i++) {
			for (j = 0; j < IMG_WIDTH; j++) {
				ii = i * (IMG_WIDTH * 2 + 2) + j * 2 + 2;
				nData = ((imagebuffer[ii] * 256) + imagebuffer[ii+1]);
				if (nMax < nData)
					nMax = nData;
				if (nMin > nData)
					nMin = nData;
			}
		}
		printk("[%d,%d]\r\n", nMin, nMax);
	} else if (cmd == 4) {
		/* show image data */
		printk("Showing image data:\n");
		for (i = 0; i < IMG_HEIGHT; i++) {
			/* pr_debug("base %p ", imagebuffer); */
#ifdef ALIG_4_BYT
			/* pr_debug("range %d - %d\n", i * (IMG_WIDTH * 2 + 4) + 2, IMG_WIDTH * 2); */
#else
			/* pr_debug("range %d - %d\n", i * (IMG_WIDTH * 2 + 2) + 2, IMG_WIDTH * 2); */
#endif
			for (j = 0; j < IMG_WIDTH; j++) {
#ifdef ALIG_4_BYT
				ii = i * (IMG_WIDTH * 2 + 4) + j * 2 + 2;
#else
				ii = i * (IMG_WIDTH * 2 + 2) + j * 2 + 2;
#endif
				nData = ((imagebuffer[ii] * 256) + imagebuffer[ii+1]);
				printk("%3d%3d ", imagebuffer[ii], imagebuffer[ii + 1]);
			}
			printk("\n");
		}
	} else if(cmd == 5) {
		if (len == 4 && user_buf[1] == 1) {
			/* write */
			if (user_buf[1] == 1) /* write */
				spi_speed = (int) (100000 * user_buf[2]);
			printk("SPI speed update to %d.\r\n", spi_speed);
		} else if (len == 3 && user_buf[1] == 0) /* read */
			printk("SPI speed = %d.\r\n", spi_speed);
		else
			printk("SPI speed update length error. len = %d.\r\n", (int) len);
#endif /*endif !FP_TEE*/
	} else if (cmd == 6) {
		/* do reset */
		efsa120s_reset(fp);
		printk("efsa120s_reset\r\n");
	} else if (cmd == 7) {
		/* enable interrupt or disable */
		if (len == 3) {
			if (user_buf[1] == 1) {
				printk("Interrupt enable.\r\n");
				efsa120s_irq_enable(fp);}
			else if (user_buf[1] == 0) {
				printk("Interrupt enable.\r\n");
				efsa120s_irq_disable(fp);
			} else
				printk("Interrupt enable/disable data error.\r\n");
		} else
			printk("Interrupt enable/disable length error.\r\n");
#ifdef ASUS_FTM
	} else if (cmd == 8) {
		/* write spi */
#ifdef ALIG_4_BYTE
		if(((len_buf - 2) % 4) != 0)
			len_buf = (len - 2) / 4 * 4 + 4;
#endif
		rxbuf = kzalloc(len_buf, GFP_KERNEL);
		if (rxbuf == NULL)
			printk("memory error.\r\n");
		txbuf = kzalloc(len_buf, GFP_KERNEL);
		if (txbuf == NULL) {
			printk("memory error.\r\n");
			kfree(rxbuf);
		} else {
			for (i=0; i< len-2; i++)
				txbuf[i] = user_buf[i+1];
			efsa120s_spi_transfer(spi, txbuf, rxbuf, len - 2);
			for (i=0; i< len - 2; i++)
				printk("[%d]%x ", i, txbuf[i]);
			printk("\r\n");
			for (i=0; i< len - 2; i++)
				printk("[%d]%x ", i, rxbuf[i]);
			printk("\r\n");
			kfree(rxbuf);
			kfree(txbuf);
		}
	} else if(cmd == 9) {
		/* read spi */
#ifdef ALIG_4_BYTE
		if ((user_buf[2] % 4) != 0)
			len_buf = user_buf[2] / 4 * 4 + 4;
#endif
		rxbuf = kzalloc(len_buf, GFP_KERNEL);
		if (rxbuf == NULL)
			printk("memory error.\r\n");
		txbuf = kzalloc(len_buf, GFP_KERNEL);
		if (txbuf == NULL) {
			printk("memory error.\r\n");
			kfree(rxbuf);
		} else {
			txbuf[0] = user_buf[1];
			efsa120s_spi_transfer(spi, txbuf, rxbuf, user_buf[2]);
			for (i=0; i< user_buf[2]; i++)
				printk("[%d]%x ", i, txbuf[i]);
			printk("\r\n");
			for (i=0; i< user_buf[2]; i++)
				printk("[%d]%x ", i, rxbuf[i]);
			printk("\r\n");
			kfree(rxbuf);
			kfree(txbuf);
		}
#endif /*endif !FP_TEE*/
	} else if (cmd == 10) {
		/* update ioirq */
		printk("IOIRQ = 0x%x -> 0x%x.\r\n", IOIRQ_STATUS, user_buf[1]);
		IOIRQ_STATUS = user_buf[1];
	} else if (cmd == 12) {
		/* send signal to pid */
		if (len == 10) {
			/* cmd + 4 + 4 + na */
			if (copy_from_user(&sig, &user_buf[1], 4))
				return -1;
			if (copy_from_user(&pid, &user_buf[5], 4))
				return -1;
			bsig_pid = 1;
			efsa120s_send_signal_to_pid();
			printk("send signal sig=%d, pid=%d.\r\n", sig, (int) pid);
		} else
			printk("send signal len error. len = %d. require 10.\r\n", (int) len);
	} else if (cmd == 13) {
		/* report power key event, if you need report any key -> input_set_capability @ probe */
		if (!(user_buf[1] == 0 || user_buf[1] == 1))
			return -1;

		printk("KEY_#0x%x = %x.\r\n", KEY_FP_INT, user_buf[1]);
		input_report_key(fp->input_dev, KEY_FP_INT, user_buf[1]); /* Added for KEY Event */
		input_sync(fp->input_dev);
		mdelay(1);
	} else if (cmd == 14) {
		/* report touch event */
		printk("KEY_#0x%x = %x. It not work now.\r\n", user_buf[1], user_buf[2]);
	}

	return (ret==0) ? len : ret;
}

static long efsa120s_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct efsa120s_data *fp = filp->private_data;
	unsigned char buf[16];
	unsigned char *pUserBuf;
	uint16_t *pkeycode;
	int keycode;
#ifdef ASUS_FTM
	int err = 0;
	int i = 0;
	int j = 0;
	int y = 0;
#endif

#ifdef DEBUG_ON
	ELAN_DEBUG(&fp->spi->dev, "%s() : cmd = [%04X]\r\n", __func__, cmd);
#endif

	switch(cmd)
	{
		case ID_IOCTL_INIT:
			if (copy_from_user(buf, (int *)arg, sizeof(unsigned char)*16))
				return -1;
			efsa120s_fingerprint_init(fp->spi, buf);
			if (copy_to_user((int *)arg, buf, sizeof(unsigned char)*16))
				return -1;
#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_INIT\r\n", __func__);
#endif
			break;

#ifdef ASUS_FTM
		case ID_IOCTL_READ_REGISTER:
			pUserBuf = (unsigned char *)arg;
			bCMD_REG = 1; /* CMD = 0, REG= 1 */
			efsa120s_read_register(fp->spi, pUserBuf);
			if (copy_to_user((unsigned char *)arg, pUserBuf, pUserBuf[0] + 2))
				return -1;

#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_READ_REGISTER\r\n", __func__);
#endif
			break;

		case ID_IOCTL_WRITE_REGISTER:
			pUserBuf = (unsigned char *)arg;
			bCMD_REG = 1; /* CMD = 0, REG= 1 */
			if (copy_from_user(buf, pUserBuf, pUserBuf[0] + 2))
				return -1;

			efsa120s_write_register(fp->spi, buf);
#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_WRITE_REGISTER\r\n", __func__);
#endif
			break;
#endif /*endif !FP_TEE*/
		case ID_IOCTL_RESET:
			efsa120s_reset(fp);
#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_RESET\r\n", __func__);
#endif
			break;

#ifdef ASUS_FTM
		case ID_IOCTL_GET_RAW_IMAGE:
#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE START\r\n", __func__);
#endif
			pUserBuf = (unsigned char *)arg;

			/* Switch INT mode or Polling mode */
			if (IOIRQ_STATUS & 0X08) {
				/* Buffer INT Enable */
#if 0
				/* Wait for INT IRQ Read complete. */
				nWaitImage_Count = 0;
				while (nWaitImage_Count < nWaitImage) {
					if (fp->bImageReady == 1)
						break;
					nWaitImage_Count++;
					udelay(1); // 1us * 1000 = 1ms
				}
				if (nWaitImage_Count >= nWaitImage)
					return -1;

				fp->bImageReady = 0;
				Image_index = 0;
#else
				/* 2016/01/14 KennyKang */
				/* Wait for INT IRQ Read complete. */
				err = wait_event_interruptible_timeout(image_waiter, fp->bImageReady == 1, HZ/2);
				if ( err == 0 ){
					return -1;
				}
				fp->bImageReady = 0;
				Image_index = 0;
#endif
			} else {
				if (status[1] & 0x1) {
					/* added by samuel */
#ifdef DEBUG_ON
					ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE 1 BYTE\r\n", __func__);
#endif
					err = efsa120s_receive_image_1BYte(fp->spi);
					if (err == -1)
						return -1;
				} else {
#ifdef DEBUG_ON
					ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE 2 BYTE\r\n", __func__);
#endif
					err = efsa120s_receive_image(fp->spi);
					if (err == -1)
						return -1;
				}
			}
			if (status[1] & 0x1)
				j = 1;
			else
				j = 2;
			for (y = 0; y < IMG_HEIGHT; y++) {
#ifdef ALIG_4_BYTE
				i=(IMG_WIDTH * j + 4) * y + 2;
#else
				i=(IMG_WIDTH * j + 2) * y + 2;
#endif
				err = copy_to_user(pUserBuf+(y*IMG_WIDTH * j), &imagebuffer[i], sizeof(unsigned char)*IMG_WIDTH * j);
				if (err)
					return -1;
			}
#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE\r\n", __func__);
#endif
			break;

		case ID_IOCTL_STATUS:
			pUserBuf = (unsigned char *)arg;

			if (copy_from_user(&status[1], pUserBuf + 1, sizeof(unsigned char)*4)) /* attention size, status[0] is read only */
				return -1;

#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_STATUS = RO[0]%02x [1]%02x [2]%02x [3]%02x [4]%02x\r\n",
										__func__, status[0], status[1], status[2], status[3] , status[4]);
#endif
			if (status[1] & 0x04) {
				IMG_WIDTH = (unsigned int) (((status[2] & 0xF) << 8) | status[4]);
				IMG_HEIGHT = (unsigned int) (((status[2] & 0xF0) << 4) | status[3]);
#ifdef DEBUG_ON
				ELAN_DEBUG(&fp->spi->dev, "update HW: Y=%d X=%d\r\n", IMG_WIDTH, IMG_HEIGHT);
#endif
				if (IMG_WIDTH_DEFAULT < IMG_WIDTH) {
					ELAN_DEBUG(&fp->spi->dev, "update HW: Y=%d -> %d\r\n", IMG_WIDTH, IMG_WIDTH_DEFAULT);
					IMG_WIDTH = IMG_WIDTH_DEFAULT;
				}
				if (IMG_HEIGHT_DEFAULT < IMG_HEIGHT) {
					ELAN_DEBUG(&fp->spi->dev, "update HW: X=%d -> %d\r\n", IMG_HEIGHT, IMG_HEIGHT_DEFAULT);
					IMG_HEIGHT = IMG_HEIGHT_DEFAULT;
				}
			} else {
				status[2] = (IMG_WIDTH >> 8);
				status[4] = (IMG_WIDTH & 0xff);
				status[2] = ((IMG_HEIGHT >> 4) & 0xf0);
				status[3] = (IMG_HEIGHT & 0xff);
#ifdef DEBUG_ON
				ELAN_DEBUG(&fp->spi->dev, "update HW: Y=%d X=%d\r\n", IMG_WIDTH, IMG_HEIGHT);
#endif
			}

			if (copy_to_user((unsigned char *)arg, status, sizeof(unsigned char)* 5)) /* attention size */
				return -1;

			status[0] &= 0xFE; /* added by samuel, clear interrupt status. */
			break;

		case ID_IOCTL_SET_AUTO_RAW_IMAGE:
			if (copy_from_user(&auto_raw_image_num, (int *)arg, sizeof(int)))
				return -1;
#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_SET_AUTO_RAW_IMAGE = %d\r\n", __func__, auto_raw_image_num);
#endif
			break;

		case ID_IOCTL_GET_AUTO_RAW_IMAGE:
			pUserBuf = (unsigned char *)arg;
			for (auto_raw_image_count = 0; auto_raw_image_count < auto_raw_image_num; auto_raw_image_count++) {
				/* Switch INT mode or Polling mode */
				if (IOIRQ_STATUS & 0X08) {
					/* Buffer INT Enable */
					/* Wait for INT IRQ Read complete. */
					nWaitImage_Count = 0;
					while (nWaitImage_Count < nWaitImage) {
						if (fp->bImageReady == 1)
							break;
						nWaitImage_Count++;
						udelay(1); /* 1us * 1000 = 1ms */
					}
					if (nWaitImage_Count >= nWaitImage)
						return -1;
					fp->bImageReady = 0;
					Image_index = 0;
				} else {
					if (status[1] & 0x1) { /* added by samuel */
#ifdef DEBUG_ON
						ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE 1 BYTE\r\n", __func__);
#endif
						err = efsa120s_receive_image_1BYte(fp->spi);
						if (err == -1)
							return -1;
					} else {
#ifdef DEBUG_ON
						ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE 2 BYTE\r\n", __func__);
#endif
						err = efsa120s_receive_image(fp->spi);
						if (err == -1)
							return -1;
					}
				}
				if (status[1] & 0x1)
					j = 1;
				else
					j = 2;
				for (y = 0; y < IMG_HEIGHT; y++) {
#ifdef ALIG_4_BYTE
					i=(IMG_WIDTH * j + 4) * y + 2;
#else
					i=(IMG_WIDTH * j + 2) * y + 2;
#endif
					err = copy_to_user(pUserBuf+(y*IMG_WIDTH*j)+auto_raw_image_count*(IMG_HEIGHT*IMG_WIDTH*j), &imagebuffer[i], sizeof(unsigned char)*IMG_WIDTH*j);
					if (err)
						return -1;
				}
				/* add interrupt event control */
			}
#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_AUTO_RAW_IMAGE\r\n", __func__);
#endif
			break;

		case ID_IOCTL_READ_CMD:
			pUserBuf = (unsigned char *)arg;
			bCMD_REG = 0; /* CMD = 0, REG= 1 */
			efsa120s_read_register(fp->spi, pUserBuf);
			if (copy_to_user((unsigned char *)arg, pUserBuf, pUserBuf[0] + 2))
				return -1;

#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_READ_CMD\r\n", __func__);
#endif
			break;

		case ID_IOCTL_WRITE_CMD:
			pUserBuf = (unsigned char *)arg;
			bCMD_REG = 0; /* CMD = 0, REG= 1 */
			if (copy_from_user(buf, pUserBuf, pUserBuf[0] + 2))
				return -1;

			if (pUserBuf[1] == START_SCAN){
				fp->bImageReady = 0;
				Image_index = 0;
			}
			efsa120s_write_register(fp->spi, buf);
#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_WRITE_CMD\r\n", __func__);
#endif
			break;
#endif /*endif !FP_TEE*/
		case ID_IOCTL_IOIRQ_STATUS:
			pUserBuf = (unsigned char *)arg;
			if (copy_from_user(&IOIRQ_STATUS, pUserBuf, 1))
				return -1;
			if ((IOIRQ_STATUS & INT_NORMAL_HIGH) != (status[0] & INT_NORMAL_HIGH)) {/* INT Normal status check */
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_IOIRQ_STATUS: INT Normal status setting error. IOIRQ = %x, Driver = %x.\r\n",
				                     __func__, (IOIRQ_STATUS & INT_NORMAL_HIGH), (status[0] & INT_NORMAL_HIGH));
				IOIRQ_STATUS = 0; /* clear */
				return -1;
			}
#ifdef DEBUG_ON
			else
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_IOIRQ_STATUS: IOIRQ = %x.\r\n", __func__, IOIRQ_STATUS);
#endif

			if ((IOIRQ_STATUS & 0xA0) || (IOIRQ_STATUS & 0X08))
				efsa120s_irq_enable(fp);

			break;

#ifdef ASUS_FTM
		case ID_IOCTL_SPI_STATUS:
			pUserBuf = (unsigned char *)arg;
			/* Update spi parameter */
			if (pUserBuf[0] & 0x80) {
				spi_speed = (unsigned int) (pUserBuf[1] | (pUserBuf[2] << 8) | (pUserBuf[3] << 16) | (pUserBuf[4] << 24));
				spi_cs_delay = (unsigned int) (pUserBuf[5] | (pUserBuf[6] << 8));
#ifdef DEBUG_ON
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_SPI_STATUS: Update SPI : Speed=%d CS_Delay=%d.\r\n", __func__, spi_speed, spi_cs_delay);
#endif
			} else {
				if (copy_to_user(&pUserBuf[1], &spi_speed, 4))
					return -1;
				if (copy_to_user(&pUserBuf[5], &spi_cs_delay, 2))
					return -1;
#ifdef DEBUG_ON
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_SPI_STATUS: Read SPI : %x %x %x %x, %x %x\r\n",
							__func__, pUserBuf[1], pUserBuf[2], pUserBuf[3], pUserBuf[4], pUserBuf[5], pUserBuf[6]);
#endif
			}
			break;
#endif /*endif !FP_TEE*/

		case ID_IOCTL_SIG_PID:
			pUserBuf = (unsigned char *)arg;
			/* Update SIG PID */
			if (pUserBuf[0] & 0x80) {
				if(copy_from_user(&sig, &pUserBuf[1], 4))
					return -1;
				if(copy_from_user(&pid, &pUserBuf[5], 4))
					return -1;
				bsig_pid = 1;
#ifdef DEBUG_ON
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_SIG_PID: Update Event: sig=%d, pid=%d.\r\n", __func__, sig, (int) pid);
#endif
			} else {
				/* read sig pid now */
				if (copy_to_user(&pUserBuf[1], &sig, 4))
					return -1;
				if (copy_to_user(&pUserBuf[5], &pid, 4))
					return -1;
#ifdef DEBUG_ON
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_SIG_PID: Read Event: sig=%d, pid=%d.\r\n", __func__, sig, (int) pid);
#endif
			}
			break;

		case ID_IOCTL_POLL_INIT:
			reinit_completion(&cmd_done);
			reinit_completion(&cmd_done_irq);
			break;

		case ID_IOCTL_INPUT_KEYCODE: //add input keycode by herman
			pkeycode = (uint16_t *)arg;
			keycode = asus_translate_keycode(pkeycode[0]);

			if (!keycode) {
				pr_err("Keycode %d not defined, ignored.\n", pkeycode[0]);
				break ;
			}
			input_report_key(fp->input_dev, keycode, 1); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(1);
			input_report_key(fp->input_dev, keycode, 0); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(1);
			ELAN_DEBUG(&fp->spi->dev, "[%s] Send keycode '(%d) completely.\n", __func__, keycode);
			break;

		case ID_IOCTL_NAV_WOE:
			ELAN_DEBUG(&fp->spi->dev, "[%s] navigation trigger woe flag...\n", __func__);
			complete(&cmd_done_irq);
			break;

		default:
#ifdef DEBUG_ON
			ELAN_DEBUG(&fp->spi->dev, "%s() : Unknown cmd\r\n", __func__);
#endif
			break;

	}
	return 0;
}

static unsigned int efsa120s_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct efsa120s_data *fp = filp->private_data;

	wait_for_completion_interruptible(&cmd_done_irq);
	poll_wait(filp, &elan_poll_wq, wait);
	mask |= POLLIN | POLLRDNORM;

	return mask;
}

/*******************************************************
Function:
efsa120s fingerprint work function
Input:
work: work
Output:
None.
 *********************************************************/
static void efsa120s_fp_work_func(struct work_struct *work)
{
	struct efsa120s_data *fp;

#ifdef ASUS_FTM
	int i;
	int j = 0;
	int k = 0;

#ifdef ALIG_4_BYTE
	char txbuf[IMG_WIDTH*2+4];
#else
	char txbuf[IMG_WIDTH*2+2];
#endif
#endif
	fp = container_of(work, struct efsa120s_data, work);

#ifdef DEBUG_ON
	ELAN_DEBUG(&fp->spi->dev,"[ELAN] %s() IOIRQ=%x.\r\n", __func__, IOIRQ_STATUS);
#endif

	status[0] |= 1; /* added by samuel */
	complete(&cmd_done_irq);

	if (IOIRQ_STATUS & 0xA0) { /* WOE Interrupt Enable */
		efsa120s_send_signal_to_pid();
		if (bState == 0) // Display off
		{
		} else {
			input_report_key(fp->input_dev, KEY_FP_INT2, 1); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(1);
			input_report_key(fp->input_dev, KEY_FP_INT2, 0); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(1);
			ELAN_DEBUG(&fp->spi->dev, "%s() RESERVED = 0x%X\n", __func__, KEY_FP_INT2);
		}
		return;
	}

#ifdef ASUS_FTM
	if (IOIRQ_STATUS & 0x08) { /* BUFFER Interrupt Enable */
		txbuf[0] = START_READ_IMAGE;

#ifdef ALIG_4_BYTE /* MTK DMA Mode So Used ALIG 4 Bytes */
		j = 4;
#else
		j = 2;
#endif
		if (status[1] & 0x1)
			k = 1;
		else
			k = 2;
#ifdef DEBUG_ON
		ELAN_DEBUG_DATA(&fp->spi->dev,"H2[%d] ", i);
#endif
		nWaitImage_Count = 0;
		i = Image_index * (IMG_WIDTH * k + j); /* k = 1 BYte or 2 Byte, j = ALIG */
		Image_index++;
#ifdef DEBUG_ON
		ELAN_DEBUG(&fp->spi->dev,"[ELAN] Image_Index=%d H=%d\n", Image_index, IMG_HEIGHT);
#endif

		if(Image_index <= IMG_HEIGHT) /* if over, memory will crash */
			efsa120s_spi_transfer(fp->spi, txbuf, &(imagebuffer[i]), IMG_WIDTH * k + 2); /* +2 is cmd & dummy */

		if (Image_index == IMG_HEIGHT) {
			fp->bImageReady = 1;
			Image_index = 0;
			/* 2016/01/14 KennyKang */
			wake_up_interruptible(&image_waiter);
		}
	}
#endif

	efsa120s_irq_enable(fp);
}

static irqreturn_t efsa120s_irq_handler(int irq, void *_fp)
{
	struct efsa120s_data *fp = _fp;

#ifdef DEBUG_ON
	ELAN_DEBUG(&fp->spi->dev, "%s()\n", __func__);
#endif
	efsa120s_irq_disable(fp);
	queue_work(efsa120s_wq, &fp->work);

	return IRQ_HANDLED;
}

static const struct file_operations efsa120s_fops = {
	.owner			= THIS_MODULE,
	.open			= efsa120s_open,
#ifdef ASUS_FTM
	.read			= efsa120s_read,
#else
	.read			= NULL,
#endif
	.write			= efsa120s_write,
	.unlocked_ioctl	= efsa120s_ioctl,
	.release		= efsa120s_close,
	.poll			= efsa120s_poll,
};

static int efsa120s_setup_cdev(struct efsa120s_data *finger)
{
#ifdef ASUS_FTM
	struct efsa120s_data *fp = spi_get_drvdata(finger->spi);
#else
	struct efsa120s_data *fp = platform_get_drvdata(finger->spi);
#endif
	dev_t devNum;
	int err = 0;

#ifdef DEBUG_ON
	ELAN_DEBUG(&fp->spi->dev, "%s()\n", __func__);
#endif

	err = alloc_chrdev_region(&devNum, 0, 1, "fingerprint");
	if (err < 0) {
		printk("Alloc char dev region fail.\r\n");
		goto alloc_chrdev_region_fail;
	}

	major_number = MAJOR(devNum);
	minor_number = MINOR(devNum);

	/* Create class under /sysfs */
	finger->spi_class = class_create(THIS_MODULE, "fingerprint_class");
	if (IS_ERR(finger->spi_class)) {
		err = -1;
		printk("class create failed\n");
		goto class_create_fail;
	}
	/* Create device under /dev */
#ifdef ASUS_FTM
	finger->spi_device = device_create(finger->spi_class, NULL, devNum, "%s", "fingerprint");
	if (IS_ERR(finger->spi_device)) {
#else
	finger->platform_device = device_create(finger->spi_class, NULL, devNum, "%s", "fingerprint");
	if (IS_ERR(finger->platform_device)) {
#endif
		err = -1;
		printk("device create failed\n");
		goto device_create_fail;
	}

	/* Init Char Dev */
	cdev_init(&finger->spi_cdev, &efsa120s_fops);
	finger->spi_cdev.owner = THIS_MODULE;
	finger->spi_cdev.ops = &efsa120s_fops;

	/* Region Chae dev under /proc/dev */
	err = cdev_add(&finger->spi_cdev, devNum, 1);
	if (err < 0) {
		printk("add chr dev failed\n");
		goto cdev_add_fail;
	}

	return 0;

cdev_add_fail:
	device_destroy(finger->spi_class, devNum);
device_create_fail:
	class_destroy(finger->spi_class);
class_create_fail:
	unregister_chrdev_region(devNum,1);
alloc_chrdev_region_fail:

	return err;
}

static int efsa120s_sysfs_create(struct efsa120s_data *sysfs)
{
#ifdef ASUS_FTM
	struct efsa120s_data *fp = spi_get_drvdata(sysfs->spi);
#else
	struct efsa120s_data *fp = platform_get_drvdata(sysfs->spi);
#endif
	int error = 0;

	mutex_init(&fp->sysfs_mutex);

	/* Register sysfs */
	error = sysfs_create_group(&fp->spi->dev.kobj, &efsa120s_attr_group);
	if (error) {
		dev_err(&fp->spi->dev, "[ELAN] Failed to create sysfs attributes, err: %d\n", error);
		goto fail_un;
	}
	return error;
fail_un:
	/* Remove sysfs */
	sysfs_remove_group(&fp->spi->dev.kobj, &efsa120s_attr_group);

	/* Release Mutex */
	if(&fp->sysfs_mutex)
		mutex_destroy(&fp->sysfs_mutex);

	return error;
}

/* asus parse dt data*/
static int elan_parse_dt(struct device *dev, struct efsa120s_data *pdata)
{

	struct device_node *np = dev->of_node;

	/* +++reset, irq gpio info+++ */
	pdata->rst_gpio = of_get_named_gpio_flags(np, "sleep-gpio",
			0, NULL);
	if (pdata->rst_gpio < 0)
		return pdata->rst_gpio;

	pdata->intr_gpio = of_get_named_gpio_flags(np, "irq-gpio",
			0, NULL);
	if (pdata->intr_gpio < 0)
		return pdata->intr_gpio;
	/* ---reset, irq gpio info--- */

	pdata->cs_pin = of_get_named_gpio_flags(np, "cs-gpio",
			0, NULL);
	if (pdata->cs_pin < 0)
		return pdata->cs_pin;

	pdata->osvcc_pin = of_get_named_gpio_flags(np, "osvcc-gpio",
			0, NULL);
	if (pdata->osvcc_pin < 0)
		return pdata->osvcc_pin;

	printk("[E_LAN] sleep_pin = %d, drdy_pin = %d, cs_pin = %d  osvcc_pin = %d  \n", pdata->rst_gpio, pdata->intr_gpio, pdata->cs_pin, pdata->osvcc_pin);

	return 0;
}
/* elan spidri pars dt data*/

static char efsa120s_gpio_config(void *_fp)
{
	struct efsa120s_data *fp = _fp;
	int ret;

	/* Configure INT GPIO (Input) */
	ret = gpio_request(fp->intr_gpio, "efsa120-irq");
	if (ret < 0) {
		printk("[ELAN] %s() IRQ%d request fail, err=0x%x.\n", __func__, fp->intr_gpio, ret);
		ret = -ENODEV;
	} else {
		gpio_direction_input(fp->intr_gpio);
		fp->isr = gpio_to_irq(fp->intr_gpio);
		printk("[ELAN] %s() IRQ%d=%d request success, err=0x%x.\n", __func__, fp->intr_gpio, fp->isr, ret);
	}

	/* Configure RST GPIO (Output) */
	ret =  gpio_request(fp->rst_gpio, "efsa120-reset");
	if (ret < 0) {
		gpio_free(fp->intr_gpio);
		free_irq(fp->isr, fp);
		printk("[ELAN] %s() RST%d request fail, err=0x%x.\n", __func__, fp->intr_gpio, ret);
		ret = -ENODEV;
	} else {
		printk("[ELAN] %s() RST%d request success, err=0x%x.\n", __func__, fp->rst_gpio, ret);
		gpio_direction_output(fp->rst_gpio, 0);
		mdelay(20);
		gpio_direction_output(fp->rst_gpio, 1);
		mdelay(20);
		printk("[ELAN] %s() Reset ...\n", __func__);
	}

	return ret;
}

/* elan spidri power init*/
static int elan_power_on(struct efsa120s_data *pdata, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(pdata->sovcc);
	if (rc) {
		printk("[%s]Regulator sovcc enable failed rc=%d\n", __func__, rc);
		return rc;
	}

	/*rc = regulator_enable(pdata->vcc);
	if (rc) {
		printk("[%s]Regulator vcc enable failed rc=%d\n", __func__, rc);
		regulator_disable(pdata->sovcc);
	}*/

	return rc;

power_off:
	rc = regulator_disable(pdata->sovcc);
	if (rc) {
		printk("[%s]Regulator sovcc disable failed rc=%d\n", __func__, rc);
		return rc;
	}

	/*rc = regulator_disable(pdata->vcc);
	if (rc) {
		printk("[%s]Regulator vcc disable failed rc=%d\n", __func__, rc);
		rc = regulator_enable(pdata->sovcc);
		if (rc) {
			printk("[%s]Regulator sovcc disable failed rc=%d\n", __func__, rc);
		}
	}*/

	return rc;
}

static int elan_power_init(struct device *dev,
		struct efsa120s_data *pdata, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;
	/* +++ pars regulator+++ */
	printk("[ELAN] elan_power_init+\n");
	pdata->sovcc = regulator_get(dev, "vfp3v3");
	if (IS_ERR( pdata->sovcc)) {
		rc = PTR_ERR(pdata->sovcc);
		printk("Regulator get elan sovcc  failed rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->sovcc) > 0) {
		printk("[ELAN] efsa120s: set regulator voltage\n");
		rc = regulator_set_voltage(pdata->sovcc, 3300000,
				3300000);
		if (rc) {
			printk("Regulator set sovcc failed vdd rc=%d\n", rc);
			goto reg_sovcc_put;
		}
	}

	/*pdata->vcc = regulator_get(dev, "vcc");
	if (IS_ERR( pdata->vcc)) {
		rc = PTR_ERR(pdata->vcc);
		printk("Regulator get vcc failed rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(pdata->vcc) > 0) {
		rc = regulator_set_voltage(pdata->vcc, 1800000,
				1800000);
		if (rc) {
			printk("Regulator set_vcc failed vdd rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}*/
	/* +++ pars regulator+++ */
	return 0;
/*reg_vcc_i2c_put:
	regulator_put(pdata->vcc);
reg_vdd_set_vtg:
	if (regulator_count_voltages(pdata->sovcc) > 0)
		regulator_set_voltage(pdata->sovcc, 0, 3300000);*/
reg_sovcc_put:
	regulator_put(pdata->sovcc);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(pdata->sovcc) > 0)
		regulator_set_voltage(pdata->sovcc, 0, 3300000);

	regulator_put(pdata->sovcc);

	/*if (regulator_count_voltages(pdata->vcc) > 0)
		regulator_set_voltage(pdata->vcc, 0, 1800000);

	regulator_put(pdata->vcc);*/
	return 0;
}
/* elan spidri power init*/

#ifdef ASUS_FTM
static int efsa120s_probe(struct spi_device *spi)
#else
static int efsa120s_probe(struct platform_device *spi)
#endif
{
	struct efsa120s_data *fp;
	struct input_dev *input_dev = NULL;
	int err = 0;
	int i;
#ifdef ASUS_FTM
	unsigned char buf[6] = {0};
#endif

#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		pr_info("[ELAN] Probe aborted on COS mode.\n");
		return 0;
	}
#endif

	printk("%s() %s\n", __func__, VERSION_LOG);
	if (of_get_pcbid_data() != ELAN_FP) {
		pr_info("probe failed\n");
		return -ENODEV;
	}

	init_completion(&cmd_done);
	init_completion(&cmd_done_irq);
#ifdef ASUS_FTM
	/* Setup SPI */
	spi->mode = SPI_MODE_0;				/* set at spi_board_info */
	spi->max_speed_hz = SPI_MAX_SPEED;	/* set at spi_board_info */
	spi->chip_select = 0;				/* set at spi_board_info */
	spi->bits_per_word = 8;				/* do not change */

	err = spi_setup(spi);
	if (err < 0) {
		printk("[ELAN] spi_setup fail (0x%x).\r\n", err);
		return err;
	}
#endif

	/* Allocate Device Data */
#ifdef ASUS_FTM
	fp = kzalloc(sizeof(struct efsa120s_data), GFP_KERNEL);
#else
	fp = devm_kzalloc(&spi->dev, sizeof(struct efsa120s_data), GFP_KERNEL);
#endif
	if (!fp) {
		printk("[ELAN] alloc efsa120s data fail.\r\n");
		goto alloc_mem_fail;
	}

	err = elan_parse_dt(&spi->dev, fp);

	/* Init Mutex */
	mutex_init(&fp->spi_mutex);
	init_waitqueue_head(&fp->efsa_wait);

	/* Init Input Device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		printk("[ELAN] alloc input_dev fail.\r\n");
		goto input_allocate_device_fail;
	}

	fp->spi = spi;
#ifdef ASUS_FTM
	spi_set_drvdata(spi, fp);
#else
	platform_set_drvdata(spi, fp);
#endif

	/* Init Sysfs */
	err = efsa120s_sysfs_create(fp);
	if (err < 0) {
		printk("[ELAN] efsa120s sysfs fail.\r\n");
		goto sysfs_create_fail;
	}

#if DISABLE_FP_FUNC
	pr_info("[ELAN] Create sysfs but not enable elan function.\n");
	return 0;
#endif

	if (elan_power_init(&spi->dev, fp, true) < 0)
		printk("[ELAN] opps elan_power_init fail ! \n");

	if (elan_power_on(fp, true) < 0)
		printk("[ELAN] opps elan_power_on fail ! \n");
	msleep(1);

	if (gpio_request(fp->osvcc_pin, "elan_osvcc")) {
		err = -EBUSY;
		printk("[ELAN] opps osvcc fail ! \n");
	}
	err = gpio_direction_output(fp->osvcc_pin, 1);
	if (err < 0) {
		pr_err("gpio_direction_output osvcc_pin failed\n");
		err = -EBUSY;
	}
	msleep(1);

	input_dev->name = "efsa120s";
	input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = &spi->dev;
	input_set_drvdata(input_dev, fp);

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
	for (i = 0; i < ARRAY_SIZE(FP_KEYCODES); ++i) {
		int report_keycode = FP_KEYCODES[i].keycode;

		input_set_capability(input_dev, EV_KEY, report_keycode);
		__set_bit(report_keycode, input_dev->keybit);
	}

	fp->input_dev = input_dev;

	/* Init Char Device */
	err = efsa120s_setup_cdev(fp);
	if (err < 0) {
		printk("[ELAN] efsa120s setup device fail.\r\n");
		goto cdev_add_fail;
	}

	/* Register Input Device */
	err = input_register_device(input_dev);
	if (err) {
		printk("[ELAN] Unable to register input device, error: %d!\r\n", err);
		goto input_dev_creat_fail;
	}

#ifdef ASUS_FTM
	/* Can use SPI Transfer now*/
	buf[0] = 4;		/* read length */
	buf[1] = 0x01;	/* reg addr */
	bCMD_REG = 1;	/* CMD = 0, REG= 1 */
	err = efsa120s_read_register(fp->spi, buf);
	if (err < 0) {
		printk("[ELAN] efsa120s read device length & width fail.\r\n");
		goto read_WH_fail;
	} else {
		printk("[ELAN] efsa120s read device %02x %02x %02x %02x.\r\n", buf[2], buf[3], buf[4], buf[5]);
		IMG_WIDTH = (unsigned int)(buf[5] - buf[4] + 1);
		IMG_HEIGHT = (unsigned int)(buf[3] - buf[2] + 1);
		IMG_WIDTH_DEFAULT = IMG_WIDTH;
		IMG_HEIGHT_DEFAULT = IMG_HEIGHT;
		printk("[ELAN] efsa120s WIDTH(Y)=%d, HEIGHT(X)=%d.\r\n", IMG_WIDTH, IMG_HEIGHT);
	}

	/* Allocate image buffer */
	err = efsa120s_kmalloc_image();
	if (err) {
		printk("[ELAN] Unable to kmalloc image buffer, error (%d).\r\n", err);
		goto kmalloc_image_fail;
	}
#endif

	/* Init EFSA120S GPIO */
	err = efsa120s_gpio_config(fp);
	if (err < 0) {
		printk("[ELAN] GPIO request fail (%d).\r\n", err);
		goto gpio_config_fail;
	}

	efsa120s_wq = create_singlethread_workqueue("efsa120s_wq");
	if (!efsa120s_wq) {
		printk("[ELAN] Work Create error! \r\n");
		goto  request_irq_fail;
	}

	INIT_WORK(&fp->work, efsa120s_fp_work_func);
	spin_lock_init(&fp->irq_lock);	/* Added for ISR 2.6.39 later */
#ifdef ASUS_FTM
	spi->irq = fp->intr_gpio;
#endif

	/* Init IRQ FUNC */
	err = request_irq(fp->isr, efsa120s_irq_handler, IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING, spi->dev.driver->name, fp);
	if (err) {
		printk("[ELAN] Failed to request IRQ %d.\r\n", err);
		goto  request_irq_fail;
	}

	irq_set_irq_wake(fp->isr, 1);
	return 0;

request_irq_fail:

gpio_config_fail:
	/* memory have been kfree in efsa120s_kmalloc_image function. */

kmalloc_image_fail:
	/* memory have been kfree in efsa120s_kmalloc_image function. */

read_WH_fail:

input_dev_creat_fail:
#ifdef ASUS_FTM
	spi_set_drvdata(spi, NULL);
#else
	platform_set_drvdata(spi, NULL);
#endif
	input_free_device(input_dev);
	input_dev = NULL;

cdev_add_fail:
	cdev_del(&fp->spi_cdev);
	/*unregister_chrdev_region(MKDEV(major_number, minor_number), 1); // marked by samuel, had been done before. */

sysfs_create_fail:

input_allocate_device_fail:

alloc_mem_fail:
	kfree(fp);
	return -ENOMEM;
}

#ifdef ASUS_FTM
static int efsa120s_remove(struct spi_device *spi)
{
	struct efsa120s_data *fp = spi_get_drvdata(spi);
#else
static int efsa120s_remove(struct platform_device *spi)
{
	struct efsa120s_data *fp = platform_get_drvdata(spi);
#endif

	if (fp->intr_gpio)
		free_irq(fp->intr_gpio, fp);

	gpio_free(fp->intr_gpio);
	gpio_free(fp->rst_gpio);

	cdev_del(&fp->spi_cdev);
	device_destroy(fp->spi_class, MKDEV(major_number, minor_number));	/* delete device node under /dev */
	class_destroy(fp->spi_class);
	unregister_chrdev_region(MKDEV(major_number, minor_number), 1);		/* delecte class create by bus */
	input_free_device(fp->input_dev);

	if (&fp->spi_mutex)
		mutex_destroy(&fp->spi_mutex);

	kfree(fp);
#ifdef ASUS_FTM
	spi_set_drvdata(spi, NULL);
#else
	platform_set_drvdata(spi, NULL);
#endif

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int efsa120s_suspend(struct device *dev)
{
	printk("[ELAN] efsa120s suspend!\n");
	return 0;
}

static int efsa120s_resume(struct device *dev)
{
	printk("[ELAN] efsa120s resume!\n");
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(efsa120s_pm_ops, efsa120s_suspend, efsa120s_resume);
#ifdef CONFIG_OF
static struct of_device_id efsa120s_metallica_table[] = {
#ifdef ASUS_FTM
	{ .compatible = "elan,efsa120s_ree",},
#else
	{ .compatible = "elan,efsa120s_tee",},
#endif
	{ },
};
#else
#define efsa120s_metallica_table NULL
#endif

#ifdef ASUS_FTM
static struct spi_driver efsa120s_driver = {
#else
static struct platform_driver efsa120s_driver = {
#endif
	.driver = {
		.name	= "efsa120s",
		.owner	= THIS_MODULE,
		.of_match_table = efsa120s_metallica_table,
	},
	.probe	= efsa120s_probe,
	.remove	= efsa120s_remove,
};

static int __init efsa120s_init(void)
{
	int status = 0;

	printk("[FP][E_LAN] %s:", __func__);
#ifdef ASUS_FTM
	printk("spi_register_driver\n", __func__);
	status = spi_register_driver(&efsa120s_driver);
#else
	printk("platform_driver_register\n", __func__);
	status = platform_driver_register(&efsa120s_driver);
#endif
	if (status < 0)
		printk("[FP][E_LAN] %s FAIL !\n", __func__);

	return status;
}

static void __exit efsa120s_exist(void)
{
#ifdef ASUS_FTM
	spi_unregister_driver(&efsa120s_driver);
#else
	platform_driver_unregister(&efsa120s_driver);
#endif
	if (efsa120s_wq)
		destroy_workqueue(efsa120s_wq);
}

module_init(efsa120s_init);
module_exit(efsa120s_exist);

MODULE_AUTHOR("JeffLee <jeff.lee@emc.com.tw>");
MODULE_DESCRIPTION("ELAN SPI FingerPrint eFSA120S driver");
MODULE_VERSION(VERSION_LOG);
MODULE_LICENSE("GPL");
