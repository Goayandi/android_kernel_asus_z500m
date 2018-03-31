#define pr_fmt(fmt) "SMB1351 %s: " fmt, __func__
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/reboot.h>
#include <linux/switch.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include "mt_charging.h"
#include "mt_battery_common.h"
#include "mt_battery_meter.h"
#include <mt-plat/upmu_common.h>
#include <mt-plat/mt_reboot.h>
#include <mt-plat/mt_boot.h>

/**********************************************************
 *
 *    [Define]
 *
 **********************************************************/

#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define I2C_TRANSFER_RETRY 2

#define CONFIG_REG			0x0
#define OTHER_CURRENT_REG	0x1
#define VARIOUS_FUNC		0x2
#define FLOAT_VOLT_REG		0x3
#define CHARGE_CTRL_REG	0x4
#define STAT_CTRL_REG		0x5
#define ENABLE_CTRL_REG	0x6
#define THERM_CTRL_A_REG	0x7
#define WDT_TIMER_CTRL_REG 0x8
#define OTG_TLIM_CTRL_REG	0xA
#define TEMP_MONITOR_REG	0xB
#define STATUS_INT_REG		0xD
#define COMP_REG			0xE
#define FLEXCHARGER_REG	0x10
#define VARIOUS_FUNC2		0x11
#define HVDCP_BAT_MISSING_CTRL_REG		0x12
#define OTG_POWER_REG		0x14
#define CMD_REG_I2C			0x30
#define CMD_REG_IL			0x31
#define CMD_REG_HVDCP		0x34
#define STATUS_REG			0x36
#define STATUS_REG_4		0x3A
#define STATUS_REG_5		0x3B
#define STATUS_REG_7		0x3D
#define IRQ_C_REG			0x42
#define IRQ_G_REG			0x47

#define SWITCHING_FREQ_MASK	0xC0
#define AUTO_AICL_LIMIT_MASK	0x10
#define BQ_CONFIG_ACCESS_MASK	0x40
#define FASTCHARGE_COMP_MASK	0x20
#define CURRENT_TERMINATION_MASK 0x40
#define INPUT_CURRENT_MODE_MASK	0x8
#define RECHARGE_STATE_MASK		0x10
#define SOFTCOLD_LIMIT_MASK	0xC
#define SOFTHOT_LIMIT_MASK		0x3
#define USB_AC_MODE_MASK		0x1
#define USB_CTRL_MODE_MASK	0x3
#define ADAPTER_ID_MODE_MASK	0x3
#define CHARGER_CONFIG_MASK	0x70
#define ADAPTER_CONFIG_MASK	0x80
#define STAT_OUTPUT_CTRL_MASK	0x20
#define HOT_ALARM_MASK	0x30
#define ENABLE_PIN_MASK	0x60
#define CHARGE_TYPE_MASK	0x10
#define CHARGE_DONE_MASK	0x20
#define FLOAT_VOLT_MASK	0x3F
#define THERM_MON_MASK	0x10
#define FASTCHARGE_MASK	0xF0
#define PRECHARGE_MASK		0xE0
#define TERMINATION_MASK	0x1C
#define AC_IN_LIMIT_MASK	0xF
#define AICL_DONE_MASK		0x80
#define DCIN_LIMIT_MASK	0xF
#define SUSPEND_MODE_MASK	0x40
#define OTG_DCIN_CURRENT_MASK		0xC
#define CHARGING_STATE_MASK	0x6
#define WATCHDOG_TIMER_MASK	0x60
#define WATCHDOG_EN_MASK		0x1

#define RERUN_APSD_MASK		0x80
#define FORCE_QC2_MODE_MASK	0x20
#define HVDCP_STATUS_MASK	0x0E
#define HVDCP_BAT_MISSING_CTRL_MASK	0xC0
#define CMD_REG_IL_MASK	    0x09
#define VARIOUS_FUNC_MASK   0x04
#define VARIOUS_FUNC_MASK2  0x02
#define TERMINATION_STATE_MASK	0x1

#define pr_err_asus(fmt, ...) \
({                      \
 static bool __print_once;       \
 \
 if (!__print_once) {            \
 __print_once = true;        \
 pr_debug(fmt, ##__VA_ARGS__);   \
 printk(KERN_ERR "[name:"KBUILD_MODNAME"&]"pr_fmt(fmt), ##__VA_ARGS__);\
 ASUSEvtlog("[BAT]"pr_fmt(fmt), ##__VA_ARGS__);\
 }   else                \
 printk(KERN_ERR "[name:"KBUILD_MODNAME"&]"pr_fmt(fmt), ##__VA_ARGS__);\
 ASUSEvtlog("[BAT]"pr_fmt(fmt), ##__VA_ARGS__);\
 })

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
extern int g_ftm_mode;

/* this is for smb1351_user_space_driver to keep reg address */
static u8 g_reg_value_smb1351;

static struct workqueue_struct *smb1351_wq;
static struct workqueue_struct *smb1351_wq2;
static struct workqueue_struct *smb1351_wq3;
static struct workqueue_struct *smb1351_wq4;
static struct workqueue_struct *smb1351_wq5;

static bool charger_status = false;
static bool charger_limit_enable = true;
static long charger_limit = 70;
static long demo_app_charger_limit = 50;
static int charger_type = 0; //CHARGER_UNKNOWN = 0
int asus_show_qc_flag;
static bool demo_app_status_flag = 0;


/* fast_chg_current[n] / 100 = mA */
static int fast_chg_current[] = {
	100000, 120000, 140000, 160000, 180000, 200000, 220000, 240000, 260000, 280000, 300000, 340000, 360000,
		380000, 400000, 464000
};

/* input_current[n] / 100 = mA */
static int input_current[] = {
	50000, 68500, 100000, 110000, 120000, 130000, 150000, 160000, 170000, 180000, 200000, 220000, 250000, 300000
};

/* this is to indicate last temperature state from 1.5 Celsius to 60 Celsius*/
enum temp_state {
	unknown_temp_state = 0,
	less_than_0,
	from_0_to_150,
	from_150_to_450,
	from_450_to_600,
	greater_than_600,
};

enum adc_type {
    DCP_1A = 0,
    DCP_ASUS_2A,
    HVDCP_ASUS_2A,
    HVDCP_1A,
};

enum qc_type {
    DCP,
	HVDCP_QC3P0,
	HVDCP_QC2P0,
};

enum chg_type {
    CT_UNDEFINED = 0,
    DCP_PB_2A,
    DCP_ASUS_750K_2A,
    DCP_OTHERS_1A,
    HVDCP_OTHERS_PB_1A,
    HVDCP_ASUS_200K_2A,
    HVDCP_ASUS_750K_2A,
    HVDCP_OTHERS_1A,
    CDP_1P5A,
    SDP_0P5A,
    OTHERS_1A,
    TYPEC_1P5A,
    TYPEC_3P0A,
    PD,
    FLOATING_0P5A,
};

extern int hw_charger_type_detection(void);
extern int mtkts_charger_get_temp(unsigned long *temp);
extern int asustek_get_battery_id(unsigned int *t);
static int of_get_smb1351_platform_data(struct device *dev);
extern u8 us5587_dump_value(void);
extern void mt_battery_update_status(void);

struct smb1351_charger {
	struct i2c_client	*client;
	struct device		*dev;
	struct mutex		read_write_lock;
	struct mutex		charging_setting_lock;
	struct mutex		notHVDCP_lock;
//	struct mutex		jeita_setting_lock;
	struct wake_lock	jeita_setting_wake_lock;
	struct delayed_work	charging_state_changed_work;
	struct delayed_work	notHVDCP_ASUS_adapter_detect_work;
	struct delayed_work	DCP_1min_wait_work;
	struct delayed_work	DCP_5sec_wait_work;
	struct delayed_work	DCP_aftPLUS_5sec_wait_work;
	int	charging_state_irq;
	int	last_charger_type;
	int	last_temp_state;
	int	otg_en_gpio;
	int	usbsw_s_gpio;
	int	adp_vh_en_gpio;
    enum adc_type ADC_Type;
    enum chg_type CHG_TYPE;
};
struct smb1351_charger *chip;
/**********************************************************
  *
  *   [I2C Function For Read/Write smb1351]
  *
  *********************************************************/
static int __smb1351_read_reg(u8 reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		pr_err_asus("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int __smb1351_write_reg(int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		pr_err_asus("i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb1351_read_reg(int reg, u8 *val)
{
	int rc;
	int i;
	for (i = 0; i < I2C_TRANSFER_RETRY; i++) {
		mutex_lock(&chip->read_write_lock);
		rc = __smb1351_read_reg(reg, val);
		mutex_unlock(&chip->read_write_lock);
		if (rc) {
			pr_err_asus("Reading %02x failed.....retry : %d\n", reg, i);
			msleep(100);
		} else
			break;
	}
	return rc;
}

static int smb1351_masked_write(int reg, u8 mask, u8 val)
{
	s32 rc;
	u8 temp;
	int i;

	rc = smb1351_read_reg(reg, &temp);
	if (rc) {
		pr_err_asus("Read Failed: reg=%02x, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	for (i = 0; i < I2C_TRANSFER_RETRY; i++) {
		mutex_lock(&chip->read_write_lock);
		rc = __smb1351_write_reg(reg, temp);
		mutex_unlock(&chip->read_write_lock);
		if (rc) {
			pr_err_asus("Writing %02x failed.....retry : %d\n", reg, i);
			msleep(100);
		} else
			break;
	}
	if (rc) {
		pr_err_asus("Write Failed: reg=%02x, rc=%d\n", reg, rc);
	}
out:
	return rc;
}

static int get_closest_input_current(int target_current)
{
	int i;
	for (i = ARRAY_SIZE(input_current) - 1; i >= 0; i--) {
		if (input_current[i] <= target_current)
			break;
	}

	if (i < 0) {
		pr_err_asus("Invalid input current setting %dmA\n",
						target_current/100);
		i = 0;
	}
	return i;
}

static int get_closest_fast_chg_current(int target_current)
{
	int i;
	for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--) {
		if (fast_chg_current[i] <= target_current)
			break;
	}

	if (i < 0) {
		pr_err_asus("Invalid fast_chg current setting %d mA\n",
						target_current/100);
		i = 0;
	}
	return i;
}

static void smb1351_enable_volatile_access(void)
{
	int rc;
	// BQ configuration volatile access, 30h[6] = 1
	rc = smb1351_masked_write(CMD_REG_I2C, BQ_CONFIG_ACCESS_MASK, 0x40);
	if (rc)
		pr_err_asus("failed to enable configuration volatile access\n");
}

static void smb1351_enable_AICL(bool enable)
{
	int rc;
	smb1351_enable_volatile_access();
	if (enable) {
		// 02h[4] = "1"
		rc = smb1351_masked_write(VARIOUS_FUNC, AUTO_AICL_LIMIT_MASK, 0x10);
		if (rc)
			pr_err_asus("failed to enable AICL\n");
	} else {
		// 02h[4] = "0"
		rc = smb1351_masked_write(VARIOUS_FUNC, AUTO_AICL_LIMIT_MASK, 0x0);
		if (rc)
			pr_err_asus("failed to disable AICL\n");
	}
}

static void smb1351_enable_charging(bool enable)
{
	int rc;
	if (charger_limit_enable && g_ftm_mode) {
		if (BMT_status.SOC >= charger_limit && enable)
			return;
	}
	if (demo_app_status_flag && (BMT_status.SOC >= demo_app_charger_limit) && enable)
		return;

	smb1351_enable_volatile_access();
	pr_notice("%s charging\n", (enable) ? "Enabling" : "Disabling");
	if (enable) {
		// Charging Enable, 06h[6:5] = "11"
		rc = smb1351_masked_write(ENABLE_CTRL_REG, ENABLE_PIN_MASK, 0x60);
		if (rc)
			pr_err_asus("failed to enable charging\n");
	} else {
		// Charging Disable, 06h[6:5] = "10"
		rc = smb1351_masked_write(ENABLE_CTRL_REG, ENABLE_PIN_MASK, 0x40);
		if (rc)
			pr_err_asus("failed to disable charging\n");
	}
}

static void smb1351_set_input_current(int current_target)
{
	int i, rc;

	pr_notice("Setting input current limit: %d mA\n", current_target);
	smb1351_enable_volatile_access();
	i = get_closest_input_current(current_target * 100);
	smb1351_enable_AICL(false);
	rc = smb1351_masked_write(CONFIG_REG, AC_IN_LIMIT_MASK, i);
	if (rc)
		pr_err_asus("failed to set input current max limit\n");
	pr_notice("Setting input current limit: %d mA\n", input_current[i]/100);
	smb1351_enable_AICL(true);
}

static void smb1351_AC_1A_setting(void)
{
	int rc;
	pr_notice("setting 1A charging\n");
	// BQ configuration volatile access, 30h[6] = 1
	smb1351_enable_volatile_access();
	// Set Input current = command register, 31h[3] = "1"
	rc = smb1351_masked_write(CMD_REG_IL, INPUT_CURRENT_MODE_MASK, 0x8);
	if (rc)
		pr_err_asus("failed to set input current = command reg.\n");
	// Set USB AC control = USB AC, 31h[0]  ="1"
	rc = smb1351_masked_write(CMD_REG_IL, USB_AC_MODE_MASK, 0x1);
	if (rc)
		pr_err_asus("failed to set USB AC control = USB AC.\n");
	// Disable AICL, 02h[4] = "0"
	// Set IUSB_IN = 1000 mA, 00h[3:0] = "0010"
	// Enable AICL, 2h[4] = "1"
	smb1351_set_input_current(1000);
}

static void smb1351_AC_setting(int target_current)
{
	int rc;

	pr_notice("setting AC target_current %d mA Charging\n", target_current);
	// BQ configuration volatile access, 30h[6] = 1
	smb1351_enable_volatile_access();
	// Set Input current = command register, 31h[3] = "1"
	rc = smb1351_masked_write(CMD_REG_IL, INPUT_CURRENT_MODE_MASK, 0x8);
	if (rc)
		pr_err_asus("failed to set input current = command reg.\n");
	// Set USB AC control = USB AC, 31h[0]  ="1"
	rc = smb1351_masked_write(CMD_REG_IL, USB_AC_MODE_MASK, 0x1);
	if (rc)
		pr_err_asus("Faile to set HC mode 31h[3] && [0] = 1\n");

	// Disable AICL, 02h[4] = "0"
	// Set IUSB_IN = target_current mA, 00h[3:0]
	// Enable AICL, 2h[4] = "1"
	smb1351_set_input_current(target_current);
}

int isHVDCP = 0;
int QC_TYPE = 0;

static void check_QC_Type(void)
{
	int rc;
	u8 reg1 = 0;
	u8 reg2 = 0;

    pr_notice("check 47h[4] = 0  %d \n", reg1);//to be removed
    // 47h[4] = "0" & 3Dh[3:1] = "001"
    rc = smb1351_read_reg(IRQ_G_REG, &reg1);
    pr_notice("check 47h[4] = 0  %d \n", reg1);//to be removed
	if (rc)
		pr_err_asus("failed to get IRQ_G_REG\n");

    rc = smb1351_read_reg(STATUS_REG_7, &reg2);
    pr_notice("check 3Dh[3:1] = 001 %d \n", reg2);//to be removed
	if (rc)
		pr_err_asus("failed to get STATUS_REG_7\n");
    reg2 &= HVDCP_STATUS_MASK;  //MASK = 0000 1110b

    if (reg1 & 0x10)
		QC_TYPE = HVDCP_QC3P0;
	else if (!(reg1 & 0x10) && (reg2 == 0x02))
        QC_TYPE = HVDCP_QC2P0;
	else
		QC_TYPE = DCP;

    pr_notice("QC_TYPE is : %d\n", QC_TYPE);
	return;
}
/*
static void smb1351_RERUN_APSD(void)
{
	int rc;
	u8 reg1 = 0;
	u8 reg2 = 0;
	u8 reg3 = 0;
	u8 reg4 = 0;
	pr_notice("RERUN APSD.\n");
	// BQ configuration volatile access, 30h[6] = 1
	smb1351_enable_volatile_access();
	// RERUN APSD, 34h[7] = "1"
	rc = smb1351_masked_write(CMD_REG_HVDCP, RERUN_APSD_MASK, 0x80);
	if (rc)
		pr_err_asus("failed to RERUN APSD.\n");
    rc = smb1351_read_reg(CMD_REG_HVDCP, &reg3);//to be removed
    pr_notice("check RERUN APSD 34h[7] = 1  %d \n", reg3);//to be removed
	// Delay 3 secs
	msleep(3500);
	// Force Quick Charge 2.0 mode, 34h[5] = "1"
	rc = smb1351_masked_write(CMD_REG_HVDCP, FORCE_QC2_MODE_MASK, 0x20);
	if (rc)
		pr_err_asus("failed to FORCE QC 2.0 mode.\n");
    rc = smb1351_read_reg(CMD_REG_HVDCP, &reg4);//to be removed
	pr_notice("check  Force Quick Charge 2.0 mode: 34h[5] = 1 %d\n", reg4); //to be removed

	//Set HC mode, 31h[3] && [0] = "1"
	rc = smb1351_masked_write(CMD_REG_IL, CMD_REG_IL_MASK, 0x09);
	if (rc)
		pr_err_asus("Faile to set HC mode 31h[3] && [0] = 1\n");

	// Disable AICL, 02h[4] = "0"
	// Set IUSB_IN = 1000 mA, 00h[3:0] = "0010"
	// Enable AICL, 2h[4] = "1"
	smb1351_set_input_current(1000);
    // 47h[4] = "0" & 3Dh[3:1] = "001"
    rc = smb1351_read_reg(IRQ_G_REG, &reg1);
    pr_notice("check 47h[4] = 0  %d \n", reg1);//to be removed
	if (rc)
		pr_err_asus("failed to get IRQ_G_REG\n");
    rc = smb1351_read_reg(STATUS_REG_7, &reg2);
    pr_notice("check 3Dh[3:1] = 001 %d \n", reg2);//to be removed
	if (rc)
		pr_err_asus("failed to get STATUS_REG_7\n");
    reg2 &= HVDCP_STATUS_MASK;  //MASK = 0000 1110b
    if (!(reg1 & 0x10) && (reg2 == 0x02))
        isHVDCP = 1;
    pr_notice("===> isHVDCP : %d\n", isHVDCP);
}
  */

static int smb1351_RERUN_APSD_2(void)
{
	int rc;
	pr_notice("RERUN APSD_2.\n");

    //Enable APSD, 02h[2] = "1"
    rc = smb1351_masked_write(VARIOUS_FUNC, VARIOUS_FUNC_MASK, 0x4);
    if (rc)
        pr_err_asus("failed to set APSD register\n");

    // RERUN APSD, 34h[7] = "1"
    rc = smb1351_masked_write(CMD_REG_HVDCP, RERUN_APSD_MASK, 0x80);
    if (rc)
        pr_err_asus("failed to RERUN APSD.\n");

	// Delay 5 secs
	if (!BMT_status.charger_exist) return 1;
	msleep(5000);
	if (!BMT_status.charger_exist) return 1;

	// Force Quick Charge 2.0 mode, 34h[5] = "1"
	rc = smb1351_masked_write(CMD_REG_HVDCP, FORCE_QC2_MODE_MASK, 0x20);
	if (rc)
		pr_err_asus("failed to FORCE QC 2.0 mode.\n");

	// Delay 100 msec
	if (!BMT_status.charger_exist) return 1;
	msleep(100);
	if (!BMT_status.charger_exist) return 1;

	// Set HVDCP = 9V, 12h[7:6] = "01"
	rc = smb1351_masked_write(HVDCP_BAT_MISSING_CTRL_REG, HVDCP_BAT_MISSING_CTRL_MASK, 0x40);
	if (rc)
		pr_err_asus("failed to set HVDCP_BAT_MISSING_CTRL_REG to 9V.\n");
	return 0;
}


static void smb1351_initial_setting(void)
{
	int rc;
	u8 temp=0;
	// BQ configuration volatile access, 30h[6] = 1
	smb1351_enable_volatile_access();

	// Set IUSB_IN = 500mA, 00h[3:0] = "0000"
	rc = smb1351_masked_write(CONFIG_REG, AC_IN_LIMIT_MASK, 0x0);
	if (rc)
		pr_err_asus("failed to set IUSB_IN to 500 mA\n");

	// Set pre-charge current = 400mA, 01h[7:5] = "010"
	rc = smb1351_masked_write(OTHER_CURRENT_REG, PRECHARGE_MASK, 0x40);
	if (rc)
		pr_err_asus("failed to set pre-charge current to 400 mA\n");

	// Set fast charge current = 1000mA, 00h[7:4] = "0000"
	rc = smb1351_masked_write(CONFIG_REG, FASTCHARGE_MASK, 0x0);
	if (rc)
		pr_err_asus("failed to set fast charge current to 1000 mA\n");

	// Set soft cold current compensation = 1000mA, 0Eh[5] = "1"
	rc = smb1351_masked_write(COMP_REG, FASTCHARGE_COMP_MASK, 0x20);
	if (rc)
		pr_err_asus("failed to set soft cold current compensation\n");

	// Set charger disable, 06h[6:5] = "10"
	smb1351_enable_charging(false);

		// Set charger voltage = 4.34V, 03h[5:0] = "101010"
		rc = smb1351_masked_write(FLOAT_VOLT_REG, FLOAT_VOLT_MASK, 0x2A);
		if (rc)
			pr_err_asus("failed to set charger voltage to 4.38 V\n");

	// Set charger enable, 06h[6:5] = "11"
	smb1351_enable_charging(true);

	// Set HVDCP = 5V, 12h[7:6] = "00"
	rc = smb1351_masked_write(HVDCP_BAT_MISSING_CTRL_REG, HVDCP_BAT_MISSING_CTRL_MASK, 0x00);
	if (rc)
		pr_err_asus("failed to set HVDCP_BAT_MISSING_CTRL_REG to 5V.\n");

	// Enable Watchdog Timer, 08h[0] = "1"
	rc = smb1351_masked_write(WDT_TIMER_CTRL_REG, WATCHDOG_EN_MASK, 0x1);
	if (rc)
		pr_err_asus("failed to enable watchdog timer\n");

	// Disable Charging indicator in STAT, 05[5] = "1"
	rc = smb1351_masked_write(STAT_CTRL_REG, STAT_OUTPUT_CTRL_MASK, 0x20);
	if (rc)
		pr_err_asus("failed to disable charger indicator in STAT\n");


	// Set termination current, 01[4:2] = "010",  400mA
	rc = smb1351_masked_write(OTHER_CURRENT_REG, TERMINATION_MASK, 0x08);
	if (rc)
		pr_err_asus("failed to set termination current 400mA\n");

	rc = smb1351_read_reg(CHARGE_CTRL_REG, &temp);
	if (rc)
		pr_err_asus("Read CHARGE_CTRL_REG 04h Failed: rc=%d\n", rc);
	pr_err_asus("Read CHARGE_CTRL_REG 04h : reg=%02x, \n", temp);

	rc = smb1351_read_reg(OTHER_CURRENT_REG, &temp);
	if (rc)
		pr_err_asus("Read OTHER_CURRENT_REG 01h Failed: rc=%d\n", rc);
	pr_err_asus("Read CHARGE_CTRL_REG 01h : reg=%02x, \n", temp);
}

static int smb1351_check_AICL_done(void)
{
	int rc;
	u8 reg = 0;
	rc = smb1351_read_reg(STATUS_REG, &reg);
	if (rc)
		pr_err_asus("failed to get AICL done bit\n");

	return (reg & AICL_DONE_MASK);
}

static int smb1351_check_AICL(unsigned int target_current)
{
	int i, rc;
	u8 reg;

	if (!smb1351_check_AICL_done())
		return 1;

	i = get_closest_input_current(target_current * 100);

	rc = smb1351_read_reg(STATUS_REG, &reg);
	if (rc)
		pr_err_asus("failed to get DCIN input current limit status\n");

	reg &= DCIN_LIMIT_MASK;
	if (reg <= i)
		return 0;
	return 1;
}

/*
 * This is a software workaround to switch charger voltage to high level if
 * no termination state was detected.
 * switch charger voltage to lower level if detect termination status
 */
/*
static void smb1351_switch_charger_voltage(bool high_volt)
{
	int rc;

	smb1351_enable_volatile_access();
	if (high_volt) {
		// Disable Current Termination Ends A Charging Cycle, 04h[6] = "1"
		rc = smb1351_masked_write(CHARGE_CTRL_REG, CURRENT_TERMINATION_MASK, 0x40);
		if (rc)
			pr_err_asus("failed to disable current termination ends a charging cycle\n");

		if (hw_rev != HW_REV_SR) {
			// Set charger voltage = 4.38V, 03h[5:0] = "101100"
			rc = smb1351_masked_write(FLOAT_VOLT_REG, FLOAT_VOLT_MASK, 0x2C);
			if (rc)
				pr_err_asus("failed to switch charger voltage to high voltage\n");
		} else {
			// Set charger voltage = 4.35V, 03h[5:0] = "101011"
			rc = smb1351_masked_write(FLOAT_VOLT_REG, FLOAT_VOLT_MASK, 0x2B);
			if (rc)
				pr_err_asus("failed to switch charger voltage to high voltage\n");
		}
	} else {
		if (hw_rev != HW_REV_SR) {
			// Set charger voltage = 4.35V, 03h[5:0] = "101011"
			rc = smb1351_masked_write(FLOAT_VOLT_REG, FLOAT_VOLT_MASK, 0x2B);
			if (rc)
				pr_err_asus("failed to switch charger voltage to low voltage\n");
		} else {
			// Set charger voltage = 4.30V, 03h[5:0] = "101000"
			rc = smb1351_masked_write(FLOAT_VOLT_REG, FLOAT_VOLT_MASK, 0x28);
			if (rc)
				pr_err_asus("failed to switch charger voltage to low voltage\n");
		}

		// Enable Current Termination Ends A Charging Cycle, 04h[6] = "0"
		rc = smb1351_masked_write(CHARGE_CTRL_REG, CURRENT_TERMINATION_MASK, 0x0);
		if (rc)
			pr_err_asus("failed to enable current termination ends a charging cycle\n");
	}
}
*/
static void smb1351_recharge_func(void)
{
    //3Ah[5] ,  1:at least 1 charging Cycle
	int rc;
	u8 reg = 0;
	rc = smb1351_read_reg(STATUS_REG_4, &reg);
	if (rc)
		pr_err_asus("failed to get STATUS_REG_4 bit\n");
    reg &= 0x20;

    if (BMT_status.SOC <= 98 && reg) {
		pr_err_asus("recharge func: conditions hold\n");
        // Charging Disable, 06h[6:5] = "10"
        rc = smb1351_masked_write(ENABLE_CTRL_REG, ENABLE_PIN_MASK, 0x40);
        if (rc)
            pr_err_asus("failed to disable charging\n");
        // Charging Enable, 06h[6:5] = "11"
        rc = smb1351_masked_write(ENABLE_CTRL_REG, ENABLE_PIN_MASK, 0x60);
        if (rc)
            pr_err_asus("failed to enable charging\n");

    }
}

static void smb1351_JEITA_config(bool enable,
					int fast_chg_target_current, int float_volt)
{
	int rc, i;
	smb1351_enable_volatile_access();

	// BQ configuration volatile access, 30h[6] = 1
	smb1351_enable_volatile_access();

	// Float voltage : 03h[5:0]=float_volt
	rc = smb1351_masked_write(FLOAT_VOLT_REG, FLOAT_VOLT_MASK, float_volt);
	if (rc)
		pr_err_asus("failed to set float voltage 03h[5:0]\n");

	smb1351_enable_charging(enable);

	// Set Fast Charge Current = fast_chg_target_current mA
	i = get_closest_fast_chg_current(fast_chg_target_current * 100);
	rc = smb1351_masked_write(CONFIG_REG, FASTCHARGE_MASK, i << 4);
	if (rc)
		pr_err_asus("failed to set fast charge current limit: %d mA\n", fast_chg_target_current);
}

static void smb1351_JEITA_Rule(void){
	int temp, rc;
	int Vchg;
	u8 reg = 0;
	wake_lock(&chip->jeita_setting_wake_lock);
//	mutex_lock(&chip->jeita_setting_lock);

	// BQ configuration volatile access, 30h[6] = 1
	smb1351_enable_volatile_access();

	// Set Hard Hot Limit = 72 Deg. C, 0Bh[5:4] = "11"
	rc = smb1351_masked_write(TEMP_MONITOR_REG, HOT_ALARM_MASK, 0x30);
	if (rc)
		pr_err_asus("failed to set hard hot limit to 72 Deg. C\n");
	// Set Soft Cold Limit Behavior = No Response,  07h[3:2] = "00"
	rc = smb1351_masked_write(THERM_CTRL_A_REG, SOFTCOLD_LIMIT_MASK, 0x0);
	if (rc)
		pr_err_asus("failed to set soft cold limit behavior = no response\n");
	// Set Soft Hot temp limit = No Response,  07h[1:0] = "00"
	rc = smb1351_masked_write(THERM_CTRL_A_REG, SOFTHOT_LIMIT_MASK, 0x0);
	if (rc)
		pr_err_asus("failed to set soft how temp limit to no response\n");

	temp = BMT_status.temperature * 10;
	if (chip->last_temp_state == less_than_0 &&
		temp <= 30)
		temp = -5;
	else if (chip->last_temp_state == from_0_to_150 &&
		temp <= 180 && temp >= 0)
		temp = 140;
	else if (chip->last_temp_state == from_450_to_600 &&
		temp >= 420  && temp < 600)
		temp = 500;
	else if (chip->last_temp_state == greater_than_600 &&
		temp >= 570)
		temp = 620;

	rc = smb1351_read_reg(FLOAT_VOLT_REG, &reg);
	if (rc)
		pr_err_asus("read charger voltage failed\n");
	reg &= FLOAT_VOLT_MASK;
	if (reg == 0x2A) //4.34V
		Vchg = 1;
	else
		Vchg = 0;   //4.1V 0x1E


	if (temp < 0) {
		//smb1351_recharge_func();
		smb1351_JEITA_config(false, 1000, 0x2A);  // Vchg = 4.34V
		chip->last_temp_state = less_than_0;
	}
	else if (temp >= 0 && temp < 150) {
		smb1351_recharge_func();
		smb1351_JEITA_config(true, 1000, 0x2A);  // Vchg = 4.34V
		chip->last_temp_state = from_0_to_150;
	}
	else if (temp >= 150 && temp < 450) {
		smb1351_JEITA_config(true, 2600, 0x2A);  // Vchg = 4.34V
		smb1351_recharge_func();
		chip->last_temp_state = from_150_to_450;
	}
	else if (temp >= 450 && temp < 600) {
			if (Vchg && BMT_status.bat_vol >= 4100) {
				smb1351_recharge_func();
				smb1351_JEITA_config(false, 1000, 0x2A);  // Vchg = 4.34V
			} else {
				smb1351_JEITA_config(true, 2600, 0x1E); // Vchg = 4.1V
			}
		chip->last_temp_state = from_450_to_600;
	}
	else if (temp >= 600) {
		//smb1351_recharge_func();
		smb1351_JEITA_config(false, 1000, 0x2A); // Vchg = 4.34V
		chip->last_temp_state = greater_than_600;
	}
	else {
		pr_err_asus("unknown temperature range\n");
		chip->last_temp_state = unknown_temp_state;
	}
//	mutex_unlock(&chip->jeita_setting_lock);
	wake_unlock(&chip->jeita_setting_wake_lock);
}

static u32 charging_hw_init(void *data)
{
	u32 status = STATUS_OK;

	upmu_set_rg_bc11_bb_ctrl(1);	/* BC11_BB_CTRL */
	upmu_set_rg_bc11_rst(1);	/* BC11_RST */
	return status;
}

static u32 charging_dump_register(void *data)
{
	u32 status = STATUS_OK;

	return status;
}

static u32 charging_enable(void *data)
{
	u32 status = STATUS_OK;
	u32 enable = *(u32 *) (data);

	smb1351_enable_charging((!enable) ? false : true);

	return status;
}

static u32 charging_set_cv_voltage(void *data)
{
	u32 status = STATUS_OK;
	return status;
}

static u32 charging_get_current(void *data)
{
	u32 status = STATUS_OK;
	int rc;
	u8 reg;

	smb1351_enable_volatile_access();
	rc = smb1351_read_reg(CONFIG_REG, &reg);
	if (rc)
		pr_err_asus("failed to get charging current limit\n");

	*(u32 *) data = fast_chg_current[(reg & 0xf0) >> 4];


	return status;
}

static u32 charging_set_current(void *data)
{
	u32 status = STATUS_OK;

	return status;
}


static u32 charging_set_input_current(void *data)
{
	u32 status = STATUS_OK;

	return status;
}

static u32 charging_get_input_current(void *data)
{
	int rc;
	u8 reg;

	smb1351_enable_volatile_access();
	rc = smb1351_read_reg(STATUS_REG, &reg);
	if (rc)
		pr_err_asus("failed to get input current limit\n");

	*(u32 *) data = input_current[reg & 0xf];
	return STATUS_OK;
}

static u32 charging_get_charging_status(void *data)
{
	u32 status = STATUS_OK;
	int rc;
	u8 reg;
	smb1351_enable_volatile_access();
	rc = smb1351_read_reg(IRQ_C_REG, &reg);
	if (rc)
		pr_err_asus("failed to get charging statue\n");
	*(u32 *) data = (reg & TERMINATION_STATE_MASK);

	pr_err_asus("register 0x42[0] is : %d \n.", (reg & TERMINATION_STATE_MASK));
	return status;
}

static u32 charging_reset_watch_dog_timer(void *data)
{
	u32 status = STATUS_OK;

	return status;
}

static u32 charging_set_hv_threshold(void *data)
{
	u32 status = STATUS_OK;
	return status;
}

static u32 charging_get_hv_status(void *data)
{
	u32 status = STATUS_OK;

	*(bool *) (data) = upmu_get_rgs_vcdt_hv_det();
	return status;
}


static u32 charging_get_battery_status(void *data)
{
	u32 status = STATUS_OK;

	/* upmu_set_baton_tdet_en(1); */
	upmu_set_rg_baton_en(1);
	*(bool *) (data) = upmu_get_rgs_baton_undet();

	return status;
}


static u32 charging_get_charger_det_status(void *data)
{
	u32 status = STATUS_OK;

	*(bool *) (data) = upmu_get_rgs_chrdet();

	return status;
}

static u32 charging_get_charger_type(void *data)
{
	u32 status = STATUS_OK;
	*(int *) (data) = hw_charger_type_detection();
	return status;
}

static u32 charging_get_is_pcm_timer_trigger(void *data)
{
	u32 status = STATUS_OK;
/*  TODO: depend on spm, which would be porting later.
	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(bool *) (data) = true;
	else
		*(bool *) (data) = false;

	battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
*/
	*(bool *) (data) = false;
	return status;
}

static u32 charging_set_platform_reset(void *data)
{
	u32 status = STATUS_OK;

	battery_log(BAT_LOG_CRTI, "charging_set_platform_reset\n");

#if 0				/* need porting of orderly_reboot(). */
	if (system_state == SYSTEM_BOOTING)
		arch_reset(0, NULL);
	else
		orderly_reboot(true);
#endif
	arch_reset(0, NULL);

	return status;
}

static u32 charging_get_platform_boot_mode(void *data)
{
	u32 status = STATUS_OK;

	*(u32 *) (data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());

	return status;
}

static u32 charging_enable_powerpath(void *data)
{
	u32 status = STATUS_OK;
	u32 enable = *(u32 *) (data);
	int rc;

	smb1351_enable_volatile_access();
	if (enable == true)
		rc = smb1351_masked_write(CMD_REG_IL, SUSPEND_MODE_MASK, 0x0);
	else
		rc = smb1351_masked_write(CMD_REG_IL, SUSPEND_MODE_MASK, 0x40);
	if (rc)
		pr_err_asus("failed to %s power path\n", enable ? "enable" : "disable");
	return status;
}

static u32 charging_boost_enable(void *data)
{
	u32 status = STATUS_OK;
	u32 enable = *(u32 *) (data);
	int rc;

	// BQ configuration volatile access, 30h[6] = 1
	smb1351_enable_volatile_access();

	if (enable == true) {
		pr_notice("Enable OTG\n");
/*
		if (!gpio_get_value_cansleep(chip->usbsw_s_gpio)){
			gpio_direction_output(chip->usbsw_s_gpio, 1);
			pr_notice("charging boost, set USBSW_S : %d\n",gpio_get_value_cansleep(chip->usbsw_s_gpio));
		}
*/
		gpio_direction_output(chip->otg_en_gpio, 1);
		// OTG current limit = 1000 mA, 0Ah[3:2] = "11"
		rc = smb1351_masked_write(OTG_TLIM_CTRL_REG, OTG_DCIN_CURRENT_MASK, 0xC);
		if (rc)
			pr_err_asus("failed to set DCIN current limit to 1000 mA\n");
	} else {
		pr_notice("Disable OTG\n");
		// OTG current limit = 500mA, 0Ah[3:2] = "01"
		rc = smb1351_masked_write(OTG_TLIM_CTRL_REG, OTG_DCIN_CURRENT_MASK, 0x4);
		if (rc)
			pr_err_asus("failed to set DCIN current limit to 500 mA\n");
		gpio_direction_output(chip->otg_en_gpio, 0);
/*
		if (gpio_get_value_cansleep(chip->usbsw_s_gpio)){
			gpio_direction_output(chip->usbsw_s_gpio, 0);
			pr_notice("charging boost, clear USBSW_S : %d\n",gpio_get_value_cansleep(chip->usbsw_s_gpio));
		}
*/
	}

	return status;
}

static u32(*const charging_func[CHARGING_CMD_NUMBER]) (void *data) = {
charging_hw_init,
	    charging_dump_register,
	    charging_enable,
	    charging_set_cv_voltage,
	    charging_get_current,
	    charging_set_current,
	    charging_get_input_current,
	    charging_set_input_current,
	    charging_get_charging_status,
	    charging_reset_watch_dog_timer,
	    charging_set_hv_threshold,
	    charging_get_hv_status,
	    charging_get_battery_status,
	    charging_get_charger_det_status,
	    charging_get_charger_type,
	    charging_get_is_pcm_timer_trigger,
	    charging_set_platform_reset,
	    charging_get_platform_boot_mode, charging_enable_powerpath, charging_boost_enable};

s32 smb1351_control_interface(int cmd, void *data)
{
	s32 status;

	if (cmd < CHARGING_CMD_NUMBER)
		status = charging_func[cmd] (data);
	else
		return STATUS_UNSUPPORTED;

	return status;
}

/*
 *	return true if cv is set to lower level
 *	lower than 4.35/4.30 V in ER/SR.
 */
bool smb1351_skip_battery_100Percent_tracking(void)
{
	int rc;
	u8 reg = 0;

	smb1351_enable_volatile_access();
	rc = smb1351_read_reg(FLOAT_VOLT_REG, &reg);
	if (rc)
		pr_err_asus("failed to read float voltage\n");
	reg &= 0x3F;

	pr_err_asus("FLOAT_VOLT_REG is: %d\n", reg);

	return (reg < 0x2A);
}

#ifdef PD_CHARGING_DRIVER_SUPPORT
void smb1351_suspend_enable(bool enable)
{
	int rc;
	if (!BMT_status.charger_exist)
		return;

	smb1351_enable_volatile_access();
	if (enable)
		rc = smb1351_masked_write(CMD_REG_IL, SUSPEND_MODE_MASK, 0x40);
	else
		rc = smb1351_masked_write(CMD_REG_IL, SUSPEND_MODE_MASK, 0x0);
	if (rc)
		pr_err_asus("failed to %s DCIN suspend\n", enable ? "enable" : "disable");
}

bool smb1351_is_battery_low(void)
{
	return (BMT_status.bat_vol < 3500);
}

static void smb1351_pd_jump_voltage(void)
{
	int rc;
	int current_target, voltage_target;

	if (!BMT_status.charger_exist)
		return;

	voltage_target = usbpd_getvoltagecurrent(USBPD_VOLTAGE);
	current_target = usbpd_getvoltagecurrent(USBPD_CURRENT);

	pr_notice("Start to jump voltage: %d mV/current: %d mA\n",
			voltage_target, current_target);

	smb1351_enable_volatile_access();
	/* FIXME : motify PD */
	smb1351_suspend_enable(true);
	msleep(500);

	smb1351_suspend_enable(false);

	// Set Input current = command register, 31h[3] = "1"
	rc = smb1351_masked_write(CMD_REG_IL, INPUT_CURRENT_MODE_MASK, 0x8);
	if (rc)
		pr_err_asus("failed to set input current = command register\n");

	// Set USB AC control = USB AC, 31h[0] ="1"
	rc = smb1351_masked_write(CMD_REG_IL, USB_AC_MODE_MASK, 0x1);
	if (rc)
		pr_err_asus("failed to set USB AC control = USB AC\n");

	// Disable AICL
	// Set IUSB_IN = IAdp(Final), 00h[3:0] = IAdp(Final)
	// Enable AICL
	smb1351_set_input_current(current_target);
}

static void smb1351_pd_5V_setting(void)
{
	int rc;
	int current_target;

	if (!BMT_status.charger_exist)
		return;

	current_target = usbpd_getvoltagecurrent(USBPD_CURRENT);
	pr_notice("PD not ready: set current: %d mA\n", current_target);

	// BQ configuration volatile access, 30h[6] = 1
	smb1351_enable_volatile_access();
	// Set Input current = command register, 31h[3] = "1"
	rc = smb1351_masked_write(CMD_REG_IL, INPUT_CURRENT_MODE_MASK, 0x8);
	if (rc)
		pr_err_asus("failed to set input current = command register\n");
	// Set USB AC control = USB AC, 31h[0]  ="1"
	rc = smb1351_masked_write(CMD_REG_IL, USB_AC_MODE_MASK, 0x1);
	if (rc)
		pr_err_asus("failed to set USB AC control = USB AC\n");

	smb1351_set_input_current(current_target);

}

static void smb1351_typeC_check_AICL(int target)
{
	if (!BMT_status.charger_exist)
		return;

	smb1351_enable_volatile_access();
	if (!smb1351_check_AICL(target)) {
		pr_notice("AICL done && result <= %d mA, update charger type\n", target);
		bat_update_charger_type(hw_charger_type_detection());
	}
}

/*
static void smb1351_typeC_flow_hsuan(void)
{
	if (!BMT_status.charger_exist)
		return;

	smb1351_AC_1A_setting();

	msleep(2000);
	if (BMT_status.charger_type == TYPEC_1_5A_CHARGER) {
		// Set IUSB_IN = 1500 mA
		pr_notice("setting 1500mA charging\n");
		smb1351_set_input_current(1500);
		smb1351_typeC_check_AICL(1300);
	} else if (BMT_status.charger_type == TYPEC_3A_CHARGER) {
		// Set IUSB_IN = 3000 mA
		pr_notice("setting 3A charging\n");
		smb1351_set_input_current(3000);
		smb1351_typeC_check_AICL(2000);
	}
}*/
static int smb1351_typeC_flow(void)
{
	if (!BMT_status.charger_exist)
		return 1;

	switch(BMT_status.TypeC_charger_type) {
		case TYPEC_1_5A_CHARGER:
			chip->CHG_TYPE = TYPEC_1P5A;
			smb1351_AC_1A_setting();
			// Set IUSB_IN = 1500 mA
			pr_notice("setting 1500mA charging\n");
			smb1351_AC_setting(1500);
            if (!BMT_status.charger_exist) return 1;
			msleep(2000);
            if (!BMT_status.charger_exist) return 1;
			smb1351_typeC_check_AICL(1300);
			break;
		case TYPEC_3A_CHARGER:
			chip->CHG_TYPE = TYPEC_3P0A;
			smb1351_AC_1A_setting();
			// Set IUSB_IN = 3000 mA
			pr_notice("setting 3A charging\n");
			smb1351_AC_setting(3000);
            if (!BMT_status.charger_exist) return 1;
			msleep(2000);
            if (!BMT_status.charger_exist) return 1;
			smb1351_typeC_check_AICL(2000);
			break;
		case TYPEC_PD_5V_CHARGER:
			chip->CHG_TYPE = PD;
			smb1351_pd_5V_setting();
			break;
		case TYPEC_PD_9V_CHARGER:
			chip->CHG_TYPE = PD;
			if (smb1351_is_battery_low()) {
				smb1351_pd_5V_setting();
				break;
			}
			smb1351_pd_jump_voltage();
			break;
		default: //others
			chip->CHG_TYPE = CT_UNDEFINED;
			pr_notice("Others in Type C flow\n");
	}
	return 0;
}
#endif

/*
static void smb1351_BC_1_2_flow(void)
{
	int rc;
	int i = 0;
	if (!BMT_status.charger_exist)
		return;

	smb1351_enable_volatile_access();

	if (BMT_status.charger_type == STANDARD_HOST ||
					BMT_status.charger_type == CHARGING_HOST) {
		pr_notice("setting 500mA charging\n");
		// Set Input current = command register, 31h[3] = "1"
		rc = smb1351_masked_write(CMD_REG_IL, INPUT_CURRENT_MODE_MASK, 0x8);
		if (rc)
			pr_err_asus("failed to set input current = command register\n");

		// Set USB AC control = USB 500, 31h[1:0] ="00"
		rc = smb1351_masked_write(CMD_REG_IL, USB_CTRL_MODE_MASK, 0x0);
		if (rc)
			pr_err_asus("failed to set USB AC control = USB 500\n");
	} else {
		do {
			smb1351_AC_1A_setting();
			//smb1351_RERUN_APSD();
			i++;
		} while (!smb1351_check_AICL(500) && i < 10);
	}
}
*/

static int asus_adapter_detect_func(bool queue_back) {
    u8 adc_result;

	if (BMT_status.charger_type != STANDARD_CHARGER)
        gpio_direction_output(chip->usbsw_s_gpio, 0);

    if (queue_back) goto asus_adapter_label1;
    pr_notice("ASUS adapter detect ++\n");
    if (gpio_get_value_cansleep(chip->adp_vh_en_gpio)){
            gpio_direction_output(chip->adp_vh_en_gpio, 0);
            pr_notice("Clear ADP_VH_EN : %d\n",gpio_get_value_cansleep(chip->adp_vh_en_gpio));
    }
    if (!gpio_get_value_cansleep(chip->usbsw_s_gpio)){
            gpio_direction_output(chip->usbsw_s_gpio, 1);
            pr_notice("Set USBSW_S : %d\n",gpio_get_value_cansleep(chip->usbsw_s_gpio));
    }
    switch (QC_TYPE) {
		case HVDCP_QC3P0:
		case HVDCP_QC2P0:
			msleep(100);
			break;
		default:
			pr_notice("queun_delay_work 30 sec \n.");
			queue_delayed_work(smb1351_wq2, &chip->notHVDCP_ASUS_adapter_detect_work, 30*HZ);
			return 1;
	}
asus_adapter_label1:
	smb1351_suspend_enable(true);
    msleep(5);

    adc_result = us5587_dump_value();
	pr_notice("adc_result 1 is : %04x\n", adc_result);
    if (adc_result <= 0x3F) {
        if (!gpio_get_value_cansleep(chip->adp_vh_en_gpio)){
            gpio_direction_output(chip->adp_vh_en_gpio, 1);
            pr_notice("Set ADP_VH_EN : %d\n",gpio_get_value_cansleep(chip->adp_vh_en_gpio));
        }
        msleep(5);
        adc_result = us5587_dump_value();
		pr_notice("adc_result 2 is : %04x\n", adc_result);
        if (adc_result > 0xD4)
            chip->CHG_TYPE = CT_UNDEFINED;
        else {
			switch (QC_TYPE) {
				case HVDCP_QC3P0:
					if (adc_result >= 0x3C && adc_result <= 0x52) //200K
						chip->CHG_TYPE = HVDCP_ASUS_200K_2A;
					else
						chip->CHG_TYPE = HVDCP_OTHERS_1A;
					break;
				case HVDCP_QC2P0:
					chip->CHG_TYPE = HVDCP_OTHERS_1A;
					break;
				default:  //DCP
					if (adc_result >= 0x92 && adc_result <= 0xA8) //750K
						chip->CHG_TYPE = DCP_ASUS_750K_2A;
					else
						chip->CHG_TYPE = CT_UNDEFINED;
					break;
			}
        }
    } else {
        if (adc_result > 0xBE)
			switch (QC_TYPE) {
				case HVDCP_QC3P0:
				case HVDCP_QC2P0:
					chip->CHG_TYPE = HVDCP_OTHERS_PB_1A;
					break;
				default:
					chip->CHG_TYPE = DCP_PB_2A;
			}
        else {
			switch (QC_TYPE) {
				case HVDCP_QC3P0:
				case HVDCP_QC2P0:
					chip->CHG_TYPE = HVDCP_OTHERS_PB_1A;
					break;
				default:
					chip->CHG_TYPE = CT_UNDEFINED;
			}
		}
	}
    //31h[6] = "0"
    smb1351_suspend_enable(false);
    //MT6391(KP_COL3) = L
    if (gpio_get_value_cansleep(chip->adp_vh_en_gpio)){
            gpio_direction_output(chip->adp_vh_en_gpio, 0);
            pr_notice("Clear ADP_VH_EN : %d\n",gpio_get_value_cansleep(chip->adp_vh_en_gpio));
    }
	if (BMT_status.charger_type != STANDARD_CHARGER)
        gpio_direction_output(chip->usbsw_s_gpio, 0);
	return 0;
}


const char *chg_type_index2string(int);

static void notHVDCP_ASUS_adapter_detect_work_func(struct work_struct *work)
{
//	mutex_lock(&chip->notHVDCP_lock);
	pr_notice("[BAT] notHVDCP_ASUS_adapter_detect_work++\n");
	if (BMT_status.charger_type != STANDARD_CHARGER) {
        gpio_direction_output(chip->usbsw_s_gpio, 0);
		goto work_EXIT;
	}
	smb1351_enable_volatile_access();
	asus_adapter_detect_func(1);
//XXXX

	switch (chip->CHG_TYPE) {
		case HVDCP_ASUS_200K_2A:
			//TODO:show "+" icon
			asus_show_qc_flag = 1;
			mt_battery_update_status(); //power_supply_changed(&chip->bat_psy);

			if (smb1351_RERUN_APSD_2()==1) return;
			smb1351_AC_setting(2000);
			break;
		case HVDCP_OTHERS_1A:
		case HVDCP_OTHERS_PB_1A:
			if (smb1351_RERUN_APSD_2()==1) return;
			break;
		case DCP_ASUS_750K_2A:
		case DCP_PB_2A:
			smb1351_AC_setting(2000);
			break;
		default:
			switch (BMT_status.TypeC_charger_type) {
				case TYPEC_1_5A_CHARGER:
				case TYPEC_3A_CHARGER:
                    if (smb1351_typeC_flow()==1) goto work_EXIT;
					break;
				default:
					chip->CHG_TYPE = DCP_OTHERS_1A;
			}

	}

	pr_err_asus("[BAT]CHG_TYPE is :%d => %s\n", chip->CHG_TYPE, chg_type_index2string(chip->CHG_TYPE));

//ooooo
	if (BMT_status.charger_exist)
		smb1351_JEITA_Rule();
work_EXIT:
	cancel_delayed_work(&chip->notHVDCP_ASUS_adapter_detect_work);
//	mutex_unlock(&chip->notHVDCP_lock);
	pr_notice("[BAT] notHVDCP_ASUS_adapter_detect_work--\n");
}

static void DCP_1min_wait_work_func(struct work_struct *work)
{
	pr_notice("[BAT] DCP_1min_wait_work_func  ++\n");
	if (!BMT_status.charger_exist) {
		cancel_delayed_work(&chip->DCP_1min_wait_work);
		return;
	}
	smb1351_charging_algorithm(DCP_1min_wait);

	pr_notice("[BAT] DCP_1min_wait_work_func  --\n");
}

static void DCP_5sec_wait_work_func(struct work_struct *work)
{
	pr_notice("[BAT] DCP_5sec_wait_work_func  ++\n");
	if (BMT_status.charger_type != STANDARD_CHARGER ) {
		gpio_direction_output(chip->usbsw_s_gpio, 0);
		cancel_delayed_work(&chip->DCP_5sec_wait_work);
		pr_err_asus("[BAT] NO DCP exist! 5sec_wait_work_func BYE!\n");
		return;
	}
	smb1351_charging_algorithm(DCP_5sec_wait);

	pr_notice("[BAT] DCP_5sec_wait_work_func  --\n");
}

static void DCP_aftPLUS_5sec_wait_work_func(struct work_struct *work)
{
	pr_notice("[BAT] DCP_aftPLUS_5sec_wait_work_func  ++\n");
	if (BMT_status.charger_type != STANDARD_CHARGER ) {
		gpio_direction_output(chip->usbsw_s_gpio, 0);
		cancel_delayed_work(&chip->DCP_aftPLUS_5sec_wait_work);
		pr_err_asus("[BAT] NO DCP exist! aftPLUS_5sec_wait_work_func BYE!\n");
		return;
	}
	smb1351_charging_algorithm(DCP_aftPLUS_5sec_wait);

	pr_notice("[BAT] DCP_aftPLUS_5sec_wait_work_func  --\n");
}

const char *index2string(int index)
{
	static char* CHARGER_TYPE_ARY[] = {"CHARGER_UNKNOWN",
		"STANDARD_HOST",      /*  USB : 450mA */
		"CHARGING_HOST",
		"NONSTANDARD_CHARGER",    /*  AC : 450mA~1A */
		"STANDARD_CHARGER",   /*  AC : ~1A */
		"APPLE_2_1A_CHARGER", /*  2.1A apple charger */
		"APPLE_1_0A_CHARGER", /*  1A apple charger */
		"APPLE_0_5A_CHARGER", /*  0.5A apple charger */
		"TYPEC_RdUSB_CHARGER",    /*  TypeC 5VUSB charger */
		"TYPEC_1_5A_CHARGER", /*  TypeC 5V1.5A charger */
		"TYPEC_3A_CHARGER", /* TypeC 5V3A charger */
		"TYPEC_PD_NO_READY_CHARGER", /*  Select the profile but not ready */
		"TYPEC_PD_5V_CHARGER", /*  PD 5V charger */
		"TYPEC_PD_9V_CHARGER", /*  PD 9V charger */
		"Out_of_range"
	};
	if (index >= 0 && index < 14)
		return CHARGER_TYPE_ARY[index];
	else
		return CHARGER_TYPE_ARY[14];
}

const char *chg_type_index2string(int index)
{
	static char* CHG_TYPE_ARY[] = {"CT_UNDEFINED",
		"DCP_PB_2A",
		"DCP_ASUS_750K_2A",
		"DCP_OTHERS_1A",
		"HVDCP_OTHERS_PB_1A",
		"HVDCP_ASUS_200K_2A",
		"HVDCP_ASUS_750K_2A",
		"HVDCP_OTHERS_1A",
		"CDP_1P5A",
		"SDP_0P5A",
		"OTHERS_1A",
		"TYPEC_1P5A",
		"TYPEC_3P0A",
		"PD",
		"FLOATING_0P5A",
		"Out_of_range"
	};
	if (index >= 0 && index < 15)
		return CHG_TYPE_ARY[index];
	else
		return CHG_TYPE_ARY[15];
}

void UTS_battery_status(void)
{
	bool isON = false;
	int ret_value = 8888;
//	const char *cable = index2string(BMT_status.charger_type);
	if (BMT_status.charger_exist){
		switch (BMT_status.charger_type){
			case STANDARD_CHARGER:
				ASUSEvtlog("[USB]set_chg_mode:ASUS AC");
				break;
			case CHARGING_HOST:
				ASUSEvtlog("[USB]set_chg_mode:CDP");
				break;
			case STANDARD_HOST:
				ASUSEvtlog("[USB]set_chg_mode:USB");
				break;
			default:
				ASUSEvtlog("[USB]set_chg_mode:UNKNOWN");
		}
		isON = true;
	}
	else
		ASUSEvtlog("[USB]set_chg_mode:None");


	ret_value = battery_meter_get_battery_current();
	if (battery_meter_get_battery_current_sign() == true)
		ret_value = 0 - ret_value;

	pr_err_asus("[BAT][Ser]report Capacity==>%d,FCC:%dmA,BMS:%d,ZCV:%d",BMT_status.UI_SOC,5900 ,BMT_status.SOC, BMT_status.ZCV);
	pr_err_asus(" V:%dmV,Cur:%dmA,Temp:%dC",BMT_status.bat_vol, BMT_status.ICharging, BMT_status.temperature);
	pr_err_asus(" BatCur:%dmA", ret_value/10);
//	ASUSEvtlog(" Cable:%d%s, Status:%s, Charging:%d", BMT_status.charger_type, *cable, BMT_status.charger_exist, isON);
	ASUSEvtlog(" Cable:%d (%s)", BMT_status.charger_type, index2string(BMT_status.charger_type));
//	ASUSEvtlog(" Status:%s, Charging:%d",if(BMT_status.charger_exist), isON);
	ASUSEvtlog(" Charging:%d", isON);

	if (smb1351_is_battery_low())
		ASUSEvtlog("[BAT]Low Voltage");
	return;
}

void smb1351_charging_algorithm(int DelayBack)
{
	int cur_type;
	int rc =0;
	int data;
	u8 reg;

	if (g_ftm_mode && charger_limit_enable && BMT_status.SOC >= charger_limit) {
		pr_notice("FACTORY MODE:Disable charging for SOC > %lu percent\n" ,charger_limit);
		smb1351_enable_volatile_access();
		smb1351_enable_charging(false);
	}
	if (demo_app_status_flag && BMT_status.SOC >= demo_app_charger_limit) {
		pr_notice("demo_app:Disable charging for SOC > %lu percent\n" ,demo_app_charger_limit);
		smb1351_enable_volatile_access();
		smb1351_enable_charging(false);
	}

	pr_notice("++\n");
	mutex_lock(&chip->charging_setting_lock);
	QC_TYPE = 0;

	if (BMT_status.charger_type != STANDARD_CHARGER)
        gpio_direction_output(chip->usbsw_s_gpio, 0);




    pr_notice("*********USBSW_S : %d\n",gpio_get_value_cansleep(chip->usbsw_s_gpio));
	cur_type = BMT_status.charger_type;
	UTS_battery_status();
	charger_type = BMT_status.charger_type;
	switch (DelayBack) {
		case 0:
			break;
		case DCP_1min_wait:
			goto DCP_1min_wait_milestone1;
		case DCP_5sec_wait:
			goto DCP_5sec_wait_milestone;
		case DCP_aftPLUS_5sec_wait:
			goto DCP_aftPLUS_5sec_wait_milestone;
		default:
			break;
	}
	if (chip->last_charger_type != cur_type) {
		pr_notice("Setting Charging Current for cable type : %d -> %d\n",
			chip->last_charger_type, cur_type);
		chip->last_charger_type = cur_type;
		if (BMT_status.charger_exist) {
			smb1351_initial_setting();
			switch (cur_type) {
#ifdef PD_CHARGING_DRIVER_SUPPORT
//				case TYPEC_PD_9V_CHARGER:
//					smb1351_pd_jump_voltage();
//					break;
//				case TYPEC_PD_5V_CHARGER:
//					smb1351_pd_5V_setting();
//					break;
//				case TYPEC_3A_CHARGER:
//				case TYPEC_1_5A_CHARGER:
//                    switch (chip->last_charger_type) {
//                        case CHARGER_UNKNOWN:
//                        case STANDARD_HOST:
//                        //Set HC mode, 31h[3] && [0] = "1"
//                            rc = smb1351_masked_write(CMD_REG_IL, CMD_REG_IL_MASK, 0x09);
//                            if (rc)
//                                pr_err_asus("failed to set input current = command register\n");
//                            smb1351_set_input_current(1500);
//					        smb1351_typeC_flow();
//					        break;
//                        default:
//					        smb1351_typeC_flow();
//                    }
//					break;
//				case TYPEC_PD_NO_READY_CHARGER:
//					// do nothing
//					break;
#endif
				case STANDARD_CHARGER:
                    gpio_direction_output(chip->usbsw_s_gpio, 1);
                    pr_notice("Set USBSW_S : %d\n",gpio_get_value_cansleep(chip->usbsw_s_gpio));
                    smb1351_AC_setting(1000);
DCP_1min_wait_milestone1:
                    if (smb1351_is_battery_low()) {  //msleep(60*1000);
						pr_notice("queun_delay_work 60 sec \n.");
						queue_delayed_work(smb1351_wq3, &chip->DCP_1min_wait_work, 60*HZ);
						goto algorithm_EXIT;
					}


					// RERUN APSD, 34h[7] = "1"
					rc = smb1351_masked_write(CMD_REG_HVDCP, RERUN_APSD_MASK, 0x80);
					if (rc)
						pr_err_asus("failed to RERUN APSD.\n");

					// Delay 5 secs
					if (BMT_status.charger_type != STANDARD_CHARGER) {
						gpio_direction_output(chip->usbsw_s_gpio, 0);
						goto algorithm_EXIT;
					}
                    pr_err_asus("----> queue DCP_5sec_wait_work_func.\n");
					queue_delayed_work(smb1351_wq4, &chip->DCP_5sec_wait_work, 5*HZ);
					goto algorithm_EXIT;
DCP_5sec_wait_milestone:
					if (BMT_status.charger_type != STANDARD_CHARGER) {
						gpio_direction_output(chip->usbsw_s_gpio, 0);
						goto algorithm_EXIT;
					}
					// Force Quick Charge 2.0 mode, 34h[5] = "1"
					rc = smb1351_masked_write(CMD_REG_HVDCP, FORCE_QC2_MODE_MASK, 0x20);
					if (rc)
						pr_err_asus("failed to FORCE QC 2.0 mode.\n");

					check_QC_Type();

                    switch (BMT_status.TypeC_charger_type) {
                        case TYPEC_PD_9V_CHARGER:
                        case TYPEC_PD_5V_CHARGER:
                            if(smb1351_typeC_flow()==1) goto algorithm_EXIT;
                            break;
                        default:
                            gpio_direction_output(chip->adp_vh_en_gpio, 0);
                            //Disable APSD, 02h[2] = "0"
                            rc = smb1351_masked_write(VARIOUS_FUNC, VARIOUS_FUNC_MASK, 0x0);
                            if (rc)
                                pr_err_asus("failed to disable APSD register\n");

                            // RERUN APSD, 34h[7] = "1"
                            rc = smb1351_masked_write(CMD_REG_HVDCP, RERUN_APSD_MASK, 0x80);
                            if (rc)
                                pr_err_asus("failed to RERUN APSD.\n");
                            if (BMT_status.charger_type != STANDARD_CHARGER) {
                                gpio_direction_output(chip->usbsw_s_gpio, 0);
                                goto algorithm_EXIT;
                            }
							msleep(500);
                            if (BMT_status.charger_type != STANDARD_CHARGER) {
                                gpio_direction_output(chip->usbsw_s_gpio, 0);
                                goto algorithm_EXIT;
                            }
                            rc = asus_adapter_detect_func(0);
							if (rc) { //the rest of work is left to delayed work.
								pr_notice("--\n");
								mutex_unlock(&chip->charging_setting_lock);
								return;
							}

                            switch (chip->CHG_TYPE) {
                                case HVDCP_ASUS_200K_2A:
                                case HVDCP_OTHERS_1A:
                                case HVDCP_OTHERS_PB_1A:
                                    if (chip->CHG_TYPE == HVDCP_ASUS_200K_2A) {
                                    //TODO:show "+" icon
                                        asus_show_qc_flag = 1;
                                        mt_battery_update_status(); //power_supply_changed(&chip->bat_psy);
                                    }
//+++
                                    //Enable APSD, 02h[2] = "1"
                                    rc = smb1351_masked_write(VARIOUS_FUNC, VARIOUS_FUNC_MASK, 0x4);
	                                if (rc)
                                    pr_err_asus("failed to set APSD register\n");

                                    // RERUN APSD, 34h[7] = "1"
                                    rc = smb1351_masked_write(CMD_REG_HVDCP, RERUN_APSD_MASK, 0x80);
                                    if (rc)
                                        pr_err_asus("failed to RERUN APSD.\n");

                                    // Delay 5 secs
                                    pr_err_asus("----> queue DCP_aftPLUS_5sec_wait_work_func.\n");
                                    queue_delayed_work(smb1351_wq5, &chip->DCP_aftPLUS_5sec_wait_work, 5*HZ);
                                    goto algorithm_EXIT;
                                    //msleep(5000);
DCP_aftPLUS_5sec_wait_milestone:
                                    // Force Quick Charge 2.0 mode, 34h[5] = "1"
                                    rc = smb1351_masked_write(CMD_REG_HVDCP, FORCE_QC2_MODE_MASK, 0x20);
                                    if (rc)
                                        pr_err_asus("failed to FORCE QC 2.0 mode.\n");

                                    // Delay 100 msec
                                    if (BMT_status.charger_type != STANDARD_CHARGER) {
                                        gpio_direction_output(chip->usbsw_s_gpio, 0);
                                        goto algorithm_EXIT;
                                    }
                                    msleep(100);
                                    if (BMT_status.charger_type != STANDARD_CHARGER) goto algorithm_EXIT;

                                    // Set HVDCP = 9V, 12h[7:6] = "01"
                                    pr_err_asus("Set HVDCP = 9V.\n");
                                    rc = smb1351_masked_write(HVDCP_BAT_MISSING_CTRL_REG, HVDCP_BAT_MISSING_CTRL_MASK, 0x40);
                                    if (rc)
                                        pr_err_asus("failed to set HVDCP_BAT_MISSING_CTRL_REG to 9V.\n");
                                    //---
                                    //
                                    if (chip->CHG_TYPE == HVDCP_ASUS_200K_2A)
                                        smb1351_AC_setting(2000);
                                    break;
                                case DCP_ASUS_750K_2A:
                                case DCP_PB_2A:
                                    smb1351_AC_setting(2000);
                                    break;
                                default:
									switch (BMT_status.TypeC_charger_type) {
										case TYPEC_1_5A_CHARGER:
										case TYPEC_3A_CHARGER:
											if (smb1351_typeC_flow()==1) goto algorithm_EXIT;
											break;
										default:
											chip->CHG_TYPE = DCP_OTHERS_1A;
									}
                            }
					}
					break;
				case CHARGING_HOST:  //CDP
					pr_notice("CHARGING_HOST (CDP) welcome! TypeC_charger_type ==> %d\n", BMT_status.TypeC_charger_type);
					smb1351_AC_1A_setting();
                    switch (BMT_status.TypeC_charger_type) {
						case TYPEC_1_5A_CHARGER:
						case TYPEC_3A_CHARGER:
                        case TYPEC_PD_9V_CHARGER:
                        case TYPEC_PD_5V_CHARGER:
                            if(smb1351_typeC_flow()==1) goto algorithm_EXIT;
                            break;
                        default:
							smb1351_AC_setting(1500);
							chip->CHG_TYPE = CDP_1P5A;
					}
					break;
				case STANDARD_HOST: //USB
					pr_notice("STANDARD_HOST welcome! TypeC_charger_type ==> %d\n", BMT_status.TypeC_charger_type);
                    switch (BMT_status.TypeC_charger_type) {
                        case TYPEC_1_5A_CHARGER:
                        case TYPEC_3A_CHARGER:
                        case TYPEC_PD_9V_CHARGER:
                        case TYPEC_PD_5V_CHARGER:
                            if(smb1351_typeC_flow()==1) goto algorithm_EXIT;
                            break;
                        default:
                            chip->CHG_TYPE = SDP_0P5A;
                    }
					break;
				case NONSTANDARD_CHARGER: //Other charging port, include ASUS PD
					pr_notice("NONSTANDARD_CHARGER welcome! TypeC_charger_type ==> %d\n", BMT_status.TypeC_charger_type);
					smb1351_AC_1A_setting();
					switch (BMT_status.TypeC_charger_type) {
						case TYPEC_1_5A_CHARGER:
						case TYPEC_3A_CHARGER:
						case TYPEC_PD_9V_CHARGER:
						case TYPEC_PD_5V_CHARGER:
							if(smb1351_typeC_flow()==1) goto algorithm_EXIT;
							break;
						default:
							smb1351_AC_setting(500);
							chip->CHG_TYPE = FLOATING_0P5A;
					}
					break;
				default:
					smb1351_AC_1A_setting();
					pr_err_asus("Unknown BC12 cable type : %d\n", cur_type);
                    if (!BMT_status.charger_exist) goto algorithm_EXIT;
					msleep(1000);
                    if (!BMT_status.charger_exist) goto algorithm_EXIT;
                    switch (BMT_status.TypeC_charger_type) {
                        case TYPEC_1_5A_CHARGER:
                        case TYPEC_3A_CHARGER:
                        case TYPEC_PD_9V_CHARGER:
                        case TYPEC_PD_5V_CHARGER:
                            pr_notice("Type C device.");
                            if (smb1351_typeC_flow()==1) goto algorithm_EXIT;
                            break;
                        default:  //case TYPEC_PD_NO_READY_CHARGER:
                            pr_err_asus("Unknown TypeC charger.");
					}

			}
		}
        else {  //adapter unplugged
			asus_show_qc_flag = 0;
            if (gpio_get_value_cansleep(chip->usbsw_s_gpio)){
                gpio_direction_output(chip->usbsw_s_gpio, 0);
                pr_notice("Adapter unplugged, clear USBSW_S : %d\n",gpio_get_value_cansleep(chip->usbsw_s_gpio));
            }
			cancel_delayed_work(&chip->notHVDCP_ASUS_adapter_detect_work);
			cancel_delayed_work(&chip->DCP_1min_wait_work);
			chip->CHG_TYPE = CT_UNDEFINED;
        }
		pr_notice("Set Charging Current Done for cable type : %s\n", index2string(cur_type));
	} else {
		if (BMT_status.charger_exist) {
			pr_notice("same charger: %s\n", index2string(cur_type));

			if (BMT_status.charger_type != STANDARD_CHARGER) {
				gpio_direction_output(chip->usbsw_s_gpio, 0);
				goto algorithm_EXIT;
			}

			switch (cur_type) {
#ifdef PD_CHARGING_DRIVER_SUPPORT
//				case TYPEC_PD_9V_CHARGER:
//				case TYPEC_PD_NO_READY_CHARGER:
//					// do nothing
//					break;
//				case TYPEC_PD_5V_CHARGER:
//					if (!smb1351_is_battery_low())
//						usbpd_jump_voltage();
//					break;
//				case TYPEC_3A_CHARGER:
//					smb1351_typeC_check_AICL(2000);
//					break;
//				case TYPEC_1_5A_CHARGER:
//					smb1351_typeC_check_AICL(1300);
//					break;
#endif
				case STANDARD_CHARGER:
					pr_notice("STANDARD_CHARGER come again! TypeC_charger_type ==> %d\n", BMT_status.TypeC_charger_type);
					switch (BMT_status.TypeC_charger_type) {
						case TYPEC_PD_9V_CHARGER:
						case TYPEC_PD_NO_READY_CHARGER:
							break;
						case TYPEC_PD_5V_CHARGER:
							if(!smb1351_is_battery_low())
								usbpd_jump_voltage();
							break;
						case TYPEC_3A_CHARGER:
							smb1351_typeC_check_AICL(2000);
							break;
						case TYPEC_1_5A_CHARGER:
							smb1351_typeC_check_AICL(1300);
							break;


					}
//					pr_notice("==> STANDARD_CHARGER, welcome!!: %d\n", cur_type);
//					while (!smb1351_check_AICL(500) && i < 10) {
//					pr_notice("==> ENTERING 10 loop!!: \n");
//						smb1351_AC_1A_setting();
//						i++;
//					}
					break;
				case CHARGING_HOST:
					pr_notice("CHARGING_HOST come again! TypeC_charger_type ==> %d\n", BMT_status.TypeC_charger_type);
					// do nothing
					break;
				case STANDARD_HOST:
					pr_notice("STANDARD_HOST come again! TypeC_charger_type ==> %d\n", BMT_status.TypeC_charger_type);
					// do nothing
					break;
				default:
					pr_err_asus("Unknown cable type : %d\n", cur_type);
					break;
			}
		}
	}

	pr_notice("%s welcome!! and TypeC_charger_type ==> %s\n", index2string(cur_type), index2string(BMT_status.TypeC_charger_type));
	pr_err_asus("[BAT]CHG_TYPE is :%d => %s\n", chip->CHG_TYPE, chg_type_index2string(chip->CHG_TYPE));

	if (BMT_status.charger_type != STANDARD_CHARGER)
        gpio_direction_output(chip->usbsw_s_gpio, 0);

	if (BMT_status.charger_exist && (!DelayBack))
		smb1351_JEITA_Rule();

algorithm_EXIT:
	rc = smb1351_read_reg(STATUS_REG, &reg);
	if (rc)
		pr_err_asus("failed get input current limit 36h[3:0].\n");
	data = input_current[reg & 0xf];
	pr_err_asus("36h[3:0] Input current limit is: %dmA.\n", data/100);


	rc = smb1351_read_reg(CONFIG_REG, &reg);
	if (rc)
		pr_err_asus("failed to get FCC limit 00h\n");
	data = input_current[reg & 0x0f];
	pr_err_asus("00h[3:0] AC input current limit is: %dmA.\n", data/100);

	mutex_unlock(&chip->charging_setting_lock);
	pr_notice("--\n");
	return;
}

static void smb1351_charging_state_changed_work_func(struct work_struct *work)
{
//	if (BMT_status.charger_exist)
//		smb1351_JEITA_Rule();
		return;
}

static irqreturn_t smb1351_charging_state_isr(int irq_num, void *fusb)
{
	pr_notice("get charging status changed irq!!\n");
	queue_delayed_work(smb1351_wq, &chip->charging_state_changed_work, 0);
	return IRQ_HANDLED;
}

static ssize_t reg_status_get(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	int i = 0;
	char tmp_buf[64];
	u8 reg;

	smb1351_enable_volatile_access();
	sprintf(tmp_buf, "smb1351 Configuration Registers Detail\n"
						"==================\n");
	strcpy(buf, tmp_buf);

	smb1351_enable_volatile_access();

	for (i = 0; i <= 20; i++) {
		smb1351_read_reg(0x0+i, &reg);
		sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", 0x0+i, reg);
		strcat(buf, tmp_buf);
	}
	for (i = 0; i <= 26; i++) {
		smb1351_read_reg(0x15+i, &reg);
		sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", 0x15+i, reg);
		strcat(buf, tmp_buf);
	}
	for (i = 0; i <= 24; i++) {
		smb1351_read_reg(0x30+i, &reg);
		sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", 0x30+i, reg);
		strcat(buf, tmp_buf);
	}

	return strlen(buf);
}

static ssize_t vbus_voltage_get(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", BMT_status.charger_vol);
}

static ssize_t chargeric_temp_get(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	unsigned long temp;
	mtkts_charger_get_temp(&temp);
	return sprintf(buf, "%lu\n", temp);
}


static ssize_t chargeric_status_get(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", charger_status);
}

static ssize_t charger_limit_enable_set(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (!g_ftm_mode)
		return -1;
	if(buf[0] == '1')
		charger_limit_enable = true;
	else if(buf[0] == '0')
		charger_limit_enable = false;
	else
		return -1;
	return count;
}
static ssize_t charger_limit_enable_get(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", charger_limit_enable);
}

static ssize_t charger_limit_set(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	long limit;
	int rc;
	if (!g_ftm_mode)
		return -1;

	rc = kstrtol(buf, 10, &limit);
	if (!rc && limit >= 0 && limit <= 100)
		charger_limit = limit;
	else
		return rc;
	return count;
}
static ssize_t charger_limit_get(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%lu\n", charger_limit);
}

static ssize_t battery_id_get(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	unsigned int bat_id;
	asustek_get_battery_id(&bat_id);
	return sprintf(buf, "%d\n", bat_id);
}

static ssize_t charger_type_get(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%s\n", chg_type_index2string(chip->CHG_TYPE));
}

static ssize_t demo_app_status_store(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t len)
{
	int tmp = 0;

	tmp = buf[0] - 48;
	if (tmp == 0) {
		demo_app_status_flag = false;
		printk("[BAT][CHG] demo_app_status_flag = 0\n");
	} else if (tmp == 1) {
		demo_app_status_flag = true;
		printk("[BAT][CHG] demo_app_status_flag = 1\n");
	}
	return len;
}

static ssize_t demo_app_status_show(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", demo_app_status_flag);
}

static DEVICE_ATTR(reg_status, S_IRUGO, reg_status_get, NULL);
static DEVICE_ATTR(vbus_voltage, S_IRUGO,vbus_voltage_get, NULL);
static DEVICE_ATTR(chargerIC_temp, S_IRUGO, chargeric_temp_get, NULL);
static DEVICE_ATTR(battery_id, S_IRUGO, battery_id_get, NULL);
static DEVICE_ATTR(chargerIC_status, S_IRUGO , chargeric_status_get, NULL);
static DEVICE_ATTR(charger_limit_enable, S_IRUGO | S_IWUSR, charger_limit_enable_get, charger_limit_enable_set);
static DEVICE_ATTR(charger_limit, S_IRUGO | S_IWUSR, charger_limit_get, charger_limit_set);
static DEVICE_ATTR(charger_type, S_IRUGO, charger_type_get, NULL);
static DEVICE_ATTR(demo_app_status, S_IRUGO | S_IWUSR, demo_app_status_show, demo_app_status_store);

static struct attribute *smb1351_charger_attributes[] = {
	&dev_attr_reg_status.attr,
	&dev_attr_vbus_voltage.attr,
	&dev_attr_chargerIC_temp.attr,
	&dev_attr_battery_id.attr,
	&dev_attr_chargerIC_status.attr,
	&dev_attr_charger_limit_enable.attr,
	&dev_attr_charger_limit.attr,
	&dev_attr_charger_type.attr,
	&dev_attr_demo_app_status.attr,
	NULL
};

static const struct attribute_group smb1351_charger_group = {
	.attrs = smb1351_charger_attributes,
};

static int smb1351_driver_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int rc;
	if (BMT_status.charger_exist) {
		pr_err_asus("smb1351_driver_suspend...\n");
		smb1351_enable_volatile_access();
		// disable Watchdog Timer, 08h[0] = "0"
		rc = smb1351_masked_write(WDT_TIMER_CTRL_REG, WATCHDOG_EN_MASK, 0x0);
		if (rc)
			pr_err_asus("failed to disable watchdog timer\n");

		// Set Hard Hot Limit = 53 Deg. C, 0Bh[5:4] = "00"
		rc = smb1351_masked_write(TEMP_MONITOR_REG, HOT_ALARM_MASK, 0x00);
		if (rc)
			pr_err_asus("failed to set hard hot limit to 53 Deg. C\n");

		// Set Soft Cold Limit Behavior = Charge current compensation,  07h[3:2] = "01"
		rc = smb1351_masked_write(THERM_CTRL_A_REG, SOFTCOLD_LIMIT_MASK, 0x4);
		if (rc)
			pr_err_asus("failed to set soft cold limit behavior = Charge current compensation\n");

		// Set Soft Hot temp limit = Float voltage compensation,  07h[1:0] = "10"
		rc = smb1351_masked_write(THERM_CTRL_A_REG, SOFTHOT_LIMIT_MASK, 0x2);
		if (rc)
			pr_err_asus("failed to set soft hot temp limit behavior to Float voltage compensation\n");
	}

	return 0;
}

static int smb1351_driver_resume(struct i2c_client *client)
{
	pr_notice("\n");

	return 0;
}

static void smb1351_driver_shutdown(struct i2c_client *client)
{
	pr_notice("\n");
}

static ssize_t show_smb1351_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_notice("0x%x\n", g_reg_value_smb1351);
	return sprintf(buf, "0x%x\n", g_reg_value_smb1351);
}

static ssize_t store_smb1351_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	pr_notice("\n");
	smb1351_enable_volatile_access();
	if (buf != NULL && size != 0) {
		pr_notice("buf is %s and size is %d\n", buf, (int)size);
		reg_address = simple_strtoul(buf, &pvalue, 16);

		if (size > 3) {
			reg_value = simple_strtoul((pvalue + 1), NULL, 16);
			pr_notice("write smb1351 reg 0x%x with value 0x%x !\n",
				reg_address, reg_value);
			ret = smb1351_masked_write(reg_address, 0xFF, reg_value);
		} else {
			ret = smb1351_read_reg(reg_address, &g_reg_value_smb1351);
			pr_notice("read smb1351 reg 0x%x with value 0x%x !\n",
				reg_address, g_reg_value_smb1351);
			pr_notice
			    ("Please use \"cat smb1351_access\" to get value\r\n");
		}
	}
	return size;
}


static DEVICE_ATTR(smb1351_access, S_IWUSR | S_IRUGO, show_smb1351_access, store_smb1351_access);

static int smb1351_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	pr_notice("smb1351_user_space_probe!\n");
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_smb1351_access);

	return 0;
}

struct platform_device smb1351_user_space_device = {
	.name = "smb1351-user",
	.id = -1,
};

static struct platform_driver smb1351_user_space_driver = {
	.probe = smb1351_user_space_probe,
	.driver = {
		   .name = "smb1351-user",
		   },
};
static int smb1351_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;

	pr_notice("++\n");
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err_asus("Couldn't allocate memory\n");
		return -ENOMEM;
	}
	chip->client = client;
	chip->dev = &client->dev;
	chip->last_charger_type = CHARGER_UNKNOWN;
	chip->last_temp_state = unknown_temp_state;
	asus_show_qc_flag = 0;
	chip->CHG_TYPE = CT_UNDEFINED;

	mutex_init(&chip->read_write_lock);
	mutex_init(&chip->charging_setting_lock);
//	mutex_init(&chip->jeita_setting_lock);
	wake_lock_init(&chip->jeita_setting_wake_lock,
			WAKE_LOCK_SUSPEND, "jeita_setting_wake_lock");
	i2c_set_clientdata(client, chip);

	smb1351_initial_setting();
//	smb1351_JEITA_Rule();   CHECK THIS

	ret = of_get_smb1351_platform_data(chip->dev);
	if (ret) {
		pr_err_asus("failed to get smb1351 platform data through dt!!\n");
		return ret;
	}

	ret = gpio_request_one(chip->otg_en_gpio, GPIOF_OUT_INIT_LOW,
			"OTG_EN");
	if (ret) {
		pr_err_asus("Couldn't request GPIO for OTG pinctrl\n");
		return ret;
	}
	ret = gpio_request_one(chip->usbsw_s_gpio, GPIOF_OUT_INIT_LOW,
			"USBSW_S");
    pr_notice("Now USBSW_S : %d\n",gpio_get_value_cansleep(chip->usbsw_s_gpio));
	if (ret) {
		pr_err_asus("Couldn't request GPIO for USBSW_S pinctrl\n");
		return ret;
	}
	ret = gpio_request_one(chip->adp_vh_en_gpio, GPIOF_OUT_INIT_LOW,
			"ADP_VH_EN");
    pr_notice("Now ADP_VH_EN : %d\n",gpio_get_value_cansleep(chip->adp_vh_en_gpio));
	if (ret) {
		pr_err_asus("Couldn't request GPIO for ADP_VH_EN pinctrl\n");
		return ret;
	}

	smb1351_wq = create_singlethread_workqueue("smb1351_wq");
	INIT_DELAYED_WORK(&chip->charging_state_changed_work,
			smb1351_charging_state_changed_work_func);

	smb1351_wq2 = create_singlethread_workqueue("smb1351_wq2");
	INIT_DELAYED_WORK(&chip->notHVDCP_ASUS_adapter_detect_work,
			notHVDCP_ASUS_adapter_detect_work_func);

	smb1351_wq3 = create_singlethread_workqueue("smb1351_wq3");
	INIT_DELAYED_WORK(&chip->DCP_1min_wait_work,
			DCP_1min_wait_work_func);

	smb1351_wq4 = create_singlethread_workqueue("smb1351_wq4");
	INIT_DELAYED_WORK(&chip->DCP_5sec_wait_work,
			DCP_5sec_wait_work_func);

	smb1351_wq5 = create_singlethread_workqueue("smb1351_wq5");
	INIT_DELAYED_WORK(&chip->DCP_aftPLUS_5sec_wait_work,
			DCP_aftPLUS_5sec_wait_work_func);

	ret =  request_any_context_irq(chip->charging_state_irq, smb1351_charging_state_isr,
			IRQF_TRIGGER_FALLING, "smb1351-charging-status", chip);
	if (ret) {
		pr_err_asus("failed to register smb1351 charging state eint isr\n");
		return -1;
	}
	ret = enable_irq_wake(chip->charging_state_irq);

	bat_charger_register(smb1351_control_interface);

	/* smb1351 user space access interface */
	ret = platform_device_register(&smb1351_user_space_device);
	if (ret) {
		pr_err_asus("Unable to device register(%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&smb1351_user_space_driver);
	if (ret) {
		pr_err_asus("Unable to register driver (%d)\n", ret);
		return ret;
	}

	smb1351_enable_volatile_access();
	ret = sysfs_create_group(&client->dev.kobj, &smb1351_charger_group);
	if (ret) {
		pr_err_asus("unable to create the sysfs\n");
	}
	ret = smb1351_masked_write(CMD_REG_I2C, BQ_CONFIG_ACCESS_MASK, 0x40);

	if (!ret)
		charger_status = true;
	gpio_direction_output(chip->usbsw_s_gpio, 0);
	pr_notice("Init clear USBSW_S : %d\n",gpio_get_value_cansleep(chip->usbsw_s_gpio));
	pr_notice("--\n");
	return 0;
}

static int smb1351_driver_remove(struct i2c_client *client)
{
	mutex_destroy(&chip->read_write_lock);
	wake_lock_destroy(&chip->jeita_setting_wake_lock);
	gpio_free(chip->otg_en_gpio);
	gpio_free(chip->usbsw_s_gpio);
	gpio_free(chip->adp_vh_en_gpio);
	return 0;
}

static const struct i2c_device_id smb1351_charger_id[] = { {"smb1351-charger", 0}, {} };

//#ifdef CONFIG_OF
static const struct of_device_id smb1351_match_table[] = {
	{.compatible = "qcom,smb1351-charger"},
	{},
};

MODULE_DEVICE_TABLE(of, smb1351_match_table);

static int of_get_smb1351_platform_data(struct device *dev)
{
	if (dev->of_node) {
                const struct of_device_id *match;

                match = of_match_device(of_match_ptr(smb1351_match_table), dev);
                if (!match) {
                        pr_err_asus("Error: No device match found\n");
                        return -ENODEV;
                }
        }
	chip->otg_en_gpio = of_get_named_gpio(dev->of_node, "otg-gpio", 0);
	chip->usbsw_s_gpio = of_get_named_gpio(dev->of_node, "usbsw_s-gpio", 0);
	chip->adp_vh_en_gpio = of_get_named_gpio(dev->of_node, "adp_vh_en-gpio", 0);
	pr_notice("OTG enable gpio: %d\n", chip->otg_en_gpio);
	pr_notice("USBSW_S gpio: %d\n", chip->usbsw_s_gpio);
	pr_notice("ADP_VH_EN gpio: %d\n", chip->adp_vh_en_gpio);

	chip->charging_state_irq = irq_of_parse_and_map(dev->of_node, 0);
	if (chip->charging_state_irq <= 0) {
		pr_err_asus("invalid eint num\n");
		chip->charging_state_irq = 0;
		return -EINVAL;
	}
	pr_notice("charging state eint: %d\n", chip->charging_state_irq);

	return 0;
}
//#else
//static int of_get_smb1351_platform_data(struct device *dev)
//{
//	return 0;
//}
//#endif

static struct i2c_driver smb1351_charger_driver = {
	.driver = {
		   .name = "smb1351-charger",
		   .owner		= THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = smb1351_match_table,
#endif
		   },
	.probe = smb1351_driver_probe,
	.shutdown = smb1351_driver_shutdown,
	.suspend = smb1351_driver_suspend,
	.resume = smb1351_driver_resume,
	.remove		= smb1351_driver_remove,
	.id_table = smb1351_charger_id,
};

module_i2c_driver(smb1351_charger_driver);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SMB1351 Charger Driver");

