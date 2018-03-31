#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/string.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/syscore_ops.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <mach/upmu_sw.h>
#else
#include <string.h>
#endif

#include "lcm_drv.h"
#include "nt35523_TIANMA_qxga_dsi_vdo_truly.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#else
#endif

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH  (1536)
#define FRAME_HEIGHT (2048)

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = {
	   .set_reset_pin = NULL,
	   .udelay = NULL,
	   .mdelay = NULL,
};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

unsigned int GPIO_TPS65132_EN_P;
unsigned int GPIO_TPS65132_EN_N;
unsigned int GPIO_LCD_RST_EN;
#define REGFLAG_DELAY                                       0xFC
#define REGFLAG_END_OF_TABLE                                0xFD

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)    lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                   lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)               lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                         lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0

#define I2C_ID_NAME "tps65132"

/*****************************************************************************
* GLobal Variable
*****************************************************************************/
static struct i2c_client *tps65132_i2c_client;
/*****************************************************************************
* Function Prototype
*****************************************************************************/
static int tps65132_probe(struct i2c_client *client,
			  const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
int tps65132_write_bytes(unsigned char addr, unsigned char value);

/*****************************************************************************
* Data Structure
*****************************************************************************/
struct tps65132_dev {
	   struct i2c_client *client;
};

static const struct i2c_device_id tps65132_id[] = {
	   {I2C_ID_NAME, 0},
	   {}
};

static const struct of_device_id tps65132_of_match[] = {
	   {.compatible = "mediatek, tps65132"},
	   {},
};

static struct i2c_driver tps65132_i2c_driver = {
	   .id_table = tps65132_id,
	   .probe = tps65132_probe,
	   .remove = tps65132_remove,
	   .driver = {
	       .owner = THIS_MODULE,
	       .name = "tps65132",
	       .of_match_table = tps65132_of_match,
	       },

};

static int dbg_reg_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}

static ssize_t dbg_reg_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
    int ret, cmd;

    sscanf(buf, "%d", &cmd);

    if (cmd == 1) {
        ret = tps65132_write_bytes(0x00, 0x0e);
        if (ret < 0)
            printk("%s ----tps6132--- i2c write error-----\n", __func__);
        else
            printk("%s ----tps6132--- i2c write success-----\n", __func__);
    } else {
        printk("%s: unknown command\n", __func__);
    }

    return count;
}

static const struct file_operations dbg_reg_fops = {
    .open = dbg_reg_open,
    .write = dbg_reg_write,
};

static int tps65132_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct dentry *dent = debugfs_create_dir("tps65132", NULL);

	   pr_warn("tps65132_i2c_probe\n");
	   pr_warn("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
	   tps65132_i2c_client = client;

	/* add debugfs nodes */
	debugfs_create_file("write_reg", S_IRUGO | S_IWUSR, dent, NULL, &dbg_reg_fops);

	   return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
	   pr_warn("tps65132_remove\n");
	   tps65132_i2c_client = NULL;
	   i2c_unregister_device(client);
	   return 0;
}

int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	   int ret = 0;
	   struct i2c_client *client = tps65132_i2c_client;
	   char write_data[2] = { 0 };

	if (client == NULL) {
		pr_err("%s: no i2 client !!\n", __func__);
		return -EPERM;
	}

	   write_data[0] = addr;
	   write_data[1] = value;
	   ret = i2c_master_send(client, write_data, 2);
	   if (ret < 0)
		pr_err("tps65132 write data fail !!\n");
	   return ret;
}
EXPORT_SYMBOL(tps65132_write_bytes);

static int __init tps65132_i2c_init(void)
{
	   pr_warn("tps65132_i2c_init\n");
	   i2c_add_driver(&tps65132_i2c_driver);
	   pr_warn("tps65132_i2c_init success\n");
	   return 0;
}

static void __exit tps65132_i2c_exit(void)
{
	   pr_warn("tps65132_i2c_exit\n");
	   i2c_del_driver(&tps65132_i2c_driver);
}

module_init(tps65132_i2c_init);
module_exit(tps65132_i2c_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL");


#define   LCM_DSI_CMD_MODE                                  0

//#define  PUSH_TABLET_USING
/* #define REGFLAG_DELAY                                       0xFFFC */
/* #define REGFLAG_END_OF_TABLE                                0xFFFD */

#ifdef PUSH_TABLET_USING
struct LCM_setting_table {
	   unsigned char cmd;
	   unsigned char count;
	   unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	/* page 0 */
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x00}},
	{0xB1, 2, {0xE8, 0x11}},
	{0xB5, 2, {0x08, 0x00}},

	/* gamma */
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x02}},
	{0xB0, 1, {0x40}},
	{0xD1, 16, {0x00, 0x40, 0x00, 0x50, 0x00, 0xDF, 0x00, 0xF5, 0x01, 0x0A,
			0x01, 0x2E, 0x01, 0x44, 0x01, 0x6C}},
	{0xD2, 16, {0x01, 0x8B, 0x01, 0xBF, 0x01, 0xE2, 0x02, 0x23, 0x02, 0x56,
			0x02, 0x58, 0x02, 0x87, 0x02, 0xBB}},
	{0xD3, 16, {0x02, 0xDE, 0x03, 0x0B, 0x03, 0x2A, 0x03, 0x50, 0x03, 0x6B,
			0x03, 0x87, 0x03, 0x99, 0x03, 0xB0}},
	{0xD4, 4, {0x03, 0xC5, 0x03, 0xF0}},

	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x00}},
	{0xBD, 4, {0X01, 0X8D, 0X20, 0X10}},
	{0xBC, 2, {0X0F, 0x21}},
	{0xC0, 3, {0X03, 0X02, 0X02}},

	/* page 1 */
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x01}},
	{0xCE, 1, {0x04}},
	{0xB3, 1, {0x32}},
	{0xC3, 2, {0x82, 0x82}},
	{0xB4, 2, {0x0F, 0x0F}},
	{0xBC, 2, {0x6E, 0x00}},
	{0xBD, 2, {0x6E, 0x00}},

	/* page 3 */
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x03}},
	{0xB0, 4, {0X00, 0X32, 0X00, 0X00}},
	{0xB2, 7, {0X02, 0X00, 0X11, 0X05, 0X00, 0X00, 0X00}},
	{0xB3, 7, {0X02, 0X00, 0X10, 0X05, 0X00, 0X00, 0X00}},
	{0xB4, 7, {0X02, 0X00, 0X0F, 0X05, 0X00, 0X00, 0X00}},
	{0xB6, 10, {0X00, 0X00, 0X17, 0X05, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00}},
	{0xBA, 7, {0XC6, 0X06, 0X00, 0X0B, 0X01, 0X50, 0X65}},
	{0xBB, 7, {0XC6, 0X06, 0X00, 0X0A, 0X01, 0X50, 0X65}},
	{0xC4, 2, {0x10, 0x00}},
	{0xC5, 2, {0x00, 0x00}},
	{0xEF, 1, {0x41}},

	/* page 5 */
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x05}},
	{0xB0, 4, {0X1F, 0X03, 0X00, 0X03}},
	{0xD0, 6, {0X00, 0X48, 0X01, 0X00, 0X00, 0X00}},
	{0xD1, 6, {0X00, 0X48, 0X02, 0X00, 0X00, 0X00}},

	/* page 6 */
	{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x06}},
	{0xB0, 5, {0X3D, 0X3D, 0X3D, 0X3D, 0X3D}},
	{0xB1, 5, {0X3D, 0X3D, 0X3D, 0X08, 0X32}},
	{0xB2, 5, {0X33, 0X10, 0X11, 0X12, 0X13}},
	{0xB3, 5, {0X14, 0X15, 0X3A, 0X03, 0X04}},
	{0xB4, 2, {0x00, 0x3D}},
	{0xB5, 5, {0X3D, 0X3D, 0X3D, 0X3D, 0X3D}},
	{0xB6, 5, {0X3D, 0X3D, 0X3D, 0X09, 0X32}},
	{0xB7, 5, {0X33, 0X16, 0X17, 0X18, 0X19}},
	{0xB8, 5, {0X1A, 0X1B, 0X3A, 0X05, 0X01}},
	{0xB9, 2, {0x02, 0x3D}},
	{0xC0, 5, {0X3D, 0X3D, 0X3D, 0X3D, 0X3D}},
	{0xC1, 5, {0X3D, 0X3D, 0X3D, 0X08, 0X33}},
	{0xC2, 5, {0X32, 0X13, 0X12, 0X11, 0X10}},
	{0xC3, 5, {0X15, 0X14, 0X3A, 0X00, 0X04}},
	{0xC4, 2, {0x03, 0x3D}},
	{0xC5, 5, {0X3D, 0X3D, 0X3D, 0X3D, 0X3D}},
	{0xC6, 5, {0X3D, 0X3D, 0X3D, 0X09, 0X33}},
	{0xC7, 5, {0X32, 0X19, 0X18, 0X17, 0X16}},
	{0xC8, 5, {0X1B, 0X1A, 0X3A, 0X02, 0X01}},
	{0xC9, 2, {0x05, 0x3D}},
	{0x35, 1, {0x00}},
	{0x51, 1, {0xFF}},
	{0x53, 1, {0x2C}},
	{0x55, 1, {0x01}},

	{0x29, 0, {} }, /* display on */
	{REGFLAG_DELAY, 20, {} },
	{0x11, 0, {} }, /* sleep out */
	{REGFLAG_DELAY, 120, {} },
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	   {0x28, 0, {} },
	   {0x10, 0, {} },
	   {REGFLAG_DELAY, 120, {} }
};

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	   unsigned int i;

	pr_warn("%s+\n", __func__);

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list,
			    force_update);
		}
	}
	pr_warn("%s-\n", __func__);
}
#endif

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	   memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	if (GPIO == 0xFFFFFFFF) {
#ifdef BUILD_LK
		printf("[LK/LCM] GPIO_TPS65132_EN_P =   0x%x\n", GPIO_TPS65132_EN_P);
		printf("[LK/LCM] GPIO_TPS65132_EN_N =   0x%x\n", GPIO_TPS65132_EN_N);
		printf("[LK/LCM] GPIO_LCD_RST_EN =  0x%x\n", GPIO_LCD_RST_EN);
#elif (defined BUILD_UBOOT)
#else
#endif

		return;
	}
#ifdef BUILD_LK
	   mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	   mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	   mt_set_gpio_out(GPIO, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
#else
	   gpio_direction_output(GPIO, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
#endif
}

static void lcm_get_params(LCM_PARAMS *params)
{
	printk("%s: LCM_ID=%d\n", __func__, g_ASUS_lcmID);

	   memset(params, 0, sizeof(LCM_PARAMS));

	   params->type = LCM_TYPE_DSI;

	   params->width = FRAME_WIDTH;
	   params->height = FRAME_HEIGHT;
	   params->lcm_if = LCM_INTERFACE_DSI_DUAL;
	   params->lcm_cmd_if = LCM_INTERFACE_DSI_DUAL;
	   params->dsi.cont_clock = 0;
	   params->dsi.clk_lp_per_line_enable = 1;

#if (LCM_DSI_CMD_MODE)
	   params->dsi.mode = CMD_MODE;
#else
	   params->dsi.mode = BURST_VDO_MODE;
#endif

/* DSI */
/* Command mode setting */
	   params->dsi.LANE_NUM = LCM_FOUR_LANE;
/* The following defined the fomat for data coming from LCD engine. */
	   params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	   params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	   params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	   params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

/* Highly depends on LCD driver capability. */
	   params->dsi.packet_size = 256;
	   params->dsi.ssc_disable = 1;
/* video mode timing */

	   params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	   params->dsi.vertical_sync_active = 2;
	   params->dsi.vertical_backporch = 20;

    if (g_ASUS_lcmID == 2)
        params->dsi.vertical_frontporch = 16; /* BOE LCM */
    else
        params->dsi.vertical_frontporch = 10; /* TIANMA LCM */

	   params->dsi.vertical_active_line = FRAME_HEIGHT;

	   params->dsi.horizontal_sync_active = 2;
	   params->dsi.horizontal_backporch = 80;
	   params->dsi.horizontal_frontporch = 120;
	   params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	   params->dsi.PLL_CLOCK = 420; /* this value must be in MTK suggested table */
	   params->dsi.ufoe_enable = 1;
	   params->dsi.ufoe_params.lr_mode_en = 1;
	   params->dsi.HS_TRAIL = 1;
	   params->physical_width = 147;
	   params->physical_height = 196;
}

static void lcm_init_power(void)
{
	   pr_warn("%s\n", __func__);
}

static void lcm_resume_power(void)
{
	   pr_warn("%s+\n", __func__);

	   /* turn on 1.8V */
	   upmu_set_rg_vgp4_vosel(3);
	   upmu_set_rg_vgp4_sw_en(1);

	   MDELAY(20);

	   pr_warn("%s-\n", __func__);
}

static void lcm_suspend_power(void)
{
	   pr_warn("%s+\n", __func__);

	   lcm_set_gpio_output(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);

	   MDELAY(2);

	   lcm_set_gpio_output(GPIO_TPS65132_EN_N, GPIO_OUT_ZERO);
	   MDELAY(2);
	   lcm_set_gpio_output(GPIO_TPS65132_EN_P, GPIO_OUT_ZERO);

	   MDELAY(2);

	   /* turn off 1.8V */
	   upmu_set_rg_vgp4_sw_en(0);

	   pr_warn("%s-\n", __func__);
}

static void lcm_init(void)
{
	   pr_warn("%s\n", __func__);
}


static void lcm_suspend(void)
{
#ifndef PUSH_TABLET_USING
	unsigned int data_array[16];
#endif

	   pr_warn("%s+\n", __func__);

#ifdef PUSH_TABLET_USING
	   push_table(lcm_suspend_setting,
	       sizeof(lcm_suspend_setting) /
	       sizeof(struct LCM_setting_table), 1);
#else
	printk("%s, dsi_set_cmdq v1\n", __func__);

	// {0x28, 0, {} }
	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(20);

	// {0x10, 0, {} }
	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(120);

#endif

	   pr_warn("%s-\n", __func__);
}

static void lcm_resume(void)
{
#ifndef PUSH_TABLET_USING
	unsigned int data_array[16];
#endif
	   unsigned char cmd = 0x0;
	   unsigned char data = 0x0a;	/* +-5.0V */
	   int ret = 0;

	   pr_warn("%s+\n", __func__);

	   lcm_set_gpio_output(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	   MDELAY(2);
	   lcm_set_gpio_output(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	   MDELAY(2);
	   lcm_set_gpio_output(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	   MDELAY(30);

	   lcm_set_gpio_output(GPIO_TPS65132_EN_P, GPIO_OUT_ONE);

	MDELAY(1);

	   ret = tps65132_write_bytes(cmd, data);
	if (ret < 0)
		pr_warn
		("[KERNEL]nt35523_TIANMA----tps6132---cmd=%0x-- i2c write error-----\n",
		cmd);
	else
		pr_warn
		("[KERNEL]nt35523_TIANMA----tps6132---cmd=%0x-- i2c write success-----\n",
		cmd);

	   MDELAY(2);

	   lcm_set_gpio_output(GPIO_TPS65132_EN_N, GPIO_OUT_ONE);

	MDELAY(1);

	   cmd = 0x01;

	   ret = tps65132_write_bytes(cmd, data);
	if (ret < 0)
		pr_warn
		("[KERNEL]nt35523_TIANMA----tps6132---cmd=%0x-- i2c write error-----\n",
		cmd);
	else
		pr_warn
		("[KERNEL]nt35523_TIANMA----tps6132---cmd=%0x-- i2c write success-----\n",
		cmd);

	MDELAY(3);

#ifdef PUSH_TABLET_USING
	   push_table(lcm_initialization_setting,
	       sizeof(lcm_initialization_setting) /
	       sizeof(struct LCM_setting_table), 1);
#else
	printk("%s, dsi_set_cmdq v1\n", __func__);

	if (g_ASUS_hwID <= 1) {
		printk("SR device, send v5 initial code\n");

		// {0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x00}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x52aa55f0;
		data_array[2] = 0x0008;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB1, 2, {0xE8, 0x11}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x0011e8b1;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xB5, 2, {0x08, 0x00}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x000008b5;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xBD, 4, {0X01, 0X8D, 0X20, 0X10}}
		data_array[0] = 0x00053902;
		data_array[1] = 0x208d01bd;
		data_array[2] = 0x10;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xBC, 2, {0X0F, 0x25}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x00250fbc;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xC0, 3, {0X03, 0X02, 0X02}}
		data_array[0] = 0x00043902;
		data_array[1] = 0x020203c0;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x01}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x52aa55f0;
		data_array[2] = 0x0108;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xCE, 1, {0x04}}
		data_array[0] = 0x04ce1500;
		dsi_set_cmdq(data_array, 1, 1);

		// {0xB3, 1, {0x32}}
		data_array[0] = 0x32b31500;
		dsi_set_cmdq(data_array, 1, 1);

		// {0xC3, 2, {0x82, 0x82}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x008282c3;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xB4, 2, {0x0F, 0x0F}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x000f0fb4;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xBC, 2, {0x4B, 0x00}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x00004bbc;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xBD, 2, {0x4B, 0x00}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x00004bbd;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x02}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x52aa55f0;
		data_array[2] = 0x0208;
		dsi_set_cmdq(data_array, 3, 1);

		//{0xB0, 1, {0x40}}
		data_array[0] = 0x40b01500;
		dsi_set_cmdq(data_array, 1, 1);

		// {0xD1, 16, {0x00, 0x40, 0x00, 0x52, 0x00, 0x6D, 0x00, 0x85, 0x00, 0xA7,
		//			0x00, 0xC7, 0x00, 0xE4, 0x01, 0x0E}}
		data_array[0] = 0x00113902;
		data_array[1] = 0x004000d1;
		data_array[2] = 0x006d0052;
		data_array[3] = 0x00a70085;
		data_array[4] = 0x01e400c7;
		data_array[5] = 0x0e;
		dsi_set_cmdq(data_array, 6, 1);

		// {0xD2, 16, {0x01, 0x34, 0x01, 0x70, 0x01, 0x9E, 0x01, 0xE7, 0x02, 0x21,
		//			0x02, 0x23, 0x02, 0x56, 0x02, 0x91}}
		data_array[0] = 0x00113902;
		data_array[1] = 0x013401d2;
		data_array[2] = 0x019e0170;
		data_array[3] = 0x022102e7;
		data_array[4] = 0x02560223;
		data_array[5] = 0x91;
		dsi_set_cmdq(data_array, 6, 1);

		// {0xD3, 16, {0x02, 0xB7, 0x02, 0xEB, 0x03, 0x0E, 0x03, 0x3A, 0x03, 0x55,
		//			0x03, 0x79, 0x03, 0x8F, 0x03, 0xA9}}
		data_array[0] = 0x00113902;
		data_array[1] = 0x02b702d3;
		data_array[2] = 0x030e03eb;
		data_array[3] = 0x0355033a;
		data_array[4] = 0x038f0379;
		data_array[5] = 0xa9;
		dsi_set_cmdq(data_array, 6, 1);

		// {0xD4, 4, {0x03, 0xC7, 0x03, 0xF0}}
		data_array[0] = 0x00053902;
		data_array[1] = 0x03c703d4;
		data_array[2] = 0xf0;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x03}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x52aa55f0;
		data_array[2] = 0x0308;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB0, 4, {0X00, 0X32, 0X00, 0X00}}
		data_array[0] = 0x00053902;
		data_array[1] = 0x003200b0;
		data_array[2] = 0x00;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB2, 7, {0X02, 0X00, 0X11, 0X05, 0X00, 0X00, 0X00}}
		data_array[0] = 0x00083902;
		data_array[1] = 0x110002b2;
		data_array[2] = 0x00000005;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB3, 7, {0X02, 0X00, 0X10, 0X05, 0X00, 0X00, 0X00}}
		data_array[0] = 0x00083902;
		data_array[1] = 0x100002b3;
		data_array[2] = 0x00000005;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB4, 7, {0X02, 0X00, 0X0F, 0X05, 0X00, 0X00, 0X00}}
		data_array[0] = 0x00083902;
		data_array[1] = 0x0f0002b4;
		data_array[2] = 0x00000005;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB6, 10, {0X00, 0X00, 0X17, 0X05, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00}}
		data_array[0] = 0x000b3902;
		data_array[1] = 0x170000b6;
		data_array[2] = 0x00000005;
		data_array[3] = 0x000000;
		dsi_set_cmdq(data_array, 4, 1);

		// {0xBA, 7, {0XC6, 0X06, 0X00, 0X0B, 0X01, 0X50, 0X65}}
		data_array[0] = 0x00083902;
		data_array[1] = 0x0006c6ba;
		data_array[2] = 0x6550010b;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xBB, 7, {0XC6, 0X06, 0X00, 0X0A, 0X01, 0X50, 0X65}}
		data_array[0] = 0x00083902;
		data_array[1] = 0x0006c6bb;
		data_array[2] = 0x6550010a;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xC4, 2, {0x10, 0x00}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x000010c4;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xC5, 2, {0x00, 0x00}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x000000c5;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x05}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x52aa55f0;
		data_array[2] = 0x0508;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB0, 4, {0X17, 0X03, 0X00, 0X03}}
		data_array[0] = 0x00053902;
		data_array[1] = 0x000317b0;
		data_array[2] = 0x03;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xD0, 6, {0X00, 0X48, 0X01, 0X00, 0X00, 0X00}}
		data_array[0] = 0x00073902;
		data_array[1] = 0x014800d0;
		data_array[2] = 0x000000;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xD1, 6, {0X00, 0X48, 0X02, 0X00, 0X00, 0X00}}
		data_array[0] = 0x00073902;
		data_array[1] = 0x024800d1;
		data_array[2] = 0x000000;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB2, 3, {0X03, 0X01, 0X00}}
		data_array[0] = 0x00043902;
		data_array[1] = 0x000103b2;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xB3, 4, {0X82, 0X23, 0X02, 0X96}}
		data_array[0] = 0x00053902;
		data_array[1] = 0x022382b3;
		data_array[2] = 0x96;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xE7, 1, {0XA9}}
		data_array[0] = 0xa9e71500;
		dsi_set_cmdq(data_array, 1, 1);

		// {0xE8, 2, {0X11, 0XFF}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x00ff11e8;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xEE, 2, {0XAA, 0X00}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x0000aaee;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x06}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x52aa55f0;
		data_array[2] = 0x0608;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB0, 5, {0X3D, 0X3D, 0X3D, 0X3D, 0X3D}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x3d3d3db0;
		data_array[2] = 0x3d3d;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB1, 5, {0X3D, 0X3D, 0X3D, 0X08, 0X32}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x3d3d3db1;
		data_array[2] = 0x3208;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB2, 5, {0X33, 0X10, 0X11, 0X12, 0X13}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x111033b2;
		data_array[2] = 0x1312;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB3, 5, {0X14, 0X15, 0X33, 0X03, 0X04}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x331514b3;
		data_array[2] = 0x0403;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB4, 2, {0x00, 0x3D}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x003d00b4;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xB5, 5, {0X3D, 0X3D, 0X3D, 0X3D, 0X3D}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x3d3d3db5;
		data_array[2] = 0x3d3d;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB6, 5, {0X3D, 0X3D, 0X3D, 0X09, 0X32}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x3d3d3db6;
		data_array[2] = 0x3209;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB7, 5, {0X33, 0X16, 0X17, 0X18, 0X19}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x171633b7;
		data_array[2] = 0x1918;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB8, 5, {0X1A, 0X1B, 0X33, 0X05, 0X01}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x331b1ab8;
		data_array[2] = 0x0105;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xB9, 2, {0x02, 0x3D}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x003d02b9;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xC0, 5, {0X3D, 0X3D, 0X3D, 0X3D, 0X3D}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x3d3d3dc0;
		data_array[2] = 0x3d3d;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xC1, 5, {0X3D, 0X3D, 0X3D, 0X09, 0X33}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x3d3d3dc1;
		data_array[2] = 0x3309;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xC2, 5, {0X32, 0X19, 0X18, 0X17, 0X16}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x181932c2;
		data_array[2] = 0x1617;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xC3, 5, {0X1B, 0X1A, 0X33, 0X02, 0X01}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x331a1bc3;
		data_array[2] = 0x0102;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xC4, 2, {0x05, 0x3D}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x003d05c4;
		dsi_set_cmdq(data_array, 2, 1);

		// {0xC5, 5, {0X3D, 0X3D, 0X3D, 0X3D, 0X3D}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x3d3d3dc5;
		data_array[2] = 0x3d3d;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xC6, 5, {0X3D, 0X3D, 0X3D, 0X08, 0X33}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x3d3d3dc6;
		data_array[2] = 0x3308;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xC7, 5, {0X32, 0X13, 0X12, 0X11, 0X10}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x121332c7;
		data_array[2] = 0x1011;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xC8, 5, {0X15, 0X14, 0X33, 0X00, 0X04}}
		data_array[0] = 0x00063902;
		data_array[1] = 0x331415c8;
		data_array[2] = 0x0400;
		dsi_set_cmdq(data_array, 3, 1);

		// {0xC9, 2, {0x03, 0x3D}}
		data_array[0] = 0x00033902;
		data_array[1] = 0x003d03c9;
		dsi_set_cmdq(data_array, 2, 1);
	}

	// {0x35, 1, {0x00}}
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	// {0x51, 1, {0xFF}}
	data_array[0] = 0xff511500;
	dsi_set_cmdq(data_array, 1, 1);

	// {0x53, 1, {0x2C}}
	data_array[0] = 0x2c531500;
	dsi_set_cmdq(data_array, 1, 1);

	// {0x55, 1, {0x00}}
	data_array[0] = 0x00551500;
	dsi_set_cmdq(data_array, 1, 1);

	// {0x11, 0, {} } /* sleep out */
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(120);

	// {0x29, 0, {} } /* display on */
	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(20);

#endif

	   pr_warn("%s-\n", __func__);
}

static void lcm_set_backlight(unsigned int level)
{
	   pr_warn("%s: level=%d\n", __func__, level);
}

LCM_DRIVER nt35523_TIANMA_qxga_dsi_vdo_truly_lcm_drv = {
	   .name = "nt35523_TIANMA_qxga_dsi_vdo_truly",
	   .set_util_funcs = lcm_set_util_funcs,
	   .get_params = lcm_get_params,
	   .init = lcm_init,
	   .suspend = lcm_suspend,
	   .resume = lcm_resume,
	   .set_backlight = lcm_set_backlight,
	   /*.compare_id     = lcm_compare_id, */
	   .init_power = lcm_init_power,
	   .resume_power = lcm_resume_power,
	   .suspend_power = lcm_suspend_power,
};
