/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "anx_ohio_driver.h"
#include "anx_ohio_private_interface.h"
#include "anx_ohio_public_interface.h"
#include "ssusb.h"
#include "mt_charging.h"
#include "mt_battery_common.h"


/* Use device tree structure data when defined "CONFIG_OF"  */
/* #define CONFIG_OF */

static int create_sysfs_interfaces(struct device *dev);

/* to access global platform data */
static struct ohio_platform_data *g_pdata;

#define DONGLE_CABLE_INSERT  1

struct i2c_client *ohio_client;

struct ohio_platform_data {
	int gpio_p_on;
	int gpio_reset;
	int gpio_cbl_det;
	int gpio_intr_comm;
	int gpio_vconn_en;
	int connect_mode;
	int cbl_det_irq;
#ifdef SUP_VDD33_CTL
	int gpio_v33_ctrl;
#endif
	struct regulator *dvdd;
	struct regulator *avdd;
	spinlock_t lock;
};

struct ohio_data {
	struct ohio_platform_data *pdata;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	struct wake_lock ohio_lock;
};

/* ohio power status, sync with interface and cable detection thread */

inline unsigned char OhioReadReg(unsigned char RegAddr)
{
	int ret = 0;

	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_read_byte_data(ohio_client, RegAddr);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
	}
	return (uint8_t) ret;

}

inline int OhioReadBlockReg(u8 RegAddr, u8 len, u8 *dat)
{
	int ret = 0;

	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_read_i2c_block_data(ohio_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
		return -EPERM;
	}

	return (int)ret;
}


inline int OhioWriteBlockReg(u8 RegAddr, u8 len, const u8 *dat)
{
	int ret = 0;

	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_write_i2c_block_data(ohio_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
		return -EPERM;
	}

	return (int)ret;
}

inline void OhioWriteReg(unsigned char RegAddr, unsigned char RegVal)
{
	int ret = 0;

	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_write_byte_data(ohio_client, RegAddr, RegVal);
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
	}
}

void ohio_power_standby(void)
{
#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif

	gpio_set_value(pdata->gpio_reset, 0);
	mdelay(1);
	gpio_set_value(pdata->gpio_p_on, 0);
	mdelay(1);

	pr_info("%s %s: Ohio power down\n", LOG_TAG, __func__);
}

void ohio_hardware_poweron(void)
{
#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif
	int retry_count, i;

	pr_info("%s %s: Ohio power on\n", LOG_TAG, __func__);

	for (retry_count = 0; retry_count < 3; retry_count++) {
#ifdef OHIO_DEBUG
		pr_info("%s %s: Ohio check ocm loading...\n", LOG_TAG, __func__);
#endif
		/*power on pin enable */
		gpio_set_value(pdata->gpio_p_on, 1);
		mdelay(10);

		/*power reset pin enable */
		gpio_set_value(pdata->gpio_reset, 1);
		mdelay(10);

		/* load delay T3 : eeprom 3.2s,  OTP 20ms */
		for (i = 0; i < OHIO_OCM_LOADING_TIME; i++) {
			/*Interface work? */
			if (OhioReadReg(0x16) == 0x80) {
#ifdef OHIO_DEBUG
				pr_info("%s %s: interface initialization\n", LOG_TAG, __func__);
#endif
				interface_init();
				send_initialized_setting();
#ifdef SUP_OHIO_INT_VECTOR
				/* open interrupt vector */
				OhioWriteReg(OHIO_INTERFACE_INTR_MASK, 0);
#endif
#ifdef OHIO_DEBUG
				if (OhioReadReg(0x7F) == 0x01)
					pr_info
					    ("%s %s: OTP chip is power on! firmware version is 0x%x\n",
					     LOG_TAG, __func__, OhioReadReg(0x44));
				else
					pr_info
					    ("%s %s: EEPROM chip is power on! firmware version is 0x%x\n",
					     LOG_TAG, __func__, OhioReadReg(0x44));
#endif
				return;
			}
			mdelay(1);
			pr_debug(".");
		}
		ohio_power_standby();
		mdelay(10);
	}

}

static void ohio_free_gpio(struct ohio_data *ohio)
{
#ifdef SUP_VDD33_CTL
	gpio_free(ohio->pdata->gpio_v33_ctrl);
#endif
	gpio_free(ohio->pdata->gpio_cbl_det);
	gpio_free(ohio->pdata->gpio_reset);
	gpio_free(ohio->pdata->gpio_p_on);
	gpio_free(ohio->pdata->gpio_intr_comm);
}

static int ohio_init_gpio(struct ohio_data *ohio)
{
	int ret = 0;

	pr_info("%s %s: ohio init gpio\n", LOG_TAG, __func__);
	/*  gpio for chip power down  */
	ret = gpio_request(ohio->pdata->gpio_p_on, "ohio_p_on_ctl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__, ohio->pdata->gpio_p_on);
		goto err0;
	}
	gpio_direction_output(ohio->pdata->gpio_p_on, 0);
	/*  gpio for chip reset  */
	ret = gpio_request(ohio->pdata->gpio_reset, "ohio_reset_n");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__, ohio->pdata->gpio_reset);
		goto err1;
	}
	gpio_direction_output(ohio->pdata->gpio_reset, 0);

	/*  gpio for ohio cable detect  */
	ret = gpio_request(ohio->pdata->gpio_cbl_det, "ohio_cbl_det");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__, ohio->pdata->gpio_cbl_det);
		goto err2;
	}
	gpio_direction_input(ohio->pdata->gpio_cbl_det);
	/*  gpio for chip interface communaction */
	ret = gpio_request(ohio->pdata->gpio_intr_comm, "ohio_intr_comm");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__, ohio->pdata->gpio_intr_comm);
		goto err3;
	}
	gpio_direction_input(ohio->pdata->gpio_intr_comm);

#ifdef SUP_VDD33_CTL
	/*  gpio for chip standby control DVDD33 */
	ret = gpio_request(ohio->pdata->gpio_v33_ctrl, "ohio_v33_ctrl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__, ohio->pdata->gpio_v33_ctrl);
		goto err4;
	}
	gpio_direction_output(ohio->pdata->gpio_v33_ctrl, 0);
err4:
	gpio_free(ohio->pdata->gpio_v33_ctrl);
#endif
	/*  gpio for vconn enable */
	ret = gpio_request(ohio->pdata->gpio_vconn_en, "ohio_vconn_en");
	if (ret) {
		pr_info("[Ohio]%s : failed to request gpio %d ret=%d\n", __func__,
			ohio->pdata->gpio_vconn_en, ret);
		goto err1;
	}
	gpio_direction_output(ohio->pdata->gpio_vconn_en, 1);
	pr_info("[Ohio]%s : vconn is pulled high\n", __func__);
	pr_info("ohio init gpio successfully\n");

	goto out;

err3:
	gpio_free(ohio->pdata->gpio_intr_comm);
err2:
	gpio_free(ohio->pdata->gpio_cbl_det);
err1:
	gpio_free(ohio->pdata->gpio_reset);
err0:
	gpio_free(ohio->pdata->gpio_p_on);

	return 1;
out:
	return 0;
}

void ohio_CC_Disconnect(void)
{
	mt_update_power_role(0);

	if (g_pdata->connect_mode == TYPEC_DFP) {
		pr_err("Ohio UFP is disconnect!\n");
		ssusb_mode_switch_typec(0);
	} else if (g_pdata->connect_mode == TYPEC_UFP) {
		pr_err("Ohio as UFP is disconnect!\n");
	} else if ((g_pdata->connect_mode == TYPEC_CHARGE_15)
		   || (g_pdata->connect_mode == TYPEC_CHARGE_30)) {
		pr_err("Ohio as charge is disconnect!\n");
	}

	g_pdata->connect_mode = -1;
}

void ohio_CC_Detect(void)
{
	u8 powerDownControl = 0, analogStatus = 0, analogCtrl7 = 0;

	powerDownControl = OhioReadReg(POWER_DOWN_CONTROL);
	analogStatus = OhioReadReg(ANALOG_STATUS);
	analogCtrl7 = OhioReadReg(ANALOG_CTRL_7);
	pr_err("ohio %s(%d): 0x48=0x%x 0x41=0x%x 0xd=0x%x 0x40=0x%x\n", __func__,
	       __LINE__, analogCtrl7, OhioReadReg(0x41), powerDownControl, analogStatus);

	if ((analogStatus & DFP_OR_UFP) == 0x0) {
		pr_err("Ohio UFP is detected!\n");
		g_pdata->connect_mode = TYPEC_DFP;
		mt_update_power_role(1);
		ssusb_mode_switch_typec(1);
	} else if ((analogCtrl7 & CC1_5P1K) && (analogCtrl7 & CC2_5P1K)
		   && !(analogCtrl7 & CC1_RA) && !(analogCtrl7 & CC2_RA)) {
		pr_err("Ohio Debug Accessory mode attached!\n");
		g_pdata->connect_mode = TYPEC_DEBUG_MODE;
	} else if ((analogCtrl7 & CC1_RA) && (analogCtrl7 & CC2_RA)) {
		pr_err("Ohio Audio adapter accessory mode attached!\n");
		g_pdata->connect_mode = TYPEC_AUDIO_MODE;
	} else if (((analogCtrl7 & CC1_RA) && (analogCtrl7 & CC2_5P1K))
		   || ((analogCtrl7 & CC2_RA) && (analogCtrl7 & CC1_5P1K))) {
		pr_err("Ohio Power cable+UFP  attached!\n");
	} else if ((analogCtrl7 & CC1_RA) || (analogCtrl7 & CC2_RA)) {
		pr_err("Ohio Power cable/No UFP attached !\n");
	}
	if ((analogStatus & DFP_OR_UFP) && (analogStatus & UFP_PLUG)) {
		if (powerDownControl & (CC1_VRD_USB | CC2_VRD_USB)) {
			pr_err("Ohio as Normal UFP connected\n ");
			g_pdata->connect_mode = TYPEC_UFP;
			mt_update_power_role(0);
			wake_up_bat();
		} else if (powerDownControl & (CC1_VRD_1P5 | CC2_VRD_1P5)) {
			pr_err("Ohio  1.5A charge is connected\n ");
			g_pdata->connect_mode = TYPEC_CHARGE_15;
			mt_update_power_role(0);
			bat_update_charger_type(TYPEC_1_5A_CHARGER);
		}
		if (powerDownControl & (CC1_VRD_3P0 | CC2_VRD_3P0)) {
			pr_err("Ohio  3.0A charge is connected\n ");
			g_pdata->connect_mode = TYPEC_CHARGE_30;
			mt_update_power_role(0);
			bat_update_charger_type(TYPEC_3A_CHARGER);
		}
	}
}

void ohio_main_process(void)
{
	/* do main loop, do what you want to do */
}

#ifdef CABLE_DET_PIN_HAS_GLITCH
static unsigned char confirmed_cable_det(void *data)
{
	struct ohio_data *anxohio = data;
	unsigned int count = 9;
	unsigned int cable_det_count = 0;
	u8 val = 0;

	do {
		val = gpio_get_value(anxohio->pdata->gpio_cbl_det);
		if (DONGLE_CABLE_INSERT == val)
			cable_det_count++;
		mdelay(1);
	} while (count--);

	if (cable_det_count > 7)
		return 1;
	else if (cable_det_count < 3)
		return 0;
	else
		return atomic_read(&ohio_power_status);
}
#endif

static irqreturn_t ohio_cbl_det_isr(int irq, void *data)
{
	struct ohio_data *ohio = data;
	int cable_connected = 0;

#ifdef CABLE_DET_PIN_HAS_GLITCH
	cable_connected = confirmed_cable_det((void *)ohio);
#else
	cable_connected = gpio_get_value(ohio->pdata->gpio_cbl_det);
#endif

	pr_notice("%s %s : cable plug %d\n", LOG_TAG, __func__, cable_connected);

	if (cable_connected == DONGLE_CABLE_INSERT) {
		wake_lock(&ohio->ohio_lock);
		irq_set_irq_type(ohio->pdata->cbl_det_irq, IRQF_TRIGGER_LOW);
		gpio_set_value(ohio->pdata->gpio_vconn_en, 1);
		pr_info("%s %s : detect cable insertion\n", LOG_TAG, __func__);
		if (atomic_read(&ohio_power_status) == 1) {
#ifdef CABLE_DET_PIN_HAS_GLITCH
			mdelay(2);
			ohio_power_standby();
#else
			return IRQ_HANDLED;
#endif
		}
		atomic_set(&ohio_power_status, 1);
		ohio_hardware_poweron();
		ohio_CC_Detect();
	} else {
		irq_set_irq_type(ohio->pdata->cbl_det_irq, IRQF_TRIGGER_HIGH);
		bat_update_charger_type(CHARGER_UNKNOWN);
		ohio_CC_Disconnect();
		gpio_set_value(ohio->pdata->gpio_vconn_en, 0);

		pr_info("%s %s : cable unplug\n", LOG_TAG, __func__);
		atomic_set(&ohio_power_status, 0);
		ohio_power_standby();
		wake_lock_timeout(&ohio->ohio_lock, HZ / 2);
	}

	return IRQ_HANDLED;
}

static irqreturn_t ohio_intr_comm_isr(int irq, void *data)
{
	if (atomic_read(&ohio_power_status) != 1)
		return IRQ_NONE;

	if (is_soft_reset_intr()) {
#ifdef OHIO_DEBUG
		pr_info("%s %s : ======I=====\n", LOG_TAG, __func__);
#endif

#ifdef SUP_OHIO_INT_VECTOR
		handle_intr_vector();
#else
		polling_interface_msg(INTERACE_TIMEOUT_MS);
		clear_soft_interrupt();
#endif

	}
	return IRQ_HANDLED;
}

static void ohio_work_func(struct work_struct *work)
{
	struct ohio_data *td = container_of(work, struct ohio_data,
					    work.work);
	int workqueu_timer = 0;

	workqueu_timer = 1;
	mutex_lock(&td->lock);
	ohio_main_process();
	mutex_unlock(&td->lock);
	queue_delayed_work(td->workqueue, &td->work, msecs_to_jiffies(workqueu_timer));
}

#ifdef CONFIG_OF
int ohio_regulator_configure(struct device *dev, struct ohio_platform_data *pdata)
{
	int ret = 0;

	pdata->dvdd = devm_regulator_get(dev, "reg-dvddio");
	pdata->avdd = devm_regulator_get(dev, "reg-avdd33");

	if (!IS_ERR(pdata->dvdd)) {

		ret = regulator_set_voltage(pdata->dvdd, 1800000, 1800000);
		if (ret != 0)
			dev_err(dev, "Fail to set 1.8V to reg-dvddio: %d\n", ret);

		ret = regulator_get_voltage(pdata->dvdd);
		pr_warn("dvdd voltage: %d\n", ret);

		ret = regulator_enable(pdata->dvdd);
		if (ret != 0)
			dev_err(dev, "Fail to enable reg-dvddio: %d\n", ret);
	} else {
		dev_err(dev, "fail to get ohio dvddio regulator!\n");
		return -1;
	}


	if (!IS_ERR(pdata->avdd)) {

		ret = regulator_set_voltage(pdata->avdd, 3300000, 3300000);
		if (ret != 0)
			dev_err(dev, "Fail to set 3.3V to reg-avdd33: %d\n", ret);

		ret = regulator_get_voltage(pdata->avdd);
		pr_warn("avdd33 voltage: %d\n", ret);

		ret = regulator_enable(pdata->avdd);
		if (ret != 0)
			dev_err(dev, "Fail to enable reg-avdd33: %d\n", ret);
	} else {
		dev_err(dev, "fail to get ohio avdd33 regulator!\n");
		return -1;
	}

	return 0;
}

static int ohio_parse_dt(struct device *dev, struct ohio_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->gpio_p_on = of_get_named_gpio_flags(np, "analogix,p-on-gpio", 0, NULL);

	pdata->gpio_reset = of_get_named_gpio_flags(np, "analogix,reset-gpio", 0, NULL);

	pdata->gpio_cbl_det = of_get_named_gpio_flags(np, "analogix,cbl-det-gpio", 0, NULL);

#ifdef SUP_VDD33_CTL
	pdata->gpio_v33_ctrl = of_get_named_gpio_flags(np, "analogix,v33-ctrl-gpio", 0, NULL);
#endif

	pdata->gpio_intr_comm = of_get_named_gpio_flags(np, "analogix,intr-comm-gpio", 0, NULL);

	pdata->gpio_vconn_en = of_get_named_gpio_flags(np, "analogix,vconn-en-gpio", 0, NULL);


	pr_info("%s gpio p_on : %d, reset : %d,  gpio_cbl_det %d vconn_en %d\n",
		LOG_TAG, pdata->gpio_p_on,
		pdata->gpio_reset, pdata->gpio_cbl_det, pdata->gpio_vconn_en);

	return 0;
}
#else
static int ohio_parse_dt(struct device *dev, struct ohio_platform_data *pdata)
{
	return -ENODEV;
}
#endif
static int ohio_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct ohio_data *ohio;
	struct ohio_platform_data *pdata;
	int ret = 0;
	int cbl_det_irq = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("%s:ohio's i2c bus doesn't support\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	ohio = kzalloc(sizeof(struct ohio_data), GFP_KERNEL);
	if (!ohio) {
		/*pr_err("%s: failed to allocate driver data\n", __func__);*/
		ret = -ENOMEM;
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(struct ohio_platform_data), GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;

		/* device tree parsing function call */
		ret = ohio_parse_dt(&client->dev, pdata);
		if (ret != 0)	/* if occurs error */
			goto err0;

		ohio->pdata = pdata;
	} else {
		ohio->pdata = client->dev.platform_data;
	}

	ohio_regulator_configure(&client->dev, client->dev.platform_data);
	mt_init_pd_policy();

	/* to access global platform data */
	g_pdata = ohio->pdata;
	ohio_client = client;
	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);

	atomic_set(&ohio_power_status, 0);

	mutex_init(&ohio->lock);

	if (!ohio->pdata) {
		ret = -EINVAL;
		goto err0;
	}

	ret = ohio_init_gpio(ohio);
	if (ret) {
		pr_err("%s: failed to initialize gpio\n", __func__);
		goto err0;
	}

	INIT_DELAYED_WORK(&ohio->work, ohio_work_func);

	ohio->workqueue = create_singlethread_workqueue("ohio_work");
	if (ohio->workqueue == NULL) {
		pr_err("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}
#if 0
	cbl_det_irq = gpio_to_irq(ohio->pdata->gpio_cbl_det);
	if (cbl_det_irq < 0) {
		pr_err("%s : failed to get gpio irq\n", __func__);
		goto err1;
	}
#else
	cbl_det_irq = irq_of_parse_and_map(client->dev.of_node, 0);
	ohio->pdata->cbl_det_irq = cbl_det_irq;
	if (!cbl_det_irq) {
		pr_err("%s : failed to get cbl gpio irq\n", __func__);
		goto err1;
	}
#endif


	wake_lock_init(&ohio->ohio_lock, WAKE_LOCK_SUSPEND, "ohio_wake_lock");

	irq_set_status_flags(cbl_det_irq, IRQ_NOAUTOEN);

	ret = request_threaded_irq(cbl_det_irq, NULL, ohio_cbl_det_isr,
				   IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "ohio-cbl-det", ohio);
	if (ret < 0) {
		pr_err("%s : failed to request irq\n", __func__);
		goto err3;
	}

	ret = irq_set_irq_wake(cbl_det_irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for cable detect", __func__);
		pr_err("interrupt wake set fail\n");
		goto err4;
	}

	ret = enable_irq_wake(cbl_det_irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for cable detect", __func__);
		pr_err("interrupt wake enable fail\n");
		goto err4;
	}
#if 0
	client->irq = gpio_to_irq(ohio->pdata->gpio_intr_comm);
	if (client->irq < 0) {
		pr_err("%s : failed to get ohio gpio comm irq\n", __func__);
		goto err3;
	}
#else
	client->irq = irq_of_parse_and_map(client->dev.of_node, 1);
	if (!client->irq) {
		pr_err("%s : failed to get ohio gpio comm irq\n", __func__);
		goto err3;
	}
#endif

	irq_set_status_flags(client->irq, IRQ_NOAUTOEN);

	ret = request_threaded_irq(client->irq, NULL, ohio_intr_comm_isr,
				   IRQF_TRIGGER_RISING | IRQF_ONESHOT, "ohio-intr-comm", ohio);
	if (ret < 0) {
		pr_err("%s : failed to request interface irq\n", __func__);
		goto err4;
	}

	ret = irq_set_irq_wake(client->irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for interface communaction", __func__);
		goto err4;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for interface communaction", __func__);
		goto err4;
	}

	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		pr_err("%s : sysfs register failed", __func__);
		goto err4;
	}
#ifdef SUP_VDD33_CTL
	gpio_set_value(pdata->gpio_v33_ctrl, 0);
	mdelay(20);
	gpio_set_value(pdata->gpio_v33_ctrl, 1);
	mdelay(2);
#endif
	/*when probe ohio device, enter standy mode */
	ohio_power_standby();

	pr_notice("ohio_i2c_probe successfully %s %s end\n", LOG_TAG, __func__);

	/* enable irq after stanby mode */
	enable_irq(cbl_det_irq);
	enable_irq(client->irq);

	goto exit;

err4:
	free_irq(client->irq, ohio);
err3:
	free_irq(cbl_det_irq, ohio);
err1:
	ohio_free_gpio(ohio);
	destroy_workqueue(ohio->workqueue);
err0:
	ohio_client = NULL;
	kfree(ohio);
exit:
	return ret;
}

static int ohio_i2c_remove(struct i2c_client *client)
{
	struct ohio_data *ohio = i2c_get_clientdata(client);

	pr_info("ohio_i2c_remove\n");
	free_irq(client->irq, ohio);
	ohio_free_gpio(ohio);
	destroy_workqueue(ohio->workqueue);
	wake_lock_destroy(&ohio->ohio_lock);
	kfree(ohio);
	return 0;
}

static int ohio_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int ohio_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ohio_id[] = {
	{"ohio", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ohio_id);

#ifdef CONFIG_OF
static struct of_device_id anx_match_table[] = {
	{.compatible = "analogix,ohio",},
	{},
};
#endif

static struct i2c_driver ohio_driver = {
	.driver = {
		   .name = "ohio",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = anx_match_table,
#endif
		   },
	.probe = ohio_i2c_probe,
	.remove = ohio_i2c_remove,
	.suspend = ohio_i2c_suspend,
	.resume = ohio_i2c_resume,
	.id_table = ohio_id,
};

static void __init ohio_init_async(void *data, async_cookie_t cookie)
{
	int ret = 0;

	ret = i2c_add_driver(&ohio_driver);
	if (ret < 0)
		pr_err("%s: failed to register ohio i2c drivern", __func__);
}

static int __init ohio_init(void)
{
	async_schedule(ohio_init_async, NULL);
	return 0;
}

static void __exit ohio_exit(void)
{
	i2c_del_driver(&ohio_driver);
}

#ifdef OHIO_DEBUG
void dump_reg(void)
{
	int i = 0;
	u8 val = 0;

	pr_info("dump registerad:\n");
	pr_info("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
	for (i = 0; i < 256; i++) {
		val = OhioReadReg(i);

		if ((i) % 0x10 == 0x00)
			pr_info("\n[%x]:%02x ", i, val);
		else
			pr_info("%02x ", val);

	}
	pr_info("\n");
}

ssize_t anx_ohio_send_pd_cmd(struct device *dev,
			     struct device_attribute *attr, const char *buf, size_t count)
{
	int cmd;
	int result;

	result = kstrtoint(buf, 0, &cmd);
	switch (cmd) {
	case TYPE_PWR_SRC_CAP:
		send_pd_msg(TYPE_PWR_SRC_CAP, 0, 0);
		break;

	case TYPE_DP_SNK_IDENTITY:
		send_pd_msg(TYPE_DP_SNK_IDENTITY, 0, 0);
		break;

	case TYPE_PSWAP_REQ:
		send_pd_msg(TYPE_PSWAP_REQ, 0, 0);
		break;
	case TYPE_DSWAP_REQ:
		send_pd_msg(TYPE_DSWAP_REQ, 0, 0);
		break;

	case TYPE_GOTO_MIN_REQ:
		send_pd_msg(TYPE_GOTO_MIN_REQ, 0, 0);
		break;

	case TYPE_PWR_OBJ_REQ:
		interface_send_request();
		break;
	case TYPE_ACCEPT:
		interface_send_accept();
		break;
	case TYPE_REJECT:
		interface_send_reject();
		break;
	case TYPE_SOFT_RST:
		send_pd_msg(TYPE_SOFT_RST, 0, 0);
		break;
	case TYPE_HARD_RST:
		send_pd_msg(TYPE_HARD_RST, 0, 0);
		break;

	case 0xFD:
		pr_info("fetch powerrole: %d\n", get_power_role());
		break;
	case 0xFE:
		pr_info("fetch datarole: %d\n", get_data_role());
		break;

	case 0xff:
		dump_reg();
		break;
	}
	return count;
}

ssize_t anx_ohio_get_data_role(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_data_role());
}

ssize_t anx_ohio_get_power_role(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_power_role());
}

ssize_t anx_ohio_rd_reg(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	int cmd;
	int result;

	result = kstrtoint(buf, 0, &cmd);
	pr_info("reg[%x] = %x\n", cmd, OhioReadReg(cmd));

	return count;

}

ssize_t anx_ohio_wr_reg(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	int cmd, val;
	int result;

	result = sscanf(buf, "%d  %d", &cmd, &val);
	pr_info("c %x val %x\n", cmd, val);
	OhioWriteReg(cmd, val);
	pr_info("reg[%x] = %x\n", cmd, OhioReadReg(cmd));
	return count;
}

ssize_t anx_ohio_dump_register(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;

	for (i = 0; i < 256; i++) {
		pr_info("%x", OhioReadReg(i));
		if (i % 0x10 == 0)
			pr_info("\n");

		snprintf(&buf[i], sizeof(u8), "%d", OhioReadReg(i));
	}

	pr_info("\n");

	return i;
}

ssize_t anx_ohio_select_rdo_index(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t count)
{
	int cmd;

	cmd = kstrtoint(buf, 0, &cmd);
	if (cmd <= 0)
		return 0;

	pr_info("NewRDO idx %d, Old idx %d\n", cmd, sel_voltage_pdo_index);
	sel_voltage_pdo_index = cmd;
	return count;
}

ssize_t anx_ohio_update_ocm(struct device *dev,
			    struct device_attribute *attr, const char *buf, size_t count)
{
	int ver, block_write;
	int result;
	struct ohio_platform_data *pdata = g_pdata;

	result = sscanf(buf, "%x %d", &ver, &block_write);
	pr_info("try to update fw 0x%x with block_write(%d)\n", ver, block_write);

	gpio_set_value(pdata->gpio_p_on, 1);
	mdelay(100);
	gpio_set_value(pdata->gpio_reset, 1);
	mdelay(100);

	if (block_write == 1 || block_write == 0)
		anx7418_update(ver, block_write);

	return count;
}

/* for debugging */
static struct device_attribute anx_ohio_device_attrs[] = {
	__ATTR(pdcmd, S_IWUSR, NULL,
	       anx_ohio_send_pd_cmd),
	__ATTR(rdreg, S_IWUSR, NULL,
	       anx_ohio_rd_reg),
	__ATTR(wrreg, S_IWUSR, NULL,
	       anx_ohio_wr_reg),
	__ATTR(rdoidx, S_IWUSR, NULL,
	       anx_ohio_wr_reg),
	__ATTR(update_ocm, S_IWUSR, NULL,
	       anx_ohio_update_ocm),
	__ATTR(dumpreg, S_IRUGO, anx_ohio_dump_register,
	       NULL),
	__ATTR(prole, S_IRUGO, anx_ohio_get_power_role,
	       NULL),
	__ATTR(drole, S_IRUGO, anx_ohio_get_data_role,
	       NULL)
};
#else
static struct device_attribute anx_ohio_device_attrs[] = { };
#endif

static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	pr_info("ohio create system fs interface ...\n");
	for (i = 0; i < ARRAY_SIZE(anx_ohio_device_attrs); i++)
		if (device_create_file(dev, &anx_ohio_device_attrs[i]))
			goto error;
	pr_info("success\n");
	return 0;
error:

	for (; i >= 0; i--)
		device_remove_file(dev, &anx_ohio_device_attrs[i]);
	pr_err("%s %s: ohio Unable to create interface", LOG_TAG, __func__);
	return -EINVAL;
}
module_init(ohio_init);
module_exit(ohio_exit);

MODULE_DESCRIPTION("USB PD Ohio driver");
MODULE_AUTHOR("Xia Junhua <jxia@analogixsemi.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.6");
