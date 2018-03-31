
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include "mt_charging.h"
#include "mt_battery_common.h"
#include <linux/usb/tcpm.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
/* for mtk battery driver */
#include "../../../power/mt81xx/mt_charging.h"


static struct notifier_block type_c_nb;
static struct platform_device *plat_dev;
static struct tcpc_device *tcpc_dev;
static int tcpc_pd_state;
static int tcpc_power_role;
static int tcpc_sink_voltage;
static int tcpc_sink_current;
static int tcpc_typec_state;
static int tcpc_pre_typec_state;
static bool tcpc_src_support_pd = false;
static struct regulator *vconn_reg;
static struct delayed_work tcpc_unlock_work;

#ifndef PD_CHARGING_DRIVER_SUPPORT
void smb1351_suspend_enable(bool enable) {};
bool smb1351_is_battery_low(void) {return 0; };
#endif

extern int tcpci_report_charger_attached(struct tcpc_device *tcpc);

static const char *event_string(int event)
{
	switch (event) {
	case TCP_NOTIFY_SOURCE_VCONN:
		return "TCP_NOTIFY_SOURCE_VCONN";
	case TCP_NOTIFY_SOURCE_VBUS:
		return "TCP_NOTIFY_SOURCE_VBUS";
	case TCP_NOTIFY_SINK_VBUS:
		return "TCP_NOTIFY_SINK_VBUS";
	case TCP_NOTIFY_PR_SWAP:
		return "TCP_NOTIFY_PR_SWAP";
	case TCP_NOTIFY_DR_SWAP:
		return "TCP_NOTIFY_DR_SWAP";
	case TCP_NOTIFY_VCONN_SWAP:
		return "TCP_NOTIFY_VCONN_SWAP";
	case TCP_NOTIFY_AMA_DP_STATE:
		return "TCP_NOTIFY_AMA_DP_STATE";
	case TCP_NOTIFY_AMA_DP_ATTENTION:
		return "TCP_NOTIFY_AMA_DP_ATTENTION";
	case TCP_NOTIFY_AMA_DP_HPD_STATE:
		return "TCP_NOTIFY_AMA_DP_HPD_STATE";
	case TCP_NOTIFY_TYPEC_STATE:
		return "TCP_NOTIFY_TYPEC_STATE";
	case TCP_NOTIFY_PD_STATE:
		return "TCP_NOTIFY_PD_STATE";
	default:
		return "UNKNOW";
	}
}

static const char *charger_string(int chg_type)
{
	switch (chg_type) {
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
		return "UNKNOW";
	}
}

static void asus_tcpc_wake_unlock_work(struct work_struct *w) {

	pr_debug("%s: Charger Type: %s\n", __func__, charger_string(BMT_status.charger_type));

	if ((BMT_status.charger_type != CHARGER_UNKNOWN)
	&& (BMT_status.charger_type != STANDARD_HOST)
	&& (BMT_status.charger_type != CHARGING_HOST)) {
		pr_info("%s: wakelock unlock, charger type: %s\n", __func__, charger_string(BMT_status.charger_type));
		tcpci_report_charger_attached(tcpc_dev);
	}
}

bool mt_usb_pd_support(void)
{
	return true;
}

bool mt_is_power_sink(void)
{
	if (tcpc_power_role == PD_ROLE_SINK)
		return true;
	else
		return false;
}

int usbpd_getvoltagecurrent(int select)
{
	int ret = 0;

	switch (select) {
	case USBPD_VOLTAGE: /* mV */
		ret = tcpc_sink_voltage;
		pr_info("%s: USBPD_VOLTAGE = %d mV\n", __func__, ret);
		return ret;
	case USBPD_CURRENT: /* mA */
		ret = tcpc_sink_current;
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
	pr_info("%s: PD jump voltage\n", __func__);
	tcpm_soft_reset(tcpc_dev);
}
EXPORT_SYMBOL(usbpd_jump_voltage);

/* unit: mA */
int mt_usb_pd_get_current(void)
{
	return tcpc_sink_current;
}

static int rt_chg_handle_source_vbus(struct tcp_notify *tcp_noti)
{
	bool enable = (tcp_noti->vbus_state.mv > 0) ? true : false;

	/* if vbus boost comes from charger ic */
	bat_charger_boost_enable(enable);

	tcpm_notify_vbus_stable(tcpc_dev);
	return 0;
}

static int rt_chg_handle_sink_vbus(struct tcp_notify *tcp_noti)
{
	tcpc_sink_voltage = tcp_noti->vbus_state.mv;
	tcpc_sink_current = tcp_noti->vbus_state.ma;
	return 0;
}

static int chg_tcp_notifer_call(struct notifier_block *nb, unsigned long event, void *data)
{
	struct tcp_notify *tcp_noti = data;
	int ret = 0;

	pr_info("%s: %s\n", __func__, event_string(event));

	switch (event) {
	case TCP_NOTIFY_PR_SWAP:
		tcpc_power_role = tcp_noti->swap_state.new_role;
		tcpc_src_support_pd = true;
		break;
	case TCP_NOTIFY_DR_SWAP:
		pr_debug("dr swap: %d\n", tcp_noti->swap_state.new_role);
		break;
	case TCP_NOTIFY_SOURCE_VCONN:
		if (tcp_noti->en_state.en && !regulator_is_enabled(vconn_reg)) {
			ret = regulator_enable(vconn_reg);
			if (ret != 0)
				pr_err("%s: turn on vconn fail!\n", __func__);
		} else if (!tcp_noti->en_state.en && regulator_is_enabled(vconn_reg)) {
			ret = regulator_disable(vconn_reg);
			if (ret != 0)
				pr_err("%s: turn off vconn fail!\n", __func__);
		}
		break;
	case TCP_NOTIFY_VCONN_SWAP:
		if (tcp_noti->swap_state.new_role == PD_ROLE_VCONN_ON && !regulator_is_enabled(vconn_reg)) {
			ret = regulator_enable(vconn_reg);
			if (ret != 0)
				pr_err("%s: turn on vconn fail!\n", __func__);
		} else if (tcp_noti->swap_state.new_role == PD_ROLE_VCONN_OFF && regulator_is_enabled(vconn_reg)) {
			ret = regulator_disable(vconn_reg);
			if (ret != 0)
				pr_err("%s: turn off vconn fail!\n", __func__);
		}

		break;
	case TCP_NOTIFY_SOURCE_VBUS:
		rt_chg_handle_source_vbus(tcp_noti);
		break;
	case TCP_NOTIFY_SINK_VBUS:
		rt_chg_handle_sink_vbus(tcp_noti);
		if (tcpc_power_role == PD_ROLE_SINK && tcpc_src_support_pd == true) {
			if (smb1351_is_battery_low()) {
				bat_update_TypeC_charger_type(TYPEC_PD_5V_CHARGER);
				pr_info("%s: TYPEC_PD_5V_CHARGER\n", __func__);
			} else {
				bat_update_TypeC_charger_type(TYPEC_PD_9V_CHARGER);
				pr_info("%s: TYPEC_PD_9V_CHARGER\n", __func__);
			}
		}
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		tcpc_pre_typec_state = tcpc_typec_state;
		tcpc_typec_state = tcp_noti->typec_state.new_state;

		if (tcpc_typec_state == TYPEC_ATTACHED_SRC) {
			tcpc_power_role = PD_ROLE_SOURCE;
			ssusb_mode_switch_typec(1);
		} else
			tcpc_power_role = PD_ROLE_SINK;

		if (tcpc_typec_state == TYPEC_UNATTACHED) {
			tcpc_src_support_pd = false;
			if (tcpc_pre_typec_state == TYPEC_ATTACHED_SRC)
				ssusb_mode_switch_typec(0);

			cancel_delayed_work_sync(&tcpc_unlock_work);
			bat_update_TypeC_charger_type(CHARGER_UNKNOWN);
			pr_info("%s: CHARGER_UNKNOWN\n", __func__);
		}
		break;
	case TCP_NOTIFY_PD_STATE:
		tcpc_pd_state = tcp_noti->pd_state.connected;

		if (tcpc_pd_state == PD_CONNECT_TYPEC_ONLY_SNK) {
			if (tcpc_sink_voltage == 5000 && tcpc_sink_current == 1500) {
				bat_update_TypeC_charger_type(TYPEC_1_5A_CHARGER);
				pr_info("%s: TYPEC_1_5A_CHARGER\n", __func__);
			} else if (tcpc_sink_voltage == 5000 && tcpc_sink_current == 3000) {
				bat_update_TypeC_charger_type(TYPEC_3A_CHARGER);
				pr_info("%s: TYPEC_3A_CHARGER\n", __func__);
			}
		} else if (tcpc_pd_state == PD_CONNECT_PE_READY_SNK) {
			tcpc_src_support_pd = true;
			if (smb1351_is_battery_low()) {
				bat_update_TypeC_charger_type(TYPEC_PD_5V_CHARGER);
				pr_info("%s: TYPEC_PD_5V_CHARGER\n", __func__);
			} else {
				bat_update_TypeC_charger_type(TYPEC_PD_9V_CHARGER);
				pr_info("%s: TYPEC_PD_9V_CHARGER\n", __func__);
			}
		} else if (tcpc_pd_state == PD_CONNECT_PE_READY_SRC)
			tcpc_src_support_pd = true;

		/* unlock cc logic wakelock when charger attached */
		if (tcpc_typec_state == TYPEC_ATTACHED_SNK) {
			pr_debug("%s: tcpc_unlock_work: wait 3 sec to check charger type\n", __func__);
			schedule_delayed_work(&tcpc_unlock_work, (3000 * HZ/1000));
		}
		break;
	default:
		break;
	};
	return NOTIFY_OK;
}


static ssize_t show_vconn_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "status: %d\n", regulator_is_enabled(vconn_reg));
}

static ssize_t store_vconn_test(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret, enable;

	if (buf != NULL) {
		ret = kstrtouint(buf, 0, &enable);
		if (ret) {
			pr_err("wrong format!\n");
			return size;
		}

		if (enable)
			ret = regulator_enable(vconn_reg);
		else
			ret = regulator_disable(vconn_reg);
	}

	return size;
}
static DEVICE_ATTR(vconn_test, S_IRUSR | S_IWUSR, show_vconn_test, store_vconn_test);


void setup_rt1711h(void)
{
	int ret;

	vconn_reg = devm_regulator_get(&plat_dev->dev, "vconn");
	if (IS_ERR(vconn_reg))
		pr_err("%s: can't get vconn regulator!\n", __func__);

	tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc_dev) {
		pr_err("%s: fail to get type_c_port0!\n", __func__);
		return;
	}
	type_c_nb.notifier_call = chg_tcp_notifer_call;
	ret = register_tcp_dev_notifier(tcpc_dev, &type_c_nb);
	if (ret < 0)
		pr_err("%s: register tcpc notifer fail!\n", __func__);
}

static int rt_typec_probe(struct platform_device *pdev)
{
	int ret = 0;

	device_create_file(&(pdev->dev), &dev_attr_vconn_test);
	plat_dev = pdev;
	setup_rt1711h();

	INIT_DELAYED_WORK(&tcpc_unlock_work, asus_tcpc_wake_unlock_work);
	return ret;
}

static int rt_typec_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&tcpc_unlock_work);
	return 0;
}

static const struct of_device_id rt_typec_of_match[] = {
	{ .compatible = "mediatek,rt-typec" },
	{ }
};
MODULE_DEVICE_TABLE(of, rt_typec_of_match);

static struct platform_driver rt_typec_driver = {
	.probe = rt_typec_probe,
	.remove = rt_typec_remove,
	.driver = {
		.name = "rt-typec",
		.of_match_table = of_match_ptr(rt_typec_of_match),
	},
};

static int __init rt_typec_init(void)
{
	int ret;

	ret = platform_driver_register(&rt_typec_driver);
	if (ret)
		pr_err("%s: unable to register driver (%d)\n", __func__, ret);

	return ret;
}

static void __exit rt_typec_exit(void)
{
}

late_initcall_sync(rt_typec_init);
module_exit(rt_typec_exit);


MODULE_AUTHOR("MengHui Lin");
MODULE_DESCRIPTION("rt typec client driver");
MODULE_LICENSE("GPL");

