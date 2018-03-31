#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif

#include "nt35523_TIANMA_qxga_dsi_vdo_truly.h"

static struct regulator *vdd_1v8_reg = NULL;

void lcm_request_gpio_control(struct device *dev)
{
	int ret;

	printk("%s++\n", __func__);

	GPIO_TPS65132_EN_P = of_get_named_gpio(dev->of_node, "gpio_lcm_pos_pwr_en", 0);
	GPIO_TPS65132_EN_N = of_get_named_gpio(dev->of_node, "gpio_lcm_neg_pwr_en", 0);
	GPIO_LCD_RST_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_rst_en", 0);
	GPIO_BL_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_led_en", 0);

	gpio_request(GPIO_TPS65132_EN_P, "GPIO_LCD_PWR_EN");
	pr_notice("[KE/LCM] GPIO_LCD_PWR_EN = 0x%x\n", GPIO_TPS65132_EN_P);

	gpio_request(GPIO_TPS65132_EN_N, "GPIO_LCD_PWR_EN");
	pr_notice("[KE/LCM] GPIO_LCD_PWR_EN = 0x%x\n", GPIO_TPS65132_EN_N);

	gpio_request(GPIO_LCD_RST_EN, "GPIO_LCD_RST_EN");
	pr_notice("[KE/LCM] GPIO_LCD_RST_EN = 0x%x\n", GPIO_LCD_RST_EN);

	gpio_request(GPIO_BL_EN, "GPIO_LCD_LED_EN");
	pr_notice("[KE/LCM] GPIO_LCD_LED_EN = 0x%x\n", GPIO_BL_EN);

	vdd_1v8_reg = regulator_get(dev, "vdd-1v8");
	if (vdd_1v8_reg == NULL) {
		pr_err("%s: Failed to get reg-vgp4\n", __func__);
		return;
	}

	ret = regulator_set_voltage(vdd_1v8_reg, 1800000, 1800000);
	if (ret)
		pr_err("%s: Failed to set reg-vgp4 voltage\n", __func__);

	ret = regulator_enable(vdd_1v8_reg);
	if (ret)
		pr_err("%s: Failed to enable reg-vgp4\n", __func__);

	printk("%s--\n", __func__);
}

static int lcm_probe(struct device *dev)
{
	lcm_request_gpio_control(dev);

	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,mt8173-lcm",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "nt35523_TIANMA_qxga_dsi_vdo_truly",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	pr_notice("LCM: Register panel driver for nt35523_TIANMA_qxga_dsi_vdo_truly\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register this driver!\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister this driver done\n");
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
