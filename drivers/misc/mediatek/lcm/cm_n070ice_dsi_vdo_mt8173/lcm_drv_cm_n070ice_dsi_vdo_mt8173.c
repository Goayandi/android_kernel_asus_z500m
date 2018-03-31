#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/device.h>

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

#include "cm_n070ice_dsi_vdo_mt8173.h"

static int lcm_request_gpio_control(struct device *dev)
{
	int ret;

	GPIO_LCD_PWR_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_pwr_en", 0);
	GPIO_LCD_PWR2_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_pwr2_en", 0);
	GPIO_LCD_RST_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_rst_en", 0);

	ret = gpio_request(GPIO_LCD_PWR_EN, "GPIO_LCD_PWR_EN");
	if (ret) {
		pr_debug("[KE/LCM] gpio request GPIO_LCD_PWR_EN = 0x%x fail with %d\n", GPIO_LCD_PWR_EN, ret);
		goto out;
	}

	ret = gpio_request(GPIO_LCD_PWR2_EN, "GPIO_LCD_PWR2_EN");
	if (ret) {
		pr_debug("[KE/LCM] gpio request GPIO_LCD_PWR2_EN = 0x%x fail with %d\n", GPIO_LCD_PWR2_EN, ret);
		goto out;
	}

	ret = gpio_request(GPIO_LCD_RST_EN, "GPIO_LCD_RST_EN");
	if (ret) {
		pr_debug("[KE/LCM] gpio request GPIO_LCD_RST_EN = 0x%x fail with %d\n", GPIO_LCD_RST_EN, ret);
		goto out;
	}

out:
	return ret;
}

static int lcm_probe(struct device *dev)
{
	int ret;

	ret = lcm_request_gpio_control(dev);

	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,mt8173-lcm",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "cm_n070ice_dsi_vdo_mt8173",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	pr_debug("LCM: Register panel driver for cm_n070ice_dsi_vdo_mt8173\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register this driver!\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_debug("LCM: Unregister this driver done\n");
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
