/*
 * ASUS Lid driver.
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio_event.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#define DRV_NAME		"asustek_fingerprintkey"
#define FINGERPRINTKEY_DEVICE_NAME		"fingerprintkey_input"
#define FINGERPRINTKEY_PHYS		"/dev/input/fingerprintkey_indev"
#define CONVERSION_TIME_MS	50

static void fingerprintkey_report_function(struct work_struct *work);
static ssize_t show_fingerprintkey_status(struct device *dev,
		struct device_attribute *attr, char *buf);

struct asustek_fingerprintkey_drvdata {
	struct input_dev *input;
	struct delayed_work dwork;
	int gpio;
	int active_low;
	int wakeup;
	unsigned int irq;
};

static DEVICE_ATTR(fingerprintkey_status, S_IRUGO, show_fingerprintkey_status, NULL);

static struct attribute *fingerprintkey_attrs[] = {
	&dev_attr_fingerprintkey_status.attr,
	NULL
};

static struct attribute_group fingerprintkey_attr_group = {
	.attrs = fingerprintkey_attrs,
};

static ssize_t show_fingerprintkey_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct asustek_fingerprintkey_drvdata *ddata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			!!gpio_get_value_cansleep(ddata->gpio));
}

static irqreturn_t fingerprintkey_interrupt_handler(int irq, void *dev_id)
{
	struct asustek_fingerprintkey_drvdata *ddata = dev_id;

	WARN_ONCE(irq != ddata->irq, "fingerprintkey_interrupt failed\n");

	schedule_delayed_work(&ddata->dwork,
			msecs_to_jiffies(CONVERSION_TIME_MS));

	return IRQ_HANDLED;
}

static void fingerprintkey_report_function(struct work_struct *work)
{
	struct asustek_fingerprintkey_drvdata *ddata =
		container_of(work, struct asustek_fingerprintkey_drvdata, dwork.work);
	int value;

	value = !!gpio_get_value_cansleep(ddata->gpio) ^ ddata->active_low;

	input_report_key(ddata->input, KEY_HOMEPAGE, !value);
	input_sync(ddata->input);

	pr_info("EV_KEY report value = %d\n", value);
}

/* translate openfirmware node properties */
static int __init asustek_fingerprintkey_get_devtree(struct device *dev)
{
	struct asustek_fingerprintkey_drvdata *ddata = dev_get_drvdata(dev);
	struct device_node *node;
	enum of_gpio_flags flags;

	node = dev->of_node;

	if (!node)
		return -ENODEV;

	if (!of_find_property(node, "gpios", NULL)) {
		static const char dt_get_err[] __initconst =
					"lid sensor without gpios\n";
		dev_err(dev, dt_get_err);
		return -EINVAL;
	}

	ddata->gpio = of_get_gpio_flags(node, 0, &flags);
	ddata->active_low = flags & OF_GPIO_ACTIVE_LOW;
	ddata->wakeup = !!of_get_property(node, "asustek_fingerprintkey,wakeup", NULL);

	return 0;
}

static int __init fingerprintkey_driver_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct device *dev = &pdev->dev;
	struct asustek_fingerprintkey_drvdata *ddata;
	struct input_dev *input;
	static const char fingerprintkey_probe[] __initconst = "ASUSTek: %s\n";

	if (!pdev)
		return -EINVAL;

	dev_info(dev, fingerprintkey_probe, __func__);
	ddata = kzalloc(sizeof(struct asustek_fingerprintkey_drvdata), GFP_KERNEL);
	input = input_allocate_device();

	if (!ddata || !input) {
		static const char errmsg[] __initconst =
					"Failed to allocate state\n";
		ret = -ENOMEM;
		dev_err(dev, errmsg);
		input_free_device(input);
		goto fail_data;
	}

	ddata->input = input;
	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = FINGERPRINTKEY_DEVICE_NAME;
	input->phys = FINGERPRINTKEY_PHYS;
	set_bit(EV_KEY, input->evbit);
	set_bit(KEY_HOMEPAGE, input->keybit);

	ret = input_register_device(input);

	if (ret) {
		static const char errmsg[] __initconst =
				"Unable to register input device, error: %d\n";
		dev_err(dev, errmsg, ret);
		input_free_device(input);
		goto fail_data;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &fingerprintkey_attr_group);

	if (ret) {
		static const char errmsg[] __initconst =
				"Unable to create sysfs, error: %d\n";
		dev_err(dev, errmsg, ret);
		goto fail_unregister;
	}

	INIT_DELAYED_WORK(&ddata->dwork, fingerprintkey_report_function);

	asustek_fingerprintkey_get_devtree(dev);

	if (!gpio_is_valid(ddata->gpio)) {
		static const char errmsg[] __initconst =
					"Invalid GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_create;
	}

	ret = gpio_request(ddata->gpio, DRV_NAME);

	if (ret < 0) {
		static const char errmsg[] __initconst =
				"Failed to request GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_create;
	}

	ret = gpio_direction_input(ddata->gpio);

	if (ret < 0) {
		static const char errmsg[] __initconst =
				"Failed to configure direction for GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_free;
	}

	irq = gpio_to_irq(ddata->gpio);
	ddata->irq = irq;

	if (irq < 0) {
		static const char errmsg[] __initconst =
				"Unable to get irq number for GPIO %d\n";
		dev_err(dev, errmsg, ddata->gpio);
		goto fail_free;
	}

	ret = request_any_context_irq(irq, fingerprintkey_interrupt_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"asustek_fingerprintkey_isr", ddata);

	if (ret < 0) {
		static const char errmsg[] __initconst =
				"Unable to claim irq %d\n";
		dev_err(dev, errmsg, irq);
		goto fail_free;
	}

	if (ddata->wakeup) {
		device_init_wakeup(&pdev->dev, 1);
		enable_irq_wake(irq);
	}

	return ret;

fail_free:
	gpio_free(ddata->gpio);
fail_create:
	sysfs_remove_group(&pdev->dev.kobj, &fingerprintkey_attr_group);
fail_unregister:
	input_unregister_device(input);
fail_data:
	kfree(ddata);
	return ret;
}

static int fingerprintkey_driver_remove(struct platform_device *pdev)
{
	struct asustek_fingerprintkey_drvdata *ddata = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &fingerprintkey_attr_group);
	free_irq(ddata->irq, ddata);
	cancel_delayed_work_sync(&ddata->dwork);

	if (gpio_is_valid(ddata->gpio))
		gpio_free(ddata->gpio);

	input_unregister_device(ddata->input);

	if (ddata->wakeup)
		device_init_wakeup(&pdev->dev, 0);

	kfree(ddata);

	return 0;
}

static const struct of_device_id asustek_fingerprintkey_of_match[] = {
	{ .compatible = DRV_NAME, },
	{},
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

static struct platform_driver asustek_fingerprintkey_driver __refdata = {
	.probe = fingerprintkey_driver_probe,
	.remove = fingerprintkey_driver_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(asustek_fingerprintkey_of_match),
	},
};

module_platform_driver(asustek_fingerprintkey_driver);
MODULE_DESCRIPTION("ASUSTek Finger Print Key Driver");
MODULE_AUTHOR("Bross Kuo <Bross_Kuo@asus.com>");
MODULE_LICENSE("GPL");
