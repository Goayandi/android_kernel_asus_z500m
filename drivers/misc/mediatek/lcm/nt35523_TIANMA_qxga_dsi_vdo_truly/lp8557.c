#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>


static struct i2c_client *ti_lp8557_client;

int ti_lp8557_pwm_on(void)
{
	printk("%s ++\n", __func__);

	i2c_smbus_write_byte_data(ti_lp8557_client, 0x00, 0x00);
	i2c_smbus_write_byte_data(ti_lp8557_client, 0x10, 0x84);
	i2c_smbus_write_byte_data(ti_lp8557_client, 0x11, 0x05);
	i2c_smbus_write_byte_data(ti_lp8557_client, 0x12, 0x2C);
	i2c_smbus_write_byte_data(ti_lp8557_client, 0x13, 0x03);
	i2c_smbus_write_byte_data(ti_lp8557_client, 0x15, 0xC3);
	i2c_smbus_write_byte_data(ti_lp8557_client, 0x7F, 0x21);
	i2c_smbus_write_byte_data(ti_lp8557_client, 0x7A, 0x88);
	i2c_smbus_write_byte_data(ti_lp8557_client, 0x16, 0x60);
	i2c_smbus_write_byte_data(ti_lp8557_client, 0x14, 0xBF);

	i2c_smbus_write_byte_data(ti_lp8557_client, 0x00, 0x01);
	printk("%s --\n", __func__);
	return 0;
}
EXPORT_SYMBOL(ti_lp8557_pwm_on);

int ti_lp8557_pwm_off(void)
{
	printk("%s ++\n", __func__);
	i2c_smbus_write_byte_data(ti_lp8557_client, 0x00, 0x00);
	printk("%s --\n", __func__);
	return 0;
}
EXPORT_SYMBOL(ti_lp8557_pwm_off);

static int ti_lp8557_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
	printk("%s ++\n", __func__);

	ti_lp8557_client = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality() failed\n");
		return -ENODEV;
	}

	ti_lp8557_client = client;

	printk("%s: i2c device name=%s, addr=0x%x, adapter nr=%d\n", __func__,
		ti_lp8557_client->name, ti_lp8557_client->addr, ti_lp8557_client->adapter->nr);

	printk("%s --\n", __func__);
	return 0;
}

static int ti_lp8557_remove(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id ti_lp8557_match_table[] = {
	{ .compatible = "ti,lp8557",},
	{ },
};
#endif

static const struct i2c_device_id ti_lp8557_id[] = {
	{"lp8557", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ti_lp8557_id);

static struct i2c_driver ti_lp8557_driver = {
	.driver = {
		.name = "lp8557",
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = ti_lp8557_match_table,
#endif
	},
	.id_table = ti_lp8557_id,
	.probe = ti_lp8557_probe,
	.remove = ti_lp8557_remove,
};

int __init ti_lp8557_init(void)
{
	printk("%s\n", __func__);
	return i2c_add_driver(&ti_lp8557_driver);
}

void __exit ti_lp8557_exit(void)
{
	printk("%s\n", __func__);
	i2c_del_driver(&ti_lp8557_driver);
}

module_init(ti_lp8557_init);
module_exit(ti_lp8557_exit);
