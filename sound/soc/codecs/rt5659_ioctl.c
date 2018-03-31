/*
 * rt5659_ioctl.h  --  RT5659 ALSA SoC audio driver IO control
 *
 * Copyright 2012 Realtek Microelectronics
 * Author: Bard <bardliao@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/spi/spi.h>
#include <sound/soc.h>
#include "rt_codec_ioctl.h"
#include "rt5659_ioctl.h"
#include "rt5659.h"

hweq_t hweq_param[] = {
	{/* NORMAL */
		{0},
		{0},
		0x0000,
	},
	{/* CLUB */
		{0},
		{0},
		0x0000,
	},
	{/* SPK */
		{0},
		{0},
		0x0000,
	},
	{/* HP */
		{0},
		{0},
		0x0000,
	},
	{/* Z500M-on-board-mic-L-EQ */
		{0},
		{0xfde8, 0x0000, 0xc18b, 0x1e91, 0x0699, 0xe606, 0x0aad, 0xf65f,
		 0xe48d, 0x16e6, 0xf510, 0xf230, 0x1561, 0x2fb2, 0x1f95, 0x0000,
		 0x0800, 0x0800},
		0xffff,
	},
	{/* Z500M-on-board-mic-R-EQ */
		{0},
		{0xfde8, 0x0000, 0xc18b, 0x1e91, 0x0699, 0xe606, 0x0aad, 0xf65f,
		 0xe48d, 0x16e6, 0xf510, 0xf230, 0x1561, 0x2fb2, 0x1f95, 0x0000,
		 0x0800, 0x0800},
		0xffff,
	},
	{/* Z500M-headset-mic-L-EQ */
		{0},
		{0x1c10, 0x0000, 0xf9c4, 0x0fc7, 0x2fb2, 0xf9c4, 0x0fc7, 0x0fec,
		 0xe904, 0x1c10, 0x0000, 0x0000, 0x1c10, 0x0000, 0x0436, 0x0000,
		 0x0800, 0x0800},
		0x003c,
	},
	{/* Z500M-headset-mic-R-EQ */
		{0},
		{0x1c10, 0x0000, 0xf9c4, 0x0fc7, 0x2fb2, 0xf9c4, 0x0fc7, 0x0fec,
		 0xe904, 0x1c10, 0x0000, 0x0000, 0x1c10, 0x0000, 0x0436, 0x0000,
		 0x0800, 0x0800},
		0x003c,
	},
};
#define RT5659_HWEQ_LEN ARRAY_SIZE(hweq_param)

int eqreg[EQ_CH_NUM][EQ_REG_NUM] = {
	{0x0344, 0x0345, 0x036a, 0x036b, 0x036c, 0x036d, 0x0366, 0x0367, 0x0368, 0x0369, 0x0364,
	 0x0365, 0x0348, 0x0349, 0x034a, 0x034e, 0x034f, 0x0350, 0x0354, 0x0355, 0x0356, 0x035a,
	 0x035b, 0x035e, 0x035f, 0x0360, 0x0340, 0x0342},
	{0x0346, 0x0347, 0x0374, 0x0375, 0x0376, 0x0377, 0x0370, 0x0371, 0x0372, 0x0373, 0x036e,
	 0x036f, 0x034b, 0x034c, 0x034d, 0x0351, 0x0352, 0x0353, 0x0357, 0x0358, 0x0359, 0x035c,
	 0x035d, 0x0361, 0x0362, 0x0363, 0x0341, 0x0343},
	{0x03d0, 0x03d2, 0x03d4, 0x03d6, 0x03d8, 0x03da, 0x03dc, 0x03de, 0x03e0, 0x03e2, 0x03e4,
	 0x03e6, 0x03e8, 0x03ea, 0x03ec, 0x03ee, 0x03f0, 0x03f2},
	{0x03d1, 0x03d3, 0x03d5, 0x03d7, 0x03d9, 0x03db, 0x03dd, 0x03df, 0x03e1, 0x03e3, 0x03e5,
	 0x03e7, 0x03e9, 0x03eb, 0x03ed, 0x03ef, 0x03f1, 0x03f3},
};

int rt5659_update_eqmode(struct snd_soc_codec *codec, int channel, int mode)
{
	struct rt_codec_ops *ioctl_ops = rt_codec_get_ioctl_ops();
	int i, upd_reg, reg, mask;

	if (codec == NULL ||  mode >= RT5659_HWEQ_LEN)
		return -EINVAL;

	dev_dbg(codec->dev, "%s(): mode=%d\n", __func__, mode);
	if (mode != NORMAL) {
		for(i = 0; i < EQ_REG_NUM; i++) {
			hweq_param[mode].reg[i] = eqreg[channel][i];
		}

		for(i = 0; i < EQ_REG_NUM; i++) {
			if(hweq_param[mode].reg[i])
			snd_soc_write(codec, hweq_param[mode].reg[i],
					hweq_param[mode].value[i]);
			else
				break;
		}
	}
	switch (channel) {
	case EQ_CH_DAC_STEREO_L:
		reg = RT5659_DAC_EQ_CTRL_3;
		mask = 0x0005;
		upd_reg = RT5659_DAC_EQ_CTRL_1;
		break;
	case EQ_CH_DAC_STEREO_R:
		reg = RT5659_DAC_EQ_CTRL_3;
		mask = 0x000a;
		upd_reg = RT5659_DAC_EQ_CTRL_1;
		break;
	case EQ_CH_ADC_STEREO_L:
		reg = RT5659_ADC_EQ_CTRL_2;
		mask = 0xafff;
		upd_reg = RT5659_ADC_EQ_CTRL_1;
		break;
	case EQ_CH_ADC_STEREO_R:
		reg = RT5659_ADC_EQ_CTRL_2;
		mask = 0x5fff;
		upd_reg = RT5659_ADC_EQ_CTRL_1;
		break;
	default:
		printk("Invalid EQ channel\n");
		return -EINVAL;
	}
	snd_soc_update_bits(codec, reg, mask, hweq_param[mode].ctrl);
	snd_soc_update_bits(codec, upd_reg,
		RT5659_EQ_UPD, RT5659_EQ_UPD);
	snd_soc_update_bits(codec, upd_reg, RT5659_EQ_UPD, 0);

	return 0;
}

int rt5659_ioctl_common(struct snd_hwdep *hw, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct snd_soc_codec *codec = hw->private_data;
	struct rt_codec_cmd __user *_rt_codec = (struct rt_codec_cmd *)arg;
	struct rt_codec_cmd rt_codec;
	//struct rt_codec_ops *ioctl_ops = rt_codec_get_ioctl_ops();
	int *buf;
	static int eq_mode[EQ_CH_NUM];

	if (copy_from_user(&rt_codec, _rt_codec, sizeof(rt_codec))) {
		dev_err(codec->dev,"copy_from_user faild\n");
		return -EFAULT;
	}
	dev_dbg(codec->dev, "%s(): rt_codec.number=%d, cmd=%d\n",
			__func__, rt_codec.number, cmd);
	buf = kmalloc(sizeof(*buf) * rt_codec.number, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;
	if (copy_from_user(buf, rt_codec.buf, sizeof(*buf) * rt_codec.number)) {
		goto err;
	}

	switch (cmd) {
	case RT_SET_CODEC_HWEQ_IOCTL:
		if (eq_mode == *buf)
			break;
		eq_mode[*buf] = *(buf + 1);
		rt5659_update_eqmode(codec, eq_mode[*buf], *buf);
		break;

	case RT_GET_CODEC_ID:
		*buf = snd_soc_read(codec, RT5659_DEVICE_ID);
		if (copy_to_user(rt_codec.buf, buf, sizeof(*buf) * rt_codec.number))
			goto err;
		break;
	default:
		break;
	}

	kfree(buf);
	return 0;

err:
	kfree(buf);
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(rt5659_ioctl_common);
