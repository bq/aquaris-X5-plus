/*
 * ALSA SoC Texas Instruments TPA6130A2 headset stereo amplifier driver
 *
 * Copyright (C) Nokia Corporation
 *
 * Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */
//#define DEBUG
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <sound/tpa6130a2-plat.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "tpa6130a2.h"

enum tpa_model {
	TPA6130A2,
	TPA6140A2,
};

static struct i2c_client *tpa6130a2_client;

/* This struct is used to save the context */
struct tpa6130a2_data {
	struct mutex mutex;
	unsigned char regs[TPA6130A2_CACHEREGNUM];
	struct regulator *supply;
	int power_gpio;
	u8 power_state:1;
	enum tpa_model id;
};

static int tpa6130a2_i2c_read(int reg)
{
	struct tpa6130a2_data *data;
	int val;

	BUG_ON(tpa6130a2_client == NULL);
	data = i2c_get_clientdata(tpa6130a2_client);

	/* If powered off, return the cached value */
	if (data->power_state) {
		val = i2c_smbus_read_byte_data(tpa6130a2_client, reg);
		if (val < 0)
			dev_err(&tpa6130a2_client->dev, "Read failed\n");
		else
			data->regs[reg] = val;
	} else {
		val = data->regs[reg];
	}
	pr_debug("yht At %d In (%s),[reg:%d]=0x%x\n",__LINE__, __FUNCTION__,reg,val);

	return val;
}

static int tpa6130a2_i2c_write(int reg, u8 value)
{
	struct tpa6130a2_data *data;
	int val = 0;

	BUG_ON(tpa6130a2_client == NULL);
	data = i2c_get_clientdata(tpa6130a2_client);

	if (data->power_state) {
		val = i2c_smbus_write_byte_data(tpa6130a2_client, reg, value);
		if (val < 0) {
			dev_err(&tpa6130a2_client->dev, "Write failed\n");
			return val;
		}
	} else {
		printk(KERN_ERR "yht--,At %d In (%s),error, power_state is not on\n",__LINE__, __FUNCTION__);
	}

	/* Either powered on or off, we save the context */
	data->regs[reg] = value;

	return val;
}

static u8 tpa6130a2_read(int reg)
{
	struct tpa6130a2_data *data;

	BUG_ON(tpa6130a2_client == NULL);
	data = i2c_get_clientdata(tpa6130a2_client);

	pr_debug("yht At %d In (%s),data->regs[%d]=0x%x\n",__LINE__, __FUNCTION__,reg,data->regs[reg]);
	return data->regs[reg];
}

static int tpa6130a2_initialize(void)
{
	struct tpa6130a2_data *data;
	int i, ret = 0;

	BUG_ON(tpa6130a2_client == NULL);
	data = i2c_get_clientdata(tpa6130a2_client);

	for (i = 1; i < TPA6130A2_REG_VERSION; i++) {
		ret = tpa6130a2_i2c_write(i, data->regs[i]);
		pr_debug("yht At %d In (%s),ret=%d,data->regs[%d]=0x%x\n",__LINE__, __FUNCTION__,ret,i,data->regs[i]);
		if (ret < 0)
			break;
	}

	return ret;
}

static int tpa6130a2_power(u8 power)
{
	struct	tpa6130a2_data *data;
	u8	val;
	int	ret = 0;

	BUG_ON(tpa6130a2_client == NULL);
	data = i2c_get_clientdata(tpa6130a2_client);
	printk(KERN_ERR "yht At %d In (%s),power=%d,power_state=%d,power_gpio state is %d\n",__LINE__, __FUNCTION__,power,data->power_state,gpio_get_value(data->power_gpio));
	//if (power == 0) power = 1;

	mutex_lock(&data->mutex);
	if (power == data->power_state)
		goto exit;

	if (power) {
		/*ret = regulator_enable(data->supply);
		if (ret != 0) {
			dev_err(&tpa6130a2_client->dev,
				"Failed to enable supply: %d\n", ret);
			goto exit;
		}*/
		/* Power on */
		if (data->power_gpio >= 0)
			gpio_set_value(data->power_gpio, 1);

		data->power_state = 1;
		ret = tpa6130a2_initialize();
		pr_debug("yht At %d In (%s),tpa6130a2_initialize() ret=%d\n",__LINE__, __FUNCTION__,ret);
		if (ret < 0) {
			dev_err(&tpa6130a2_client->dev,
				"Failed to initialize chip\n");
			if (data->power_gpio >= 0)
				gpio_set_value(data->power_gpio, 0);
			//regulator_disable(data->supply);
			data->power_state = 0;
			goto exit;
		}
		pr_debug("yht At %d In (%s),set data->power_gpio state is %d, ret=%d\n",__LINE__, __FUNCTION__,gpio_get_value(data->power_gpio),ret);
	} else {
		/* set SWS */
		val = tpa6130a2_read(TPA6130A2_REG_CONTROL);
		val |= TPA6130A2_SWS;
		tpa6130a2_i2c_write(TPA6130A2_REG_CONTROL, val);
		pr_debug("yht At %d In (%s),regs[%d]=0x%x,data->power_state=%d\n",__LINE__, __FUNCTION__,TPA6130A2_REG_CONTROL,val,data->power_state);

		tpa6130a2_read(TPA6130A2_REG_CONTROL);
		tpa6130a2_read(TPA6130A2_REG_VOL_MUTE);
		tpa6130a2_read(TPA6130A2_REG_OUT_IMPEDANCE);
		/* Power off */
		if (data->power_gpio >= 0)
			gpio_set_value(data->power_gpio, 0);

		/*ret = regulator_disable(data->supply);
		if (ret != 0) {
			dev_err(&tpa6130a2_client->dev,
				"Failed to disable supply: %d\n", ret);
			goto exit;
		}*/
		pr_debug("yht At %d In (%s), get data->power_gpio state is %d,ret=%d\n",__LINE__, __FUNCTION__,gpio_get_value(data->power_gpio),ret);

		data->power_state = 0;
	}

exit:
	mutex_unlock(&data->mutex);
	return ret;
}

static int tpa6130a2_get_volsw(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct tpa6130a2_data *data;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;

	BUG_ON(tpa6130a2_client == NULL);
	data = i2c_get_clientdata(tpa6130a2_client);

	mutex_lock(&data->mutex);

	ucontrol->value.integer.value[0] =
		(tpa6130a2_read(reg) >> shift) & mask;

	if (invert)
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];

	mutex_unlock(&data->mutex);
	return 0;
}

static int tpa6130a2_put_volsw(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct tpa6130a2_data *data;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned int val = (ucontrol->value.integer.value[0] & mask);
	unsigned int val_reg;

	BUG_ON(tpa6130a2_client == NULL);
	data = i2c_get_clientdata(tpa6130a2_client);

	if (invert)
		val = max - val;

	mutex_lock(&data->mutex);

	val_reg = tpa6130a2_read(reg);
	if (((val_reg >> shift) & mask) == val) {
		mutex_unlock(&data->mutex);
		return 0;
	}

	val_reg &= ~(mask << shift);
	val_reg |= val << shift;
	tpa6130a2_i2c_write(reg, val_reg);

	mutex_unlock(&data->mutex);

	return 1;
}

/*
 * TPA6130 volume. From -59.5 to 4 dB with increasing step size when going
 * down in gain.
 */
static const unsigned int tpa6130_tlv[] = {
	TLV_DB_RANGE_HEAD(10),
	0, 1, TLV_DB_SCALE_ITEM(-5950, 600, 0),
	2, 3, TLV_DB_SCALE_ITEM(-5000, 250, 0),
	4, 5, TLV_DB_SCALE_ITEM(-4550, 160, 0),
	6, 7, TLV_DB_SCALE_ITEM(-4140, 190, 0),
	8, 9, TLV_DB_SCALE_ITEM(-3650, 120, 0),
	10, 11, TLV_DB_SCALE_ITEM(-3330, 160, 0),
	12, 13, TLV_DB_SCALE_ITEM(-3040, 180, 0),
	14, 20, TLV_DB_SCALE_ITEM(-2710, 110, 0),
	21, 37, TLV_DB_SCALE_ITEM(-1960, 74, 0),
	38, 63, TLV_DB_SCALE_ITEM(-720, 45, 0),
};

static const struct snd_kcontrol_new tpa6130a2_controls[] = {
	SOC_SINGLE_EXT_TLV("TPA6130A2 Headphone Playback Volume",
		       TPA6130A2_REG_VOL_MUTE, 0, 0x3f, 0,
		       tpa6130a2_get_volsw, tpa6130a2_put_volsw,
		       tpa6130_tlv),
};

static const unsigned int tpa6140_tlv[] = {
	TLV_DB_RANGE_HEAD(3),
	0, 8, TLV_DB_SCALE_ITEM(-5900, 400, 0),
	9, 16, TLV_DB_SCALE_ITEM(-2500, 200, 0),
	17, 31, TLV_DB_SCALE_ITEM(-1000, 100, 0),
};

static const struct snd_kcontrol_new tpa6140a2_controls[] = {
	SOC_SINGLE_EXT_TLV("TPA6140A2 Headphone Playback Volume",
		       TPA6130A2_REG_VOL_MUTE, 1, 0x1f, 0,
		       tpa6130a2_get_volsw, tpa6130a2_put_volsw,
		       tpa6140_tlv),
};

/*
 * Enable or disable channel (left or right)
 * The bit number for mute and amplifier are the same per channel:
 * bit 6: Right channel
 * bit 7: Left channel
 * in both registers.
 */
static void tpa6130a2_channel_enable(u8 channel, int enable)
{
	u8	val;

	if (enable) {
		/* Enable channel */
		/* Enable amplifier */
		val = tpa6130a2_read(TPA6130A2_REG_CONTROL);
		val |= channel;
		val &= ~TPA6130A2_SWS;
		tpa6130a2_i2c_write(TPA6130A2_REG_CONTROL, val);
		pr_debug("yht At %d In (%s),regs[%d]=0x%x\n",__LINE__, __FUNCTION__,TPA6130A2_REG_CONTROL,val);

		/* Unmute channel */
		val = tpa6130a2_read(TPA6130A2_REG_VOL_MUTE);
		val &= ~channel;
		tpa6130a2_i2c_write(TPA6130A2_REG_VOL_MUTE, val);
		pr_debug("yht At %d In (%s),regs[%d]=0x%x\n",__LINE__, __FUNCTION__,TPA6130A2_REG_VOL_MUTE,val);

		#if defined(CONFIG_SND_SOC_TPA6130A2)
		/* disable high impedance mode */
		val = tpa6130a2_read(TPA6130A2_REG_OUT_IMPEDANCE);
		val |= 0x3;
		tpa6130a2_i2c_write(TPA6130A2_REG_OUT_IMPEDANCE, val);
		pr_debug("yht At %d In (%s),regs[%d]=0x%x\n",__LINE__, __FUNCTION__,TPA6130A2_REG_OUT_IMPEDANCE,val);
		#endif

	} else {
		/* Disable channel */
		/* Mute channel */
		val = tpa6130a2_read(TPA6130A2_REG_VOL_MUTE);
		val |= channel;
		tpa6130a2_i2c_write(TPA6130A2_REG_VOL_MUTE, val);
		pr_debug("yht At %d In (%s),regs[%d]=0x%x\n",__LINE__, __FUNCTION__,TPA6130A2_REG_VOL_MUTE,val);

		/* Disable amplifier */
		val = tpa6130a2_read(TPA6130A2_REG_CONTROL);
		val &= ~channel;
		tpa6130a2_i2c_write(TPA6130A2_REG_CONTROL, val);
		pr_debug("yht At %d In (%s),regs[%d]=0x%x\n",__LINE__, __FUNCTION__,TPA6130A2_REG_CONTROL,val);

		#if defined(CONFIG_SND_SOC_TPA6130A2)
		/* enable high impedance mode */
		val = tpa6130a2_read(TPA6130A2_REG_OUT_IMPEDANCE);
		val &= 0xFC;
		tpa6130a2_i2c_write(TPA6130A2_REG_OUT_IMPEDANCE, val);
		pr_debug("yht At %d In (%s),regs[%d]=0x%x\n",__LINE__, __FUNCTION__,TPA6130A2_REG_OUT_IMPEDANCE,val);

		// software shutdown
		//val = tpa6130a2_read(TPA6130A2_REG_CONTROL);		
		//val |= TPA6130A2_SWS;
		//tpa6130a2_i2c_write(TPA6130A2_REG_CONTROL, val);
		//pr_debug("yht At %d In (%s),regs[%d]=0x%x\n",__LINE__, __FUNCTION__,TPA6130A2_REG_CONTROL,val);
		#endif

	}
}

int tpa6130a2_stereo_enable(struct snd_soc_codec *codec, int enable)
{
	int ret = 0;
	#if defined(CONFIG_SND_SOC_TPA6130A2)
	struct	tpa6130a2_data *data;

	if (tpa6130a2_client == NULL)
		return -ENODEV;
	data = i2c_get_clientdata(tpa6130a2_client);
	pr_debug("yht At %d In (%s),enable=%d,data->power_state=%d\n",__LINE__, __FUNCTION__,enable,data->power_state);
	if (enable == data->power_state) {
		return ret;
	}
	pr_debug("yht At %d In (%s),start to run\n",__LINE__, __FUNCTION__);
	#endif

	if (enable) {
		ret = tpa6130a2_power(1);
		if (ret < 0)
			return ret;
		tpa6130a2_channel_enable(TPA6130A2_HP_EN_R | TPA6130A2_HP_EN_L,
					 1);
		tpa6130a2_read(TPA6130A2_REG_CONTROL);
		tpa6130a2_read(TPA6130A2_REG_VOL_MUTE);
		tpa6130a2_read(TPA6130A2_REG_OUT_IMPEDANCE);
	} else {
		tpa6130a2_channel_enable(TPA6130A2_HP_EN_R | TPA6130A2_HP_EN_L,
					 0);
		ret = tpa6130a2_power(0);
	}
	printk(KERN_ERR "yht--,At %d In (%s),enable=%d,regs[%d]=0x%x,regs[%d]=0x%x,regs[%d]=0x%x\n",__LINE__, __FUNCTION__,enable,
		TPA6130A2_REG_CONTROL,data->regs[TPA6130A2_REG_CONTROL],TPA6130A2_REG_VOL_MUTE,data->regs[TPA6130A2_REG_VOL_MUTE],TPA6130A2_REG_OUT_IMPEDANCE,data->regs[TPA6130A2_REG_OUT_IMPEDANCE]);

	return ret;
}
EXPORT_SYMBOL_GPL(tpa6130a2_stereo_enable);

int tpa6130a2_add_controls(struct snd_soc_codec *codec)
{
	struct	tpa6130a2_data *data;

	if (tpa6130a2_client == NULL)
		return -ENODEV;
	pr_debug("yht At %d In (%s)\n",__LINE__, __FUNCTION__);

	data = i2c_get_clientdata(tpa6130a2_client);

	if (data->id == TPA6140A2)
		return snd_soc_add_codec_controls(codec, tpa6140a2_controls,
						ARRAY_SIZE(tpa6140a2_controls));
	else
		return snd_soc_add_codec_controls(codec, tpa6130a2_controls,
						ARRAY_SIZE(tpa6130a2_controls));
}
EXPORT_SYMBOL_GPL(tpa6130a2_add_controls);

#if defined(CONFIG_SND_SOC_TPA6130A2)
static int parse_tpa6130a2_info(struct device *dev, struct tpa6130a2_data *data)
{
	int ret = 0;
	struct device_node *np = dev->of_node;

	data->power_gpio = of_get_named_gpio(np, "qcom,tpa6130a2-en", 0);
	if ((!gpio_is_valid(data->power_gpio))) {
		pr_err("Error reading idata->power_gpio =%d\n", data->power_gpio);
		ret = -EINVAL;
		goto err_get;
	} else
		pr_err("data->power_gpio=%d\n", data->power_gpio);
	pr_debug("yht At %d In (%s),ok, ret=%d\n",__LINE__, __FUNCTION__,ret);

	if (gpio_is_valid(data->power_gpio)){
		pr_debug("yht At %d In (%s)\n",__LINE__, __FUNCTION__);
		ret = gpio_request_one(data->power_gpio,GPIOF_DIR_OUT | GPIOF_INIT_LOW, "tpa6130a2-en-pin");
		if (ret) {
			pr_err("unable to request gpio [%d]\n", data->power_gpio);
			pr_debug("yht At %d In (%s),ok, ret=%d\n",__LINE__, __FUNCTION__,ret);
			goto err_request;
		}
/*		ret = gpio_request(data->power_gpio, "tpa6130a2-en-pin");
		if (ret < 0) {
			pr_err("unable to request gpio [%d]\n", data->power_gpio);
			pr_debug("yht At %d In (%s),ok, ret=%d\n",__LINE__, __FUNCTION__,ret);
			goto err_request;
		}
		
		pr_debug("yht At %d In (%s),set gpio_direction_output\n",__LINE__, __FUNCTION__);
		ret = gpio_direction_output(data->power_gpio,0);
		if (ret < 0) {
			pr_err("unable to request gpio [%d]\n", data->power_gpio);
			pr_debug("yht At %d In (%s),ok, ret=%d\n",__LINE__, __FUNCTION__,ret);
			goto err_request;
		}*/

		pr_debug("yht At %d In (%s),set data->power_gpio to low\n",__LINE__, __FUNCTION__);
		gpio_set_value(data->power_gpio, 0);
		pr_debug("yht At %d In (%s),set data->power_gpio state is %d\n",__LINE__, __FUNCTION__,gpio_get_value(data->power_gpio));
	}
	pr_debug("yht At %d In (%s),ok, ret=%d\n",__LINE__, __FUNCTION__,ret);

	return ret;

err_request:
err_get:
	pr_debug("yht At %d In (%s),error, ret=%d\n",__LINE__, __FUNCTION__,ret);
	ret = -EINVAL;
	return ret;
}
#endif

static int tpa6130a2_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct device *dev;
	struct tpa6130a2_data *data;
	//struct tpa6130a2_platform_data *pdata;
	const char *regulator;
	int ret;

	dev = &client->dev;
#if defined(CONFIG_SND_SOC_TPA6130A2)
	pr_debug("yht At %d In (%s),\n",__LINE__, __FUNCTION__);

/*	if (client->dev.platform_data == NULL) {
		dev_err(dev, "Platform data not set\n");
		dump_stack();
		return -ENODEV;
	}*/
#endif

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Can not allocate memory\n");
		return -ENOMEM;
	}

	tpa6130a2_client = client;

#if defined(CONFIG_SND_SOC_TPA6130A2)
	ret = parse_tpa6130a2_info(&client->dev, data);
	if (ret < 0) {
		pr_err("invalid earPA pin - %d\n", data->power_gpio);
		return -EINVAL;
	}
	pr_debug("yht At %d In (%s),\n",__LINE__, __FUNCTION__);
#endif

	i2c_set_clientdata(tpa6130a2_client, data);

	//pdata = client->dev.platform_data;
	//data->power_gpio = pdata->power_gpio;
	data->id = id->driver_data;
	pr_debug("yht At %d In (%s),data->power_gpio=%d,data->id=%d\n",__LINE__, __FUNCTION__,data->power_gpio,data->id);

	mutex_init(&data->mutex);

	/* Set default register values */
	/*
	data->regs[TPA6130A2_REG_CONTROL] =	TPA6130A2_SWS;
	data->regs[TPA6130A2_REG_VOL_MUTE] =	TPA6130A2_MUTE_R |
						TPA6130A2_MUTE_L;
	*/
	/*
	// this is for test
	data->regs[TPA6130A2_REG_CONTROL] =	TPA6130A2_HP_EN_L |TPA6130A2_HP_EN_R;
	data->regs[TPA6130A2_REG_VOL_MUTE] =	TPA6130A2_VOLUME(0x3f);
	data->regs[TPA6130A2_REG_OUT_IMPEDANCE] =	TPA6130A2_HIZ_R | TPA6130A2_HIZ_L;
	//data->regs[TPA6130A2_REG_VERSION] =
	*/
	 // temparory remove
	data->regs[TPA6130A2_REG_CONTROL] =	TPA6130A2_SWS;
	data->regs[TPA6130A2_REG_VOL_MUTE] = TPA6130A2_MUTE_R | TPA6130A2_MUTE_L | TPA6130A2_VOLUME(0x3f);
	//data->regs[TPA6130A2_REG_OUT_IMPEDANCE] =	TPA6130A2_HIZ_R | TPA6130A2_HIZ_L;

#if defined(CONFIG_SND_SOC_TPA6130A2)
/*	if (data->power_gpio >= 0) {
		ret = devm_gpio_request(dev, data->power_gpio,
					"tpa6130a2 enable");
		if (ret < 0) {
			dev_err(dev, "Failed to request power GPIO (%d)\n",
				data->power_gpio);
			goto err_gpio;
		}
		gpio_direction_output(data->power_gpio, 0);
	}*/
	pr_debug("yht At %d In (%s),\n",__LINE__, __FUNCTION__);
#endif

	switch (data->id) {
	default:
		dev_warn(dev, "Unknown TPA model (%d). Assuming 6130A2\n",
			 data->id);
	case TPA6130A2:
		regulator = "Vdd";
		break;
	case TPA6140A2:
		regulator = "AVdd";
		break;
	}

/*	data->supply = devm_regulator_get(dev, regulator);
	if (IS_ERR(data->supply)) {
		ret = PTR_ERR(data->supply);
		dev_err(dev, "Failed to request supply: %d\n", ret);
		goto err_gpio;
	}*/

	ret = tpa6130a2_power(1);
	if (ret != 0)
		goto err_gpio;


	/* Read version */
	ret = tpa6130a2_i2c_read(TPA6130A2_REG_VERSION) &
				 TPA6130A2_VERSION_MASK;
	pr_debug("yht At %d In (%s),ret=%d\n",__LINE__, __FUNCTION__,ret);
	if ((ret != 1) && (ret != 2))
		dev_warn(dev, "UNTESTED version detected (%d)\n", ret);

	/* Disable the chip */
	ret = tpa6130a2_power(0);
	if (ret != 0)
		goto err_gpio;

	return 0;

err_gpio:
	tpa6130a2_client = NULL;

	return ret;
}

static int tpa6130a2_remove(struct i2c_client *client)
{
	tpa6130a2_power(0);
	tpa6130a2_client = NULL;

	return 0;
}

static const struct i2c_device_id tpa6130a2_id[] = {
	{ "tpa6130a2", TPA6130A2 },
	{ "tpa6140a2", TPA6140A2 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tpa6130a2_id);

static struct i2c_driver tpa6130a2_i2c_driver = {
	.driver = {
		.name = "tpa6130a2",
		.owner = THIS_MODULE,
	},
	.probe = tpa6130a2_probe,
	.remove = tpa6130a2_remove,
	.id_table = tpa6130a2_id,
};

module_i2c_driver(tpa6130a2_i2c_driver);

MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_DESCRIPTION("TPA6130A2 Headphone amplifier driver");
MODULE_LICENSE("GPL");
