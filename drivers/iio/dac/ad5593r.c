// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD5593R Digital <-> Analog converters driver
 *
 * Copyright 2015-2016 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 */

#include "ad5592r-base.h"

#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/acpi.h>

#define AD5593R_MODE_CONF		(0 << 4)
#define AD5593R_MODE_DAC_WRITE		(1 << 4)
#define AD5593R_MODE_ADC_READBACK	(4 << 4)
#define AD5593R_MODE_DAC_READBACK	(5 << 4)
#define AD5593R_MODE_GPIO_READBACK	(6 << 4)
#define AD5593R_MODE_REG_READBACK	(7 << 4)

/* Parameters for dynamic channel mode setting */
static char *ch_mode = "";
module_param(ch_mode, charp, 0400);

static int ad5593r_write_dac(struct ad5592r_state *st, unsigned chan, u16 value)
{
	struct i2c_client *i2c = to_i2c_client(st->dev);

	return i2c_smbus_write_word_swapped(i2c,
			AD5593R_MODE_DAC_WRITE | chan, value);
}

static int ad5593r_read_adc(struct ad5592r_state *st, unsigned chan, u16 *value)
{
	struct i2c_client *i2c = to_i2c_client(st->dev);
	s32 val;

	val = i2c_smbus_write_word_swapped(i2c,
			AD5593R_MODE_CONF | AD5592R_REG_ADC_SEQ, BIT(chan));
	if (val < 0)
		return (int) val;

	val = i2c_smbus_read_word_swapped(i2c, AD5593R_MODE_ADC_READBACK);
	if (val < 0)
		return (int) val;

	*value = (u16) val;

	return 0;
}

static int ad5593r_reg_write(struct ad5592r_state *st, u8 reg, u16 value)
{
	struct i2c_client *i2c = to_i2c_client(st->dev);

	return i2c_smbus_write_word_swapped(i2c,
			AD5593R_MODE_CONF | reg, value);
}

static int ad5593r_reg_read(struct ad5592r_state *st, u8 reg, u16 *value)
{
	struct i2c_client *i2c = to_i2c_client(st->dev);
	s32 val;

	val = i2c_smbus_read_word_swapped(i2c, AD5593R_MODE_REG_READBACK | reg);
	if (val < 0)
		return (int) val;

	*value = (u16) val;

	return 0;
}

static int ad5593r_gpio_read(struct ad5592r_state *st, u8 *value)
{
	struct i2c_client *i2c = to_i2c_client(st->dev);
	s32 val;

	val = i2c_smbus_read_word_swapped(i2c, AD5593R_MODE_GPIO_READBACK);
	if (val < 0)
		return (int) val;

	*value = (u8) val;

	return 0;
}

static const struct ad5592r_rw_ops ad5593r_rw_ops = {
	.write_dac = ad5593r_write_dac,
	.read_adc = ad5593r_read_adc,
	.reg_write = ad5593r_reg_write,
	.reg_read = ad5593r_reg_read,
	.gpio_read = ad5593r_gpio_read,
};

static void ad5593r_check_new_channel_mode(void)
{
	char *new_mode, tmp[2];
	u8 new_ch_modes[AD559XR_CHANNEL_NR];
	int idx;

	if (strlen(ch_mode) != AD559XR_CHANNEL_NR)
		return;

	new_mode = ch_mode;

	/* Check if all channel modes are valid */
	for (idx = 0; idx < AD559XR_CHANNEL_NR; idx++) {
		switch (new_mode[idx]) {
		case '0':
		case '1':
		case '2':
		case '3':
		case '8':
			continue;
		default:
			/* Invalid setting exist, ignore the settings */
			return;
		}
	}

	/* Set the new modes to ad5592r-base driver to setup the new channel modes */
	memset(tmp, 0, 2);
	for (idx = 0; idx < AD559XR_CHANNEL_NR; idx++) {
		tmp[0] = new_mode[idx];
		if (kstrtou8(tmp, 10, &new_ch_modes[AD559XR_CHANNEL_NR - idx - 1])) {
			/* Converting error, ignore the settings */
			return;
		}
	}

	ad5592r_update_default_channel_modes(new_ch_modes);
}

static int ad5593r_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	ad5593r_check_new_channel_mode();
	return ad5592r_probe(&i2c->dev, id->name, &ad5593r_rw_ops);
}

static int ad5593r_i2c_remove(struct i2c_client *i2c)
{
	return ad5592r_remove(&i2c->dev);
}

static const struct i2c_device_id ad5593r_i2c_ids[] = {
	{ .name = "ad5593r", },
	{},
};
MODULE_DEVICE_TABLE(i2c, ad5593r_i2c_ids);

static const struct of_device_id ad5593r_of_match[] = {
	{ .compatible = "adi,ad5593r", },
	{},
};
MODULE_DEVICE_TABLE(of, ad5593r_of_match);

static const struct acpi_device_id ad5593r_acpi_match[] = {
	{"ADS5593", },
	{ },
};
MODULE_DEVICE_TABLE(acpi, ad5593r_acpi_match);

static struct i2c_driver ad5593r_driver = {
	.driver = {
		.name = "ad5593r",
		.of_match_table = of_match_ptr(ad5593r_of_match),
		.acpi_match_table = ACPI_PTR(ad5593r_acpi_match),
	},
	.probe = ad5593r_i2c_probe,
	.remove = ad5593r_i2c_remove,
	.id_table = ad5593r_i2c_ids,
};
module_i2c_driver(ad5593r_driver);

MODULE_AUTHOR("Paul Cercueil <paul.cercueil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5593R multi-channel converters");
MODULE_LICENSE("GPL v2");
