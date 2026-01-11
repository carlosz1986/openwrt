// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hasivo STC8 MFD driver for I2C-controlled LED registers
 *
 * This driver provides LED control for POE status indicators.
 * LED configuration is parsed from device tree child nodes.
 *
 * Some registers require an "execute bit" to be set when writing.
 * This is configured via DT properties:
 *   - hasivo,execute-bit: the bit value (default 0x40)
 *   - hasivo,execute-bit-registers: list of registers needing this bit
 */

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

struct stc8_led {
	struct led_classdev	cdev;
	struct stc8_mfd		*mfd;
	u8			reg;
	u8			mask;
};

struct stc8_mfd {
	struct device		*dev;
	struct regmap		*regmap;
	u32			exec_bit;
	u32			*exec_regs;
	size_t			num_exec_regs;
	struct stc8_led		*leds;
	size_t			num_leds;
};

static bool stc8_is_exec_reg(struct stc8_mfd *mfd, unsigned int reg)
{
	size_t i;

	for (i = 0; i < mfd->num_exec_regs; i++) {
		if (mfd->exec_regs[i] == reg)
			return true;
	}
	return false;
}

static int stc8_led_set(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct stc8_led *led = container_of(cdev, struct stc8_led, cdev);
	struct stc8_mfd *mfd = led->mfd;
	unsigned int val;
	int ret;

	ret = regmap_read(mfd->regmap, led->reg, &val);
	if (ret)
		return ret;

	if (brightness)
		val |= led->mask;
	else
		val &= ~led->mask;

	if (stc8_is_exec_reg(mfd, led->reg))
		val |= mfd->exec_bit;

	return regmap_write(mfd->regmap, led->reg, val);
}

static enum led_brightness stc8_led_get(struct led_classdev *cdev)
{
	struct stc8_led *led = container_of(cdev, struct stc8_led, cdev);
	unsigned int val;

	if (regmap_read(led->mfd->regmap, led->reg, &val))
		return 0;

	return (val & led->mask) ? LED_ON : LED_OFF;
}

static int stc8_parse_exec_regs(struct stc8_mfd *mfd, struct device_node *np)
{
	int count;

	mfd->exec_bit = 0x40;
	of_property_read_u32(np, "hasivo,execute-bit", &mfd->exec_bit);

	count = of_property_count_u32_elems(np, "hasivo,execute-bit-registers");
	if (count <= 0)
		return 0;

	mfd->exec_regs = devm_kcalloc(mfd->dev, count, sizeof(u32), GFP_KERNEL);
	if (!mfd->exec_regs)
		return -ENOMEM;

	mfd->num_exec_regs = count;
	return of_property_read_u32_array(np, "hasivo,execute-bit-registers",
					  mfd->exec_regs, count);
}

static int stc8_register_leds(struct stc8_mfd *mfd, struct device_node *np)
{
	struct device_node *child;
	struct stc8_led *led;
	const char *str;
	u32 reg_mask[2];
	int num_leds = 0;
	int ret;

	/* Count valid LED child nodes */
	for_each_child_of_node(np, child) {
		if (!of_property_read_u32_array(child, "reg", reg_mask, 2))
			num_leds++;
	}

	if (num_leds == 0)
		return 0;

	mfd->leds = devm_kcalloc(mfd->dev, num_leds, sizeof(*mfd->leds),
				 GFP_KERNEL);
	if (!mfd->leds)
		return -ENOMEM;

	mfd->num_leds = num_leds;
	led = mfd->leds;

	for_each_child_of_node(np, child) {
		if (of_property_read_u32_array(child, "reg", reg_mask, 2))
			continue;

		if (of_property_read_string(child, "label", &str)) {
			dev_err(mfd->dev, "LED node missing 'label'\n");
			of_node_put(child);
			return -EINVAL;
		}

		led->mfd = mfd;
		led->reg = reg_mask[0];
		led->mask = reg_mask[1];
		led->cdev.name = str;
		led->cdev.max_brightness = 1;
		led->cdev.brightness_set_blocking = stc8_led_set;
		led->cdev.brightness_get = stc8_led_get;

		if (!of_property_read_string(child, "linux,default-trigger", &str))
			led->cdev.default_trigger = str;

		ret = devm_led_classdev_register(mfd->dev, &led->cdev);
		if (ret) {
			dev_err(mfd->dev, "Failed to register LED %s\n",
				led->cdev.name);
			of_node_put(child);
			return ret;
		}

		dev_dbg(mfd->dev, "LED %s: reg=0x%02x mask=0x%02x\n",
			led->cdev.name, led->reg, led->mask);
		led++;
	}

	dev_dbg(mfd->dev, "Registered %zu LEDs\n", mfd->num_leds);
	return 0;
}

static const struct regmap_config stc8_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int stc8_probe(struct i2c_client *client)
{
	struct stc8_mfd *mfd;
	int ret;

	mfd = devm_kzalloc(&client->dev, sizeof(*mfd), GFP_KERNEL);
	if (!mfd)
		return -ENOMEM;

	mfd->dev = &client->dev;
	i2c_set_clientdata(client, mfd);

	mfd->regmap = devm_regmap_init_i2c(client, &stc8_regmap_config);
	if (IS_ERR(mfd->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(mfd->regmap),
				     "Failed to init regmap\n");

	ret = stc8_parse_exec_regs(mfd, client->dev.of_node);
	if (ret)
		return ret;

	ret = stc8_register_leds(mfd, client->dev.of_node);
	if (ret)
		return ret;

	dev_dbg(&client->dev, "STC8 MFD initialized\n");
	return 0;
}

static const struct of_device_id stc8_of_match[] = {
	{ .compatible = "hasivo,stc8-mfd" },
	{ }
};
MODULE_DEVICE_TABLE(of, stc8_of_match);

static struct i2c_driver stc8_driver = {
	.driver = {
		.name = "hasivo-stc8-mfd",
		.of_match_table = stc8_of_match,
	},
	.probe = stc8_probe,
};
module_i2c_driver(stc8_driver);

MODULE_AUTHOR("Bevan Weiss <bevan.weiss@gmail.com>");
MODULE_DESCRIPTION("Hasivo STC8 MFD driver for POE LED control");
MODULE_LICENSE("GPL");
