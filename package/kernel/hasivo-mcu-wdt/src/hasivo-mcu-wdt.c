// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hasivo MCU Watchdog Driver
 *
 * Hardware watchdog driver for the external management MCU found on
 * Hasivo/Horaco network switches. Communicates over I2C.
 *
 * Copyright (C) 2026 OpenWrt contributors
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/watchdog.h>

/* Device identification */
#define HASIVO_MCU_ID_VALUE		0x91

/* Device ID register address depends on I2C address variant */
#define HASIVO_0x6F_REG_DEVICE_ID	0xfd
#define HASIVO_0x6E_REG_DEVICE_ID	0x66

/* Watchdog registers (same address on both variants) */
#define HASIVO_REG_WDT_STEP1		0x09
#define HASIVO_REG_WDT_STEP2		0x0a
#define HASIVO_REG_WDT_KEEPALIVE	0x0b

/* Watchdog command values */
#define HASIVO_WDT_STEP1_MAGIC		0x4f
#define HASIVO_WDT_CMD_OPEN		0x4f
#define HASIVO_WDT_CMD_CLOSE		0x3f
#define HASIVO_WDT_KEEPALIVE_MAGIC	0xee

/*
 * Watchdog timeout is fixed in MCU firmware, not software-configurable.
 * Vendor uses 2-second keepalive interval. Timeout is estimated 10-30s.
 */
#define HASIVO_WDT_TIMEOUT		20
#define HASIVO_WDT_KEEPALIVE_INTERVAL	2

struct hasivo_mcu {
	struct i2c_client *client;
	struct watchdog_device wdd;
};

static int hasivo_mcu_wdt_send(struct hasivo_mcu *mcu, u8 cmd)
{
	int ret;

	ret = i2c_smbus_write_byte_data(mcu->client, HASIVO_REG_WDT_STEP1,
					HASIVO_WDT_STEP1_MAGIC);
	if (ret)
		return ret;

	return i2c_smbus_write_byte_data(mcu->client, HASIVO_REG_WDT_STEP2,
					 cmd);
}

static int hasivo_mcu_wdt_start(struct watchdog_device *wdd)
{
	struct hasivo_mcu *mcu = watchdog_get_drvdata(wdd);

	return hasivo_mcu_wdt_send(mcu, HASIVO_WDT_CMD_OPEN);
}

static int hasivo_mcu_wdt_stop(struct watchdog_device *wdd)
{
	struct hasivo_mcu *mcu = watchdog_get_drvdata(wdd);

	return hasivo_mcu_wdt_send(mcu, HASIVO_WDT_CMD_CLOSE);
}

static int hasivo_mcu_wdt_ping(struct watchdog_device *wdd)
{
	struct hasivo_mcu *mcu = watchdog_get_drvdata(wdd);

	return i2c_smbus_write_byte_data(mcu->client,
					 HASIVO_REG_WDT_KEEPALIVE,
					 HASIVO_WDT_KEEPALIVE_MAGIC);
}

static const struct watchdog_info hasivo_mcu_wdt_info = {
	.identity	= "Hasivo MCU Watchdog",
	.options	= WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops hasivo_mcu_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= hasivo_mcu_wdt_start,
	.stop		= hasivo_mcu_wdt_stop,
	.ping		= hasivo_mcu_wdt_ping,
};

static int hasivo_mcu_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct hasivo_mcu *mcu;
	u8 reg_id;
	int ret;

	mcu = devm_kzalloc(dev, sizeof(*mcu), GFP_KERNEL);
	if (!mcu)
		return -ENOMEM;

	mcu->client = client;
	i2c_set_clientdata(client, mcu);

	/* Select register map based on I2C address */
	if (client->addr == 0x6f)
		reg_id = HASIVO_0x6F_REG_DEVICE_ID;
	else
		reg_id = HASIVO_0x6E_REG_DEVICE_ID;

	/* Verify MCU presence by reading device ID */
	ret = i2c_smbus_read_byte_data(client, reg_id);
	if (ret < 0) {
		dev_err(dev, "failed to read device ID register: %d\n", ret);
		return ret;
	}

	if (ret != HASIVO_MCU_ID_VALUE) {
		dev_err(dev, "unexpected device ID: 0x%02x (expected 0x%02x)\n",
			ret, HASIVO_MCU_ID_VALUE);
		return -ENODEV;
	}

	/* Disable the hardware watchdog to prevent unwanted resets */
	ret = hasivo_mcu_wdt_send(mcu, HASIVO_WDT_CMD_CLOSE);
	if (ret) {
		dev_err(dev, "failed to disable watchdog: %d\n", ret);
		return ret;
	}

	dev_info(dev, "Hasivo MCU found at 0x%02x, watchdog disabled\n",
		 client->addr);

	/* Register watchdog device for optional userspace use */
	mcu->wdd.info = &hasivo_mcu_wdt_info;
	mcu->wdd.ops = &hasivo_mcu_wdt_ops;
	mcu->wdd.parent = dev;
	mcu->wdd.timeout = HASIVO_WDT_TIMEOUT;
	mcu->wdd.min_timeout = HASIVO_WDT_KEEPALIVE_INTERVAL;
	mcu->wdd.max_timeout = HASIVO_WDT_TIMEOUT;

	watchdog_set_drvdata(&mcu->wdd, mcu);
	watchdog_stop_on_unregister(&mcu->wdd);

	ret = devm_watchdog_register_device(dev, &mcu->wdd);
	if (ret) {
		dev_err(dev, "failed to register watchdog: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id hasivo_mcu_of_match[] = {
	{ .compatible = "hasivo,mcu-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, hasivo_mcu_of_match);

static const struct i2c_device_id hasivo_mcu_id[] = {
	{ "hasivo-mcu-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hasivo_mcu_id);

static struct i2c_driver hasivo_mcu_driver = {
	.driver = {
		.name		= "hasivo-mcu-wdt",
		.of_match_table	= hasivo_mcu_of_match,
	},
	.probe		= hasivo_mcu_probe,
	.id_table	= hasivo_mcu_id,
};
module_i2c_driver(hasivo_mcu_driver);

MODULE_DESCRIPTION("Hasivo MCU Watchdog Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("OpenWrt contributors");
