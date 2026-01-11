// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hasivo HS104 PoE PSE controller driver
 *
 * 4-port IEEE 802.3af/at/bt PoE PSE controller with I2C interface.
 * Supports power monitoring, per-port enable/disable, and LED triggers
 * for power delivery status indication.
 *
 * Copyright (c) 2025 Bevan Weiss <bevan.weiss@gmail.com>
 */

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pse-pd/pse.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define HS104_MAX_PORTS		4
#define HS104_POLL_INTERVAL_MS	500

/* Register map */
#define HS104_REG_PW_STATUS	0x01		/* Power delivery status */
#define HS104_REG_INPUT_V	0x02		/* Input voltage, 16-bit BE, 10mV */
#define HS104_REG_PORT0_I	0x04		/* Port current, 16-bit BE, 1mA */
#define HS104_REG_DEVID		0x0C		/* Device ID */
#define HS104_REG_PORT0_CLASS	0x0D		/* Port power class */
#define HS104_REG_PW_EN		0x14		/* Port enable control */
#define HS104_REG_PROTOCOL	0x19		/* Protocol per port (2 bits each) */
#define HS104_REG_TOTAL_POWER	0x1D		/* Total power, 16-bit BE, 10mW */
#define HS104_REG_PORT0_POWER	0x21		/* Port power, 16-bit BE, 10mW */

#define HS104_DEVICE_ID		0x91
#define HS104_EXECUTE		0x40		/* Execute bit for writes */

/* Protocol encoding (2 bits per port in PROTOCOL register) */
#define HS104_PROTO_MASK	0x3
#define HS104_PROTO_BT		0		/* 802.3bt - 60W */
#define HS104_PROTO_HIPO	1		/* Hi-PoE - 90W */
#define HS104_PROTO_AT		2		/* 802.3at - 30W */
#define HS104_PROTO_AF		3		/* 802.3af - 15.4W */

/* Power limits in mW */
#define HS104_PW_AF		15400
#define HS104_PW_AT		30000
#define HS104_PW_BT		60000
#define HS104_PW_HIPO		90000

/* Unit conversion steps */
#define HS104_UV_STEP		10000		/* 10mV -> uV */
#define HS104_UA_STEP		1000		/* 1mA -> uA */
#define HS104_MW_STEP		10		/* 10mW -> mW */

/*
 * Port bit mapping is reversed: port 0 uses bit 3, port 1 uses bit 2, etc.
 * This matches the hardware register layout.
 */
#define HS104_PORT_BIT(p)	BIT(3 - (p))

struct hs104_port {
	struct led_trigger	trig_delivering;
	struct led_trigger	trig_enabled;
	bool			delivering;
	bool			enabled;
};

struct hs104_priv {
	struct device		*dev;
	struct regmap		*regmap;
	struct delayed_work	poll_work;
	struct hs104_port	ports[HS104_MAX_PORTS];
};

static inline struct hs104_priv *to_hs104(struct pse_controller_dev *pcdev)
{
	return dev_get_drvdata(pcdev->dev);
}

/* Read 16-bit big-endian register and apply unit conversion */
static int hs104_read_be16(struct hs104_priv *priv, unsigned int reg,
			   unsigned int step)
{
	__be16 val;
	int ret;

	ret = regmap_bulk_read(priv->regmap, reg, &val, sizeof(val));
	if (ret)
		return ret;

	return (be16_to_cpu(val) & 0x3fff) * step;
}

/* Convert protocol code to power limit in mW */
static int hs104_proto_to_mw(unsigned int proto)
{
	switch (proto) {
	case HS104_PROTO_AF:	return HS104_PW_AF;
	case HS104_PROTO_AT:	return HS104_PW_AT;
	case HS104_PROTO_BT:	return HS104_PW_BT;
	case HS104_PROTO_HIPO:	return HS104_PW_HIPO;
	default:		return 0;
	}
}

static void hs104_update_led(struct led_trigger *trig, bool *state, bool new)
{
	if (*state == new)
		return;

	*state = new;
	led_trigger_event(trig, new ? LED_FULL : LED_OFF);
}

static void hs104_poll_work(struct work_struct *work)
{
	struct hs104_priv *priv = container_of(work, struct hs104_priv,
					       poll_work.work);
	unsigned int pw_en, pw_status;
	int i;

	if (regmap_read(priv->regmap, HS104_REG_PW_EN, &pw_en))
		goto resched;
	if (regmap_read(priv->regmap, HS104_REG_PW_STATUS, &pw_status))
		goto resched;

	for (i = 0; i < HS104_MAX_PORTS; i++) {
		struct hs104_port *port = &priv->ports[i];
		bool enabled = !!(pw_en & HS104_PORT_BIT(i));
		bool delivering = !!(pw_status & HS104_PORT_BIT(i));

		hs104_update_led(&port->trig_enabled, &port->enabled, enabled);
		hs104_update_led(&port->trig_delivering, &port->delivering,
				 delivering);
	}

resched:
	schedule_delayed_work(&priv->poll_work,
			      msecs_to_jiffies(HS104_POLL_INTERVAL_MS));
}

/* PSE controller operations */

static int hs104_pi_enable(struct pse_controller_dev *pcdev, int port)
{
	struct hs104_priv *priv = to_hs104(pcdev);
	unsigned int val;
	int ret;

	if (port < 0 || port >= HS104_MAX_PORTS)
		return -EINVAL;

	ret = regmap_read(priv->regmap, HS104_REG_PW_EN, &val);
	if (ret)
		return ret;

	val = (val | HS104_PORT_BIT(port)) | HS104_EXECUTE;

	dev_dbg(pcdev->dev, "port %d: enable (0x%02x)\n", port, val);
	return regmap_write(priv->regmap, HS104_REG_PW_EN, val);
}

static int hs104_pi_disable(struct pse_controller_dev *pcdev, int port)
{
	struct hs104_priv *priv = to_hs104(pcdev);
	unsigned int val;
	int ret;

	if (port < 0 || port >= HS104_MAX_PORTS)
		return -EINVAL;

	ret = regmap_read(priv->regmap, HS104_REG_PW_EN, &val);
	if (ret)
		return ret;

	val = (val & ~HS104_PORT_BIT(port)) | HS104_EXECUTE;

	dev_dbg(pcdev->dev, "port %d: disable (0x%02x)\n", port, val);
	return regmap_write(priv->regmap, HS104_REG_PW_EN, val);
}

static int hs104_pi_is_enabled(struct pse_controller_dev *pcdev, int port)
{
	struct hs104_priv *priv = to_hs104(pcdev);
	unsigned int val;
	int ret;

	if (port < 0 || port >= HS104_MAX_PORTS)
		return -EINVAL;

	ret = regmap_read(priv->regmap, HS104_REG_PW_EN, &val);
	if (ret)
		return ret;

	return !!(val & HS104_PORT_BIT(port));
}

static int hs104_pi_get_voltage(struct pse_controller_dev *pcdev, int port)
{
	struct hs104_priv *priv = to_hs104(pcdev);

	/* Input voltage is shared across all ports */
	return hs104_read_be16(priv, HS104_REG_INPUT_V, HS104_UV_STEP);
}

static int hs104_pi_get_pw_limit(struct pse_controller_dev *pcdev, int port)
{
	struct hs104_priv *priv = to_hs104(pcdev);
	unsigned int val, proto;
	int ret;

	if (port < 0 || port >= HS104_MAX_PORTS)
		return -EINVAL;

	ret = regmap_read(priv->regmap, HS104_REG_PROTOCOL, &val);
	if (ret)
		return ret;

	proto = (val >> (port * 2)) & HS104_PROTO_MASK;
	return hs104_proto_to_mw(proto) ?: -ENODATA;
}

static int hs104_pi_set_pw_limit(struct pse_controller_dev *pcdev,
				 int port, int max_mw)
{
	struct hs104_priv *priv = to_hs104(pcdev);
	unsigned int proto, mask;

	if (port < 0 || port >= HS104_MAX_PORTS)
		return -EINVAL;

	if (max_mw <= HS104_PW_AF)
		proto = HS104_PROTO_AF;
	else if (max_mw <= HS104_PW_AT)
		proto = HS104_PROTO_AT;
	else if (max_mw <= HS104_PW_BT)
		proto = HS104_PROTO_BT;
	else if (max_mw <= HS104_PW_HIPO)
		proto = HS104_PROTO_HIPO;
	else
		return -EINVAL;

	mask = HS104_PROTO_MASK << (port * 2);
	proto <<= (port * 2);

	return regmap_update_bits(priv->regmap, HS104_REG_PROTOCOL, mask, proto);
}

static const struct ethtool_c33_pse_pw_limit_range hs104_pw_ranges[] = {
	{ .min = HS104_PW_AF,   .max = HS104_PW_AF },
	{ .min = HS104_PW_AT,   .max = HS104_PW_AT },
	{ .min = HS104_PW_BT,   .max = HS104_PW_BT },
	{ .min = HS104_PW_HIPO, .max = HS104_PW_HIPO },
};

static int hs104_ethtool_get_status(struct pse_controller_dev *pcdev,
				    unsigned long port,
				    struct netlink_ext_ack *extack,
				    struct pse_control_status *st)
{
	struct hs104_priv *priv = to_hs104(pcdev);
	unsigned int val, proto;
	bool enabled, delivering;
	int ret;

	if (port >= HS104_MAX_PORTS)
		return -EINVAL;

	/* Admin state */
	ret = regmap_read(priv->regmap, HS104_REG_PW_EN, &val);
	if (ret)
		return ret;

	enabled = !!(val & HS104_PORT_BIT(port));
	st->c33_admin_state = enabled ? ETHTOOL_C33_PSE_ADMIN_STATE_ENABLED :
					ETHTOOL_C33_PSE_ADMIN_STATE_DISABLED;

	/* Power delivery status */
	ret = regmap_read(priv->regmap, HS104_REG_PW_STATUS, &val);
	if (ret)
		return ret;

	delivering = !!(val & HS104_PORT_BIT(port));
	st->c33_pw_status = delivering ? ETHTOOL_C33_PSE_PW_D_STATUS_DELIVERING :
					 ETHTOOL_C33_PSE_PW_D_STATUS_DISABLED;

	/* Power class */
	ret = regmap_read(priv->regmap, HS104_REG_PORT0_CLASS + port, &val);
	if (ret)
		return ret;
	st->c33_pw_class = val;

	/* Actual power consumption */
	ret = hs104_read_be16(priv, HS104_REG_PORT0_POWER + port * 2,
			      HS104_MW_STEP);
	if (ret < 0)
		return ret;
	st->c33_actual_pw = ret;

	/* Power limit from protocol setting */
	ret = regmap_read(priv->regmap, HS104_REG_PROTOCOL, &val);
	if (ret)
		return ret;

	proto = (val >> (port * 2)) & HS104_PROTO_MASK;
	st->c33_avail_pw_limit = hs104_proto_to_mw(proto);

	/* Available power limit ranges */
	st->c33_pw_limit_ranges = kmemdup(hs104_pw_ranges,
					  sizeof(hs104_pw_ranges), GFP_KERNEL);
	if (!st->c33_pw_limit_ranges)
		return -ENOMEM;

	st->c33_pw_limit_nb_ranges = ARRAY_SIZE(hs104_pw_ranges);

	return 0;
}

static const struct pse_controller_ops hs104_ops = {
	.ethtool_get_status	= hs104_ethtool_get_status,
	.pi_enable		= hs104_pi_enable,
	.pi_disable		= hs104_pi_disable,
	.pi_is_enabled		= hs104_pi_is_enabled,
	.pi_get_voltage		= hs104_pi_get_voltage,
	.pi_get_pw_limit	= hs104_pi_get_pw_limit,
	.pi_set_pw_limit	= hs104_pi_set_pw_limit,
};

/* Sysfs interface for direct port control */

static ssize_t ports_enabled_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct hs104_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, HS104_REG_PW_EN, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%c%c%c%c\n",
			  (val & HS104_PORT_BIT(0)) ? '1' : '0',
			  (val & HS104_PORT_BIT(1)) ? '1' : '0',
			  (val & HS104_PORT_BIT(2)) ? '1' : '0',
			  (val & HS104_PORT_BIT(3)) ? '1' : '0');
}

static ssize_t ports_enabled_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct hs104_priv *priv = dev_get_drvdata(dev);
	unsigned int val = 0;
	int i, ret;

	if (count < 4)
		return -EINVAL;

	for (i = 0; i < 4; i++) {
		if (buf[i] == '1')
			val |= HS104_PORT_BIT(i);
		else if (buf[i] != '0')
			return -EINVAL;
	}

	ret = regmap_write(priv->regmap, HS104_REG_PW_EN, HS104_EXECUTE | val);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(ports_enabled);

static ssize_t ports_delivering_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct hs104_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, HS104_REG_PW_STATUS, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%c%c%c%c\n",
			  (val & HS104_PORT_BIT(0)) ? '1' : '0',
			  (val & HS104_PORT_BIT(1)) ? '1' : '0',
			  (val & HS104_PORT_BIT(2)) ? '1' : '0',
			  (val & HS104_PORT_BIT(3)) ? '1' : '0');
}
static DEVICE_ATTR_RO(ports_delivering);

static struct attribute *hs104_attrs[] = {
	&dev_attr_ports_enabled.attr,
	&dev_attr_ports_delivering.attr,
	NULL,
};
ATTRIBUTE_GROUPS(hs104);

/* Driver initialization */

static const struct regmap_config hs104_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int hs104_init_triggers(struct hs104_priv *priv)
{
	struct device *dev = priv->dev;
	int i, ret;

	for (i = 0; i < HS104_MAX_PORTS; i++) {
		struct hs104_port *port = &priv->ports[i];

		port->trig_delivering.name = devm_kasprintf(dev, GFP_KERNEL,
					"%s:port%d_delivering", dev_name(dev), i);
		port->trig_enabled.name = devm_kasprintf(dev, GFP_KERNEL,
					"%s:port%d_enabled", dev_name(dev), i);

		if (!port->trig_delivering.name || !port->trig_enabled.name)
			return -ENOMEM;

		ret = devm_led_trigger_register(dev, &port->trig_delivering);
		if (ret)
			return ret;

		ret = devm_led_trigger_register(dev, &port->trig_enabled);
		if (ret)
			return ret;
	}

	return 0;
}

static int hs104_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct pse_controller_dev *pcdev;
	struct hs104_priv *priv;
	unsigned int devid;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	pcdev = devm_kzalloc(dev, sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev)
		return -ENOMEM;

	priv->dev = dev;
	dev_set_drvdata(dev, priv);

	priv->regmap = devm_regmap_init_i2c(client, &hs104_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	/* Verify device ID */
	ret = regmap_read(priv->regmap, HS104_REG_DEVID, &devid);
	if (ret)
		return ret;

	if ((devid & 0xff) != HS104_DEVICE_ID) {
		dev_err(dev, "Unknown device ID: 0x%02x\n", devid);
		return -ENODEV;
	}

	/* Initialize LED triggers */
	ret = hs104_init_triggers(priv);
	if (ret)
		return ret;

	/* Register PSE controller */
	pcdev->ops = &hs104_ops;
	pcdev->dev = dev;
	pcdev->owner = THIS_MODULE;
	pcdev->nr_lines = HS104_MAX_PORTS;
	pcdev->of_pse_n_cells = 1;
	pcdev->types = ETHTOOL_PSE_C33;

	ret = devm_pse_controller_register(dev, pcdev);
	if (ret)
		return ret;

	/* Enable all ports by default - HS104 only powers valid PDs */
	ret = regmap_write(priv->regmap, HS104_REG_PW_EN, HS104_EXECUTE | 0x0f);
	if (ret)
		dev_warn(dev, "Failed to enable ports: %d\n", ret);

	/* Start LED status polling */
	INIT_DELAYED_WORK(&priv->poll_work, hs104_poll_work);
	schedule_delayed_work(&priv->poll_work,
			      msecs_to_jiffies(HS104_POLL_INTERVAL_MS));

	dev_dbg(dev, "HS104 PSE controller initialized\n");
	return 0;
}

static void hs104_remove(struct i2c_client *client)
{
	struct hs104_priv *priv = dev_get_drvdata(&client->dev);

	cancel_delayed_work_sync(&priv->poll_work);
}

static const struct of_device_id hs104_of_match[] = {
	{ .compatible = "hasivo,hs104" },
	{ }
};
MODULE_DEVICE_TABLE(of, hs104_of_match);

static struct i2c_driver hs104_driver = {
	.driver = {
		.name		= "hasivo-hs104",
		.of_match_table	= hs104_of_match,
		.dev_groups	= hs104_groups,
	},
	.probe	= hs104_probe,
	.remove	= hs104_remove,
};
module_i2c_driver(hs104_driver);

MODULE_AUTHOR("Bevan Weiss <bevan.weiss@gmail.com>");
MODULE_DESCRIPTION("Hasivo HS104 PoE PSE Controller");
MODULE_LICENSE("GPL");
