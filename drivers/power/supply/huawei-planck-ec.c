// SPDX-License-Identifier: GPL-2.0-only
/* 
 * HUAWEI MateBook E (2019) embedded controller
 * Copyright (c) 2024, NewWheat
 *
 * Based on acer-aspire1-ec.c
 * Copyright (c) 2024, Nikita Travkin <nikita@trvn.ru>
 */
#include <asm-generic/unaligned.h>
#include <linux/backlight.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/string.h>
#include <linux/workqueue_types.h>

#define PLANCK_EC_EVENT			0x31

#define PLANCK_EC_EVENT_LID_OPEN		0x20
#define PLANCK_EC_EVENT_LID_CLOSE		0x21
#define PLANCK_EC_EVENT_FG_CHARGE		0x22
#define PLANCK_EC_EVENT_FG_DISCHARGE	0x23
#define PLANCK_EC_EVENT_FG_CHG_0x30		0x30
#define PLANCK_EC_EVENT_FG_CHG_0x31		0x31
#define PLANCK_EC_EVENT_FG_CHG_0x32		0x32

#define PLANCK_EC_RAM_READ		0x21
#define PLANCK_EC_RAM_WRITE		0x20

#define PLANCK_EC_FG_OEM		0x8c
#define PLANCK_EC_FG_DATA		0x80
#define PLANCK_EC_FG_STATE		0xdd

#define PLANCK_EC_ADP_STATE		0xdb
#define PLANCK_EC_AC_STATUS		BIT(0)

#define PLANCK_EC_BACKLIGHT		0xff

#define PLANCK_EC_FG_FLAG_DISCHARGING	BIT(0)
#define PLANCK_EC_FG_FLAG_CHARGING		BIT(1)

struct planck_ec {
	struct i2c_client *client;
	struct mutex lock;
	struct power_supply *bat_psy;
	struct power_supply *adp_psy;
	struct input_dev *idev;
	struct backlight_device *backlight_dev;
};

static int planck_ec_ram_read(struct i2c_client *client, u8 off, u8 *data, u8 data_len)
{
	i2c_smbus_write_byte_data(client, PLANCK_EC_RAM_READ, off);
	i2c_smbus_read_i2c_block_data(client, PLANCK_EC_RAM_READ, data_len, data);
	return 0;
}

static int planck_ec_ram_write(struct i2c_client *client, u8 off, u8 data)
{
	u8 tmp[2] = {off, data};

	i2c_smbus_write_i2c_block_data(client, PLANCK_EC_RAM_WRITE, sizeof(tmp), tmp);
	return 0;
}

static irqreturn_t planck_ec_irq_handler(int irq, void *data)
{
	struct planck_ec *ec = data;
	int id;

	usleep_range(15000, 30000);

	mutex_lock(&ec->lock);
	id = i2c_smbus_read_byte_data(ec->client, PLANCK_EC_EVENT);
	mutex_unlock(&ec->lock);

	if (id < 0) {
		dev_err(&ec->client->dev, "Failed to read event id: %pe\n", ERR_PTR(id));
		return IRQ_HANDLED;
	}

	switch (id) {
	case 0x0: /* No event */
		break;

	case PLANCK_EC_EVENT_LID_CLOSE:
		input_report_switch(ec->idev, SW_LID, 1);
		input_sync(ec->idev);
		dev_info(&ec->client->dev, "LID_CLOSE event triggered\n");
		break;

	case PLANCK_EC_EVENT_LID_OPEN:
		input_report_switch(ec->idev, SW_LID, 0);
		input_sync(ec->idev);
		dev_info(&ec->client->dev, "LID_OPEN event triggered\n");
		break;

	case PLANCK_EC_EVENT_FG_CHARGE:
		power_supply_changed(ec->bat_psy);
		power_supply_changed(ec->adp_psy);
		dev_info(&ec->client->dev, "PLANCK_EC_EVENT_FG_CHARGE event triggered\n");
		break;

	case PLANCK_EC_EVENT_FG_DISCHARGE:
		power_supply_changed(ec->bat_psy);
		power_supply_changed(ec->adp_psy);
		dev_info(&ec->client->dev, "PLANCK_EC_EVENT_FG_DISCHARGE event triggered\n");
		break;

	case PLANCK_EC_EVENT_FG_CHG_0x30:
		power_supply_changed(ec->bat_psy);
		//power_supply_changed(ec->adp_psy);
		dev_info(&ec->client->dev, "0x31 event triggered\n");
		break;

	case PLANCK_EC_EVENT_FG_CHG_0x31:
		power_supply_changed(ec->bat_psy);
		//power_supply_changed(ec->adp_psy);
		dev_info(&ec->client->dev, "0x31 event triggered\n");
		break;

	case PLANCK_EC_EVENT_FG_CHG_0x32:
		power_supply_changed(ec->bat_psy);
		//power_supply_changed(ec->adp_psy);
		dev_info(&ec->client->dev, "0x32 event triggered\n");
		break;

	case 0x10: /* Unknown event (maybe HPD?) */
		dev_info(&ec->client->dev, "event 0x10 triggered\n");
		break;

	case 0x40: /* Unknown event (maybe HPD?) */
		dev_info(&ec->client->dev, "event 0x40 triggered\n");
		break;

	default:
		dev_warn(&ec->client->dev, "Unknown event id: 0x%x\n", id);
	}

	return IRQ_HANDLED;
}

struct planck_ec_psy_data {
	__le16 null1;
	__le16 serial_number;
	__le16 design_capacity;
	__le16 design_voltage;
	__le16 null2;
	__le16 null3;
	__le16 null4;
	__le16 null5;
	__le16 voltage_now;
	__le16 current_now;
	__le16 last_full_capacity;
	__le16 capacity_now;
	__le16 null6;
} __packed;

static const char * const planck_ec_bat_psy_battery_oem[] = {
	"DYNAPACK",
	"Sunwoda-S",
	"Unknown",
};

static int planck_ec_bat_psy_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct planck_ec *ec = power_supply_get_drvdata(psy);
	struct planck_ec_psy_data data;
	int str_index = 0;
	char serial_number[20];
	u8 oem;
	u8 state;

	mutex_lock(&ec->lock);
	planck_ec_ram_read(ec->client, PLANCK_EC_FG_DATA, (u8 *)&data, sizeof(data));
	planck_ec_ram_read(ec->client, PLANCK_EC_FG_OEM, &oem, sizeof(oem));
	planck_ec_ram_read(ec->client, PLANCK_EC_FG_STATE, &state, sizeof(state));
	mutex_unlock(&ec->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = le16_to_cpu(data.voltage_now) * 1000;
		//dev_info(&ec->client->dev, "voltage_now: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = le16_to_cpu(data.design_voltage) * 1000;
		//dev_info(&ec->client->dev, "design_voltage: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = le16_to_cpu(data.capacity_now) * 1000;
		//dev_info(&ec->client->dev, "capacity_now: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = le16_to_cpu(data.last_full_capacity) * 1000;
		//dev_info(&ec->client->dev, "last_full_capacity: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = le16_to_cpu(data.design_capacity) * 1000;
		//dev_info(&ec->client->dev, "design_capacity: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = le16_to_cpu(data.capacity_now) * 100
			      / le16_to_cpu(data.last_full_capacity);
		//dev_info(&ec->client->dev, "battery percentage: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = (s16)le16_to_cpu(data.current_now) * 1000;
		//dev_info(&ec->client->dev, "current_now: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;

	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "HB30C4J7ECW-21";
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		str_index = oem - 1;

		if (str_index >= 0 && str_index < ARRAY_SIZE(planck_ec_bat_psy_battery_oem))
			val->strval = planck_ec_bat_psy_battery_oem[str_index];
		else {
			dev_err(&ec->client->dev, "battery oem unknown: %d\n", str_index);
			val->strval = "Unknown";
		}
		break;

	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		snprintf(serial_number, sizeof(serial_number), "%d", data.serial_number);
		val->strval = kasprintf(GFP_KERNEL, "%s", serial_number);
		if (!val->strval)
			return -ENOMEM;
		//dev_info(&ec->client->dev, "serial_number: %d\n", data.serial_number);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		if (state & PLANCK_EC_FG_FLAG_DISCHARGING)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state & PLANCK_EC_FG_FLAG_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_FULL;
		//dev_info(&ec->client->dev, "battery_status_now: %d\n", val->intval);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property planck_ec_bat_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc planck_ec_bat_psy_desc = {
	.name		= "planck-ec-bat",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= planck_ec_bat_psy_get_property,
	.properties	= planck_ec_bat_psy_props,
	.num_properties	= ARRAY_SIZE(planck_ec_bat_psy_props),
};

static int planck_ec_adp_psy_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct planck_ec *ec = power_supply_get_drvdata(psy);
	u8 tmp;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		mutex_lock(&ec->lock);
		planck_ec_ram_read(ec->client, PLANCK_EC_ADP_STATE, &tmp, sizeof(tmp));
		mutex_unlock(&ec->lock);
		val->intval = !!(tmp & PLANCK_EC_AC_STATUS);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property planck_ec_adp_psy_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc planck_ec_adp_psy_desc = {
	.name		= "planck-ec-adp",
	.type		= POWER_SUPPLY_TYPE_USB_TYPE_C,
	.get_property	= planck_ec_adp_psy_get_property,
	.properties	= planck_ec_adp_psy_props,
	.num_properties	= ARRAY_SIZE(planck_ec_adp_psy_props),
};

static void planck_ec_backlight_set_brightness(struct planck_ec *ec, int brightness)
{
	mutex_lock(&ec->lock);
   	planck_ec_ram_write(ec->client, PLANCK_EC_BACKLIGHT, brightness);
	mutex_unlock(&ec->lock);
}

static int planck_ec_backlight_update_status(struct backlight_device *bl)
{
	struct planck_ec *ec = bl_get_data(bl);
	int brightness = backlight_get_brightness(bl);

	planck_ec_backlight_set_brightness(ec, brightness);

	return 0;
}

static const struct backlight_ops planck_ec_backlight_ops = {
    .update_status = planck_ec_backlight_update_status,
};

static const struct backlight_properties planck_ec_backlight_props = {
	.type = BACKLIGHT_RAW,
	.scale = BACKLIGHT_SCALE_LINEAR,
	.max_brightness = 255,
	.brightness = 128,
};

static int planck_ec_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {0};
	struct device *dev = &client->dev;
	struct planck_ec *ec;
	int ret;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	mutex_init(&ec->lock);
	ec->client = client;
	i2c_set_clientdata(client, ec);

	/* Battery status reports */
	psy_cfg.drv_data = ec;
	ec->bat_psy = devm_power_supply_register(dev, &planck_ec_bat_psy_desc, &psy_cfg);
	if (IS_ERR(ec->bat_psy))
		return dev_err_probe(dev, PTR_ERR(ec->bat_psy),
				     "Failed to register battery power supply\n");

	ec->adp_psy = devm_power_supply_register(dev, &planck_ec_adp_psy_desc, &psy_cfg);
	if (IS_ERR(ec->adp_psy))
		return dev_err_probe(dev, PTR_ERR(ec->adp_psy),
				     "Failed to register AC power supply\n");

	/* Lid switch */
	ec->idev = devm_input_allocate_device(dev);
	if (!ec->idev)
		return -ENOMEM;

	ec->idev->name = "planck-ec";
	ec->idev->phys = "planck-ec/input0";
	input_set_capability(ec->idev, EV_SW, SW_LID);

	ret = input_register_device(ec->idev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register input device\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					planck_ec_irq_handler, IRQF_ONESHOT,
					dev_name(dev), ec);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request irq\n");

	/* Backlight */
	ec->backlight_dev = devm_backlight_device_register(&client->dev, "planck-ec-backlight",
										&client->dev, ec, &planck_ec_backlight_ops, &planck_ec_backlight_props);

    if (IS_ERR(ec->backlight_dev))
		return dev_err_probe(dev, PTR_ERR(ec->backlight_dev), 
					 "Failed to register backlight device\n");

	backlight_update_status(ec->backlight_dev);

	return 0;
}

static const struct i2c_device_id planck_ec_id[] = {
	{ "huawei-planck-ec", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, planck_ec_id);

static const struct of_device_id planck_ec_of_match[] = {
	{ .compatible = "huawei,planck-ec", },
	{ }
};
MODULE_DEVICE_TABLE(of, planck_ec_of_match);

static struct i2c_driver planck_ec_driver = {
	.driver = {
		.name = "huawei-planck-ec",
		.of_match_table = planck_ec_of_match,
		//.dev_groups = planck_ec_groups,
	},
	.probe = planck_ec_probe,
	.id_table = planck_ec_id,
};
module_i2c_driver(planck_ec_driver);

MODULE_DESCRIPTION("HUAWEI MateBook E (2019) embedded controller");
MODULE_AUTHOR("NewWheat");
MODULE_LICENSE("GPL");
