// SPDX-License-Identifier: GPL-2.0-only
/* 
 * HUAWEI MateBook E (2019) embedded controller
 * Copyright (c) 2024, NewWheat
 *
 * Based on acer-aspire1-ec.c
 */
#include <asm-generic/unaligned.h>
//#include <drm/drm_bridge.h>
#include <linux/backlight.h>
#include <linux/bits.h>
#include <linux/delay.h>
//#include <linux/gpio.h>
//#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/string.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue_types.h>

#include "../../usb/typec/ucsi/ucsi.h"

#define PLANCK_EC_EVENT			0x31

#define PLANCK_EC_EVENT_UCSI			0x10
#define PLANCK_EC_EVENT_LID_OPEN		0x20
#define PLANCK_EC_EVENT_LID_CLOSE		0x21
#define PLANCK_EC_EVENT_FG_CHARGE		0x22
#define PLANCK_EC_EVENT_FG_DISCHARGE	0x23
#define PLANCK_EC_EVENT_FG_CHG_0x30		0x30
#define PLANCK_EC_EVENT_FG_CHG_0x31		0x31
#define PLANCK_EC_EVENT_FG_CHG_0x32		0x32
#define PLANCK_EC_EVENT_HPD_DETECT		0x40

#define PLANCK_EC_RAM_READ		0x21
#define PLANCK_EC_RAM_WRITE		0x20

#define PLANCK_EC_FG_OEM		0x8c
#define PLANCK_EC_FG_DATA		0x80
#define PLANCK_EC_FG_STATE		0xdd

#define PLANCK_EC_ADP_STATE		0xdb
#define PLANCK_EC_AC_STATUS		BIT(0)

#define PLANCK_EC_FG_FLAG_DISCHARGING	BIT(0)
#define PLANCK_EC_FG_FLAG_CHARGING		BIT(1)

#define PLANCK_EC_BACKLIGHT		0xff
#define PLANCK_EC_HPD_STATUS	0x40

#define PLANCK_EC_UCSI_RAM_READ_REQUEST	0xa2
#define PLANCK_EC_UCSI_RAM_READ_RESPOND	0xa0
#define PLANCK_EC_UCSI_READ		0x0
#define PLANCK_EC_UCSI_WRITE	0xa1

struct planck_ec_ucsi_data {
	u16 version;
	u16 res;
	u8 cci[4];
	u8 control[8];
	u8 in[16];
	u8 out[16];
} __packed;

static int planck_ec_ucsi_update(struct ucsi *ucsi);
static int planck_ec_ucsi_read_cci(struct ucsi *ucsi, u32 *cci);
//static void planck_ec_bridge_update_hpd_work(struct work_struct *work);

struct planck_ec {
	struct i2c_client *client;
	struct mutex lock;
	struct power_supply *bat_psy;
	struct power_supply *adp_psy;
	struct input_dev *idev;
	struct backlight_device *backlight_dev;
//	struct drm_bridge bridge;
	struct gpio_desc *hpd_gpio;
	struct work_struct work;
//	bool bridge_configured;
	struct ucsi *ucsi;
	struct planck_ec_ucsi_data ucsi_data;
	u64 ucsi_cmd;
};

static int planck_ec_ram_read(struct i2c_client *client, u8 off, u8*data, u8 data_len)
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

/* IRQ Handler */

static irqreturn_t planck_ec_irq_handler(int irq, void *data)
{
	struct planck_ec *ec = data;
	int id;
	u32 cci;

	usleep_range(15000, 30000);

	mutex_lock(&ec->lock);
	id = i2c_smbus_read_byte_data(ec->client, PLANCK_EC_EVENT);
	mutex_unlock(&ec->lock);

	if (id < 0) {
		dev_err(&ec->client->dev, "Failed to read event ID: %pe\n", ERR_PTR(id));
		return IRQ_HANDLED;
	}

	switch (id) {
	case 0x0: /* No event */
		break;
	
	case PLANCK_EC_EVENT_UCSI:
		planck_ec_ucsi_update(ec->ucsi);
		planck_ec_ucsi_read_cci(ec->ucsi, &cci);
		ucsi_notify_common(ec->ucsi, cci);
		//dev_info(&ec->client->dev, "UCSI event triggered\n");
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
		dev_info(&ec->client->dev, "FG_CHARGE event triggered\n");
		break;

	case PLANCK_EC_EVENT_FG_DISCHARGE:
		power_supply_changed(ec->bat_psy);
		power_supply_changed(ec->adp_psy);
		dev_info(&ec->client->dev, "FG_DISCHARGE event triggered\n");

		/*
		u8 tmp;
		planck_ec_ram_read(ec->client, planck_EC_HPD_STATUS, &tmp, sizeof(tmp));
		dev_info(&ec->client->dev, "HPD status: %d\n", tmp);
		*/

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

	case PLANCK_EC_EVENT_HPD_DETECT:
		ucsi_connector_change(ec->ucsi, 1);
		dev_info(&ec->client->dev, "HPD_DETECT event triggered\n");

/*		if (ec->bridge_configured)
 *			planck_ec_bridge_update_hpd_work(&ec->work);
 */
		break;

	default:
		dev_warn(&ec->client->dev, "Unknown event ID: 0x%x\n", id);
	}

	return IRQ_HANDLED;
}

/* Power Supply */

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
	__le16 cycle_count;
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
	char serial_number[10];
	u8 oem;
	u8 state;

	mutex_lock(&ec->lock);
	planck_ec_ram_read(ec->client, PLANCK_EC_FG_DATA, (u8*)&data, sizeof(data));
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
			dev_err(&ec->client->dev, "Battery OEM unknown: %d\n", str_index);
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

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = le16_to_cpu(data.cycle_count);
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
	POWER_SUPPLY_PROP_CYCLE_COUNT,
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

/* Backlight */

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

/* USB-C DP Altmode */
/*
static int planck_ec_bridge_attach(struct drm_bridge *bridge, enum drm_bridge_attach_flags flags)
{
	return flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR ? 0 : -EINVAL;
}

static void planck_ec_bridge_update_hpd_work(struct work_struct *work)
{
	struct planck_ec *ec = container_of(work, struct planck_ec, work);
	int hpd_status = gpiod_get_value_cansleep(ec->hpd_gpio);

	dev_info(&ec->client->dev, "HPD status: %d\n", hpd_status);

	if (hpd_status)
		drm_bridge_hpd_notify(&ec->bridge, connector_status_connected);
	else
		drm_bridge_hpd_notify(&ec->bridge, connector_status_disconnected);
}

static void planck_ec_bridge_hpd_enable(struct drm_bridge *bridge)
{
	struct planck_ec *ec = container_of(bridge, struct planck_ec, bridge);

	schedule_work(&ec->work);
}

static const struct drm_bridge_funcs planck_ec_bridge_funcs = {
	.hpd_enable = planck_ec_bridge_hpd_enable,
	.attach = planck_ec_bridge_attach,
};
*/
/* UCSI */

static int planck_ec_ram_usci_read(struct i2c_client *client, u8 off, u8*data, u8 data_len)
{
	i2c_smbus_write_byte_data(client, PLANCK_EC_UCSI_RAM_READ_REQUEST, off);
	i2c_smbus_read_i2c_block_data(client, PLANCK_EC_UCSI_RAM_READ_RESPOND, data_len, data);
	return 0;
}

struct planck_ec_ucsi_in_data {
	u8 version[2];
	u8 null1[2];
	u8 cci[4];
	u8 null2[8];
	u8 msg[16];
} __packed;

struct planck_ec_ucsi_out_data {
	u8 null1[9];
	u8 control[8];
	u8 null2[16];
	u8 msg[16];
} __packed;

static int planck_ec_ucsi_update(struct ucsi *ucsi)
{
	struct planck_ec *ec = ucsi_get_drvdata(ucsi);
	struct planck_ec_ucsi_in_data data;

	mutex_lock(&ec->lock);
	planck_ec_ram_usci_read(ec->client, PLANCK_EC_UCSI_READ, (u8*)&data, sizeof(data));
	mutex_unlock(&ec->lock);

	//dev_warn(&ec->client->dev, "UCSI update\n");
	//dev_warn(&ec->client->dev, "0x%llx %llx | 0x%x\n", ((u64*)data.msg)[0], ((u64*)data.msg)[1], *(u32*)data.cci);
	//dev_info(&ec->client->dev, "UCSI version: %x", *(u16*)data.version);

	memcpy(ec->ucsi_data.in, data.msg, sizeof(data.msg));
	memcpy(ec->ucsi_data.cci, data.cci, sizeof(data.cci));

	return 0;
}

static int planck_ec_ucsi_get_version(struct planck_ec *ec)
{
	u16 version;

	mutex_lock(&ec->lock);
	planck_ec_ram_usci_read(ec->client, PLANCK_EC_UCSI_READ, (u8*)&version, sizeof(version));
	mutex_unlock(&ec->lock);

	return version;
}

static int planck_ec_ucsi_read_version(struct ucsi *ucsi, u16 *version)
{
	struct planck_ec *ec = ucsi_get_drvdata(ucsi);
	int ret;

	//dev_warn(&ec->client->dev, "UCSI read! d[%d] %ld b\n", offset, val_len);

	if (sizeof(*version) > sizeof(ec->ucsi_data) - UCSI_VERSION)
		return -EINVAL;

	/* This is polled in core but everything else fires an event */
	ret = planck_ec_ucsi_update(ucsi);
	if (ret)
		return ret;

	memcpy(version, (u8*)&ec->ucsi_data + UCSI_VERSION, sizeof(*version));

	return 0;
}

static int planck_ec_ucsi_read_cci(struct ucsi *ucsi, u32 *cci)
{
	struct planck_ec *ec = ucsi_get_drvdata(ucsi);
	int ret;

	//dev_warn(&ec->client->dev, "UCSI read! d[%d] %ld b\n", offset, val_len);

	if (sizeof(*cci) > sizeof(ec->ucsi_data) - UCSI_CCI)
		return -EINVAL;

	/* This is polled in core but everything else fires an event */
	if (UCSI_COMMAND(ec->ucsi_cmd) == UCSI_PPM_RESET) {
		ret = planck_ec_ucsi_update(ucsi);
		if (ret)
			return ret;
	}

	memcpy(cci, (u8*)&ec->ucsi_data + UCSI_CCI, sizeof(*cci));

	return 0;
}

static int planck_ec_ucsi_read_message_in(struct ucsi *ucsi, void *val, size_t val_len)
{
	struct planck_ec *ec = ucsi_get_drvdata(ucsi);
	int ret;

	//dev_warn(&ec->client->dev, "UCSI read! d[%d] %ld b\n", offset, val_len);

	if (val_len > sizeof(ec->ucsi_data) - UCSI_VERSION)
		return -EINVAL;

	/* This is polled in core but everything else fires an event */
	if (UCSI_COMMAND(ec->ucsi_cmd) == UCSI_PPM_RESET) {
		ret = planck_ec_ucsi_update(ucsi);
		if (ret)
			return ret;
	}

	memcpy(val, (u8*)&ec->ucsi_data + UCSI_MESSAGE_IN, val_len);

	return 0;
}

static int planck_ec_ucsi_async_control(struct ucsi *ucsi, u64 command)
{
	struct planck_ec *ec = ucsi_get_drvdata(ucsi);
	struct planck_ec_ucsi_out_data data;


	if (sizeof(command) > sizeof(ec->ucsi_data) - UCSI_CONTROL)
		return -EINVAL;

	memcpy((u8*)&ec->ucsi_data + UCSI_CONTROL, &command, sizeof(command));
	ec->ucsi_cmd = command;

	memcpy(data.msg, ec->ucsi_data.out, sizeof(data.msg));
	memcpy(data.control, ec->ucsi_data.control, sizeof(data.control));

	//dev_warn(&ec->client->dev, "UCSI write! d[%d] %ld b\n", offset, val_len);
	//dev_warn(&ec->client->dev, "0x%llx %llx | 0x%llx\n", ((u64*)data.msg)[0], ((u64*)data.msg)[1], *(u64*)data.control);

	mutex_lock(&ec->lock);
	i2c_smbus_write_i2c_block_data(ec->client, PLANCK_EC_UCSI_WRITE, sizeof(data), (u8*)&data);
	mutex_unlock(&ec->lock);

	return 0;
}

static const struct ucsi_operations planck_ec_ucsi_ops = {
	.read_version = planck_ec_ucsi_read_version,
	.read_cci = planck_ec_ucsi_read_cci,
	.read_message_in = planck_ec_ucsi_read_message_in,
	.sync_control = ucsi_sync_control_common,
	.async_control = planck_ec_ucsi_async_control,
};

static int planck_ec_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {0};
	struct device *dev = &client->dev;
	//struct fwnode_handle *fwnode;
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

	/* Backlight */
	ec->backlight_dev = devm_backlight_device_register(&client->dev, "planck-ec-backlight",
							&client->dev, ec, &planck_ec_backlight_ops, &planck_ec_backlight_props);

    if (IS_ERR(ec->backlight_dev))
		return dev_err_probe(dev, PTR_ERR(ec->backlight_dev), 
					 "Failed to register backlight device\n");

	backlight_update_status(ec->backlight_dev);

	/* External Type-C display attach reports */
	/*
	fwnode = device_get_named_child_node(dev, "connector");
	ec->hpd_gpio = devm_gpiod_get(dev, "hpd", GPIOD_IN);
	if (IS_ERR(ec->hpd_gpio))
		dev_err(dev, "Failed to get hpd gpio\n");

	if (fwnode && !(IS_ERR(ec->hpd_gpio))) {
		INIT_WORK(&ec->work, planck_ec_bridge_update_hpd_work);
		ec->bridge.funcs = &planck_ec_bridge_funcs;
		ec->bridge.of_node = to_of_node(fwnode);
		ec->bridge.ops = DRM_BRIDGE_OP_HPD;
		ec->bridge.type = DRM_MODE_CONNECTOR_USB;

		ret = devm_drm_bridge_add(dev, &ec->bridge);
		if (ret) {
			fwnode_handle_put(fwnode);
			return dev_err_probe(dev, ret, "Failed to register drm bridge\n");
		}

		ec->bridge_configured = true;
	}*/

	/* Type-C UCSI interface. */
	ec->ucsi = ucsi_create(dev, &planck_ec_ucsi_ops);
	if (IS_ERR(ec->ucsi))
		return dev_err_probe(dev, PTR_ERR(ec->ucsi), "Failed to create UCSI.\n");

	ucsi_set_drvdata(ec->ucsi, ec);

	ec->ucsi_data.version = planck_ec_ucsi_get_version(ec);

	ret = ucsi_register(ec->ucsi);
	if (ret) {
		ucsi_destroy(ec->ucsi);
		return dev_err_probe(dev, ret, "Failed to register UCSI.\n");
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					planck_ec_irq_handler, IRQF_ONESHOT,
					dev_name(dev), ec);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request irq\n");

	return 0;
}

static void planck_ec_remove(struct i2c_client *client)
{
	struct planck_ec *ec = i2c_get_clientdata(client);

	ucsi_unregister(ec->ucsi);
	ucsi_destroy(ec->ucsi);
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
	},
	.probe = planck_ec_probe,
	.remove = planck_ec_remove,
	.id_table = planck_ec_id,
};
module_i2c_driver(planck_ec_driver);

MODULE_DESCRIPTION("HUAWEI MateBook E (2019) embedded controller");
MODULE_AUTHOR("NewWheat");
MODULE_LICENSE("GPL");
