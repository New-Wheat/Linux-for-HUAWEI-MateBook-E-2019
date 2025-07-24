// SPDX-License-Identifier: GPL-2.0-only
/* 
 * HUAWEI MateBook E (2019) embedded controller
 * Copyright (c) 2024, NewWheat <newwheatzjz@outlook.com>
 *
 * Based on acer-aspire1-ec.c
 */
#include <drm/drm_bridge.h>
#include <linux/backlight.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/string.h>
#include <linux/unaligned.h>
// #include <linux/usb/typec_altmode.h>
// #include <linux/usb/typec_dp.h>
// #include <linux/usb/typec_mux.h>
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

#define PLANCK_EC_CACHE_TIME	5000
#define PLANCK_EC_BAT_STATUS	BIT(1)
#define PLANCK_EC_FG_OEM		0x8c
#define PLANCK_EC_FG_DATA		0x80
#define PLANCK_EC_FG_STATE		0xdd

#define PLANCK_EC_ADP_STATE		0xdb
#define PLANCK_EC_ADP_STATUS	BIT(0)

#define PLANCK_EC_FG_FLAG_DISCHARGING	BIT(0)
#define PLANCK_EC_FG_FLAG_CHARGING	BIT(1)

#define PLANCK_EC_BACKLIGHT		0xff
#define PLANCK_EC_BACKLIGHT_MAX	255
#define PLANCK_EC_BACKLIGHT_DEFAULT	127
#define PLANCK_EC_HPD_STATUS	0x40

#define PLANCK_EC_UCSI_RAM_READ_REQUEST	0xa2
#define PLANCK_EC_UCSI_RAM_READ_RESPOND	0xa0
#define PLANCK_EC_UCSI_READ		0x0
#define PLANCK_EC_UCSI_WRITE	0xa1

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

struct planck_ec_ucsi_in_data {
	u16 version;
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

static int planck_ec_ucsi_read(struct ucsi *ucsi, struct planck_ec_ucsi_in_data *data, bool force);
static int planck_ec_ucsi_read_cci(struct ucsi *ucsi, u32 *cci);
static void planck_ec_bridge_update_hpd_work(struct work_struct *work);

struct planck_ec {
	struct i2c_client *client;
	struct mutex lock;
	struct power_supply *bat_psy;
	struct power_supply *adp_psy;
	unsigned long last_status_update;
	struct planck_ec_psy_data data;
	u8 present;
	u8 oem;
	u8 state;
	struct input_dev *idev;
	struct backlight_device *backlight_dev;
	struct drm_bridge bridge;
	struct typec_switch *typec_switch;
	// struct typec_mux *typec_mux;
	// struct typec_mux_state state;
	// struct typec_altmode dp_alt;
	struct work_struct work;
	bool bridge_configured;
	struct ucsi *ucsi;
	struct planck_ec_ucsi_in_data ucsi_in;
	u64 ucsi_command;
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
		planck_ec_ucsi_read(ec->ucsi, NULL, true);
		planck_ec_ucsi_read_cci(ec->ucsi, &cci);
		if (UCSI_CCI_CONNECTOR(cci))
			ucsi_connector_change(ec->ucsi, UCSI_CCI_CONNECTOR(cci));
		ucsi_notify_common(ec->ucsi, cci);
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
		//ucsi_connector_change(ec->ucsi, 1);
		dev_info(&ec->client->dev, "HPD_DETECT event triggered\n");
		if (ec->bridge_configured)
			planck_ec_bridge_update_hpd_work(&ec->work);
		break;

	default:
		dev_warn(&ec->client->dev, "Unknown event ID: 0x%x\n", id);
	}

	return IRQ_HANDLED;
}

/* Power Supply */

static int planck_ec_get_bat_property(struct planck_ec *ec)
{
	if (time_before(jiffies, ec->last_status_update + msecs_to_jiffies(PLANCK_EC_CACHE_TIME)))
		return 0;
	else {
		mutex_lock(&ec->lock);
		planck_ec_ram_read(ec->client, PLANCK_EC_FG_DATA, (u8*)&ec->data, sizeof(ec->data));
		planck_ec_ram_read(ec->client, PLANCK_EC_ADP_STATE, &ec->present, sizeof(ec->present));
		planck_ec_ram_read(ec->client, PLANCK_EC_FG_OEM, &ec->oem, sizeof(ec->oem));
		planck_ec_ram_read(ec->client, PLANCK_EC_FG_STATE, &ec->state, sizeof(ec->state));
		mutex_unlock(&ec->lock);

		ec->last_status_update = jiffies;
	}

	return 1;
}

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
	int str_index = 0;
	char serial_number[5];
	int ret;

	ret = planck_ec_get_bat_property(ec);
	if(ret)
		dev_info(&ec->client->dev, "Updating battery properties");

	switch (psp) {
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = le16_to_cpu(ec->data.voltage_now) * 1000;
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = le16_to_cpu(ec->data.design_voltage) * 1000;
			break;

		case POWER_SUPPLY_PROP_CHARGE_NOW:
			val->intval = le16_to_cpu(ec->data.capacity_now) * 1000;
			break;

		case POWER_SUPPLY_PROP_CHARGE_FULL:
			val->intval = le16_to_cpu(ec->data.last_full_capacity) * 1000;
			break;

		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = le16_to_cpu(ec->data.design_capacity) * 1000;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = le16_to_cpu(ec->data.capacity_now) * 100
					/ le16_to_cpu(ec->data.last_full_capacity);
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = (s16)le16_to_cpu(ec->data.current_now) * 1000;
			break;

		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = !!(ec->present & PLANCK_EC_BAT_STATUS);
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
			str_index = ec->oem - 1;

			if (str_index >= 0 && str_index < ARRAY_SIZE(planck_ec_bat_psy_battery_oem))
				val->strval = planck_ec_bat_psy_battery_oem[str_index];
			else {
				dev_err(&ec->client->dev, "Battery OEM unknown: %d\n", str_index);
				val->strval = "Unknown";
			}
			break;

		case POWER_SUPPLY_PROP_SERIAL_NUMBER:
			snprintf(serial_number, sizeof(serial_number), "%d", ec->data.serial_number);
			val->strval = kasprintf(GFP_KERNEL, "%s", serial_number);
			if (!val->strval)
				dev_err(&ec->client->dev, "Failed to allocate memory for serial number\n");
			break;

		case POWER_SUPPLY_PROP_STATUS:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			if ((ec->state & 0x03) & PLANCK_EC_FG_FLAG_CHARGING)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else if ((ec->state & 0x03) & PLANCK_EC_FG_FLAG_DISCHARGING)
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else if (!(ec->state & 0x03))
				val->intval = POWER_SUPPLY_STATUS_FULL;
			break;

		case POWER_SUPPLY_PROP_CYCLE_COUNT:
			val->intval = le16_to_cpu(ec->data.cycle_count);
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
		val->intval = !!(tmp & PLANCK_EC_ADP_STATUS);
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
	.max_brightness = PLANCK_EC_BACKLIGHT_MAX,
	.brightness = PLANCK_EC_BACKLIGHT_DEFAULT,
};

/* USB-C DP Altmode */

static int planck_ec_bridge_attach(struct drm_bridge *bridge, enum drm_bridge_attach_flags flags)
{
	return flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR ? 0 : -EINVAL;
}

struct planck_ec_hpd_data {
	u8 ccst;
	u8 muxc;
	u8 dppn;
	u8 d7t3;
	u8 d7t4;
	u8 d7t5;
	u8 d7t6;
	u8 hsfl;
} __packed;

// static enum typec_orientation planck_ec_typec_orientation(u8 orientation)
// {
// 	if (orientation == 0)
// 		return TYPEC_ORIENTATION_NORMAL;
// 	else if (orientation == 1)
// 		return TYPEC_ORIENTATION_REVERSE;
// 	else
// 		return TYPEC_ORIENTATION_NONE;
// }

static void planck_ec_bridge_update_hpd_work(struct work_struct *work)
{
	struct planck_ec *ec = container_of(work, struct planck_ec, work);
	// struct typec_displayport_data dp_data = {};
	struct planck_ec_hpd_data data;
	// int ret;
	//int hpd_status = gpiod_get_value_cansleep(ec->hpd_gpio);

	mutex_lock(&ec->lock);
	planck_ec_ram_read(ec->client, PLANCK_EC_HPD_STATUS, (u8 *)&data, sizeof(data));
	mutex_unlock(&ec->lock);

	//dev_info(&ec->client->dev, "HPD status: %d\n", hpd_status);
	dev_info(&ec->client->dev, "ccst: %d\n", data.ccst);
	dev_info(&ec->client->dev, "muxc: %d\n", data.muxc);
	dev_info(&ec->client->dev, "dppn: %d\n", data.dppn);
	dev_info(&ec->client->dev, "d7t3: %d\n", data.d7t3);
	dev_info(&ec->client->dev, "d7t4: %d\n", data.d7t4);
	dev_info(&ec->client->dev, "d7t5: %d\n", data.d7t5);
	dev_info(&ec->client->dev, "d7t6: %d\n", data.d7t6);
	dev_info(&ec->client->dev, "hsfl: %d\n", data.hsfl);

	// typec_switch_set(ec->typec_switch, planck_ec_typec_orientation(data.ccst));
	drm_bridge_hpd_notify(&ec->bridge,
				  data.d7t3 ?
				  connector_status_connected :
				  connector_status_disconnected);
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

// static void planck_ec_put_mux(void *data)
// {
// 	typec_mux_put(data);
// }

// static void planck_ec_put_switch(void *data)
// {
// 	typec_switch_put(data);
// }

/* UCSI */

static int planck_ec_ram_usci_read(struct i2c_client *client, u8 off, u8*data, u8 data_len)
{
	i2c_smbus_write_byte_data(client, PLANCK_EC_UCSI_RAM_READ_REQUEST, off);
	i2c_smbus_read_i2c_block_data(client, PLANCK_EC_UCSI_RAM_READ_RESPOND, data_len, data);
	return 0;
}

static int planck_ec_ucsi_read(struct ucsi *ucsi, struct planck_ec_ucsi_in_data *data, bool force)
{
	struct planck_ec *ec = ucsi_get_drvdata(ucsi);

	if (force || ec->ucsi_command == UCSI_PPM_RESET) {
		mutex_lock(&ec->lock);
		planck_ec_ram_usci_read(ec->client, PLANCK_EC_UCSI_READ, (u8*)&ec->ucsi_in, sizeof(ec->ucsi_in));
		mutex_unlock(&ec->lock);
	}

	if (!data)
		return 0;

	memcpy(data, &ec->ucsi_in, sizeof(*data));

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
	struct planck_ec_ucsi_in_data data;

	planck_ec_ucsi_read(ucsi, &data, false);
	memcpy(version, (u8*)&data.version, sizeof(*version));

	return 0;
}

static int planck_ec_ucsi_read_cci(struct ucsi *ucsi, u32 *cci)
{
	struct planck_ec_ucsi_in_data data;

	planck_ec_ucsi_read(ucsi, &data, false);
	memcpy(cci, (u8*)&data.cci, sizeof(*cci));

	return 0;
}

static int planck_ec_ucsi_read_message_in(struct ucsi *ucsi, void *val, size_t val_len)
{
	struct planck_ec_ucsi_in_data data;

	planck_ec_ucsi_read(ucsi, &data, false);
	memcpy(val, (u8*)&data.msg, val_len);

	return 0;
}

static int planck_ec_ucsi_async_control(struct ucsi *ucsi, u64 command)
{
	struct planck_ec *ec = ucsi_get_drvdata(ucsi);
	struct planck_ec_ucsi_out_data data = {0};

	memcpy(&data.control, &command, sizeof(command));
	ec->ucsi_command = command;

	mutex_lock(&ec->lock);
	i2c_smbus_write_i2c_block_data(ec->client, PLANCK_EC_UCSI_WRITE, sizeof(data), (u8*)&data);
	mutex_unlock(&ec->lock);

	return 0;
}

// static void planck_ec_ucsi_update_connector(struct ucsi_connector *con)
// {
// 	con->typec_cap.orientation_aware = true;
// }

// static void planck_ec_ucsi_connector_status(struct ucsi_connector *con)
// {
// 	struct planck_ec *ec = ucsi_get_drvdata(con->ucsi);
// 	struct planck_ec_hpd_data data;
// 	int orientation;

// 	mutex_lock(&ec->lock);
// 	planck_ec_ram_read(ec->client, PLANCK_EC_HPD_STATUS, (u8 *)&data, sizeof(data));
// 	mutex_unlock(&ec->lock);

// 	orientation = (data.ccst + 1) % 3;

// 	if (orientation >= 0) {
// 		typec_set_orientation(con->port, orientation);
// 	}
// }

static const struct ucsi_operations planck_ec_ucsi_ops = {
	.read_version = planck_ec_ucsi_read_version,
	.read_cci = planck_ec_ucsi_read_cci,
	.read_message_in = planck_ec_ucsi_read_message_in,
	.sync_control = ucsi_sync_control_common,
	.async_control = planck_ec_ucsi_async_control,
	// .update_connector = planck_ec_ucsi_update_connector,
	// .connector_status = planck_ec_ucsi_connector_status,
};

static int planck_ec_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {0};
	struct device *dev = &client->dev;
	struct fwnode_handle *fwnode;
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
	fwnode = device_get_named_child_node(dev, "connector");

	// ec->hpd_gpio = devm_gpiod_get(dev, "hpd", GPIOD_IN);
	// if (IS_ERR(ec->hpd_gpio))
	// 	return dev_err_probe(dev, PTR_ERR(ec->hpd_gpio), "Failed to get hpd gpio\n");

	if (fwnode) {
		INIT_WORK(&ec->work, planck_ec_bridge_update_hpd_work);
		ec->bridge.funcs = &planck_ec_bridge_funcs;
		ec->bridge.of_node = to_of_node(fwnode);
		ec->bridge.ops = DRM_BRIDGE_OP_HPD;
		ec->bridge.type = DRM_MODE_CONNECTOR_USB;

		// ec->bridge = devm_drm_dp_hpd_bridge_alloc(dev, to_of_node(fwnode));
		// if (IS_ERR(ec->bridge)) {
		// 	fwnode_handle_put(fwnode);
		// 	return PTR_ERR(ec->bridge);
		// }

		ret = devm_drm_bridge_add(dev, &ec->bridge);
		if (ret) {
			fwnode_handle_put(fwnode);
			return dev_err_probe(dev, ret, "failed to register drm bridge\n");
		}

		ec->bridge_configured = true;

		// ec->dp_alt.svid = USB_TYPEC_DP_SID;
		// ec->dp_alt.mode = USB_TYPEC_DP_MODE;
		// ec->dp_alt.active = 1;

		// ec->typec_mux = fwnode_typec_mux_get(fwnode);
		// if (IS_ERR(ec->typec_mux)) {
		// 	fwnode_handle_put(fwnode);
		// 	return dev_err_probe(dev, PTR_ERR(ec->typec_mux),
		// 			     "failed to acquire mode-switch\n");
		// }

		// ret = devm_add_action_or_reset(dev, planck_ec_put_mux,
		// 			       ec->typec_mux);
		// if (ret) {
		// 	fwnode_handle_put(fwnode);
		// 	return ret;
		// }

		// ec->typec_switch = fwnode_typec_switch_get(fwnode);
		// if (IS_ERR(ec->typec_switch)) {
		// 	fwnode_handle_put(fwnode);
		// 	return dev_err_probe(dev, PTR_ERR(ec->typec_switch),
		// 			     "failed to acquire orientation-switch\n");
		// }

		// ret = devm_add_action_or_reset(dev, planck_ec_put_switch,
		// 			       ec->typec_switch);
		// if (ret) {
		// 	fwnode_handle_put(fwnode);
		// 	return ret;
		// }
	}

	/* Type-C UCSI interface */
	ec->ucsi = ucsi_create(dev, &planck_ec_ucsi_ops);
	if (IS_ERR(ec->ucsi))
		return dev_err_probe(dev, PTR_ERR(ec->ucsi), "Failed to create UCSI.\n");

	ucsi_set_drvdata(ec->ucsi, ec);

	ec->ucsi_in.version = planck_ec_ucsi_get_version(ec);

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
