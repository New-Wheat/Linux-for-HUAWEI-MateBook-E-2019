// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2025
 *
 * NewWheat <newwheatzjz@outlook.com>
 */
#include <linux/auxiliary_bus.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/platform_data/huawei-planck-ec.h>

#define PLANCK_EC_PSY_CACHE_TIME		5000

#define PLANCK_EC_PSY_READ_DATA_CMD		0x80
#define PLANCK_EC_PSY_READ_OEM_CMD		0x8c
#define PLANCK_EC_PSY_READ_STATE_CMD		0xdd
#define PLANCK_EC_PSY_READ_ONLINE_CMD		0xdb

#define PLANCK_EC_PSY_BAT_PRESENT		BIT(1)
#define PLANCK_EC_PSY_BAT_DISCHARGING		BIT(0)
#define PLANCK_EC_PSY_BAT_CHARGING		BIT(1)

#define PLANCK_EC_PSY_ADP_ONLINE		BIT(0)

#define PLANCK_EC_PSY_EVENT_22		0x22
#define PLANCK_EC_PSY_EVENT_23		0x23
#define PLANCK_EC_PSY_EVENT_30		0x30
#define PLANCK_EC_PSY_EVENT_31		0x31
#define PLANCK_EC_PSY_EVENT_32		0x32

struct planck_ec_psy_data {
	__le16 null1;
	__le16 serial_number;		/* _BIX */
	__le16 design_capacity;		/* _BIX */
	__le16 design_voltage;		/* _BIX */
	__le16 null2;
	__le16 null3;
	__le16 null4;
	__le16 null5;
	__le16 voltage_now;		/* _BST */
	__le16 current_now;		/* _BST */
	__le16 last_full_capacity;		/* _BIX */
	__le16 capacity_now;		/* _BST */
	__le16 cycle_count;		/* _BIX */
} __packed;

struct planck_ec_psy {
	struct planck_ec *ec;
	struct device *dev;
    struct fwnode_handle *fwnode;
	struct notifier_block nb;
	struct power_supply *bat_psy;
	struct power_supply *adp_psy;
	unsigned long last_status_update;
	struct planck_ec_psy_data data;
	u8 oem;		/* _BIX */
	u8 state;		/* _BST */
	u8 online;		/* _STA & _PSR */
};

static int planck_ec_psy_update_data(struct planck_ec_psy *ec_psy)
{
	int ret = 0;

	ret = planck_ec_transfer(ec_psy->ec, PLANCK_EC_PSY_READ_DATA_CMD,
							 (u8*)&ec_psy->data, sizeof(ec_psy->data), PLANCK_EC_READ_COMMON);

	return ret;
}

static int planck_ec_psy_info_update(struct planck_ec_psy *ec_psy)
{
	int ret = 0;

	ret |= planck_ec_psy_update_data(ec_psy);
	ret |= planck_ec_transfer(ec_psy->ec, PLANCK_EC_PSY_READ_OEM_CMD,
							  &ec_psy->oem, sizeof(ec_psy->oem), PLANCK_EC_READ_COMMON);
	ret |= planck_ec_transfer(ec_psy->ec, PLANCK_EC_PSY_READ_ONLINE_CMD,
							  &ec_psy->online, sizeof(ec_psy->online), PLANCK_EC_READ_COMMON);

	return ret;
}

static int planck_ec_psy_status_update(struct planck_ec_psy *ec_psy)
{
	int ret = 0;

	if (time_after(jiffies, ec_psy->last_status_update
							 + msecs_to_jiffies(PLANCK_EC_PSY_CACHE_TIME))) {
		ret |= planck_ec_psy_update_data(ec_psy);
		ret |= planck_ec_transfer(ec_psy->ec, PLANCK_EC_PSY_READ_STATE_CMD,
								  &ec_psy->state, sizeof(ec_psy->state), PLANCK_EC_READ_COMMON);

		ec_psy->last_status_update = jiffies;
	}

    return ret;
}

/* Battery */

static const char * const planck_ec_psy_bat_oem[] = {
	"DYNAPACK",
	"Sunwoda-S",
};

static bool planck_ec_psy_bat_is_charged(struct planck_ec_psy *ec_psy)
{
	if (ec_psy->data.capacity_now >= ec_psy->data.design_capacity ||
		ec_psy->data.capacity_now >= ec_psy->data.last_full_capacity)
		return true;

	return false;
}

static int planck_ec_psy_bat_get_property(struct power_supply *psy,
										  enum power_supply_property psp,
										  union power_supply_propval *val)
{
	struct planck_ec_psy *ec_psy = power_supply_get_drvdata(psy);
	int str_index = ec_psy->oem - 1;
	char serial_number[6];
	int ret;

	ret = planck_ec_psy_status_update(ec_psy);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		if(planck_ec_psy_bat_is_charged(ec_psy))
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (ec_psy->state & PLANCK_EC_PSY_BAT_DISCHARGING)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (ec_psy->state & PLANCK_EC_PSY_BAT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !!(ec_psy->online & PLANCK_EC_PSY_BAT_PRESENT);
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = le16_to_cpu(ec_psy->data.cycle_count);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = le16_to_cpu(ec_psy->data.design_voltage) * 1000;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = le16_to_cpu(ec_psy->data.voltage_now) * 1000;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = (s16)le16_to_cpu(ec_psy->data.current_now) * 1000;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = le16_to_cpu(ec_psy->data.design_capacity) * 1000;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = le16_to_cpu(ec_psy->data.last_full_capacity) * 1000;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = le16_to_cpu(ec_psy->data.capacity_now) * 1000;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = le16_to_cpu(ec_psy->data.capacity_now) * 100
				      / le16_to_cpu(ec_psy->data.last_full_capacity);
		break;

	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "HB30C4J7ECW-21";
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		if (str_index >= 0 && str_index < ARRAY_SIZE(planck_ec_psy_bat_oem))
			val->strval = planck_ec_psy_bat_oem[str_index];
		else {
			dev_err(ec_psy->dev, "Battery OEM unknown: %d\n", str_index);
			val->strval = "Unknown";
		}
		break;

	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		snprintf(serial_number, sizeof(serial_number), "%d", ec_psy->data.serial_number);
		val->strval = kasprintf(GFP_KERNEL, "%s", serial_number);
		if (!val->strval)
			dev_err(ec_psy->dev, "Failed to allocate memory for serial number\n");
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property planck_ec_psy_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc planck_ec_psy_bat_desc = {
	.name = "planck-ec-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = planck_ec_psy_bat_get_property,
	.properties	= planck_ec_psy_bat_props,
	.num_properties	= ARRAY_SIZE(planck_ec_psy_bat_props),
};

/* Adapter */

static int planck_ec_psy_adp_get_property(struct power_supply *psy,
										  enum power_supply_property psp,
										  union power_supply_propval *val)
{
	struct planck_ec_psy *ec_psy = power_supply_get_drvdata(psy);
	int ret;

	ret = planck_ec_psy_info_update(ec_psy);
	if (ret)
		return 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(ec_psy->online & PLANCK_EC_PSY_ADP_ONLINE);
		break;

	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = POWER_SUPPLY_USB_TYPE_C;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property planck_ec_psy_adp_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc planck_ec_psy_adp_desc = {
	.name = "planck-ec-adapter",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_C),
	.get_property = planck_ec_psy_adp_get_property,
	.properties	= planck_ec_psy_adp_props,
	.num_properties	= ARRAY_SIZE(planck_ec_psy_adp_props),
};

static int planck_ec_psy_notify(struct notifier_block *nb,
								unsigned long action, void *data)
{
	struct planck_ec_psy *ec_psy = container_of(nb, struct planck_ec_psy, nb);

	switch (action) {
	case PLANCK_EC_PSY_EVENT_22:
	case PLANCK_EC_PSY_EVENT_23:
		power_supply_changed(ec_psy->adp_psy);
		power_supply_changed(ec_psy->bat_psy);
		break;

	case PLANCK_EC_PSY_EVENT_31:
		planck_ec_psy_info_update(ec_psy);
		fallthrough;
	case PLANCK_EC_PSY_EVENT_30:
	case PLANCK_EC_PSY_EVENT_32:
		power_supply_changed(ec_psy->bat_psy);
		break;

	default:
		return NOTIFY_OK;
	}

	return NOTIFY_OK;
}

static int planck_ec_psy_probe(struct auxiliary_device *adev,
							   const struct auxiliary_device_id *id)
{
	struct planck_ec *ec = adev->dev.platform_data;
	struct power_supply_config adp_cfg = {};
	struct power_supply_config bat_cfg = {};
	struct device *dev = &adev->dev;
	struct planck_ec_psy *ec_psy;
	int ret;

	ec_psy = devm_kzalloc(&adev->dev, sizeof(*ec_psy), GFP_KERNEL);
	if (!ec_psy)
		return -ENOMEM;

	ec_psy->ec = ec;
	ec_psy->dev = dev;
	ec_psy->fwnode = adev->dev.parent->fwnode;
	ec_psy->nb.notifier_call = planck_ec_psy_notify;

	auxiliary_set_drvdata(adev, ec_psy);

    bat_cfg.drv_data = ec_psy;
	bat_cfg.fwnode = ec_psy->fwnode;
	bat_cfg.no_wakeup_source = true;
	ec_psy->bat_psy = power_supply_register(dev, &planck_ec_psy_bat_desc, &bat_cfg);
	if (IS_ERR(ec_psy->bat_psy)) {
		return dev_err_probe(ec_psy->dev, PTR_ERR(ec_psy->bat_psy),
							 "Failed to register battery supply\n");
	}

	adp_cfg.drv_data = ec_psy;
	adp_cfg.fwnode = ec_psy->fwnode;
	adp_cfg.supplied_to = (char **)&planck_ec_psy_bat_desc.name;
	adp_cfg.num_supplicants = 1;
	adp_cfg.no_wakeup_source = true;
	ec_psy->adp_psy = devm_power_supply_register(dev, &planck_ec_psy_adp_desc, &adp_cfg);
	if (IS_ERR(ec_psy->adp_psy)) {
		return dev_err_probe(dev, PTR_ERR(ec_psy->adp_psy),
							 "Failed to register AC adapter supply\n");
	}

    planck_ec_psy_info_update(ec_psy);

	ret = planck_ec_register_notify(ec_psy->ec, &ec_psy->nb);
	if (ret < 0) {
		dev_err(ec_psy->dev, "Failed to register notify block: %pe\n", ERR_PTR(ret));
		return ret;
	}

	return 0;
}

static void planck_ec_psy_remove(struct auxiliary_device *adev)
{
	struct planck_ec_psy *ec_psy = auxiliary_get_drvdata(adev);

	planck_ec_unregister_notify(ec_psy->ec, &ec_psy->nb);
	power_supply_unregister(ec_psy->bat_psy);
}

static const struct auxiliary_device_id planck_ec_psy_id[] = {
	{ .name = PLANCK_MOD_NAME "." PLANCK_DEV_PSY, },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, planck_ec_psy_id);

static struct auxiliary_driver planck_ec_psy_driver = {
	.name = PLANCK_DEV_PSY,
	.id_table = planck_ec_psy_id,
	.probe = planck_ec_psy_probe,
	.remove = planck_ec_psy_remove,
};

module_auxiliary_driver(planck_ec_psy_driver);

MODULE_DESCRIPTION("HUAWEI MateBook E (2019) psy");
MODULE_LICENSE("GPL");
