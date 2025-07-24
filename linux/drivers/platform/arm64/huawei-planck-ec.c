// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2025
 *
 * NewWheat <newwheatzjz@outlook.com>
 */
#include <linux/auxiliary_bus.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/irqreturn.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/platform_data/huawei-planck-ec.h>

#define PLANCK_EC_READ_EVENT_REG		0x31
#define PLANCK_EC_READ_COMMON_REQ_REG		0x21
#define PLANCK_EC_READ_COMMON_RESP_REG		0x21
#define PLANCK_EC_READ_UCSI_REQ_REG		0xa2
#define PLANCK_EC_READ_UCSI_RESP_REG		0xa0

#define PLANCK_EC_WRITE_BACKLIGHT_REG		0x20
#define PLANCK_EC_WRITE_UCSI_REG		0xa1

#define PLANCK_EC_WRITE_BACKLIGHT_CMD		0xff

#define PLANCK_EC_BACKLIGHT_DEFAULT		127
#define PLANCK_EC_BACKLIGHT_MAX		255

#define PLANCK_EC_EVENT_LID_OPEN		0x20
#define PLANCK_EC_EVENT_LID_CLOSE		0x21

struct planck_ec {
	struct i2c_client *client;
	struct mutex lock;
	struct blocking_notifier_head notifier_list;
	struct backlight_device *bl;
	struct input_dev *idev;
};

static int planck_ec_read_event(struct planck_ec *ec, u8 *data)
{
	struct i2c_client *client = ec->client;

	guard(mutex)(&ec->lock);

	*data = i2c_smbus_read_byte_data(client, PLANCK_EC_READ_EVENT_REG);

	if (*data < 0) {
		dev_err(&client->dev, "EC read_event failed: %pe\n", ERR_PTR(*data));
	}

	return *data < 0 ? *data : 0;
}

static int planck_ec_read_common(struct planck_ec *ec, u8 cmd, u8 *data, u8 data_len)
{
	struct i2c_client *client = ec->client;
	int ret = 0;

	guard(mutex)(&ec->lock);

	ret |= i2c_smbus_write_byte_data(client, PLANCK_EC_READ_COMMON_REQ_REG, cmd);
	ret |= i2c_smbus_read_i2c_block_data(client, PLANCK_EC_READ_COMMON_RESP_REG, data_len, data);

	if (ret < 0) {
		dev_err(&client->dev, "EC read_common failed: %pe, cmd: 0x%x\n", ERR_PTR(ret), cmd);
		return ret;
	}

	return 0;
}

static int planck_ec_read_ucsi(struct planck_ec *ec, u8 cmd, u8 *data, u8 data_len)
{
	struct i2c_client *client = ec->client;
	int ret = 0;

	guard(mutex)(&ec->lock);

	ret |= i2c_smbus_write_byte_data(client, PLANCK_EC_READ_UCSI_REQ_REG, cmd);
	ret |= i2c_smbus_read_i2c_block_data(client, PLANCK_EC_READ_UCSI_RESP_REG, data_len, data);

	if (ret < 0) {
		dev_err(&client->dev, "EC read_ucsi failed: %pe, cmd: 0x%x\n", ERR_PTR(ret), cmd);
		return ret;
	}

	return 0;
}

static int planck_ec_write_backlight(struct planck_ec *ec, u8 *data)
{
	struct i2c_client *client = ec->client;
	u8 tmp[2] = {PLANCK_EC_WRITE_BACKLIGHT_CMD, *data};
	int ret;

	guard(mutex)(&ec->lock);

	ret = i2c_smbus_write_i2c_block_data(client, PLANCK_EC_WRITE_BACKLIGHT_REG, sizeof(tmp), tmp);

	if (ret < 0) {
		dev_err(&client->dev, "EC write_backlight failed: %pe\n", ERR_PTR(ret));
		return ret;
	}

	return 0;
}

static int planck_ec_write_ucsi(struct planck_ec *ec, u8 *data, u8 data_len)
{
	struct i2c_client *client = ec->client;
	int ret;

	guard(mutex)(&ec->lock);

	ret = i2c_smbus_write_i2c_block_data(client, PLANCK_EC_WRITE_UCSI_REG, data_len, data);

	if (ret < 0) {
		dev_err(&client->dev, "EC write_ucsi failed: %pe\n", ERR_PTR(ret));
		return ret;
	}

	return 0;
}

/* Common APIs */

int planck_ec_transfer(struct planck_ec *ec, u8 cmd,
					   u8 *data, u8 data_len, enum planck_ec_transfer_operation op)
{
	int ret = 0;

	/* We sleep here to avoid -ENXIO caused by frequent i2c transfers */
	msleep(13);

	switch (op) {
	case PLANCK_EC_READ_EVENT:
		ret = planck_ec_read_event(ec, data);
		break;

	case PLANCK_EC_READ_COMMON:
		ret = planck_ec_read_common(ec, cmd, data, data_len);
		break;

	case PLANCK_EC_READ_UCSI:
		ret = planck_ec_read_ucsi(ec, cmd, data, data_len);
		break;

	case PLANCK_EC_WRITE_BACKLIGHT:
		ret = planck_ec_write_backlight(ec, data);
		break;

	case PLANCK_EC_WRITE_UCSI:
		ret = planck_ec_write_ucsi(ec, data, data_len);
		break;

	default:
		return 0;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(planck_ec_transfer);

int planck_ec_register_notify(struct planck_ec *ec, struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ec->notifier_list, nb);
}
EXPORT_SYMBOL_GPL(planck_ec_register_notify);

void planck_ec_unregister_notify(struct planck_ec *ec, struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&ec->notifier_list, nb);
}
EXPORT_SYMBOL_GPL(planck_ec_unregister_notify);

/* Backlight */

static int planck_ec_backlight_update_status(struct backlight_device *bl)
{
	struct planck_ec *ec = bl_get_data(bl);
	int brightness = backlight_get_brightness(bl);
	int ret = 0;

	ret = planck_ec_transfer(ec, 0, (u8*)&brightness, sizeof(brightness), PLANCK_EC_WRITE_BACKLIGHT);

	return ret;
}

static const struct backlight_ops planck_ec_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = planck_ec_backlight_update_status,
};

static const struct backlight_properties planck_ec_backlight_props = {
	.type = BACKLIGHT_RAW,
	.scale = BACKLIGHT_SCALE_LINEAR,
	.brightness = PLANCK_EC_BACKLIGHT_DEFAULT,
	.max_brightness = PLANCK_EC_BACKLIGHT_MAX,
};

/* IRQ Handler */

static irqreturn_t planck_ec_irq_handler(int irq, void *data)
{
	struct planck_ec *ec = data;
	int id = 0;

	planck_ec_transfer(ec, 0, (u8*)&id, sizeof(id), PLANCK_EC_READ_EVENT);

	if (id < 0) {
		return IRQ_HANDLED;
	}

	switch (id) {
	case PLANCK_EC_EVENT_LID_OPEN:
		input_report_switch(ec->idev, SW_LID, 0);
		input_sync(ec->idev);
		break;

	case PLANCK_EC_EVENT_LID_CLOSE:
		input_report_switch(ec->idev, SW_LID, 1);
		input_sync(ec->idev);
		break;

	default:
		blocking_notifier_call_chain(&ec->notifier_list, id, ec);
	}

	return IRQ_HANDLED;
}

static void planck_ec_aux_release(struct device *dev)
{
	struct auxiliary_device *adev = to_auxiliary_dev(dev);

	kfree(adev);
}

static void planck_ec_aux_remove(void *data)
{
	struct auxiliary_device *adev = data;

	auxiliary_device_delete(adev);
	auxiliary_device_uninit(adev);
}

static int planck_aux_init(struct device *parent, const char *name,
						   struct planck_ec *ec)
{
	struct auxiliary_device *adev;
	int ret;

	adev = kzalloc(sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->name = name;
	adev->id = 0;
	adev->dev.parent = parent;
	adev->dev.release = planck_ec_aux_release;
	adev->dev.platform_data = ec;

	device_set_of_node_from_dev(&adev->dev, parent);

	ret = auxiliary_device_init(adev);
	if (ret < 0) {
		kfree(adev);
		return ret;
	}

	ret = auxiliary_device_add(adev);
	if (ret < 0) {
		auxiliary_device_uninit(adev);
		return ret;
	}

	return devm_add_action_or_reset(parent, planck_ec_aux_remove, adev);
}

static int planck_ec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct planck_ec *ec;
	int ret;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	mutex_init(&ec->lock);
	ec->client = client;
	BLOCKING_INIT_NOTIFIER_HEAD(&ec->notifier_list);

	ec->bl = devm_backlight_device_register(&client->dev, "planck-ec-backlight",
											&client->dev, ec, &planck_ec_backlight_ops,
											&planck_ec_backlight_props);

    if (IS_ERR(ec->bl))
		return dev_err_probe(dev, PTR_ERR(ec->bl), 
							 "Failed to register backlight device\n");

	backlight_update_status(ec->bl);

	ec->idev = devm_input_allocate_device(dev);
	if (!ec->idev)
		return -ENOMEM;

	ec->idev->name = "planck-ec-lid";
	ec->idev->phys = "planck-ec/input0";
	input_set_capability(ec->idev, EV_SW, SW_LID);

	ret = input_register_device(ec->idev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to register input device\n");

	ret = devm_request_threaded_irq(dev, client->irq,
									NULL, planck_ec_irq_handler,
									IRQF_ONESHOT, dev_name(dev), ec);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to request irq\n");

	ret = planck_aux_init(dev, PLANCK_DEV_PSY, ec);
	if (ret < 0)
		return ret;

	return planck_aux_init(dev, PLANCK_DEV_UCSI, ec);
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
	.id_table = planck_ec_id,
};
module_i2c_driver(planck_ec_driver);

MODULE_DESCRIPTION("HUAWEI MateBook E (2019) embedded controller");
MODULE_AUTHOR("NewWheat <newwheatzjz@outlook.com>");
MODULE_LICENSE("GPL");
