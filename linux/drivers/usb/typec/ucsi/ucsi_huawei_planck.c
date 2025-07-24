// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2025
 *
 * NewWheat <newwheatzjz@outlook.com>
 */
#include <linux/auxiliary_bus.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_data/huawei-planck-ec.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/usb/typec_dp.h>

#include <drm/bridge/aux-bridge.h>

#include "ucsi.h"

#define PLANCK_EC_UCSI_READ_DATA_CMD		0x0
#define PLANCK_EC_UCSI_READ_HPD_CMD		0x40

#define PLANCK_EC_UCSI_EVENT_UCSI		0x10
#define PLANCK_EC_UCSI_EVENT_HPD		0x40

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

struct planck_ec_ucsi {
	struct planck_ec *ec;
	struct auxiliary_device *bridge;
	struct notifier_block nb;
	struct ucsi *ucsi;
    struct planck_ec_ucsi_in_data ucsi_in;
};

/* UCSI */

static int planck_ec_ucsi_update(struct planck_ec_ucsi *ec_ucsi)
{
	int ret;

	ret = planck_ec_transfer(ec_ucsi->ec, PLANCK_EC_UCSI_READ_DATA_CMD,
							 (u8*)&ec_ucsi->ucsi_in, sizeof(ec_ucsi->ucsi_in), PLANCK_EC_READ_UCSI);

	return ret < 0 ? ret : 0;
}

static int planck_ec_ucsi_read_version(struct ucsi *ucsi, u16 *version)
{
	struct planck_ec_ucsi *ec_ucsi = ucsi_get_drvdata(ucsi);
	int ret;

	ret = planck_ec_ucsi_update(ec_ucsi);
	if (ret < 0)
		return ret;

	memcpy(version, (u8*)&ec_ucsi->ucsi_in.version, sizeof(*version));

	return 0;
}

static int planck_ec_ucsi_poll_cci(struct ucsi *ucsi, u32 *cci)
{
	struct planck_ec_ucsi *ec_ucsi = ucsi_get_drvdata(ucsi);
	int ret;

	ret = planck_ec_ucsi_update(ec_ucsi);
	if (ret < 0)
		return ret;

	memcpy(cci, (u8*)&ec_ucsi->ucsi_in.cci, sizeof(*cci));

	return 0;
}

static int planck_ec_ucsi_read_cci(struct ucsi *ucsi, u32 *cci)
{
	struct planck_ec_ucsi *ec_ucsi = ucsi_get_drvdata(ucsi);

	memcpy(cci, (u8*)&ec_ucsi->ucsi_in.cci, sizeof(*cci));

	return 0;
}

static int planck_ec_ucsi_read_message_in(struct ucsi *ucsi, void *val, size_t val_len)
{
	struct planck_ec_ucsi *ec_ucsi = ucsi_get_drvdata(ucsi);

	memcpy(val, (u8*)&ec_ucsi->ucsi_in.msg, val_len);

	return 0;
}

static int planck_ec_ucsi_async_control(struct ucsi *ucsi, u64 command)
{
	struct planck_ec_ucsi *ec_ucsi = ucsi_get_drvdata(ucsi);
	struct planck_ec_ucsi_out_data data = {0};
    int ret;

	memcpy(&data.control, &command, sizeof(command));

	ret = planck_ec_transfer(ec_ucsi->ec, 0, (u8*)&data, sizeof(data), PLANCK_EC_WRITE_UCSI);

    if (ret < 0) {
		return ret;
	}

	return 0;
}

static int planck_ec_ucsi_sync_control(struct ucsi *ucsi, u64 command, u32 *cci,
                                       void *data, size_t data_len)
{
	int ret;

	if (UCSI_COMMAND(command) == UCSI_GET_ALTERNATE_MODES &&
		UCSI_ALTMODE_RECIPIENT(command) == UCSI_RECIPIENT_CON &&
		UCSI_ALTMODE_OFFSET(command) == 0) {
		static const struct ucsi_altmode alt = {
			.svid = USB_TYPEC_DP_SID,
			.mid = USB_TYPEC_DP_MODE,
		};

		memset(data, 0, data_len);
		memcpy(data, &alt, min(sizeof(alt), data_len));
		*cci = UCSI_CCI_COMMAND_COMPLETE | UCSI_SET_CCI_LENGTH(sizeof(alt));
		return 0;
	}

	ret = ucsi_sync_control_common(ucsi, command, cci, data, data_len);
	if (ret < 0)
		return ret;

	if (UCSI_COMMAND(command) == UCSI_GET_CURRENT_CAM && data) {
		((u8 *)data)[0]--;
	}

	return 0;
}

static bool planck_ec_ucsi_update_altmodes(struct ucsi *ucsi, u8 recipient,
										   struct ucsi_altmode *orig,
										   struct ucsi_altmode *updated)
{
	int i;

	if (orig[0].svid == 0 || recipient != UCSI_RECIPIENT_SOP)
		return false;

	/* EC is nice and repeats altmodes again and again. Ignore copies. */
	for (i = 1; i < UCSI_MAX_ALTMODES; i++) {
		if (orig[i].svid == orig[0].svid) {
			dev_dbg(ucsi->dev, "Found duplicate altmodes, starting from %d\n", i);
			memset(&orig[i], 0, (UCSI_MAX_ALTMODES - i) * sizeof(*orig));
			break;
		}
	}

	return false;
}

static void planck_ec_ucsi_update_connector(struct ucsi_connector *con)
{
	con->typec_cap.orientation_aware = true;
}

static const struct ucsi_operations planck_ec_ucsi_ops = {
	.read_version = planck_ec_ucsi_read_version,
	.poll_cci = planck_ec_ucsi_poll_cci,
	.read_cci = planck_ec_ucsi_read_cci,
	.read_message_in = planck_ec_ucsi_read_message_in,
	.async_control = planck_ec_ucsi_async_control,
	.sync_control = planck_ec_ucsi_sync_control,
	.update_altmodes = planck_ec_ucsi_update_altmodes,
	.update_connector = planck_ec_ucsi_update_connector,
};

/* DP Altmode */

struct planck_ec_ucsi_port_data {
	u8 ccst;
	u8 muxc;
	u8 dppn;
	u8 d7t3;
	u8 d7t4;
	u8 d7t5;
	u8 d7t6;
	u8 hsfl;
} __packed;

static void planck_ec_ucsi_read_port_status(struct planck_ec_ucsi *ec_ucsi)
{
	struct planck_ec_ucsi_port_data data;
	int ret;

	ret = planck_ec_transfer(ec_ucsi->ec, PLANCK_EC_UCSI_READ_HPD_CMD,
							 (u8 *)&data, sizeof(data), PLANCK_EC_READ_COMMON);

	if (ret < 0) {
		return;
	}

	dev_dbg(ec_ucsi->ucsi->dev, "muxc: %d, ccst: %d, dppn: %d, hsfl: %d\n",
			 data.muxc, data.ccst, data.dppn, data.hsfl);

	if (ec_ucsi->ucsi->connector && ec_ucsi->ucsi->connector[0].port)
		typec_set_orientation(ec_ucsi->ucsi->connector[0].port,
							  data.ccst == 1 ?
							  TYPEC_ORIENTATION_REVERSE :
							  TYPEC_ORIENTATION_NORMAL);

	if (ec_ucsi->bridge)
		drm_aux_hpd_bridge_notify(&ec_ucsi->bridge->dev,
								  data.d7t3 != 0 ?
								  connector_status_connected :
								  connector_status_disconnected);
}

static int planck_ec_ucsi_notify(struct notifier_block *nb,
								 unsigned long action, void *data)
{
	struct planck_ec_ucsi *ec_ucsi = container_of(nb, struct planck_ec_ucsi, nb);
	u32 cci;
	int ret;

	switch (action) {
	case PLANCK_EC_UCSI_EVENT_UCSI:
		ret = planck_ec_ucsi_poll_cci(ec_ucsi->ucsi, &cci);
		if (ret < 0)
			return NOTIFY_DONE;

		ucsi_notify_common(ec_ucsi->ucsi, cci);
		break;

	case PLANCK_EC_UCSI_EVENT_HPD:
		planck_ec_ucsi_read_port_status(ec_ucsi);
		ucsi_connector_change(ec_ucsi->ucsi, 1);
		break;

	default:
		return NOTIFY_OK;
	}

	return NOTIFY_OK;
}

static int planck_ec_ucsi_probe(struct auxiliary_device *adev,
								const struct auxiliary_device_id *id)
{
	struct planck_ec *ec = adev->dev.platform_data;
	struct planck_ec_ucsi *ec_ucsi;
	struct fwnode_handle *fwnode;
	int ret;

	ec_ucsi = devm_kzalloc(&adev->dev, sizeof(*ec_ucsi), GFP_KERNEL);
	if (!ec_ucsi)
		return -ENOMEM;

	ec_ucsi->ec = ec;
	ec_ucsi->nb.notifier_call = planck_ec_ucsi_notify;

	fwnode = device_get_named_child_node(&adev->dev, "connector");
	if (fwnode) {
		ec_ucsi->bridge = devm_drm_dp_hpd_bridge_alloc(&adev->dev, to_of_node(fwnode));
		if (IS_ERR(ec_ucsi->bridge))
			return PTR_ERR(ec_ucsi->bridge);

		ret = devm_drm_dp_hpd_bridge_add(&adev->dev, ec_ucsi->bridge);
		if (ret < 0)
			return dev_err_probe(&adev->dev, ret, "Failed to add drm aux-bridge\n");
	}
	else {
		dev_err(&adev->dev, "Child node \"connector\" not found\n");
		return -ENODEV;
	}

	ec_ucsi->ucsi = ucsi_create(&adev->dev, &planck_ec_ucsi_ops);
	if (IS_ERR(ec_ucsi->ucsi))
		return PTR_ERR(ec_ucsi->ucsi);

	ucsi_set_drvdata(ec_ucsi->ucsi, ec_ucsi);

	auxiliary_set_drvdata(adev, ec_ucsi);

	ret = planck_ec_register_notify(ec, &ec_ucsi->nb);
	if (ret < 0) {
		dev_err_probe(&adev->dev, ret, "Failed to register notify block\n");
		goto err_destroy;
	}

	ret = ucsi_register(ec_ucsi->ucsi);
	if (ret < 0) {
		dev_err_probe(&adev->dev, ret, "Failed to register UCSI\n");
		goto err_unregister;
	}

	return 0;

err_unregister:
	planck_ec_unregister_notify(ec_ucsi->ec, &ec_ucsi->nb);

err_destroy:
	ucsi_destroy(ec_ucsi->ucsi);

	return ret;
}

static void planck_ec_ucsi_remove(struct auxiliary_device *adev)
{
	struct planck_ec_ucsi *ec_ucsi = auxiliary_get_drvdata(adev);

	ucsi_unregister(ec_ucsi->ucsi);
	planck_ec_unregister_notify(ec_ucsi->ec, &ec_ucsi->nb);
	ucsi_destroy(ec_ucsi->ucsi);
}

static const struct auxiliary_device_id planck_ec_ucsi_id[] = {
	{ .name = PLANCK_MOD_NAME "." PLANCK_DEV_UCSI, },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, planck_ec_ucsi_id);

static struct auxiliary_driver planck_ec_ucsi_driver = {
	.name = PLANCK_DEV_UCSI,
	.id_table = planck_ec_ucsi_id,
	.probe = planck_ec_ucsi_probe,
	.remove = planck_ec_ucsi_remove,
};

module_auxiliary_driver(planck_ec_ucsi_driver);

MODULE_DESCRIPTION("HUAWEI MateBook E (2019) UCSI");
MODULE_LICENSE("GPL");
