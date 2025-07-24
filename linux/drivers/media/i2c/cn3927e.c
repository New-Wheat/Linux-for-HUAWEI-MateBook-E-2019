// SPDX-License-Identifier: GPL-2.0
// based on dw9714.c

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>

#define CN3927E_NAME		"cn3927e"
#define CN3927E_MAX_FOCUS_POS	1023
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define CN3927E_FOCUS_STEPS	1
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define CN3927E_CTRL_STEPS	16
#define CN3927E_CTRL_DELAY_US	1000
/*
 * S[3:2] = 0x00, codes per step for "Linear Slope Control"
 * S[1:0] = 0x00, step period
 */
#define CN3927E_DEFAULT_S 0x0
#define CN3927E_VAL(data, s) ((data) << 4 | (s))

/* cn3927e device structure */
struct cn3927e_device {
	struct v4l2_ctrl_handler ctrls_vcm;
	struct v4l2_subdev sd;
	u16 current_val;
	struct regulator *vcc;
};

static inline struct cn3927e_device *to_cn3927e_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct cn3927e_device, ctrls_vcm);
}

static inline struct cn3927e_device *sd_to_cn3927e_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct cn3927e_device, sd);
}

static int cn3927e_i2c_write(struct i2c_client *client, u16 data)
{
	int ret;
	__be16 val = cpu_to_be16(data);

	ret = i2c_master_send(client, (const char *)&val, sizeof(val));
	if (ret != sizeof(val)) {
		dev_err(&client->dev, "I2C write fail\n");
		return -EIO;
	}
	return 0;
}

static int cn3927e_t_focus_vcm(struct cn3927e_device *cn3927e_dev, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&cn3927e_dev->sd);

	cn3927e_dev->current_val = val;

	return cn3927e_i2c_write(client, CN3927E_VAL(val, CN3927E_DEFAULT_S));
}

static int cn3927e_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cn3927e_device *dev_vcm = to_cn3927e_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		return cn3927e_t_focus_vcm(dev_vcm, ctrl->val);

	return -EINVAL;
}

static const struct v4l2_ctrl_ops cn3927e_vcm_ctrl_ops = {
	.s_ctrl = cn3927e_set_ctrl,
};

static int cn3927e_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int cn3927e_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops cn3927e_int_ops = {
	.open = cn3927e_open,
	.close = cn3927e_close,
};

static const struct v4l2_subdev_core_ops cn3927e_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops cn3927e_ops = {
	.core = &cn3927e_core_ops,
};

static void cn3927e_subdev_cleanup(struct cn3927e_device *cn3927e_dev)
{
	v4l2_async_unregister_subdev(&cn3927e_dev->sd);
	v4l2_ctrl_handler_free(&cn3927e_dev->ctrls_vcm);
	media_entity_cleanup(&cn3927e_dev->sd.entity);
}

static int cn3927e_init_controls(struct cn3927e_device *dev_vcm)
{
	struct v4l2_ctrl_handler *hdl = &dev_vcm->ctrls_vcm;
	const struct v4l2_ctrl_ops *ops = &cn3927e_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, CN3927E_MAX_FOCUS_POS, CN3927E_FOCUS_STEPS, 0);

	if (hdl->error)
		dev_err(dev_vcm->sd.dev, "%s fail error: 0x%x\n",
			__func__, hdl->error);
	dev_vcm->sd.ctrl_handler = hdl;
	return hdl->error;
}

static int cn3927e_probe(struct i2c_client *client)
{
	struct cn3927e_device *cn3927e_dev;
	int rval;

	cn3927e_dev = devm_kzalloc(&client->dev, sizeof(*cn3927e_dev),
				  GFP_KERNEL);
	if (cn3927e_dev == NULL)
		return -ENOMEM;

	cn3927e_dev->vcc = devm_regulator_get(&client->dev, "vcc");
	if (IS_ERR(cn3927e_dev->vcc))
		return PTR_ERR(cn3927e_dev->vcc);

	rval = regulator_enable(cn3927e_dev->vcc);
	if (rval < 0) {
		dev_err(&client->dev, "failed to enable vcc: %d\n", rval);
		return rval;
	}

	usleep_range(1000, 2000);

	v4l2_i2c_subdev_init(&cn3927e_dev->sd, client, &cn3927e_ops);
	cn3927e_dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	cn3927e_dev->sd.internal_ops = &cn3927e_int_ops;

	rval = cn3927e_init_controls(cn3927e_dev);
	if (rval)
		goto err_cleanup;

	rval = media_entity_pads_init(&cn3927e_dev->sd.entity, 0, NULL);
	if (rval < 0)
		goto err_cleanup;

	cn3927e_dev->sd.entity.function = MEDIA_ENT_F_LENS;

	rval = v4l2_async_register_subdev(&cn3927e_dev->sd);
	if (rval < 0)
		goto err_cleanup;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

err_cleanup:
	regulator_disable(cn3927e_dev->vcc);
	v4l2_ctrl_handler_free(&cn3927e_dev->ctrls_vcm);
	media_entity_cleanup(&cn3927e_dev->sd.entity);

	return rval;
}

static void cn3927e_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cn3927e_device *cn3927e_dev = sd_to_cn3927e_vcm(sd);
	int ret;

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)) {
		ret = regulator_disable(cn3927e_dev->vcc);
		if (ret) {
			dev_err(&client->dev,
				"Failed to disable vcc: %d\n", ret);
		}
	}
	pm_runtime_set_suspended(&client->dev);
	cn3927e_subdev_cleanup(cn3927e_dev);
}

/*
 * This function sets the vcm position, so it consumes least current
 * The lens position is gradually moved in units of CN3927E_CTRL_STEPS,
 * to make the movements smoothly.
 */
static int __maybe_unused cn3927e_vcm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cn3927e_device *cn3927e_dev = sd_to_cn3927e_vcm(sd);
	int ret, val;

	if (pm_runtime_suspended(&client->dev))
		return 0;

	for (val = cn3927e_dev->current_val & ~(CN3927E_CTRL_STEPS - 1);
	     val >= 0; val -= CN3927E_CTRL_STEPS) {
		ret = cn3927e_i2c_write(client,
				       CN3927E_VAL(val, CN3927E_DEFAULT_S));
		if (ret)
			dev_err_once(dev, "%s I2C failure: %d", __func__, ret);
		usleep_range(CN3927E_CTRL_DELAY_US, CN3927E_CTRL_DELAY_US + 10);
	}

	ret = regulator_disable(cn3927e_dev->vcc);
	if (ret)
		dev_err(dev, "Failed to disable vcc: %d\n", ret);

	return ret;
}

/*
 * This function sets the vcm position to the value set by the user
 * through v4l2_ctrl_ops s_ctrl handler
 * The lens position is gradually moved in units of CN3927E_CTRL_STEPS,
 * to make the movements smoothly.
 */
static int  __maybe_unused cn3927e_vcm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cn3927e_device *cn3927e_dev = sd_to_cn3927e_vcm(sd);
	int ret, val;

	if (pm_runtime_suspended(&client->dev))
		return 0;

	ret = regulator_enable(cn3927e_dev->vcc);
	if (ret) {
		dev_err(dev, "Failed to enable vcc: %d\n", ret);
		return ret;
	}
	usleep_range(1000, 2000);

	for (val = cn3927e_dev->current_val % CN3927E_CTRL_STEPS;
	     val < cn3927e_dev->current_val + CN3927E_CTRL_STEPS - 1;
	     val += CN3927E_CTRL_STEPS) {
		ret = cn3927e_i2c_write(client,
				       CN3927E_VAL(val, CN3927E_DEFAULT_S));
		if (ret)
			dev_err_ratelimited(dev, "%s I2C failure: %d",
						__func__, ret);
		usleep_range(CN3927E_CTRL_DELAY_US, CN3927E_CTRL_DELAY_US + 10);
	}

	return 0;
}

static const struct i2c_device_id cn3927e_id_table[] = {
	{ CN3927E_NAME, 0 },
	{ { 0 } }
};
MODULE_DEVICE_TABLE(i2c, cn3927e_id_table);

static const struct of_device_id cn3927e_of_table[] = {
	{ .compatible = "chipnext,cn3927e" },
	{ { 0 } }
};
MODULE_DEVICE_TABLE(of, cn3927e_of_table);

static const struct dev_pm_ops cn3927e_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cn3927e_vcm_suspend, cn3927e_vcm_resume)
	SET_RUNTIME_PM_OPS(cn3927e_vcm_suspend, cn3927e_vcm_resume, NULL)
};

static struct i2c_driver cn3927e_i2c_driver = {
	.driver = {
		.name = CN3927E_NAME,
		.pm = &cn3927e_pm_ops,
		.of_match_table = cn3927e_of_table,
	},
	.probe = cn3927e_probe,
	.remove = cn3927e_remove,
	.id_table = cn3927e_id_table,
};

module_i2c_driver(cn3927e_i2c_driver);

MODULE_AUTHOR("NewWheat <newwheatzjz@outlook.com>");
MODULE_DESCRIPTION("cn3927e VCM driver");
MODULE_LICENSE("GPL v2");
