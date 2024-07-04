// SPDX-License-Identifier: GPL-2.0-only
/*
 * Prophesee Driver for AXI4-Stream Event Stream Smart Tracker
 *
 * See dt-binding for the IP purpose
 *
 * Copyright (C) Prophesee S.A.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>

#define PAD_SINK 0
#define PAD_SOURCE 1


#define REG_CONTROL (0x0)
union global_ctrl {
	struct {
		u32 enable:1;
		u32 reset:1;
		u32 clear:1;
		u32:29;
	};
	u32 raw;
};

#define REG_CONFIG (0x4)
union global_cfg {
	struct {
		u32 bypass:1;
		u32:31;
	};
	u32 raw;
};

#define REG_VERSION (0x10)

/**
 * struct psee_esst - Prophesee generic structure of a streaming IP
 * @subdev: V4L2 subdev
 * @pads: media pads
 * @formats: V4L2 media bus formats
 * @dev: (OF) device
 * @iomem: device I/O register space remapped to kernel virtual memory
 * @clk: video core clock
 */
struct psee_esst {
	struct v4l2_subdev subdev;
	struct media_pad pads[2];
	struct v4l2_mbus_framefmt formats[2];
	struct device *dev;
	void __iomem *iomem;
	resource_size_t iosize;
	struct clk *clk;
};

static inline struct psee_esst *to_esst(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct psee_esst, subdev);
}

/*
 * Register related operations
 */
static inline u32 read_reg(struct psee_esst *esst, u32 addr)
{
	return ioread32(esst->iomem + addr);
}

static inline void write_reg(struct psee_esst *esst, u32 addr, u32 value)
{
	iowrite32(value, esst->iomem + addr);
}

/*
 * V4L2 Subdevice Video Operations
 */

static int s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct psee_esst *esst = to_esst(subdev);
	union global_ctrl control = { .raw = 0 };

	if (!enable) {
		control.enable = 0;
		control.clear = 1;
	} else {
		control.clear = 0;
		control.enable = 1;
	}

	write_reg(esst, REG_CONTROL, control.raw);

	return 0;
}

/*
 * V4L2 Subdevice Pad Operations
 */

static struct v4l2_mbus_framefmt *
__get_pad_format(struct psee_esst *esst, struct v4l2_subdev_state *sd_state,
	unsigned int pad, u32 which)
{
	struct v4l2_mbus_framefmt *format;

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		format = v4l2_subdev_get_try_format(&esst->subdev, sd_state, pad);
		break;
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		format = &esst->formats[pad];
		break;
	default:
		format = NULL;
		break;
	}

	return format;
}

static int enum_mbus_code(struct v4l2_subdev *subdev, struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_mbus_code_enum *code)
{
	struct v4l2_mbus_framefmt *format;

	if (code->index)
		return -EINVAL;

	format = v4l2_subdev_get_try_format(subdev, sd_state, code->pad);

	code->code = format->code;

	return 0;
}

static int get_format(struct v4l2_subdev *subdev, struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_format *fmt)
{
	struct psee_esst *esst = to_esst(subdev);
	struct v4l2_mbus_framefmt *format;

	format = __get_pad_format(esst, sd_state, fmt->pad, fmt->which);
	if (!format)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

static int set_format(struct v4l2_subdev *subdev, struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_format *fmt)
{
	struct psee_esst *esst = to_esst(subdev);
	struct v4l2_mbus_framefmt *format;
	union global_cfg config;

	format = __get_pad_format(esst, sd_state, fmt->pad, fmt->which);
	if (!format)
		return -EINVAL;

	config.raw = read_reg(esst, REG_CONFIG);

	if (fmt->pad == PAD_SINK) {
		/* Save the new format */
		*format = fmt->format;
		/* Bypass the IP if not in Evt 2.1*/
		if (format->code == MEDIA_BUS_FMT_PSEE_EVT21)
			config.bypass = 0;
		else
			config.bypass = 1;
		/* Propagate the format to the source pad */
		format = __get_pad_format(esst, sd_state, PAD_SOURCE, fmt->which);
		*format = fmt->format;
		write_reg(esst, REG_CONFIG, config.raw);
	} else {
		/* This IP does no format conversion */
		struct v4l2_mbus_framefmt *input_format;

		input_format = __get_pad_format(esst, sd_state, PAD_SINK, fmt->which);
		/* output is always the same as input */
		*format = *input_format;
		/* Let the userspace know about it */
		fmt->format = *input_format;
	}

	return 0;
}

/*
 * V4L2 Subdevice Operations
 */
static int log_status(struct v4l2_subdev *sd)
{
	struct psee_esst *esst = to_esst(sd);
	struct device *dev = esst->dev;
	union global_ctrl control;
	union global_cfg config;
	u32 version;

	control.raw = read_reg(esst, REG_CONTROL);
	config.raw = read_reg(esst, REG_CONFIG);
	version = read_reg(esst, REG_VERSION);

	dev_info(dev, "***** Event Stream Smart Tracker driver *****\n");
	dev_info(dev, "Version = 0x%x\n", version);
	dev_info(dev, "Control = %s %s(0x%x)\n",
		control.enable ? "ENABLED" : "DISABLED",
		control.clear ? "CLEARING " : "",
		control.raw);
	dev_info(dev, "Config = %s (0x%x)\n",
		config.bypass ? "BYPASSED" : "USED",
		config.raw);

	dev_info(dev, "I/O space = 0x%llx\n", esst->iosize);
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct psee_esst *esst = to_esst(sd);

	/* check if the address is aligned */
	if (reg->reg & 3ul)
		return -EINVAL;

	/* check if the provided address is in the mapped space */
	if (reg->reg >= esst->iosize)
		return -EINVAL;

	reg->size = 4;
	reg->val = read_reg(esst, reg->reg);
	return 0;
}

static int s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct psee_esst *esst = to_esst(sd);

	/* check if the address is aligned */
	if (reg->reg & 3ul)
		return -EINVAL;

	/* check if the provided address is in the mapped space */
	if (reg->reg >= esst->iosize)
		return -EINVAL;

	write_reg(esst, reg->reg, reg->val);
	return 0;
}
#endif

static const struct v4l2_subdev_core_ops core_ops = {
	.log_status = log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = g_register,
	.s_register = s_register,
#endif
};

static const struct v4l2_subdev_video_ops video_ops = {
	.s_stream = s_stream,
};

static const struct v4l2_subdev_pad_ops pad_ops = {
	.enum_mbus_code		= enum_mbus_code,
	.get_fmt		= get_format,
	.set_fmt		= set_format,
	.link_validate		= v4l2_subdev_link_validate_default,
};

static const struct v4l2_subdev_ops ops = {
	.core	= &core_ops,
	.video	= &video_ops,
	.pad	= &pad_ops,
};

/*
 * Media Operations
 */

static const struct media_entity_operations media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * Platform Device Driver
 */

static int parse_of(struct psee_esst *esst)
{
	struct device *dev = esst->dev;
	struct device_node *node = dev->of_node;
	struct device_node *ports;
	struct device_node *port;
	u32 port_id;
	int ret;

	ports = of_get_child_by_name(node, "ports");
	if (ports == NULL)
		ports = node;

	/* Get the format description for each pad */
	for_each_child_of_node(ports, port) {
		if (port->name && (of_node_cmp(port->name, "port") == 0)) {
			ret = of_property_read_u32(port, "reg", &port_id);
			if (ret < 0) {
				dev_err(dev, "no reg in DT");
				return ret;
			}

			if (port_id != 0 && port_id != 1) {
				dev_err(dev, "invalid reg in DT");
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int probe(struct platform_device *pdev)
{
	struct psee_esst *esst;
	struct v4l2_subdev *subdev;
	struct resource *io_space;
	int ret;

	esst = devm_kzalloc(&pdev->dev, sizeof(*esst), GFP_KERNEL);
	if (!esst)
		return -ENOMEM;

	esst->dev = &pdev->dev;

	ret = parse_of(esst);
	if (ret < 0)
		return ret;

	io_space = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	esst->iomem = devm_ioremap_resource(esst->dev, io_space);
	if (IS_ERR(esst->iomem))
		return PTR_ERR(esst->iomem);
	esst->iosize = resource_size(io_space);

	esst->clk = devm_clk_get(esst->dev, NULL);
	if (IS_ERR(esst->clk))
		return PTR_ERR(esst->clk);

	clk_prepare_enable(esst->clk);

	/* Initialize V4L2 subdevice and media entity */
	subdev = &esst->subdev;
	v4l2_subdev_init(subdev, &ops);
	/* It may not be the right function, but at least it's pixel in/pixel out */
	subdev->entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_ENC_CONV;
	subdev->dev = &pdev->dev;
	strscpy(subdev->name, dev_name(&pdev->dev), sizeof(subdev->name));
	v4l2_set_subdevdata(subdev, esst);
	subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	esst->pads[PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	esst->pads[PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	subdev->entity.ops = &media_ops;
	ret = media_entity_pads_init(&subdev->entity, 2, esst->pads);
	if (ret < 0)
		goto error;

	platform_set_drvdata(pdev, esst);

	ret = v4l2_async_register_subdev(subdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register subdev\n");
		goto error;
	}

	return 0;

error:
	media_entity_cleanup(&subdev->entity);
	clk_disable_unprepare(esst->clk);
	return ret;
}

static int remove(struct platform_device *pdev)
{
	struct psee_esst *esst = platform_get_drvdata(pdev);
	struct v4l2_subdev *subdev = &esst->subdev;

	v4l2_async_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);

	clk_disable_unprepare(esst->clk);

	return 0;
}

static const struct of_device_id of_id_table[] = {
	{ .compatible = "psee,event-stream-smart-tracker" },
	{ }
};
MODULE_DEVICE_TABLE(of, of_id_table);

static struct platform_driver psee_event_stream_smart_tracker = {
	.driver			= {
		.name		= "psee-event-stream-smart-tracker",
		.of_match_table	= of_id_table,
	},
	.probe			= probe,
	.remove			= remove,
};

module_platform_driver(psee_event_stream_smart_tracker);

MODULE_DESCRIPTION("Prophesee Event Stream Smart Tracker Driver");
MODULE_LICENSE("GPL");
