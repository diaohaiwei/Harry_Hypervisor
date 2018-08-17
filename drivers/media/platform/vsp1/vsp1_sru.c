/*
 * vsp1_sru.c  --  R-Car VSP1 Super Resolution Unit
 *
 * Copyright (C) 2013 Renesas Corporation
 *
 * Contact: Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/gfp.h>

#include <media/v4l2-subdev.h>

#include "vsp1.h"
#include "vsp1_dl.h"
#include "vsp1_sru.h"

#define SRU_MIN_SIZE				4U
#define SRU_MAX_SIZE				8190U

/* -----------------------------------------------------------------------------
 * Device Access
 */

static inline void vsp1_sru_write(struct vsp1_sru *sru, struct vsp1_dl_list *dl,
				  u32 reg, u32 data)
{
	vsp1_dl_list_write(dl, reg, data);
}

/* -----------------------------------------------------------------------------
 * Controls
 */

#define V4L2_CID_VSP1_SRU_INTENSITY		(V4L2_CID_USER_BASE | 0x1001)

struct vsp1_sru_param {
	u32 ctrl0;
	u32 ctrl2;
};

#define VI6_SRU_CTRL0_PARAMS(p0, p1)			\
	(((p0) << VI6_SRU_CTRL0_PARAM0_SHIFT) |		\
	 ((p1) << VI6_SRU_CTRL0_PARAM1_SHIFT))

#define VI6_SRU_CTRL2_PARAMS(p6, p7, p8)		\
	(((p6) << VI6_SRU_CTRL2_PARAM6_SHIFT) |		\
	 ((p7) << VI6_SRU_CTRL2_PARAM7_SHIFT) |		\
	 ((p8) << VI6_SRU_CTRL2_PARAM8_SHIFT))

static const struct vsp1_sru_param vsp1_sru_params[] = {
	{
		.ctrl0 = VI6_SRU_CTRL0_PARAMS(256, 4) | VI6_SRU_CTRL0_EN,
		.ctrl2 = VI6_SRU_CTRL2_PARAMS(24, 40, 255),
	}, {
		.ctrl0 = VI6_SRU_CTRL0_PARAMS(256, 4) | VI6_SRU_CTRL0_EN,
		.ctrl2 = VI6_SRU_CTRL2_PARAMS(8, 16, 255),
	}, {
		.ctrl0 = VI6_SRU_CTRL0_PARAMS(384, 5) | VI6_SRU_CTRL0_EN,
		.ctrl2 = VI6_SRU_CTRL2_PARAMS(36, 60, 255),
	}, {
		.ctrl0 = VI6_SRU_CTRL0_PARAMS(384, 5) | VI6_SRU_CTRL0_EN,
		.ctrl2 = VI6_SRU_CTRL2_PARAMS(12, 27, 255),
	}, {
		.ctrl0 = VI6_SRU_CTRL0_PARAMS(511, 6) | VI6_SRU_CTRL0_EN,
		.ctrl2 = VI6_SRU_CTRL2_PARAMS(48, 80, 255),
	}, {
		.ctrl0 = VI6_SRU_CTRL0_PARAMS(511, 6) | VI6_SRU_CTRL0_EN,
		.ctrl2 = VI6_SRU_CTRL2_PARAMS(16, 36, 255),
	},
};

static int sru_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vsp1_sru *sru =
		container_of(ctrl->handler, struct vsp1_sru, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_VSP1_SRU_INTENSITY:
		sru->intensity = ctrl->val;
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops sru_ctrl_ops = {
	.s_ctrl = sru_s_ctrl,
};

static const struct v4l2_ctrl_config sru_intensity_control = {
	.ops = &sru_ctrl_ops,
	.id = V4L2_CID_VSP1_SRU_INTENSITY,
	.name = "Intensity",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1,
	.max = 6,
	.def = 1,
	.step = 1,
};

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */

static int sru_enum_mbus_code(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	static const unsigned int codes[] = {
		MEDIA_BUS_FMT_ARGB8888_1X32,
		MEDIA_BUS_FMT_AYUV8_1X32,
	};

	return vsp1_subdev_enum_mbus_code(subdev, cfg, code, codes,
					  ARRAY_SIZE(codes));
}

static int sru_enum_frame_size(struct v4l2_subdev *subdev,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct vsp1_sru *sru = to_sru(subdev);
	struct v4l2_subdev_pad_config *config;
	struct v4l2_mbus_framefmt *format;
	int ret = 0;

	config = vsp1_entity_get_pad_config(&sru->entity, cfg, fse->which);
	if (!config)
		return -EINVAL;

	format = vsp1_entity_get_pad_format(&sru->entity, config, SRU_PAD_SINK);

	mutex_lock(&sru->entity.lock);

	if (fse->index || fse->code != format->code) {
		ret = -EINVAL;
		goto done;
	}

	if (fse->pad == SRU_PAD_SINK) {
		fse->min_width = SRU_MIN_SIZE;
		fse->max_width = SRU_MAX_SIZE;
		fse->min_height = SRU_MIN_SIZE;
		fse->max_height = SRU_MAX_SIZE;
	} else {
		fse->min_width = format->width;
		fse->min_height = format->height;
		if (format->width <= SRU_MAX_SIZE / 2 &&
		    format->height <= SRU_MAX_SIZE / 2 &&
		    sru->force_identity_mode == false) {
			fse->max_width = format->width * 2;
			fse->max_height = format->height * 2;
		} else {
			fse->max_width = format->width;
			fse->max_height = format->height;
		}
	}

done:
	mutex_unlock(&sru->entity.lock);
	return ret;
}

static void sru_try_format(struct vsp1_sru *sru,
			   struct v4l2_subdev_pad_config *config,
			   unsigned int pad, struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_mbus_framefmt *format;
	unsigned int input_area;
	unsigned int output_area;

	switch (pad) {
	case SRU_PAD_SINK:
		/* Default to YUV if the requested format is not supported. */
		if (fmt->code != MEDIA_BUS_FMT_ARGB8888_1X32 &&
		    fmt->code != MEDIA_BUS_FMT_AYUV8_1X32)
			fmt->code = MEDIA_BUS_FMT_AYUV8_1X32;

		fmt->width = clamp(fmt->width, SRU_MIN_SIZE, SRU_MAX_SIZE);
		fmt->height = clamp(fmt->height, SRU_MIN_SIZE, SRU_MAX_SIZE);
		break;

	case SRU_PAD_SOURCE:
		/* The SRU can't perform format conversion. */
		format = vsp1_entity_get_pad_format(&sru->entity, config,
						    SRU_PAD_SINK);
		fmt->code = format->code;

		/* We can upscale by 2 in both direction, but not independently.
		 * Compare the input and output rectangles areas (avoiding
		 * integer overflows on the output): if the requested output
		 * area is larger than 1.5^2 the input area upscale by two,
		 * otherwise don't scale.
		 */
		input_area = format->width * format->height;
		output_area = min(fmt->width, SRU_MAX_SIZE)
			    * min(fmt->height, SRU_MAX_SIZE);

		if (fmt->width <= SRU_MAX_SIZE / 2 &&
		    fmt->height <= SRU_MAX_SIZE / 2 &&
		    output_area > input_area * 9 / 4 &&
		    sru->force_identity_mode == false) {
			fmt->width = format->width * 2;
			fmt->height = format->height * 2;
		} else {
			fmt->width = format->width;
			fmt->height = format->height;
		}
		break;
	}

	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

static int sru_set_format(struct v4l2_subdev *subdev,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct vsp1_sru *sru = to_sru(subdev);
	struct v4l2_subdev_pad_config *config;
	struct v4l2_mbus_framefmt *format;
	int ret = 0;

	mutex_lock(&sru->entity.lock);

	config = vsp1_entity_get_pad_config(&sru->entity, cfg, fmt->which);
	if (!config) {
		ret = -EINVAL;
		goto done;
	}

	sru_try_format(sru, config, fmt->pad, &fmt->format);

	format = vsp1_entity_get_pad_format(&sru->entity, config, fmt->pad);
	*format = fmt->format;

	if (fmt->pad == SRU_PAD_SINK) {
		/* Propagate the format to the source pad. */
		format = vsp1_entity_get_pad_format(&sru->entity, config,
						    SRU_PAD_SOURCE);
		*format = fmt->format;

		sru_try_format(sru, config, SRU_PAD_SOURCE, format);
	}

done:
	mutex_unlock(&sru->entity.lock);
	return ret;
}

static const struct v4l2_subdev_pad_ops sru_pad_ops = {
	.init_cfg = vsp1_entity_init_cfg,
	.enum_mbus_code = sru_enum_mbus_code,
	.enum_frame_size = sru_enum_frame_size,
	.get_fmt = vsp1_subdev_get_pad_format,
	.set_fmt = sru_set_format,
};

static const struct v4l2_subdev_ops sru_ops = {
	.pad    = &sru_pad_ops,
};

/* -----------------------------------------------------------------------------
 * VSP1 Entity Operations
 */

static void sru_configure(struct vsp1_entity *entity,
			  struct vsp1_pipeline *pipe,
			  struct vsp1_dl_list *dl,
			  enum vsp1_entity_params params)
{
	const struct vsp1_sru_param *param;
	struct vsp1_sru *sru = to_sru(&entity->subdev);
	struct v4l2_mbus_framefmt *input;
	struct v4l2_mbus_framefmt *output;
	u32 ctrl0;

	if (params != VSP1_ENTITY_PARAMS_INIT)
		return;

	input = vsp1_entity_get_pad_format(&sru->entity, sru->entity.config,
					   SRU_PAD_SINK);
	output = vsp1_entity_get_pad_format(&sru->entity, sru->entity.config,
					    SRU_PAD_SOURCE);

	if (input->code == MEDIA_BUS_FMT_ARGB8888_1X32)
		ctrl0 = VI6_SRU_CTRL0_PARAM2 | VI6_SRU_CTRL0_PARAM3
		      | VI6_SRU_CTRL0_PARAM4;
	else
		ctrl0 = VI6_SRU_CTRL0_PARAM3;

	if (input->width != output->width)
		ctrl0 |= VI6_SRU_CTRL0_MODE_UPSCALE;

	param = &vsp1_sru_params[sru->intensity - 1];

	ctrl0 |= param->ctrl0;

	vsp1_sru_write(sru, dl, VI6_SRU_CTRL0, ctrl0);
	vsp1_sru_write(sru, dl, VI6_SRU_CTRL1, VI6_SRU_CTRL1_PARAM5);
	vsp1_sru_write(sru, dl, VI6_SRU_CTRL2, param->ctrl2);
}

static unsigned int sru_max_width(struct vsp1_entity *entity,
				  struct vsp1_pipeline *pipe)
{
	struct vsp1_sru *sru = to_sru(&entity->subdev);
	struct v4l2_mbus_framefmt *input;
	struct v4l2_mbus_framefmt *output;

	input = vsp1_entity_get_pad_format(&sru->entity, sru->entity.config,
					   SRU_PAD_SINK);
	output = vsp1_entity_get_pad_format(&sru->entity, sru->entity.config,
					    SRU_PAD_SOURCE);

	if (input->width != output->width)
		return 512;
	else
		return 256;
}

static const struct vsp1_entity_operations sru_entity_ops = {
	.configure = sru_configure,
	.max_width = sru_max_width,
};

/* -----------------------------------------------------------------------------
 * Initialization and Cleanup
 */

struct vsp1_sru *vsp1_sru_create(struct vsp1_device *vsp1)
{
	struct vsp1_sru *sru;
	int ret;

	sru = devm_kzalloc(vsp1->dev, sizeof(*sru), GFP_KERNEL);
	if (sru == NULL)
		return ERR_PTR(-ENOMEM);

	sru->entity.ops = &sru_entity_ops;
	sru->entity.type = VSP1_ENTITY_SRU;

	ret = vsp1_entity_init(vsp1, &sru->entity, "sru", 2, &sru_ops,
			       MEDIA_ENT_F_PROC_VIDEO_SCALER);
	if (ret < 0)
		return ERR_PTR(ret);

	/* Initialize the control handler. */
	v4l2_ctrl_handler_init(&sru->ctrls, 1);
	v4l2_ctrl_new_custom(&sru->ctrls, &sru_intensity_control, NULL);

	sru->intensity = 1;
	sru->force_identity_mode = false;

	sru->entity.subdev.ctrl_handler = &sru->ctrls;

	if (sru->ctrls.error) {
		dev_err(vsp1->dev, "sru: failed to initialize controls\n");
		ret = sru->ctrls.error;
		vsp1_entity_destroy(&sru->entity);
		return ERR_PTR(ret);
	}

	return sru;
}