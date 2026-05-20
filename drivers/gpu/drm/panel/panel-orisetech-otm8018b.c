// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for OriseTech OTM8018B MIPI-DSI panel as found on:
 *  - Prestigio MultiPhone PAP5500 DUO (boyi variant, 480x854)
 *
 * The bootloader (MTK LK) already runs the full Orise CMD2 init sequence
 * via the in-LCM "otm8018b_dsi_vdo_lcm_drv_boyi" driver. This kernel
 * driver therefore performs only the minimum sleep-exit / display-on
 * handshake; if a true post-reset init is needed in the future, port the
 * register payload table from the qcomlt-4.9 panel-otm8018b.c (Vinay
 * Simha BN, InforceComputing) into a CMD2 init function.
 *
 * Based on drivers/gpu/drm/panel/panel-orisetech-otm8009a.c
 * Copyright (C) 2017 STMicroelectronics - All Rights Reserved
 * Author: Philippe Cornu <philippe.cornu@st.com>
 * Author: Yannick Fertre <yannick.fertre@st.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct otm8018b {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	struct regulator *vcc;
	bool prepared;
};

static const struct drm_display_mode default_mode = {
	/* Timings extracted from stock LK get_params() for the
	 * otm8018b_dsi_vdo_lcm_drv_boyi LCM driver (PAP5500 BOOTIMG.bin).
	 * htotal 690 * vtotal 900 * 60 Hz = 37.26 MHz pixel clock. */
	.clock = 37260,
	.hdisplay    = 480,
	.hsync_start = 480 + 100,
	.hsync_end   = 480 + 100 + 10,
	.htotal      = 480 + 100 + 10 + 100,
	.vdisplay    = 854,
	.vsync_start = 854 + 20,
	.vsync_end   = 854 + 20 + 6,
	.vtotal      = 854 + 20 + 6 + 20,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.width_mm  = 56,
	.height_mm = 100,
};

static inline struct otm8018b *panel_to_otm8018b(struct drm_panel *panel)
{
	return container_of(panel, struct otm8018b, panel);
}

static int otm8018b_prepare(struct drm_panel *panel)
{
	struct otm8018b *ctx = panel_to_otm8018b(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	int ret;

	ret = regulator_enable(ctx->vcc);
	if (ret < 0) {
		dev_err(panel->dev, "failed to enable vcc: %d\n", ret);
		return ret;
	}

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		usleep_range(10000, 12000);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		usleep_range(10000, 12000);
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(120);
	}

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	if (dsi_ctx.accum_err) {
		regulator_disable(ctx->vcc);
		return dsi_ctx.accum_err;
	}

	ctx->prepared = true;
	return 0;
}

static int otm8018b_unprepare(struct drm_panel *panel)
{
	struct otm8018b *ctx = panel_to_otm8018b(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	if (ctx->reset_gpio)
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	regulator_disable(ctx->vcc);
	ctx->prepared = false;
	return dsi_ctx.accum_err;
}

static int otm8018b_get_modes(struct drm_panel *panel,
			      struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode)
		return -ENOMEM;

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm  = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	return 1;
}

static const struct drm_panel_funcs otm8018b_drm_funcs = {
	.prepare   = otm8018b_prepare,
	.unprepare = otm8018b_unprepare,
	.get_modes = otm8018b_get_modes,
};

static int otm8018b_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct otm8018b *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct otm8018b, panel,
				   &otm8018b_drm_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset-gpios\n");

	ctx->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR(ctx->vcc))
		return dev_err_probe(dev, PTR_ERR(ctx->vcc),
				     "failed to request vcc supply\n");

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;

	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* Stock LK programs LCM_DSI_MODE_CON = SYNC_PULSE_VDO_MODE (= 1)
	 * at LCM_PARAMS+284. Cross-verified against the Lenovo A369i open-source
	 * MT6572 OTM8018B driver (same SoC + same panel) which uses the same mode.
	 * Dropping SYNC_PULSE makes the panel show garbage rows/columns — DO NOT
	 * remove this flag. */
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
			  MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_LPM |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach failed: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void otm8018b_remove(struct mipi_dsi_device *dsi)
{
	struct otm8018b *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id orisetech_otm8018b_of_match[] = {
	{ .compatible = "orisetech,otm8018b" },
	{ }
};
MODULE_DEVICE_TABLE(of, orisetech_otm8018b_of_match);

static struct mipi_dsi_driver orisetech_otm8018b_driver = {
	.probe  = otm8018b_probe,
	.remove = otm8018b_remove,
	.driver = {
		.name = "panel-orisetech-otm8018b",
		.of_match_table = orisetech_otm8018b_of_match,
	},
};
module_mipi_dsi_driver(orisetech_otm8018b_driver);

MODULE_DESCRIPTION("DRM driver for OriseTech OTM8018B MIPI-DSI panel");
MODULE_LICENSE("GPL v2");
