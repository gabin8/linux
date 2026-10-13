// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6572 MDP: V4L2 memory-to-memory scaler and rotator.
 *
 * Frames go MDP_RDMA -> RSZ0 -> WROT, cut into full-height vertical strips
 * (mt6572-mdp-tile.h). Each strip is one cycle of display mutex 1: take it,
 * program the engines, release it to commit the registers and start the
 * strip. MDP_RDMA's frame-done interrupt ends a strip.
 *
 * Source buffers are cached scatter-gather memory, so a decoder can render
 * into them and read them back as references at full speed; the IOMMU maps
 * each one as a single range for MDP_RDMA. Destination buffers are
 * contiguous in IOVA and meant for export to the display.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-dma-sg.h>

#include "mt6572-mdp-regs.h"
#include "mt6572-mdp-tile.h"

#define MDP_NAME		"mt6572-mdp"
#define MDP_MUTEX		1
#define MDP_MUTEX_MODS		(MUTEX_MOD_WROT | MUTEX_MOD_RSZ0 | \
				 MUTEX_MOD_MDP_RDMA)
#define MDP_MIN_SIZE		16
#define MDP_MAX_IN_W		1280	/* MDP_RDMA input limit */
#define MDP_MAX_SIZE		2047
#define MDP_MAX_PITCH		16384	/* bytes, any padding the application asks for */
#define MDP_STRIP_ROT		224	/* WROT tile width when rotating */
#define MDP_STRIP		480
#define MDP_TIMEOUT_MS		500
#define MDP_AUTOSUSPEND_MS	100

enum { MDP_RDMA, MDP_RSZ0, MDP_WROT, MDP_NR_ENGINES };

static const char * const mdp_engine_compat[MDP_NR_ENGINES] = {
	[MDP_RDMA] = "mediatek,mt6572-mdp-rdma",
	[MDP_RSZ0] = "mediatek,mt6572-mdp-rsz",	/* the first one */
	[MDP_WROT] = "mediatek,mt6572-mdp-wrot",
};

struct mdp_engine {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rst;
};

enum mdp_lb { LB_RGB16, LB_420P, LB_420SP, LB_422I };

#define MDP_FMT_SRC		BIT(0)
#define MDP_FMT_DST		BIT(1)

/* One buffer per frame; chroma planes, if any, follow the luma plane. */
struct mdp_fmt {
	u32 fourcc;
	u8 dir;			/* MDP_FMT_SRC / MDP_FMT_DST */
	u8 depth;		/* bits per pixel over all planes */
	u8 cpp;			/* bytes per pixel in plane 0 */
	u8 cpp1;		/* bytes per chroma sample in planes 1/2 */
	u8 planes;
	u8 hs, vs;		/* chroma subsampling shifts */
	bool yuv;
	u8 rdma_fmt, rdma_swap;	/* source: MDP_RDMA SRC_CON */
	bool y_lsb;
	enum mdp_lb lb;
	u8 wrot_fmt, wrot_swap;	/* destination: WROT CTRL */
};

static const struct mdp_fmt mdp_formats[] = {
	{ .fourcc = V4L2_PIX_FMT_RGB565, .dir = MDP_FMT_SRC | MDP_FMT_DST,
	  .depth = 16, .cpp = 2, .planes = 1, .rdma_fmt = RDMA_FMT_RGB565,
	  .lb = LB_RGB16, .wrot_fmt = WROT_FMT_RGB565 },
	/* memory B, G, R, X: WROT swap 0 */
	{ .fourcc = V4L2_PIX_FMT_XBGR32, .dir = MDP_FMT_DST,
	  .depth = 32, .cpp = 4, .planes = 1, .wrot_fmt = WROT_FMT_32BPP },
	{ .fourcc = V4L2_PIX_FMT_NV12, .dir = MDP_FMT_SRC, .depth = 12,
	  .cpp = 1, .cpp1 = 2, .planes = 2, .hs = 1, .vs = 1, .yuv = true,
	  .rdma_fmt = RDMA_FMT_420SP, .lb = LB_420SP },
	{ .fourcc = V4L2_PIX_FMT_NV21, .dir = MDP_FMT_SRC, .depth = 12,
	  .cpp = 1, .cpp1 = 2, .planes = 2, .hs = 1, .vs = 1, .yuv = true,
	  .rdma_fmt = RDMA_FMT_420SP, .rdma_swap = RDMA_SWAP_UV,
	  .lb = LB_420SP },
	{ .fourcc = V4L2_PIX_FMT_YUV420, .dir = MDP_FMT_SRC, .depth = 12,
	  .cpp = 1, .cpp1 = 1, .planes = 3, .hs = 1, .vs = 1, .yuv = true,
	  .rdma_fmt = RDMA_FMT_420P, .lb = LB_420P },
	/* planes in memory order (Y, V, U); stock swaps U/V instead */
	{ .fourcc = V4L2_PIX_FMT_YVU420, .dir = MDP_FMT_SRC, .depth = 12,
	  .cpp = 1, .cpp1 = 1, .planes = 3, .hs = 1, .vs = 1, .yuv = true,
	  .rdma_fmt = RDMA_FMT_420P, .rdma_swap = RDMA_SWAP_UV,
	  .lb = LB_420P },
	{ .fourcc = V4L2_PIX_FMT_YUYV, .dir = MDP_FMT_SRC, .depth = 16,
	  .cpp = 2, .planes = 1, .hs = 1, .yuv = true,
	  .rdma_fmt = RDMA_FMT_422I, .y_lsb = true, .lb = LB_422I },
	{ .fourcc = V4L2_PIX_FMT_UYVY, .dir = MDP_FMT_SRC, .depth = 16,
	  .cpp = 2, .planes = 1, .hs = 1, .yuv = true,
	  .rdma_fmt = RDMA_FMT_422I, .lb = LB_422I },
};

struct mdp_frame {
	const struct mdp_fmt *fmt;
	u32 width;
	u32 height;
	u32 bytesperline;
	u32 sizeimage;
	enum v4l2_colorspace colorspace;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;
	enum v4l2_xfer_func xfer_func;
	u32 sequence;
};

struct mdp_ctx {
	struct v4l2_fh fh;
	struct mdp_dev *mdp;
	struct v4l2_ctrl_handler ctrls;
	struct mdp_frame src;
	struct mdp_frame dst;
	unsigned int rotation;
};

struct mdp_dev {
	struct device *dev;
	struct mdp_engine eng[MDP_NR_ENGINES];
	void __iomem *mmsys;
	void __iomem *mutex;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m;
	struct mutex lock;		/* ioctl serialisation */
	struct mutex hw_lock;		/* the running job, below */
	struct delayed_work timeout;
	struct mdp_ctx *cur;
	struct mdp_plan plan;
	unsigned int strip;
	unsigned int rotation;
	unsigned int stride;
	dma_addr_t src_addr;
	dma_addr_t dst_addr;
};

static inline struct mdp_ctx *fh_to_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct mdp_ctx, fh);
}

/* ---- engines ---------------------------------------------------------- */

/*
 * The engine waits below last microseconds, so they busy-poll: a sleeping
 * poll rounds up to a whole tick without high-resolution timers, which on
 * an MT6572 with only the GPT clock event is 10 ms per strip.
 */

static void mdp_mask_write(void __iomem *reg, u32 val, u32 mask)
{
	writel((readl(reg) & ~mask) | (val & mask), reg);
}

static void mdp_rdma_reset(void __iomem *rdma)
{
	u32 sta;

	writel(1, rdma + RDMA_RESET);
	readl_poll_timeout_atomic(rdma + RDMA_MON_STA_1, sta,
				  FIELD_GET(RDMA_MON_STA_1_STATE, sta) ==
				  RDMA_STATE_IDLE, 1, 1000);
	writel(0, rdma + RDMA_RESET);
}

/*
 * The line buffer must be set up for every tile, as stock onConfigTile does
 * per source format. Left at reset, the fetch overruns it and rows come out
 * replaced by the row 16 lines further down. w is the strip width in pixels.
 */
static void mdp_rdma_line_buffer(void __iomem *rdma, enum mdp_lb lb,
				 unsigned int w)
{
	unsigned int n, c = w / 2, mf_par, sf_par, mb_con;

	writel(0, rdma + RDMA_MB_BASE);
	writel(RDMA_SRAM_SB_BASE, rdma + RDMA_SB_BASE);

	if (lb == LB_RGB16) {
		n = min(RDMA_SRAM_SIZE / (w * 2), RDMA_MAX_LB_DEPTH);
		writel(0, rdma + RDMA_MB_DEPTH);
		writel((w * 2) << 12, rdma + RDMA_MF_PAR);
		writel(0, rdma + RDMA_MB_CON);
		writel(n, rdma + RDMA_SB_DEPTH);
		writel(0, rdma + RDMA_SF_PAR);
		writel((n * DIV_ROUND_UP(w, n)) << 16 | DIV_ROUND_UP(w, n),
		       rdma + RDMA_SB_CON);
		return;
	}

	n = min(RDMA_SRAM_SIZE / w, lb == LB_422I ? 8U : 4U);
	mb_con = (n * DIV_ROUND_UP(w, n)) << 16 | DIV_ROUND_UP(w, n);
	switch (lb) {
	case LB_420SP:
		mf_par = w << 12 | n;
		sf_par = w << 12 | 2 * n;
		break;
	case LB_422I:
		mf_par = (2 * w) << 12 | 2 * n;
		sf_par = 0;
		break;
	default:	/* LB_420P */
		mf_par = w << 12 | n;
		sf_par = c << 12 | n;
		break;
	}
	writel(n, rdma + RDMA_MB_DEPTH);
	writel(mf_par, rdma + RDMA_MF_PAR);
	writel(mb_con, rdma + RDMA_MB_CON);
	writel(n, rdma + RDMA_SB_DEPTH);
	writel(sf_par, rdma + RDMA_SF_PAR);
	writel((n * DIV_ROUND_UP(c, n)) << 16 | DIV_ROUND_UP(c, n),
	       rdma + RDMA_SB_CON);
}

/* Source strip: columns x0 .. x0 + w - 1 of every row, all planes. */
static void mdp_rdma_config(struct mdp_dev *mdp, unsigned int x0,
			    unsigned int w, bool rgb2yuv)
{
	void __iomem *rdma = mdp->eng[MDP_RDMA].base;
	const struct mdp_frame *fr = &mdp->cur->src;
	const struct mdp_fmt *f = fr->fmt;
	unsigned int h = fr->height, pitch = fr->bytesperline, pitch1 = 0;
	dma_addr_t base1 = 0, base2 = 0;
	u32 con;

	if (f->planes > 1) {
		pitch1 = f->planes == 2 ? pitch : pitch >> f->hs;
		base1 = mdp->src_addr + pitch * h;
		base2 = base1 + pitch1 * (h >> f->vs);
	}
	con = FIELD_PREP(RDMA_SRC_CON_FORMAT, f->rdma_fmt) |
	      FIELD_PREP(RDMA_SRC_CON_SWAP, f->rdma_swap) |
	      (f->yuv ? RDMA_SRC_CON_CUS_REP_VH : RDMA_SRC_CON_RGB_PAD) |
	      (f->y_lsb ? RDMA_SRC_CON_Y_LSB : 0);

	mdp_rdma_reset(rdma);
	writel(0, rdma + RDMA_INT_STATUS);
	writel(RDMA_INT_FRAME_DONE, rdma + RDMA_INT_ENABLE);
	writel(0, rdma + RDMA_CON);
	writel(RDMA_GMCIF_CON_VAL, rdma + RDMA_GMCIF_CON);
	mdp_rdma_line_buffer(rdma, f->lb, w);
	writel(con, rdma + RDMA_SRC_CON);
	writel(mdp->src_addr, rdma + RDMA_SRC_BASE_0);
	writel(base1, rdma + RDMA_SRC_BASE_1);
	writel(base2, rdma + RDMA_SRC_BASE_2);
	writel(pitch, rdma + RDMA_MF_BKGD_SIZE_BYTE);
	writel(pitch1, rdma + RDMA_SF_BKGD_SIZE_BYTE);
	writel(MDP_SIZE(w, h), rdma + RDMA_MF_SRC_SIZE);
	writel(MDP_SIZE(w, h), rdma + RDMA_MF_CLIP_SIZE);
	writel(0, rdma + RDMA_MF_OFFSET_1);
	writel(mdp->src_addr + pitch * h, rdma + RDMA_SRC_END_0);
	writel(x0 * f->cpp, rdma + RDMA_SRC_OFFSET_0);
	writel((x0 >> f->hs) * f->cpp1, rdma + RDMA_SRC_OFFSET_1);
	writel((x0 >> f->hs) * f->cpp1, rdma + RDMA_SRC_OFFSET_2);
	mdp_mask_write(rdma + RDMA_TRANSFORM_0,
		       rgb2yuv ? RDMA_TRANSFORM_RGB2YUV : 0, RDMA_TRANSFORM_MASK);
	writel(1, rdma + RDMA_EN);
}

/* Once per frame: soft-reset pulse and enable, then relay or scaling setup. */
static void mdp_rsz_frame(void __iomem *rsz, const struct mdp_plan *p,
			  bool first)
{
	if (first) {
		mdp_mask_write(rsz + RSZ_ENABLE, RSZ_ENABLE_RESET, RSZ_ENABLE_RESET);
		mdp_mask_write(rsz + RSZ_ENABLE, 0, RSZ_ENABLE_RESET);
		mdp_mask_write(rsz + RSZ_ENABLE, RSZ_ENABLE_EN, RSZ_ENABLE_EN);
	}
	if (p->bypass) {
		mdp_mask_write(rsz + RSZ_ENABLE, 0, RSZ_ENABLE_EN);
		return;
	}
	mdp_mask_write(rsz + RSZ_CONTROL, p->control, RSZ_CONTROL_MASK);
	mdp_mask_write(rsz + RSZ_H_STEP, p->hstep, RSZ_STEP_MASK);
	mdp_mask_write(rsz + RSZ_V_STEP, p->vstep, RSZ_STEP_MASK);
	mdp_mask_write(rsz + RSZ_LUMA_V_INT, 0, RSZ_INT_MASK);
	mdp_mask_write(rsz + RSZ_LUMA_V_SUB, 0, RSZ_SUB_MASK);
	mdp_mask_write(rsz + RSZ_CHROMA_V_INT, 0, RSZ_INT_MASK);
	mdp_mask_write(rsz + RSZ_CHROMA_V_SUB, 0, RSZ_SUB_MASK);
}

static void mdp_rsz_strip(void __iomem *rsz, const struct mdp_plan *p,
			  const struct mdp_strip *s)
{
	u32 in = MDP_SIZE(s->ix1 - s->ix0 + 1, p->h);
	u32 out = MDP_SIZE(s->ox1 - s->ox0 + 1, p->oh);

	if (s->ix0)
		in |= RSZ_IN_LEFT_PAD;
	if (s->ix1 & 1)
		in |= RSZ_IN_ODD_END;
	if (s->ox1 < p->ow - 1)
		out |= RSZ_OUT_MORE;
	mdp_mask_write(rsz + RSZ_INPUT_IMAGE, in, RSZ_INPUT_MASK);
	mdp_mask_write(rsz + RSZ_OUTPUT_IMAGE, out, RSZ_OUTPUT_MASK);
	mdp_mask_write(rsz + RSZ_LUMA_H_INT, s->lhi, RSZ_INT_MASK);
	mdp_mask_write(rsz + RSZ_LUMA_H_SUB, s->lhs, RSZ_SUB_MASK);
	mdp_mask_write(rsz + RSZ_CHROMA_H_INT, s->chi, RSZ_INT_MASK);
	mdp_mask_write(rsz + RSZ_CHROMA_H_SUB, s->chs, RSZ_SUB_MASK);
}

/* 1-plane line-buffer lines for a WROT tile width (stock onConfigTile) */
static unsigned int mdp_wrot_buf_lines(unsigned int w)
{
	if (w <= 256)
		return min(ALIGN(w, 4), 32U);
	if (w <= 336)
		return 24;
	if (w <= 400)
		return 20;
	return 16;
}

/*
 * Byte offset of WROT's first write for the output tile starting at column
 * x of an ow x oh pre-rotation frame (stock onConfigTile). Rotated, WROT
 * walks the tile backwards from there.
 */
static u32 mdp_wrot_offset(unsigned int x, unsigned int ow, unsigned int oh,
			   unsigned int stride, unsigned int cpp,
			   unsigned int rot)
{
	switch (rot) {
	case 90:
		return x * stride + oh * cpp - 1;
	case 180:
		return (oh - 1) * stride + (ow - x) * cpp - 1;
	case 270:
		return (ow - 1 - x) * stride;
	default:
		return x * cpp;
	}
}

static void mdp_wrot_config(struct mdp_dev *mdp, const struct mdp_strip *s,
			    u32 mat)
{
	void __iomem *wrot = mdp->eng[MDP_WROT].base;
	const struct mdp_fmt *f = mdp->cur->dst.fmt;
	const struct mdp_plan *p = &mdp->plan;
	unsigned int in_w = s->ox1 - s->ox0 + 1;
	u32 sta;

	writel(1, wrot + WROT_SOFT_RST);
	readl_poll_timeout_atomic(wrot + WROT_SOFT_RST_STAT, sta,
				  !(sta & BIT(0)), 1, 1000);
	writel(0, wrot + WROT_SOFT_RST);

	writel(mdp->dst_addr, wrot + WROT_BASE_ADDR);
	writel(0, wrot + WROT_BASE_ADDR_C);
	writel(0, wrot + WROT_BASE_ADDR_V);
	writel(WROT_CTRL_UV_XY_RGB | WROT_CTRL_CROP_EN |
	       FIELD_PREP(WROT_CTRL_ROT, mdp->rotation / 90) |
	       FIELD_PREP(WROT_CTRL_SWAP, f->wrot_swap) |
	       FIELD_PREP(WROT_CTRL_FMT, f->wrot_fmt), wrot + WROT_CTRL);
	writel(mdp->stride, wrot + WROT_STRIDE);
	writel(0, wrot + WROT_STRIDE_C);
	writel(0, wrot + WROT_STRIDE_V);
	writel(WROT_DMA_PERF_VAL, wrot + WROT_DMA_PERF);
	mdp_mask_write(wrot + WROT_MAT_CTRL, mat, WROT_MAT_CTRL_MASK);
	writel(WROT_DITHER_VAL, wrot + WROT_DITHER);
	writel(WROT_RSV_1_VAL, wrot + WROT_RSV_1);
	writel(1, wrot + WROT_ROT_EN);
	writel(mdp_wrot_offset(s->wx0, p->ow, p->oh, mdp->stride, f->cpp,
			       mdp->rotation), wrot + WROT_OFST_ADDR);
	writel(0, wrot + WROT_OFST_ADDR_C);
	writel(0, wrot + WROT_OFST_ADDR_V);
	writel(MDP_SIZE(in_w, p->oh), wrot + WROT_IN_SIZE);
	writel(MDP_SIZE(s->ox1 - s->wx0 + 1, p->oh), wrot + WROT_TAR_SIZE);
	writel(s->wx0 - s->ox0, wrot + WROT_CROP_OFST);
	writel(in_w << 16 | mdp_wrot_buf_lines(in_w) << 8,
	       wrot + WROT_MAIN_BUF_SIZE);
}

static int mdp_wrot_wait_idle(struct mdp_dev *mdp)
{
	void __iomem *wrot = mdp->eng[MDP_WROT].base;
	u32 v;

	writel(WROT_DBG_SEL_STATE, wrot + WROT_DBG_SEL);
	return readl_poll_timeout_atomic(wrot + WROT_DBG_OUT, v,
					 !(v & WROT_DBG_BUSY), 1, 10000);
}

static void mdp_route(struct mdp_dev *mdp, bool on)
{
	void __iomem *mmsys = mdp->mmsys;

	writel(on ? MDP_RDMA_MOUT_RSZ0 : 0, mmsys + MMSYS_MDP_RDMA_MOUT_EN);
	writel(on ? MDP_RSZ0_MOUT_WROT : 0, mmsys + MMSYS_MDP_RSZ0_MOUT_EN);
	writel(0, mmsys + MMSYS_MDP_RSZ1_MOUT_EN);
	writel(0, mmsys + MMSYS_MDP_TDSHP_MOUT_EN);
	writel(MDP_RSZ0_SEL_RDMA, mmsys + MMSYS_MDP_RSZ0_SEL);
	writel(0, mmsys + MMSYS_MDP_RSZ1_SEL);
	writel(0, mmsys + MMSYS_MDP_TDSHP_SEL);
	writel(MDP_WROT_SEL_RSZ0, mmsys + MMSYS_MDP_WROT_SEL);
	writel(0, mmsys + MMSYS_MDP_WDMA_SEL);
}

/* Take the mutex: register writes stay pending until it is released. */
static int mdp_mutex_get(struct mdp_dev *mdp)
{
	void __iomem *mutex = mdp->mutex;
	u32 v;
	int ret;

	writel(1, mutex + MUTEX_EN(MDP_MUTEX));
	writel(1, mutex + MUTEX_RST(MDP_MUTEX));
	writel(0, mutex + MUTEX_RST(MDP_MUTEX));
	writel(MUTEX_GET_REQ, mutex + MUTEX_GET(MDP_MUTEX));
	ret = readl_poll_timeout_atomic(mutex + MUTEX_GET(MDP_MUTEX), v,
					v & MUTEX_GET_GRANTED, 1, 5000);
	writel(MDP_MUTEX_MODS, mutex + MUTEX_MOD(MDP_MUTEX));
	writel(MUTEX_SOF_SINGLE, mutex + MUTEX_SOF(MDP_MUTEX));
	return ret;
}

/* Releasing the mutex commits the registers and starts the strip. */
static int mdp_mutex_go(struct mdp_dev *mdp)
{
	void __iomem *mutex = mdp->mutex;
	u32 v;
	int ret;

	writel(0, mutex + MUTEX_GET(MDP_MUTEX));
	ret = readl_poll_timeout_atomic(mutex + MUTEX_REG_COMMIT, v,
					!(v & MDP_MUTEX_MODS), 1, 5000);
	v = readl(mutex + MUTEX_INTSTA);
	writel(v & ~BIT(MDP_MUTEX), mutex + MUTEX_INTSTA);
	return ret;
}

static void mdp_mutex_stop(struct mdp_dev *mdp)
{
	writel(0, mdp->mutex + MUTEX_EN(MDP_MUTEX));
	writel(0, mdp->mutex + MUTEX_MOD(MDP_MUTEX));
}

/*
 * Hard reset, only to recover from a failed job: every strip soft-resets
 * the engines anyway, and each MMSYS reset sleeps for a millisecond.
 */
static void mdp_reset_engines(struct mdp_dev *mdp)
{
	int i;

	for (i = 0; i < MDP_NR_ENGINES; i++)
		reset_control_reset(mdp->eng[i].rst);
}

/* ---- jobs ------------------------------------------------------------- */

/*
 * Colour handling. YUV sources are converted to RGB in WROT with the matrix
 * of their encoding. RGB sources pass through, except that RSZ scales
 * channels 1/2 as half-width chroma, so a scaled RGB frame goes through
 * YUV (full range) and back, as stock does.
 */
static void mdp_csc(const struct mdp_ctx *ctx, bool scaled, bool *rgb2yuv,
		    u32 *mat)
{
	const struct mdp_frame *src = &ctx->src;

	*rgb2yuv = false;
	*mat = 0;
	if (src->fmt->yuv) {
		if (src->quantization == V4L2_QUANTIZATION_FULL_RANGE)
			*mat = WROT_MAT_FULL;
		else if (src->ycbcr_enc == V4L2_YCBCR_ENC_709)
			*mat = WROT_MAT_BT709;
		else
			*mat = WROT_MAT_BT601;
	} else if (scaled) {
		*rgb2yuv = true;
		*mat = WROT_MAT_FULL;
	}
}

static int mdp_start_strip(struct mdp_dev *mdp)
{
	const struct mdp_strip *s = &mdp->plan.s[mdp->strip];
	bool rgb2yuv;
	u32 mat;
	int ret;

	mdp_csc(mdp->cur, !mdp->plan.bypass, &rgb2yuv, &mat);
	ret = mdp_mutex_get(mdp);
	if (ret)
		return ret;
	mdp_route(mdp, true);
	mdp_rdma_config(mdp, s->ix0, s->ix1 - s->ix0 + 1, rgb2yuv);
	mdp_rsz_frame(mdp->eng[MDP_RSZ0].base, &mdp->plan, !mdp->strip);
	mdp_rsz_strip(mdp->eng[MDP_RSZ0].base, &mdp->plan, s);
	mdp_wrot_config(mdp, s, mat);
	return mdp_mutex_go(mdp);
}

static void mdp_job_done(struct mdp_dev *mdp, enum vb2_buffer_state state)
{
	struct mdp_ctx *ctx = mdp->cur;
	struct vb2_v4l2_buffer *src, *dst;

	mdp_mutex_stop(mdp);
	mdp_route(mdp, false);
	if (state != VB2_BUF_STATE_DONE)
		mdp_reset_engines(mdp);
	mdp->cur = NULL;

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	vb2_set_plane_payload(&dst->vb2_buf, 0, ctx->dst.sizeimage);
	v4l2_m2m_buf_done(src, state);
	v4l2_m2m_buf_done(dst, state);

	pm_runtime_mark_last_busy(mdp->dev);
	pm_runtime_put_autosuspend(mdp->dev);
	v4l2_m2m_job_finish(mdp->m2m, ctx->fh.m2m_ctx);
}

static void mdp_device_run(void *priv)
{
	struct mdp_ctx *ctx = priv;
	struct mdp_dev *mdp = ctx->mdp;
	struct vb2_v4l2_buffer *src, *dst;
	unsigned int ow, oh, max;
	struct sg_table *sgt;
	int ret;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	v4l2_m2m_buf_copy_metadata(src, dst);
	src->sequence = ctx->src.sequence++;
	dst->sequence = ctx->dst.sequence++;

	mutex_lock(&mdp->hw_lock);
	mdp->cur = ctx;
	ret = pm_runtime_resume_and_get(mdp->dev);
	if (ret) {
		mdp->cur = NULL;
		mutex_unlock(&mdp->hw_lock);
		v4l2_m2m_buf_done_and_job_finish(mdp->m2m, ctx->fh.m2m_ctx,
						 VB2_BUF_STATE_ERROR);
		return;
	}

	mdp->rotation = ctx->rotation;
	ow = mdp->rotation % 180 ? ctx->dst.height : ctx->dst.width;
	oh = mdp->rotation % 180 ? ctx->dst.width : ctx->dst.height;
	max = mdp->rotation % 180 ? MDP_STRIP_ROT : MDP_STRIP;
	if (mdp_plan_strips(&mdp->plan, ctx->src.width, ctx->src.height, ow,
			    oh, max)) {
		dev_dbg(mdp->dev, "cannot scale %ux%u to %ux%u\n",
			ctx->src.width, ctx->src.height, ow, oh);
		mdp_job_done(mdp, VB2_BUF_STATE_ERROR);
		mutex_unlock(&mdp->hw_lock);
		return;
	}
	sgt = vb2_dma_sg_plane_desc(&src->vb2_buf, 0);
	mdp->src_addr = sg_dma_address(sgt->sgl);
	mdp->dst_addr = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	mdp->stride = ctx->dst.bytesperline;
	mdp->strip = 0;

	schedule_delayed_work(&mdp->timeout, msecs_to_jiffies(MDP_TIMEOUT_MS));
	ret = mdp_start_strip(mdp);
	if (ret) {
		cancel_delayed_work(&mdp->timeout);
		dev_err(mdp->dev, "strip 0 did not start: %d\n", ret);
		mdp_job_done(mdp, VB2_BUF_STATE_ERROR);
	}
	mutex_unlock(&mdp->hw_lock);
}

static irqreturn_t mdp_irq(int irq, void *data)
{
	struct mdp_dev *mdp = data;
	void __iomem *rdma = mdp->eng[MDP_RDMA].base;

	if (!(readl(rdma + RDMA_INT_STATUS) & RDMA_INT_FRAME_DONE))
		return IRQ_NONE;
	writel(0, rdma + RDMA_INT_STATUS);
	return IRQ_WAKE_THREAD;
}

/* MDP_RDMA has read the strip; once WROT is idle, run the next one. */
static irqreturn_t mdp_irq_thread(int irq, void *data)
{
	struct mdp_dev *mdp = data;
	int ret;

	mutex_lock(&mdp->hw_lock);
	if (!mdp->cur)
		goto out;
	ret = mdp_wrot_wait_idle(mdp);
	if (!ret && ++mdp->strip < mdp->plan.n)
		ret = mdp_start_strip(mdp);
	if (ret || mdp->strip >= mdp->plan.n) {
		cancel_delayed_work(&mdp->timeout);
		if (ret)
			dev_err(mdp->dev, "strip %u failed: %d\n", mdp->strip,
				ret);
		mdp_job_done(mdp, ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	}
out:
	mutex_unlock(&mdp->hw_lock);
	return IRQ_HANDLED;
}

static void mdp_timeout(struct work_struct *work)
{
	struct mdp_dev *mdp = container_of(to_delayed_work(work),
					   struct mdp_dev, timeout);

	mutex_lock(&mdp->hw_lock);
	if (mdp->cur) {
		dev_err(mdp->dev, "timeout in strip %u of %u\n", mdp->strip,
			mdp->plan.n);
		mdp_job_done(mdp, VB2_BUF_STATE_ERROR);
	}
	mutex_unlock(&mdp->hw_lock);
}

static const struct v4l2_m2m_ops mdp_m2m_ops = {
	.device_run = mdp_device_run,
};

/* ---- formats ---------------------------------------------------------- */

static struct mdp_frame *mdp_frame(struct mdp_ctx *ctx, enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->src : &ctx->dst;
}

static const struct mdp_fmt *mdp_find_fmt(u32 fourcc, u8 dir)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mdp_formats); i++)
		if (mdp_formats[i].fourcc == fourcc && mdp_formats[i].dir & dir)
			return &mdp_formats[i];
	return NULL;
}

static const struct mdp_fmt *mdp_try_pix(struct v4l2_pix_format *pix,
					 bool output)
{
	u8 dir = output ? MDP_FMT_SRC : MDP_FMT_DST;
	const struct mdp_fmt *f = mdp_find_fmt(pix->pixelformat, dir);

	if (!f)
		f = &mdp_formats[0];
	pix->pixelformat = f->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->width = clamp(ALIGN(pix->width, 2), MDP_MIN_SIZE,
			   output ? MDP_MAX_IN_W : MDP_MAX_SIZE);
	pix->height = clamp(ALIGN(pix->height, 2), MDP_MIN_SIZE, MDP_MAX_SIZE);
	/*
	 * Keep a padded line from the application: GStreamer copies frames
	 * at its own stride (I420 854 wide -> 856) whatever the driver says.
	 * The chroma planes of 3-plane formats run at half the luma pitch.
	 */
	pix->bytesperline = clamp(pix->bytesperline, pix->width * f->cpp,
				  MDP_MAX_PITCH);
	if (f->planes == 3)
		pix->bytesperline = ALIGN(pix->bytesperline, 2);
	pix->sizeimage = pix->bytesperline * pix->height * f->depth /
			 (f->cpp * 8);
	if (!pix->colorspace)
		pix->colorspace = f->yuv ? V4L2_COLORSPACE_SMPTE170M :
					   V4L2_COLORSPACE_SRGB;
	return f;
}

/* The capture side reports the colorimetry of the output side. */
static void mdp_copy_colorimetry(struct v4l2_pix_format *pix,
				 const struct mdp_frame *src)
{
	pix->colorspace = src->colorspace;
	pix->xfer_func = src->xfer_func;
	pix->ycbcr_enc = src->ycbcr_enc;
	pix->quantization = src->quantization;
}

static int mdp_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	strscpy(cap->driver, MDP_NAME, sizeof(cap->driver));
	strscpy(cap->card, "MediaTek MT6572 MDP", sizeof(cap->card));
	return 0;
}

static int mdp_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	u8 dir = V4L2_TYPE_IS_OUTPUT(f->type) ? MDP_FMT_SRC : MDP_FMT_DST;
	unsigned int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(mdp_formats); i++) {
		if (!(mdp_formats[i].dir & dir))
			continue;
		if (n++ == f->index) {
			f->pixelformat = mdp_formats[i].fourcc;
			return 0;
		}
	}
	return -EINVAL;
}

static int mdp_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mdp_frame *fr = mdp_frame(fh_to_ctx(file), f->type);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->pixelformat = fr->fmt->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->width = fr->width;
	pix->height = fr->height;
	pix->bytesperline = fr->bytesperline;
	pix->sizeimage = fr->sizeimage;
	pix->colorspace = fr->colorspace;
	pix->ycbcr_enc = fr->ycbcr_enc;
	pix->quantization = fr->quantization;
	pix->xfer_func = fr->xfer_func;
	return 0;
}

static int mdp_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mdp_ctx *ctx = fh_to_ctx(file);

	mdp_try_pix(&f->fmt.pix, V4L2_TYPE_IS_OUTPUT(f->type));
	if (!V4L2_TYPE_IS_OUTPUT(f->type))
		mdp_copy_colorimetry(&f->fmt.pix, &ctx->src);
	return 0;
}

static int mdp_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mdp_ctx *ctx = fh_to_ctx(file);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	struct mdp_frame *fr = mdp_frame(ctx, f->type);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (vb2_is_busy(vq))
		return -EBUSY;
	fr->fmt = mdp_try_pix(pix, V4L2_TYPE_IS_OUTPUT(f->type));
	if (!V4L2_TYPE_IS_OUTPUT(f->type))
		mdp_copy_colorimetry(pix, &ctx->src);
	fr->width = pix->width;
	fr->height = pix->height;
	fr->bytesperline = pix->bytesperline;
	fr->sizeimage = pix->sizeimage;
	fr->colorspace = pix->colorspace;
	fr->ycbcr_enc = pix->ycbcr_enc;
	fr->quantization = pix->quantization;
	fr->xfer_func = pix->xfer_func;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		ctx->dst.colorspace = fr->colorspace;
		ctx->dst.xfer_func = fr->xfer_func;
		ctx->dst.ycbcr_enc = fr->ycbcr_enc;
		ctx->dst.quantization = fr->quantization;
	}
	return 0;
}

static const struct v4l2_ioctl_ops mdp_ioctl_ops = {
	.vidioc_querycap		= mdp_querycap,
	.vidioc_enum_fmt_vid_cap	= mdp_enum_fmt,
	.vidioc_enum_fmt_vid_out	= mdp_enum_fmt,
	.vidioc_g_fmt_vid_cap		= mdp_g_fmt,
	.vidioc_g_fmt_vid_out		= mdp_g_fmt,
	.vidioc_try_fmt_vid_cap		= mdp_try_fmt,
	.vidioc_try_fmt_vid_out		= mdp_try_fmt,
	.vidioc_s_fmt_vid_cap		= mdp_s_fmt,
	.vidioc_s_fmt_vid_out		= mdp_s_fmt,
	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/* ---- queues ----------------------------------------------------------- */

static int mdp_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct mdp_frame *fr = mdp_frame(vb2_get_drv_priv(vq), vq->type);

	if (*nplanes)
		return sizes[0] < fr->sizeimage ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = fr->sizeimage;
	return 0;
}

static int mdp_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;
	return vbuf->field == V4L2_FIELD_NONE ? 0 : -EINVAL;
}

static int mdp_buf_prepare(struct vb2_buffer *vb)
{
	struct mdp_frame *fr = mdp_frame(vb2_get_drv_priv(vb->vb2_queue),
					 vb->type);

	if (vb2_plane_size(vb, 0) < fr->sizeimage)
		return -EINVAL;
	if (!V4L2_TYPE_IS_OUTPUT(vb->type))
		vb2_set_plane_payload(vb, 0, fr->sizeimage);
	/* MDP_RDMA takes one base address: the IOMMU must map it in one piece */
	else if (vb2_dma_sg_plane_desc(vb, 0)->nents != 1)
		return -EINVAL;
	return 0;
}

static void mdp_buf_queue(struct vb2_buffer *vb)
{
	struct mdp_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int mdp_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct mdp_ctx *ctx = vb2_get_drv_priv(vq);

	mdp_frame(ctx, vq->type)->sequence = 0;
	return 0;
}

static void mdp_stop_streaming(struct vb2_queue *vq)
{
	struct mdp_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vb;

	for (;;) {
		vb = V4L2_TYPE_IS_OUTPUT(vq->type) ?
		     v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		     v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vb)
			break;
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops mdp_qops = {
	.queue_setup	= mdp_queue_setup,
	.buf_out_validate = mdp_buf_out_validate,
	.buf_prepare	= mdp_buf_prepare,
	.buf_queue	= mdp_buf_queue,
	.start_streaming = mdp_start_streaming,
	.stop_streaming	= mdp_stop_streaming,
};

static int mdp_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct mdp_ctx *ctx = priv;
	struct mdp_dev *mdp = ctx->mdp;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &mdp_qops;
	src_vq->mem_ops = &vb2_dma_sg_memops;	/* cleaned at QBUF */
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &mdp->lock;
	src_vq->dev = mdp->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &mdp_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &mdp->lock;
	dst_vq->dev = mdp->eng[MDP_WROT].dev;
	return vb2_queue_init(dst_vq);
}

/* ---- controls, open/release ------------------------------------------- */

static int mdp_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mdp_ctx *ctx = container_of(ctrl->handler, struct mdp_ctx,
					   ctrls);

	if (ctrl->id == V4L2_CID_ROTATE)
		ctx->rotation = ctrl->val;
	return 0;
}

static const struct v4l2_ctrl_ops mdp_ctrl_ops = {
	.s_ctrl = mdp_s_ctrl,
};

static void mdp_default_frame(struct mdp_frame *fr)
{
	struct v4l2_pix_format pix = { .width = 640, .height = 480 };

	fr->fmt = mdp_try_pix(&pix, true);
	fr->width = pix.width;
	fr->height = pix.height;
	fr->bytesperline = pix.bytesperline;
	fr->sizeimage = pix.sizeimage;
	fr->colorspace = pix.colorspace;
	fr->ycbcr_enc = pix.ycbcr_enc;
	fr->quantization = pix.quantization;
	fr->xfer_func = pix.xfer_func;
}

static int mdp_open(struct file *file)
{
	struct mdp_dev *mdp = video_drvdata(file);
	struct mdp_ctx *ctx;
	int ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;
	ctx->mdp = mdp;
	mdp_default_frame(&ctx->src);
	mdp_default_frame(&ctx->dst);

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	v4l2_ctrl_handler_init(&ctx->ctrls, 1);
	v4l2_ctrl_new_std(&ctx->ctrls, &mdp_ctrl_ops, V4L2_CID_ROTATE, 0, 270,
			  90, 0);
	if (ctx->ctrls.error) {
		ret = ctx->ctrls.error;
		goto err_ctrls;
	}
	ctx->fh.ctrl_handler = &ctx->ctrls;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(mdp->m2m, ctx, mdp_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_ctrls;
	}
	v4l2_fh_add(&ctx->fh, file);
	return 0;

err_ctrls:
	v4l2_ctrl_handler_free(&ctx->ctrls);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int mdp_release(struct file *file)
{
	struct mdp_ctx *ctx = fh_to_ctx(file);
	struct mdp_dev *mdp = ctx->mdp;

	mutex_lock(&mdp->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&mdp->lock);
	v4l2_ctrl_handler_free(&ctx->ctrls);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations mdp_fops = {
	.owner		= THIS_MODULE,
	.open		= mdp_open,
	.release	= mdp_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/* ---- platform --------------------------------------------------------- */

static int mdp_runtime_suspend(struct device *dev)
{
	struct mdp_dev *mdp = dev_get_drvdata(dev);
	int i;

	for (i = MDP_NR_ENGINES - 1; i >= 0; i--)
		clk_disable_unprepare(mdp->eng[i].clk);
	return 0;
}

static int mdp_runtime_resume(struct device *dev)
{
	struct mdp_dev *mdp = dev_get_drvdata(dev);
	int i, ret;

	for (i = 0; i < MDP_NR_ENGINES; i++) {
		ret = clk_prepare_enable(mdp->eng[i].clk);
		if (ret) {
			while (--i >= 0)
				clk_disable_unprepare(mdp->eng[i].clk);
			return ret;
		}
	}
	return 0;
}

static void mdp_put_device(void *dev)
{
	put_device(dev);
}

static void mdp_put_clk(void *clk)
{
	clk_put(clk);
}

static void mdp_put_reset(void *rst)
{
	reset_control_put(rst);
}

static void mdp_iounmap(void *base)
{
	iounmap(base);
}

/* The other engines and the shared blocks are found by compatible. */
static int mdp_get_engine(struct mdp_dev *mdp, int i)
{
	struct mdp_engine *e = &mdp->eng[i];
	struct device *dev = mdp->dev;
	struct platform_device *pdev;
	struct device_node *np;
	int ret;

	np = i == MDP_RDMA ? of_node_get(dev->of_node) :
	     of_find_compatible_node(NULL, NULL, mdp_engine_compat[i]);
	if (!np)
		return dev_err_probe(dev, -ENODEV, "no %s node\n",
				     mdp_engine_compat[i]);

	pdev = of_find_device_by_node(np);
	e->clk = of_clk_get(np, 0);
	e->rst = of_reset_control_get_exclusive(np, NULL);
	e->base = devm_of_iomap(dev, np, 0, NULL);
	of_node_put(np);

	if (!pdev)
		return -EPROBE_DEFER;
	e->dev = &pdev->dev;
	ret = devm_add_action_or_reset(dev, mdp_put_device, e->dev);
	if (ret)
		return ret;
	if (IS_ERR(e->clk))
		return dev_err_probe(dev, PTR_ERR(e->clk), "%s clock\n",
				     mdp_engine_compat[i]);
	ret = devm_add_action_or_reset(dev, mdp_put_clk, e->clk);
	if (ret)
		return ret;
	if (IS_ERR(e->rst))
		return dev_err_probe(dev, PTR_ERR(e->rst), "%s reset\n",
				     mdp_engine_compat[i]);
	ret = devm_add_action_or_reset(dev, mdp_put_reset, e->rst);
	if (ret)
		return ret;
	if (IS_ERR(e->base))
		return dev_err_probe(dev, PTR_ERR(e->base), "%s registers\n",
				     mdp_engine_compat[i]);
	return 0;
}

/* MMSYS and the mutex belong to the display drivers: map, don't claim. */
static void __iomem *mdp_map_shared(struct mdp_dev *mdp, const char *compat)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, compat);
	void __iomem *base;

	if (!np)
		return NULL;
	base = of_iomap(np, 0);
	of_node_put(np);
	if (base && devm_add_action_or_reset(mdp->dev, mdp_iounmap, base))
		return NULL;
	return base;
}

static int mdp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mdp_dev *mdp;
	struct video_device *vdev;
	int i, irq, ret;

	mdp = devm_kzalloc(dev, sizeof(*mdp), GFP_KERNEL);
	if (!mdp)
		return -ENOMEM;
	mdp->dev = dev;
	platform_set_drvdata(pdev, mdp);
	mutex_init(&mdp->lock);
	mutex_init(&mdp->hw_lock);
	INIT_DELAYED_WORK(&mdp->timeout, mdp_timeout);

	for (i = 0; i < MDP_NR_ENGINES; i++) {
		ret = mdp_get_engine(mdp, i);
		if (ret)
			return ret;
	}
	mdp->mmsys = mdp_map_shared(mdp, "mediatek,mt6572-mmsys");
	mdp->mutex = mdp_map_shared(mdp, "mediatek,mt6572-disp-mutex");
	if (!mdp->mmsys || !mdp->mutex)
		return dev_err_probe(dev, -ENODEV, "no mmsys or mutex\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_threaded_irq(dev, irq, mdp_irq, mdp_irq_thread,
					IRQF_ONESHOT, dev_name(dev), mdp);
	if (ret)
		return ret;

	/* let the IOMMU map a scatter-gather buffer as one IOVA range */
	for (i = 0; i < MDP_NR_ENGINES; i++)
		dma_set_max_seg_size(mdp->eng[i].dev, UINT_MAX);

	pm_runtime_set_autosuspend_delay(dev, MDP_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = v4l2_device_register(dev, &mdp->v4l2_dev);
	if (ret)
		return ret;
	mdp->m2m = v4l2_m2m_init(&mdp_m2m_ops);
	if (IS_ERR(mdp->m2m)) {
		ret = PTR_ERR(mdp->m2m);
		goto err_v4l2;
	}

	vdev = &mdp->vdev;
	strscpy(vdev->name, MDP_NAME, sizeof(vdev->name));
	vdev->fops = &mdp_fops;
	vdev->ioctl_ops = &mdp_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->lock = &mdp->lock;
	vdev->v4l2_dev = &mdp->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, mdp);
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	dev_info(dev, "registered %s\n", video_device_node_name(vdev));
	return 0;

err_m2m:
	v4l2_m2m_release(mdp->m2m);
err_v4l2:
	v4l2_device_unregister(&mdp->v4l2_dev);
	return ret;
}

static void mdp_remove(struct platform_device *pdev)
{
	struct mdp_dev *mdp = platform_get_drvdata(pdev);

	video_unregister_device(&mdp->vdev);
	cancel_delayed_work_sync(&mdp->timeout);
	v4l2_m2m_release(mdp->m2m);
	v4l2_device_unregister(&mdp->v4l2_dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(mdp_pm_ops, mdp_runtime_suspend,
				 mdp_runtime_resume, NULL);

static const struct of_device_id mdp_of_match[] = {
	{ .compatible = "mediatek,mt6572-mdp-rdma" },
	{ }
};
MODULE_DEVICE_TABLE(of, mdp_of_match);

static struct platform_driver mdp_driver = {
	.probe = mdp_probe,
	.remove = mdp_remove,
	.driver = {
		.name = MDP_NAME,
		.of_match_table = mdp_of_match,
		.pm = pm_ptr(&mdp_pm_ops),
	},
};
module_platform_driver(mdp_driver);

MODULE_DESCRIPTION("MediaTek MT6572 MDP scaler and rotator");
MODULE_LICENSE("GPL");
