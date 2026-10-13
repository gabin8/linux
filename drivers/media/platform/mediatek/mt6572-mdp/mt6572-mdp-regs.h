/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT6572 MDP engine registers.
 *
 * MDP_RDMA and the MMSYS/mutex registers follow the MT6572 functional spec.
 * The spec has no RSZ or WROT register map; those layouts, and the values
 * marked "stock", follow what the stock MT6572 MDP library writes.
 */
#ifndef MT6572_MDP_REGS_H
#define MT6572_MDP_REGS_H

#include <linux/bits.h>

/* MDP_RDMA (8-byte register stride) */
#define RDMA_EN			0x000	/* arms the engine; the mutex SOF starts it */
#define RDMA_RESET		0x008
#define RDMA_INT_ENABLE		0x010
#define RDMA_INT_STATUS		0x018	/* write 0 to clear */
#define RDMA_INT_FRAME_DONE	BIT(0)
#define RDMA_CON		0x020
#define RDMA_GMCIF_CON		0x028
#define RDMA_GMCIF_CON_VAL	0x00020370	/* stock: 0x20070 under mask 0x30070 */
#define RDMA_SRC_CON		0x030
#define RDMA_SRC_CON_FORMAT	GENMASK(3, 0)
#define RDMA_SRC_CON_SWAP	GENMASK(6, 4)
#define RDMA_SRC_CON_RGB_PAD	BIT(7)		/* expand 565 with zero LSBs */
#define RDMA_SRC_CON_CUS_REP_VH	(3 << 9)	/* chroma up-sampling: V+H filter */
#define RDMA_SRC_CON_Y_LSB	BIT(16)		/* packed 4:2:2 with Y first */
#define RDMA_FMT_420P		0
#define RDMA_FMT_420SP		1
#define RDMA_FMT_422I		6
#define RDMA_FMT_RGB565		12
#define RDMA_SWAP_UV		2
#define RDMA_SRC_BASE_0		0x040
#define RDMA_SRC_BASE_1		0x048
#define RDMA_SRC_BASE_2		0x050
#define RDMA_MF_BKGD_SIZE_BYTE	0x060
#define RDMA_MF_SRC_SIZE	0x070	/* [28:16] H, [12:0] W */
#define RDMA_MF_CLIP_SIZE	0x078
#define RDMA_MF_OFFSET_1	0x080
#define RDMA_MF_PAR		0x088	/* [28:12] line bytes */
#define RDMA_SF_BKGD_SIZE_BYTE	0x090
#define RDMA_SF_PAR		0x0b8
#define RDMA_MB_DEPTH		0x0c0	/* resets to 16 lines: must be set */
#define RDMA_MB_BASE		0x0c8
#define RDMA_MB_CON		0x0d0
#define RDMA_SB_DEPTH		0x0d8
#define RDMA_SB_BASE		0x0e0
#define RDMA_SB_CON		0x0e8	/* [29:16] depth * blocks, [12:0] blocks */
#define RDMA_SRC_END_0		0x100
#define RDMA_SRC_OFFSET_0	0x118
#define RDMA_SRC_OFFSET_1	0x120
#define RDMA_SRC_OFFSET_2	0x128
#define RDMA_TRANSFORM_0	0x200	/* resets with RGB->YUV on */
#define RDMA_TRANSFORM_MASK	0x0f110000
#define RDMA_TRANSFORM_RGB2YUV	BIT(16)	/* matrix 0: JPEG full range */
#define RDMA_MON_STA_1		0x408
#define RDMA_MON_STA_1_STATE	GENMASK(18, 8)
#define RDMA_STATE_IDLE		0x1
#define RDMA_SRAM_SB_BASE	0xa00
#define RDMA_SRAM_SIZE		2560
#define RDMA_MAX_LB_DEPTH	8

/* RSZ (stock DpEngine_SCL); written under the stock masks */
#define RSZ_ENABLE		0x000
#define RSZ_ENABLE_EN		BIT(0)
#define RSZ_ENABLE_RESET	BIT(16)
#define RSZ_CONTROL		0x004
#define RSZ_CONTROL_MASK	0x040001f3
#define RSZ_INPUT_IMAGE		0x00c
#define RSZ_INPUT_MASK		0xdfff1fff
#define RSZ_IN_LEFT_PAD		BIT(30)	/* 2 px of left context present */
#define RSZ_IN_ODD_END		BIT(31)	/* last input column is odd */
#define RSZ_OUTPUT_IMAGE	0x010
#define RSZ_OUTPUT_MASK		0x9fff1fff
#define RSZ_OUT_MORE		BIT(31)	/* more strips follow */
#define RSZ_H_STEP		0x014
#define RSZ_V_STEP		0x018
#define RSZ_STEP_MASK		0x007fffff
#define RSZ_LUMA_H_INT		0x01c
#define RSZ_LUMA_H_SUB		0x020
#define RSZ_LUMA_V_INT		0x024
#define RSZ_LUMA_V_SUB		0x028
#define RSZ_CHROMA_H_INT	0x02c
#define RSZ_CHROMA_H_SUB	0x030
#define RSZ_CHROMA_V_INT	0x034
#define RSZ_CHROMA_V_SUB	0x038
#define RSZ_INT_MASK		0x1fff
#define RSZ_SUB_MASK		0x1fffff

/* WROT. Base address registers at 0x028/0x034/0x064, not the MT8183 layout */
#define WROT_CTRL		0x000
#define WROT_CTRL_FMT		GENMASK(2, 0)
#define WROT_FMT_RGB565		4
#define WROT_FMT_32BPP		5
#define WROT_CTRL_SWAP		GENMASK(9, 8)
#define WROT_CTRL_CROP_EN	BIT(12)		/* stock always sets it */
#define WROT_CTRL_ROT		GENMASK(21, 20)	/* clockwise, 90 degree steps */
#define WROT_CTRL_UV_XY_RGB	(0xa << 28)	/* stock value for RGB */
#define WROT_DMA_PERF		0x004
#define WROT_DMA_PERF_VAL	0x07000000	/* stock */
#define WROT_MAIN_BUF_SIZE	0x008	/* [28:16] tile W, [14:8] lines */
#define WROT_SOFT_RST		0x010
#define WROT_SOFT_RST_STAT	0x014
#define WROT_DBG_SEL		0x018	/* [13:8] debug select */
#define WROT_DBG_SEL_STATE	0xb00
#define WROT_CROP_OFST		0x020
#define WROT_TAR_SIZE		0x024	/* [28:16] H, [12:0] W */
#define WROT_BASE_ADDR		0x028
#define WROT_OFST_ADDR		0x02c	/* byte offset of the first write */
#define WROT_STRIDE		0x030
#define WROT_BASE_ADDR_C	0x034
#define WROT_OFST_ADDR_C	0x038
#define WROT_STRIDE_C		0x03c
#define WROT_DITHER		0x054
#define WROT_DITHER_VAL		0xff000000	/* stock: alpha */
#define WROT_BASE_ADDR_V	0x064
#define WROT_OFST_ADDR_V	0x068
#define WROT_STRIDE_V		0x06c
#define WROT_RSV_1		0x070
#define WROT_RSV_1_VAL		0x80000000	/* stock: EOL select */
#define WROT_IN_SIZE		0x078	/* [28:16] H, [12:0] W */
#define WROT_ROT_EN		0x07c
#define WROT_MAT_CTRL		0x084
#define WROT_MAT_CTRL_MASK	0xf3
#define WROT_MAT_FULL		0x41	/* YUV->RGB, JPEG full range */
#define WROT_MAT_BT601		0x61	/* YUV->RGB, BT.601 limited range */
#define WROT_MAT_BT709		0x71	/* YUV->RGB, BT.709 limited range */
#define WROT_DBG_OUT		0x0d0
#define WROT_DBG_BUSY		GENMASK(4, 0)

#define MDP_SIZE(w, h)		((h) << 16 | (w))

/* MMSYS_CONFIG MDP muxes; 0 is the reset value and "unused" */
#define MMSYS_MDP_RDMA_MOUT_EN	0x020
#define MDP_RDMA_MOUT_RSZ0	BIT(0)
#define MMSYS_MDP_RSZ0_MOUT_EN	0x024
#define MDP_RSZ0_MOUT_WROT	BIT(1)
#define MMSYS_MDP_RSZ1_MOUT_EN	0x028
#define MMSYS_MDP_TDSHP_MOUT_EN	0x02c
#define MMSYS_MDP_RSZ0_SEL	0x038
#define MDP_RSZ0_SEL_RDMA	0
#define MMSYS_MDP_RSZ1_SEL	0x03c
#define MMSYS_MDP_TDSHP_SEL	0x040
#define MMSYS_MDP_WROT_SEL	0x044
#define MDP_WROT_SEL_RSZ0	0
#define MMSYS_MDP_WDMA_SEL	0x048

/* DISP_MUTEX. The display owns mutex 0; stock gives mutex 1 to MDP */
#define MUTEX_INTSTA		0x004	/* write 0 to clear */
#define MUTEX_REG_COMMIT	0x00c	/* per-module "new values pending" */
#define MUTEX_EN(n)		(0x20 + 0x20 * (n))
#define MUTEX_GET(n)		(0x24 + 0x20 * (n))
#define MUTEX_GET_REQ		BIT(0)
#define MUTEX_GET_GRANTED	BIT(1)
#define MUTEX_RST(n)		(0x28 + 0x20 * (n))
#define MUTEX_MOD(n)		(0x2c + 0x20 * (n))
#define MUTEX_SOF(n)		(0x30 + 0x20 * (n))
#define MUTEX_SOF_SINGLE	0
#define MUTEX_MOD_WROT		BIT(0)
#define MUTEX_MOD_RSZ0		BIT(1)
#define MUTEX_MOD_MDP_RDMA	BIT(5)

#endif
