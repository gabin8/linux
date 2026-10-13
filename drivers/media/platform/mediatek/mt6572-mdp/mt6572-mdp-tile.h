/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT6572 MDP strip planner for MDP_RDMA -> RSZ -> WROT, following the stock
 * libdpframework tile calculation (DpTileEngine / DpEngine_SCL 4-tap helpers).
 * Frames are cut into full-height vertical strips, left to right.
 */
#ifndef MT6572_MDP_TILE_H
#define MT6572_MDP_TILE_H

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/minmax.h>
#include <linux/types.h>

#define MDP_MAX_STRIPS		32
#define RSZ_P			32768		/* 4-tap step unit, 2^15 */
#define RSZ_MAX_W		288		/* RSZ line buffer, 4-tap */
#define RSZ_CTRL_H_EN		BIT(0)
#define RSZ_CTRL_V_EN		BIT(1)
#define RSZ_CTRL_H_UP		BIT(4)
#define RSZ_CTRL_FIXED		BIT(26)		/* always set by stock */

struct mdp_strip {
	unsigned int ix0, ix1;	/* MDP_RDMA source columns */
	unsigned int ox0, ox1;	/* RSZ output columns = WROT input */
	unsigned int wx0;	/* first WROT output column */
	unsigned int lhi, lhs;	/* RSZ luma H integer / sub-pixel offset */
	unsigned int chi, chs;	/* RSZ chroma H integer / sub-pixel offset */
};

struct mdp_plan {
	unsigned int w, h, ow, oh;
	bool bypass;		/* 1:1, RSZ relays */
	unsigned int control, hstep, vstep;
	unsigned int n;
	struct mdp_strip s[MDP_MAX_STRIPS];
};

static inline unsigned int mdp_odd_down(unsigned int x)
{
	return (x & 1) ? x : x - 1;
}

static inline unsigned int mdp_rsz_step(unsigned int in, unsigned int out)
{
	return ((in - 1) * RSZ_P + (out - 1) / 2) / (out - 1);
}

/*
 * Plan a w x h -> ow x oh job whose WROT strips are at most wrot_max wide.
 * Only the 4-tap range (down to 1/2) is supported: -EINVAL beyond it.
 */
static inline int mdp_plan_strips(struct mdp_plan *p, unsigned int w,
				  unsigned int h, unsigned int ow,
				  unsigned int oh, unsigned int wrot_max)
{
	unsigned int w2 = (w + 1) & ~1u, ow2 = (ow + 1) & ~1u;
	unsigned int x0 = 0, hstep;
	bool hdown = ow < w, hup = ow > w;

	p->w = w;
	p->h = h;
	p->ow = ow;
	p->oh = oh;
	p->n = 0;
	p->bypass = w == ow && h == oh;

	if (p->bypass) {
		for (x0 = 0; x0 < w; x0 += wrot_max) {
			unsigned int x1 = min(x0 + wrot_max, w) - 1;

			if (p->n == MDP_MAX_STRIPS)
				return -EINVAL;
			p->s[p->n++] = (struct mdp_strip) {
				.ix0 = x0, .ix1 = x1,
				.ox0 = x0, .ox1 = x1,
				.wx0 = x0,
			};
		}
		return 0;
	}

	if (w < 2 || h < 2 || ow < 2 || oh < 2 ||
	    w > 2 * (ow - 1) || h > 2 * (oh - 1))
		return -EINVAL;	/* beyond 4-tap: needs the accumulation modes */

	hstep = mdp_rsz_step(w, ow);
	p->hstep = hstep;
	p->vstep = mdp_rsz_step(h, oh);
	p->control = RSZ_CTRL_FIXED | RSZ_CTRL_V_EN | RSZ_CTRL_H_EN |
		     (hup ? RSZ_CTRL_H_UP : 0);

	for (;;) {
		struct mdp_strip *s;
		unsigned int wx1, l, r, is, ie, ix0, ix1, f0, f1, oe, rel;

		if (p->n == MDP_MAX_STRIPS)
			return -EINVAL;
		s = &p->s[p->n++];

		/* backward: WROT -> RSZ -> RDMA */
		wx1 = min(x0 + wrot_max - 1, ow - 1);
		if (wx1 == ow - 1)
			r = ow2 - 1;
		else
			r = (wx1 & 1) ? wx1 + 2 : wx1 - 1;	/* 2 extra out px */
		l = x0 & ~1u;
		if (hdown && r + 1 >= l + RSZ_MAX_W)
			r = mdp_odd_down(l + RSZ_MAX_W - 1);

		is = l * hstep < RSZ_P ? 0 : ((l * hstep / RSZ_P - 1) & ~1u);
		if (r * hstep + 2 * RSZ_P > RSZ_P * (w2 - 1))
			ie = w2 - 1;
		else
			ie = (r * hstep / RSZ_P + 2) | 1;
		ix0 = is >= 2 ? is - 2 : 0;		/* 2-px left pad */
		ix1 = ie + 1 >= w2 ? w - 1 : ie;
		if (hup)
			ix1 = min(ix1, ix0 + RSZ_MAX_W - 1);

		/* forward: RDMA -> RSZ -> WROT */
		f0 = ix0 ? ix0 + 2 : 0;
		if (ix1 + 1 >= w)
			f1 = w2 - 1;
		else
			f1 = (ix1 & 1) ? ix1 : ix1 - 1;
		if (f1 == w2 - 1)
			oe = ow2 - 1;
		else	/* largest x with x * hstep < (f1 - 1) * P */
			oe = mdp_odd_down(((f1 - 1) * RSZ_P - 1) / hstep);
		rel = l * hstep - f0 * RSZ_P;
		oe = min(oe, r);

		s->ix0 = ix0;
		s->ix1 = ix1;
		s->ox0 = l;
		s->ox1 = oe >= ow2 - 1 ? ow - 1 : oe - 2;
		s->wx0 = x0;
		s->lhi = rel / RSZ_P;
		s->lhs = rel % RSZ_P;
		s->chi = s->lhi >> 1;
		s->chs = s->lhs + (s->lhi & 1) * RSZ_P;

		if (s->ox1 < x0)
			return -EINVAL;	/* no progress */
		if (s->ox1 == ow - 1)
			return 0;
		x0 = s->ox1 + 1;
	}
}

#endif
