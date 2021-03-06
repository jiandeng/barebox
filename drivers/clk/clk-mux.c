/*
 * clk-mux.c - generic barebox clock support. Based on Linux clk support
 *
 * Copyright (c) 2012 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <common.h>
#include <io.h>
#include <malloc.h>
#include <linux/clk.h>
#include <linux/err.h>

struct clk_mux {
	struct clk clk;
	void __iomem *reg;
	int shift;
	int width;
};

static int clk_mux_get_parent(struct clk *clk)
{
	struct clk_mux *m = container_of(clk, struct clk_mux, clk);
	int idx = readl(m->reg) >> m->shift & ((1 << m->width) - 1);

	return idx;
}

static int clk_mux_set_parent(struct clk *clk, u8 idx)
{
	struct clk_mux *m = container_of(clk, struct clk_mux, clk);
	u32 val;

	val = readl(m->reg);
	val &= ~(((1 << m->width) - 1) << m->shift);
	val |= idx << m->shift;
	writel(val, m->reg);

	return 0;
}

struct clk_ops clk_mux_ops = {
	.get_parent = clk_mux_get_parent,
	.set_parent = clk_mux_set_parent,
};

struct clk *clk_mux(const char *name, void __iomem *reg,
		u8 shift, u8 width, const char **parents, u8 num_parents)
{
	struct clk_mux *m = xzalloc(sizeof(*m));
	int ret;

	m->reg = reg;
	m->shift = shift;
	m->width = width;
	m->clk.ops = &clk_mux_ops;
	m->clk.name = name;
	m->clk.parent_names = parents;
	m->clk.num_parents = num_parents;

	ret = clk_register(&m->clk);
	if (ret) {
		free(m);
		return ERR_PTR(ret);
	}

	return &m->clk;
}
