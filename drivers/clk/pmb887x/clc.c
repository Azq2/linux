// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "pmb887x-clc: " fmt

#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/reboot.h>

#define	PMB887X_MOD_CLC					0x00
#define	PMB887X_MOD_CLC_DISR			BIT(0)
#define	PMB887X_MOD_CLC_DISS			BIT(1)
#define	PMB887X_MOD_CLC_SPEN			BIT(2)
#define	PMB887X_MOD_CLC_EDIS			BIT(3)
#define	PMB887X_MOD_CLC_SBWE			BIT(4)
#define	PMB887X_MOD_CLC_FSOE			BIT(5)
#define	PMB887X_MOD_CLC_RMC				GENMASK(15, 8)
#define	PMB887X_MOD_CLC_RMC_SHIFT		8

#define to_clc(_hw) container_of(_hw, struct pmb887x_clc, hw)

struct pmb887x_clc {
	struct clk_hw hw;
	void __iomem *reg;
	u32 rmc;
};

static int pmb887x_clc_enable(struct clk_hw *hw) {
	struct pmb887x_clc *clc = to_clc(hw);
	writel((clc->rmc << PMB887X_MOD_CLC_RMC_SHIFT), clc->reg);
	return 0;
}

static void pmb887x_clc_disable(struct clk_hw *hw) {
	struct pmb887x_clc *clc = to_clc(hw);
	writel(0, clc->reg);
}

static int pmb887x_clc_is_enabled(struct clk_hw *hw) {
	struct pmb887x_clc *clc = to_clc(hw);
	return (readl(clc->reg) & PMB887X_MOD_CLC_RMC) != 0;
}

static int pmb887x_clc_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate) {
	struct pmb887x_clc *clc = to_clc(hw);
	unsigned long reg_value;
	
	clc->rmc = parent_rate / rate;
	
	if (pmb887x_clc_is_enabled(hw)) {
		reg_value = readl(clc->reg) & ~PMB887X_MOD_CLC_RMC;
		reg_value |= clc->rmc << PMB887X_MOD_CLC_RMC_SHIFT;
		writel(reg_value, clc->reg);
	}
	
	return 0;
}

static long pmb887x_clc_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *parent_rate) {
	u32 new_rmc = DIV_ROUND_CLOSEST(*parent_rate, rate);
	return *parent_rate / max(1U, min(0xFFU, new_rmc));
}

static unsigned long pmb887x_clc_recalc_rate(struct clk_hw *hw, unsigned long parent_rate) {
	struct pmb887x_clc *clc = to_clc(hw);
	return parent_rate / clc->rmc;
}

static const struct clk_ops pmb887x_clc_ops = {
	.enable			= pmb887x_clc_enable,
	.disable		= pmb887x_clc_disable,
	.is_enabled		= pmb887x_clc_is_enabled,
	.recalc_rate	= pmb887x_clc_recalc_rate,
	.set_rate		= pmb887x_clc_set_rate,
	.round_rate		= pmb887x_clc_round_rate,
};

static struct clk_hw *__init pmb887x_clc_register(void __iomem *reg, const char *name, const char *parent_name) {
	int ret;
	struct pmb887x_clc *clc;
	struct clk_init_data init;
	u32 rmc;
	
	clc = kzalloc(sizeof(*clc), GFP_KERNEL);
	if (!clc)
		return ERR_PTR(-ENOMEM);
	
	init.name = name;
	init.ops = &pmb887x_clc_ops;
	init.flags = CLK_GET_RATE_NOCACHE;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);
	
	clc->hw.init = &init;
	clc->reg = reg + PMB887X_MOD_CLC;
	
	rmc = (readl(clc->reg) & PMB887X_MOD_CLC_RMC) >> PMB887X_MOD_CLC_RMC_SHIFT;
	clc->rmc = rmc ? rmc : 1;
	
	pr_debug("register clock: %s\n", name);
	
	ret = clk_hw_register(NULL, &clc->hw);
	if (ret) {
		kfree(clc);
		return ERR_PTR(ret);
	}
	
	return &clc->hw;
}

static void __init pmb887x_clc_of(struct device_node *np) {
	struct clk_hw *hw;
	const char *clk_name = np->name;
	const char *parent_name;
	void __iomem *reg;
	int index;
	u32 rate;
	
	index = of_property_match_string(of_get_parent(np), "reg-names", "pmb887x-clc");
	reg = of_iomap(of_get_parent(np), index > 0 ? index : 0);
	if (!reg) {
		pr_err("%pOFn: must have REG in the parent node.\n", np);
		return;
	}
	
	parent_name = of_clk_get_parent_name(np, 0);
	hw = pmb887x_clc_register(reg, clk_name, parent_name);
	if (IS_ERR(hw)) {
		pr_err("%pOFn: clk registration fail.\n", np);
		return;
	}
	
	of_clk_add_hw_provider(np, of_clk_hw_simple_get, hw);
	
	pr_debug("%pOFn: registered clk", np);
	
	if (of_property_read_u32(np, "clock-frequency", &rate) == 0) {
		pr_debug("%pOFn: set initial freq: %d Hz", np, rate);
		clk_set_rate(hw->clk, rate);
	}
}

CLK_OF_DECLARE(pmb887x_clk_clc, "pmb887x,clc", pmb887x_clc_of);
