/*
 * pinctrl-kinetis.c - iomux for Kinetis MCUs
 *
 * Based on pinctrl-imx.c by Dong Aisheng <dong.aisheng@linaro.org>
 *
 * Copyright (C) 2015 Paul Osmialowski <pawelo@king.net.pl>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>

#define MAX_PORT_NUM	6

static struct kinetis_pinctrl *kpdev = NULL;

/**
 * struct kinetis_pin_group - describes a single pin
 * @mux_reg: the mux register for this pin.
 * @mux_mode: the mux mode for this pin.
 */
struct kinetis_pin {
	u32 mux_reg;
	u32 mux_mode;
};

/**
 * struct kinetis_pin_group - describes a pin group
 * @name: the name of this specific pin group
 * @npins: the number of pins in this group array, i.e. the number of
 *	elements in .pins so we can iterate over that array
 * @pins: array of pins
 */
struct kinetis_pin_group {
	const char *name;
	unsigned npins;
	struct kinetis_pin *pins;
};

/**
 * struct kinetis_pmx_func - describes pinmux functions
 * @name: the name of this specific function
 * @groups: corresponding pin groups
 * @num_groups: the number of groups
 */
struct kinetis_pmx_func {
	const char *name;
	const char **groups;
	unsigned num_groups;
};

struct kinetis_pinctrl_soc_info {
	struct device			*dev;
	struct kinetis_pin_group	*groups;
	unsigned int			ngroups;
	struct kinetis_pmx_func		*functions;
	unsigned int			nfunctions;
};

struct kinetis_pinctrl {
	struct device			*dev;
	struct pinctrl_dev		*pctl;
	void __iomem			*base;
	struct clk			*clk[MAX_PORT_NUM];
	struct kinetis_pinctrl_soc_info	info;
};

static const struct of_device_id kinetis_pinctrl_of_match[] = {
	{ .compatible = "fsl,kinetis-padconf", },
	{ /* sentinel */ }
};

/*
 * PORTx register map
 */
struct kinetis_port_regs {
	u32 pcr[32];	/* Pin Control Registers */
	u32 gpclr;	/* Global Pin Control Low Register */
	u32 gpchr;	/* Global Pin Control High Register */
	u32 rsv0[6];
	u32 isfr;	/* Interrupt Status Flag Register */
};

/*
 * PORTx registers base
 */
#define KINETIS_PORT_PTR(base, reg) \
	(&(((struct kinetis_port_regs *)(base))->reg))
#define KINETIS_PORT_RD(base, reg) readl_relaxed(KINETIS_PORT_PTR(base, reg))
#define KINETIS_PORT_WR(base, reg, val) \
		writel_relaxed((val), KINETIS_PORT_PTR(base, reg))
#define KINETIS_PORT_SET(base, reg, mask) \
	KINETIS_PORT_WR(base, reg, (KINETIS_PORT_RD(base, reg)) | (mask))
#define KINETIS_PORT_RESET(base, reg, mask) \
	KINETIS_PORT_WR(base, reg, (KINETIS_PORT_RD(base, reg)) & (~(mask)))

static const struct kinetis_pin_group *kinetis_pinctrl_find_group_by_name(
				const struct kinetis_pinctrl_soc_info *info,
				const char *name)
{
	const struct kinetis_pin_group *grp = NULL;
	int i;

	for (i = 0; i < info->ngroups; i++) {
		if (!strcmp(info->groups[i].name, name)) {
			grp = &info->groups[i];
			break;
		}
	}

	return grp;
}

static int kinetis_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct kinetis_pinctrl *kpctl = pinctrl_dev_get_drvdata(pctldev);
	const struct kinetis_pinctrl_soc_info *info = &kpctl->info;

	return info->ngroups;
}

static const char *kinetis_get_group_name(struct pinctrl_dev *pctldev,
				unsigned selector)
{
	struct kinetis_pinctrl *kpctl = pinctrl_dev_get_drvdata(pctldev);
	const struct kinetis_pinctrl_soc_info *info = &kpctl->info;

	return info->groups[selector].name;
}

static int kinetis_dt_node_to_map(struct pinctrl_dev *pctldev,
			struct device_node *np,
			struct pinctrl_map **map, unsigned *num_maps)
{
	struct kinetis_pinctrl *kpctl = pinctrl_dev_get_drvdata(pctldev);
	const struct kinetis_pinctrl_soc_info *info = &kpctl->info;
	const struct kinetis_pin_group *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;

	/*
	 * first find the group of this node and check if we need create
	 * config maps for pins
	 */
	grp = kinetis_pinctrl_find_group_by_name(info, np->name);
	if (!grp) {
		dev_err(info->dev, "unable to find group for node %s\n",
			np->name);
		return -EINVAL;
	}

	new_map = kmalloc(sizeof(struct pinctrl_map), GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	*map = new_map;
	*num_maps = 1;

	/* create mux map */
	parent = of_get_parent(np);
	if (!parent) {
		dev_err(info->dev, "%s has no parent\n", np->name);
		kfree(new_map);
		return -EINVAL;
	}
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = parent->name;
	new_map[0].data.mux.group = np->name;
	of_node_put(parent);

	dev_dbg(info->dev, "maps: function %s group %s\n",
		(*map)->data.mux.function, (*map)->data.mux.group);

	return 0;
}

static void kinetis_dt_free_map(struct pinctrl_dev *pctldev,
				struct pinctrl_map *map, unsigned num_maps)
{
	kfree(map);
}

static int kinetis_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct kinetis_pinctrl *kpctl = pinctrl_dev_get_drvdata(pctldev);
	const struct kinetis_pinctrl_soc_info *info = &kpctl->info;

	return info->nfunctions;
}

static const char *kinetis_get_function_name(struct pinctrl_dev *pctldev,
				unsigned selector)
{
	struct kinetis_pinctrl *kpctl = pinctrl_dev_get_drvdata(pctldev);
	const struct kinetis_pinctrl_soc_info *info = &kpctl->info;

	return info->functions[selector].name;
}

static int kinetis_get_function_groups(struct pinctrl_dev *pctldev,
				unsigned selector,
				const char * const **groups,
				unsigned * const num_groups)
{
	struct kinetis_pinctrl *kpctl = pinctrl_dev_get_drvdata(pctldev);
	const struct kinetis_pinctrl_soc_info *info = &kpctl->info;

	*groups = info->functions[selector].groups;
	*num_groups = info->functions[selector].num_groups;

	return 0;
}

static int kinetis_set_mux(struct pinctrl_dev *pctldev, unsigned selector,
				unsigned group)
{
	struct kinetis_pinctrl *kpctl = pinctrl_dev_get_drvdata(pctldev);
	const struct kinetis_pinctrl_soc_info *info = &kpctl->info;
	struct kinetis_pin_group *grp = &info->groups[group];
	unsigned int npins = grp->npins;
	struct kinetis_pin *pin;
	int i;

	for (i = 0; i < npins; i++) {
		pin = &grp->pins[i];
		KINETIS_PORT_WR(kpctl->base + 0x1000 * (pin->mux_reg / 32),
			pcr[pin->mux_reg % 32], pin->mux_mode);
	}

	return 0;
}

static const struct pinctrl_ops kinetis_pctrl_ops = {
	.get_groups_count = kinetis_get_groups_count,
	.get_group_name = kinetis_get_group_name,
	.dt_node_to_map = kinetis_dt_node_to_map,
	.dt_free_map = kinetis_dt_free_map,
};

static const struct pinmux_ops kinetis_pmx_ops = {
	.get_functions_count = kinetis_get_functions_count,
	.get_function_name = kinetis_get_function_name,
	.get_function_groups = kinetis_get_function_groups,
	.set_mux = kinetis_set_mux,
};

static const struct pinctrl_pin_desc kinetis_pinctrl_pads[] = {
	{ .number = 0, .name = "PORTA_0" },
	{ .number = 1, .name = "PORTA_1" },
	{ .number = 2, .name = "PORTA_2" },
	{ .number = 3, .name = "PORTA_3" },
	{ .number = 4, .name = "PORTA_4" },
	{ .number = 5, .name = "PORTA_5" },
	{ .number = 6, .name = "PORTA_6" },
	{ .number = 7, .name = "PORTA_7" },
	{ .number = 8, .name = "PORTA_8" },
	{ .number = 9, .name = "PORTA_9" },
	{ .number = 10, .name = "PORTA_10" },
	{ .number = 11, .name = "PORTA_11" },
	{ .number = 12, .name = "PORTA_12" },
	{ .number = 13, .name = "PORTA_13" },
	{ .number = 14, .name = "PORTA_14" },
	{ .number = 15, .name = "PORTA_15" },
	{ .number = 16, .name = "PORTA_16" },
	{ .number = 17, .name = "PORTA_17" },
	{ .number = 18, .name = "PORTA_18" },
	{ .number = 19, .name = "PORTA_19" },
	{ .number = 20, .name = "PORTA_20" },
	{ .number = 21, .name = "PORTA_21" },
	{ .number = 22, .name = "PORTA_22" },
	{ .number = 23, .name = "PORTA_23" },
	{ .number = 24, .name = "PORTA_24" },
	{ .number = 25, .name = "PORTA_25" },
	{ .number = 26, .name = "PORTA_26" },
	{ .number = 27, .name = "PORTA_27" },
	{ .number = 28, .name = "PORTA_28" },
	{ .number = 29, .name = "PORTA_29" },
	{ .number = 30, .name = "PORTA_30" },
	{ .number = 31, .name = "PORTA_31" },
	{ .number = 32, .name = "PORTB_0" },
	{ .number = 33, .name = "PORTB_1" },
	{ .number = 34, .name = "PORTB_2" },
	{ .number = 35, .name = "PORTB_3" },
	{ .number = 36, .name = "PORTB_4" },
	{ .number = 37, .name = "PORTB_5" },
	{ .number = 38, .name = "PORTB_6" },
	{ .number = 39, .name = "PORTB_7" },
	{ .number = 40, .name = "PORTB_8" },
	{ .number = 41, .name = "PORTB_9" },
	{ .number = 42, .name = "PORTB_10" },
	{ .number = 43, .name = "PORTB_11" },
	{ .number = 44, .name = "PORTB_12" },
	{ .number = 45, .name = "PORTB_13" },
	{ .number = 46, .name = "PORTB_14" },
	{ .number = 47, .name = "PORTB_15" },
	{ .number = 48, .name = "PORTB_16" },
	{ .number = 49, .name = "PORTB_17" },
	{ .number = 50, .name = "PORTB_18" },
	{ .number = 51, .name = "PORTB_19" },
	{ .number = 52, .name = "PORTB_20" },
	{ .number = 53, .name = "PORTB_21" },
	{ .number = 54, .name = "PORTB_22" },
	{ .number = 55, .name = "PORTB_23" },
	{ .number = 56, .name = "PORTB_24" },
	{ .number = 57, .name = "PORTB_25" },
	{ .number = 58, .name = "PORTB_26" },
	{ .number = 59, .name = "PORTB_27" },
	{ .number = 60, .name = "PORTB_28" },
	{ .number = 61, .name = "PORTB_29" },
	{ .number = 62, .name = "PORTB_30" },
	{ .number = 63, .name = "PORTB_31" },
	{ .number = 64, .name = "PORTC_0" },
	{ .number = 65, .name = "PORTC_1" },
	{ .number = 66, .name = "PORTC_2" },
	{ .number = 67, .name = "PORTC_3" },
	{ .number = 68, .name = "PORTC_4" },
	{ .number = 69, .name = "PORTC_5" },
	{ .number = 70, .name = "PORTC_6" },
	{ .number = 71, .name = "PORTC_7" },
	{ .number = 72, .name = "PORTC_8" },
	{ .number = 73, .name = "PORTC_9" },
	{ .number = 74, .name = "PORTC_10" },
	{ .number = 75, .name = "PORTC_11" },
	{ .number = 76, .name = "PORTC_12" },
	{ .number = 77, .name = "PORTC_13" },
	{ .number = 78, .name = "PORTC_14" },
	{ .number = 79, .name = "PORTC_15" },
	{ .number = 80, .name = "PORTC_16" },
	{ .number = 81, .name = "PORTC_17" },
	{ .number = 82, .name = "PORTC_18" },
	{ .number = 83, .name = "PORTC_19" },
	{ .number = 84, .name = "PORTC_20" },
	{ .number = 85, .name = "PORTC_21" },
	{ .number = 86, .name = "PORTC_22" },
	{ .number = 87, .name = "PORTC_23" },
	{ .number = 88, .name = "PORTC_24" },
	{ .number = 89, .name = "PORTC_25" },
	{ .number = 90, .name = "PORTC_26" },
	{ .number = 91, .name = "PORTC_27" },
	{ .number = 92, .name = "PORTC_28" },
	{ .number = 93, .name = "PORTC_29" },
	{ .number = 94, .name = "PORTC_30" },
	{ .number = 95, .name = "PORTC_31" },
	{ .number = 96, .name = "PORTD_0" },
	{ .number = 97, .name = "PORTD_1" },
	{ .number = 98, .name = "PORTD_2" },
	{ .number = 99, .name = "PORTD_3" },
	{ .number = 100, .name = "PORTD_4" },
	{ .number = 101, .name = "PORTD_5" },
	{ .number = 102, .name = "PORTD_6" },
	{ .number = 103, .name = "PORTD_7" },
	{ .number = 104, .name = "PORTD_8" },
	{ .number = 105, .name = "PORTD_9" },
	{ .number = 106, .name = "PORTD_10" },
	{ .number = 107, .name = "PORTD_11" },
	{ .number = 108, .name = "PORTD_12" },
	{ .number = 109, .name = "PORTD_13" },
	{ .number = 110, .name = "PORTD_14" },
	{ .number = 111, .name = "PORTD_15" },
	{ .number = 112, .name = "PORTD_16" },
	{ .number = 113, .name = "PORTD_17" },
	{ .number = 114, .name = "PORTD_18" },
	{ .number = 115, .name = "PORTD_19" },
	{ .number = 116, .name = "PORTD_20" },
	{ .number = 117, .name = "PORTD_21" },
	{ .number = 118, .name = "PORTD_22" },
	{ .number = 119, .name = "PORTD_23" },
	{ .number = 120, .name = "PORTD_24" },
	{ .number = 121, .name = "PORTD_25" },
	{ .number = 122, .name = "PORTD_26" },
	{ .number = 123, .name = "PORTD_27" },
	{ .number = 124, .name = "PORTD_28" },
	{ .number = 125, .name = "PORTD_29" },
	{ .number = 126, .name = "PORTD_30" },
	{ .number = 127, .name = "PORTD_31" },
	{ .number = 128, .name = "PORTE_0" },
	{ .number = 129, .name = "PORTE_1" },
	{ .number = 130, .name = "PORTE_2" },
	{ .number = 131, .name = "PORTE_3" },
	{ .number = 132, .name = "PORTE_4" },
	{ .number = 133, .name = "PORTE_5" },
	{ .number = 134, .name = "PORTE_6" },
	{ .number = 135, .name = "PORTE_7" },
	{ .number = 136, .name = "PORTE_8" },
	{ .number = 137, .name = "PORTE_9" },
	{ .number = 138, .name = "PORTE_10" },
	{ .number = 139, .name = "PORTE_11" },
	{ .number = 140, .name = "PORTE_12" },
	{ .number = 141, .name = "PORTE_13" },
	{ .number = 142, .name = "PORTE_14" },
	{ .number = 143, .name = "PORTE_15" },
	{ .number = 144, .name = "PORTE_16" },
	{ .number = 145, .name = "PORTE_17" },
	{ .number = 146, .name = "PORTE_18" },
	{ .number = 147, .name = "PORTE_19" },
	{ .number = 148, .name = "PORTE_20" },
	{ .number = 149, .name = "PORTE_21" },
	{ .number = 150, .name = "PORTE_22" },
	{ .number = 151, .name = "PORTE_23" },
	{ .number = 152, .name = "PORTE_24" },
	{ .number = 153, .name = "PORTE_25" },
	{ .number = 154, .name = "PORTE_26" },
	{ .number = 155, .name = "PORTE_27" },
	{ .number = 156, .name = "PORTE_28" },
	{ .number = 157, .name = "PORTE_29" },
	{ .number = 158, .name = "PORTE_30" },
	{ .number = 159, .name = "PORTE_31" },
	{ .number = 160, .name = "PORTF_0" },
	{ .number = 161, .name = "PORTF_1" },
	{ .number = 162, .name = "PORTF_2" },
	{ .number = 163, .name = "PORTF_3" },
	{ .number = 164, .name = "PORTF_4" },
	{ .number = 165, .name = "PORTF_5" },
	{ .number = 166, .name = "PORTF_6" },
	{ .number = 167, .name = "PORTF_7" },
	{ .number = 168, .name = "PORTF_8" },
	{ .number = 169, .name = "PORTF_9" },
	{ .number = 170, .name = "PORTF_10" },
	{ .number = 171, .name = "PORTF_11" },
	{ .number = 172, .name = "PORTF_12" },
	{ .number = 173, .name = "PORTF_13" },
	{ .number = 174, .name = "PORTF_14" },
	{ .number = 175, .name = "PORTF_15" },
	{ .number = 176, .name = "PORTF_16" },
	{ .number = 177, .name = "PORTF_17" },
	{ .number = 178, .name = "PORTF_18" },
	{ .number = 179, .name = "PORTF_19" },
	{ .number = 180, .name = "PORTF_20" },
	{ .number = 181, .name = "PORTF_21" },
	{ .number = 182, .name = "PORTF_22" },
	{ .number = 183, .name = "PORTF_23" },
	{ .number = 184, .name = "PORTF_24" },
	{ .number = 185, .name = "PORTF_25" },
	{ .number = 186, .name = "PORTF_26" },
	{ .number = 187, .name = "PORTF_27" },
	{ .number = 188, .name = "PORTF_28" },
	{ .number = 189, .name = "PORTF_29" },
	{ .number = 190, .name = "PORTF_30" },
	{ .number = 191, .name = "PORTF_31" },
};

static struct pinctrl_desc kinetis_pinctrl_desc = {
	.pctlops = &kinetis_pctrl_ops,
	.pmxops = &kinetis_pmx_ops,
	.pins = kinetis_pinctrl_pads,
	.npins = 192,
	.owner = THIS_MODULE,
};

#define KINETIS_PIN_SIZE 8

static int kinetis_pinctrl_parse_groups(struct device_node *np,
				    struct kinetis_pin_group *grp,
				    struct kinetis_pinctrl_soc_info *info,
				    u32 index)
{
	int size;
	const int pin_size = KINETIS_PIN_SIZE;
	const __be32 *list;
	int i;

	dev_dbg(info->dev, "group(%d): %s\n", index, np->name);

	/* Initialise group */
	grp->name = np->name;

	list = of_get_property(np, "fsl,pins", &size);
	if (!list) {
		dev_err(info->dev, "no fsl,pins property in node %s\n",
					np->full_name);
		return -EINVAL;
	}

	/* we do not check return since it's safe node passed down */
	if (!size || size % pin_size) {
		dev_err(info->dev, "Invalid fsl,pins property in node %s\n",
					np->full_name);
		return -EINVAL;
	}

	grp->npins = size / pin_size;
	grp->pins = devm_kzalloc(info->dev,
				grp->npins * sizeof(struct kinetis_pin),
				GFP_KERNEL);
	if (!grp->pins)
		return -ENOMEM;

	for (i = 0; i < grp->npins; i++) {
		grp->pins[i].mux_reg = be32_to_cpu(*list++);
		grp->pins[i].mux_mode = be32_to_cpu(*list++);
	}

	return 0;
}

static int kinetis_pinctrl_parse_functions(struct device_node *np,
				       struct kinetis_pinctrl_soc_info *info,
				       u32 index)
{
	struct device_node *child;
	struct kinetis_pmx_func *func;
	struct kinetis_pin_group *grp;
	static u32 grp_index = 0;
	u32 i = 0;

	dev_dbg(info->dev, "parse function(%d): %s\n", index, np->name);

	func = &info->functions[index];

	/* Initialise function */
	func->name = np->name;
	func->num_groups = of_get_child_count(np);
	if (func->num_groups == 0) {
		dev_err(info->dev, "no groups defined in %s\n", np->full_name);
		return -EINVAL;
	}
	func->groups = devm_kzalloc(info->dev,
			func->num_groups * sizeof(char *), GFP_KERNEL);

	for_each_child_of_node(np, child) {
		func->groups[i] = child->name;
		grp = &info->groups[grp_index++];
		kinetis_pinctrl_parse_groups(child, grp, info, i++);
	}

	return 0;
}

/*
 * Check if the DT contains pins in the direct child nodes. This indicates the
 * newer DT format to store pins. This function returns true if the first found
 * fsl,pins property is in a child of np. Otherwise false is returned.
 */
static bool kinetis_pinctrl_dt_is_flat_functions(struct device_node *np)
{
	struct device_node *function_np;
	struct device_node *pinctrl_np;

	for_each_child_of_node(np, function_np) {
		if (of_property_read_bool(function_np, "fsl,pins"))
			return true;

		for_each_child_of_node(function_np, pinctrl_np) {
			if (of_property_read_bool(pinctrl_np, "fsl,pins"))
				return false;
		}
	}

	return true;
}

static int kinetis_pinctrl_probe_dt(struct platform_device *pdev,
				struct kinetis_pinctrl_soc_info *info)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	u32 nfuncs = 0;
	u32 i = 0;
	bool flat_funcs;

	if (!np)
		return -ENODEV;

	flat_funcs = kinetis_pinctrl_dt_is_flat_functions(np);
	if (flat_funcs) {
		nfuncs = 1;
	} else {
		nfuncs = of_get_child_count(np);
		if (nfuncs <= 0) {
			dev_err(&pdev->dev, "no functions defined\n");
			return -EINVAL;
		}
	}

	info->nfunctions = nfuncs;
	info->functions = devm_kzalloc(&pdev->dev,
				nfuncs * sizeof(struct kinetis_pmx_func),
				GFP_KERNEL);
	if (!info->functions)
		return -ENOMEM;

	if (flat_funcs) {
		info->ngroups = of_get_child_count(np);
	} else {
		info->ngroups = 0;
		for_each_child_of_node(np, child)
			info->ngroups += of_get_child_count(child);
	}
	if (!(info->ngroups)) {
		dev_err(&pdev->dev, "no groups found\n");
		return -EINVAL;
	}
	info->groups = devm_kzalloc(&pdev->dev,
			info->ngroups * sizeof(struct kinetis_pin_group),
			GFP_KERNEL);
	if (!info->groups)
		return -ENOMEM;

	if (flat_funcs) {
		kinetis_pinctrl_parse_functions(np, info, 0);
	} else {
		for_each_child_of_node(np, child)
			kinetis_pinctrl_parse_functions(child, info, i++);
	}

	return 0;
}

static int kinetis_pinctrl_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct kinetis_pinctrl *kpctl;
	struct resource *res;
	int ret, cur_clk;

	kpctl = devm_kzalloc(&pdev->dev, sizeof(struct kinetis_pinctrl),
					GFP_KERNEL);
	if (!kpctl)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	kpctl->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(kpctl->base))
		return PTR_ERR(kpctl->base);

	for (cur_clk = 0; cur_clk < MAX_PORT_NUM; cur_clk++) {
		kpctl->clk[cur_clk] = of_clk_get(np, cur_clk);
		if (IS_ERR(kpctl->clk[cur_clk])) {
			dev_err(&pdev->dev, "failed to get clock for PORT_%c\n",
					'A' + cur_clk);
			goto err_clk_enable;
		}

		ret = clk_prepare_enable(kpctl->clk[cur_clk]);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable clock for PORT_%c\n",
					'A' + cur_clk);
			clk_put(kpctl->clk[cur_clk]);
			goto err_clk_enable;
		}
	}

	kpctl->info.dev = &pdev->dev;
	ret = kinetis_pinctrl_probe_dt(pdev, &kpctl->info);
	if (ret) {
		dev_err(&pdev->dev, "failed to probe dt properties\n");
		goto err_pinctrl_register;
	}

	kpctl->dev = &pdev->dev;
	kinetis_pinctrl_desc.name = dev_name(&pdev->dev);

	platform_set_drvdata(pdev, kpctl);
	kpctl->pctl = pinctrl_register(&kinetis_pinctrl_desc,
					&pdev->dev, kpctl);
	if (IS_ERR(kpctl->pctl)) {
		dev_err(&pdev->dev, "could not register Kinetis I/O mux\n");
		ret = PTR_ERR(kpctl->pctl);
		goto err_pinctrl_register;
	}

	dev_info(&pdev->dev, "initialized Kinetis I/O mux\n");

	kpdev = kpctl;

	return 0;

err_pinctrl_register:

err_clk_enable:

	while (--cur_clk) {
		clk_disable_unprepare(kpctl->clk[cur_clk]);
		clk_put(kpctl->clk[cur_clk]);
	}

	return ret;
}

static int kinetis_pinctrl_remove(struct platform_device *pdev)
{
	struct kinetis_pinctrl *kpctl = platform_get_drvdata(pdev);
	int cur_clk;

	pinctrl_unregister(kpctl->pctl);

	for (cur_clk = 0; cur_clk < MAX_PORT_NUM; cur_clk++) {
		clk_disable_unprepare(kpctl->clk[cur_clk]);
		clk_put(kpctl->clk[cur_clk]);
	}

	kpdev = NULL;

	return 0;
}

static struct platform_driver kinetis_pinctrl_driver = {
	.driver = {
		.name = "kinetis-pinctrl",
		.of_match_table = kinetis_pinctrl_of_match,
	},
	.probe = kinetis_pinctrl_probe,
	.remove = kinetis_pinctrl_remove,
};

static int __init kinetis_pinctrl_init(void)
{
	return platform_driver_register(&kinetis_pinctrl_driver);
}
arch_initcall(kinetis_pinctrl_init);

static void __exit kinetis_pinctrl_exit(void)
{
	platform_driver_unregister(&kinetis_pinctrl_driver);
}
module_exit(kinetis_pinctrl_exit);

MODULE_DESCRIPTION("Freescale Kinetis pinctrl driver");
MODULE_LICENSE("GPL v2");

int kinetis_pinctrl_get_isfr(int port)
{
	if (!kpdev || (port < 0 || port > 5))
		return 0;


	return KINETIS_PORT_RD(kpdev->base + 0x1000 * port, isfr);
}

void kinetis_pinctrl_set_isfr(int port, int mask)
{
	if (!kpdev || (port < 0 || port > 5))
		return;

	KINETIS_PORT_WR(kpdev->base + 0x1000 * port, isfr, mask);
}

void kinetis_pinctrl_set_irqc(int port, int pin, int mask)
{
	int val;

	if (!kpdev)
		return;

	if (port < 0 || port > 5)
		return;

	if (pin < 0 || pin > 31)
		return;

	val = KINETIS_PORT_RD(kpdev->base + 0x1000 * port, pcr[pin]);
	KINETIS_PORT_WR(kpdev->base + 0x1000 * port, pcr[pin], val | mask);
}
