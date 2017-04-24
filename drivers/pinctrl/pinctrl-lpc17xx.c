/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>
#include <dt-bindings/pinctrl/lpc17xx.h>

#include "core.h"
#include "pinctrl-utils.h"

enum iocon_pin_type {
	IOCON_PIN_TYPE__D = 0,
	IOCON_PIN_TYPE__A,
	IOCON_PIN_TYPE__U,
	IOCON_PIN_TYPE__I,
	IOCON_PIN_TYPE__W,
	IOCON_PIN_TYPE__END
};

static const char *iocon_pin_type_names[IOCON_PIN_TYPE__END] = {
	"D (digital)",
	"A (analog)",
	"U (usb)",
	"I (i2c)",
	"W (digital)",
};

/* Type A (analog): pins that include an analog function */
static int iocon_pins_type_a[] = {
	IOCON_PIN(0, 12),
	IOCON_PIN(0, 13),
	IOCON_PIN(0, 23),
	IOCON_PIN(0, 24),
	IOCON_PIN(0, 25),
	IOCON_PIN(0, 26),
	IOCON_PIN(1, 30),
	IOCON_PIN(1, 31),
};

/* Type U (usb): pins that include a USB D+ or D- function */
static int iocon_pins_type_u[] = {
	IOCON_PIN(0, 29),
	IOCON_PIN(0, 30),
	IOCON_PIN(0, 31),
};

/* Type I (i2c): pins that include a specialized I2C function */
static int iocon_pins_type_i[] = {
	IOCON_PIN(0, 27),
	IOCON_PIN(0, 28),
	IOCON_PIN(5, 2),
	IOCON_PIN(5, 3),
};

/*
 * Type W: pins are otherwise the same as Type D,
 * but include a selectable input glitch filter,
 * and default to pull-down/pull-up disabled
 */
static int iocon_pins_type_w[] = {
	IOCON_PIN(0, 7),
	IOCON_PIN(0, 8),
	IOCON_PIN(0, 9),
};

static struct pinctrl_gpio_range lpc17xx_pinctrl_gpio_range = {
	.npins = IOCON_NR_PORTS * IOCON_PINS_PER_PORT,
};

static enum iocon_pin_type iocon_pins_type_table[IOCON_NR_PORTS * IOCON_PINS_PER_PORT];

/* The rest are pins of Type D (digital) */

struct lpc17xx_pin {
	int			idx;
	unsigned long		config;
	u32			mux;
	enum iocon_pin_type	type;
};

struct lpc17xx_group {
	const char		*name;
	unsigned		npins;
	unsigned int		*pin_ids;
	struct lpc17xx_pin	*pins;
};

struct lpc17xx_func {
	const char		*name;
	const char		**groups;
	unsigned		num_groups;
};

struct lpc17xx_pinctrl {
	struct device		*dev;
	struct pinctrl_dev	*pctl;
	void __iomem		*base;
	unsigned int		npins;
	struct pinctrl_pin_desc	*pins;
	unsigned int		nfunctions;
	struct lpc17xx_func	*functions;
	unsigned int		ngroups;
	struct lpc17xx_group	*groups;
};

static int lpc17xx_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct lpc17xx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	return ipctl->ngroups;
}

static const char *lpc17xx_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						  unsigned group)
{
	struct lpc17xx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	return ipctl->groups[group].name;
}

static int lpc17xx_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					  unsigned group, const unsigned **pins, unsigned *num_pins)
{
	struct lpc17xx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	*pins		= ipctl->groups[group].pin_ids;
	*num_pins	= ipctl->groups[group].npins;
	return 0;
}

static int lpc17xx_pmx_get_funcs_count(struct pinctrl_dev *pctldev)
{
	struct lpc17xx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	return ipctl->nfunctions;
}

static const char *lpc17xx_pmx_get_func_name(struct pinctrl_dev *pctldev,
					     unsigned selector)
{
	struct lpc17xx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	return ipctl->functions[selector].name;
}

static int lpc17xx_pmx_get_groups(struct pinctrl_dev *pctldev, unsigned selector,
				  const char * const **groups,
				  unsigned * const num_groups)
{
	struct lpc17xx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	*groups = ipctl->functions[selector].groups;
	*num_groups = ipctl->functions[selector].num_groups;
	return 0;
}

static int lpc17xx_pmx_set(struct pinctrl_dev *pctldev, unsigned selector,
			   unsigned group)
{
	struct lpc17xx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	struct lpc17xx_group *grp = &ipctl->groups[group];
	unsigned npins = grp->npins;
	int i;

	dev_dbg(ipctl->dev, "enable function %s group %s\n",
		ipctl->functions[selector].name, grp->name);

	for (i = 0; i < npins; i++) {
		struct lpc17xx_pin *pin = &grp->pins[i];
		u32 orig = readl_relaxed(ipctl->base + 4 * pin->idx);
		u32 tmp = orig & ~IOCON_FUNC_MASK;

		/* Don't touch the config */
		tmp |= pin->mux & IOCON_FUNC_MASK;
		dev_dbg(ipctl->dev, "%s: pin %d: 0x%x -> 0x%x\n", __func__, pin->idx, orig, tmp);
		writel_relaxed(tmp, ipctl->base + 4 * pin->idx);
	}

	return 0;
}

static int lpc17xx_pinconf_get(struct pinctrl_dev *pctldev,
			       unsigned pin_id, unsigned long *config)
{
	struct lpc17xx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	*config = readl_relaxed(ipctl->base + 4 * pin_id) & ~IOCON_FUNC_MASK;
	return 0;
}

static int lpc17xx_pinconf_set(struct pinctrl_dev *pctldev,
			       unsigned pin_id, unsigned long *configs,
			       unsigned num_configs)
{
	struct lpc17xx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	u32 orig = readl_relaxed(ipctl->base + 4 * pin_id);
	u32 tmp = orig & IOCON_FUNC_MASK;

	WARN_ON(num_configs != 1);

	/* Don't touch the mux */
	tmp |= configs[0] & ~IOCON_FUNC_MASK;
	dev_dbg(ipctl->dev, "%s: pin %d: 0x%x -> 0x%x\n", __func__, pin_id, orig, tmp);
	writel_relaxed(tmp, ipctl->base + 4 * pin_id);

	return 0;
}

static const inline struct lpc17xx_group *lpc17xx_pinctrl_find_group_by_name(
	struct lpc17xx_pinctrl *ipctl,
	const char *name)
{
	const struct lpc17xx_group *grp = NULL;
	int i;

	for (i = 0; i < ipctl->ngroups; i++) {
		if (!strcmp(ipctl->groups[i].name, name)) {
			grp = &ipctl->groups[i];
			break;
		}
	}

	return grp;
}

static int lpc17xx_dt_node_to_map(struct pinctrl_dev *pctldev,
				  struct device_node *np,
				  struct pinctrl_map **map, unsigned *num_maps)
{
	struct lpc17xx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct lpc17xx_group *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;
	int map_num;
	int i;

	/*
	 * first find the group of this node and check if we need create
	 * config maps for pins
	 */
	grp = lpc17xx_pinctrl_find_group_by_name(ipctl, np->name);
	if (!grp) {
		dev_err(ipctl->dev, "unable to find group for node %s\n",
			np->name);
		return -EINVAL;
	}

	map_num = grp->npins + 1;
	new_map = kmalloc(map_num * sizeof(*new_map), GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	*map = new_map;
	*num_maps = map_num;

	/* create mux map */
	parent = of_get_parent(np);
	if (!parent) {
		kfree(new_map);
		return -EINVAL;
	}
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = parent->name;
	new_map[0].data.mux.group = np->name;
	of_node_put(parent);

	/* create config map */
	new_map++;
	for (i = 0; i < grp->npins; i++) {
		new_map[i].type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map[i].data.configs.group_or_pin = pin_get_name(pctldev, grp->pins[i].idx);
		new_map[i].data.configs.configs = &grp->pins[i].config;
		new_map[i].data.configs.num_configs = 1;
	}

	dev_dbg(pctldev->dev, "maps: function %s group %s num %d\n",
		(*map)->data.mux.function, (*map)->data.mux.group, map_num);

	return 0;
}

static void lpc17xx_dt_free_map(struct pinctrl_dev *pctldev,
				struct pinctrl_map *map, unsigned num_maps)
{
	kfree(map);
}

static int lpc17xx_pmx_gpio_request_enable(struct pinctrl_dev *pctldev,
					   struct pinctrl_gpio_range *range, unsigned offset)
{
	struct lpc17xx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	u32 new_cfg, orig_cfg = readl_relaxed(ipctl->base + 4 * offset);
	bool found_in_dts = false;
	unsigned int group;

	/* Find the pinctrl config with GPIO mux mode for the requested pin */
	for (group = 0; group < ipctl->ngroups; group++) {
		struct lpc17xx_group *grp = &ipctl->groups[group];
		unsigned int pin;

		for (pin = 0; pin < grp->npins; pin++) {
			struct lpc17xx_pin *lpc_pin = &grp->pins[pin];
			if ((lpc_pin->idx == offset) && (lpc_pin->mux == IOCON_FUNC__GPIO)) {
				new_cfg = lpc_pin->mux | lpc_pin->config;
				found_in_dts = true;
				break;
			}
		}

		if (found_in_dts)
			break;
	}

	if (!found_in_dts) {
		dev_err(ipctl->dev, "%s: pin %d is not described in dts, unable to set up\n", __func__, offset);
		return -EINVAL;
	}

	dev_dbg(ipctl->dev, "%s: pin %d: 0x%x -> 0x%x\n", __func__, offset, orig_cfg, new_cfg);
	writel_relaxed(new_cfg, ipctl->base + 4 * offset);

	return 0;
}

static const struct pinctrl_ops lpc17xx_pctl_ops = {
	.get_groups_count	= lpc17xx_pinctrl_get_groups_count,
	.get_group_name		= lpc17xx_pinctrl_get_group_name,
	.get_group_pins		= lpc17xx_pinctrl_get_group_pins,
	.dt_node_to_map		= lpc17xx_dt_node_to_map,
	.dt_free_map		= lpc17xx_dt_free_map,

};

static const struct pinmux_ops lpc17xx_pmx_ops = {
	.get_functions_count	= lpc17xx_pmx_get_funcs_count,
	.get_function_name	= lpc17xx_pmx_get_func_name,
	.get_function_groups	= lpc17xx_pmx_get_groups,
	.set_mux		= lpc17xx_pmx_set,
	.gpio_request_enable	= lpc17xx_pmx_gpio_request_enable,
};

static const struct pinconf_ops lpc17xx_conf_ops = {
	.pin_config_get = lpc17xx_pinconf_get,
	.pin_config_set = lpc17xx_pinconf_set,
};

static struct pinctrl_desc lpc17xx_pinctrl_desc = {
	.pctlops	= &lpc17xx_pctl_ops,
	.pmxops		= &lpc17xx_pmx_ops,
	.confops	= &lpc17xx_conf_ops,
	.owner		= THIS_MODULE,
};

static int lpc17xx_pinctrl_parse_groups(struct device_node *np,
					struct lpc17xx_group *grp,
					struct lpc17xx_pinctrl *ipctl,
					u32 index)
{
	int size, pin_size = IOCON_DTS_ARGUMENTS * sizeof(u32);
	const __be32 *list;
	int i;

	dev_dbg(ipctl->dev, "group(%d): %s\n", index, np->name);

	grp->name = np->name;

	list = of_get_property(np, "nxp,pins", &size);
	if (!list) {
		dev_err(ipctl->dev, "no nxp,pins property in node %s\n", np->full_name);
		return -EINVAL;
	}

	/* we do not check return since it's safe node passed down */
	if (!size || size % pin_size) {
		dev_err(ipctl->dev, "Invalid nxp,pins property in node %s\n", np->full_name);
		return -EINVAL;
	}

	grp->npins = size / pin_size;
	grp->pins = devm_kzalloc(ipctl->dev,
				 grp->npins * sizeof(*grp->pins),
				 GFP_KERNEL);
	grp->pin_ids = devm_kzalloc(ipctl->dev,
				    grp->npins * sizeof(*grp->pin_ids),
				    GFP_KERNEL);
	if (!grp->pins || ! grp->pin_ids)
		return -ENOMEM;

	for (i = 0; i < grp->npins; i++) {
		struct lpc17xx_pin *pin = &grp->pins[i];

		pin->idx	= be32_to_cpu(*list++);
		pin->mux	= be32_to_cpu(*list++);
		pin->config	= be32_to_cpu(*list++);
		pin->type	= iocon_pins_type_table[pin->idx];
		grp->pin_ids[i]	= pin->idx;
		dev_dbg(ipctl->dev, "%s: pin %d.%d (%d): type %s, mux %d, config 0x%x\n",
			np->name,
			pin->idx / IOCON_PINS_PER_PORT,
			pin->idx % IOCON_PINS_PER_PORT,
			pin->idx,
			iocon_pin_type_names[pin->type],
			pin->mux, (unsigned int)pin->config);
	}

	return 0;
}

static int lpc17xx_pinctrl_parse_functions(struct device_node *np,
					   struct lpc17xx_pinctrl *ipctl,
					   u32 index)
{
	struct device_node *child;
	struct lpc17xx_func *func = &ipctl->functions[index];
	struct lpc17xx_group *grp;
	static u32 grp_index;
	u32 i = 0;

	dev_dbg(ipctl->dev, "parse function(%d): %s\n", index, np->name);

	/* Initialise function */
	func->name = np->name;
	func->num_groups = of_get_child_count(np);
	if (func->num_groups == 0) {
		dev_err(ipctl->dev, "no groups defined in %s\n", np->full_name);
		return -EINVAL;
	}
	func->groups = devm_kzalloc(ipctl->dev,
				    func->num_groups * sizeof(*func->groups),
				    GFP_KERNEL);

	for_each_child_of_node(np, child) {
		func->groups[i] = child->name;
		grp = &ipctl->groups[grp_index++];
		lpc17xx_pinctrl_parse_groups(child, grp, ipctl, i++);
	}

	return 0;
}

static int lpc17xx_pinctrl_probe_dt(struct lpc17xx_pinctrl *ipctl)
{
	struct device_node *np = ipctl->dev->of_node;
	if (!np)
		return -ENODEV;

	/* Flat model */
	ipctl->nfunctions = 1;
	ipctl->functions = devm_kzalloc(ipctl->dev,
					ipctl->nfunctions * sizeof(*ipctl->functions),
					GFP_KERNEL);
	if (!ipctl->functions)
		return -ENOMEM;

	ipctl->ngroups = of_get_child_count(np);
	ipctl->groups = devm_kzalloc(ipctl->dev,
				     ipctl->ngroups * sizeof(*ipctl->groups),
				     GFP_KERNEL);
	if (!ipctl->groups)
		return -ENOMEM;

	lpc17xx_pinctrl_parse_functions(np, ipctl, 0);

	return 0;
}

static int lpc17xx_pctl_probe(struct platform_device *pdev)
{
	struct lpc17xx_pinctrl *ipctl;
	struct resource *res;
	int ret;
	int i;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "Unable to find pinctrl node\n");
		return -EINVAL;
	}

	ipctl = devm_kzalloc(&pdev->dev, sizeof(*ipctl), GFP_KERNEL);
	if (!ipctl)
		return -ENOMEM;

	ipctl->dev = &pdev->dev;
	platform_set_drvdata(pdev, ipctl);

	ipctl->npins = IOCON_NR_PORTS * IOCON_PINS_PER_PORT;
	ipctl->pins = devm_kmalloc(ipctl->dev,
				   ipctl->npins * sizeof(*ipctl->pins),
				   GFP_KERNEL);
	if (!ipctl->pins)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ipctl->base = devm_ioremap_resource(ipctl->dev, res);
	if (IS_ERR(ipctl->base))
		return PTR_ERR(ipctl->base);

	lpc17xx_pinctrl_desc.name = dev_name(ipctl->dev);
	lpc17xx_pinctrl_desc.pins = ipctl->pins;
	lpc17xx_pinctrl_desc.npins = ipctl->npins;

	for (i = 0; i < ARRAY_SIZE(iocon_pins_type_a); ++i)
		iocon_pins_type_table[iocon_pins_type_a[i]] = IOCON_PIN_TYPE__A;
	for (i = 0; i < ARRAY_SIZE(iocon_pins_type_u); ++i)
		iocon_pins_type_table[iocon_pins_type_a[i]] = IOCON_PIN_TYPE__U;
	for (i = 0; i < ARRAY_SIZE(iocon_pins_type_i); ++i)
		iocon_pins_type_table[iocon_pins_type_a[i]] = IOCON_PIN_TYPE__I;
	for (i = 0; i < ARRAY_SIZE(iocon_pins_type_w); ++i)
		iocon_pins_type_table[iocon_pins_type_a[i]] = IOCON_PIN_TYPE__W;

	for (i = 0; i < ipctl->npins; ++i) {
		ipctl->pins[i].number = i;
		ipctl->pins[i].name = NULL;
		ipctl->pins[i].drv_data = NULL;
	}

	ret = lpc17xx_pinctrl_probe_dt(ipctl);
	if (ret)
		return ret;

	ipctl->pctl = pinctrl_register(&lpc17xx_pinctrl_desc, ipctl->dev, ipctl);
	if (!ipctl->pctl) {
		dev_err(ipctl->dev, "Failed pinctrl registration\n");
		return -EINVAL;
	}

	pinctrl_add_gpio_range(ipctl->pctl, &lpc17xx_pinctrl_gpio_range);

	dev_info(&pdev->dev, "initialized LPC17xx pinctrl driver\n");
	return 0;
}

static struct of_device_id lpc17xx_pctl_of_match[] = {
	{ .compatible = "nxp,lpc17xx-pinctrl" },
	{ /* sentinel */ }
};

static struct platform_driver lpc17xx_pctl_driver = {
	.driver = {
		.name = "lpc17xx-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = lpc17xx_pctl_of_match,
	},
	.probe = lpc17xx_pctl_probe,
};

static int __init lpc17xx_pctl_init(void)
{
	return platform_driver_register(&lpc17xx_pctl_driver);
}
arch_initcall(lpc17xx_pctl_init);
