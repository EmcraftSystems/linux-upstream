/*
 * Copyright (C) Maxime Coquelin 2015
 * Author:  Maxime Coquelin <mcoquelin.stm32@gmail.com>
 * License terms:  GNU General Public License (GPL), version 2
 *
 * Heavily based on pinctrl-st.c from Srinivas Kandagatla
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <dt-bindings/pinctrl/pinctrl-stm32.h>
#include <mach/gpio.h>
#include "core.h"

#define STM32_GPIO_PINS_PER_BANK 16
#define OF_GPIO_ARGS_MIN 4

#define STM32_PINCONF_UNPACK(conf, param)\
				((conf >> STM32_PINCONF_ ##param ##_SHIFT) \
				& STM32_PINCONF_ ##param ##_MASK)

#define STM32_PINCONF_PACK(conf, val, param)	(conf |=\
				((val & STM32_PINCONF_ ##param ##_MASK) << \
					STM32_PINCONF_ ##param ##_SHIFT))

#define STM32_PINCONF_SPEED_MASK		0x3
#define STM32_PINCONF_SPEED_SHIFT		3
#define STM32_PINCONF_UNPACK_SPEED(conf)\
				STM32_PINCONF_UNPACK(conf, SPEED)
#define STM32_PINCONF_PACK_SPEED(conf, val)\
				STM32_PINCONF_PACK(conf, val, SPEED)

#define STM32_PINCONF_TYPE_MASK			0x1
#define STM32_PINCONF_TYPE_SHIFT		2
#define STM32_PINCONF_UNPACK_TYPE(conf)\
				STM32_PINCONF_UNPACK(conf, TYPE)
#define STM32_PINCONF_PACK_TYPE(conf, val)\
				STM32_PINCONF_PACK(conf, val, TYPE)

#define STM32_PINCONF_PUPD_MASK			0x3
#define STM32_PINCONF_PUPD_SHIFT		0
#define STM32_PINCONF_UNPACK_PUPD(conf)\
				STM32_PINCONF_UNPACK(conf, PUPD)
#define STM32_PINCONF_PACK_PUPD(conf, val)\
				STM32_PINCONF_PACK(conf, val, PUPD)


#define STM32_PINCONF_ALT_MASK			0xf
#define STM32_PINCONF_ALT_SHIFT		2
#define STM32_PINCONF_UNPACK_ALT(conf)\
				STM32_PINCONF_UNPACK(conf, ALT)
#define STM32_PINCONF_PACK_ALT(conf, val)\
				STM32_PINCONF_PACK(conf, val, ALT)

#define STM32_PINCONF_MODE_MASK			0x3
#define STM32_PINCONF_MODE_SHIFT		0
#define STM32_PINCONF_UNPACK_MODE(conf)\
				STM32_PINCONF_UNPACK(conf, MODE)
#define STM32_PINCONF_PACK_MODE(conf, val)\
				STM32_PINCONF_PACK(conf, val, MODE)



#define gpio_range_to_bank(chip) \
		container_of(chip, struct stm32_gpio_bank, range)

#define gpio_chip_to_bank(chip) \
		container_of(chip, struct stm32_gpio_bank, gpio_chip)

struct stm32_pinconf {
	int		pin;
	const char	*name;
	unsigned long	config;
	int		altfunc;
};

struct stm32_pmx_func {
	const char	*name;
	const char	**groups;
	unsigned	ngroups;
};

struct stm32_pctl_group {
	const char		*name;
	unsigned int		*pins;
	unsigned		npins;
	struct stm32_pinconf	*pin_conf;
};

struct stm32_gpio_bank {
	void __iomem *base;
	struct gpio_chip		gpio_chip;
	struct pinctrl_gpio_range	range;
	spinlock_t                      lock;
};

struct stm32_pinctrl {
	struct device		*dev;
	struct pinctrl_dev		*pctl;
	struct stm32_gpio_bank	*banks;
	int			nbanks;
	struct stm32_pmx_func	*functions;
	int			nfunctions;
	struct stm32_pctl_group	*groups;
	int			ngroups;
	struct irq_domain	*domain;
	struct regmap		*regmap;
	struct regmap_field	*irqmux[STM32_GPIO_PINS_PER_BANK];
};

/* Reconfiguration disabled mask, per port */
static u32 stm32_reconf_dis[STM32_GPIO_PORTS];

static inline int stm32_gpio_pin(int gpio)
{
	return gpio % STM32_GPIO_PINS_PER_BANK;
}

/* Pinconf  */
static void stm32_pinconf_set_config(struct stm32_gpio_bank *bank,
				int pin, unsigned long config)
{
	u32 type, speed, pupd, val;
	unsigned long flags;

	type = STM32_PINCONF_UNPACK_TYPE(config);
	spin_lock_irqsave(&bank->lock, flags);
	val = readl_relaxed(bank->base + STM32_GPIO_TYPER);
	val &= ~BIT(pin);
	val |= type << pin;
	writel_relaxed(val, bank->base + STM32_GPIO_TYPER);
	spin_unlock_irqrestore(&bank->lock, flags);

	speed = STM32_PINCONF_UNPACK_SPEED(config);
	spin_lock_irqsave(&bank->lock, flags);
	val = readl_relaxed(bank->base + STM32_GPIO_SPEEDR);
	val &= ~GENMASK(pin * 2 + 1, pin * 2);
	val |= speed << (pin * 2);
	writel_relaxed(val, bank->base + STM32_GPIO_SPEEDR);
	spin_unlock_irqrestore(&bank->lock, flags);

	pupd = STM32_PINCONF_UNPACK_PUPD(config);
	spin_lock_irqsave(&bank->lock, flags);
	val = readl_relaxed(bank->base + STM32_GPIO_PUPDR);
	val &= ~GENMASK(pin * 2 + 1, pin * 2);
	val |= pupd << (pin * 2);
	writel_relaxed(val, bank->base + STM32_GPIO_PUPDR);
	spin_unlock_irqrestore(&bank->lock, flags);
}

static void stm32_pinconf_get_config(struct stm32_gpio_bank *bank,
				int pin, unsigned long *config)
{
	u32 val;

	val = readl_relaxed(bank->base + STM32_GPIO_TYPER);
	val = val >> pin;
	STM32_PINCONF_PACK_TYPE(*config, val);

	val = readl_relaxed(bank->base + STM32_GPIO_SPEEDR);
	val = val >> (pin * 2);
	STM32_PINCONF_PACK_SPEED(*config, val);

	val = readl_relaxed(bank->base + STM32_GPIO_PUPDR);
	val = val >> (pin * 2);
	STM32_PINCONF_PACK_PUPD(*config, val);
}

static int stm32_pinconf_set(struct pinctrl_dev *pctldev, unsigned pin_id,
			unsigned long *configs, unsigned num_configs)
{
	struct pinctrl_gpio_range *range =
			 pinctrl_find_gpio_range_from_pin(pctldev, pin_id);
	struct stm32_gpio_bank *bank = gpio_range_to_bank(range);
	int pin = stm32_gpio_pin(pin_id);
	int i;

	for (i = 0; i < num_configs; i++)
		stm32_pinconf_set_config(bank, pin, configs[i]);

	return 0;
}

static int stm32_pinconf_get(struct pinctrl_dev *pctldev,
			     unsigned pin_id, unsigned long *config)
{
	struct pinctrl_gpio_range *range =
			 pinctrl_find_gpio_range_from_pin(pctldev, pin_id);
	struct stm32_gpio_bank *bank = gpio_range_to_bank(range);
	int pin = stm32_gpio_pin(pin_id);

	*config = 0;
	stm32_pinconf_get_config(bank, pin, config);

	return 0;
}

static void stm32_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				   struct seq_file *s, unsigned pin_id)
{
	unsigned long config;

	stm32_pinconf_get(pctldev, pin_id, &config);

	seq_printf(s, "[PUPD:%ld,TYPE:%ld,SPEED:%ld]\n",
		STM32_PINCONF_UNPACK_PUPD(config),
		STM32_PINCONF_UNPACK_TYPE(config),
		STM32_PINCONF_UNPACK_SPEED(config));
}

static struct pinconf_ops stm32_confops = {
	.pin_config_get		= stm32_pinconf_get,
	.pin_config_set		= stm32_pinconf_set,
	.pin_config_dbg_show	= stm32_pinconf_dbg_show,
};

static void stm32_pctl_set_function(struct stm32_gpio_bank *bank,
		int pin_id, int function)
{
	u32 mode, alt, val;
	int pin = stm32_gpio_pin(pin_id);
	int alt_shift = (pin % 8) * 4;
	int alt_offset = STM32_GPIO_AFRL + (pin / 8) * 4;
	unsigned long flags;

	mode = STM32_PINCONF_UNPACK_MODE(function);
	alt = STM32_PINCONF_UNPACK_ALT(function);

	spin_lock_irqsave(&bank->lock, flags);

	val = readl_relaxed(bank->base + alt_offset);
	val &= ~GENMASK(alt_shift + 3, alt_shift);
	val |= (alt << alt_shift);
	writel_relaxed(val, bank->base + alt_offset);

	val = readl_relaxed(bank->base + STM32_GPIO_MODER);
	val &= ~GENMASK(pin * 2 + 1, pin * 2);
	val |= mode << (pin * 2);
	writel_relaxed(val, bank->base + STM32_GPIO_MODER);

	spin_unlock_irqrestore(&bank->lock, flags);
}

/* Pinmux */
static int stm32_pmx_get_funcs_count(struct pinctrl_dev *pctldev)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	return info->nfunctions;
}

static const char *stm32_pmx_get_fname(struct pinctrl_dev *pctldev,
	unsigned selector)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	return info->functions[selector].name;
}

static int stm32_pmx_get_groups(struct pinctrl_dev *pctldev,
	unsigned selector, const char * const **grps, unsigned * const ngrps)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	*grps = info->functions[selector].groups;
	*ngrps = info->functions[selector].ngroups;

	return 0;
}

static int stm32_pmx_set_mux(struct pinctrl_dev *pctldev, unsigned fselector,
			unsigned group)
{
	struct pinctrl_gpio_range *range;
	struct stm32_gpio_bank *bank;
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	struct stm32_pinconf *conf = info->groups[group].pin_conf;
	int i;

	for (i = 0; i < info->groups[group].npins; i++) {
		range = pinctrl_find_gpio_range_from_pin(pctldev, conf[i].pin);
		bank = gpio_range_to_bank(range);
		stm32_pctl_set_function(bank, conf[i].pin, conf[i].altfunc);
	}

	return 0;
}

static int stm32_pmx_set_gpio_direction(struct pinctrl_dev *pctldev,
			struct pinctrl_gpio_range *range, unsigned gpio,
			bool input)
{
	struct stm32_gpio_bank *bank = gpio_range_to_bank(range);

	stm32_pctl_set_function(bank, gpio, !input);

	return 0;
}

static struct pinmux_ops stm32_pmxops = {
	.get_functions_count	= stm32_pmx_get_funcs_count,
	.get_function_name	= stm32_pmx_get_fname,
	.get_function_groups	= stm32_pmx_get_groups,
	.set_mux		= stm32_pmx_set_mux,
	.gpio_set_direction	= stm32_pmx_set_gpio_direction,
};

/* Pinctrl Groups */
static int stm32_pctl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	return info->ngroups;
}

static const char *stm32_pctl_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned selector)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	return info->groups[selector].name;
}

static int stm32_pctl_get_group_pins(struct pinctrl_dev *pctldev,
	unsigned selector, const unsigned **pins, unsigned *npins)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	if (selector >= info->ngroups)
		return -EINVAL;

	*pins = info->groups[selector].pins;
	*npins = info->groups[selector].npins;

	return 0;
}

static const inline struct stm32_pctl_group *stm32_pctl_find_group_by_name(
	const struct stm32_pinctrl *info, const char *name)
{
	int i;

	for (i = 0; i < info->ngroups; i++) {
		if (!strcmp(info->groups[i].name, name))
			return &info->groups[i];
	}

	return NULL;
}

static int stm32_pctl_dt_node_to_map(struct pinctrl_dev *pctldev,
	struct device_node *np, struct pinctrl_map **map, unsigned *num_maps)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	const struct stm32_pctl_group *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;
	int map_num, i;

	grp = stm32_pctl_find_group_by_name(info, np->name);
	if (!grp) {
		dev_err(info->dev, "unable to find group for node %s\n",
			np->name);
		return -EINVAL;
	}

	map_num = grp->npins + 1;
	new_map = devm_kzalloc(pctldev->dev,
				sizeof(*new_map) * map_num, GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	parent = of_get_parent(np);
	if (!parent) {
		devm_kfree(pctldev->dev, new_map);
		return -EINVAL;
	}

	*map = new_map;
	*num_maps = map_num;
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = parent->name;
	new_map[0].data.mux.group = np->name;
	of_node_put(parent);

	/* create config map per pin */
	new_map++;
	for (i = 0; i < grp->npins; i++) {
		new_map[i].type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map[i].data.configs.group_or_pin =
				pin_get_name(pctldev, grp->pins[i]);
		new_map[i].data.configs.configs = &grp->pin_conf[i].config;
		new_map[i].data.configs.num_configs = 1;
	}
	dev_info(pctldev->dev, "maps: function %s group %s num %d\n",
		(*map)->data.mux.function, grp->name, map_num);

	return 0;
}

static void stm32_pctl_dt_free_map(struct pinctrl_dev *pctldev,
			struct pinctrl_map *map, unsigned num_maps)
{
}

static struct pinctrl_ops stm32_pctlops = {
	.get_groups_count	= stm32_pctl_get_groups_count,
	.get_group_pins		= stm32_pctl_get_group_pins,
	.get_group_name		= stm32_pctl_get_group_name,
	.dt_node_to_map		= stm32_pctl_dt_node_to_map,
	.dt_free_map		= stm32_pctl_dt_free_map,
};

static void stm32_pctl_dt_child_count(struct stm32_pinctrl *info,
				     struct device_node *np)
{
	struct device_node *child;

	for_each_child_of_node(np, child) {
		if (of_property_read_bool(child, "gpio-controller")) {
			info->nbanks++;
		} else {
			info->nfunctions++;
			info->ngroups += of_get_child_count(child);
		}
	}
}

static int stm32_pctl_dt_parse_groups(struct device_node *np,
	struct stm32_pctl_group *grp, struct stm32_pinctrl *info, int idx)
{
	const __be32 *list;
	struct property *pp;
	struct stm32_pinconf *conf;
	struct device_node *pins;
	int i = 0, npins = 0, nr_props;

	pins = of_get_child_by_name(np, "st,pins");
	if (!pins)
		return -ENODATA;

	for_each_property_of_node(pins, pp) {
		/* Skip those we do not want to proceed */
		if (!strcmp(pp->name, "name"))
			continue;

		if (pp  && (pp->length / sizeof(__be32)) >= OF_GPIO_ARGS_MIN) {
			npins++;
		} else {
			pr_warn("Invalid st,pins in %s node\n", np->name);
			return -EINVAL;
		}
	}

	grp->npins = npins;
	grp->name = np->name;
	grp->pins = devm_kzalloc(info->dev, npins * sizeof(u32), GFP_KERNEL);
	grp->pin_conf = devm_kzalloc(info->dev,
					npins * sizeof(*conf), GFP_KERNEL);

	if (!grp->pins || !grp->pin_conf)
		return -ENOMEM;

	/* <bank offset mux pull type speed> */
	for_each_property_of_node(pins, pp) {
		if (!strcmp(pp->name, "name"))
			continue;
		nr_props = pp->length / sizeof(u32);
		list = pp->value;
		conf = &grp->pin_conf[i];

		/* bank & offset */
		be32_to_cpup(list++);
		be32_to_cpup(list++);
		conf->pin = of_get_named_gpio(pins, pp->name, 0);
		conf->name = pp->name;
		grp->pins[i] = conf->pin;
		/* mux */
		conf->altfunc = be32_to_cpup(list++);
		conf->config = 0;
		/* pull-up/down */
		conf->config |= be32_to_cpup(list++);
		if (nr_props > OF_GPIO_ARGS_MIN) {
			/* push-pull/open-drain */
			conf->config |= be32_to_cpup(list++);
			/* speed */
			conf->config |= be32_to_cpup(list++);
		}
		i++;
	}
	of_node_put(pins);

	return 0;
}

static int stm32_pctl_parse_functions(struct device_node *np,
			struct stm32_pinctrl *info, u32 index, int *grp_index)
{
	struct device_node *child;
	struct stm32_pmx_func *func;
	struct stm32_pctl_group *grp;
	int i = 0, ret;

	func = &info->functions[index];
	func->name = np->name;
	func->ngroups = of_get_child_count(np);
	if (func->ngroups == 0) {
		dev_err(info->dev, "No groups defined\n");
		return -EINVAL;
	}
	func->groups = devm_kzalloc(info->dev,
			func->ngroups * sizeof(char *), GFP_KERNEL);
	if (!func->groups)
		return -ENOMEM;

	for_each_child_of_node(np, child) {
		func->groups[i] = child->name;
		grp = &info->groups[*grp_index];
		*grp_index += 1;
		ret = stm32_pctl_dt_parse_groups(child, grp, info, i++);
		if (ret)
			return ret;
	}
	dev_info(info->dev, "Function[%d\t name:%s,\tgroups:%d]\n",
				index, func->name, func->ngroups);

	return 0;
}

static inline void __stm32_gpio_set(struct stm32_gpio_bank *bank,
	unsigned offset, int value)
{
	if (!value)
		offset += STM32_GPIO_PINS_PER_BANK;

	writel_relaxed(BIT(offset), bank->base + STM32_GPIO_BSRR);
}

static int stm32_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_request_gpio(chip->base + offset);
}

static void stm32_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_free_gpio(chip->base + offset);
}

static int stm32_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct stm32_gpio_bank *bank = gpio_chip_to_bank(chip);

	return !!(readl_relaxed(bank->base + STM32_GPIO_IDR) & BIT(offset));
}

static void stm32_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct stm32_gpio_bank *bank = gpio_chip_to_bank(chip);

	__stm32_gpio_set(bank, offset, value);
}

static int stm32_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_gpio_direction_input(chip->base + offset);

	return 0;
}

static int stm32_gpio_direction_output(struct gpio_chip *chip,
	unsigned offset, int value)
{
	struct stm32_gpio_bank *bank = gpio_chip_to_bank(chip);

	__stm32_gpio_set(bank, offset, value);
	pinctrl_gpio_direction_output(chip->base + offset);

	return 0;
}

static int stm32_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct stm32_pinctrl *pctl = dev_get_drvdata(chip->dev);
	struct stm32_gpio_bank *bank = gpio_chip_to_bank(chip);
	unsigned int irq;

	regmap_field_write(pctl->irqmux[offset], bank->range.id);

	pr_info("%s: Event line %d now points to fire interrupts from bank %c\n", __func__, offset, 'A' + bank->range.id);

	irq = irq_create_mapping(pctl->domain, offset);
	if (!irq) {
		printk("%s: irq_create_mapping failed\n", __func__);
		return -ENXIO;
	}

	return irq;
}

static struct gpio_chip stm32_gpio_template = {
	.request		= stm32_gpio_request,
	.free			= stm32_gpio_free,
	.get			= stm32_gpio_get,
	.set			= stm32_gpio_set,
	.direction_input	= stm32_gpio_direction_input,
	.direction_output	= stm32_gpio_direction_output,
	.ngpio			= STM32_GPIO_PINS_PER_BANK,
	.to_irq			= stm32_gpio_to_irq,
};

static int stm32_gpiolib_register_bank(struct stm32_pinctrl *info,
	int bank_nr, struct device_node *np)
{
	struct stm32_gpio_bank *bank = &info->banks[bank_nr];
	struct pinctrl_gpio_range *range = &bank->range;
	struct device *dev = info->dev;
	struct resource res;
	struct reset_control *rstc;
	struct clk *clk;
	int bank_num = of_alias_get_id(np, "gpio");
	int err, i;
	u32 val;

	rstc = of_reset_control_get(np, NULL);
	if (!IS_ERR(rstc))
		reset_control_deassert(rstc);

	if (of_address_to_resource(np, 0, &res))
		return -ENODEV;

	bank->base = devm_ioremap_resource(dev, &res);
	if (IS_ERR(bank->base))
		return PTR_ERR(bank->base);

	/*
	 * Reconfigure INPUT (reset value) pins to ANALOG
	 */
	val = readl_relaxed(bank->base + STM32_GPIO_MODER);
	for (i = 0; i < STM32_GPIO_PINS_PER_BANK; i++) {
		if (stm32_reconf_dis[bank_num] & BIT(i))
			continue;
		if (val & GENMASK(i * 2 + 1, i * 2))
			continue;
		val |= GENMASK(i * 2 + 1, i * 2);
	}
	writel_relaxed(val, bank->base + STM32_GPIO_MODER);

	bank->gpio_chip = stm32_gpio_template;
	bank->gpio_chip.base = bank_num * STM32_GPIO_PINS_PER_BANK;
	bank->gpio_chip.of_node = np;
	bank->gpio_chip.dev = dev;
	spin_lock_init(&bank->lock);

	of_property_read_string(np, "st,bank-name", &range->name);
	bank->gpio_chip.label = range->name;

	clk = of_clk_get(np, 0);
	if (!IS_ERR(clk))
		clk_prepare_enable(clk);

	range->id = bank_num;
	range->pin_base = range->base = range->id * STM32_GPIO_PINS_PER_BANK;
	range->npins = bank->gpio_chip.ngpio;
	range->gc = &bank->gpio_chip;
	err  = gpiochip_add(&bank->gpio_chip);
	if (err) {
		dev_err(dev, "Failed to add gpiochip(%d)!\n", bank_num);
		return err;
	}
	dev_info(dev, "%s bank added.\n", range->name);

	return 0;
}

/*
 * This function fills the <moder> array with the modified values of MODERx
 * registers: all <AF> settings are replaced with <ANALOG>. The resulted
 * values (from <moder>) are then programmed to the MODERx registers by the
 * power-management code (executed from internal RAM, after placing external
 * RAM to the self-refresh mode), and allow to achieve the minimal power
 * consumption.
 */
int stm32_pctrl_alt_to_analog(struct platform_device *pdev, u32 *moder, u32 len)
{
	struct stm32_pinctrl *info = platform_get_drvdata(pdev);
	struct stm32_gpio_bank *bank;
	u32 val;
	int i, k;

	if (len != info->nbanks)
		return -EINVAL;

	for (i = 0; i < info->nbanks; i++) {
		bank = &info->banks[i];
		val = readl_relaxed(bank->base + STM32_GPIO_MODER);
		for (k = 0; k < STM32_GPIO_PINS_PER_BANK; k++) {
			if ((val & GENMASK(k * 2 + 1, k * 2)) !=
			    (ALT << (k * 2)))
				continue;
			val |= GENMASK(k * 2 + 1, k * 2);
		}
		moder[i] = val;
	}

	return 0;
}
EXPORT_SYMBOL(stm32_pctrl_alt_to_analog);

/*
 * Disable reconfiguting the specified pins during pinctrl initialization
 */
int stm32_pctrl_reconf_disable(int bank, u32 mask)
{
	if (bank >= ARRAY_SIZE(stm32_reconf_dis))
		return -EINVAL;

	stm32_reconf_dis[bank] |= mask;

	return 0;
}
EXPORT_SYMBOL(stm32_pctrl_reconf_disable);

static int stm32_pctrl_dt_setup_irq(struct platform_device *pdev,
			   struct stm32_pinctrl *pctl)
{
	struct device_node *np = pdev->dev.of_node, *parent;
	struct device *dev = &pdev->dev;
	struct regmap *rm;
	int offset, ret, i;

	parent = of_irq_find_parent(np);
	if (!parent)
		return -ENXIO;

	pctl->domain = irq_find_host(parent);
	if (!pctl->domain)
		return -ENXIO;

	pctl->regmap = syscon_regmap_lookup_by_phandle(np, "st,syscfg");
	if (IS_ERR(pctl->regmap)) {
		return PTR_ERR(pctl->regmap);
	} else {
		const u32 *p = of_get_property(np, "st,syscfg", NULL);
		struct device_node *syscfg_np = p ? of_find_node_by_phandle(be32_to_cpup(p)) : NULL;
		struct platform_device *pdev = syscfg_np ? of_find_device_by_node(syscfg_np) : NULL;
		struct clk *syscfg_clk = pdev ? devm_clk_get(&pdev->dev, "core_clk") : NULL;
		if (!p || !syscfg_np || !pdev || !syscfg_clk) {
			dev_err(dev, "%s: unable to get syscfg clock\n", __func__);
			return -ENXIO;
		}
		if (IS_ERR(syscfg_clk)) {
			int err = PTR_ERR(syscfg_clk);
			dev_err(dev, "%s: unable to get syscfg clock: %d\n", __func__, err);
			return err;
		}

		clk_prepare_enable(syscfg_clk);
	}

	rm = pctl->regmap;

	ret = of_property_read_u32_index(np, "st,syscfg", 1, &offset);
	if (ret)
		return ret;

	for (i = 0; i < STM32_GPIO_PINS_PER_BANK; i++) {
		struct reg_field mux;

		mux.reg = offset + (i / 4) * 4;
		mux.lsb = (i % 4) * 4;
		mux.msb = mux.lsb + 3;

		pctl->irqmux[i] = devm_regmap_field_alloc(dev, rm, mux);
		if (IS_ERR(pctl->irqmux[i]))
			return PTR_ERR(pctl->irqmux[i]);
	}

	return 0;
}

static int stm32_pctl_probe_dt(struct platform_device *pdev,
	struct pinctrl_desc *pctl_desc, struct stm32_pinctrl *info)
{
	struct device_node *np = pdev->dev.of_node;
	struct pinctrl_pin_desc *pdesc;
	struct device_node *child;
	int grp_index = 0;
	int i = 0, j = 0, k = 0, bank = 0,  ret = 0;

	stm32_pctl_dt_child_count(info, np);
	if (!info->nbanks) {
		dev_err(&pdev->dev, "you need atleast one gpio bank\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev, "nbanks = %d\n", info->nbanks);
	dev_info(&pdev->dev, "nfunctions = %d\n", info->nfunctions);
	dev_info(&pdev->dev, "ngroups = %d\n", info->ngroups);

	info->functions = devm_kzalloc(&pdev->dev,
		info->nfunctions * sizeof(*info->functions), GFP_KERNEL);

	info->groups = devm_kzalloc(&pdev->dev,
			info->ngroups * sizeof(*info->groups), GFP_KERNEL);

	info->banks = devm_kzalloc(&pdev->dev,
			info->nbanks * sizeof(*info->banks), GFP_KERNEL);

	if (!info->functions || !info->groups || !info->banks)
		return -ENOMEM;

	pctl_desc->npins = info->nbanks * STM32_GPIO_PINS_PER_BANK;
	pdesc =	devm_kzalloc(&pdev->dev,
			sizeof(*pdesc) * pctl_desc->npins, GFP_KERNEL);
	if (!pdesc)
		return -ENOMEM;

	pctl_desc->pins = pdesc;

	for_each_child_of_node(np, child) {
		if (of_property_read_bool(child, "gpio-controller")) {
			const char *bank_name;

			ret = stm32_gpiolib_register_bank(info, bank, child);
			if (ret)
				return ret;

			k = info->banks[bank].range.pin_base;
			bank_name = info->banks[bank].range.name;
			for (j = 0; j < STM32_GPIO_PINS_PER_BANK; j++, k++) {
				pdesc->number = k;
				pdesc->name = kasprintf(GFP_KERNEL, "%s[%d]",
							bank_name, j);
				pdesc++;
			}
			bank++;
		} else {
			ret = stm32_pctl_parse_functions(child, info,
							i++, &grp_index);
			if (ret) {
				dev_err(&pdev->dev, "No functions found.\n");
				return ret;
			}
		}
	}

	return 0;
}

static int stm32_pctl_probe(struct platform_device *pdev)
{
	struct stm32_pinctrl *info;
	struct pinctrl_desc *pctl_desc;
	int ret, i;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "device not found.\n");
		return -EINVAL;
	}

	pctl_desc = devm_kzalloc(&pdev->dev, sizeof(*pctl_desc), GFP_KERNEL);
	if (!pctl_desc)
		return -ENOMEM;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);
	ret = stm32_pctl_probe_dt(pdev, pctl_desc, info);
	if (ret)
		return ret;

	ret = stm32_pctrl_dt_setup_irq(pdev, info);
	if (ret)
		return ret;

	pctl_desc->owner	= THIS_MODULE;
	pctl_desc->pctlops	= &stm32_pctlops;
	pctl_desc->pmxops	= &stm32_pmxops;
	pctl_desc->confops	= &stm32_confops;
	pctl_desc->name		= dev_name(&pdev->dev);

	info->pctl = pinctrl_register(pctl_desc, &pdev->dev, info);
	if (!info->pctl) {
		dev_err(&pdev->dev, "Failed pinctrl registration\n");
		return -EINVAL;
	}

	for (i = 0; i < info->nbanks; i++)
		pinctrl_add_gpio_range(info->pctl, &info->banks[i].range);

	return 0;
}

static struct of_device_id stm32_pctl_of_match[] = {
	{ .compatible = "st,stm32-pinctrl" },
	{ /* sentinel */ }
};

static struct platform_driver stm32_pctl_driver = {
	.driver = {
		.name = "stm32-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = stm32_pctl_of_match,
	},
	.probe = stm32_pctl_probe,
};

static int __init stm32_pctl_init(void)
{
	return platform_driver_register(&stm32_pctl_driver);
}
arch_initcall(stm32_pctl_init);
