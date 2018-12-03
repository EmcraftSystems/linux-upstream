// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 ROHM Semiconductors
// bd71837-regulator.c ROHM BD71837MWV/BD71847MWV regulator driver

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/rohm-bd718x7.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

#define BUCK1_RUN_DEFAULT               900000
#define BUCK1_SUSP_DEFAULT              900000
#define BUCK1_IDLE_DEFAULT              900000

#define BUCK2_RUN_DEFAULT               1000000
#define BUCK2_IDLE_DEFAULT              900000

#define BUCK3_RUN_DEFAULT               1000000
#define BUCK4_RUN_DEFAULT               1000000

#define BD71837_DVS_BUCK_NUM		4	/* Buck 1/2/3/4 support DVS */
#define BD71837_DVS_RUN_IDLE_SUSP	3
#define BD71837_DVS_RUN_IDLE		2
#define BD71837_DVS_RUN				1

struct bd71837_buck_dvs {
	u32 voltage[BD71837_DVS_RUN_IDLE_SUSP];
};

/*
 * BUCK1/2/3/4
 * BUCK1RAMPRATE[1:0] BUCK1 DVS ramp rate setting
 * 00: 10.00mV/usec 10mV 1uS
 * 01: 5.00mV/usec	10mV 2uS
 * 10: 2.50mV/usec	10mV 4uS
 * 11: 1.25mV/usec	10mV 8uS
 */
static int bd718xx_buck1234_set_ramp_delay(struct regulator_dev *rdev,
					   int ramp_delay)
{
	int id = rdev->desc->id;
	unsigned int ramp_value = BUCK_RAMPRATE_10P00MV;

	dev_dbg(&rdev->dev, "Buck[%d] Set Ramp = %d\n", id + 1,
		ramp_delay);
	switch (ramp_delay) {
	case 1 ... 1250:
		ramp_value = BUCK_RAMPRATE_1P25MV;
		break;
	case 1251 ... 2500:
		ramp_value = BUCK_RAMPRATE_2P50MV;
		break;
	case 2501 ... 5000:
		ramp_value = BUCK_RAMPRATE_5P00MV;
		break;
	case 5001 ... 10000:
		ramp_value = BUCK_RAMPRATE_10P00MV;
		break;
	default:
		ramp_value = BUCK_RAMPRATE_10P00MV;
		dev_err(&rdev->dev,
			"%s: ramp_delay: %d not supported, setting 10000mV//us\n",
			rdev->desc->name, ramp_delay);
	}

	return regmap_update_bits(rdev->regmap, BD718XX_REG_BUCK1_CTRL + id,
				  BUCK_RAMPRATE_MASK, ramp_value << 6);
}

static struct regulator_ops bd718xx_pickable_range_ldo_ops = {
	.list_voltage = regulator_list_voltage_pickable_linear_range,
	.get_voltage_sel = regulator_get_voltage_sel_pickable_regmap,
};

static struct regulator_ops bd718xx_pickable_range_buck_ops = {
	.list_voltage = regulator_list_voltage_pickable_linear_range,
	.get_voltage_sel = regulator_get_voltage_sel_pickable_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

static struct regulator_ops bd718xx_ldo_regulator_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static struct regulator_ops bd718xx_ldo_regulator_nolinear_ops = {
	.list_voltage = regulator_list_voltage_table,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static struct regulator_ops bd718xx_buck_regulator_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

static struct regulator_ops bd718xx_buck_regulator_nolinear_ops = {
	.list_voltage = regulator_list_voltage_table,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

static struct regulator_ops bd718xx_dvs_buck12_regulator_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay = bd718xx_buck1234_set_ramp_delay,
};

static struct regulator_ops bd71837_dvs_buck34_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay = bd718xx_buck1234_set_ramp_delay,
};

/*
 * BD71837 BUCK1/2/3/4
 * BD71847 BUCK1/2
 * 0.70 to 1.30V (10mV step)
 */
static const struct regulator_linear_range bd718xx_dvs_buck_volts[] = {
	REGULATOR_LINEAR_RANGE(700000, 0x00, 0x3C, 10000),
	REGULATOR_LINEAR_RANGE(1300000, 0x3D, 0x3F, 0),
};

/*
 * BD71837 BUCK5
 * 0.7V to 1.35V  (range 0)
 * and
 * 0.675 to 1.325 (range 1)
 */
static const struct regulator_linear_range bd71837_buck5_volts[] = {
	/* Ranges when VOLT_SEL bit is 0 */
	REGULATOR_LINEAR_RANGE(700000, 0x00, 0x03, 100000),
	REGULATOR_LINEAR_RANGE(1050000, 0x04, 0x05, 50000),
	REGULATOR_LINEAR_RANGE(1200000, 0x06, 0x07, 150000),
	/* Ranges when VOLT_SEL bit is 1  */
	REGULATOR_LINEAR_RANGE(675000, 0x0, 0x3, 100000),
	REGULATOR_LINEAR_RANGE(1025000, 0x4, 0x5, 50000),
	REGULATOR_LINEAR_RANGE(1175000, 0x6, 0x7, 150000),
};

/*
 * Range selector for first 3 linear ranges is 0x0
 * and 0x1 for last 3 ranges.
 */
static const unsigned int bd71837_buck5_volt_range_sel[] = {
	0x0, 0x0, 0x0, 0x80, 0x80, 0x80
};

/*
 * BD71847 BUCK3
 */
static const struct regulator_linear_range bd71847_buck3_volts[] = {
	/* Ranges when VOLT_SEL bits are 00 */
	REGULATOR_LINEAR_RANGE(700000, 0x00, 0x03, 100000),
	REGULATOR_LINEAR_RANGE(1050000, 0x04, 0x05, 50000),
	REGULATOR_LINEAR_RANGE(1200000, 0x06, 0x07, 150000),
	/* Ranges when VOLT_SEL bits are 01 */
	REGULATOR_LINEAR_RANGE(550000, 0x0, 0x7, 50000),
	/* Ranges when VOLT_SEL bits are 11 */
	REGULATOR_LINEAR_RANGE(675000, 0x0, 0x3, 100000),
	REGULATOR_LINEAR_RANGE(1025000, 0x4, 0x5, 50000),
	REGULATOR_LINEAR_RANGE(1175000, 0x6, 0x7, 150000),
};

static const unsigned int bd71847_buck3_volt_range_sel[] = {
	0x0, 0x0, 0x0, 0x40, 0x80, 0x80, 0x80
};

static const struct regulator_linear_range bd71847_buck4_volts[] = {
	REGULATOR_LINEAR_RANGE(3000000, 0x00, 0x03, 100000),
	REGULATOR_LINEAR_RANGE(2600000, 0x00, 0x03, 100000),
};

static const unsigned int bd71847_buck4_volt_range_sel[] = { 0x0, 0x40 };

/*
 * BUCK6
 * 3.0V to 3.3V (step 100mV)
 */
static const struct regulator_linear_range bd71837_buck6_volts[] = {
	REGULATOR_LINEAR_RANGE(3000000, 0x00, 0x03, 100000),
};

/*
 * BD71837 BUCK7
 * BD71847 BUCK5
 * 000 = 1.605V
 * 001 = 1.695V
 * 010 = 1.755V
 * 011 = 1.8V (Initial)
 * 100 = 1.845V
 * 101 = 1.905V
 * 110 = 1.95V
 * 111 = 1.995V
 */
static const unsigned int bd718xx_3rd_nodvs_buck_volts[] = {
	1605000, 1695000, 1755000, 1800000, 1845000, 1905000, 1950000, 1995000
};

/*
 * BUCK8
 * 0.8V to 1.40V (step 10mV)
 */
static const struct regulator_linear_range bd718xx_4th_nodvs_buck_volts[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x3C, 10000),
};

/*
 * LDO1
 * 3.0 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd718xx_ldo1_volts[] = {
	REGULATOR_LINEAR_RANGE(3000000, 0x00, 0x03, 100000),
	REGULATOR_LINEAR_RANGE(1600000, 0x00, 0x03, 100000),
};

static const unsigned int bd718xx_ldo1_volt_range_sel[] = { 0x0, 0x20 };

/*
 * LDO2
 * 0.8 or 0.9V
 */
const unsigned int ldo_2_volts[] = {
	900000, 800000
};

/*
 * LDO3
 * 1.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd718xx_ldo3_volts[] = {
	REGULATOR_LINEAR_RANGE(1800000, 0x00, 0x0F, 100000),
};

/*
 * LDO4
 * 0.9 to 1.8V (100mV step)
 */
static const struct regulator_linear_range bd718xx_ldo4_volts[] = {
	REGULATOR_LINEAR_RANGE(900000, 0x00, 0x09, 100000),
};

/*
 * LDO5 for BD71837
 * 1.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd71837_ldo5_volts[] = {
	REGULATOR_LINEAR_RANGE(1800000, 0x00, 0x0F, 100000),
};

/*
 * LDO5 for BD71837
 * 1.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd71847_ldo5_volts[] = {
	REGULATOR_LINEAR_RANGE(1800000, 0x00, 0x0F, 100000),
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x0F, 100000),
};

static const unsigned int bd71847_ldo5_volt_range_sel[] = { 0x0, 0x20 };

/*
 * LDO6
 * 0.9 to 1.8V (100mV step)
 */
static const struct regulator_linear_range bd718xx_ldo6_volts[] = {
	REGULATOR_LINEAR_RANGE(900000, 0x00, 0x09, 100000),
};

/*
 * LDO7
 * 1.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range bd71837_ldo7_volts[] = {
	REGULATOR_LINEAR_RANGE(1800000, 0x00, 0x0F, 100000),
};

struct reg_init {
	unsigned int reg;
	unsigned int mask;
	unsigned int val;
};

struct state_volts {
	int r_i_s_voltages[3];
	uint8_t r_i_s_regs[3];
	uint8_t r_i_s_masks[3];
};

struct bd718xx_regulator_data {
	struct regulator_desc desc;
	const struct reg_init *additional_inits;
	int additional_init_amnt;
	struct state_volts *state_volts;
};

static struct state_volts buck1_default_dvs = {
	.r_i_s_voltages = {
		BUCK1_RUN_DEFAULT,
		BUCK1_IDLE_DEFAULT,
		BUCK1_SUSP_DEFAULT
	},
	.r_i_s_regs = {
		BD718XX_REG_BUCK1_VOLT_RUN,
		BD718XX_REG_BUCK1_VOLT_IDLE,
		BD718XX_REG_BUCK1_VOLT_SUSP,
	},
};
static struct state_volts buck2_default_dvs = {
	.r_i_s_voltages = {
		BUCK2_RUN_DEFAULT,
		BUCK2_IDLE_DEFAULT,
	},
	.r_i_s_regs = {
		BD718XX_REG_BUCK2_VOLT_RUN,
		BD718XX_REG_BUCK2_VOLT_IDLE,
		0 /* Suspend not supported */,
	},
};
static struct state_volts buck3_default_dvs = {
	.r_i_s_voltages = {
		BUCK3_RUN_DEFAULT,
	},
	.r_i_s_regs = {
		BD71837_REG_BUCK3_VOLT_RUN,
		0 /* Neither idle... */,
		0 /* ...nor suspend are supported*/,
	},
};
static struct state_volts buck4_default_dvs = {
	.r_i_s_voltages = {
		BUCK4_RUN_DEFAULT,
	},
	.r_i_s_regs = {
		BD71837_REG_BUCK4_VOLT_RUN,
		0 /* Neither idle... */,
		0 /* ...nor suspend are supported*/,
	},
};


/*
 * There is a HW quirk in BD71837. The shutdown sequence timings for
 * bucks/LDOs which are controlled via register interface are changed.
 * At PMIC poweroff the voltage for BUCK6/7 is cut immediately at the
 * beginning of shut-down sequence. As bucks 6 and 7 are parent
 * supplies for LDO5 and LDO6 - this causes LDO5/6 voltage
 * monitoring to errorneously detect under voltage and force PMIC to
 * emergency state instead of poweroff. In order to avoid this we
 * disable voltage monitoring for LDO5 and LDO6
 */	
static const struct reg_init bd71837_buck3_inits[] = {
	{
		.reg = BD71837_REG_BUCK3_CTRL,
		.mask = BD718XX_BUCK_SEL,
		.val = BD718XX_BUCK_SEL,
	},
};
static const struct reg_init bd71837_buck4_inits[] = {
	{
		.reg = BD71837_REG_BUCK4_CTRL,
		.mask = BD718XX_BUCK_SEL,
		.val = BD718XX_BUCK_SEL,
	},
};

static const struct reg_init bd71837_ldo5_inits[] = {
	{
		.reg = BD718XX_REG_MVRFLTMASK2,
		.mask = BD718XX_LDO5_VRMON80,
		.val = BD718XX_LDO5_VRMON80,
	},
};

static const struct reg_init bd71837_ldo6_inits[] = {
	{
		.reg = BD718XX_REG_MVRFLTMASK2,
		.mask = BD718XX_LDO6_VRMON80,
		.val = BD718XX_LDO6_VRMON80,
	},
};

static const struct bd718xx_regulator_data bd71847_regulators[] = {
	{
		.desc = {
			.name = "buck1",
			.of_match = of_match_ptr("BUCK1"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK1,
			.ops = &bd718xx_dvs_buck12_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_DVS_BUCK_VOLTAGE_NUM,
			.linear_ranges = bd718xx_dvs_buck_volts,
			.n_linear_ranges =
				ARRAY_SIZE(bd718xx_dvs_buck_volts),
			.vsel_reg = BD718XX_REG_BUCK1_VOLT_RUN,
			.vsel_mask = DVS_BUCK_RUN_MASK,
			.owner = THIS_MODULE,
		},
		.state_volts = &buck1_default_dvs,
	},
	{
		.desc = {
			.name = "buck2",
			.of_match = of_match_ptr("BUCK2"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK2,
			.ops = &bd718xx_dvs_buck12_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_DVS_BUCK_VOLTAGE_NUM,
			.linear_ranges = bd718xx_dvs_buck_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_dvs_buck_volts),
			.vsel_reg = BD718XX_REG_BUCK2_VOLT_RUN,
			.vsel_mask = DVS_BUCK_RUN_MASK,
			.owner = THIS_MODULE,
		},
		.state_volts = &buck2_default_dvs,
	},
	{
		.desc = {
			.name = "buck3",
			.of_match = of_match_ptr("BUCK3"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK3,
			.ops = &bd718xx_pickable_range_buck_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD71847_BUCK3_VOLTAGE_NUM,
			.linear_ranges = bd71847_buck3_volts,
			.n_linear_ranges =
				ARRAY_SIZE(bd71847_buck3_volts),
			.vsel_reg = BD718XX_REG_1ST_NODVS_BUCK_VOLT,
			.vsel_mask = BD718XX_1ST_NODVS_BUCK_MASK,
			.vsel_range_reg = BD718XX_REG_1ST_NODVS_BUCK_VOLT,
			.vsel_range_mask = BD71847_BUCK3_RANGE_MASK,
			.linear_range_selectors = bd71847_buck3_volt_range_sel,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "buck4",
			.of_match = of_match_ptr("BUCK4"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK4,
			.ops = &bd718xx_pickable_range_buck_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD71847_BUCK4_VOLTAGE_NUM,
			.linear_ranges = bd71847_buck4_volts,
			.n_linear_ranges =
				ARRAY_SIZE(bd71847_buck4_volts),
			.vsel_reg = BD718XX_REG_2ND_NODVS_BUCK_VOLT,
			.vsel_mask = BD71847_BUCK4_MASK,
			.vsel_range_reg = BD718XX_REG_2ND_NODVS_BUCK_VOLT,
			.vsel_range_mask = BD71847_BUCK4_RANGE_MASK,
			.linear_range_selectors = bd71847_buck4_volt_range_sel,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "buck5",
			.of_match = of_match_ptr("BUCK5"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK5,
			.ops = &bd718xx_buck_regulator_nolinear_ops,
			.type = REGULATOR_VOLTAGE,
			.volt_table = &bd718xx_3rd_nodvs_buck_volts[0],
			.n_voltages = ARRAY_SIZE(bd718xx_3rd_nodvs_buck_volts),
			.vsel_reg = BD718XX_REG_3RD_NODVS_BUCK_VOLT,
			.vsel_mask = BD718XX_3RD_NODVS_BUCK_MASK,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "buck6",
			.of_match = of_match_ptr("BUCK6"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK6,
			.ops = &bd718xx_buck_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_4TH_NODVS_BUCK_VOLTAGE_NUM,
			.linear_ranges = bd718xx_4th_nodvs_buck_volts,
			.n_linear_ranges =
				ARRAY_SIZE(bd718xx_4th_nodvs_buck_volts),
			.vsel_reg = BD718XX_REG_4TH_NODVS_BUCK_VOLT,
			.vsel_mask = BD718XX_4TH_NODVS_BUCK_MASK,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "ldo1",
			.of_match = of_match_ptr("LDO1"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO1,
			.ops = &bd718xx_pickable_range_ldo_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_LDO1_VOLTAGE_NUM,
			.linear_ranges = bd718xx_ldo1_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_ldo1_volts),
			.vsel_reg = BD718XX_REG_LDO1_VOLT,
			.vsel_mask = BD718XX_LDO1_MASK,
			.vsel_range_reg = BD718XX_REG_LDO1_VOLT,
			.vsel_range_mask = BD718XX_LDO1_RANGE_MASK,
			.linear_range_selectors = bd718xx_ldo1_volt_range_sel,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "ldo2",
			.of_match = of_match_ptr("LDO2"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO2,
			.ops = &bd718xx_ldo_regulator_nolinear_ops,
			.type = REGULATOR_VOLTAGE,
			.volt_table = &ldo_2_volts[0],
			.vsel_reg = BD718XX_REG_LDO2_VOLT,
			.vsel_mask = BD718XX_LDO2_MASK,
			.n_voltages = ARRAY_SIZE(ldo_2_volts),
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "ldo3",
			.of_match = of_match_ptr("LDO3"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO3,
			.ops = &bd718xx_ldo_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_LDO3_VOLTAGE_NUM,
			.linear_ranges = bd718xx_ldo3_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_ldo3_volts),
			.vsel_reg = BD718XX_REG_LDO3_VOLT,
			.vsel_mask = BD718XX_LDO3_MASK,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "ldo4",
			.of_match = of_match_ptr("LDO4"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO4,
			.ops = &bd718xx_ldo_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_LDO4_VOLTAGE_NUM,
			.linear_ranges = bd718xx_ldo4_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_ldo4_volts),
			.vsel_reg = BD718XX_REG_LDO4_VOLT,
			.vsel_mask = BD718XX_LDO4_MASK,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "ldo5",
			.of_match = of_match_ptr("LDO5"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO5,
			.ops = &bd718xx_pickable_range_ldo_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD71847_LDO5_VOLTAGE_NUM,
			.linear_ranges = bd71847_ldo5_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71847_ldo5_volts),
			.vsel_reg = BD718XX_REG_LDO5_VOLT,
			.vsel_mask = BD71847_LDO5_MASK,
			.vsel_range_reg = BD718XX_REG_LDO5_VOLT,
			.vsel_range_mask = BD71847_LDO5_RANGE_MASK,
			.linear_range_selectors = bd71847_ldo5_volt_range_sel,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "ldo6",
			.of_match = of_match_ptr("LDO6"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO6,
			.ops = &bd718xx_ldo_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_LDO6_VOLTAGE_NUM,
			.linear_ranges = bd718xx_ldo6_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_ldo6_volts),
			/* LDO6 is supplied by buck5 */
			.supply_name = "buck5",
			.vsel_reg = BD718XX_REG_LDO6_VOLT,
			.vsel_mask = BD718XX_LDO6_MASK,
			.owner = THIS_MODULE,
		},
	},
};

static const struct bd718xx_regulator_data bd71837_regulators[] = {
	{
		.desc = {
			.name = "buck1",
			.of_match = of_match_ptr("BUCK1"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK1,
			.ops = &bd718xx_dvs_buck12_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_DVS_BUCK_VOLTAGE_NUM,
			.linear_ranges = bd718xx_dvs_buck_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_dvs_buck_volts),
			.vsel_reg = BD718XX_REG_BUCK1_VOLT_RUN,
			.vsel_mask = DVS_BUCK_RUN_MASK,
			.owner = THIS_MODULE,
		},
		.state_volts = &buck1_default_dvs,
	},
	{
		.desc = {
			.name = "buck2",
			.of_match = of_match_ptr("BUCK2"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK2,
			.ops = &bd718xx_dvs_buck12_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_DVS_BUCK_VOLTAGE_NUM,
			.linear_ranges = bd718xx_dvs_buck_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_dvs_buck_volts),
			.vsel_reg = BD718XX_REG_BUCK2_VOLT_RUN,
			.vsel_mask = DVS_BUCK_RUN_MASK,
			.owner = THIS_MODULE,
		},
		.state_volts = &buck2_default_dvs,
	},
	{
		.desc = {
			.name = "buck3",
			.of_match = of_match_ptr("BUCK3"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK3,
			.ops = &bd71837_dvs_buck34_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_DVS_BUCK_VOLTAGE_NUM,
			.linear_ranges = bd718xx_dvs_buck_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_dvs_buck_volts),
			.vsel_reg = BD71837_REG_BUCK3_VOLT_RUN,
			.vsel_mask = DVS_BUCK_RUN_MASK,
			.enable_reg = BD71837_REG_BUCK3_CTRL,
			.enable_mask = BD718XX_BUCK_EN,
			.owner = THIS_MODULE,
		},
		.state_volts = &buck3_default_dvs,
		.additional_inits = bd71837_buck3_inits,
		.additional_init_amnt = ARRAY_SIZE(bd71837_buck3_inits),
	},
	{
		.desc = {
			.name = "buck4",
			.of_match = of_match_ptr("BUCK4"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK4,
			.ops = &bd71837_dvs_buck34_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_DVS_BUCK_VOLTAGE_NUM,
			.linear_ranges = bd718xx_dvs_buck_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_dvs_buck_volts),
			.vsel_reg = BD71837_REG_BUCK4_VOLT_RUN,
			.vsel_mask = DVS_BUCK_RUN_MASK,
			.enable_reg = BD71837_REG_BUCK4_CTRL,
			.enable_mask = BD718XX_BUCK_EN,
			.owner = THIS_MODULE,
		},
		.state_volts = &buck4_default_dvs,
		.additional_inits = bd71837_buck4_inits,
		.additional_init_amnt = ARRAY_SIZE(bd71837_buck4_inits),
	},
	{
		.desc = {
			.name = "buck5",
			.of_match = of_match_ptr("BUCK5"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK5,
			.ops = &bd718xx_pickable_range_buck_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD71837_BUCK5_VOLTAGE_NUM,
			.linear_ranges = bd71837_buck5_volts,
			.n_linear_ranges =
				ARRAY_SIZE(bd71837_buck5_volts),
			.vsel_reg = BD718XX_REG_1ST_NODVS_BUCK_VOLT,
			.vsel_mask = BD71837_BUCK5_MASK,
			.vsel_range_reg = BD718XX_REG_1ST_NODVS_BUCK_VOLT,
			.vsel_range_mask = BD71837_BUCK5_RANGE_MASK,
			.linear_range_selectors = bd71837_buck5_volt_range_sel,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "buck6",
			.of_match = of_match_ptr("BUCK6"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK6,
			.ops = &bd718xx_buck_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD71837_BUCK6_VOLTAGE_NUM,
			.linear_ranges = bd71837_buck6_volts,
			.n_linear_ranges =
				ARRAY_SIZE(bd71837_buck6_volts),
			.vsel_reg = BD718XX_REG_2ND_NODVS_BUCK_VOLT,
			.vsel_mask = BD71837_BUCK6_MASK,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "buck7",
			.of_match = of_match_ptr("BUCK7"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK7,
			.ops = &bd718xx_buck_regulator_nolinear_ops,
			.type = REGULATOR_VOLTAGE,
			.volt_table = &bd718xx_3rd_nodvs_buck_volts[0],
			.n_voltages = ARRAY_SIZE(bd718xx_3rd_nodvs_buck_volts),
			.vsel_reg = BD718XX_REG_3RD_NODVS_BUCK_VOLT,
			.vsel_mask = BD718XX_3RD_NODVS_BUCK_MASK,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "buck8",
			.of_match = of_match_ptr("BUCK8"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_BUCK8,
			.ops = &bd718xx_buck_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_4TH_NODVS_BUCK_VOLTAGE_NUM,
			.linear_ranges = bd718xx_4th_nodvs_buck_volts,
			.n_linear_ranges =
				ARRAY_SIZE(bd718xx_4th_nodvs_buck_volts),
			.vsel_reg = BD718XX_REG_4TH_NODVS_BUCK_VOLT,
			.vsel_mask = BD718XX_4TH_NODVS_BUCK_MASK,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "ldo1",
			.of_match = of_match_ptr("LDO1"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO1,
			.ops = &bd718xx_pickable_range_ldo_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_LDO1_VOLTAGE_NUM,
			.linear_ranges = bd718xx_ldo1_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_ldo1_volts),
			.vsel_reg = BD718XX_REG_LDO1_VOLT,
			.vsel_mask = BD718XX_LDO1_MASK,
			.vsel_range_reg = BD718XX_REG_LDO1_VOLT,
			.vsel_range_mask = BD718XX_LDO1_RANGE_MASK,
			.linear_range_selectors = bd718xx_ldo1_volt_range_sel,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "ldo2",
			.of_match = of_match_ptr("LDO2"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO2,
			.ops = &bd718xx_ldo_regulator_nolinear_ops,
			.type = REGULATOR_VOLTAGE,
			.volt_table = &ldo_2_volts[0],
			.vsel_reg = BD718XX_REG_LDO2_VOLT,
			.vsel_mask = BD718XX_LDO2_MASK,
			.n_voltages = ARRAY_SIZE(ldo_2_volts),
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "ldo3",
			.of_match = of_match_ptr("LDO3"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO3,
			.ops = &bd718xx_ldo_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_LDO3_VOLTAGE_NUM,
			.linear_ranges = bd718xx_ldo3_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_ldo3_volts),
			.vsel_reg = BD718XX_REG_LDO3_VOLT,
			.vsel_mask = BD718XX_LDO3_MASK,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "ldo4",
			.of_match = of_match_ptr("LDO4"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO4,
			.ops = &bd718xx_ldo_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_LDO4_VOLTAGE_NUM,
			.linear_ranges = bd718xx_ldo4_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_ldo4_volts),
			.vsel_reg = BD718XX_REG_LDO4_VOLT,
			.vsel_mask = BD718XX_LDO4_MASK,
			.owner = THIS_MODULE,
		},
	},
	{
		.desc = {
			.name = "ldo5",
			.of_match = of_match_ptr("LDO5"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO5,
			.ops = &bd718xx_ldo_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD71837_LDO5_VOLTAGE_NUM,
			.linear_ranges = bd71837_ldo5_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71837_ldo5_volts),
			/* LDO5 is supplied by buck6 */
			.supply_name = "buck6",
			.vsel_reg = BD718XX_REG_LDO5_VOLT,
			.vsel_mask = BD71837_LDO5_MASK,
			.owner = THIS_MODULE,
		},
		.additional_inits = bd71837_ldo5_inits,
		.additional_init_amnt = ARRAY_SIZE(bd71837_ldo5_inits),
	},
	{
		.desc = {
			.name = "ldo6",
			.of_match = of_match_ptr("LDO6"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO6,
			.ops = &bd718xx_ldo_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD718XX_LDO6_VOLTAGE_NUM,
			.linear_ranges = bd718xx_ldo6_volts,
			.n_linear_ranges = ARRAY_SIZE(bd718xx_ldo6_volts),
			/* LDO6 is supplied by buck7 */
			.supply_name = "buck7",
			.vsel_reg = BD718XX_REG_LDO6_VOLT,
			.vsel_mask = BD718XX_LDO6_MASK,
			.owner = THIS_MODULE,
		},
		.additional_inits = bd71837_ldo6_inits,
		.additional_init_amnt = ARRAY_SIZE(bd71837_ldo6_inits),
	},
	{
		.desc = {
			.name = "ldo7",
			.of_match = of_match_ptr("LDO7"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD718XX_LDO7,
			.ops = &bd718xx_ldo_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = BD71837_LDO7_VOLTAGE_NUM,
			.linear_ranges = bd71837_ldo7_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71837_ldo7_volts),
			.vsel_reg = BD71837_REG_LDO7_VOLT,
			.vsel_mask = BD71837_LDO7_MASK,
			.owner = THIS_MODULE,
		},
	},
};

struct bd718xx_pmic_inits {
	const struct bd718xx_regulator_data *r_datas;
	unsigned int r_amount;
};

/** @brief buck1/2 dvs enable/voltage from device tree
 * @param pdev platfrom device pointer
 * @param buck_dvs pointer
 * @return void
 */
static int of_bd71837_buck_dvs(struct platform_device *pdev, int num_bucks)
{
	int ret;
	int *dvs_storage[] = {
		&buck1_default_dvs.r_i_s_voltages[0],
		&buck2_default_dvs.r_i_s_voltages[0],
		&buck3_default_dvs.r_i_s_voltages[0],
		&buck4_default_dvs.r_i_s_voltages[0],
	};
	const char *dvs_props[] = {
		"rohm,pmic-buck1-dvs-voltage",
		"rohm,pmic-buck2-dvs-voltage",
		"rohm,pmic-buck3-dvs-voltage",
		"rohm,pmic-buck4-dvs-voltage",
		NULL,
	};
	int num_expected_values[] = {
		BD71837_DVS_RUN_IDLE_SUSP,
		BD71837_DVS_RUN_IDLE,
		BD71837_DVS_RUN,
		BD71837_DVS_RUN,
	};
	int i;

	if (!pdev->dev.parent->of_node) {
		dev_err(&pdev->dev, "could not find pmic sub-node\n");
		return -ENOENT;
	}

	for (i=0; dvs_props[i] && i < num_bucks; i++) {

		ret = of_property_read_u32_array(pdev->dev.parent->of_node,
						 dvs_props[i], dvs_storage[i],
						 num_expected_values[i]);

		if (ret == -EINVAL)
			break;

		if (ret) {
			dev_err(&pdev->dev,
				"Bad amount of dvs voltages for buck%d in DT\n",
				i);
			return ret;
		}
	}

	return 0;
}

static int bd718xx_probe(struct platform_device *pdev)
{
	struct bd718xx *mfd;
	struct regulator_config config = { 0 };
	struct bd718xx_pmic_inits pmic_regulators[] = {
		[BD718XX_TYPE_BD71837] = {
			.r_datas = bd71837_regulators,
			.r_amount = ARRAY_SIZE(bd71837_regulators),
		},
		[BD718XX_TYPE_BD71847] = {
			.r_datas = bd71847_regulators,
			.r_amount = ARRAY_SIZE(bd71847_regulators),
		},
	};

	int i, j, err, num_dvs_bucks = 4;

	mfd = dev_get_drvdata(pdev->dev.parent);
	if (!mfd) {
		dev_err(&pdev->dev, "No MFD driver data\n");
		err = -EINVAL;
		goto err;
	}

	if (mfd->chip_type >= BD718XX_TYPE_AMOUNT ||
	    !pmic_regulators[mfd->chip_type].r_datas) {
		dev_err(&pdev->dev, "Unsupported chip type\n");
		err = -EINVAL;
		goto err;
	}
	if (mfd->chip_type == BD718XX_TYPE_BD71847)
		num_dvs_bucks = 2;
	err = of_bd71837_buck_dvs(pdev, num_dvs_bucks);

	if (err)
		goto err;
	/* Register LOCK release */
	err = regmap_update_bits(mfd->regmap, BD718XX_REG_REGLOCK,
				 (REGLOCK_PWRSEQ | REGLOCK_VREG), 0);
	if (err) {
		dev_err(&pdev->dev, "Failed to unlock PMIC (%d)\n", err);
		goto err;
	} else {
		dev_dbg(&pdev->dev, "Unlocked lock register 0x%x\n",
			BD718XX_REG_REGLOCK);
	}

	for (i = 0; i < pmic_regulators[mfd->chip_type].r_amount; i++) {

		const struct regulator_desc *desc;
		struct regulator_dev *rdev;
		const struct bd718xx_regulator_data *r;

		r = &pmic_regulators[mfd->chip_type].r_datas[i];
		desc = &r->desc;

		config.dev = pdev->dev.parent;
		config.regmap = mfd->regmap;

		rdev = devm_regulator_register(&pdev->dev, desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev,
				"failed to register %s regulator\n",
				desc->name);
			err = PTR_ERR(rdev);
			goto err;
		}
		if (r->state_volts) {
			/* This simple loop works as long as the run, idle
			 * and suspend voltages for buck are selected with
			 * identical mask/value. This is true for BD71837
			 */
			for (j = 0; j < 3; j++) {
				if (r->state_volts->r_i_s_regs[j]) {
					int sel = regulator_map_voltage_iterate(rdev,
						r->state_volts->r_i_s_voltages[j],
						r->state_volts->r_i_s_voltages[j]);
					if (sel < 0) {
						dev_err(&pdev->dev,
							"selector for voltage [%d] not found\n",
							r->state_volts->r_i_s_voltages[j]);
					} else {
						err = regmap_update_bits(mfd->regmap,
							r->state_volts->r_i_s_regs[j],
							DVS_BUCK_RUN_MASK,
							sel);
						if (err) {
							dev_err(&pdev->dev,
								"Failed to write DVS voltage\n");
							goto err;
						}
					}
				}
			}
		}
		for (j = 0; j < r->additional_init_amnt; j++) {
			err = regmap_update_bits(mfd->regmap,
						 r->additional_inits[j].reg,
						 r->additional_inits[j].mask,
						 r->additional_inits[j].val);
			if (err) {
				dev_err(&pdev->dev,
					"Buck (%s) initialization failed\n",
					desc->name);
				goto err;
			}
		}
	}

err:
	return err;
}

static struct platform_driver bd718xx_regulator = {
	.driver = {
		.name = "bd718xx-pmic",
	},
	.probe = bd718xx_probe,
};

module_platform_driver(bd718xx_regulator);

MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_DESCRIPTION("BD71837/BD71847 voltage regulator driver");
MODULE_LICENSE("GPL");
