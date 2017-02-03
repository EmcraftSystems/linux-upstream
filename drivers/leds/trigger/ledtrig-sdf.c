/*
 * LED Serial Data Flash Activity Trigger
 *
 * Copyright (C) EmCraft Systems 2017
 * Author: Yuri Tikhonov <yur@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>

#define BLINK_DELAY 30

DEFINE_LED_TRIGGER(ledtrig_sdf);
static unsigned long sdf_blink_delay = BLINK_DELAY;

void ledtrig_sdf_activity(void)
{
	led_trigger_blink_oneshot(ledtrig_sdf,
				  &sdf_blink_delay, &sdf_blink_delay, 0);
}
EXPORT_SYMBOL(ledtrig_sdf_activity);

static int __init ledtrig_sdf_init(void)
{
	led_trigger_register_simple("sdf", &ledtrig_sdf);
	return 0;
}

static void __exit ledtrig_sdf_exit(void)
{
	led_trigger_unregister_simple(ledtrig_sdf);
}

module_init(ledtrig_sdf_init);
module_exit(ledtrig_sdf_exit);

MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_DESCRIPTION("LED Serial Data Flash Activity Trigger");
MODULE_LICENSE("GPL");
