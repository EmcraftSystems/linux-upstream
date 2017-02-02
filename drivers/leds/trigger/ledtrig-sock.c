/*
 * LED Socket Activity Trigger
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

DEFINE_LED_TRIGGER(ledtrig_sock);

void ledtrig_sock_activity(int count)
{
	enum led_brightness brt = count ? LED_FULL : LED_OFF;

	led_trigger_event(ledtrig_sock, brt);
}
EXPORT_SYMBOL(ledtrig_sock_activity);

static int __init ledtrig_sock_init(void)
{
	led_trigger_register_simple("sock", &ledtrig_sock);
	return 0;
}

static void __exit ledtrig_sock_exit(void)
{
	led_trigger_unregister_simple(ledtrig_sock);
}

module_init(ledtrig_sock_init);
module_exit(ledtrig_sock_exit);

MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_DESCRIPTION("LED Socket Activity Trigger");
MODULE_LICENSE("GPL");
