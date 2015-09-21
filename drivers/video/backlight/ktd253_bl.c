/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/ktd_bl.h>
#define __BACKLIGHT_DEBUG__   0

#define BACKLIGHT_DEV_NAME	"panel"

int BL_brightness;
static int bl_enable;
int current_brightness;
int wakeup_brightness;


#if defined(CONFIG_MACH_CS02)
#include "ktd253_bl_cs02.h"
#else
#include "ktd253_bl.h"
#endif

#define MAX_BRIGHTNESS_IN_BLU	32 // backlight-IC MAX VALUE
static int lcd_brightness = DEFAULT_PULSE;
static u8 is_dimming = 0;
static DEFINE_SPINLOCK(bl_ctrl_lock);

#define NB_BRT_LEVEL (int)(sizeof(brt_table_ktd)/sizeof(struct brt_value))

u8 ktd_backlight_is_dimming(void)
{
	return is_dimming;
}

void ktd_backlight_set_brightness(int level)
{
	int pulse;
	int tune_level = 0;
	int i;

	spin_lock(&bl_ctrl_lock);
	if (level > 0) {
		if (level < MIN_BRIGHTNESS) {
			tune_level = DIMMING_VALUE; /* DIMMING */
			is_dimming = 1;
		} else {
			for (i = 0; i < NB_BRT_LEVEL; i++) {
				if (level <= brt_table_ktd[i].level
						&& level > brt_table_ktd[i+1].level) {
					tune_level = brt_table_ktd[i].tune_level;
					break;
				}
			}
			is_dimming = 0;
		}
	} /*  BACKLIGHT is KTD model */
	printk("set_brightness : level(%d) tune (%d)\n",level, tune_level);
	current_brightness = level;

	if (!tune_level) {
		gpio_set_value(GPIO_BL_CTRL, 0);
		mdelay(3);
		lcd_brightness = tune_level;
		goto out;
	}

	if (unlikely(lcd_brightness < 0)) {
		int val = gpio_get_value(GPIO_BL_CTRL);
		if (val) {
			lcd_brightness = 0;
			gpio_set_value(GPIO_BL_CTRL, 0);
			mdelay(3);
			printk(KERN_INFO "LCD Baklight init in boot time on kernel\n");
		}
	}
	if (!lcd_brightness) {
		gpio_set_value(GPIO_BL_CTRL, 1);
		udelay(BL_INIT_DELAY);
		lcd_brightness = MAX_BRIGHTNESS_IN_BLU;
	}

	pulse = (tune_level - lcd_brightness + MAX_BRIGHTNESS_IN_BLU)
		% MAX_BRIGHTNESS_IN_BLU;

	for (; pulse > 0; pulse--) {
		gpio_set_value(GPIO_BL_CTRL, 0);
		udelay(3);
		gpio_set_value(GPIO_BL_CTRL, 1);
		udelay(3);
	}

	lcd_brightness = tune_level;
out:
	mdelay(1);
	spin_unlock(&bl_ctrl_lock);
	return;
}

void backlight_set_brightness(int level)
{
	ktd_backlight_set_brightness(level);
	return;
}

static int ktd_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	wakeup_brightness = brightness;
	if (!bl_enable && !current_brightness) {
		printk(KERN_INFO "[Backlight] no need to set backlight ---\n");
	} else {
		ktd_backlight_set_brightness(brightness);
	}

	return 0;
}

static int ktd_backlight_get_brightness(struct backlight_device *bl)
{
	BL_brightness = bl->props.brightness;
	return BL_brightness;
}

static const struct backlight_ops ktd_backlight_ops = {
	.update_status	= ktd_backlight_update_status,
	.get_brightness	= ktd_backlight_get_brightness,
};

int ktd_backlight_disable(void)
{
	bl_enable = false;

	return 0;
}

int ktd_backlight_enable(void)
{
	bl_enable = true;

	return 0;
}

static int ktd_backlight_probe(struct platform_device *pdev)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	int ret;

	printk("[coko] %s\n",__FUNCTION__);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = MAX_BRIGHTNESS;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register(BACKLIGHT_DEV_NAME, &pdev->dev, NULL,
			&ktd_backlight_ops, &props);

	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		return PTR_ERR(bl);
	}

	bl->props.max_brightness = MAX_BRIGHTNESS;
	bl->props.brightness = DEFAULT_BRIGHTNESS;

	platform_set_drvdata(pdev, bl);

	if (gpio_request(GPIO_BL_CTRL,"BL_CTRL")) {
		printk(KERN_ERR "Request GPIO failed,""gpio: %d \n", GPIO_BL_CTRL);
	}

	gpio_direction_output(GPIO_BL_CTRL, 1);

	bl_enable = true;
	backlight_update_status(bl);
	printk("[coko] %s end\n",__FUNCTION__);
	return 0;
out:
	return ret;
}

static int ktd_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	backlight_device_unregister(bl);
	gpio_direction_output(GPIO_BL_CTRL, 0);
	mdelay(3);

	return 0;
}

void ktd_backlight_shutdown(struct platform_device *pdev)
{
	printk("[coko] %s\n",__FUNCTION__);
	gpio_direction_output(GPIO_BL_CTRL, 0);
	mdelay(5);
}

static struct platform_driver ktd_backlight_driver = {
	.driver		= {
		.name	= BACKLIGHT_DEV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= ktd_backlight_probe,
	.remove		= ktd_backlight_remove,
	.shutdown       = ktd_backlight_shutdown,
};

static int __init ktd_backlight_init(void)
{
	return platform_driver_register(&ktd_backlight_driver);
}
module_init(ktd_backlight_init);

static void __exit ktd_backlight_exit(void)
{
	platform_driver_unregister(&ktd_backlight_driver);
}
module_exit(ktd_backlight_exit);

MODULE_DESCRIPTION("KTD based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ktd-backlight");



