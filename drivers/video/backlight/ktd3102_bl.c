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
#define MAX_BRIGHTNESS_IN_BLU	32 // backlight-IC MAX VALUE

static bool bl_enable;
int current_brightness;
int wakeup_brightness;
struct ktd_bl_info *g_bl_info;

static int prev_tune_level;
static DEFINE_SPINLOCK(bl_ctrl_lock);

u8 ktd_backlight_is_dimming(void)
{
	if (!g_bl_info) {
		printk("%s, g_bl_info is null\n", __func__);
		return 0;
	}
	return (current_brightness <= g_bl_info->min_brightness) ? 1 : 0;
}

static inline int find_tune_level(struct brt_value *brt_table, int nr_brt_level, int level)
{
	int i = 0;
	int curr_level, next_level;

	if (unlikely(level > 255 || level < 0)) {
		printk("%s, %d level - out of range\n", __func__,
				level);
		return -1;
	}

	if (unlikely(nr_brt_level <= 0)) {
		printk("%s, %d nr_brt_level - out of range\n",
				__func__, nr_brt_level);
		return -1;
	}

	for (i = 0; i < nr_brt_level - 1; i++) {
		curr_level = brt_table[i].level;
		next_level = brt_table[i + 1].level;
		if (level <= curr_level &&
				level > next_level) {
			return brt_table[i].tune_level;
		}
	}

	return brt_table[nr_brt_level - 1].tune_level;
}

int ktd_backlight_set_brightness(struct ktd_bl_info *bl_info, int level)
{
	int pulse;
	int tune_level = 0;

	struct brt_value *brt_table_ktd = bl_info->brt_table;
	unsigned int nr_brt_level = bl_info->sz_table;
	u8 min_brightness = bl_info->min_brightness;
	unsigned int gpio_bl_ctrl = bl_info->gpio_bl_ctrl;
	unsigned int gpio_bl_pwm_en = bl_info->gpio_bl_pwm_en;

	spin_lock(&bl_ctrl_lock);
	if (level > 0) {
		tune_level = find_tune_level(brt_table_ktd, nr_brt_level, level);
		if (tune_level < 0) {
			printk("%s, failed to find tune_level. (level : %d)\n",
					__func__, level);
			goto out;
		}
	}
	printk("set_brightness : level(%d) tune (%d)\n",level, tune_level);
	current_brightness = level;

	if (!tune_level) {
		gpio_direction_output(gpio_bl_pwm_en, 0);
		if (gpio_get_value(gpio_bl_pwm_en))
			printk(KERN_ERR "%s, error : gpio_bl_pwm_en is %s\n",
					__func__, gpio_get_value(gpio_bl_pwm_en) ? "HIGH" : "LOW");

		/* wait until vout gets stable */
		msleep(100);
		gpio_direction_output(gpio_bl_ctrl, 0);
		/* wait until chip gets stable */
		msleep(100);
		prev_tune_level = tune_level;
		goto out;
	}

	if (unlikely(prev_tune_level < 0)) {
		int val = gpio_get_value(gpio_bl_ctrl);
		if (val) {
			prev_tune_level = 0;
			gpio_direction_output(gpio_bl_ctrl, 0);
			mdelay(3);
			printk(KERN_INFO "LCD Baklight init in boot time on kernel\n");
		}
	}

	if (!prev_tune_level) {
		gpio_direction_output(gpio_bl_ctrl, 1);
		/* Ts should be more than 3msec */
		mdelay(3);
		prev_tune_level = MAX_BRIGHTNESS_IN_BLU;
	}

	pulse = (tune_level - prev_tune_level + MAX_BRIGHTNESS_IN_BLU)
		% MAX_BRIGHTNESS_IN_BLU;

	for (; pulse > 0; pulse--) {
		gpio_direction_output(gpio_bl_ctrl, 0);
		udelay(20);
		gpio_direction_output(gpio_bl_ctrl, 1);
		udelay(20);
	}

	mdelay(1);
	if (!gpio_get_value(gpio_bl_pwm_en)) {
		gpio_direction_output(gpio_bl_pwm_en, 1);
		if (!gpio_get_value(gpio_bl_pwm_en))
			printk(KERN_ERR "%s, error : gpio_bl_pwm_en is %s\n",
					__func__, gpio_get_value(gpio_bl_pwm_en) ? "HIGH" : "LOW");
		udelay(10);
	}

	prev_tune_level = tune_level;
out:
	spin_unlock(&bl_ctrl_lock);
	return 0;
}

void backlight_set_brightness(int level)
{
	if (!g_bl_info) {
		printk("%s, g_bl_info is null\n", __func__);
		return;
	}
	ktd_backlight_set_brightness(g_bl_info, level);

	return;
}

static int ktd_backlight_update_status(struct backlight_device *bl)
{
	struct device *dev = bl->dev.parent;
	struct ktd_bl_info *bl_info;
	int brightness = bl->props.brightness;
	if (!dev) {
		printk("%s, dev is null\n");
		return 0;
	}

	bl_info = (struct ktd_bl_info *)dev->platform_data;
	if (!bl_info) {
		printk("%s, bl_info is null\n");
		return 0;
	}

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	wakeup_brightness = brightness;
	if (!bl_enable && !current_brightness) {
		printk(KERN_INFO "[Backlight] no need to set backlight ---\n");
	} else {
		ktd_backlight_set_brightness(bl_info, brightness);
	}

	return 0;
}

static int ktd_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
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
	struct ktd_bl_info *bl_info =
		(struct ktd_bl_info *)pdev->dev.platform_data;

	printk("%s +\n",__FUNCTION__);

	if (!pdev) {
		printk("%s, warning : pdev is null\n", __func__);
		return 0;
	}

	if (!bl_info) {
		printk("%s, warning : platform_data is null\n", __func__);
		return 0;
	}

	g_bl_info = bl_info;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = bl_info->max_brightness;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register(BACKLIGHT_DEV_NAME, &pdev->dev, NULL,
			&ktd_backlight_ops, &props);

	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		return PTR_ERR(bl);
	}
	bl->props.max_brightness = bl_info->max_brightness;
	bl->props.brightness = bl_info->def_brightness;
	prev_tune_level =
		find_tune_level(bl_info->brt_table,
				bl_info->sz_table,
				bl_info->def_brightness);

	platform_set_drvdata(pdev, bl);

	if (gpio_request(bl_info->gpio_bl_ctrl,"BL_CTRL")) {
		printk(KERN_ERR "Request GPIO failed,""gpio: %d \n", bl_info->gpio_bl_ctrl);
	}

	if (gpio_request(bl_info->gpio_bl_pwm_en,"BL_PWM_EN")) {
		printk(KERN_ERR "Request GPIO failed,""gpio: %d \n", bl_info->gpio_bl_pwm_en);
	}
	bl_enable = true;
	backlight_update_status(bl);
	printk("%s -\n",__FUNCTION__);
	return 0;
}

static int ktd_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct ktd_bl_info *bl_info =
		(struct ktd_bl_info *)bl->dev.platform_data;

	if (unlikely(!bl_info)) {
		printk("%s, warning : platform_data is null\n", __func__);
		return 0;
	}

	backlight_device_unregister(bl);
	gpio_direction_output(bl_info->gpio_bl_ctrl, 0);
	mdelay(3);
	gpio_direction_output(bl_info->gpio_bl_pwm_en, 0);

	return 0;
}

void ktd_backlight_shutdown(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct ktd_bl_info *bl_info =
		(struct ktd_bl_info *)bl->dev.platform_data;

	if (unlikely(!bl_info)) {
		printk("%s, warning : platform_data is null\n", __func__);
		return;
	}

	printk("%s +\n",__FUNCTION__);
	gpio_direction_output(bl_info->gpio_bl_ctrl, 0);
	mdelay(5);
	gpio_direction_output(bl_info->gpio_bl_pwm_en, 0);
	printk("%s -\n",__FUNCTION__);
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
