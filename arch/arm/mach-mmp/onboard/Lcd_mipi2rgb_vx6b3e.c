/*
 * VX6B3E MIPI to RGB Bridge Chip
 *
 *
 * Copyright (C) 2013, Samsung Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>

#include <mach/cputype.h>
#include <mach/mfp-pxa1088-cs05.h>
#include <mach/regs-mpmu.h>
#include <mach/pxa988.h>
#include <mach/pxa168fb.h>
#include <mach/Lcd_mipi2rgb_vx6b3e.h>
#include "lcd_hx8369_param.h"

struct pxa168fb_info *fbi_global = NULL;
struct Vx6b3e_bridge_info *g_vx6b3e = NULL;

int hx8369_backlight_updata(void);

int vx6b3e_brige_init_with_i2c(void)
{
	vx6b3e_i2c_write32(0x700, 0x5C900040);
	vx6b3e_i2c_write32(0x704, 0x1F026E);
	vx6b3e_i2c_write32(0x70C, 0x00004604);
	vx6b3e_i2c_write32(0x710, /*0x54D004B vee off*/0x07CD0017);
	vx6b3e_i2c_write32(0x714, 0x0);

	vx6b3e_i2c_write32(0x718, 0x00000102);
	vx6b3e_i2c_write32(0x71C, 0xA8000B);
	vx6b3e_i2c_write32(0x720, 0x0);

	vx6b3e_i2c_write32(0x154, 0x00000000);
	vx6b3e_i2c_write32(0x154, 0x80000000);
	mdelay(1); /* For pll locking */
	vx6b3e_i2c_write32(0x700, 0x5C900040);
	vx6b3e_i2c_write32(0x70C, 0x00007FA6/*0x5646*/);
	vx6b3e_i2c_write32(0x718, 0x00000002);


	vx6b3e_i2c_write32(0x154, 0x00000000);	
	vx6b3e_i2c_write32(0x154, 0x80000000);
	mdelay(1); /* For pll locking */

	vx6b3e_i2c_write32(0x120, 0x5);
	vx6b3e_i2c_write32(0x124, 0x45901E0);
	vx6b3e_i2c_write32(0x128, 0x102809);
	vx6b3e_i2c_write32(0x12C, 0x30);
	vx6b3e_i2c_write32(0x130, 0x3C70);
	vx6b3e_i2c_write32(0x134, 0x00000005);
	vx6b3e_i2c_write32(0x138, 0xFF8000);
	vx6b3e_i2c_write32(0x13C, 0x0);


	vx6b3e_i2c_write32(0x140, 0x10000);
	vx6b3e_i2c_write32(0x20C, 0x22);
	vx6b3e_i2c_write32(0x21C, 0x514/*0x514 100us to 0 us*/);
	vx6b3e_i2c_write32(0x224, 0x0);
	vx6b3e_i2c_write32(0x228, 0x50000);
	vx6b3e_i2c_write32(0x22C, 0xFF04);
	vx6b3e_i2c_write32(0x230, 0x1);
	vx6b3e_i2c_write32(0x234, 0xCA033E10);
	vx6b3e_i2c_write32(0x238, 0x00000060);
	vx6b3e_i2c_write32(0x23C, 0x82E86030);
	vx6b3e_i2c_write32(0x244, 0x001E0285);
	vx6b3e_i2c_write32(0x258, 0x60011);

	vx6b3e_i2c_write32(0x158, 0x0);
	vx6b3e_i2c_write32(0x158, 0x1);
	mdelay(1); /* For pll locking */

	vx6b3e_i2c_write32(0x308, 0xffffffff);
	vx6b3e_i2c_write32(0x30C, 0x2202);
	vx6b3e_i2c_write32(0x310, 0xffffff);
	vx6b3e_i2c_write32(0x314, 0x1ffff);
	vx6b3e_i2c_write32(0x318, 0x17);
	vx6b3e_i2c_write32(0x31C, 0xff);	
	vx6b3e_i2c_write32(0x320, 0x32001E0);
	vx6b3e_i2c_write32(0x328, 0x22);
	vx6b3e_i2c_write32(0x32C, 0x32);
	vx6b3e_i2c_write32(0x330, 0x96);
	vx6b3e_i2c_write32(0x334, 0x331);
	vx6b3e_i2c_write32(0x338, 0x6);
	vx6b3e_i2c_write32(0x33C, 0x9);
	vx6b3e_i2c_write32(0x340, 0xA);
	vx6b3e_i2c_write32(0x344, 0x12);
	vx6b3e_i2c_write32(0x350, 0x514);
	vx6b3e_i2c_write32(0x354, 0x4);
	vx6b3e_i2c_write32(0x358, 0x3);
	vx6b3e_i2c_write32(0x35C, 0x0);
	vx6b3e_i2c_write32(0x364, 0x26000E);
	vx6b3e_i2c_write32(0x368, 0x4);
	vx6b3e_i2c_write32(0x36C, 0x07040700);
	vx6b3e_i2c_write32(0x370, 0x06050F01);
	vx6b3e_i2c_write32(0x374, 0x1);
	vx6b3e_i2c_write32(0x378, 0xC2033810);
	vx6b3e_i2c_write32(0x37C, 0x00000060);
	vx6b3e_i2c_write32(0x380, 0x82E86030);
	vx6b3e_i2c_write32(0x384, 0x28614088);
	vx6b3e_i2c_write32(0x388, 0x00110285);
	vx6b3e_i2c_write32(0x38C, 0x10600008);
	vx6b3e_i2c_write32(0x394, 0x400882A8);
	vx6b3e_i2c_write32(0x608, 0x50F);
	vx6b3e_i2c_write32(0x154, 0x00000000);
	vx6b3e_i2c_write32(0x154, 0x80000000);
	mdelay(1); /* For pll locking */

	vx6b3e_i2c_write32(0x300, 0x1); 
	mdelay(1); /* For pll locking */

	vx6b3e_i2c_release(); /*Internal path to MIPI*/

	return 0;
}


int vx6b3e_brige_init_with_mipi(struct pxa168fb_info *fbi)
{
	vx6b3e_mipi_write(fbi,0x700, 0x34000040,4);
	vx6b3e_mipi_write(fbi,0x704, 0x1F0023,4);
	vx6b3e_mipi_write(fbi,0x70C, 0x00004604,4);
	vx6b3e_mipi_write(fbi,0x710, /*0x54D004B vee off*/0x004D04B7,4);
	vx6b3e_mipi_write(fbi,0x714, 0x0,4);

	vx6b3e_mipi_write(fbi,0x718, 0x00000102,4);
	vx6b3e_mipi_write(fbi,0x71C, 0xA8000B,4);
	vx6b3e_mipi_write(fbi,0x720, 0x0,4);

	vx6b3e_mipi_write(fbi,0x154, 0x00000000,4);
	vx6b3e_mipi_write(fbi,0x154, 0x80000000,4);
	mdelay(1); /* For pll locking */
	vx6b3e_mipi_write(fbi,0x700, 0x34000040,4);
	vx6b3e_mipi_write(fbi,0x70C, 0x00007FB6/*0x5646*/,4);
	vx6b3e_mipi_write(fbi,0x718, 0x00000002,4);


	vx6b3e_mipi_write(fbi,0x154, 0x00000000,4);	
	vx6b3e_mipi_write(fbi,0x154, 0x80000000,4);
	mdelay(1); /* For pll locking */

	vx6b3e_mipi_write(fbi,0x120, 0x5,4);
	vx6b3e_mipi_write(fbi,0x124, 0x41901E0,4);
	vx6b3e_mipi_write(fbi,0x128, 0x102809,4);
	vx6b3e_mipi_write(fbi,0x12C, 0x30,4);
	vx6b3e_mipi_write(fbi,0x130, 0x3C70,4);
	vx6b3e_mipi_write(fbi,0x134, 0x00000005,4);
	vx6b3e_mipi_write(fbi,0x138, 0xFF8000,4);
	vx6b3e_mipi_write(fbi,0x13C, 0x0,4);


	vx6b3e_mipi_write(fbi,0x140, 0x10000,4);
	/*Add for power consumtion*/
	vx6b3e_mipi_write(fbi,0x174, /*0xff vee off*/0xff,4);
	/*end*/

	vx6b3e_mipi_write(fbi,0x20C, 0x22,4);
	vx6b3e_mipi_write(fbi,0x21C, 0x514,4);
	vx6b3e_mipi_write(fbi,0x224, 0x0,4);
	vx6b3e_mipi_write(fbi,0x228, 0x50000,4);
	vx6b3e_mipi_write(fbi,0x22C, 0xFF04,4);
	vx6b3e_mipi_write(fbi,0x230, 0x1,4);
	vx6b3e_mipi_write(fbi,0x234, 0xCA033E10,4);
	vx6b3e_mipi_write(fbi,0x238, 0x00000060,4);
	vx6b3e_mipi_write(fbi,0x23C, 0x82E86030,4);
	vx6b3e_mipi_write(fbi,0x244, 0x001E0285,4);
	vx6b3e_mipi_write(fbi,0x258, 0x60010,4);

	vx6b3e_mipi_write(fbi,0x158, 0x0,4);
	vx6b3e_mipi_write(fbi,0x158, 0x1,4);
	mdelay(1); /* For pll locking */

	vx6b3e_mipi_write(fbi,0x308, 0xffffffff,4);
	vx6b3e_mipi_write(fbi,0x30C, 0x2202,4);
	vx6b3e_mipi_write(fbi,0x310, 0xffffff,4);
	vx6b3e_mipi_write(fbi,0x314, 0x1ffff,4);
	vx6b3e_mipi_write(fbi,0x318, 0x17,4);
	vx6b3e_mipi_write(fbi,0x320, 0x32001E0,4);
	vx6b3e_mipi_write(fbi,0x328, 0x22,4);
	vx6b3e_mipi_write(fbi,0x32C, 0x62,4);
	vx6b3e_mipi_write(fbi,0x330, 0x12C,4);
	vx6b3e_mipi_write(fbi,0x334, 0x32F,4);
	vx6b3e_mipi_write(fbi,0x338, 0x6,4);
	vx6b3e_mipi_write(fbi,0x33C, 0x9,4);
	vx6b3e_mipi_write(fbi,0x340, 0xA,4);
	vx6b3e_mipi_write(fbi,0x344, 0x15,4);
	vx6b3e_mipi_write(fbi,0x350, 0x514,4);
	vx6b3e_mipi_write(fbi,0x354, 0x4,4);
	vx6b3e_mipi_write(fbi,0x358, 0x3,4);
	vx6b3e_mipi_write(fbi,0x35C, 0x0,4);
	vx6b3e_mipi_write(fbi,0x364, 0x2C000F,4);
	vx6b3e_mipi_write(fbi,0x368, 0x5,4);
	vx6b3e_mipi_write(fbi,0x36C, 0x08050800,4);
	vx6b3e_mipi_write(fbi,0x370, 0x07051201,4);
	vx6b3e_mipi_write(fbi,0x374, 0x1,4);
	vx6b3e_mipi_write(fbi,0x378, 0xD20B3810,4);
	vx6b3e_mipi_write(fbi,0x37C, 0x00000060,4);
	vx6b3e_mipi_write(fbi,0x380, 0x82E86030,4);
	vx6b3e_mipi_write(fbi,0x384, 0x28614088,4);
	vx6b3e_mipi_write(fbi,0x388, 0x00110285,4);
	vx6b3e_mipi_write(fbi,0x38C, 0x10600008,4);
	vx6b3e_mipi_write(fbi,0x394, 0x400882A8,4);
	vx6b3e_mipi_write(fbi,0x608, 0x50F,4);
	vx6b3e_mipi_write(fbi,0x154, 0x00000000,4);
	vx6b3e_mipi_write(fbi,0x154, 0x80000000,4);
	mdelay(1); /* For pll locking */

	vx6b3e_mipi_write(fbi,0x300, 0x1,4);	
	mdelay(1); /* For pll locking */

	return 0;
}

int vx6b3e_brige_init(struct pxa168fb_info *fbi)
{

	printk("VX6B3E ...START.....\n");

#if defined( VX6B3E_CONTROL_BY_I2C )

	vx6b3e_brige_init_with_i2c();

#else
	vx6b3e_brige_init_with_mipi(fbi);
#endif

	printk("VX6B3E ..END.....\n");

	return 0;
}

int vx6b3e_reset(void)
{

	printk("vx6b3e_reset\n");

	if (gpio_request(VX6B3E_RESET, "lcd reset gpio")) {
		pr_err("gpio %d request failed\n", VX6B3E_RESET);
		goto regu_lcd_vdd;
	}

	gpio_direction_output(VX6B3E_RESET, 0);
	udelay(20);
	gpio_direction_output(VX6B3E_RESET, 1);
	udelay(20);

	gpio_free(VX6B3E_RESET);

	return 0;

regu_lcd_vdd:
	gpio_free(VX6B3E_RESET);

	return -EIO;
}

int vx6b3e_power(struct pxa168fb_info *fbi, int on)
{

	printk("vx6b3e_power =[%d]\n",on);

	if (gpio_request(VX6B3E_PWR_EN_VXE_1P2, "lcd pwr 1p2")) {
		pr_err("gpio %d request failed\n", VX6B3E_PWR_EN_VXE_1P2);
		goto regu_lcd_vdd;
	}
	if (gpio_request(VX6B3E_PWR_EN_VXE_1P8, "lcd pwr 1p8")) {
		pr_err("gpio %d request failed\n", VX6B3E_PWR_EN_VXE_1P8);
		goto regu_lcd_vdd;
	}

	if (gpio_request(GPIO_LCD_VXMIPISWITCH_EN, "mipi switch 1")) {
		pr_err("gpio %d request failed\n", GPIO_LCD_VXMIPISWITCH_EN);
		goto regu_lcd_vdd;
	}
	if (gpio_request(GPIO_LCD_SWITCH_OE, "mipi switch oe 1")) {
		pr_err("gpio %d request failed\n", GPIO_LCD_SWITCH_OE);
		goto regu_lcd_vdd;
	}

	if (gpio_request(GPIO_LCD_VXMIPISWITCH2_EN, "mipi switch 2")) {
		pr_err("gpio %d request failed\n", GPIO_LCD_VXMIPISWITCH2_EN);
		goto regu_lcd_vdd;
	}
	if (gpio_request(GPIO_LCD_SWITCH2_OE, "mipi switch oe 2")) {
		pr_err("gpio %d request failed\n", GPIO_LCD_SWITCH2_OE);
		goto regu_lcd_vdd;
	}

	if (gpio_request(GPIO_13M_EN, "sysclk 13m en")) {
		pr_err("gpio %d request failed\n", GPIO_13M_EN);
		goto regu_lcd_vdd;
	}

	if (on) {

		gpio_direction_output(GPIO_LCD_VXMIPISWITCH_EN, 0);
		gpio_direction_output(GPIO_LCD_SWITCH_OE, 0);

		gpio_direction_output(GPIO_LCD_VXMIPISWITCH2_EN, 1);
		gpio_direction_output(GPIO_LCD_SWITCH2_OE, 0);

		gpio_direction_output(VX6B3E_PWR_EN_VXE_1P2, 0);
		gpio_direction_output(VX6B3E_PWR_EN_VXE_1P8, 0);
		mdelay(100);

		gpio_direction_output(VX6B3E_PWR_EN_VXE_1P2, 1);
		mdelay(1);
		gpio_direction_output(VX6B3E_PWR_EN_VXE_1P8, 1);
		mdelay(1);
		
	/*VX6B3E's system clk enable*/
		//mpmu_vcxoen();
		gpio_direction_output(GPIO_13M_EN, 1);
		mdelay(20);

		vx6b3e_reset();

		mdelay(1);

	} else {
		gpio_direction_output(VX6B3E_PWR_EN_VXE_1P8, 0);	
		gpio_direction_output(VX6B3E_PWR_EN_VXE_1P2, 0);
		gpio_direction_output(GPIO_13M_EN, 0);		
	}

	gpio_free(VX6B3E_PWR_EN_VXE_1P2);
	gpio_free(VX6B3E_PWR_EN_VXE_1P8);
	gpio_free(GPIO_LCD_VXMIPISWITCH_EN);
	gpio_free(GPIO_LCD_SWITCH_OE);
	gpio_free(GPIO_LCD_VXMIPISWITCH2_EN);
	gpio_free(GPIO_LCD_SWITCH2_OE);
	gpio_free(GPIO_13M_EN);

	return 0;

regu_lcd_vdd:
	
	gpio_free(VX6B3E_PWR_EN_VXE_1P2);
	gpio_free(VX6B3E_PWR_EN_VXE_1P8);
	gpio_free(GPIO_LCD_VXMIPISWITCH_EN);
	gpio_free(GPIO_LCD_SWITCH_OE);
	gpio_free(GPIO_LCD_VXMIPISWITCH2_EN);
	gpio_free(GPIO_LCD_SWITCH2_OE);
	gpio_free(GPIO_13M_EN);

	return -EIO;
}


static int hx8369_cs05_init(struct pxa168fb_info *fbi)
{
	printk("%s\n", __func__);

	vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video1),hx8369b_55BC90_video1);		
	vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video2),hx8369b_55BC90_video2);
	vx6b3e_i2cTomipi_write(DTYPE_DCS_WRITE1, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video3),hx8369b_55BC90_video3);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video4_vx1),hx8369b_55BC90_video4_vx1);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video4_vx2),hx8369b_55BC90_video4_vx2);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video5),hx8369b_55BC90_video5);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
		sizeof(hx8369b_blackscreen),hx8369b_blackscreen);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video6),hx8369b_55BC90_video6);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_WRITE2, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video7),hx8369b_55BC90_video7);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_WRITE2, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video8),hx8369b_55BC90_video8);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_WRITE2, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video9),hx8369b_55BC90_video9);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_WRITE2, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video10),hx8369b_55BC90_video10);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video11),hx8369b_55BC90_video11);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_WRITE2, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video12),hx8369b_55BC90_video12);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video13),hx8369b_55BC90_video13);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video14),hx8369b_55BC90_video14);
	vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
		sizeof(hx8369b_55BC90_video15),hx8369b_55BC90_video15);

	return 0;
}

static int hx8369_cs05_enable(struct pxa168fb_info *fbi)
{
	printk("%s\n", __func__);

	/*Sleep out*/
	vx6b3e_i2cTomipi_write(DTYPE_DCS_WRITE, QL_VX_LCD_VC,\
		sizeof(sleep_out),sleep_out);
	msleep(HX8369B_SLEEP_OUT_DELAY);

	vx6b3e_i2cTomipi_write(DTYPE_DCS_WRITE, QL_VX_LCD_VC,\
		sizeof(display_on),display_on);
	msleep(HX8369B_DISP_OFF_DELAY);

	return 0;
}

static int hx8369_cs05_disable(struct pxa168fb_info *fbi)
{
	printk("%s\n", __func__);

	/*
	 * Sperated lcd off routine multi tasking issue
	 * becasue of share of cpu0 after suspend.
	 */

	/*sleep in*/
	vx6b3e_i2cTomipi_write(DTYPE_DCS_WRITE, QL_VX_LCD_VC,\
		sizeof(display_off),display_off);
	msleep(HX8369B_DISP_OFF_DELAY);

	vx6b3e_i2cTomipi_write(DTYPE_DCS_WRITE, QL_VX_LCD_VC,\
		sizeof(sleep_in),sleep_in);
	msleep(HX8369B_SLEEP_IN_DELAY);

	return 0;
}


static ssize_t Negative_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct Vx6b3e_bridge_info *pvx6b3e = g_vx6b3e;

	ret = sprintf(buf,"Negative_show : %d\n", pvx6b3e->negative);

	return ret;
}

static ssize_t Negative_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct Vx6b3e_bridge_info *pvx6b3e = g_vx6b3e;

	unsigned int value;
	int ret;
	ret = strict_strtoul(buf, 0, (unsigned long *)&value);
	printk("%s:value=%d\n", __func__, value);

	mutex_lock(&pvx6b3e->lock);
	if (value == 1) {
		pvx6b3e->negative = 1;
		pxa168_dsi_cmd_array_tx(fbi_global,
				hx8369b_video_display_mDNIe_negative_cmds,
				ARRAY_SIZE(hx8369b_video_display_mDNIe_negative_cmds));
	} else {
		pvx6b3e->negative = 0;
		pxa168_dsi_cmd_array_tx(fbi_global,
				&hx8369b_video_display_mDNIe_scenario_cmds[0], 1);

	}
	mutex_unlock(&pvx6b3e->lock);

	return size;
}

static struct device_attribute mdnie_attributes[] = {
	__ATTR(negative, 0644, Negative_store, Negative_show),
	__ATTR_NULL,
};

static ssize_t auto_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct Vx6b3e_bridge_info *pvx6b3e = g_vx6b3e;
	char *pos = buf;

	pos += sprintf(pos, "%d, %d, ", pvx6b3e->auto_brightness, pvx6b3e->cabc);

	return pos - buf;
}

static ssize_t auto_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct Vx6b3e_bridge_info *pvx6b3e = g_vx6b3e;
	int value = 0, rc = 0;

	if (fbi_global->active == 0)
		return size;

	rc = strict_strtoul(buf, (unsigned int)0, (unsigned long *)&value);
	
	if (rc < 0)
		return rc;
	else {
		mutex_lock(&pvx6b3e->lock);

		printk("auto_brightness[%d] -> [%d]\n",\
				pvx6b3e->auto_brightness, value);

		if (pvx6b3e->auto_brightness != value) {

			pvx6b3e->auto_brightness = value;
			pvx6b3e->cabc = (value) ? CABC_ON : CABC_OFF;			

			hx8369_backlight_updata();
		}

		mutex_unlock(&pvx6b3e->lock);
	}
	return size;
}

static DEVICE_ATTR(auto_brightness, 0644, auto_brightness_show, auto_brightness_store);

static ssize_t lcd_panelName_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "HX8369B");
}

static ssize_t lcd_MTP_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int panel_id = get_panel_id();

	((unsigned char*)buf)[0] = (panel_id >> 16) & 0xFF;
	((unsigned char*)buf)[1] = (panel_id >> 8) & 0xFF;
	((unsigned char*)buf)[2] = panel_id & 0xFF;

	printk("ldi mtpdata: %x %x %x\n", buf[0], buf[1], buf[2]);

	return 3;
}

static ssize_t lcd_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int panel_id = get_panel_id();

	return sprintf(buf, "INH_%x%x%x",
			(panel_id >> 16) & 0xFF,
			(panel_id >> 8) & 0xFF,
			panel_id  & 0xFF);
}

static DEVICE_ATTR(lcd_MTP, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, lcd_MTP_show, NULL);
static DEVICE_ATTR(lcd_panelName, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, lcd_panelName_show, NULL);
static DEVICE_ATTR(lcd_type, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, lcd_type_show, NULL);

int hx8369_backlight_updata(void)
{
	if (!fbi_global->output_on) {

		vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
			sizeof(hx8369b_dtc_B9h),hx8369b_dtc_B9h);
		vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
			sizeof(hx8369b_dtc_C9h),hx8369b_dtc_C9h);
		vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
			sizeof(hx8369b_backlight_51h),hx8369b_backlight_51h);
		vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
			sizeof(hx8369b_dtc_53h),hx8369b_dtc_53h);
		vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
			sizeof(hx8369b_dtc_55h),hx8369b_dtc_55h);

		msleep(5);

	} else {
		vx6b3e_i2cTomipi_write(DTYPE_GEN_LWRITE, QL_VX_LCD_VC,\
			sizeof(hx8369b_backlight_51h),hx8369b_backlight_51h);
	}

	printk("current backlight=[%d]\n",hx8369b_backlight_51h[1]);

	return 0;
}

int hx8369_int_pwm_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

	hx8369b_backlight_51h[1] = brightness;

	if (!fbi_global->active)
		return 0;

	if (max > 255)
		max = 255;

	hx8369_backlight_updata();
	
	return 0;
}

int hx8369_int_pwm_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops hx8369_int_pwm_ops = {
	.update_status	= hx8369_int_pwm_update_status,
	.get_brightness	= hx8369_int_pwm_get_brightness,
};

static int hx8369_cs05_probe(struct pxa168fb_info *fbi)
{
	struct Vx6b3e_bridge_info *vx6b3eInfo;
	int ret = 0;

	fbi_global = fbi;

	printk("%s, probe hx8369b\n", __func__);

	/*For Bridge alloc*/
	vx6b3eInfo = kzalloc(sizeof(struct Vx6b3e_bridge_info), GFP_KERNEL);

	if (!vx6b3eInfo) {
		pr_err("failed to allocate vx6b3eInfo\n");
		ret = -ENOMEM;
		goto error1;
	}

	/*For Mdnie class*/
	vx6b3eInfo->Mdnie = class_create(THIS_MODULE, "mdnie");
	if (IS_ERR_OR_NULL(vx6b3eInfo->Mdnie)) {
		printk("failed to create mdnie class\n");
		ret = -EINVAL;
		goto error1;		
	}
	vx6b3eInfo->Mdnie->dev_attrs = mdnie_attributes;

	vx6b3eInfo->dev_mdnie = device_create( vx6b3eInfo->Mdnie, NULL, 0, &vx6b3eInfo, "mdnie");

	if (IS_ERR_OR_NULL(vx6b3eInfo->dev_mdnie)) 
	{
		printk("Failed to create Mdnie class!\n");
		ret = -EINVAL;
		goto error1;
	}

	/*For lcd class*/
	vx6b3eInfo->lcd = lcd_device_register("panel", NULL, NULL, NULL);
	if (IS_ERR_OR_NULL(vx6b3eInfo->lcd)) 
	{
		printk("Failed to create lcd class!\n");
		ret = -EINVAL;
		goto error1;
	}

	if (device_create_file(&vx6b3eInfo->lcd->dev, &dev_attr_lcd_panelName) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_lcd_panelName.attr.name);
	if (device_create_file(&vx6b3eInfo->lcd->dev, &dev_attr_lcd_MTP) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_lcd_MTP.attr.name);
	if (device_create_file(&vx6b3eInfo->lcd->dev, &dev_attr_lcd_type) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_lcd_type.attr.name);

	/*For backlight*/
	vx6b3eInfo->bd = backlight_device_register("panel", vx6b3eInfo->dev_bd, vx6b3eInfo,
					&hx8369_int_pwm_ops,NULL);

	if (device_create_file(&vx6b3eInfo->bd->dev, &dev_attr_auto_brightness) < 0)
		printk("Failed to create auto_brightness\n");

	vx6b3eInfo->bd->props.max_brightness = MAX_BRIGHTNESS_LEVEL;
	vx6b3eInfo->bd->props.brightness = DEFAULT_BRIGHTNESS;
	vx6b3eInfo->bd->props.type = BACKLIGHT_RAW;
	vx6b3eInfo->cabc = CABC_OFF;
	vx6b3eInfo->auto_brightness = false;
	vx6b3eInfo->negative = false;
	mutex_init(&vx6b3eInfo->lock);

	g_vx6b3e = vx6b3eInfo;

	return ret;

error1:
	kfree(vx6b3eInfo);	
	return ret;

}

struct pxa168fb_mipi_lcd_driver hx8369_cs05_lcd_driver = {
	.probe		= hx8369_cs05_probe,
	.init		= hx8369_cs05_init,
	.disable	= hx8369_cs05_disable,
	.enable		= hx8369_cs05_enable,
};
