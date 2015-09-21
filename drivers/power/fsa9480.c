/*
 * For TI 6111 musb ic
 *
 * Copyright (C) 2009 Samsung Electronics
 * Wonguk Jeong <wonguk.jeong@samsung.com>
 * Minkyu Kang <mk7.kang@samsung.com>
 *
 * Modified by Sumeet Pawnikar <sumeet.p@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <plat/microusbic.h>
#include <plat/mfp.h>

#include <plat/vbus.h>

#include <linux/mfd/88pm80x.h>
#include <linux/wakelock.h>

#if defined(CONFIG_SPA)
#include <mach/spa.h>
static void (*spa_external_event)(int, int) = NULL;
extern struct class *sec_class;
#endif
#include <mach/fsa9480.h>
#include <mach/irqs.h>

#include <linux/delay.h>
#include <linux/gpio-pxa.h>


/*
#include "mv/mvUsbDevApi.h"
#include "mv/mvUsbCh9.h"
#include <mach/hardware.h>
#include <plat/vbus.h>
#include <plat/pxausb_comp.h>
#include <plat/pxa3xx_otg.h>
#include <plat/pxa_u2o.h>
#include "mvUsb.h"
*/
#ifdef CONFIG_ONECHIP_DUAL_MODEM
#define GPIO_UART_SEL		SM_CSN1_GPIO_104
#endif

/* FSA9480 I2C registers */
#define FSA9480_REG_DEVID              0x01
#define FSA9480_REG_CTRL               0x02
#define FSA9480_REG_INT1               0x03
#define FSA9480_REG_INT2               0x04
#define FSA9480_REG_INT1_MASK          0x05
#define FSA9480_REG_INT2_MASK          0x06
#define FSA9480_REG_ADC                        0x07
#define FSA9480_REG_TIMING1            0x08
#define FSA9480_REG_TIMING2            0x09
#define FSA9480_REG_DEV_T1             0x0a
#define FSA9480_REG_DEV_T2             0x0b
#define FSA9480_REG_BTN1               0x0c
#define FSA9480_REG_BTN2               0x0d
#define FSA9480_REG_CK                 0x0e
#define FSA9480_REG_CK_INT1            0x0f
#define FSA9480_REG_CK_INT2            0x10
#define FSA9480_REG_CK_INTMASK1                0x11
#define FSA9480_REG_CK_INTMASK2                0x12
#define FSA9480_REG_MANSW1             0x13
#define FSA9480_REG_MANSW2             0x14

/* MANSW1 */
#define VAUDIO                 0x90
#define UART                   0x6c
#define AUDIO                  0x48
#define DHOST                  0x24
#define AUTO                   0x0

/*FSA9485 MANSW1*/
#define VAUDIO_9485            0x93
#define AUDIO_9485             0x4B
#define DHOST_9485             0x27

/* MANSW2 */
#define MANSW2_JIG		(1 << 2)

/* Control */
#define SWITCH_OPEN            (1 << 4)
#define RAW_DATA               (1 << 3)
#define MANUAL_SWITCH          (1 << 2)
#define WAIT                   (1 << 1)
#define INT_MASK               (1 << 0)
#define CTRL_MASK              (SWITCH_OPEN | RAW_DATA | MANUAL_SWITCH | \
                                       WAIT | INT_MASK)
/* Device Type 1*/
#define DEV_USB_OTG            (1 << 7)
#define DEV_DEDICATED_CHG      (1 << 6)
#define DEV_USB_CHG            (1 << 5)
#define DEV_CAR_KIT            (1 << 4)
#define DEV_UART               (1 << 3)
#define DEV_USB                        (1 << 2)
#define DEV_AUDIO_2            (1 << 1)
#define DEV_AUDIO_1            (1 << 0)

#define FSA9480_DEV_T1_HOST_MASK               (DEV_USB_OTG)
#define FSA9480_DEV_T1_USB_MASK                (DEV_USB)
#define FSA9480_DEV_T1_UART_MASK       (DEV_UART)
#define FSA9480_DEV_T1_CHARGER_MASK    (DEV_DEDICATED_CHG | DEV_USB_CHG | DEV_CAR_KIT)
#define FSA9480_DEV_T1_AUDIO_MASK    (DEV_AUDIO_1 | DEV_AUDIO_2)

/* Device Type 2*/
#define DEV_AV                 (1 << 6)
#define DEV_TTY                        (1 << 5)
#define DEV_PPD                        (1 << 4)
#define DEV_JIG_UART_OFF       (1 << 3)
#define DEV_JIG_UART_ON                (1 << 2)
#define DEV_JIG_USB_OFF                (1 << 1)
#define DEV_JIG_USB_ON         (1 << 0)

#define FSA9480_DEV_T2_USB_MASK                (DEV_JIG_USB_OFF | DEV_JIG_USB_ON)
#define FSA9480_DEV_T2_UART_MASK       (DEV_JIG_UART_OFF | DEV_JIG_UART_ON)

#define FSA9480_DEV_T2_JIG_MASK                (DEV_JIG_USB_OFF | DEV_JIG_USB_ON | \
                                       DEV_JIG_UART_OFF | DEV_JIG_UART_ON)

#define DEV_MHL                 (DEV_AV)
#define FSA9480_DEV_T2_MHL_MASK         (DEV_MHL)

#ifdef CONFIG_ONECHIP_DUAL_MODEM
#define SWITCH_UART_AP	1
#define SWITCH_UART_CP	2
#define SWITCH_USB_AP	3
#define SWITCH_USB_CP	4
#endif

struct fsa9480_usbsw {
       struct i2c_client               *client;
       struct fsa9480_platform_data    *pdata;
       struct work_struct              work;
       int                             dev1;
       int                             dev2;
       int                             mansw;
#ifdef CONFIG_ONECHIP_DUAL_MODEM
	   int				uart_path;
#endif
	   u8                              id;
};

static struct fsa9480_usbsw *chip;

/*Totoro*/
struct pm860x_vbus_info {
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	struct resource		*res;
	struct work_struct      wq;
	unsigned int		base;
	int			irq_status;
	int			irq_id;
	int			idpin;
	int			supply;
	int			work_data;
	int			work_mask;
};

//static struct wake_lock JIGConnect_idle_wake;
//static struct wake_lock JIGConnect_suspend_wake;

//static DEFINE_SPINLOCK(lock);

//#define VBUS_DETECT /*TotoroTD*/

#ifdef CONFIG_VIDEO_MHL_V1
/*for MHL cable insertion*/
static int isMHLconnected=0;
static int isDeskdockconnected=0;
#endif



static int isProbe=0;
static int fsa9480_is_ovp = 0;
static int dev_fsa880 = 0;

#ifdef CONFIG_ONECHIP_DUAL_MODEM
int switch_sel = 0;
int __init swsel_setup(char *str)
{
	switch_sel = str[0]-48;
	printk("[%s] switch_sel=%d\n", __func__, switch_sel);
	return 1;
}
__setup("swsel=", swsel_setup);
#endif
static ssize_t adc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct fsa9480_usbsw *usbsw = chip;
	u8 adc_value[] = "1C";
	u8 adc_fail = 0;
 
      if (usbsw->dev2 & FSA9480_DEV_T2_JIG_MASK) 
	{
		printk("adc_show JIG_UART_OFF\n");
		return sprintf(buf,"%s\n",adc_value);
      	}
	else
	{
		printk("adc_show no detect\n");
		return sprintf(buf,"%d\n",adc_fail);
	}
}

static DEVICE_ATTR(adc,S_IRUGO|S_IWUSR|S_IWGRP|S_IXOTH/*0665*/, adc_show, NULL);

static int fsa9480_write_reg(struct i2c_client *client,        u8 reg, u8 data)
{
       int ret = 0;
       u8 buf[2];
       struct i2c_msg msg[1];

       buf[0] = reg;
       buf[1] = data;

       msg[0].addr = client->addr;
       msg[0].flags = 0;
       msg[0].len = 2;
       msg[0].buf = buf;

	   printk("[FSA9480] fsa9480_write_reg   reg[0x%2x] data[0x%2x]\n",buf[0],buf[1]);

       ret = i2c_transfer(client->adapter, msg, 1);
       if (ret != 1) {
               printk("\n [FSA9480] i2c Write Failed (ret=%d) \n", ret);
               return -1;
       }

	//printk("[FSA9480] i2c Write success reg[0x%2x] data[0x%2x] ret[%d]\n", buf[0], buf[1], ret);
       
       return ret;
}

static int fsa9480_read_reg(struct i2c_client *client, u8 reg, u8 *data)
{
       int ret = 0;
       u8 buf[1];
       struct i2c_msg msg[2];

       buf[0] = reg;

        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = buf;

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 1;
        msg[1].buf = buf;
		
       ret = i2c_transfer(client->adapter, msg, 2);
       if (ret != 2) {
               printk("[FSA9480] i2c Read Failed reg:0x%x(ret=%d)\n", reg, ret);
               return -1;
       }
       *data = buf[0];

       printk("[FSA9480] i2c Read success reg:0x%x[0x%x]\n", reg, buf[0]);
       return 0;
}

static void fsa9480_read_adc_value(void)
{
	u8 adc=0;
    struct fsa9480_usbsw *usbsw = chip;
    struct i2c_client *client = usbsw->client;
	
	fsa9480_read_reg(client, FSA9480_REG_ADC, &adc);
	printk("[FSA9480] %s: adc is 0x%x\n",__func__,adc);
}
#ifdef CONFIG_VIDEO_MHL_V1
static void DisableFSA9480Interrupts(void)
{
       struct fsa9480_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
        printk ("DisableFSA9480Interrupts-2\n");

     fsa9480_write_reg(client, FSA9480_REG_INT1_MASK, 0xFF);
     fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x1F);
	 
} // DisableFSA9480Interrupts()

static void EnableFSA9480Interrupts(void)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;
	u8 intr, intr2;

	printk ("EnableFSA9480Interrupts\n");

     /*clear interrupts*/
     fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
     fsa9480_read_reg(client, FSA9480_REG_INT2, &intr2);
	 
     fsa9480_write_reg(client, FSA9480_REG_INT1_MASK, 0x00);
     fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x00);

} //EnableFSA9480Interrupts()
#endif

static void fsa9480_id_open(void)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	pr_alert("fsa9480 id_open\n");
	fsa9480_write_reg(client, 0x1B, 1);
}

void fsa9480_set_switch(const char *buf)
{
       struct fsa9480_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
       u8 value = 0;
       unsigned int path = 0;

       fsa9480_read_reg(client, FSA9480_REG_CTRL, &value);

       if (!strncmp(buf, "VAUDIO", 6)) {
	   	       if(usbsw->id == 0)
			   	path = VAUDIO_9485;
			   else
			   	path = VAUDIO;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "UART", 4)) {
               path = UART;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "AUDIO", 5)) {
               if(usbsw->id == 0)
			   	path = AUDIO_9485;
			   else
                path = AUDIO;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "DHOST", 5)) {
               path = DHOST;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "AUTO", 4)) {
               path = AUTO;
               value |= MANUAL_SWITCH;
       } else {
               printk(KERN_ERR "Wrong command\n");
               return;
       }

       usbsw->mansw = path;
       fsa9480_write_reg(client, FSA9480_REG_MANSW1, path);
       fsa9480_write_reg(client, FSA9480_REG_CTRL, value);
}
EXPORT_SYMBOL_GPL(fsa9480_set_switch);

#ifdef CONFIG_ONECHIP_DUAL_MODEM
//--------------------------------------------
/** @brief Switch between auto and manual mode
	@param on : (0:manual, 1:auto)
	@return 0 for sucess
*///-------------------------------------
int fsa9480_set_auto_mode(int on)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;
	u8 value;

	fsa9480_read_reg(client, FSA9480_REG_CTRL, &value);
	value &= ~MANUAL_SWITCH;
	value |= on << 2;

	fsa9480_write_reg(client, FSA9480_REG_CTRL, value);
}

int fsa9480_set_path(int path)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;
	u8 value;
	int gpio_uart_sel = mfp_to_gpio(MFP_PIN_GPIO104);

	fsa9480_read_reg(client, FSA9480_REG_MANSW1, &value);
	value &= ~(0x3F<<2);

	switch (path)	{
	case SWITCH_UART_AP:
		value |= UART;
		gpio_set_value(gpio_uart_sel, 0);
		pr_info("%s: UART_AP, GPIO_UART_SEL=%d\n", __func__, path, gpio_get_value(gpio_uart_sel));
		fsa9480_set_auto_mode(1);
		break;
	case SWITCH_UART_CP:
		value |= UART;
		gpio_set_value(gpio_uart_sel, 1);
		pr_info("%s: UART_CP, GPIO_UART_SEL=%d\n", __func__, path, gpio_get_value(gpio_uart_sel));
		fsa9480_set_auto_mode(0);
		break;
	case SWITCH_USB_AP:
		value |= DHOST;
		pr_info("%s: USB_AP\n", __func__);
		fsa9480_set_auto_mode(1);
		break;
	case SWITCH_USB_CP:
		value |= AUDIO;
		pr_info("%s: USB_CP\n", __func__);
		fsa9480_set_auto_mode(0);
		break;
	default:
		break;
	}

	fsa9480_write_reg(client, FSA9480_REG_MANSW1, value);

	return 1;
}
#endif

ssize_t fsa9480_get_switch(char *buf)
{
struct fsa9480_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
       u8 value;

       fsa9480_read_reg(client, FSA9480_REG_MANSW1, &value);

       if (value == VAUDIO)
               return sprintf(buf, "VAUDIO\n");
       else if (value == UART)
               return sprintf(buf, "UART\n");
       else if (value == AUDIO)
               return sprintf(buf, "AUDIO\n");
       else if (value == DHOST)
               return sprintf(buf, "DHOST\n");
       else if (value == AUTO)
               return sprintf(buf, "AUTO\n");
       else
               return sprintf(buf, "%x", value);
}
EXPORT_SYMBOL_GPL(fsa9480_get_switch);


#ifdef CONFIG_VIDEO_MHL_V1
void FSA9480_EnableIntrruptByMHL(bool _bDo)
{
	struct fsa9480_platform_data *pdata = chip->pdata;
	struct i2c_client *client = chip->client;
	char buf[16];

	if(true == _bDo)
	{
		fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1E);
		EnableFSA9480Interrupts();
	}
	else
	{
		DisableFSA9480Interrupts();
	}

	fsa9480_get_switch(buf);
	printk("[%s] fsa switch status = %s\n",__func__, buf);
}

/*MHL call this function to change VAUDIO path*/
void FSA9480_CheckAndHookAudioDock(void)
{
   struct fsa9480_platform_data *pdata = chip->pdata;
   struct i2c_client *client = chip->client;

   printk("[FSA9480] %s: FSA9485 VAUDIO\n",__func__);
   
   isMHLconnected = 0;
   isDeskdockconnected = 1;
   
   if (pdata->mhl_cb)
   	       pdata->mhl_cb(FSA9480_DETACHED);

   EnableFSA9480Interrupts();

   if(chip->id == 0)
	chip->mansw = VAUDIO_9485;
   else
	chip->mansw = VAUDIO;

   /*make ID change report*/
   fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x16);
   
   if(pdata->deskdock_cb)
           pdata->deskdock_cb(FSA9480_ATTACHED);   

}
EXPORT_SYBMOL_GPL(FSA9480_CheckAndHookAudioDock);
#endif


static ssize_t fsa9480_show_status(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
       struct fsa9480_usbsw *usbsw = dev_get_drvdata(dev);
       struct i2c_client *client = usbsw->client;
       u8 devid, ctrl, adc, dev1, dev2, intr;
       u8 intmask1, intmask2, time1, time2, mansw1;

       fsa9480_read_reg(client, FSA9480_REG_DEVID, &devid);
       fsa9480_read_reg(client, FSA9480_REG_CTRL, &ctrl);
       fsa9480_read_reg(client, FSA9480_REG_ADC, &adc);
       fsa9480_read_reg(client, FSA9480_REG_INT1_MASK, &intmask1);
       fsa9480_read_reg(client, FSA9480_REG_INT2_MASK, &intmask2);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &dev1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &dev2);
       fsa9480_read_reg(client, FSA9480_REG_TIMING1, &time1);
       fsa9480_read_reg(client, FSA9480_REG_TIMING2, &time2);
       fsa9480_read_reg(client, FSA9480_REG_MANSW1, &mansw1);

       fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
       intr &= 0xffff;

       return sprintf(buf, "Device ID(%02x), CTRL(%02x)\n"
                       "ADC(%02x), DEV_T1(%02x), DEV_T2(%02x)\n"
                       "INT(%04x), INTMASK(%02x, %02x)\n"
                       "TIMING(%02x, %02x), MANSW1(%02x)\n",
                       devid, ctrl, adc, dev1, dev2, intr,
                       intmask1, intmask2, time1, time2, mansw1);
}

static ssize_t fsa9480_show_manualsw(struct device *dev,
               struct device_attribute *attr, char *buf)
{
       return fsa9480_get_switch(buf);

}

static ssize_t fsa9480_set_manualsw(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
       fsa9480_set_switch(buf);
       return count;
}

static ssize_t fsa9480_set_syssleep(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;
	u8 value = 0;

	if (!strncmp(buf, "1", 1))
	{
		//wake_unlock(&JIGConnect_idle_wake);
		//wake_unlock(&JIGConnect_suspend_wake);

		fsa9480_read_reg(client, FSA9480_REG_CTRL, &value);
		value &= ~MANUAL_SWITCH;
		fsa9480_write_reg(client, FSA9480_REG_CTRL, value);
	
		fsa9480_read_reg(client, FSA9480_REG_MANSW2, &value);
		value &= ~MANSW2_JIG;
		fsa9480_write_reg(client, FSA9480_REG_MANSW2, value);
	}
	else
	{
		fsa9480_read_reg(client, FSA9480_REG_CTRL, &value);
		value |= MANUAL_SWITCH;
		fsa9480_write_reg(client, FSA9480_REG_CTRL, value);
	}
	return count;
}
extern ssize_t usb_state_show_attrs(struct device *dev, struct device_attribute *attr, char *buf);

#ifdef CONFIG_ONECHIP_DUAL_MODEM
static ssize_t uart_sel_show_attrs(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsa9480_usbsw *usbsw = chip;

	if (usbsw->uart_path == SWITCH_UART_AP)
		return sprintf(buf, "AP");
	else
		return sprintf(buf, "CP");
}

static ssize_t uart_sel_set_attrs(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	pr_info("func:%s buf:%s,count:%d\n", __func__, buf, count);

	if (!strncasecmp(buf, "AP", 2)) {
		usbsw->uart_path = SWITCH_UART_AP;
		pr_info("%s: AP_UART_MODE\n", __func__);
	}
#if defined(CONFIG_SWITCH_DUAL_MODEM)
	else if (!strncasecmp(buf, "CP2", 3)) {
		int ret = max77803_muic_set_uart_path_pass2
			(info, UART_PATH_CP);
		if (ret >= 0)
			info->muic_data->uart_path = UART_PATH_CP;
		else
			pr_err("%s: Change(CP2) fail!!"
						, __func__);
		if (gpio_is_valid(GPIO_UART_SEL))	{
			gpio_set_value(GPIO_UART_SEL, GPIO_LEVEL_HIGH);
			info->muic_data->uart_path = UART_PATH_CP_ESC;
			pr_info("%s: CP2 %d\n",
					__func__,
					gpio_get_value(GPIO_UART_SEL));
		} else	{
			pr_err("%s: Change GPIO failed",
					__func__);
		}
	}
#endif
	else if (!strncasecmp(buf, "CP", 2)) {
		usbsw->uart_path = SWITCH_UART_CP;
		pr_info("%s: CP_UART_MODE\n", __func__);
#if defined(CONFIG_SWITCH_DUAL_MODEM)
		if (gpio_is_valid(GPIO_UART_SEL))	{
			gpio_set_value(GPIO_UART_SEL, GPIO_LEVEL_LOW);
			pr_info("%s: CP %d\n",
					__func__,
					gpio_get_value(GPIO_UART_SEL));
		} else	{
			pr_err("%s: Change GPIO failed",
					__func__);
		}
#endif
	}
	else {
		pr_info("%s: Wrong command\n" , __func__);
	}

	if (!fsa9480_set_path(usbsw->uart_path))
		pr_info("%s: can't switch to %s\n", __func__, buf);

	return count;

}
static ssize_t usb_sel_show_attrs(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsa9480_usbsw *usbsw = chip;

	if (usbsw->mansw== SWITCH_USB_AP)
		return sprintf(buf, "PDA");
	else
		return sprintf(buf, "MODEM");
}

static ssize_t usb_sel_set_attrs(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	pr_info("func:%s buf:%s,count:%d\n", __func__, buf, count);

	if (!strncasecmp(buf, "PDA", 3)) {
		usbsw->mansw = SWITCH_USB_AP;
		pr_info("%s: AP_USB_MODE\n", __func__);
#if defined(CONFIG_SWITCH_DUAL_MODEM)
	} else if (!strncasecmp(buf, "MODEM2", 6)) {
		usbsw->mansw = AUDIO;
		gpio_set_value(GPIO_USB_SEL, GPIO_LEVEL_HIGH);
		dev_info(info->dev, "%s: MODEM2 %d\n", __func__,
			gpio_get_value(GPIO_USB_SEL));
#endif
	} else if (!strncasecmp(buf, "MODEM", 5)) {
		usbsw->mansw = SWITCH_USB_CP;
#if defined(CONFIG_SWITCH_DUAL_MODEM)
		gpio_set_value(GPIO_USB_SEL, GPIO_LEVEL_LOW);
		dev_info(info->dev, "%s: MODEM %d\n", __func__,
			gpio_get_value(GPIO_USB_SEL));
#endif
		pr_info("%s: CP_USB_MODE\n", __func__);
	} else
		pr_info("%s: Wrong command\n", __func__);

	if (!fsa9480_set_path(usbsw->mansw))
		pr_info("%s: can't switch to %s\n", __func__, buf);

	return count;
}
#endif

static DEVICE_ATTR(status, S_IRUGO, fsa9480_show_status, NULL);
static DEVICE_ATTR(switch, S_IRUGO | S_IWUSR,
               fsa9480_show_manualsw, fsa9480_set_manualsw);
static DEVICE_ATTR(syssleep, S_IWUSR, NULL, fsa9480_set_syssleep);
static DEVICE_ATTR(usb_state, S_IRUGO, usb_state_show_attrs, NULL);
#ifdef CONFIG_ONECHIP_DUAL_MODEM
static DEVICE_ATTR(usb_sel, 0664, usb_sel_show_attrs, usb_sel_set_attrs);
static DEVICE_ATTR(uart_sel, 0664, uart_sel_show_attrs, uart_sel_set_attrs);
#endif

static struct attribute *fsa9480_attributes[] = {
       &dev_attr_status.attr,
       &dev_attr_switch.attr,
       &dev_attr_syssleep.attr,
       NULL
};

static const struct attribute_group fsa9480_group = {
       .attrs = fsa9480_attributes,
};

static irqreturn_t fsa9480_irq_handler(int irq, void *data)
{
       struct fsa9480_usbsw *usbsw = data;

       if (!work_pending(&usbsw->work)) {
               disable_irq_nosync(irq);
               schedule_work(&usbsw->work);
       }

       return IRQ_HANDLED;
}

/* SW RESET for TI USB:To fix no USB recog problem after jig attach&detach*/
static void TI_SWreset(struct fsa9480_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;

	printk("[FSA9480] TI_SWreset ...Start\n");
	disable_irq(client->irq); 

	/*Hold SCL&SDA Low more than 30ms*/
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO49),0);
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO50),0);
	msleep(31);
	/*Make SCL&SDA High again*/
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO49),1);
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO50),1);
	/*Should I make this input setting? Not Sure*/
	//gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO64));
	//gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO66));

	/*Write SOME Init register value again*/
	fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x20);
	if (dev_fsa880)
		fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x4);
	else
		fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x1E);

	enable_irq(client->irq); 
	printk("[FSA9480] TI_SWreset ...Done\n");
}
 
 //extern void mv_usb_connect_change(int status); // khMa
static void fsa9480_detect_dev(struct fsa9480_usbsw *usbsw, u8 intr)
{
       u8 val1, val2;// , ctrl,temp;
       struct fsa9480_platform_data *pdata = usbsw->pdata;
       struct i2c_client *client = usbsw->client;

#if 0 // Not for TI
       /*reset except CP USB and AV dock*/
	   if ((usbsw->mansw != AUDIO) && (usbsw->mansw != AUDIO_9485)
	   	   && (usbsw->mansw != VAUDIO) && (usbsw->mansw != VAUDIO_9485))
       	{
		  /*reset UIC when mansw is not set*/
		  printk("[FSA9480] %s: reset UIC mansw is 0x%x\n",__func__,usbsw->mansw);
          fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x1E);
		  usbsw->mansw = AUTO;
       	} 
#endif

       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);
#if 0//Not for TI	   
       fsa9480_read_reg(client, FSA9480_REG_CTRL, &ctrl);

       dev_info(&client->dev, "intr: 0x%x, dev1: 0x%x, dev2: 0x%x, ctrl: 0x%x\n",
                       intr, val1, val2,ctrl);
#endif

	if (intr==0x00)
	{
		printk("[FSA9480] (intr==0x00) chip reset !!!!!\n");

		TI_SWreset(usbsw);
		return;
	}
	else if((intr==0x01) &&(val1==0x00) && (val2==0x00) && (isProbe == 0))
	{
		printk("[FSA9480] (intr==0x01) &&(val1==0x00) && (val2==0x00) !!!!!\n");
		fsa9480_read_adc_value();
		msleep(50);

		fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
		fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);

		if ((val1==0x00) && (val2==0x00))
		{
			TI_SWreset(usbsw);
			return;
		}
	}
	else if ((val1==0xFF) || (val2==0xFF))
	{
		printk("[FSA9480] abnormal device! val1 = 0x%x, val2 = 0x%x\n", val1, val2);
		TI_SWreset(usbsw);
		return;
	}

       /* Attached */
       if (intr & (1 << 0)) 
	{
               if (val1 & FSA9480_DEV_T1_USB_MASK ||
                       val2 & FSA9480_DEV_T2_USB_MASK) {
			printk("[FSA9480] FSA9480_USB ATTACHED*****\n");
#if defined (VBUS_DETECT) /* Enable clock to AP USB block */
			pxa_vbus_handler(VBUS_HIGH);
#endif			
#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_USB_ATTACHED);
			}
#endif
#ifdef CONFIG_ONECHIP_DUAL_MODEM
			fsa9480_set_path(usbsw->mansw);	/* switch */
#endif
			//if (pdata->usb_cb)
			//		  pdata->usb_cb(FSA9480_ATTACHED);
#if 0 // Not for TI
			if (usbsw->mansw)
			{
				  fsa9480_write_reg(client,FSA9480_REG_MANSW1, usbsw->mansw);
				  fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1A);
			}
#endif
               }

               if (val1 & FSA9480_DEV_T1_UART_MASK ||
                       val2 & FSA9480_DEV_T2_UART_MASK) {
                       if (pdata->uart_cb)
                               pdata->uart_cb(FSA9480_ATTACHED);
#ifdef CONFIG_ONECHIP_DUAL_MODEM
			fsa9480_set_path(usbsw->uart_path);	/* switch */
#endif
               }

               if (val1 & FSA9480_DEV_T1_CHARGER_MASK) {
			printk("[FSA9480] Charger ATTACHED*****\n");
                       if (pdata->charger_cb)
                               pdata->charger_cb(FSA9480_ATTACHED);
#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_TA_ATTACHED);
			}
#endif
               }

               if (val2 & FSA9480_DEV_T2_JIG_MASK) {
			printk("[FSA9480] JIG ATTACHED*****\n"); 			   
			//wake_lock(&JIGConnect_idle_wake);
			//wake_lock(&JIGConnect_suspend_wake);
                       if (pdata->jig_cb)
                               pdata->jig_cb(FSA9480_ATTACHED);
#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_JIG_ATTACHED);
			}
#endif
               }
		if (fsa9480_is_ovp)
		{
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_BATTERY, SPA_BATTERY_EVENT_OVP_CHARGE_STOP);
			}
		}			
#if 0 // Not for TI	
		if (val1 & DEV_CAR_KIT) {
			fsa9480_write_reg(client,FSA9480_REG_CK, 0x02);
			fsa9480_read_reg(client, FSA9480_REG_CK_INT1, &temp);				 
		}
#ifdef CONFIG_EXTRA_DOCK_SPEAKER	
               if ((val1 & FSA9480_DEV_T1_AUDIO_MASK) ||(val2 & DEV_JIG_UART_ON) ) {
			   	        printk("[FSA9480] %s : CAR DOCK connected\n",__func__);
			   	       /*set forcely to AV*/
                       if(usbsw->id == 0)
				           fsa9480_write_reg(client,FSA9480_REG_MANSW1, VAUDIO_9485);
					   else
					   	   fsa9480_write_reg(client,FSA9480_REG_MANSW1, VAUDIO);
				       fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1A);
					   
                       if (pdata->cardock_cb)
                               pdata->cardock_cb(FSA9480_ATTACHED);
               }			   
#endif
#ifdef CONFIG_VIDEO_MHL_V1
	       if(val2 & FSA9480_DEV_T2_MHL_MASK) {
		   		if(isDeskdockconnected)
		   			{
		   			  printk("[FSA9480] %s : Deskdock already inserted\n",__func__);
				      return;
		   			}
		    printk("[FSA9480] %s : MHL connected\n",__func__);
		
		    /*set open by manual switching*/
			if(usbsw->id == 0)
				fsa9480_write_reg(client,FSA9480_REG_MANSW1, 0x90);
			else
				fsa9480_write_reg(client,FSA9480_REG_MANSW1, 0x91);
		    fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1A);

	   FSA9480_EnableIntrruptByMHL(false);	

                    if(pdata->mhl_cb) {
                        pdata->mhl_cb(FSA9480_ATTACHED);
						isMHLconnected = 1;
                    }
                }
#else
          if(val2 & DEV_AV ) {
		  	   if(pdata->deskdock_cb)
                       pdata->deskdock_cb(FSA9480_ATTACHED); 
			   usbsw->mansw = VAUDIO_9485;
			   printk("[FSA9480] %s: VAUDIO-ATTACHED\n",__func__);
          	}
#endif
#endif
       } 
	else if (intr & (1 << 1)) 
	{    /* DETACH */
               if (usbsw->dev1 & FSA9480_DEV_T1_USB_MASK ||
                       usbsw->dev2 & FSA9480_DEV_T2_USB_MASK) {
                       printk("[FSA9480] FSA9480_USB Detached*****\n");
#if defined (VBUS_DETECT) /* Disable clock to AP USB block */
			pxa_vbus_handler(VBUS_LOW);
#endif			
#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_USB_DETACHED);
			}
#endif
		   //if (pdata->usb_cb)
		   //		 pdata->usb_cb(FSA9480_DETACHED);
               }

               if (usbsw->dev1 & FSA9480_DEV_T1_UART_MASK ||
                       usbsw->dev2 & FSA9480_DEV_T2_UART_MASK) {
                       if (pdata->uart_cb)
                               pdata->uart_cb(FSA9480_DETACHED);
               }

               if (usbsw->dev1 & FSA9480_DEV_T1_CHARGER_MASK) {
			printk("[FSA9480] Charger Detached*****\n");
                       if (pdata->charger_cb)
                               pdata->charger_cb(FSA9480_DETACHED);
#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_TA_DETACHED);
			}
#endif
               }

               if (usbsw->dev2 & FSA9480_DEV_T2_JIG_MASK) {      
			printk("[FSA9480] JIG Detached*****\n");			   	
			//wake_unlock(&JIGConnect_idle_wake);
			//wake_unlock(&JIGConnect_suspend_wake);
                       if (pdata->jig_cb)
                               pdata->jig_cb(FSA9480_DETACHED);
#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_JIG_DETACHED);
			}
#endif
			/*SW RESET for TI USB:To fix no USB recog problem after jig attach&detach*/
			TI_SWreset(usbsw);
               }

#if 0 // Not for TI 
#ifdef CONFIG_EXTRA_DOCK_SPEAKER
               if ((usbsw->dev1 & FSA9480_DEV_T1_AUDIO_MASK)||(usbsw->dev2 & DEV_JIG_UART_ON)) {
                       if (pdata->cardock_cb)
                               pdata->cardock_cb(FSA9480_DETACHED);
               }
			   
#endif
#ifdef CONFIG_VIDEO_MHL_V1
	       if (usbsw->dev2 & FSA9480_DEV_T2_MHL_MASK) {
		   	   if(isMHLconnected)
		   	   	{
		           printk("[FSA9480] %s : MHL disconnected\n",__func__);

				   isMHLconnected = 0;
				   
                  // if (pdata->mhl_cb)
                  //       pdata->mhl_cb(FSA9480_DETACHED);
		   	   	}
			   else
			   	{
			   	  if (pdata->deskdock_cb)
                         pdata->deskdock_cb(FSA9480_DETACHED);
				  fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1E);
				  isDeskdockconnected=0;
				  usbsw->mansw = AUTO;
			   	}
              }
#else
         if (usbsw->dev2 & DEV_AV) {
		 	     if (pdata->deskdock_cb)
                         pdata->deskdock_cb(FSA9480_DETACHED);
				 fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1E);
				 usbsw->mansw = AUTO;
         	}
#endif
       /*deskdock detach case*/
       if(isDeskdockconnected)
       	{
       	 if (pdata->deskdock_cb)
                         pdata->deskdock_cb(FSA9480_DETACHED);
				  fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1E);
				  isDeskdockconnected=0;
				  usbsw->mansw = AUTO;
       	}
 
	   if ((usbsw->mansw != AUDIO) && (usbsw->mansw != AUDIO_9485)
	   	   && (usbsw->mansw != VAUDIO) && (usbsw->mansw != VAUDIO_9485))
	  	  usbsw->mansw = AUTO;
#endif	
       }
	else if (intr & (1 << 5)) 
	{
		fsa9480_is_ovp = 1;
		printk("[FSA9480] FSA9480_OVP detected*****\n");
#if defined(CONFIG_SPA)
		if(spa_external_event)
		{
			spa_external_event(SPA_CATEGORY_BATTERY, SPA_BATTERY_EVENT_OVP_CHARGE_STOP);
		}
#endif
	}
	else if (intr & (1 << 7))
	{
		if (fsa9480_is_ovp)
		{
			fsa9480_is_ovp = 0;
			printk("[FSA9480] FSA9480_OVP released*****\n");
#if defined(CONFIG_SPA)
	             if(spa_external_event)
			{
				if ((val1==0x00) && (val2==0x00))
				{
					spa_external_event(SPA_CATEGORY_BATTERY, SPA_BATTERY_EVENT_OVP_CHARGE_RESTART);
					spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_TA_DETACHED);
				}
				else
				spa_external_event(SPA_CATEGORY_BATTERY, SPA_BATTERY_EVENT_OVP_CHARGE_RESTART);
			}
#endif
			
		}
	}

       usbsw->dev1 = val1;
       usbsw->dev2 = val2;

       chip->dev1 = val1;
       chip->dev2 = val2;

#if 0 // Not for TI
       fsa9480_read_reg(client, FSA9480_REG_CTRL, &ctrl);
       ctrl &= ~INT_MASK;
       fsa9480_write_reg(client, FSA9480_REG_CTRL, ctrl);

       fsa9480_read_reg(client, FSA9480_REG_MANSW1, &temp); //khMa	   
#endif
}

int get_real_usbic_state(void)
{
       struct fsa9480_usbsw *usbsw = chip;
       int ret = MICROUSBIC_NO_DEVICE ;
       u8 val1 = 0;
       u8 val2 = 0;

       /* read real usb ic state
       val1 = chip->dev1;
       val2 = chip->dev2;
       */
       struct i2c_client *client = usbsw->client;
       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);

       if (val1 & FSA9480_DEV_T1_USB_MASK)
               ret = MICROUSBIC_USB_CABLE;
       else if (val1 & FSA9480_DEV_T1_CHARGER_MASK)
               ret = MICROUSBIC_USB_CHARGER;
       else if (val1 & FSA9480_DEV_T1_UART_MASK)
               ret = MICROUSBIC_USB_CHARGER;
	   else if (val1 & FSA9480_DEV_T1_HOST_MASK)
		   	   ret = MICROUSBIC_HOST;

       if (ret ==  MICROUSBIC_NO_DEVICE) {
               if (val2 & DEV_JIG_USB_ON)
				   ret = MICROUSBIC_JIG_USB_ON;
			   else if (val2 & FSA9480_DEV_T2_MHL_MASK)
				   ret = MICROUSBIC_MHL_CHARGER;
       }

       return ret;
}

static void fsa9480_work_cb(struct work_struct *work)
{
       u8 intr, intr2;//, val1, val2;
       struct fsa9480_usbsw *usbsw =
               container_of(work, struct fsa9480_usbsw, work);
       struct i2c_client *client = usbsw->client;

#if 0 // Not for TI
       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);

	   /*check AV cable for volume up & down*/
	   if(val2 & DEV_AV )
	   	if( (usbsw->mansw != VAUDIO) && (usbsw->mansw != VAUDIO_9485))
	   	{
		DisableFSA9480Interrupts();
	   	}
#endif
	//mdelay(50); /*3. Grab interrupt right away*/

       /* clear interrupt */
	fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
	fsa9480_read_reg(client, FSA9480_REG_INT2, &intr2);
	intr &= 0xffff;

	enable_irq(client->irq); /* 1. moved to above */

#if 0 // Not for TI
	if((val2 & DEV_AV) && (intr2 == 0x04))
		fsa9480_read_adc_value();
	else
#endif
         /* device detection */
       fsa9480_detect_dev(usbsw, intr);
}

static int fsa9480_irq_init(struct fsa9480_usbsw *usbsw)
{
       //struct fsa9480_platform_data *pdata = usbsw->pdata;
       struct i2c_client *client = usbsw->client;
       int ret, irq = -1;
       u8 intr, intr2, ctrl_init;
       u8 mansw1;
       unsigned int ctrl = CTRL_MASK;

       /* clear interrupt */
       fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
	   fsa9480_read_reg(client, FSA9480_REG_INT2, &intr2);
       intr &= 0xffff;

       /* unmask interrupt (attach/detach only) */
       ret = fsa9480_write_reg(client, FSA9480_REG_INT1_MASK, 0x00);
       if (ret < 0)
               return ret;

#if 0/*FSA9480's Interrupt Mask init setting*/
       //ret = fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x00);
#endif
       /*TI USB : not to get Connect Interrupt : no more double interrupt*/
	ret = fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x20);
       if (ret < 0)
               return ret;

       fsa9480_read_reg(client, FSA9480_REG_MANSW1, &mansw1);
       usbsw->mansw = mansw1;

       ctrl &= ~INT_MASK;              /* Unmask Interrupt */

       if (usbsw->mansw)
               ctrl &= ~MANUAL_SWITCH; /* Manual Switching Mode */


	fsa9480_read_reg(client, FSA9480_REG_CTRL, &ctrl_init);
	if ((ctrl_init == 0x1F) || (ctrl_init  == 0x1E))
		fsa9480_write_reg(client, FSA9480_REG_CTRL, ctrl);
 	else
	{
		dev_fsa880 = 1;
		fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x4);
	}
	   INIT_WORK(&usbsw->work, fsa9480_work_cb);

	ret = gpio_request(pxa_irq_to_gpio(client->irq),"fsa9480 irq");
	if(ret)
	{
		dev_err(&client->dev,"fsa9480: Unable to get gpio %d\n", client->irq);
		goto gpio_out;
	}
	   gpio_direction_input(pxa_irq_to_gpio(client->irq));

       ret = request_irq(client->irq, fsa9480_irq_handler,
                       IRQF_NO_SUSPEND|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_LOW,
                       "fsa9480 micro USB", usbsw); /*2. Low level detection*/
       if (ret) {
               dev_err(&client->dev,
                       "fsa9480: Unable to get IRQ %d\n", irq);
               goto out;
       }

       return 0;
gpio_out:
	   gpio_free(pxa_irq_to_gpio(client->irq));
out:
       return ret;
}

static int __devinit fsa9480_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
       struct fsa9480_usbsw *usbsw;
	 struct device *switch_dev;	
       int ret = 0;
       printk("[FSA9480] PROBE ......\n");
	   
	isProbe = 1;
       //add for AT Command 
       //wake_lock_init(&JIGConnect_idle_wake, WAKE_LOCK_IDLE, "jig_connect_idle_wake");
       //wake_lock_init(&JIGConnect_suspend_wake, WAKE_LOCK_SUSPEND, "jig_connect_suspend_wake");

       usbsw = kzalloc(sizeof(struct fsa9480_usbsw), GFP_KERNEL);
       if (!usbsw) {
               dev_err(&client->dev, "failed to allocate driver data\n");
               return -ENOMEM;
       }

       usbsw->client = client;
       usbsw->pdata = client->dev.platform_data;

       chip = usbsw;

       i2c_set_clientdata(client, usbsw);

       ret = fsa9480_irq_init(usbsw);
       if (ret)
               goto fsa9480_probe_fail;

	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");

	if (device_create_file(switch_dev, &dev_attr_adc) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_adc.attr.name);
	if (device_create_file(switch_dev,&dev_attr_usb_state) < 0)
		pr_err("Failed to create device file(%s)!\n",dev_attr_usb_state.attr.name);

#ifdef CONFIG_ONECHIP_DUAL_MODEM
	if (device_create_file(switch_dev,&dev_attr_usb_sel) < 0)
		pr_err("Failed to create device file(%s)!\n",dev_attr_usb_sel.attr.name);
	if (device_create_file(switch_dev,&dev_attr_uart_sel) < 0)
		pr_err("Failed to create device file(%s)!\n",dev_attr_usb_sel.attr.name);
#endif

	dev_set_drvdata(switch_dev,usbsw);

       ret = sysfs_create_group(&client->dev.kobj, &fsa9480_group);
       if (ret) {
               dev_err(&client->dev,
                               "[FSA9480] Creating fsa9480 attribute group failed");
               goto fsa9480_probe_fail2;
       }

       fsa9480_write_reg(client, FSA9480_REG_TIMING1, 0x6);
	   fsa9480_read_reg(client, FSA9480_REG_DEVID, &usbsw->id);

       if (chip->pdata->reset_cb)
               chip->pdata->reset_cb();

	   chip->pdata->id_open_cb = fsa9480_id_open;
	   
#if defined (VBUS_DETECT)
//	chip->pdata = pm860x_pdata->vbus;
	//printk("[FSA9480] yo 1 ...\n");
	vbus = kzalloc(sizeof(struct pm860x_vbus_info), GFP_KERNEL);
	if (!vbus) {
		ret = -ENOMEM;
		goto out_mem;
	}
	dev_set_drvdata(&client->dev, vbus);
	
	vbus->res = kzalloc(sizeof(struct resource), GFP_KERNEL);
	if (!vbus->res) {
		ret = -ENOMEM;
		goto out_mem2;
	}
	vbus->res->start = pdata->vbus->reg_base;
	vbus->res->end = pdata ->vbus->reg_end;

	memset(&info,0,sizeof(struct pxa_vbus_info));
	info.dev = &client->dev;
	info.res = vbus->res;

	pxa_vbus_init(&info);

	data = VBUS_A_VALID | VBUS_A_SESSION_VALID | VBUS_B_SESSION_VALID | VBUS_B_SESSION_END | VBUS_ID;
	pxa_unmask_vbus(data);

#endif

#if defined(CONFIG_SPA)
	spa_external_event = spa_get_external_event_handler();
#endif

       /* device detection */
       fsa9480_detect_dev(usbsw, 1);
	isProbe = 0;

	/*reset UIC*/
	if (dev_fsa880)
		fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x4);
	else
		fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x1E);

	/*set timing1 to 100ms*/
	fsa9480_write_reg(client, FSA9480_REG_TIMING1, 0x1);

#ifdef CONFIG_ONECHIP_DUAL_MODEM
	if (!gpio_request(mfp_to_gpio(MFP_PIN_GPIO104), "uart_sel")) {
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO104), 0);
		gpio_free(mfp_to_gpio(MFP_PIN_GPIO104));
		printk("[%s] request GPIO_UART_SEL sucess!\n", __func__);
	}
	else	{
		printk("[%s] failed GPIO_UART_SEL !!!\n", __func__);
	}

	if (switch_sel & 0x1)	{
		usbsw->mansw = SWITCH_USB_AP;
	}
	else	{
		usbsw->mansw = SWITCH_USB_CP;
	}
	if (switch_sel & 0x2)	{
		usbsw->uart_path= SWITCH_UART_AP;
	}
	else	{
		usbsw->uart_path= SWITCH_UART_CP;
	}
	fsa9480_set_path(usbsw->uart_path);
#endif

        printk("[FSA9480] PROBE Done.\n");
        return 0;

#if defined (VBUS_DETECT)
out_mem:
	return ret;
out_mem2:
	kfree(vbus);
#endif

fsa9480_probe_fail2:
       if (client->irq)
               free_irq(client->irq, NULL);
fsa9480_probe_fail:
       i2c_set_clientdata(client, NULL);
       kfree(usbsw);
       return ret;
}
static int __devexit fsa9480_remove(struct i2c_client *client)
{
       struct fsa9480_usbsw *usbsw = i2c_get_clientdata(client);
       if (client->irq)
               free_irq(client->irq, NULL);
       i2c_set_clientdata(client, NULL);

       sysfs_remove_group(&client->dev.kobj, &fsa9480_group);
       kfree(usbsw);
       return 0;
}

static int fsa9480_suspend(struct i2c_client *client,pm_message_t state)
{
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO49),1); // set_value
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO50),1);

	printk("[FSA9480] fsa9480_suspend  enable_irq_wake...\n");
		enable_irq_wake(client->irq);

       return 0;
}


#ifdef CONFIG_PM
static int fsa9480_resume(struct i2c_client *client)
{
#if 0 /*Totoro: No need to read at resume */
       struct fsa9480_usbsw *usbsw = i2c_get_clientdata(client);
       u8 intr;
       u8 val1, val2;

       /* for hibernation */
       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);

       if (val1 || val2)
               intr = 1 << 0;
       else
               intr = 1 << 1;

       /* device detection */
       fsa9480_detect_dev(usbsw, intr);
#endif
	   printk("[FSA9480] fsa9480_resume  disable_irq_wake...\n");
	   disable_irq_wake(client->irq);
       return 0;
}
#else
#define fsa9480_resume         NULL
#endif

static const struct i2c_device_id fsa9480_id[] = {
       {"fsa9480", 0},
       {}
};
MODULE_DEVICE_TABLE(i2c, fsa9480_id);

static struct i2c_driver fsa9480_i2c_driver = {
       .driver = {
               .name = "fsa9480",
       },
       .probe = fsa9480_probe,
       .remove = __devexit_p(fsa9480_remove),       
       .resume = fsa9480_resume,
       .suspend= fsa9480_suspend,
       .id_table = fsa9480_id,
};

static int __init fsa9480_init(void)
{
	printk("[FSA9480] fsa9480_init\n");
       return i2c_add_driver(&fsa9480_i2c_driver);
}

module_init(fsa9480_init);

#ifdef CONFIG_CHARGER_DETECT_BOOT
charger_module_init(fsa9480_init);
#endif

static void __exit fsa9480_exit(void)
{
       i2c_del_driver(&fsa9480_i2c_driver);
}

module_exit(fsa9480_exit);

MODULE_AUTHOR("Wonguk.Jeong <wonguk.jeong@samsung.com>");
MODULE_DESCRIPTION("FSA9480 USB Switch driver");
MODULE_LICENSE("GPL");


