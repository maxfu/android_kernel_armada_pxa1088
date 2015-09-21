/* drivers/mfd/richtek/rt8973.c
 * Driver to Richtek RT8973 micro USB switch device
 *
 * Copyright (C) 2012
 * Author: Patrick Chang <patrick_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/platform_data/rtmusc.h>
#include <linux/platform_data/mv_usb.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>
#include "rt8973.h"

#define DEVICE_NAME "rt8973"
#define RT8973_DRV_NAME "rt8973"

#define EN_REV0_INT_BOUNCE_FIX 0

#if EN_REV0_INT_BOUNCE_FIX
#define EN_ADCCHG_MASK() en_adcchg_mask()
#define DIS_ADCCHG_MASK() dis_adcchg_mask()
#else
#define EN_ADCCHG_MASK()
#define DIS_ADCCHG_MASK()
#endif

#if defined(CONFIG_SPA)
#include <mach/spa.h>
static void (*spa_external_event) (int, int) = NULL;
extern struct class *sec_class;
#endif
#if defined(CONFIG_BATTERY_SAMSUNG)
extern struct class *sec_class;
#endif

static struct rt8973_data *pDrvData = NULL;	/*driver data */
static struct platform_device *rtmus_dev = NULL;	/* Device structure */
static struct rtmus_platform_data platform_data;
static struct workqueue_struct *rtmus_work_queue = NULL;	/* self-own work queue */

#define RT8973_JIG_CALLBACK(attach,mode) platform_data.jig_callback(attach,mode)
#define RT8973_CHARGER_CALLBACK(attach) platform_data.charger_callback(attach)
#define RT8973_UART_CALLBACK(attach) platform_data.uart_callback(attach)
#define RT8973_OTG_CALLBACK(attach) platform_data.otg_callback(attach)
#define RT8973_USB_CALLBACK(attach) platform_data.usb_callback(attach)
#define RT8973_OTP_CALLBACK(detect) platform_data.over_temperature_callback(detect)
#define RT8973_OVP_CALLBACK(detect) platform_data.over_voltage_callback(detect)
#ifdef CONFIG_BATTERY_SAMSUNG
#define RT8973_SEC_CHARGER_CALLBACK(detect) platform_data.sec_charger_callback(detect)
#endif

#define ID_NONE 0
#define ID_USB  1
#define ID_UART 2
#define ID_CHARGER  3
#define ID_JIG  4
#define ID_UNKNOW 5
#define ID_OTG  6

#define MAX_DCDT_retry 2

static char *devices_name[] = { "NONE",
	"USB",
	"UART",
	"CHARGER",
	"JIG",
	"UNKONW",
	"OTG",
};


static int32_t DCDT_retry = 0;



#define I2C_RW_RETRY_MAX 2
#define I2C_RW_RETRY_DELAY 20

#if 0
#define I2CRByte(x) i2c_smbus_read_byte_data(pClient,x)
#define I2CWByte(x,y) i2c_smbus_write_byte_data(pClient,x,y)
#else
#define I2CRByte(x) retry_i2c_smbus_read_byte_data(pClient,x)
#define I2CWByte(x,y) retry_i2c_smbus_write_byte_data(pClient,x,y)
static s32 retry_i2c_smbus_write_byte_data(struct i2c_client *client,
					   u8 command, u8 value)
{
	s32 result = -1, i;
	for (i = 0; i < I2C_RW_RETRY_MAX && result < 0; i++) {
		result = i2c_smbus_write_byte_data(client, command, value);
		if (unlikely(result < 0))
			msleep(I2C_RW_RETRY_DELAY);
	}
	return result;
}

static s32 retry_i2c_smbus_read_byte_data(struct i2c_client *client, u8 command)
{
	s32 result = -1, i;
	for (i = 0; i < I2C_RW_RETRY_MAX && result < 0; i++) {
		result = i2c_smbus_read_byte_data(client, command);
		if (unlikely(result < 0))
			msleep(I2C_RW_RETRY_DELAY);
	}
	return result;
}
#endif

static int rt_sysfs_create_files(struct kobject *kobj, struct attribute **attrs)
{
	int err;
	while (*attrs != NULL) {
		err = sysfs_create_file(kobj, *attrs);
		if (err)
			return err;
		attrs++;
	}
	return 0;
}

static ssize_t accessory_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	return sprintf(buf, "%s\n", devices_name[pDrvData->accessory_id]);
}

static ssize_t chip_id_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "0x%x\n", pDrvData->chip_id);
}

static ssize_t driver_version_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", RTMUSC_DRIVER_VER);
}

static ssize_t usbid_adc_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	return sprintf(buf, "0x%x\n", pDrvData->usbid_adc);
}

static ssize_t factory_mode_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", pDrvData->factory_mode);
}

static ssize_t operating_mode_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", pDrvData->operating_mode);
}

static ssize_t operating_mode_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t len)
{
	uint32_t value = simple_strtoul(buf, NULL, 10);
	struct i2c_client *pClient = pDrvData->client;
	int32_t regCtrl1 = I2CRByte(RT8973_REG_CONTROL_1);
	pDrvData->operating_mode = value;
	if (value)
		regCtrl1 &= (~0x04);	// Manual Contrl
	else
		regCtrl1 |= 0x04;	// Auto Control
	I2CWByte(RT8973_REG_CONTROL_1, regCtrl1);
	return len;
}

static struct kobj_attribute accessory_attribute =
    (struct kobj_attribute)__ATTR_RO(accessory);
static struct kobj_attribute chip_id_attribute =
    (struct kobj_attribute)__ATTR_RO(chip_id);
static struct kobj_attribute driver_version_attribute =
    (struct kobj_attribute)__ATTR_RO(driver_version);
static struct kobj_attribute usbid_adc_attribute =
    (struct kobj_attribute)__ATTR_RO(usbid_adc);
static struct kobj_attribute factory_mode_attribute =
    (struct kobj_attribute)__ATTR_RO(factory_mode);
static struct kobj_attribute operating_mode_attribute =
    (struct kobj_attribute)__ATTR_RW(operating_mode);

static struct attribute *rt8973_attrs[] = {
	&accessory_attribute.attr,
	&chip_id_attribute.attr,
	&driver_version_attribute.attr,
	&factory_mode_attribute.attr,
	&usbid_adc_attribute.attr,
	&operating_mode_attribute.attr,
	NULL,
};

#ifdef CONFIG_RT_SYSFS_DBG
static ssize_t reg_addr_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf, int32_t nRegAddr)
{
	struct i2c_client *pClient = pDrvData->client;
	return sprintf(buf, "%d\n", I2CRByte(nRegAddr));
}

static ssize_t reg_addr_store(struct kobject *kobj,
			      struct kobj_attribute *attr,
			      const char *buf, size_t len, int32_t nRegAddr)
{
	uint32_t value = simple_strtoul(buf, NULL, 10);
	struct i2c_client *pClient = pDrvData->client;
	I2CWByte(nRegAddr, value);
	return len;
}

static ssize_t enable_irq_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t len)
{
	uint32_t value = simple_strtoul(buf, NULL, 10);
	INFO("GPIO %d Value = %d\n", CONFIG_RTMUSC_INT_GPIO_NUMBER,
	     gpio_get_value(CONFIG_RTMUSC_INT_GPIO_NUMBER));
	if (value == 1)
		enable_irq(pDrvData->irq);
	else
		disable_irq(pDrvData->irq);
	return len;
}

#define regRO(addr) \
    static ssize_t reg##addr##_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf) \
    { return reg_addr_show(kobj, attr, buf, addr); }

#define regRW(addr) \
    static ssize_t reg##addr##_store(struct kobject * kobj,struct kobj_attribute * attr,const char * buf, size_t len) \
    { return reg_addr_store(kobj,attr,buf,len,addr);} \
    regRO(addr)

regRO(0x01)
    regRW(0x02)
    regRO(0x03)
    regRO(0x04)
    regRW(0x05)
    regRO(0x07)
    regRO(0x0A)
    regRO(0x0B)
    regRW(0x13)
    regRW(0x14)
    regRW(0x1B)

static struct kobj_attribute reg0x01_attribute =
    (struct kobj_attribute)__ATTR_RO(reg0x01);
static struct kobj_attribute reg0x02_attribute =
    (struct kobj_attribute)__ATTR_RW(reg0x02);
static struct kobj_attribute reg0x03_attribute =
    (struct kobj_attribute)__ATTR_RO(reg0x03);
static struct kobj_attribute reg0x04_attribute =
    (struct kobj_attribute)__ATTR_RO(reg0x04);
static struct kobj_attribute reg0x05_attribute =
    (struct kobj_attribute)__ATTR_RW(reg0x05);
static struct kobj_attribute reg0x07_attribute =
    (struct kobj_attribute)__ATTR_RO(reg0x07);
static struct kobj_attribute reg0x0A_attribute =
    (struct kobj_attribute)__ATTR_RO(reg0x0A);
static struct kobj_attribute reg0x0B_attribute =
    (struct kobj_attribute)__ATTR_RO(reg0x0B);
static struct kobj_attribute reg0x13_attribute =
    (struct kobj_attribute)__ATTR_RW(reg0x13);
static struct kobj_attribute reg0x14_attribute =
    (struct kobj_attribute)__ATTR_RW(reg0x14);
static struct kobj_attribute reg0x1B_attribute =
    (struct kobj_attribute)__ATTR_RW(reg0x1B);

static struct kobj_attribute enable_irq_attribute =
    (struct kobj_attribute)__ATTR_WO(enable_irq);
static struct attribute *rt8973_dbg_attrs[] = {
	&reg0x01_attribute.attr,
	&reg0x02_attribute.attr,
	&reg0x03_attribute.attr,
	&reg0x04_attribute.attr,
	&reg0x05_attribute.attr,
	&reg0x07_attribute.attr,
	&reg0x0A_attribute.attr,
	&reg0x0B_attribute.attr,
	&reg0x13_attribute.attr,
	&reg0x14_attribute.attr,
	&reg0x1B_attribute.attr,
	&accessory_attribute.attr,
	&chip_id_attribute.attr,
	&driver_version_attribute.attr,
	&factory_mode_attribute.attr,
	&usbid_adc_attribute.attr,
	&operating_mode_attribute.attr,
	&enable_irq_attribute.attr,
	NULL,
};

static struct attribute_group rt8973_dbg_attrs_group = {
	.name = "dbg",
	.attrs = rt8973_dbg_attrs,
};
#endif //CONFIG_RT_SYSFS_DBG

static const struct i2c_device_id richtek_musc_id[] = {
	{"rt8973", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, richtek_musc_id);
inline int32_t wait_for_interrupt(void)
{
	msleep(RT8973_WAIT_DELAY);
	return gpio_get_value(CONFIG_RTMUSC_INT_GPIO_NUMBER);
}

static void en_int_mask(uint8_t mask)
{
	struct i2c_client *pClient = pDrvData->client;
	int32_t regIntMask;
	regIntMask = I2CRByte(RT8973_REG_INTERRUPT_MASK);
	if (regIntMask < 0) {
		ERR("Can't read RT8973_REG_INTERRUPT_MASK(0x%x)"
		    ", return value = %x", RT8973_REG_INTERRUPT_MASK,
		    regIntMask);
		return;
	}
	I2CWByte(RT8973_REG_INTERRUPT_MASK, regIntMask | mask);
}

#if EN_REV0_INT_BOUNCE_FIX
void en_adcchg_mask(void)
{
	if (((pDrvData->chip_id & 0xf8) >> 3) == 0) {	//REV0
		INFO("Disable ADCCHG & ATTACH event\n");
		en_int_mask(0x61);	// ENABLE ADCCHG, ATTACH MASK
	}
}

static void dis_int_mask(uint8_t mask)
{
	struct i2c_client *pClient = pDrvData->client;
	int32_t regIntMask;
	regIntMask = I2CRByte(RT8973_REG_INTERRUPT_MASK);
	if (regIntMask < 0) {
		ERR("Can't read RT8973_REG_INTERRUPT_MASK(0x%x)"
		    ", return value = %x", RT8973_REG_INTERRUPT_MASK,
		    regIntMask);
		return;
	}
	I2CWByte(RT8973_REG_INTERRUPT_MASK, regIntMask & (~mask));

}

void dis_adcchg_mask(void)
{
	if (((pDrvData->chip_id & 0xf8) >> 3) == 0) {	//REV0
		INFO("Enable ADCCHG & ATTACH event\n");
		dis_int_mask(0x61);	// DISABLE ADCCHG, ATTACH MASK
	}
}

#endif
static int enable_interrupt(int32_t enable)
{
	struct i2c_client *pClient = pDrvData->client;
	int32_t regCtrl1;
	regCtrl1 = I2CRByte(RT8973_REG_CONTROL_1);
	if (enable)
		regCtrl1 &= (~0x01);
	else
		regCtrl1 |= 0x01;
	return I2CWByte(RT8973_REG_CONTROL_1, regCtrl1);

}

inline void do_attach_work(int32_t regIntFlag, int32_t regDev1, int32_t regDev2)
{
	int32_t regADC;
	int32_t regCtrl1;
	int32_t regIntFlag2;
	struct i2c_client *pClient = pDrvData->client;
	regIntFlag2 = I2CRByte(RT8973_REG_INT_FLAG2);

	INFO("regIntFlag=%02x, regIntFlag2=%02x, regDev1=%02x, regDev2=%02x \n", 
	   regIntFlag, regIntFlag2, regDev1, regDev2);
	
	if (regIntFlag & RT8973_INT_DCDTIMEOUT_MASK) {
		regADC = I2CRByte(RT8973_REG_ADC) & 0x1f;
		if (regADC == 0x1d || regADC == 0x1c
		    || regADC == 0x19 || regADC == 0x18) {
			INFO("No VBUS JIG\n");
			switch (regADC) {
			case 0x1d:	//Factory Mode : JIG UART ON = 1
				if (pDrvData->operating_mode) {
					I2CWByte(RT8973_REG_MANUAL_SW1, 0x6c);
					I2CWByte(RT8973_REG_MANUAL_SW2, 0x0c);
				}
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode = RTMUSC_FM_BOOT_ON_UART;
				EN_ADCCHG_MASK();
#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_JIG_UART_ON);
				}
#else
				if (platform_data.jig_callback) {
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_ON_UART);
				}
#endif
				break;
			case 0x1c:	//Factory Mode : JIG UART OFF = 1
				if (pDrvData->operating_mode) {
					I2CWByte(RT8973_REG_MANUAL_SW1, 0x6c);
					I2CWByte(RT8973_REG_MANUAL_SW2, 0x04);
				}
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode =
				    RTMUSC_FM_BOOT_OFF_UART;
				EN_ADCCHG_MASK();
#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_JIG_UART_OFF);
				}
#else
				if (platform_data.jig_callback) {
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_OFF_UART);
				}
#endif
				break;
			case 0x19:	//Factory Mode : JIG USB ON = 1
				if (pDrvData->operating_mode) {
					I2CWByte(RT8973_REG_MANUAL_SW1, 0x24);
					I2CWByte(RT8973_REG_MANUAL_SW2, 0x0c);
				}
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode = RTMUSC_FM_BOOT_ON_USB;
				EN_ADCCHG_MASK();
#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.usb_callback) {
					RT8973_USB_CALLBACK(1);
				}
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_JIG_USB_ON);
				}
#else
				if (platform_data.jig_callback) {
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_ON_USB);
				}
#endif
				break;
			case 0x18:	//Factory Mode : JIG USB OFF= 1
				if (pDrvData->operating_mode) {
					I2CWByte(RT8973_REG_MANUAL_SW1, 0x6c);
					I2CWByte(RT8973_REG_MANUAL_SW2, 0x04);
				}
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode = RTMUSC_FM_BOOT_OFF_USB;
				EN_ADCCHG_MASK();
#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.usb_callback) {
					RT8973_USB_CALLBACK(1);
				}
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_JIG_USB_OFF);
				}
#else
				if (platform_data.jig_callback) {
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_OFF_USB);
				}
#endif
			default:
				;
			}
			en_int_mask(0x08);
			return;
		}
		INFO("Redo USB charger detection\n");
		regCtrl1 = I2CRByte(RT8973_REG_CONTROL_1);
		if (regCtrl1 < 0)
			ERR("I2C read error\n");
		if (DCDT_retry >= MAX_DCDT_retry) {
			WARNING("Exceed max DCD_T retry\n");
			en_int_mask(0x08);
			/* > DCD retry
			   If UVLO ==0, this accessory should be a charger
			 */
			if ((regIntFlag2 & 0x02) == 0) {
				INFO("DCD_T retry failed : found VBUS ==> might be dock or unknown charger accessory\n");
				pDrvData->accessory_id = ID_CHARGER;
#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_0x1A);
				}
#else
				if (platform_data.charger_callback) {
					RT8973_CHARGER_CALLBACK(1);
				}
#endif
			} else if (pDrvData->accessory_id == ID_CHARGER)	// last accessory == CHARGER
			{
				INFO("Charger lost power\n");
				pDrvData->accessory_id = ID_UNKNOW;
#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_NONE);
				}
#else
				if (platform_data.charger_callback) {
					RT8973_CHARGER_CALLBACK(0);
				}
#endif
			} else {
				pDrvData->accessory_id = ID_UNKNOW;
				WARNING("Detected unkown accessory!!\n");
			}
			return;
		}
		I2CWByte(RT8973_REG_CONTROL_1, (uint8_t) (regCtrl1 & 0x9f));
		msleep(RT8973_WAIT_DELAY);
		I2CWByte(RT8973_REG_CONTROL_1, (uint8_t) (regCtrl1 | 0x60));
		DCDT_retry++;
		return;
	}

	if ((regIntFlag & RT8973_INT_CHGDET_MASK) && (regDev1 & 0x50)) {
		pDrvData->accessory_id = ID_CHARGER;
#ifdef CONFIG_BATTERY_SAMSUNG
		if (platform_data.sec_charger_callback) {
			RT8973_SEC_CHARGER_CALLBACK
			    (MUIC_RT8973_CABLE_TYPE_TA);
		}
#else
		if (platform_data.charger_callback) {
			RT8973_CHARGER_CALLBACK(1);
		}
#endif
	}
	regADC = I2CRByte(RT8973_REG_ADC) & 0x1f;
	pDrvData->usbid_adc = regADC;
	if (regIntFlag & RT8973_INT_ADCCHG_MASK) {
		if (pDrvData->operating_mode) {	// Manual Switch
			switch (regADC) {
			case 0x16:	// UART cable
				I2CWByte(RT8973_REG_MANUAL_SW1, 0x6c);
				pDrvData->accessory_id = ID_UART;
				EN_ADCCHG_MASK();
				if (platform_data.uart_callback)
					RT8973_UART_CALLBACK(1);
				break;
			case 0x1d:	//Factory Mode : JIG UART ON = 1
				I2CWByte(RT8973_REG_MANUAL_SW1, 0x6c);
				I2CWByte(RT8973_REG_MANUAL_SW2, 0x0c);
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode = RTMUSC_FM_BOOT_ON_UART;
				EN_ADCCHG_MASK();
				if (platform_data.jig_callback)
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_ON_UART);
				break;
			case 0x1c:	//Factory Mode : JIG UART OFF = 1
				I2CWByte(RT8973_REG_MANUAL_SW1, 0x6c);
				I2CWByte(RT8973_REG_MANUAL_SW2, 0x04);
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode =
				    RTMUSC_FM_BOOT_OFF_UART;
				EN_ADCCHG_MASK();
				if (platform_data.jig_callback)
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_OFF_UART);
				break;
			case 0x19:	//Factory Mode : JIG USB ON = 1
				I2CWByte(RT8973_REG_MANUAL_SW1, 0x24);
				I2CWByte(RT8973_REG_MANUAL_SW2, 0x0c);
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode = RTMUSC_FM_BOOT_ON_USB;
				EN_ADCCHG_MASK();
				if (platform_data.jig_callback)
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_ON_USB);
				break;
			case 0x18:	//Factory Mode : JIG USB OFF= 1
				I2CWByte(RT8973_REG_MANUAL_SW1, 0x6c);
				I2CWByte(RT8973_REG_MANUAL_SW2, 0x04);
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode = RTMUSC_FM_BOOT_OFF_USB;
				EN_ADCCHG_MASK();
				if (platform_data.jig_callback)
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_OFF_USB);
				break;
			case 0x1a:
				// desk dock
				if ((regIntFlag2 & 0x02) == 0) {
					INFO("DCD_T retry failed : found VBUS ==> might be dock or unknown charger accessory\n");
					pDrvData->accessory_id = ID_CHARGER;
					if (platform_data.charger_callback)
						RT8973_CHARGER_CALLBACK(1);
				} else if (pDrvData->accessory_id == ID_CHARGER)	// last accessory == CHARGER
				{
					INFO("Charger lost power\n");
					pDrvData->accessory_id = ID_UNKNOW;
					if (platform_data.charger_callback)
						RT8973_CHARGER_CALLBACK(0);
				} else {
					pDrvData->accessory_id = ID_UNKNOW;
					WARNING
					    ("Detected unkown accessory!!\n");
				}
				break;
			case 0x15:
				// audio dock or smart hub dock or unknown accessory
				if ((regIntFlag2 & 0x02) == 0) {
					INFO("DCD_T retry failed : found VBUS ==> might be dock or unknown charger accessory\n");
					pDrvData->accessory_id = ID_CHARGER;
					if (platform_data.charger_callback)
						RT8973_CHARGER_CALLBACK(1);
				} else if (pDrvData->accessory_id == ID_CHARGER)	// last accessory == CHARGER
				{
					INFO("Charger lost power\n");
					pDrvData->accessory_id = ID_UNKNOW;
					if (platform_data.charger_callback)
						RT8973_CHARGER_CALLBACK(0);
				} else {
					pDrvData->accessory_id = ID_UNKNOW;
					WARNING
					    ("Detected unkown accessory!!\n");
				}
				break;
				//Unknow accessory
			case 0x1b:
			case 0x1e:
				pDrvData->accessory_id = ID_UNKNOW;
				WARNING("Detected unkown accessory!!\n");
				break;
			case 0x00:
				if (regDev1 & 0x01) {	// OTG
					I2CWByte(RT8973_REG_MANUAL_SW1, 0x24);
					I2CWByte(RT8973_REG_MANUAL_SW2, 0x01);
					pDrvData->accessory_id = ID_OTG;
					if (platform_data.otg_callback)
						RT8973_OTG_CALLBACK(1);

				} else {
					if (pDrvData->accessory_id == ID_CHARGER)	// last accessory == CHARGER
					{
						INFO("Charger lost power\n");
						pDrvData->accessory_id =
						    ID_UNKNOW;
						if (platform_data.
						    charger_callback)
							RT8973_CHARGER_CALLBACK
							    (0);
					}
				}
				break;
			case 0x17:
				pDrvData->accessory_id = ID_CHARGER;
				if (platform_data.charger_callback)
					RT8973_CHARGER_CALLBACK(1);
				break;
			default:
				if (pDrvData->accessory_id == ID_CHARGER)	// last accessory == CHARGER
				{
					INFO("Charger lost power\n");
					pDrvData->accessory_id = ID_UNKNOW;
					if (platform_data.charger_callback)
						RT8973_CHARGER_CALLBACK(0);
				}
				pDrvData->accessory_id = ID_UNKNOW;
				WARNING("Unknow USB ID ADC = 0x%x\n", regADC);
			}
		} else {	//automatic switch
			switch (regADC) {
			case 0x16:	// UART cable
				// auto switch -- ignore
				INFO("Auto Switch Mode UART cable\n");
				pDrvData->accessory_id = ID_UART;
				EN_ADCCHG_MASK();

#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_UART);
				}
#else
				if (platform_data.uart_callback) {
					RT8973_UART_CALLBACK(1);
				}
#endif
				break;
			case 0x1d:	//Factory Mode : JIG UART ON = 1
				// auto switch -- ignore
				INFO("Auto Switch Mode JIG UART ON= 1\n");
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode = RTMUSC_FM_BOOT_ON_UART;
				EN_ADCCHG_MASK();

#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_JIG_UART_ON);
				}
#else
				if (platform_data.jig_callback) {
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_ON_UART);
				}
#endif
				break;
			case 0x1c:	//Factory Mode : JIG UART OFF = 1
				// auto switch -- ignore
				INFO("Auto Switch Mode JIG UART OFF= 1\n");
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode =
				    RTMUSC_FM_BOOT_OFF_UART;
				EN_ADCCHG_MASK();
#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_JIG_UART_OFF);
				}
#else
				if (platform_data.jig_callback) {
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_OFF_UART);
				}
#endif
				break;
			case 0x19:	//Factory Mode : JIG USB ON = 1
				// auto switch -- ignore
				INFO("Auto Switch Mode JIG USB ON= 1\n");
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode = RTMUSC_FM_BOOT_ON_USB;
				EN_ADCCHG_MASK();
#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.usb_callback) {
					RT8973_USB_CALLBACK(1);
				}
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_JIG_USB_ON);
				}
#else
				if (platform_data.jig_callback) {
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_ON_USB);
				}
#endif
				break;
			case 0x18:	//Factory Mode : JIG USB OFF= 1
				// auto switch -- ignore
				INFO("Auto Switch Mode JIG USB OFF= 1\n");
				pDrvData->accessory_id = ID_JIG;
				pDrvData->factory_mode = RTMUSC_FM_BOOT_OFF_USB;
				EN_ADCCHG_MASK();
#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.usb_callback) {
					RT8973_USB_CALLBACK(1);
				}
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_JIG_USB_OFF);
				}
#else
				if (platform_data.jig_callback) {
					RT8973_JIG_CALLBACK(1,
							    RTMUSC_FM_BOOT_OFF_USB);
				}
#endif
				break;
			case 0x1a:
				if ((regIntFlag2 & 0x02) == 0) {
					INFO("DCD_T retry failed : found VBUS ==> might be dock or unknown charger accessory 0x1A\n");
					pDrvData->accessory_id = ID_CHARGER;
#ifdef CONFIG_BATTERY_SAMSUNG
					if (platform_data.sec_charger_callback) {
						RT8973_SEC_CHARGER_CALLBACK
						    (MUIC_RT8973_CABLE_TYPE_0x1A);
					}
#else
					if (platform_data.charger_callback) {
						RT8973_CHARGER_CALLBACK(1);
					}
#endif
				} else if (pDrvData->accessory_id == ID_CHARGER)	// last accessory == CHARGER
				{
					INFO("Charger lost power\n");
					pDrvData->accessory_id = ID_UNKNOW;
#ifdef CONFIG_BATTERY_SAMSUNG
					if (platform_data.sec_charger_callback) {
						RT8973_SEC_CHARGER_CALLBACK
						    (MUIC_RT8973_CABLE_TYPE_NONE);
					}
#else
					if (platform_data.charger_callback) {
						RT8973_CHARGER_CALLBACK(0);
					}
#endif
				} else {
					pDrvData->accessory_id = ID_UNKNOW;
					WARNING
					    ("Detected unkown accessory!!\n");
				}
				break;
			case 0x15:
				if ((regIntFlag2 & 0x02) == 0) {
					INFO("DCD_T retry failed : found VBUS ==> might be dock or unknown charger accessory 0x15\n");
					pDrvData->accessory_id = ID_CHARGER;
#ifdef CONFIG_BATTERY_SAMSUNG
					if (platform_data.sec_charger_callback) {
						RT8973_SEC_CHARGER_CALLBACK
						    (MUIC_RT8973_CABLE_TYPE_0x15);
					}
#else
					if (platform_data.charger_callback) {
						RT8973_CHARGER_CALLBACK(1);
					}
#endif
				} else if (pDrvData->accessory_id == ID_CHARGER)	// last accessory == CHARGER
				{
					INFO("Charger lost power\n");
					pDrvData->accessory_id = ID_UNKNOW;
#ifdef CONFIG_BATTERY_SAMSUNG
					if (platform_data.sec_charger_callback) {
						RT8973_SEC_CHARGER_CALLBACK
						    (MUIC_RT8973_CABLE_TYPE_NONE);
					}
#else
					if (platform_data.charger_callback) {
						RT8973_CHARGER_CALLBACK(0);
					}
#endif
				} else {
					pDrvData->accessory_id = ID_UNKNOW;
					WARNING
					    ("Detected unkown accessory!!\n");
				}
				break;
			case 0x1b:	//Unknow accessory
			case 0x1e:
				pDrvData->accessory_id = ID_UNKNOW;
				WARNING("Detected unkown accessory!!\n");
				break;
			case 0x00:
				if (regDev1 & 0x01) {	// OTG
					regCtrl1 = I2CRByte(RT8973_REG_CONTROL_1);	/* in automatic mode, switch to manual mode */
					regCtrl1 &= (~0x04);	// Manual Contrl
					I2CWByte(RT8973_REG_CONTROL_1,
						 regCtrl1);
					I2CWByte(RT8973_REG_MANUAL_SW1, 0x24);
					I2CWByte(RT8973_REG_MANUAL_SW2, 0x01);
					pDrvData->accessory_id = ID_OTG;
#ifdef CONFIG_BATTERY_SAMSUNG
					if (platform_data.sec_charger_callback) {
						RT8973_SEC_CHARGER_CALLBACK
						    (MUIC_RT8973_CABLE_TYPE_OTG);
					}
#else
					if (platform_data.otg_callback) {
						RT8973_OTG_CALLBACK(1);
					}
#endif
				} else {
					if (pDrvData->accessory_id == ID_CHARGER)	// last accessory == CHARGER
					{
						INFO("Charger lost power\n");
						pDrvData->accessory_id =
						    ID_UNKNOW;
#ifdef CONFIG_BATTERY_SAMSUNG
						if (platform_data.
						    sec_charger_callback) {
							RT8973_SEC_CHARGER_CALLBACK
							    (MUIC_RT8973_CABLE_TYPE_NONE);
						}
#else
						if (platform_data.
						    charger_callback) {
							RT8973_CHARGER_CALLBACK
							    (0);
						}
#endif
					}
				}
				break;
			case 0x17:
				pDrvData->accessory_id = ID_CHARGER;
#ifdef CONFIG_BATTERY_SAMSUNG
				if (platform_data.sec_charger_callback) {
					RT8973_SEC_CHARGER_CALLBACK
					    (MUIC_RT8973_CABLE_TYPE_TYPE1_CHARGER);
				}
#else
				if (platform_data.charger_callback) {
					RT8973_CHARGER_CALLBACK(1);
				}
#endif
				break;
			default:
				WARNING("Unknow USB ID ADC = 0x%x\n", regADC);
				if (pDrvData->accessory_id == ID_CHARGER)	// last accessory == CHARGER
				{
					INFO("Charger lost power\n");
					pDrvData->accessory_id = ID_UNKNOW;
#ifdef CONFIG_BATTERY_SAMSUNG
					if (platform_data.sec_charger_callback) {
						RT8973_SEC_CHARGER_CALLBACK
						    (MUIC_RT8973_CABLE_TYPE_NONE);
					}
#else
					if (platform_data.charger_callback) {
						RT8973_CHARGER_CALLBACK(0);
					}
#endif
				}
				pDrvData->accessory_id = ID_UNKNOW;
			}
		}
		return;
	}
	if (regDev1 & 0x24) {	//SDPort / CDPort
		INFO("Standard USB Port connected! / USB Charging Downstream Port connected!\n");
		pDrvData->accessory_id = ID_USB;
#ifdef CONFIG_BATTERY_SAMSUNG
		if (platform_data.usb_callback) {
			RT8973_USB_CALLBACK(1);
		}
		if (platform_data.sec_charger_callback) {
			RT8973_SEC_CHARGER_CALLBACK(MUIC_RT8973_CABLE_TYPE_USB);
		}
#else
		if (platform_data.usb_callback) {
			RT8973_USB_CALLBACK(1);
		}
#endif
		if (pDrvData->operating_mode) {
			I2CWByte(RT8973_REG_MANUAL_SW1, 0x24);
		}
		return;
	}
	INFO("Unknown event in Attached Routine\n");
}

inline void do_detach_work(int32_t regIntFlag)
{
	struct i2c_client *pClient = pDrvData->client;
	int32_t regCtrl1;
	if (regIntFlag & 0x40) {
		if (pDrvData->operating_mode) {
			I2CWByte(RT8973_REG_MANUAL_SW1, 0);
			I2CWByte(RT8973_REG_MANUAL_SW2, 0);
		}

	}
	switch (pDrvData->accessory_id) {
	case ID_USB:
#ifdef CONFIG_BATTERY_SAMSUNG
		if (platform_data.sec_charger_callback) {
			RT8973_SEC_CHARGER_CALLBACK
			    (MUIC_RT8973_CABLE_TYPE_NONE);
		}
		if (platform_data.usb_callback) {
			pDrvData->accessory_id = ID_NONE;
			pDrvData->factory_mode = RTMUSC_FM_NONE;
			RT8973_USB_CALLBACK(0);
		}
#else
		if (platform_data.usb_callback) {
			pDrvData->accessory_id = ID_NONE;
			pDrvData->factory_mode = RTMUSC_FM_NONE;
			RT8973_USB_CALLBACK(0);
		}
#endif
		break;
	case ID_UART:
#ifdef CONFIG_BATTERY_SAMSUNG
		if (platform_data.sec_charger_callback) {
			RT8973_SEC_CHARGER_CALLBACK
			    (MUIC_RT8973_CABLE_TYPE_NONE);
		}
#else
		if (platform_data.uart_callback) {
			RT8973_UART_CALLBACK(0);
		}
#endif
		break;
	case ID_JIG:
#ifdef CONFIG_BATTERY_SAMSUNG
		if (platform_data.sec_charger_callback) {
			RT8973_SEC_CHARGER_CALLBACK
			    (MUIC_RT8973_CABLE_TYPE_NONE);
		}
		if ((pDrvData->factory_mode == RTMUSC_FM_BOOT_ON_USB) ||
		    (pDrvData->factory_mode == RTMUSC_FM_BOOT_OFF_USB)) {
			if (platform_data.usb_callback) {
				pDrvData->accessory_id = ID_NONE;
				pDrvData->factory_mode = RTMUSC_FM_NONE;
				RT8973_USB_CALLBACK(0);
			}
		}
#else
		if (platform_data.jig_callback) {
			RT8973_JIG_CALLBACK(0, pDrvData->factory_mode);
		}
#endif
		break;
	case ID_CHARGER:
#ifdef CONFIG_BATTERY_SAMSUNG
		if (platform_data.sec_charger_callback) {
			RT8973_SEC_CHARGER_CALLBACK
			    (MUIC_RT8973_CABLE_TYPE_NONE);
		}
#else
		if (platform_data.charger_callback) {
			RT8973_CHARGER_CALLBACK(0);
		}
#endif
		break;
	case ID_OTG:
		printk("ID_OTG\n");
#ifdef CONFIG_BATTERY_SAMSUNG
		if (platform_data.sec_charger_callback) {
			RT8973_SEC_CHARGER_CALLBACK
			    (MUIC_RT8973_CABLE_TYPE_NONE);
		}
#else
		if (platform_data.otg_callback) {
			RT8973_OTG_CALLBACK(0);
		}
#endif
		if (pDrvData->operating_mode == 0) {
			regCtrl1 = I2CRByte(RT8973_REG_CONTROL_1);
			regCtrl1 |= 0x04;	// Automatically Contrl
			I2CWByte(RT8973_REG_CONTROL_1, regCtrl1);
		}
		break;
	default:
		INFO("Unknown accessory detach accessory_id = %d \n",
		     pDrvData->accessory_id);
		;
	}
	pDrvData->accessory_id = ID_NONE;
	pDrvData->factory_mode = RTMUSC_FM_NONE;
	DCDT_retry = 0;
	I2CWByte(RT8973_REG_INTERRUPT_MASK, 0x0);
	if (pDrvData->operating_mode) {
		I2CWByte(RT8973_REG_MANUAL_SW1, 0);
		I2CWByte(RT8973_REG_MANUAL_SW2, 0);
	}
}

static irqreturn_t rt8973musc_irq_handler(int irq, void *data);

static void rt8973musc_work(struct work_struct *work)
{
	int32_t regIntFlag;
	int32_t regIntFlag2;
	int32_t regADC;
	int32_t regDev1, regDev2;
	struct i2c_client *pClient = pDrvData->client;
	struct rtmus_platform_data *pdata = &platform_data;

	regIntFlag = I2CRByte(RT8973_REG_INT_FLAG);
	INFO("Interrupt Flag = 0x%x\n", regIntFlag);
	if (regIntFlag & RT8973_INT_ATTACH_MASK) {
		regDev1 = I2CRByte(RT8973_REG_DEVICE_1);
		regDev2 = I2CRByte(RT8973_REG_DEVICE_2);
		if (unlikely(regIntFlag & RT8973_INT_DETACH_MASK)) {
			INFO("There is un-handled event!!\n");
			if (regDev1 == 0 && regDev2 == 0)
				do_detach_work(regIntFlag);
			else
				do_attach_work(regIntFlag, regDev1, regDev2);
		} else {
			do_attach_work(regIntFlag, regDev1, regDev2);
		}
	} else if (regIntFlag & RT8973_INT_DETACH_MASK) {
		do_detach_work(regIntFlag);
	} else {
		regIntFlag2 = I2CRByte(RT8973_REG_INT_FLAG2);
		if (regIntFlag & 0x80)	// OTP
		{
			INFO("Warning : over temperature voltage\n");
			if (likely(pdata->over_temperature_callback))
				RT8973_OTP_CALLBACK(1);
		} else if (regIntFlag & 0x10)	// OVP
		{
			INFO("Warning : VBUS over voltage\n");
			if (pdata->over_voltage_callback)
				RT8973_OVP_CALLBACK(1);
		} else if (regIntFlag & 0x08)	//DCT =1 & attach = 0
		{
			INFO("only DCT = 1, do attach work");
			regDev1 = I2CRByte(RT8973_REG_DEVICE_1);
			regDev2 = I2CRByte(RT8973_REG_DEVICE_2);
			do_attach_work(regIntFlag, regDev1, regDev2);
		} else if (pDrvData->accessory_id == ID_CHARGER && ((regIntFlag2 & 0x02) == 0x02))	// lost VBUS
		{
			regADC = I2CRByte(RT8973_REG_ADC);
			if (regADC == 0x1f)
				pDrvData->accessory_id = ID_NONE;
			else
				pDrvData->accessory_id = ID_UNKNOW;
			DCDT_retry = 0;
			I2CWByte(RT8973_REG_INTERRUPT_MASK, 0x0);
#ifdef CONFIG_BATTERY_SAMSUNG
			if (platform_data.sec_charger_callback) {
				RT8973_SEC_CHARGER_CALLBACK
				    (MUIC_RT8973_CABLE_TYPE_NONE);
			}
#else
			if (platform_data.charger_callback) {
				RT8973_CHARGER_CALLBACK(0);
			}
#endif
		} else if ((regIntFlag & 0x20) !=
			   (pDrvData->prev_int_flag & 0x20)) {
			INFO("triggered by connect = %d\n",
			     (regIntFlag & 0x20) ? 1 : 0);
		} else {
			if (pDrvData->prev_int_flag & 0x80) {
				if (likely(pdata->over_temperature_callback))
					RT8973_OTP_CALLBACK(0);
			} else if (likely(pDrvData->prev_int_flag & 0x10))	// OVP
			{
				if (pdata->over_voltage_callback)
					RT8973_OVP_CALLBACK(0);
			} else {
				INFO("Unknow event\n");
			}
		}
	}

	pDrvData->prev_int_flag = regIntFlag;

}

static irqreturn_t rt8973musc_irq_handler(int irq, void *data)
{
	struct rt8973_data *pData = (struct rt8973_data *)data;
	wake_lock_timeout(&(pData->muic_wake_lock), 1 * HZ);
	INFO("RT8973 interrupt triggered!\n");
	queue_work(rtmus_work_queue, &pData->work);
	return IRQ_HANDLED;
}

static bool init_reg_setting(void)
{
	struct i2c_client *pClient = pDrvData->client;
	int32_t regCtrl1;
	INFO("Initialize register setting!!\n");
	pDrvData->chip_id = I2CRByte(RT8973_REG_CHIP_ID);
	if (pDrvData->chip_id < 0) {
		ERR("I2C read error(reture %d)\n", pDrvData->chip_id);
		return false;
	}
	if ((pDrvData->chip_id & 0x3) != 0x02) {
		ERR("Mismatch chip id, reture %d\n", pDrvData->chip_id);
		return false;
	}
	pDrvData->operating_mode = OPERATING_MODE;
	if (platform_data.reset_callback)
		platform_data.reset_callback();
	I2CWByte(RT8973_REG_RESET, 0x01);
	msleep(RT8973_WAIT_DELAY);
	I2CWByte(RT8973_REG_INTERRUPT_MASK2, 0x02);	// Turn off POR
	regCtrl1 = I2CRByte(RT8973_REG_CONTROL_1);
	INFO("reg_ctrl1 = 0x%x\n", regCtrl1);
	if (regCtrl1 != 0xe5 && regCtrl1 != 0xc5) {
		ERR("Reg Ctrl 1 != 0xE5 or 0xC5\n");
		return false;
	}
	if (pDrvData->operating_mode != 0) {
		regCtrl1 &= (~0x04);
		I2CWByte(RT8973_REG_CONTROL_1, regCtrl1);
	}
	pDrvData->prev_int_flag = 0;	//I2CRByte(RT8973_REG_INT_FLAG);

	if (((pDrvData->chip_id & 0xf8) >> 3) == 0) {
		regCtrl1 |= (0x08);
		I2CWByte(RT8973_REG_CONTROL_1, regCtrl1);
	}
	INFO("prev_int_flag = 0x%x\n", pDrvData->prev_int_flag);

	msleep(RT8973_WAIT_DELAY);
	INFO("Set initial value OK\n");
	/*
	   INFO("GPIO %d Value = %d\n",CONFIG_RTMUSC_INT_GPIO_NUMBER,
	   gpio_get_value(CONFIG_RTMUSC_INT_GPIO_NUMBER)); */
	return true;
}

#define RT8973_DEV2_JIG_MASK                (RTMUSC_FM_BOOT_OFF_UART | RTMUSC_FM_BOOT_ON_UART | \
                                       RTMUSC_FM_BOOT_OFF_USB | RTMUSC_FM_BOOT_ON_USB)
static ssize_t adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 adc_value[] = "1C";
	u8 adc_fail = 0;

	if (pDrvData->factory_mode & RT8973_DEV2_JIG_MASK) {
		printk("adc_show JIG\n");
		return sprintf(buf, "%s\n", adc_value);
	} else {
		printk("adc_show no detect\n");
		return sprintf(buf, "%d\n", adc_fail);
	}
}

static usb_state_show_attrs(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	if (pDrvData->accessory_id == ID_USB)
		return sprintf(buf, "USB_STATE_CONFIGURED\n");
	else
		return sprintf(buf, "USB_STATE_NOTCONFIGURED\n");
}

static ssize_t usb_sel_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "PDA");
}

static DEVICE_ATTR(adc, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH /*0665 */ ,
		   adc_show, NULL);
static DEVICE_ATTR(usb_state, S_IRUGO, usb_state_show_attrs, NULL);
static DEVICE_ATTR(usb_sel, S_IRUGO, usb_sel_show_attrs, NULL);

#if defined(CONFIG_BATTERY_SAMSUNG)
static int sec_get_usb_vbus(unsigned int *level)
{
	if ((pDrvData->accessory_id == ID_USB) ||
	    ((pDrvData->accessory_id == ID_JIG) &&
	     ((pDrvData->factory_mode == RTMUSC_FM_BOOT_ON_USB) ||
	      (pDrvData->factory_mode == RTMUSC_FM_BOOT_OFF_USB)))) {
		INFO("set VBUS_HIGH\n");
		INFO(" pDrvData->accessory_id = %d, pDrvData->factory_mode = %d \n", pDrvData->accessory_id, pDrvData->factory_mode);
		*level = VBUS_HIGH;
	} else {
		INFO("set VBUS_LOW\n");
		INFO(" pDrvData->accessory_id = %d, pDrvData->factory_mode = %d \n", pDrvData->accessory_id, pDrvData->factory_mode);
		*level = VBUS_LOW;
	}
	return 0;
}
#endif

static int rt8973musc_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	int err;
	struct rt8973_data *drv_data;
	struct device *switch_dev;

	INFO("I2C is probing (%s)%d\n", id->name, (int32_t) id->driver_data);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		ERR("No Support for I2C_FUNC_SMBUS_BYTE_DATA\n");
		err = -ENODEV;
		goto i2c_check_functionality_fail;
	}

	drv_data = kzalloc(sizeof(struct rt8973_data), GFP_KERNEL);
	drv_data->client = client;
	if (client->dev.platform_data)
		memcpy(&platform_data, client->dev.platform_data,
		       sizeof(struct rtmus_platform_data));
	else
		memset(&platform_data, 0, sizeof(struct rtmus_platform_data));
	pDrvData = drv_data;
	i2c_set_clientdata(client, drv_data);
	rtmus_work_queue = create_workqueue("rt8973mus_wq");
	INIT_WORK(&drv_data->work, rt8973musc_work);

#if defined(CONFIG_BATTERY_SAMSUNG)
	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, vbus, get_vbus,
				sec_get_usb_vbus);
#endif

#if CONFIG_RTMUSC_IRQ_NUMBER<0
	client->irq = gpio_to_irq(CONFIG_RTMUSC_INT_GPIO_NUMBER);
#endif
	INFO("RT8973 irq # = %d\n", client->irq);

#ifdef CONFIG_RTMUSC_INT_CONFIG
	INFO("gpio pin # = %d\n", (int)CONFIG_RTMUSC_INT_GPIO_NUMBER);
	err = gpio_request(CONFIG_RTMUSC_INT_GPIO_NUMBER, "RT8973_EINT");	// Request for gpio pin
	if (err < 0)
		WARNING("Request GPIO %d failed\n",
			(int)CONFIG_RTMUSC_INT_GPIO_NUMBER);
	err = gpio_direction_input(CONFIG_RTMUSC_INT_GPIO_NUMBER);
	if (err < 0)
		WARNING("Set GPIO Direction to input : failed\n");
#if !defined(CONFIG_CPU_PXA988) && !defined(CONFIG_CPU_PXA1088)
	//Marvell APs don't support HW gpio Debounce-time setting routine
	err = gpio_set_debounce(CONFIG_RTMUSC_INT_GPIO_NUMBER, 1 * 1000);	// 1ms
	if (err < 0)
		WARNING("Set GPIO debounce failed (%d)\n", err);
#endif	
#endif // CONFIG_RTMUSC_INT_CONFIG
	pDrvData->irq = client->irq;

	wake_lock_init(&(drv_data->muic_wake_lock), WAKE_LOCK_SUSPEND,
		       "muic_wakelock");

#if defined(CONFIG_SPA)
	spa_external_event = spa_get_external_event_handler();
#endif

	if (!init_reg_setting()) {
		err = -EINVAL;
		goto init_fail;
	}
	err =
	    request_irq(client->irq, rt8973musc_irq_handler, RT8973_IRQF_MODE,
			DEVICE_NAME, drv_data);
	if (err < 0) {
		ERR("request_irq(%d) failed for (%d)\n", client->irq, err);
		goto request_irq_fail;
	}
	INFO("request IRQ OK...\n");
	err = enable_irq_wake(client->irq);
	if (err < 0) {
		WARNING("enable_irq_wake(%d) failed for (%d)\n", client->irq,
			err);
	}
	enable_interrupt(1);

	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");

	if (device_create_file(switch_dev, &dev_attr_adc) < 0)
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_adc.attr.name);
	if (device_create_file(switch_dev, &dev_attr_usb_state) < 0)
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_usb_state.attr.name);
	if (device_create_file(switch_dev, &dev_attr_usb_sel) < 0)
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_usb_sel.attr.name);

	pm_runtime_set_active(&client->dev);
	return 0;

request_irq_fail:
init_fail:
	wake_lock_destroy(&(drv_data->muic_wake_lock));
#ifdef CONFIG_RTMUSC_INT_CONFIG
	gpio_free(CONFIG_RTMUSC_INT_GPIO_NUMBER);
	destroy_workqueue(rtmus_work_queue);
	kfree(pDrvData);
	pDrvData = NULL;
#endif
i2c_check_functionality_fail:
	return err;
}

static int rt8973musc_remove(struct i2c_client *client)
{
	if (pDrvData) {
		disable_irq(pDrvData->irq);
		free_irq(pDrvData->irq, pDrvData);
#if CONFIG_RTMUSC_INT_CONFIG
		gpio_free(CONFIG_RTMUSC_INT_GPIO_NUMBER);
#endif // CONFIG_RTMUSC_INT_CONFIG
		wake_lock_destroy(&(pDrvData->muic_wake_lock));
		if (rtmus_work_queue)
			destroy_workqueue(rtmus_work_queue);
		kfree(pDrvData);
		pDrvData = NULL;
	}
	return 0;
}

#ifdef CONFIG_PM

static void rt8973musc_shutdown(struct i2c_client *client)
{
	struct i2c_client *pClient = client;
	if (platform_data.reset_callback)
		platform_data.reset_callback();
	I2CWByte(RT8973_REG_RESET, 0x01);
}

#endif // CONFIG_PM

static struct i2c_driver rt8973_i2c_driver = {
	.driver = {
		   .name = RT8973_DRV_NAME,
		   },
	.probe = rt8973musc_probe,
	.remove = rt8973musc_remove,
#ifdef CONFIG_PM
	.shutdown = rt8973musc_shutdown,
	.resume = NULL,
	.suspend = NULL,
#endif // CONFIG_PM
	.id_table = richtek_musc_id,
};

static int __init rt8973_init(void)
{

	int ret;
	ret = i2c_add_driver(&rt8973_i2c_driver);
	if (ret) {
		WARNING("i2c_add_driver fail\n");
		return ret;
	}

	if (pDrvData == NULL) {
		WARNING("pDrvData = NULL\n");
		ret = -EINVAL;
		goto alloc_device_fail;
	}
	rtmus_dev = platform_device_alloc(DEVICE_NAME, -1);

	if (!rtmus_dev) {
		WARNING("rtmus_dev = NULL\n");
		ret = -ENOMEM;
		goto alloc_device_fail;
	}
	ret = platform_device_add(rtmus_dev);
	if (ret) {
		WARNING("platform_device_add() failed!\n");
		goto sysfs_add_device_fail;
	}

	ret = rt_sysfs_create_files(&(rtmus_dev->dev.kobj), rt8973_attrs);
	if (ret)
		goto sysfs_create_fail;
#ifdef CONFIG_RT_SYSFS_DBG
	ret =
	    sysfs_create_group(&(rtmus_dev->dev.kobj), &rt8973_dbg_attrs_group);
	if (ret)
		goto sysfs_create_fail;
#endif //CONFIG_RT_SYSFS_DBG

	INFO("RT8973 Module initialized.\n");
	return 0;
sysfs_create_fail:
	platform_device_put(rtmus_dev);
sysfs_add_device_fail:
	platform_device_unregister(rtmus_dev);
alloc_device_fail:
	i2c_del_driver(&rt8973_i2c_driver);
	ERR("RT8973 Module initialization failed.\n");
	return ret;
}

static void __exit rt8973_exit(void)
{
	if (rtmus_dev) {
		platform_device_put(rtmus_dev);
		platform_device_unregister(rtmus_dev);

	}
	i2c_del_driver(&rt8973_i2c_driver);
	INFO("RT8973 Module deinitialized.\n");
}

#if defined(CONFIG_SPA)
void rt8973_spa_usb_attached()
{
	if (spa_external_event) {
		spa_external_event(SPA_CATEGORY_DEVICE,
				   SPA_DEVICE_EVENT_USB_ATTACHED);
		INFO("rt8973_spa_usb_attached\n");
	}
}

void rt8973_spa_usb_detached()
{
	if (spa_external_event) {
		spa_external_event(SPA_CATEGORY_DEVICE,
				   SPA_DEVICE_EVENT_USB_DETACHED);
		INFO("rt8973_spa_usb_detached\n");
	}
}

void rt8973_spa_ta_attached()
{
	if (spa_external_event) {
		spa_external_event(SPA_CATEGORY_DEVICE,
				   SPA_DEVICE_EVENT_TA_ATTACHED);
		INFO("rt8973_spa_ta_attached\n");
	}
}

void rt8973_spa_ta_detached()
{
	if (spa_external_event) {
		spa_external_event(SPA_CATEGORY_DEVICE,
				   SPA_DEVICE_EVENT_TA_DETACHED);
		INFO("rt8973_spa_ta_detached\n");
	}
}

void rt8973_spa_jig_attached()
{
	if (spa_external_event) {
		spa_external_event(SPA_CATEGORY_DEVICE,
				   SPA_DEVICE_EVENT_JIG_ATTACHED);
		INFO("rt8973_spa_jig_attached\n");
	}
}

void rt8973_spa_jig_detached()
{
	if (spa_external_event) {
		spa_external_event(SPA_CATEGORY_DEVICE,
				   SPA_DEVICE_EVENT_JIG_DETACHED);
		INFO("rt8973_spa_jig_detached\n");
	}
}

void rt8973_spa_ovp_detected()
{
	if (spa_external_event) {
		spa_external_event(SPA_CATEGORY_BATTERY,
				   SPA_BATTERY_EVENT_OVP_CHARGE_STOP);
		INFO("rt8973_spa_ovp_detected\n");
	}
}

void rt8973_spa_ovp_released()
{
	if (spa_external_event) {
		spa_external_event(SPA_CATEGORY_BATTERY,
				   SPA_BATTERY_EVENT_OVP_CHARGE_RESTART);
		INFO("rt8973_spa_ovp_released\n");
	}
}
#endif

MODULE_AUTHOR("Patrick Chang <weichung.chang@gmail.com>");
MODULE_DESCRIPTION("Richtek micro USB switch device diver");
MODULE_LICENSE("GPL");
late_initcall(rt8973_init);
module_exit(rt8973_exit);
