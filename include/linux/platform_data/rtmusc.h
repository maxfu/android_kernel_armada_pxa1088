/* include/linux/platform_data/rtmusc.h
 * Driver to Richtek RT8969 micro USB switch device
 *
 * Copyright (C) 2012
 * Author: Patrick Chang <weichung.chang@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __RTMUSC_H_
#define __RTMUSC_H_
#include <linux/types.h>

#define RTMUSC_FM_NONE          0x00
#define RTMUSC_FM_BOOT_ON_UART  0x01
#define RTMUSC_FM_BOOT_OFF_UART 0x02
#define RTMUSC_FM_BOOT_ON_USB   0x03
#define RTMUSC_FM_BOOT_OFF_USB  0x04


struct rtmus_platform_data {
        void (*usb_callback) (uint8_t attached);
        void (*uart_callback) (uint8_t attached);
        void (*charger_callback) (uint8_t attached);
        void (*jig_callback) (uint8_t attached,uint8_t factory_mode);
        void (*over_temperature_callback)(uint8_t detected);
        void (*charging_complete_callback)(void);
        void (*over_voltage_callback)(uint8_t detected);
		void (*otg_callback)(uint8_t attached);
		void (*reset_callback)(void);
#ifdef CONFIG_BATTERY_SAMSUNG
		void (*sec_charger_callback)(uint8_t cable_type);
#endif
};

#ifdef CONFIG_BATTERY_SAMSUNG
enum {
	MUIC_RT8973_CABLE_TYPE_NONE = 0,
	MUIC_RT8973_CABLE_TYPE_UART,	  		//adc 0x16	
	MUIC_RT8973_CABLE_TYPE_USB,				//adc 0x1f (none id)
	MUIC_RT8973_CABLE_TYPE_OTG,				//adc 0x00, regDev1&0x01

	/* TA Group */
	MUIC_RT8973_CABLE_TYPE_TA,				//adc 0x1f (none id)
	MUIC_RT8973_CABLE_TYPE_0x15,			//adc 0x15	
	MUIC_RT8973_CABLE_TYPE_TYPE1_CHARGER,	//adc 0x17 (id : 200k)
	MUIC_RT8973_CABLE_TYPE_0x1A,			//adc 0x1A	

	/* JIG Group */
	MUIC_RT8973_CABLE_TYPE_JIG_USB_OFF,		//adc 0x18	
	MUIC_RT8973_CABLE_TYPE_JIG_USB_ON,		//adc 0x19
	MUIC_RT8973_CABLE_TYPE_JIG_UART_OFF,	//adc 0x1C
	MUIC_RT8973_CABLE_TYPE_JIG_UART_ON,		//adc 0x1D
};
#endif

#if defined(CONFIG_SPA)
void rt8973_spa_usb_attached();
void rt8973_spa_usb_detached();
void rt8973_spa_ta_attached();
void rt8973_spa_ta_detached();
void rt8973_spa_jig_attached();
void rt8973_spa_jig_detached();
void rt8973_spa_ovp_detected();
void rt8973_spa_ovp_released();
#endif

#endif /* __RTMUSC_H_ */
