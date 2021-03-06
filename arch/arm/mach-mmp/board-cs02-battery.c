/* arch/arm/mach-mmp/board-wilcox-battery.c
 *
 * Copyright (C) 2012 Samsung Electronics Co, Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/switch.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/regulator/machine.h>
#include <linux/platform_device.h>
#include <mach/mfp-pxa986-cs02.h>

#include <linux/battery/sec_battery.h>
#include <linux/battery/sec_fuelgauge.h>
#include <linux/battery/sec_charger.h>
#include <linux/gpio_event.h>
#if defined(CONFIG_RT8973)
#include <linux/platform_data/rtmusc.h>
#endif


#include "board-cs02.h"
#include "common.h"

#if defined(CONFIG_BATTERY_SAMSUNG)

#define SEC_CHARGER_I2C_ID	2
#define SEC_FUELGAUGE_I2C_ID	6

#define SEC_BATTERY_PMIC_NAME ""

#define TA_ADC_LOW              800
#define TA_ADC_HIGH             2200

/* cable state */
bool is_cable_attached;
unsigned int lpcharge;

static struct s3c_adc_client *temp_adc_client;

static sec_bat_adc_table_data_t lt02_temp_table[] = {
        {404, 600},
        {548, 500},
        {696, 420},
        {725, 400},
        {927, 300},
        {1144, 200},
        {1377, 100},
        {1460, 50},
        {1582, 0},
        {1647, -50},
        {1738, -100},
};

static sec_bat_adc_region_t cable_adc_value_table[] = {
	{ 0,    0 },    /* POWER_SUPPLY_TYPE_UNKNOWN */
	{ 0,    500 },  /* POWER_SUPPLY_TYPE_BATTERY */
	{ 0,    0 },    /* POWER_SUPPLY_TYPE_UPS */
	{ 1000, 1500 }, /* POWER_SUPPLY_TYPE_MAINS */
	{ 0,    0 },    /* POWER_SUPPLY_TYPE_USB */
	{ 0,    0 },    /* POWER_SUPPLY_TYPE_OTG */
	{ 0,    0 },    /* POWER_SUPPLY_TYPE_DOCK */
	{ 0,    0 },    /* POWER_SUPPLY_TYPE_MISC */
};

static sec_charging_current_t charging_current_table[] = {
	{0,     0,      0,      0},     /* POWER_SUPPLY_TYPE_UNKNOWN */
	{0,     0,      0,      0},     /* POWER_SUPPLY_TYPE_BATTERY */
	{0,     0,      0,      0},     /* POWER_SUPPLY_TYPE_UPS */
	{750,  750,   100,    1800},     /* POWER_SUPPLY_TYPE_MAINS */
	{550,   500,    100,    1800},     /* POWER_SUPPLY_TYPE_USB */
	{550,   500,    100,    1800},     /* POWER_SUPPLY_TYPE_USB_DCP */
	{550,   500,    100,    1800},     /* POWER_SUPPLY_TYPE_USB_CDP */
	{550,   500,    100,    1800},     /* POWER_SUPPLY_TYPE_USB_ACA */
	{550,   500,    100,   1800},     /* POWER_SUPPLY_TYPE_MISC */
	{0,     0,      0,      0},     /* POWER_SUPPLY_TYPE_CARDOCK */
	{550,   500,    100,    1800},     /* POWER_SUPPLY_TYPE_WPC */
	{750,  750,   100,    1800},     /* POWER_SUPPLY_TYPE_UARTOFF */
};

/* unit: seconds */
static int polling_time_table[] = {
	2,     /* BASIC */
	2,     /* CHARGING */
	10,     /* DISCHARGING */
	10,     /* NOT_CHARGING */
	300,    /* SLEEP */
};

static struct power_supply *charger_supply;
static bool is_jig_on;

static bool sec_bat_gpio_init(void)
{
	return true;
}

static bool sec_fg_gpio_init(void)
{
	return true;
}

static bool sec_chg_gpio_init(void)
{
	int ret;

	ret = gpio_request(mfp_to_gpio(GPIO098_GPIO_98), "bq24157_CD");
	if(ret)
		return false;

	ret = gpio_request(mfp_to_gpio(GPIO008_GPIO_8), "charger-irq");
	if(ret)
		return false;

	gpio_direction_input(mfp_to_gpio(GPIO008_GPIO_8));

	return true;
}

/* Get LP charging mode state */

static int battery_get_lpm_state(char *str)
{
	get_option(&str, &lpcharge);
	pr_info("%s: Low power charging mode: %d\n", __func__, lpcharge);

	return lpcharge;
}
__setup("lpcharge=", battery_get_lpm_state);

static bool sec_bat_is_lpm(void)
{
	return lpcharge == 1 ? true : false;
}

void check_jig_status(int status)
{
	if (status) {
		pr_info("%s: JIG On so reset fuel gauge capacity\n", __func__);
		is_jig_on = true;
	}

}

static bool sec_bat_check_jig_status(void)
{
	return is_jig_on;
}

int current_cable_type = POWER_SUPPLY_TYPE_BATTERY;
u8 attached_cable;


void sec_charger_cb(u8 cable_type)
{

	pr_info("%s: cable type (0x%02x)\n", __func__, cable_type);

	union power_supply_propval value;
	struct power_supply *psy = power_supply_get_by_name("battery");

	attached_cable = cable_type;

	switch (cable_type) {
	case MUIC_RT8973_CABLE_TYPE_NONE:
		current_cable_type = POWER_SUPPLY_TYPE_BATTERY;
		break;
	case MUIC_RT8973_CABLE_TYPE_USB:
		current_cable_type = POWER_SUPPLY_TYPE_USB;
		break;
	case MUIC_RT8973_CABLE_TYPE_TA:
		current_cable_type = POWER_SUPPLY_TYPE_MAINS;
		break;
	case MUIC_RT8973_CABLE_TYPE_OTG:
		goto skip;
	case MUIC_RT8973_CABLE_TYPE_JIG_UART_OFF:
		if (!gpio_get_value(mfp_to_gpio(GPIO008_GPIO_8))) {
			pr_info("%s cable type POWER_SUPPLY_TYPE_UARTOFF\n", __func__);
			current_cable_type = POWER_SUPPLY_TYPE_UARTOFF;
		}
		else {
		current_cable_type = POWER_SUPPLY_TYPE_BATTERY;
		}
		break;
	case MUIC_RT8973_CABLE_TYPE_JIG_USB_ON:
	case MUIC_RT8973_CABLE_TYPE_JIG_USB_OFF:
		current_cable_type = POWER_SUPPLY_TYPE_USB;
		break;
	case MUIC_RT8973_CABLE_TYPE_0x1A:
	case MUIC_RT8973_CABLE_TYPE_TYPE1_CHARGER:
		current_cable_type = POWER_SUPPLY_TYPE_MAINS;
		break;
	case MUIC_RT8973_CABLE_TYPE_0x15:
		current_cable_type = POWER_SUPPLY_TYPE_MISC;
		break;
	default:
		pr_err("%s: invalid type for charger:%d\n",
			__func__, cable_type);
		current_cable_type = POWER_SUPPLY_TYPE_UNKNOWN;
		goto skip;
	}

	if (!psy || !psy->set_property)
		pr_err("%s: fail to get battery psy\n", __func__);
	else {
		value.intval = current_cable_type;
		psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	}

skip:
	return 0;
}

/*
int extended_cable_type;

static int sec_bat_get_cable_from_extended_cable_type(
	int input_extended_cable_type)
{
	int cable_main, cable_sub, cable_power;
	int cable_type = POWER_SUPPLY_TYPE_UNKNOWN;
	union power_supply_propval value;
	int charge_current_max = 0, charge_current = 0;

	cable_main = GET_MAIN_CABLE_TYPE(input_extended_cable_type);
	if (cable_main != POWER_SUPPLY_TYPE_UNKNOWN)
		extended_cable_type = (extended_cable_type &
			~(int)ONLINE_TYPE_MAIN_MASK) |
			(cable_main << ONLINE_TYPE_MAIN_SHIFT);
	cable_sub = GET_SUB_CABLE_TYPE(input_extended_cable_type);
	if (cable_sub != ONLINE_SUB_TYPE_UNKNOWN)
		extended_cable_type = (extended_cable_type &
			~(int)ONLINE_TYPE_SUB_MASK) |
			(cable_sub << ONLINE_TYPE_SUB_SHIFT);
	cable_power = GET_POWER_CABLE_TYPE(input_extended_cable_type);
	if (cable_power != ONLINE_POWER_TYPE_UNKNOWN)
		extended_cable_type = (extended_cable_type &
			~(int)ONLINE_TYPE_PWR_MASK) |
			(cable_power << ONLINE_TYPE_PWR_SHIFT);

	switch (cable_main) {
	case POWER_SUPPLY_TYPE_CARDOCK:
		switch (cable_power) {
		case ONLINE_POWER_TYPE_BATTERY:
			cable_type = POWER_SUPPLY_TYPE_BATTERY;
			break;
		case ONLINE_POWER_TYPE_TA:
			switch (cable_sub) {
			case ONLINE_SUB_TYPE_MHL:
				cable_type = POWER_SUPPLY_TYPE_USB;
				break;
			case ONLINE_SUB_TYPE_AUDIO:
			case ONLINE_SUB_TYPE_DESK:
			case ONLINE_SUB_TYPE_SMART_NOTG:
			case ONLINE_SUB_TYPE_KBD:
				cable_type = POWER_SUPPLY_TYPE_MAINS;
				break;
			case ONLINE_SUB_TYPE_SMART_OTG:
				cable_type = POWER_SUPPLY_TYPE_CARDOCK;
				break;
			}
			break;
		case ONLINE_POWER_TYPE_USB:
			cable_type = POWER_SUPPLY_TYPE_USB;
			break;
		default:
			cable_type = current_cable_type;
			break;
		}
		break;
	case POWER_SUPPLY_TYPE_MISC:
		switch (cable_sub) {
		case ONLINE_SUB_TYPE_MHL:
			switch (cable_power) {
			case ONLINE_POWER_TYPE_BATTERY:
				cable_type = POWER_SUPPLY_TYPE_BATTERY;
				break;
			case ONLINE_POWER_TYPE_TA:
				cable_type = POWER_SUPPLY_TYPE_MAINS;
				charge_current_max = 700;
				charge_current = 700;
				break;
			case ONLINE_POWER_TYPE_USB:
				cable_type = POWER_SUPPLY_TYPE_USB;
				charge_current_max = 300;
				charge_current = 300;
				break;
			}
			break;
		default:
			cable_type = cable_main;
			break;
		}
		break;
	default:
		cable_type = cable_main;
		break;
	}

	if (charge_current_max == 0) {
		charge_current_max =
			charging_current_table[cable_type].input_current_limit;
		charge_current =
			charging_current_table[cable_type].
			fast_charging_current;
	}
	value.intval = charge_current_max;
	psy_do_property(sec_battery_pdata.charger_name, set,
			POWER_SUPPLY_PROP_CURRENT_MAX, value);
	value.intval = charge_current;
	psy_do_property(sec_battery_pdata.charger_name, set,
			POWER_SUPPLY_PROP_CURRENT_AVG, value);
	return cable_type;
}
*/


static int sec_bat_check_cable_callback(void)
{
	int ta_nconnected;
	union power_supply_propval value;
	struct power_supply *psy = power_supply_get_by_name("battery");

	ta_nconnected = gpio_get_value(mfp_to_gpio(GPIO008_GPIO_8));

	pr_info("%s : ta_nconnected : %d\n", __func__, ta_nconnected);

	if (ta_nconnected)
		current_cable_type = POWER_SUPPLY_TYPE_BATTERY;
	else
		current_cable_type = POWER_SUPPLY_TYPE_MAINS;

	if(attached_cable == MUIC_RT8973_CABLE_TYPE_0x1A) {
		if (ta_nconnected)
			current_cable_type = POWER_SUPPLY_TYPE_BATTERY;
		else
			current_cable_type = POWER_SUPPLY_TYPE_MISC;
	} else
		return current_cable_type;

	if (!psy || !psy->set_property)
		pr_err("%s: fail to get battery psy\n", __func__);
	else {
		value.intval = current_cable_type;
		psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	}

	return current_cable_type;
}

static void sec_bat_initial_check(void)
{
	union power_supply_propval value;

	sec_bat_check_cable_callback();

//	current_cable_type = POWER_SUPPLY_TYPE_MAINS;

	if (POWER_SUPPLY_TYPE_BATTERY < current_cable_type) {
#if 0
		value.intval = current_cable_type;
		psy_do_property("battery", set,
			POWER_SUPPLY_PROP_ONLINE, value);
#endif
	} else {
		psy_do_property("sec-charger", get,
				POWER_SUPPLY_PROP_ONLINE, value);
		if (value.intval == POWER_SUPPLY_TYPE_WPC) {
			value.intval =
				POWER_SUPPLY_TYPE_WPC;
			psy_do_property("battery", set,
					POWER_SUPPLY_PROP_ONLINE, value);
		} else {
			value.intval =
				POWER_SUPPLY_TYPE_BATTERY;
				psy_do_property("sec-charger", set,
					POWER_SUPPLY_PROP_ONLINE, value);
		}
	}
}

static bool sec_bat_check_cable_result_callback(int cable_type)
{
	bool ret = true;
	current_cable_type = cable_type;

	switch (cable_type) {
	case POWER_SUPPLY_TYPE_USB:
		pr_info("%s set vbus applied\n",
				__func__);
		break;
	case POWER_SUPPLY_TYPE_BATTERY:
		pr_info("%s set vbus cut\n",
				__func__);
		break;
	case POWER_SUPPLY_TYPE_MAINS:
		break;
	default:
		pr_err("%s cable type (%d)\n",
				__func__, cable_type);
		ret = false;
		break;
	}
	/* omap4_tab3_tsp_ta_detect(cable_type); */

	return ret;
}

/* callback for battery check
 * return : bool
 * true - battery detected, false battery NOT detected
 */
static bool sec_bat_check_callback(void) { return true; }
static bool sec_bat_check_result_callback(void) { return true; }

/* callback for OVP/UVLO check
 * return : int
 * battery health
 */
static int sec_bat_ovp_uvlo_callback(void)
{
	int health;
	health = POWER_SUPPLY_HEALTH_GOOD;

	return health;
}

static bool sec_bat_ovp_uvlo_result_callback(int health) { return true; }

/*
 * val.intval : temperature
 */
static bool sec_bat_get_temperature_callback(
		enum power_supply_property psp,
		union power_supply_propval *val) { return true; }

static bool sec_fg_fuelalert_process(bool is_fuel_alerted) { return true; }

static bool sec_bat_adc_none_init(struct platform_device *pdev) { return true; }
static bool sec_bat_adc_none_exit(void) { return true; }
static int sec_bat_adc_none_read(unsigned int channel) { return 0; }

static bool sec_bat_adc_ap_init(struct platform_device *pdev) { return true; }
static bool sec_bat_adc_ap_exit(void) { return true; }
static int sec_bat_adc_ap_read(unsigned int channel) { return 0; }

static bool sec_bat_adc_ic_init(struct platform_device *pdev) { return true; }
static bool sec_bat_adc_ic_exit(void) { return true; }
static int sec_bat_adc_ic_read(unsigned int channel) { return 0; }

sec_battery_platform_data_t sec_battery_pdata = {
	/* NO NEED TO BE CHANGED */
	.initial_check = sec_bat_initial_check,
	.bat_gpio_init = sec_bat_gpio_init,
	.fg_gpio_init = sec_fg_gpio_init,
	.chg_gpio_init = sec_chg_gpio_init,

	.is_lpm = sec_bat_is_lpm,
	.check_jig_status = sec_bat_check_jig_status,
	.check_cable_callback =
		sec_bat_check_cable_callback,
//	.get_cable_from_extended_cable_type =
//	        sec_bat_get_cable_from_extended_cable_type,
	.check_cable_result_callback =
		sec_bat_check_cable_result_callback,
	.check_battery_callback =
		sec_bat_check_callback,
	.check_battery_result_callback =
		sec_bat_check_result_callback,
	.ovp_uvlo_callback = sec_bat_ovp_uvlo_callback,
	.ovp_uvlo_result_callback =
		sec_bat_ovp_uvlo_result_callback,
	.fuelalert_process = sec_fg_fuelalert_process,
	.get_temperature_callback =
		sec_bat_get_temperature_callback,

	.adc_api[SEC_BATTERY_ADC_TYPE_NONE] = {
		.init = sec_bat_adc_none_init,
		.exit = sec_bat_adc_none_exit,
		.read = sec_bat_adc_none_read
	},
	.adc_api[SEC_BATTERY_ADC_TYPE_AP] = {
		.init = sec_bat_adc_ap_init,
		.exit = sec_bat_adc_ap_exit,
//		.read = sec_bat_adc_ap_read
	},
	.adc_api[SEC_BATTERY_ADC_TYPE_IC] = {
		.init = sec_bat_adc_ic_init,
		.exit = sec_bat_adc_ic_exit,
		.read = sec_bat_adc_ic_read
	},
	.cable_adc_value = cable_adc_value_table,
	.charging_current = charging_current_table,
	.polling_time = polling_time_table,
	/* NO NEED TO BE CHANGED */

	.pmic_name = SEC_BATTERY_PMIC_NAME,

	.adc_check_count = 5,

	.adc_type = {
		SEC_BATTERY_ADC_TYPE_NONE,	/* CABLE_CHECK */
		SEC_BATTERY_ADC_TYPE_NONE,      /* BAT_CHECK */
		SEC_BATTERY_ADC_TYPE_NONE,	/* TEMP */
		SEC_BATTERY_ADC_TYPE_NONE,	/* TEMP_AMB */
		SEC_BATTERY_ADC_TYPE_NONE,      /* FULL_CHECK */
	},

	/* Battery */
	.vendor = "SDI SDI",
	.technology = POWER_SUPPLY_TECHNOLOGY_LION,
	.bat_polarity_ta_nconnected = 1,        /* active HIGH */
	.bat_irq_attr = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.cable_check_type = SEC_BATTERY_CABLE_CHECK_INT,
	.cable_source_type = //SEC_BATTERY_CABLE_SOURCE_CALLBACK,
	SEC_BATTERY_CABLE_SOURCE_EXTERNAL,
//	SEC_BATTERY_CABLE_SOURCE_EXTENDED,

	.event_check = false,
	.event_waiting_time = 60,

	/* Monitor setting */
	.polling_type = SEC_BATTERY_MONITOR_ALARM,
	.monitor_initial_count = 3,

	/* Battery check */
	.battery_check_type = SEC_BATTERY_CHECK_FUELGAUGE,
	.check_count = 1,

	/* Battery check by ADC */
	.check_adc_max = 600,
	.check_adc_min = 50,

	/* OVP/UVLO check */
	.ovp_uvlo_check_type = SEC_BATTERY_OVP_UVLO_CHGPOLLING,

	.thermal_source = SEC_BATTERY_THERMAL_SOURCE_FG,
	.temp_check_type = SEC_BATTERY_TEMP_CHECK_TEMP,

        .temp_check_count = 1,
        .temp_high_threshold_event = 600,
        .temp_high_recovery_event = 400,
        .temp_low_threshold_event = -80,
        .temp_low_recovery_event = 0,
        .temp_high_threshold_normal = 600,
        .temp_high_recovery_normal = 400,
        .temp_low_threshold_normal = -80,
        .temp_low_recovery_normal = 0,
        .temp_high_threshold_lpm = 600,
        .temp_high_recovery_lpm = 400,
        .temp_low_threshold_lpm = -80,
        .temp_low_recovery_lpm = 0,

	.full_check_type = SEC_BATTERY_FULLCHARGED_SOC,
	.full_check_type_2nd = SEC_BATTERY_FULLCHARGED_TIME,
	.full_check_count = 1,
	/* .full_check_adc_1st = 26500, */
	/*.full_check_adc_2nd = 25800, */
	.chg_polarity_full_check = 1,
	.full_condition_type = SEC_BATTERY_FULL_CONDITION_SOC,
	.full_condition_soc = 100,
	.full_condition_ocv = 4300,

	.recharge_condition_type =
                SEC_BATTERY_RECHARGE_CONDITION_AVGVCELL,

	.recharge_condition_soc = 98,
	.recharge_condition_avgvcell = 4280,
	.recharge_condition_vcell = 4300,

	.charging_total_time = 6 * 60 * 60,
        .recharging_total_time = 90 * 60,
        .charging_reset_time = 10 * 60,

	/* Fuel Gauge */
	.fg_irq_attr = IRQF_TRIGGER_FALLING,
	.fuel_alert_soc = 1,
	.repeated_fuelalert = false,
	.capacity_calculation_type =
		SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE,
	.capacity_max = 1000,
	.capacity_min = 0,
	.capacity_max_margin = 30,

	/* Charger */
	.chg_polarity_en = 0,   /* active LOW charge enable */
	.chg_polarity_status = 0,
	.chg_irq_attr = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,

	.chg_float_voltage = 4360,
};

static struct platform_device sec_device_battery = {
	.name = "sec-battery",
	.id = -1,
	.dev = {
		.platform_data = &sec_battery_pdata,
	},
};

static struct i2c_gpio_platform_data i2c_sec_charger_bus_data = {
	.sda_pin = mfp_to_gpio(GPIO017_GPIO_17),
	.scl_pin = mfp_to_gpio(GPIO016_GPIO_16),
	.udelay  = 3,
	.timeout = 100,
};

static struct platform_device i2c_sec_charger_bus_device = {
	.name		= "i2c-gpio",
	.id		= 8, /* pxa92x-i2c are bus 0, 1 so start at 2 */
	.dev = {
		.platform_data = &i2c_sec_charger_bus_data,
	}
};

static struct i2c_board_info sec_brdinfo_charger[] __initdata = {
	{
		I2C_BOARD_INFO("sec-charger",
				SEC_CHARGER_I2C_SLAVEADDR),
		.platform_data = &sec_battery_pdata,
	},
};

static struct pxa_i2c_board_gpio sec_charger_i2c_gpio[] = {
	{
		.type = "sec-charger",
		.gpio = mfp_to_gpio(GPIO008_GPIO_8),
	},
};

static struct platform_device *sec_battery_devices[] __initdata = {
	&sec_device_battery,
	&i2c_sec_charger_bus_device,
};

static void charger_gpio_init(void)
{
	pxa_init_i2c_gpio_irq(ARRAY_AND_SIZE(sec_charger_i2c_gpio),
					ARRAY_AND_SIZE(sec_brdinfo_charger));

	sec_battery_pdata.chg_irq = gpio_to_irq(mfp_to_gpio(GPIO008_GPIO_8));
}

void __init pxa986_cs02_battery_init(void)
{
	pr_info("%s: cs02 charger init\n", __func__);

	charger_gpio_init();

	platform_add_devices(sec_battery_devices,
		ARRAY_SIZE(sec_battery_devices));

	i2c_register_board_info(8, ARRAY_AND_SIZE(sec_brdinfo_charger));
}
#endif
