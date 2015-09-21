/*
 *  rt9455_charger.c
 *  Richtek RT9455 Charger Driver
 *
 *  Copyright (C) 2013 Richtek Electronics
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
//#define DEBUG

#include <linux/battery/sec_charger.h>

struct rt9455_chip_init_data rt9455_chip_init_data = {
	.irq_mask = {
		.mask1 = {
			.bitfield = {
				.BATABM = 0,
				.VINOVPIM = 0,
				.TSDM = 0,
			},
		},
		.mask2 = {
			.bitfield = {
				.CHDPMIM = 0,
				.CHTREGIM = 0,
				.CH32MIM = 0,
				.CHRCHGIM = 0,
				.CHTERMIM = 0,
				.CHBATOVIM = 0,
				.CHRVPIM = 0,
			},
		},
		.mask3 = {
			.bitfield = {
				.BST32SIM = 0,
				.BSTLOWVIM = 0,
				.BSTOLIM = 0,
				.BSTVIMOVIM = 0,
			},
		},
	},
	.init_ctrlval = {
		.ctrl2 = {
			.bitfield = {
				.OPA_MODE = OPA_CHARGER_MODE,
				.HZ_BIT = 0,
				.IAICR_INT = IAICR_INTERNAL,
				.TE = 0,
				.HIGH_OCP = OCP_2P5A,
				.SEL_SWFREQ = SEL_FREQ_3MHz,
				.IAICR = IAICR_LIMIT_500MA,
			},
		},
		.ctrl3 = {
			.bitfield = {
				.OTG_EN = 0,
				.OTG_PL = OTGPL_ACTIVE_HIGH,
				.VOREG = 0x19,
			},
		},
		.ctrl5 = {
			.bitfield = {
				.IEOC = IEOC_20P,
				.IPREC = IPREC_60MA,
				.VDPM = VDPM_DISABLE,
				.TMR_EN = 1,
			},
		},
		.ctrl6 = {
			.bitfield = {
				.VPREC = VPREC_2P4V,
				.ICHRG = ICHRG_20MV,
			},
		},
		.ctrl7 = {
			.bitfield = {
				.VMREG = 0x0F,
				.CHG_EN = 1,
				.BATD_EN = 0,
			},
		},
	},
};

static inline int rt9455_read_device(struct i2c_client *i2c,
				      int reg, int bytes, void *dest)
{
	int ret;
	if (bytes > 1)
		ret = i2c_smbus_read_i2c_block_data(i2c, reg, bytes, dest);
	else {
		ret = i2c_smbus_read_byte_data(i2c, reg);
		if (ret < 0)
			return ret;
		*(unsigned char *)dest = (unsigned char)ret;
	}
	return ret;
}

static int rt9455_reg_read(struct i2c_client *i2c, int reg)
{
	int ret;
	RT_DBG("I2C Read (client : 0x%x) reg = 0x%x\n",
           (unsigned int)i2c,(unsigned int)reg);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	return ret;
}

static int rt9455_reg_write(struct i2c_client *i2c, int reg, unsigned char data)
{
	int ret;
	RT_DBG("I2C Write (client : 0x%x) reg = 0x%x, data = 0x%x\n",
           (unsigned int)i2c,(unsigned int)reg,(unsigned int)data);
	ret = i2c_smbus_write_byte_data(i2c, reg, data);
	return ret;
}

static int rt9455_assign_bits(struct i2c_client *i2c, int reg,
		unsigned char mask, unsigned char data)
{
	unsigned char value;
	int ret;
	ret = rt9455_read_device(i2c, reg, 1, &value);
	if (ret < 0)
		goto out;
	value &= ~mask;
	value |= (data&mask);
	ret = i2c_smbus_write_byte_data(i2c,reg,value);
out:
	return ret;
}

static int rt9455_set_bits(struct i2c_client *i2c, int reg,
		unsigned char mask)
{
	return rt9455_assign_bits(i2c,reg,mask,mask);
}

static int rt9455_clr_bits(struct i2c_client *i2c, int reg,
		unsigned char mask)
{
	return rt9455_assign_bits(i2c,reg,mask,0);
}

static void rt9455_test_read(struct i2c_client *client)
{
	u8 data = 0;
	u32 addr = 0;
	for (addr = 0; addr < RT9455_REG_MAX; addr++) {
		data = rt9455_reg_read(client, addr);
		dev_dbg(&client->dev,
			"RT9455 addr : 0x%02x data : 0x%02x\n", addr, data);
	}
}

static void rt9455_read_regs(struct i2c_client *client, char *str)
{
	u8 data = 0;
	int addr = 0;

	for (addr = 0; addr < RT9455_REG_MAX; addr++) {
		data = rt9455_reg_read(client, addr);
		sprintf(str+strlen(str), "0x%x, ", data);
	}
}

static int rt9455_chip_reset(struct i2c_client *client)
{
	struct rt9455_chip_init_data *init_data = &rt9455_chip_init_data;
	int ret;

	RT_DBG("\n");
	//Before the IC reset, inform the customer
	if (init_data->callbacks)
		init_data->callbacks->reset_callback(RT9455_EVENT_BEFORE_RST);

	ret = rt9455_set_bits(client, RT9455_REG_CTRL4, RT9455_RST_MASK);
	if (ret < 0)
		return -EIO;

	if (init_data->callbacks)
		init_data->callbacks->reset_callback(RT9455_EVENT_AFTER_RST);


	return 0;
}

static int rt9455_chip_reg_init(struct i2c_client *client)
{
	struct rt9455_chip_init_data *init_data = &rt9455_chip_init_data;

	RT_DBG("\n");
	//ctrl value
	RT_DBG("write_init_ctrlval\n");
	rt9455_reg_write(client, RT9455_REG_CTRL2, init_data->init_ctrlval.ctrl2.val);
	rt9455_reg_write(client, RT9455_REG_CTRL3, init_data->init_ctrlval.ctrl3.val);
	rt9455_reg_write(client, RT9455_REG_CTRL5, init_data->init_ctrlval.ctrl5.val);
	rt9455_reg_write(client, RT9455_REG_CTRL6, init_data->init_ctrlval.ctrl6.val);
	rt9455_reg_write(client, RT9455_REG_CTRL7, init_data->init_ctrlval.ctrl7.val);
	// irq mask value
	RT_DBG("write_irq_mask\n");
	rt9455_reg_write(client, RT9455_REG_MASK1, init_data->irq_mask.mask1.val);
	rt9455_reg_write(client, RT9455_REG_MASK2, init_data->irq_mask.mask2.val);
	rt9455_reg_write(client, RT9455_REG_MASK3, init_data->irq_mask.mask3.val);
	return 0;
}

static int rt9455_get_charging_status(struct i2c_client *i2c)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	int ret;

	ret = rt9455_reg_read(i2c, RT9455_REG_CTRL1);
	if (ret<0)
		dev_err(&i2c->dev, "%s fail\n", __func__);

	RT_DBG("CTRL1 value %02x\n", ret);
	switch (ret&0x30)
	{
		case 0x00:
			status = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case 0x10:
			status = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case 0x20:
			status = POWER_SUPPLY_STATUS_FULL;
			break;
		case 0x30:
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
	}
	return status;
}

static int rt9455_get_charge_type(struct i2c_client *i2c)
{
	int status = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	int ret;

	ret = rt9455_reg_read(i2c, RT9455_REG_CTRL1);
	if (ret<0)
		dev_err(&i2c->dev, "%s fail\n", __func__);

	RT_DBG("CTRL1 value %02x\n", ret);
	switch (ret&0x30)
	{
		case 0x10:
			status = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		default:
			status = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
	}
	return status;
}

static int rt9455_get_charging_health(struct i2c_client *i2c)
{
	struct rt9455_chip_init_data *init_data = &rt9455_chip_init_data;
	int status = POWER_SUPPLY_HEALTH_GOOD;
	unsigned char irq_stat[3] = {0};

	RT_DBG("\n");
	if (rt9455_read_device(i2c, RT9455_REG_IRQ1, 3, irq_stat) >= 0)
	{
		if (irq_stat[0])
		{
			//RT9455_EVENT_TSDI, RT9455_EVENT_VINOVPI
			if (irq_stat[0] & RT9455_EVENT_TSDI)
				status = POWER_SUPPLY_HEALTH_OVERHEAT;
			if (irq_stat[0] & RT9455_EVENT_VINOVPI)
				status = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			if (init_data->callbacks)
				init_data->callbacks->general_callback(irq_stat[0]);
		}
		if (irq_stat[1])
		{
			// RT9455_EVENT_CHRVPI
			if (irq_stat[1] & RT9455_EVENT_CHRVPI)
				status = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
			if (init_data->callbacks)
				init_data->callbacks->charger_callback(irq_stat[1]);
		}
		if (irq_stat[2])
		{
			if (init_data->callbacks)
				init_data->callbacks->boost_callback(irq_stat[2]);
		}
	}
	else
	{
		dev_err(&i2c->dev, "%s: read irq status fail\n", __func__);
	}

	return status;
}

static void rt9455_charger_otg_control(
				struct i2c_client *i2c)
{
	struct sec_charger_info *charger = i2c_get_clientdata(i2c);

	RT_DBG("\n");
	if (charger->cable_type ==
		POWER_SUPPLY_TYPE_BATTERY) {
		dev_info(&i2c->dev, "%s : turn off OTG\n", __func__);
		/* turn off OTG */
		rt9455_clr_bits(i2c, RT9455_REG_CTRL2, RT9455_OPAMODE_MASK);
	} else {
		dev_info(&i2c->dev, "%s : turn on OTG\n", __func__);
		/* turn on OTG */
		rt9455_set_bits(i2c, RT9455_REG_CTRL2, RT9455_OPAMODE_MASK);
	}
}

static void rt9455_enable_charger_switch(struct i2c_client *i2c, int onoff)
{
	if (onoff)
		rt9455_set_bits(i2c, RT9455_REG_CTRL7, RT9455_CHGEN_MASK);
	else
		rt9455_clr_bits(i2c, RT9455_REG_CTRL7, RT9455_CHGEN_MASK);
}

static void rt9455_enable_charging_termination(struct i2c_client *i2c, int onoff)
{
	RT_DBG("onoff = %d\n", onoff);
	if (onoff)
		rt9455_set_bits(i2c, RT9455_REG_CTRL2, RT9455_TEEN_MASK);
	else
		rt9455_clr_bits(i2c, RT9455_REG_CTRL2, RT9455_TEEN_MASK);
}

static void rt9455_set_input_current_limit(struct i2c_client *i2c, int current_limit)
{
	u8 data;

	RT_DBG("current_limit = %d\n", current_limit);
	if (current_limit <= 100)
		data = 0;
	else if (current_limit > 100 && current_limit <= 500)
		data = 0x1;
	else if (current_limit > 500 && current_limit <= 1000)
		data = 0x2;
	else
		data = 0x3;

	rt9455_assign_bits(i2c, RT9455_REG_CTRL2, RT9455_AICR_LIMIT_MASK,
		data << RT9455_AICR_LIMIT_SHIFT);
}

static void rt9455_set_regulation_voltage(struct i2c_client *i2c, int float_voltage)
{
	u8 data;

	RT_DBG("float voltage = %d\n", float_voltage);
	if (float_voltage < 3500)
		data = 0;
	else if (float_voltage >= 3500 && float_voltage <= 4450)
	{
		data = (float_voltage-3500)/20;
	}
	else
		data = 0x3f;

	rt9455_assign_bits(i2c, RT9455_REG_CTRL3, RT9455_VOREG_MASK,
		data << RT9455_VOREG_SHIFT);
}

static void rt9455_set_fast_charging_current(struct i2c_client *i2c, int charging_current)
{
	u8 data;

	RT_DBG("charging_current = %d\n", charging_current);
	if (charging_current < 500)
		data = 0;
	else if (charging_current >= 500 && charging_current <= 1550)
	{
		data = (charging_current-500)/150;
	}
	else
		data = 0x7;

	rt9455_assign_bits(i2c, RT9455_REG_CTRL6, RT9455_ICHRG_MASK,
		data << RT9455_ICHRG_SHIFT);
}

static int rt9455_get_fast_charging_current(struct i2c_client *i2c)
{
	/* Rsns 0.040 Ohm */
	int current_val;

	RT_DBG("\n");
	current_val = rt9455_reg_read(i2c, RT9455_REG_CTRL6);
	current_val &= RT9455_ICHRG_MASK;
	current_val >>= RT9455_ICHRG_SHIFT;

	current_val = current_val*150 + 500;
	return current_val;
}

static void rt9455_set_termination_current_limit(struct i2c_client *i2c, int current_limit)
{
	int charging_current = rt9455_get_fast_charging_current(i2c);
	int charging_10p = charging_current/10;
	u8 data;

	RT_DBG("current_limit = %d\n", current_limit);
	if (current_limit < charging_10p*2)
		data = 0;
	else if (current_limit >= charging_10p*2 && current_limit < charging_10p*3)
		data = 0x2;
	else
		data = 0x1;

	rt9455_assign_bits(i2c, RT9455_REG_CTRL5, RT9455_IEOC_MASK,
		data << RT9455_IEOC_SHIFT); 
}

static void rt9455_charger_function_control(
				struct i2c_client *client)
{
	struct sec_charger_info *charger = i2c_get_clientdata(client);
	union power_supply_propval val;
	int full_check_type;

	RT_DBG("\n");
	if (charger->charging_current < 0) {
		dev_dbg(&client->dev,
			"%s : OTG is activated. Ignore command!\n", __func__);
		return;
	}

	if (charger->cable_type ==
		POWER_SUPPLY_TYPE_BATTERY) {
		//disable charge function
		rt9455_enable_charger_switch(client , 0);
	} else {
		psy_do_property("battery", get,
			POWER_SUPPLY_PROP_CHARGE_NOW, val);
		if (val.intval == SEC_BATTERY_CHARGING_1ST)
			full_check_type = charger->pdata->full_check_type;
		else
			full_check_type = charger->pdata->full_check_type_2nd;
		/* Termination setting */
		switch (full_check_type) {
		case SEC_BATTERY_FULLCHARGED_CHGGPIO:
		case SEC_BATTERY_FULLCHARGED_CHGINT:
		case SEC_BATTERY_FULLCHARGED_CHGPSY:
			/* Enable Current Termination */
			rt9455_enable_charging_termination(client, 1);
			break;
		default:
			rt9455_enable_charging_termination(client, 0);
			break;
		}
		/* Input current limit */
		dev_dbg(&client->dev, "%s : input current (%dmA)\n",
			__func__, charger->pdata->charging_current
			[charger->cable_type].input_current_limit);

		rt9455_set_input_current_limit(client, charger->pdata->charging_current
			[charger->cable_type].input_current_limit);

		/* Float voltage */
		dev_dbg(&client->dev, "%s : float voltage (%dmV)\n",
			__func__, charger->pdata->chg_float_voltage);

		rt9455_set_regulation_voltage(client, charger->pdata->chg_float_voltage);

		/* Fast charge and Termination current */
		dev_dbg(&client->dev, "%s : fast charging current (%dmA)\n",
				__func__, charger->charging_current);

		rt9455_set_fast_charging_current(client, charger->charging_current);

		dev_dbg(&client->dev, "%s : termination current (%dmA)\n",
			__func__, charger->pdata->charging_current[
			charger->cable_type].full_check_current_1st);

		rt9455_set_termination_current_limit(client, charger->pdata->charging_current[
			charger->cable_type].full_check_current_1st);

		// enable charger switch
		rt9455_enable_charger_switch(client, 1);
	}
}

bool sec_hal_chg_get_property(struct i2c_client *client,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	struct sec_charger_info *charger = i2c_get_clientdata(client);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = rt9455_get_charging_status(client);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = rt9455_get_charge_type(client);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = rt9455_get_charging_health(client);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (charger->charging_current) {
			val->intval = rt9455_get_fast_charging_current(client);
		} else
			val->intval = 0;
		dev_dbg(&client->dev,
			"%s : set-current(%dmA), current now(%dmA)\n",
			__func__, charger->charging_current, val->intval);
		break;
	default:
		return false;
	}
	return true;
}

bool sec_hal_chg_set_property(struct i2c_client *client,
			      enum power_supply_property psp,
			      const union power_supply_propval *val)
{
	struct sec_charger_info *charger = i2c_get_clientdata(client);

	switch (psp) {
	/* val->intval : type */
	case POWER_SUPPLY_PROP_ONLINE:
		if (charger->pdata->chg_gpio_en) {
			if (gpio_request(charger->pdata->chg_gpio_en,
				"CHG_EN") < 0) {
				dev_err(&client->dev,
					"failed to request vbus_in gpio\n");
				break;
			}
			if (charger->cable_type ==
				POWER_SUPPLY_TYPE_BATTERY)
				gpio_set_value_cansleep(
					charger->pdata->chg_gpio_en,
					charger->pdata->chg_polarity_en ?
					0 : 1);
			else
				gpio_set_value_cansleep(
					charger->pdata->chg_gpio_en,
					charger->pdata->chg_polarity_en ?
					1 : 0);
			gpio_free(charger->pdata->chg_gpio_en);
		}
		break;
	/* val->intval : charging current */
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (charger->charging_current < 0)
			rt9455_charger_otg_control(client);
		else if (charger->charging_current > 0){
			rt9455_charger_function_control(client);
		}
		else {
			rt9455_charger_function_control(client);
			rt9455_charger_otg_control(client);
		}
		rt9455_test_read(client);
		break;
	default:
		return false;
	}
	return true;
}

ssize_t sec_hal_chg_show_attrs(struct device *dev,
				const ptrdiff_t offset, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct sec_charger_info *chg =
		container_of(psy, struct sec_charger_info, psy_chg);
	int i = 0;
	char *str = NULL;

	switch (offset) {
	case CHG_REG:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%x\n",
			chg->reg_addr);
		break;
	case CHG_DATA:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%x\n",
			chg->reg_data);
		break;
	case CHG_REGS:
		str = kzalloc(sizeof(char)*1024, GFP_KERNEL);
		if (!str)
			return -ENOMEM;

		rt9455_read_regs(chg->client, str);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n",
			str);

		kfree(str);
		break;
	default:
		i = -EINVAL;
		break;
	}

	return i;
}

ssize_t sec_hal_chg_store_attrs(struct device *dev,
				const ptrdiff_t offset,
				const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct sec_charger_info *chg =
		container_of(psy, struct sec_charger_info, psy_chg);
	int ret = 0;
	int x = 0;
	u8 data = 0;

	switch (offset) {
	case CHG_REG:
		if (sscanf(buf, "%x\n", &x) == 1) {
			chg->reg_addr = x;
			data = rt9455_reg_read(chg->client,
				chg->reg_addr);
			chg->reg_data = data;
			dev_dbg(dev, "%s: (read) addr = 0x%x, data = 0x%x\n",
				__func__, chg->reg_addr, chg->reg_data);
			ret = count;
		}
		break;
	case CHG_DATA:
		if (sscanf(buf, "%x\n", &x) == 1) {
			data = (u8)x;
			dev_dbg(dev, "%s: (write) addr = 0x%x, data = 0x%x\n",
				__func__, chg->reg_addr, data);
			rt9455_reg_write(chg->client,
				chg->reg_addr, data);
			ret = count;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

bool sec_hal_chg_init(struct i2c_client *client)
{
	RT_DBG("\n");
	rt9455_chip_reset(client);
	rt9455_chip_reg_init(client);
	rt9455_test_read(client);
	return true;
}

bool sec_hal_chg_suspend(struct i2c_client *client)
{
	return true;
}

bool sec_hal_chg_resume(struct i2c_client *client)
{
	return true;
}
