/*
 * Base driver for Marvell 88PM822
 *
 * Copyright (C) 2012 Marvell International Ltd.
 * Haojian Zhuang <haojian.zhuang@marvell.com>
 * Joseph(Yossi) Hanin <yhanin@marvell.com>
 * Qiao Zhou <zhouqiao@marvell.com>
 * Yipeng Yao <ypyao@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/88pm822.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/proc_fs.h>

#define PM822_BASE_REG_NUM		0xF0
#define PM822_POWER_REG_NUM		0xD9
#define PM822_GPADC_REG_NUM		0xC8
#define PM822_PROC_FILE			"driver/pm822_reg"

/* Interrupt Number in 88PM822 */
enum {
	PM822_IRQ_ONKEY,	/* EN1b0 *//*0 */
	PM822_IRQ_RSVD1,	/* EN1b1 */
	PM822_IRQ_CHG,		/* EN1b2 */
	PM822_IRQ_BAT,		/* EN1b3 */
	PM822_IRQ_RTC,		/* EN1b4 */
	PM822_IRQ_CLASSD,	/* EN1b5 *//*5 */
	PM822_IRQ_VBAT,		/* EN2b0 */
	PM822_IRQ_VSYS,		/* EN2b1 */
	PM822_IRQ_RSVD2,	/* EN2b2 */
	PM822_IRQ_TINT,		/* EN2b3 */
	PM822_IRQ_GPADC0,	/* EN3b0 *//*10 */
	PM822_IRQ_GPADC1,	/* EN3b1 */
	PM822_IRQ_GPADC2,	/* EN3b2 */
	PM822_IRQ_GPADC3,	/* EN3b3 */
	PM822_MAX_IRQ,
};

static const struct i2c_device_id pm822_id_table[] = {
	{"88PM822", 0},
	{} /* NULL terminated */
};
MODULE_DEVICE_TABLE(i2c, pm822_id_table);

static int reg_pm822 = 0xffff;
static int pg_index;
static struct pm822_chip *g_pm822_chip;

static struct resource onkey_resources[] = {
	{
	 .name = "88pm822-onkey",
	 .start = PM822_IRQ_ONKEY,
	 .end = PM822_IRQ_ONKEY,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct mfd_cell onkey_devs[] = {
	{
	 .name = "88pm822-onkey",
	 .num_resources = 1,
	 .resources = &onkey_resources[0],
	 .id = -1,
	 },
};

static struct resource regulator_resources[] = {
	{PM822_ID_BUCK1, PM822_ID_BUCK1, "buck-1", IORESOURCE_IO,},
	{PM822_ID_BUCK2, PM822_ID_BUCK2, "buck-2", IORESOURCE_IO,},
	{PM822_ID_BUCK3, PM822_ID_BUCK3, "buck-3", IORESOURCE_IO,},
	{PM822_ID_BUCK4, PM822_ID_BUCK4, "buck-4", IORESOURCE_IO,},
	{PM822_ID_BUCK5, PM822_ID_BUCK5, "buck-5", IORESOURCE_IO,},
	{PM822_ID_LDO1, PM822_ID_LDO1, "ldo-01", IORESOURCE_IO,},
	{PM822_ID_LDO2, PM822_ID_LDO2, "ldo-02", IORESOURCE_IO,},
	{PM822_ID_LDO3, PM822_ID_LDO3, "ldo-03", IORESOURCE_IO,},
	{PM822_ID_LDO4, PM822_ID_LDO4, "ldo-04", IORESOURCE_IO,},
	{PM822_ID_LDO5, PM822_ID_LDO5, "ldo-05", IORESOURCE_IO,},
	{PM822_ID_LDO6, PM822_ID_LDO6, "ldo-06", IORESOURCE_IO,},
	{PM822_ID_LDO7, PM822_ID_LDO7, "ldo-07", IORESOURCE_IO,},
	{PM822_ID_LDO8, PM822_ID_LDO8, "ldo-08", IORESOURCE_IO,},
	{PM822_ID_LDO9, PM822_ID_LDO9, "ldo-09", IORESOURCE_IO,},
	{PM822_ID_LDO10, PM822_ID_LDO10, "ldo-10", IORESOURCE_IO,},
	{PM822_ID_LDO11, PM822_ID_LDO11, "ldo-11", IORESOURCE_IO,},
	{PM822_ID_LDO12, PM822_ID_LDO12, "ldo-12", IORESOURCE_IO,},
	{PM822_ID_LDO13, PM822_ID_LDO13, "ldo-13", IORESOURCE_IO,},
	{PM822_ID_LDO14, PM822_ID_LDO14, "ldo-14", IORESOURCE_IO,},
};

static struct mfd_cell regulator_devs[] = {
	{"88pm822-regulator", 0,},
	{"88pm822-regulator", 1,},
	{"88pm822-regulator", 2,},
	{"88pm822-regulator", 3,},
	{"88pm822-regulator", 4,},
	{"88pm822-regulator", 5,},
	{"88pm822-regulator", 6,},
	{"88pm822-regulator", 7,},
	{"88pm822-regulator", 8,},
	{"88pm822-regulator", 9,},
	{"88pm822-regulator", 10,},
	{"88pm822-regulator", 11,},
	{"88pm822-regulator", 12,},
	{"88pm822-regulator", 13,},
	{"88pm822-regulator", 14,},
	{"88pm822-regulator", 15,},
	{"88pm822-regulator", 16,},
	{"88pm822-regulator", 17,},
	{"88pm822-regulator", 18,},
};

static struct regulator_init_data regulator_pdata[ARRAY_SIZE(regulator_devs)];

static const struct regmap_irq pm822_irqs[] = {
	/* INT0 */
	[PM822_IRQ_ONKEY] = {
		.mask = PM822_IRQ_ONKEY_EN,
	},
	[PM822_IRQ_RSVD1] = {
		.mask = 0,
	},
	[PM822_IRQ_CHG] = {
		.mask = PM822_IRQ_CHG_EN,
	},
	[PM822_IRQ_BAT] = {
		.mask = PM822_IRQ_BAT_EN,
	},
	[PM822_IRQ_RTC] = {
		.mask = PM822_IRQ_RTC_EN,
	},
	[PM822_IRQ_CLASSD] = {
		.mask = PM822_IRQ_CLASSD_EN,
	},
	/* INT1 */
	[PM822_IRQ_VBAT] = {
		.reg_offset = 1,
		.mask = PM822_IRQ_VBAT_EN,
	},
	[PM822_IRQ_VSYS] = {
		.reg_offset = 1,
		.mask = PM822_IRQ_VSYS_EN,
	},
	[PM822_IRQ_RSVD2] = {
		.reg_offset = 1,
		.mask = 0,
	},
	[PM822_IRQ_TINT] = {
		.reg_offset = 1,
		.mask = PM822_IRQ_TINT_EN,
	},
	/* INT2 */
	[PM822_IRQ_GPADC0] = {
		.reg_offset = 2,
		.mask = PM822_IRQ_GPADC0_EN,
	},
	[PM822_IRQ_GPADC1] = {
		.reg_offset = 2,
		.mask = PM822_IRQ_GPADC1_EN,
	},
	[PM822_IRQ_GPADC2] = {
		.reg_offset = 2,
		.mask = PM822_IRQ_GPADC2_EN,
	},
	[PM822_IRQ_GPADC3] = {
		.reg_offset = 2,
		.mask = PM822_IRQ_GPADC3_EN,
	},
};

static ssize_t pm822_proc_read(char *buf, char **start, off_t off,
		int count, int *eof, void *data)
{
	unsigned int reg_val = 0;
	int len = 0;
	struct pm822_chip *chip = data;
	int i;

	if (reg_pm822 == 0xffff) {
		pr_info("pm822: base page:\n");
		for (i = 0; i < PM822_BASE_REG_NUM; i++) {
			regmap_read(chip->regmap, i, &reg_val);
			pr_info("[0x%02x]=0x%02x\n", i, reg_val);
		}
		pr_info("pm822: power page:\n");
		for (i = 0; i < PM822_POWER_REG_NUM; i++) {
			regmap_read(chip->subchip->regmap_power, i, &reg_val);
			pr_info("[0x%02x]=0x%02x\n", i, reg_val);
		}
		pr_info("pm822: gpadc page:\n");
		for (i = 0; i < PM822_GPADC_REG_NUM; i++) {
			regmap_read(chip->subchip->regmap_gpadc, i, &reg_val);
			pr_info("[0x%02x]=0x%02x\n", i, reg_val);
		}

		len = 0;
	} else {

		switch (pg_index) {
		case 0:
			regmap_read(chip->regmap, reg_pm822, &reg_val);
			break;
		case 1:
			regmap_read(chip->subchip->regmap_power, reg_pm822, &reg_val);
			break;
		case 2:
			regmap_read(chip->subchip->regmap_gpadc, reg_pm822, &reg_val);
			break;
		default:
			pr_err("pg_index error!\n");
			return 0;
		}

		len = sprintf(buf, "reg_pm822=0x%x, pg_index=0x%x, val=0x%x\n",
			      reg_pm822, pg_index, reg_val);
	}
	return len;
}

static ssize_t pm822_proc_write(struct file *filp,
				       const char *buff, size_t len,
				       void *data)
{
	u8 reg_val;
	struct pm822_chip *chip = data;

	char messages[20], index[20];
	memset(messages, '\0', 20);
	memset(index, '\0', 20);

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		if ((strlen(messages) != 10) &&
		    (strlen(messages) != 9)) {
			pr_err("Right format: -0x[page_addr] 0x[reg_addr]\n");
			return len;
		}
		/* set the register index */
		memcpy(index, messages + 1, 3);

		if (kstrtoint(index, 16, &pg_index) < 0)
			return -EINVAL;

		pr_info("pg_index = 0x%x\n", pg_index);

		memcpy(index, messages + 5, 4);
		if (kstrtoint(index, 16, &reg_pm822) < 0)
			return -EINVAL;
		pr_info("reg_pm822 = 0x%x\n", reg_pm822);
	} else if ('+' == messages[0]) {
		/* enable to get all the reg value */
		reg_pm822 = 0xffff;
		pr_info("read all reg enabled!\n");
	} else {
		if ((reg_pm822 == 0xffff) ||
		    ('0' != messages[0])) {
			pr_err("Right format: -0x[page_addr] 0x[reg_addr]\n");
			return len;
		}
		/* set the register value */
		if (kstrtou8(messages, 16, &reg_val) < 0)
			return -EINVAL;

		switch (pg_index) {
		case 0:
			regmap_write(chip->regmap, reg_pm822, reg_val & 0xff);
			break;
		case 1:
			regmap_write(chip->subchip->regmap_power, reg_pm822, reg_val & 0xff);
			break;
		case 2:
			regmap_write(chip->subchip->regmap_gpadc, reg_pm822, reg_val & 0xff);
			break;
		default:
			pr_err("pg_index error!\n");
			break;

		}
	}

	return len;
}

static int __devinit device_gpadc_init(struct pm822_chip *chip,
				       struct pm822_platform_data *pdata)
{
	/* FIXME add inital seq for gpadc */
	return 0;
}

static int __devinit device_regulator_init(struct pm822_chip *chip,
				struct pm822_platform_data *pdata)
{
	struct regulator_init_data *initdata;
	int ret = 0;
	int i, seq;

	if (!pdata || !pdata->regulator) {
		dev_warn(chip->dev, "Regulator pdata is unavailable!\n");
		return 0;
	}

	if (pdata->num_regulators > ARRAY_SIZE(regulator_devs))
		pdata->num_regulators = ARRAY_SIZE(regulator_devs);

	for (i = 0; i < pdata->num_regulators; i++) {
		initdata = &pdata->regulator[i];
		seq = *(unsigned int *)initdata->driver_data;
		if ((seq < 0) || (seq > PM822_ID_RG_MAX)) {
			dev_err(chip->dev, "Wrong ID(%d) on regulator(%s)\n",
				seq, initdata->constraints.name);
			ret = -EINVAL;
			goto out_err;
		}
		memcpy(&regulator_pdata[i], &pdata->regulator[i],
		       sizeof(struct regulator_init_data));
		regulator_devs[i].platform_data = &regulator_pdata[i];
		regulator_devs[i].pdata_size =
		    sizeof(struct regulator_init_data);
		regulator_devs[i].num_resources = 1;
		regulator_devs[i].resources = &regulator_resources[seq];

		ret = mfd_add_devices(chip->dev, 0, &regulator_devs[i], 1,
				      &regulator_resources[seq], 0);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add regulator subdev\n");
			goto out_err;
		}
	}
	dev_info(chip->dev, "[%s]:Added mfd regulator_devs\n", __func__);
	return 0;

out_err:
	return ret;
}

static int __devinit device_pm822_irq_init(struct pm822_chip *chip)
{
	struct regmap *map = chip->regmap;
	unsigned long flags = IRQF_ONESHOT;
	struct irq_desc *desc;
	int data, mask, ret = -EINVAL;

	if (!map || !chip->irq) {
		dev_err(chip->dev, "incorrect parameters\n");
		return -EINVAL;
	}

	/* irq pin: low active
	 * irq clear on write
	 * irq status bit: not set when it is masked
	 */
	mask = PM822_INV_INT | PM822_INT_CLEAR_MODE | PM822_INT_MASK_MODE;
	data = PM822_INT_CLEAR_MODE;
	ret = regmap_update_bits(map, PM822_WAKEUP2, mask, data);
	if (ret < 0)
		goto out;

	ret = regmap_add_irq_chip(chip->regmap, chip->irq, flags, -1,
				chip->regmap_irq_chip, &chip->irq_data);

	chip->irq_base = regmap_irq_chip_get_base(chip->irq_data);

	desc = irq_to_desc(chip->irq);
	irq_get_chip(chip->irq_base)->irq_set_wake =
		desc->irq_data.chip->irq_set_wake;
out:
	return ret;
}

static void device_pm822_irq_exit(struct pm822_chip *chip)
{
	regmap_del_irq_chip(chip->irq, chip->irq_data);
}

static struct regmap_irq_chip pm822_irq_chip = {
	.name = "88pm822",
	.irqs = pm822_irqs,
	.num_irqs = ARRAY_SIZE(pm822_irqs),

	.num_regs = 3,
	.status_base = PM822_INT_STATUS1,
	.mask_base = PM822_INT_EN1,
	.ack_base = PM822_INT_STATUS1,
	.mask_invert = 1,
};

const struct regmap_config pm822_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int pm822_pages_init(struct pm822_chip *chip)
{
	struct pm822_subchip *subchip = chip->subchip;
	struct i2c_client *client = chip->client;

	if (subchip->power_page_addr) {
		subchip->power_page =
		    i2c_new_dummy(client->adapter, subchip->power_page_addr);
		subchip->regmap_power =
		    devm_regmap_init_i2c(subchip->power_page,
					 &pm822_regmap_config);
		i2c_set_clientdata(subchip->power_page, chip);
	} else
		dev_info(chip->dev,
			 "PM822: No power_page_addr\n");

	if (subchip->gpadc_page_addr) {
		subchip->gpadc_page = i2c_new_dummy(client->adapter,
						    subchip->gpadc_page_addr);
		subchip->regmap_gpadc =
		    devm_regmap_init_i2c(subchip->gpadc_page,
					 &pm822_regmap_config);
		i2c_set_clientdata(subchip->gpadc_page, chip);
	} else
		dev_info(chip->dev,
			 "PM822: No gpadc_page_addr\n");

	return 0;
}

static void pm822_pages_exit(struct pm822_chip *chip)
{
	struct pm822_subchip *subchip = chip->subchip;

	if (subchip->power_page) {
		regmap_exit(subchip->regmap_power);
		i2c_unregister_device(subchip->power_page);
	}
	if (subchip->gpadc_page) {
		regmap_exit(subchip->regmap_gpadc);
		i2c_unregister_device(subchip->gpadc_page);
	}
	i2c_unregister_device(chip->client);
}

static int __devinit device_pm822_init(struct pm822_chip *chip,
				     struct pm822_platform_data *pdata)
{
	int ret;
	unsigned int val;

	ret = regmap_read(chip->regmap, PM822_CHIP_ID, &val);
	if (ret < 0) {
		dev_err(chip->dev, "PM822: Failed to read CHIP ID: %d\n", ret);
		goto out;
	}
	chip->version = val & 0xFF;
	dev_info(chip->dev, "88PM822:Marvell 88PM822 (ID:0x%X) detected\n", val);

	ret = device_gpadc_init(chip, pdata);
	if (ret < 0) {
		dev_err(chip->dev, "[%s]Failed to init gpadc\n", __func__);
		goto out;
	}

	chip->regmap_irq_chip = &pm822_irq_chip;

	ret = device_pm822_irq_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "[%s]Failed to init pm822 irq\n", __func__);
		goto out;
	}

	if (device_regulator_init(chip, pdata)) {
		dev_err(chip->dev, "Failed to init regulators\n");
		goto out_dev;
	}

	ret =
	    mfd_add_devices(chip->dev, 0, &onkey_devs[0],
			    ARRAY_SIZE(onkey_devs), &onkey_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add onkey subdev\n");
		goto out_dev;
	} else
		dev_info(chip->dev, "[%s]:Added mfd onkey_devs\n", __func__);

	/* FIXME battery */
	/* FIXME vibrator */
	/* FIXME rtc */
	/* FIXME usb */

	if (chip->proc_file == NULL) {
		chip->proc_file =
			create_proc_entry(PM822_PROC_FILE, 0644, NULL);
		if (chip->proc_file) {
			chip->proc_file->read_proc = pm822_proc_read;
			chip->proc_file->write_proc = (write_proc_t  *)pm822_proc_write;
			chip->proc_file->data = chip;
		} else
			pr_info("pm822 proc file create failed!\n");
	}

	return 0;
out_dev:
	mfd_remove_devices(chip->dev);
	device_pm822_irq_exit(chip);
out:
	return ret;
}

static int __devinit pm822_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret = 0;
	struct pm822_chip *chip;
	struct regmap *map;
	struct pm822_subchip *subchip;
	struct pm822_platform_data *pdata = client->dev.platform_data;

	chip = devm_kzalloc(&client->dev, sizeof(struct pm822_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto out_init;
	}

	map = devm_regmap_init_i2c(client, &pm822_regmap_config);
	if (IS_ERR(map)) {
		ret = PTR_ERR(map);
		dev_err(&client->dev, "Failed to allocate register map: %d\n", ret);
		goto err_free_chip;
	}

	chip->client = client;
	chip->regmap = map;
	chip->irq = client->irq;
	chip->dev = &client->dev;
	dev_set_drvdata(chip->dev, chip);
	i2c_set_clientdata(chip->client, chip);

	device_init_wakeup(&client->dev, 1);

	subchip = devm_kzalloc(&client->dev, sizeof(struct pm822_subchip), GFP_KERNEL);
	if (!subchip) {
		ret = -ENOMEM;
		goto err_free_chip;
	}

	subchip->power_page_addr = pdata->power_page_addr;
	subchip->gpadc_page_addr = pdata->gpadc_page_addr;
	chip->subchip = subchip;

	ret = pm822_pages_init(chip);
	if (ret) {
		dev_err(&client->dev, "pm822_pages_init failed!\n");
		goto err_free_subchip;
	}

	ret = device_pm822_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "%s failed!\n", __func__);
		goto err_exit_page;
	}

	if (pdata->plat_config)
		pdata->plat_config(chip, pdata);

	if (!g_pm822_chip)
		g_pm822_chip = chip;

	return 0;

err_exit_page:
	pm822_pages_exit(chip);
err_free_subchip:
	devm_kfree(&client->dev, subchip);
err_free_chip:
	devm_kfree(&client->dev, chip);
out_init:
	return ret;
}

static int __devexit pm822_remove(struct i2c_client *client)
{
	struct pm822_chip *chip = i2c_get_clientdata(client);

	g_pm822_chip = NULL;
	mfd_remove_devices(chip->dev);
	device_pm822_irq_exit(chip);

	pm822_pages_exit(chip);
	remove_proc_entry(PM822_PROC_FILE, NULL);
	devm_kfree(&client->dev, chip->subchip);
	devm_kfree(&client->dev, chip);

	return 0;
}

int pm822_extern_read(int page, int reg)
{
	int ret;
	unsigned int val;
	struct pm822_chip *chip = g_pm822_chip;
	if (!chip) {
		pr_err("%s: chip is NULL\n", __func__);
		return -EINVAL;
	}
	switch (page) {
	case PM822_BASE_PAGE:
		ret = regmap_read(chip->regmap, reg, &val);
		break;
	case PM822_POWER_PAGE:
		ret = regmap_read(chip->subchip->regmap_power,
				  reg, &val);
		break;
	case PM822_GPADC_PAGE:
		ret = regmap_read(chip->subchip->regmap_gpadc,
				  reg, &val);
		break;
	default:
		ret = -1;
		break;
	}

	if (ret < 0) {
		pr_err("fail to read reg 0x%x\n", reg);
		return ret;
	}

	return val;
}
EXPORT_SYMBOL(pm822_extern_read);

int pm822_extern_write(int page, int reg, unsigned char val)
{
	int ret;
	struct pm822_chip *chip = g_pm822_chip;
	if (!chip) {
		pr_err("%s: chip is NULL\n", __func__);
		return -EINVAL;
	}
	switch (page) {
	case PM822_BASE_PAGE:
		ret = regmap_write(chip->regmap, reg, val);
		break;
	case PM822_POWER_PAGE:
		ret = regmap_write(chip->subchip->regmap_power,
				  reg, val);
		break;
	case PM822_GPADC_PAGE:
		ret = regmap_write(chip->subchip->regmap_gpadc,
				  reg, val);
		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}
EXPORT_SYMBOL(pm822_extern_write);

int pm822_extern_setbits(int page, int reg,
			 unsigned char mask, unsigned char val)
{
	int ret;
	struct pm822_chip *chip = g_pm822_chip;
	if (!chip) {
		pr_err("%s: chip is NULL\n", __func__);
		return -EINVAL;
	}
	switch (page) {
	case PM822_BASE_PAGE:
		ret = regmap_update_bits(chip->regmap, reg, mask, val);
		break;
	case PM822_POWER_PAGE:
		ret = regmap_update_bits(chip->subchip->regmap_power,
					 reg, mask, val);
		break;
	case PM822_GPADC_PAGE:
		ret = regmap_update_bits(chip->subchip->regmap_gpadc,
					 reg, mask, val);
		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}
EXPORT_SYMBOL(pm822_extern_setbits);

#ifdef CONFIG_PM
static int pm822_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct pm822_chip *chip = i2c_get_clientdata(client);
	int i, tmp = chip->wu_flag;

	if (chip && tmp &&
	    device_may_wakeup(chip->dev)) {
		enable_irq_wake(chip->irq);

		for (i = 0; i < 32; i++) {
			if (tmp & (1 << i))
				enable_irq_wake(chip->irq_base + i);
		}
	}

	return 0;
}

static int pm822_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct pm822_chip *chip = i2c_get_clientdata(client);
	int i, tmp = chip->wu_flag;

	if (chip && tmp &&
	    device_may_wakeup(chip->dev)) {
		disable_irq_wake(chip->irq);

		for (i = 0; i < 32; i++) {
			if (tmp & (1 << i))
				disable_irq_wake(chip->irq_base + i);
		}
	}

	return 0;
}

SIMPLE_DEV_PM_OPS(pm822_pm_ops, pm822_suspend, pm822_resume);

#endif

static struct i2c_driver pm822_driver = {
	.driver = {
		.name = "88PM822",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &pm822_pm_ops,
#endif
		},
	.probe = pm822_probe,
	.remove = __devexit_p(pm822_remove),
	.id_table = pm822_id_table,
};

static int __init pm822_i2c_init(void)
{
	return i2c_add_driver(&pm822_driver);
}
subsys_initcall(pm822_i2c_init);

static void __exit pm822_i2c_exit(void)
{
	i2c_del_driver(&pm822_driver);
}
module_exit(pm822_i2c_exit);

MODULE_DESCRIPTION("PMIC Driver for Marvell 88PM822");
MODULE_AUTHOR("Yipeng Yao <ypyao@marvell.com>");
MODULE_LICENSE("GPL");
