/*
 * projector.c  --  projector module driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Inbum Choi <inbum.choi@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/projector/projector.h>
#include <linux/projector/beam_gpio_i2c.h>

#include "prj_dpp3430_data.h"

#ifndef	GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW		0
#define GPIO_LEVEL_HIGH		1
#endif

/*
#define DPP_I2C_EMUL
#define LED_COMPENSATION
*/
#define PROJECTOR_DEBUG

#define DPP3430_I2C			0x1B
#define M08980_I2C			0x36


#define DPPDATAFLASH(ARRAY)	(dpp_flash(ARRAY, sizeof(ARRAY)/sizeof(ARRAY[0])))
#define M08980CTRL(ARRAY) 		(dpp_flash(ARRAY, sizeof(ARRAY)/sizeof(ARRAY[0])))

#define ChangeDuty(array, data) (array[3] = (data))		// I2C_Write( 0x36, 0x22, 0xSeq Number); Cal data set 19° Ǿ ִ Sequence number

//static int motor_step;
static int motor_abs_step;
static int once_motor_verified;

static int verify_value;

#define MAX_MOTOR_STEP 60



static int brightness = BRIGHT_HIGH;

struct workqueue_struct *projector_work_queue;
struct workqueue_struct *stepmotor_work_queue;
struct work_struct projector_work_power_on;
struct work_struct projector_work_power_off;
struct work_struct projector_work_motor_cw;
struct work_struct projector_work_motor_ccw;
struct work_struct projector_work_testmode_on;
struct work_struct projector_work_rotate_screen;
struct work_struct projector_work_read_test;

struct device *sec_projector;
extern struct class *sec_class;

static int screen_direction=0;

static int status;
static int saved_pattern = -1;

static unsigned int not_calibrated = 10;
unsigned char RGB_BUF[MAX_LENGTH];
static unsigned char seq_number;

volatile unsigned char flash_rgb_level_data[3][3][2] = {{{0,},},};

struct projector_dpp3430_info {
	struct i2c_client			*client;
	struct projector_dpp3430_platform_data	*pdata;
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend			earlysuspend;
#endif

};

struct projector_M08980_info {
	struct i2c_client			*client;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend			earlysuspend;
#endif

};

struct projector_dpp3430_info *info = NULL;
struct projector_M08980_info *M08980_info = NULL;

int dpp_flash( unsigned char *DataSetArray, int iNumArray)
{
	int i = 0;
	int CurrentDataIndex = 0;
	int Bytes = 0;
	int R_Bytes = 0;
	int cnt=0;

	printk(KERN_ERR "[%s] \n", __func__);

	for (i = 0; i < iNumArray;) 
	{
		msleep(1);		//temp 091201 , to prevent from abnormal operation like nosie screen
		if(DataSetArray[i + OFFSET_I2C_DIRECTION] == PRJ_READ)
		{
			Bytes =  DataSetArray[i + OFFSET_I2C_NUM_BYTES];
			R_Bytes = DataSetArray[i + OFFSET_I2C_NUM_BYTES+1];
			CurrentDataIndex = i + OFFSET_I2C_DATA_START+1;
		}
		else{
			Bytes =  DataSetArray[i + OFFSET_I2C_NUM_BYTES];
			R_Bytes = 0;
			CurrentDataIndex = i + OFFSET_I2C_DATA_START;
		}


		switch(DataSetArray[i + OFFSET_I2C_DIRECTION])
		{
			case PRJ_WRITE:
#ifdef DPP_I2C_EMUL				
			i2c_master_send(info->client, &DataSetArray[CurrentDataIndex], Bytes);
#else
			beam_i2c_write(info->client, DPP3430_I2C, &DataSetArray[CurrentDataIndex], Bytes);
#endif
#ifdef PROJECTOR_DEBUG
			printk("PRJ_WRITE [%d]:",Bytes);
			for(cnt=0;cnt<Bytes;cnt++)
			printk("%x ", DataSetArray[CurrentDataIndex+cnt]);
			printk("\n");
#endif		
			break;

			case PRJMSPD_WRITE:
#ifdef DPP_I2C_EMUL				
			i2c_master_send(info->client, &DataSetArray[CurrentDataIndex], Bytes);
#else
			beam_i2c_write(M08980_info->client, M08980_I2C, &DataSetArray[CurrentDataIndex], Bytes);
#endif
#ifdef PROJECTOR_DEBUG
			printk("PRJMSPD_WRITE [%d]:",Bytes);
			for(cnt=0;cnt<Bytes;cnt++)
			printk("%x ", DataSetArray[CurrentDataIndex+cnt]);
			printk("\n");
#endif		
			break;

			case PRJ_READ:
			memset(RGB_BUF, 0x0, sizeof(RGB_BUF));
#ifdef DPP_I2C_EMUL				
			i2c_master_recv(info->client, RGB_BUF, Bytes);
#else
			beam_i2c_read(info->client, DPP3430_I2C, &DataSetArray[CurrentDataIndex], RGB_BUF, Bytes, R_Bytes);
#endif

#ifdef PROJECTOR_DEBUG
			printk(KERN_INFO "PRJ_READ[%d] ",R_Bytes);
			for(cnt=0;cnt<R_Bytes;cnt++)
				printk(KERN_INFO "%x ", 	RGB_BUF[cnt]);
			printk("\n");
#endif
			break;

/*
			case PRJMSPD_READ:
			memset(RGB_BUF, 0x0, sizeof(RGB_BUF));

#ifdef DPP_I2C_EMUL				
			i2c_master_recv(info->client, RGB_BUF, Bytes);
#else
			beam_i2c_read(M08980_info->client, M08980_I2C, RGB_BUF, Bytes);
#endif

#ifdef PROJECTOR_DEBUG
			printk(KERN_INFO "PRJMSPD_READ[%d] ",Bytes);
			for(cnt=0;cnt<Bytes;cnt++)
				printk(KERN_INFO "%x ", 	RGB_BUF[cnt]);
			printk("\n");
#endif
			break;
*/
			default:
			printk(KERN_INFO "[%s] data is invalid !!\n", __func__);
			return -EINVAL;

		}
		 i = (CurrentDataIndex + Bytes + R_Bytes);
	}
	return 0;
}

void set_proj_status(int enProjectorStatus)
{
	status = enProjectorStatus;
	printk(KERN_INFO "[%s] projector status : %d\n", __func__, status);
}

int get_proj_status(void)
{
	printk(KERN_ERR "[%s] : %d\n", __func__,status);
	return status;
}

int get_proj_motor_status(void)
{
	int pi_int;

	pi_int = gpio_get_value(info->pdata->gpio_pi_int);

	printk(KERN_ERR "[%s] (%d) \n", __func__,pi_int);

	if(pi_int)
		return MOTOR_OUTOFRANGE;
	else
		return MOTOR_INRANGE;
}	


EXPORT_SYMBOL(get_proj_status);

static void projector_motor_cw_work(struct work_struct *work)
{
 
	M08980CTRL(Motor_CW_out);
	printk(KERN_INFO "[%s] CW 1 step\n", __func__);
	motor_abs_step--;

	if(get_proj_motor_status() == MOTOR_OUTOFRANGE)
	{
		int i;
		for(i=0;i<10;i++)
		{
			M08980CTRL(Motor_CCW_out);
			printk(KERN_INFO "[%s] Motor out of ragne \n", __func__);
			motor_abs_step++;

			if(get_proj_motor_status() == MOTOR_INRANGE) break;
		}
	}
}

void projector_motor_cw(void)
{
	queue_work(stepmotor_work_queue, &projector_work_motor_cw);
}

static void projector_motor_ccw_work(struct work_struct *work)
{

	M08980CTRL(Motor_CCW_out);
	printk(KERN_INFO "[%s] CCW 1 step\n", __func__);
	motor_abs_step++;

	if(get_proj_motor_status() == MOTOR_OUTOFRANGE)
	{
		int i;
		for(i=0;i<10;i++)
		{
			M08980CTRL(Motor_CW_out);
			printk(KERN_INFO "[%s] Motor out of ragne \n", __func__);
			motor_abs_step--;

			if(get_proj_motor_status() == MOTOR_INRANGE) break;
		}
	}
}

void projector_motor_ccw(void)
{
	queue_work(stepmotor_work_queue, &projector_work_motor_ccw);
}

void set_led_current(int level)
{
	printk(KERN_ERR "[%s] level:%d\n", __func__, level);
		
	RED_MSB_DAC[3] = flash_rgb_level_data[level - 1][0][0];
	RED_LSB_DAC[3] = flash_rgb_level_data[level - 1][0][1];
	GREEN_MSB_DAC[3] = flash_rgb_level_data[level - 1][1][0];
	GREEN_LSB_DAC[3] = flash_rgb_level_data[level - 1][1][1];
	BLUE_MSB_DAC[3] = flash_rgb_level_data[level - 1][2][0];
	BLUE_LSB_DAC[3] = flash_rgb_level_data[level - 1][2][1];

	M08980CTRL(RED_MSB_DAC);
	M08980CTRL(RED_LSB_DAC);
	M08980CTRL(GREEN_MSB_DAC);
	M08980CTRL(GREEN_LSB_DAC);
	M08980CTRL(BLUE_MSB_DAC);
	M08980CTRL(BLUE_LSB_DAC);
}

void set_prj_rdy_gpio(int v_value)
{
	printk(KERN_ERR "[%s] v_value = %d \n", __func__, v_value);

	if(gpio_direction_output(info->pdata->gpio_prj_rdy, v_value ? 1/*GPIO_LEVEL_HIGH*/:0/*GPIO_LEVEL_LOW*/))
	{
		printk(KERN_ERR "[%s] (gpio_request : PRJ_RDY(%d) error \n", __func__,(info->pdata->gpio_prj_rdy));
	}
}

void pwron_seq_gpio(void)
{
	printk(KERN_ERR "[%s] gpio_request \n", __func__);

	
//	set_prj_rdy_gpio(0);
	if(gpio_direction_output(info->pdata->gpio_prj_on, GPIO_LEVEL_HIGH))
	{
		printk(KERN_ERR "[%s] (gpio_request : MP_ON(%d) error \n", __func__,(info->pdata->gpio_prj_on));
	}
	msleep(10);

	if(gpio_direction_output(info->pdata->gpio_mp_on, GPIO_LEVEL_HIGH))
	{
		printk(KERN_ERR "[%s] (gpio_request : PRJ_EN(%d) error \n", __func__,(info->pdata->gpio_mp_on));
	}
	msleep(10);
}


void pwron_seq_direction(void)
{
	switch (get_proj_rotation()) {
	case PRJ_ROTATE_0:
		DPPDATAFLASH(Output_Rotate_0);
		break;
	case PRJ_ROTATE_90:
		DPPDATAFLASH(Output_Rotate_90);
		break;
	case PRJ_ROTATE_180:
		DPPDATAFLASH(Output_Rotate_180);
		break;
	case PRJ_ROTATE_270:
		DPPDATAFLASH(Output_Rotate_270);
		break;
	default:
		break;
	};
}

void pwron_seq_source_res(int value)
{
	if (value == LCD_VIEW) {
		DPPDATAFLASH(VSWVGA_RGB888/*WVGA_RGB888*/);
	} else if (value == INTERNAL_PATTERN) {
		DPPDATAFLASH(fWVGA_RGB888);
	}
}

/* BEAM_HW : i */
void pwron_seq_fdr(void)
{
	int cnt;
	unsigned int dac;
	
	DPPDATAFLASH(InitData_FlashDataTypeSelect);

	DPPDATAFLASH(InitData_FlashUpdatePrecheck);

	if ( RGB_BUF[0] != 0)
	{
		printk(KERN_ERR "[%s] Flash Data Type Error\n", __func__);
	}		
	DPPDATAFLASH(InitData_FlashDataLength);
	DPPDATAFLASH(InitData_ReadFlashData);
	
	for(cnt = 0; cnt < 9; cnt++)
	{
		dac = 0;
		dac |= RGB_BUF[2*cnt] << 8 | RGB_BUF[2*cnt+1];
		
		not_calibrated = (dac < 2 || dac > 999) ? 1 : 0;
		
		flash_rgb_level_data[cnt/3][cnt%3][0] = RGB_BUF[2*cnt];
		flash_rgb_level_data[cnt/3][cnt%3][1] = RGB_BUF[2*cnt+1];
	}
	seq_number = RGB_BUF[18];		
	printk(KERN_ERR "[%s] seq_number %x\n", __func__, seq_number);		
	
}



static void proj_pwron_seq_work(struct work_struct *work)
{

	printk(KERN_ERR "[%s] \n", __func__);

	if (get_proj_status() == PRJ_ON_INTERNAL) 
	{
		DPPDATAFLASH(Output_Curtain_Enable);
		pwron_seq_direction();
		pwron_seq_source_res(LCD_VIEW);
		msleep(100);
		DPPDATAFLASH(External_source);
		DPPDATAFLASH(Output_Curtain_Disable);
	} 
	else 
	{
		pwron_seq_gpio();

		// M08980 soft reset & init	
		M08980CTRL(M08980_SoftReset);
		msleep(10);			
		M08980CTRL(M08980_Init);
		msleep(10);
		
		//M08980 PRJ ON
		M08980CTRL(M08980_PRJON);
		
		msleep(500);
		msleep(500);
		msleep(500);
		msleep(500);

		// M08980 LED ON & Alarm Clear
		M08980CTRL(M08980_LEDON);
		msleep(10);
		M08980CTRL(M08980_AlarmClear);	
		

		pwron_seq_direction();
		pwron_seq_source_res(LCD_VIEW);

		pwron_seq_fdr();
		set_led_current(brightness);

		ChangeDuty(Dmd_seq, seq_number);
		DPPDATAFLASH(Dmd_seq);

		DPPDATAFLASH(External_source);
		msleep(50);
		
		DPPDATAFLASH(RGB_led_on);
	}
	set_proj_status(PRJ_ON_RGB_LCD);
	
}

static void proj_testmode_pwron_seq_work(struct work_struct *work)
{
	pwron_seq_gpio();

	DPPDATAFLASH(Internal_pattern_direction);
	pwron_seq_source_res(INTERNAL_PATTERN);

	pwron_seq_fdr();
	set_led_current(brightness);

	ChangeDuty(Dmd_seq, seq_number);
	DPPDATAFLASH(Dmd_seq);

	switch (saved_pattern) {
	case CHECKER:
		DPPDATAFLASH(I_4x4checker);
		verify_value = 20;
		break;
// BEGIN CSLBSuP PPNFAD a.jakubiak/m.wengierow
	case CHECKER_16x9:
		DPPDATAFLASH(I_16x9checker);
		verify_value = 40;
		break;
// END CSLBSuP PPNFAD a.jakubiak/m.wengierow
	case WHITE:
		DPPDATAFLASH(I_white);
		verify_value = 21;
		break;
	case BLACK:
		DPPDATAFLASH(I_black);
		verify_value = 22;
		break;
	case LEDOFF:
		DPPDATAFLASH(RGB_led_off);
		set_proj_status(RGB_LED_OFF);
		verify_value = 23;
		break;
	case RED:
		DPPDATAFLASH(I_red);
		verify_value = 24;
		break;
	case GREEN:
		DPPDATAFLASH(I_green);
		verify_value = 25;
		break;
	case BLUE:
		DPPDATAFLASH(I_blue);
		verify_value = 26;
		break;
	case STRIPE:
		DPPDATAFLASH(I_stripe);
		verify_value = 28;
		break;
	default:
		break;
	}
	saved_pattern = -1;

	set_proj_status(PRJ_ON_INTERNAL);
}

void proj_testmode_pwron_seq(void)
{
	queue_work(projector_work_queue, &projector_work_testmode_on);
}

void ProjectorPowerOnSequence(void)
{
	queue_work(projector_work_queue, &projector_work_power_on);
}

void ProjectorReadTest(void)
{
	queue_work(projector_work_queue, &projector_work_read_test);
}

static void proj_pwroff_seq_work(struct work_struct *work)
{

	// M08980 LED Off
	M08980CTRL(M08980_LEDOFF);
	msleep(10);
	
	gpio_direction_output(info->pdata->gpio_prj_on, GPIO_LEVEL_LOW);
	msleep(10);

	// M08980 Projector Off
	M08980CTRL(M08980_PRJOFF);
	msleep(400);		// HW
	
	gpio_direction_output(info->pdata->gpio_mp_on, GPIO_LEVEL_LOW);
		
	set_proj_status(PRJ_OFF);
}

void ProjectorPowerOffSequence(void)
{
	queue_work(projector_work_queue, &projector_work_power_off);
}

static void projector_rotate_screen_work(struct work_struct *work)
{
	if (status == PRJ_ON_RGB_LCD) {
		switch (screen_direction) {
		case PRJ_ROTATE_0:
			DPPDATAFLASH(Output_Curtain_Enable);
			DPPDATAFLASH(Output_Rotate_0);
			msleep(50);
			DPPDATAFLASH(Output_Curtain_Disable);
			break;
		case PRJ_ROTATE_90:
			DPPDATAFLASH(Output_Curtain_Enable);
			DPPDATAFLASH(Output_Rotate_90);
			msleep(50);
			DPPDATAFLASH(Output_Curtain_Disable);
			break;
		case PRJ_ROTATE_180:
			DPPDATAFLASH(Output_Curtain_Enable);
			DPPDATAFLASH(Output_Rotate_180);
			msleep(50);
			DPPDATAFLASH(Output_Curtain_Disable);
			break;
		case PRJ_ROTATE_270:
			DPPDATAFLASH(Output_Curtain_Enable);
			DPPDATAFLASH(Output_Rotate_270);
			msleep(50);
			DPPDATAFLASH(Output_Curtain_Disable);
			break;
		default:
			break;
		}
	}
}

static void projector_read_test_work(struct work_struct *work)
{
	if (status == PRJ_ON_RGB_LCD) {
		printk("%s : status is PRJ_ON_RGB_LCD", __func__);
		
	}
	else{
		printk("%s : status is not PRJ_ON_RGB_LCD", __func__);
	}
	

}

int get_proj_rotation(void)
{
	return screen_direction;
}

int get_proj_brightness(void)
{
	return brightness;
}

int __devinit dpp3430_i2c_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
	int ret = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "[%s] need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		return ret;
	}
	
	info = kzalloc(sizeof(struct projector_dpp3430_info), GFP_KERNEL);
	if (!info) {
		printk(KERN_ERR "[%s] fail to memory allocation.\n", __func__);
		return -1;
	}

	info->client = client;
	info->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, info);

	
	if(gpio_request((info->pdata->gpio_mp_on), "MP_ON"))
	{
		printk(KERN_ERR "[%s] (gpio_request : MP_ON(%d) error\n", __func__,(info->pdata->gpio_mp_on));
	}
	
	if(gpio_request((info->pdata->gpio_prj_on), "PRJ_EN"))
	{
		printk(KERN_ERR "[%s] (gpio_request : PRJ_EN(%d) error \n", __func__,(info->pdata->gpio_prj_on));
	}

	if(gpio_request((info->pdata->gpio_prj_rdy), "PRJ_RDY"))
	{
		printk(KERN_ERR "[%s] (gpio_request : PRJ_RDY(%d) error \n", __func__,(info->pdata->gpio_prj_rdy));
	}
	
	projector_work_queue = create_singlethread_workqueue("projector_work_queue");
	if (!projector_work_queue) {
		printk(KERN_ERR "[%s] i2c_probe fail.\n", __func__);
		return -ENOMEM;
	}

	stepmotor_work_queue = create_singlethread_workqueue("stepmotor_work_queue");
	if (!stepmotor_work_queue) {
		printk(KERN_ERR "[%s] i2c_probe fail.\n", __func__);
		return -ENOMEM;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	info->earlysuspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->earlysuspend.suspend = projector_module_early_suspend;
	info->earlysuspend.resume  = projector_module_late_resume;
	register_early_suspend(&info->earlysuspend);
#endif
	INIT_WORK(&projector_work_power_on, proj_pwron_seq_work);
	INIT_WORK(&projector_work_power_off, proj_pwroff_seq_work);
	INIT_WORK(&projector_work_motor_cw, projector_motor_cw_work);
	INIT_WORK(&projector_work_motor_ccw, projector_motor_ccw_work);
	INIT_WORK(&projector_work_testmode_on, proj_testmode_pwron_seq_work);
	INIT_WORK(&projector_work_rotate_screen, projector_rotate_screen_work);
	INIT_WORK(&projector_work_read_test, projector_read_test_work);
	
	printk(KERN_ERR "[%s] dpp3430_i2c_probe(MP%d,  PRJ.: %d\n", __func__,info->pdata->gpio_mp_on,info->pdata->gpio_prj_on);

	return 0;
}

__devexit int dpp3430_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id dpp3430_i2c_id[] = {
	{ "dpp3430", 0 },
	{ }
};
MODULE_DEVICE_TABLE(dpp3430_i2c, dpp3430_i2c_id);

static struct i2c_driver dpp3430_i2c_driver = {
	.driver = {
		.name = "dpp3430",
		.owner = THIS_MODULE,
	},
	.probe = dpp3430_i2c_probe,
	.remove = __devexit_p(dpp3430_i2c_remove),
	.id_table = dpp3430_i2c_id,
};


int __devinit M08980_i2c_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
	int ret = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "[%s] need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		return ret;
	}
	M08980_info = kzalloc(sizeof(struct projector_M08980_info), GFP_KERNEL);
	if (!M08980_info) {
		printk(KERN_ERR "[%s] fail to memory allocation.\n", __func__);
		return -1;
	}

	M08980_info->client = client;
	//M08980_info->pdata = client->dev.platform_data;
	
	i2c_set_clientdata(client, M08980_info);


	printk("[%s] \n", __func__);

	return 0;
	
}

static const struct i2c_device_id M08980_i2c_id[] = {
	{ "M08980", 0 },
	{ }
};


__devexit int M08980_i2c_remove(struct i2c_client *client)
{
	return 0;
}

MODULE_DEVICE_TABLE(M08980_i2c, M08980_i2c_id);

static struct i2c_driver M08980_i2c_driver = {
	.driver = {
		.name = "M08980",
		.owner = THIS_MODULE,
	},
	.probe = M08980_i2c_probe,
	.remove = __devexit_p(M08980_i2c_remove),
	.id_table = M08980_i2c_id,
};


int projector_module_open(struct inode *inode, struct file *file)
{
		return 0;
}

int projector_module_release(struct inode *inode, struct file *file)
{
		return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
void projector_module_early_suspend(struct early_suspend *h)
{
	pr_info("%s\n", __func__);
	if (status == PRJ_OFF) {
		gpio_direction_output(info->pdata->gpio_prj_on, GPIO_LEVEL_LOW);
	}
}

void projector_module_late_resume(struct early_suspend *h)
{
}

#endif


static struct file_operations projector_module_fops = {
	.owner = THIS_MODULE,
	.open = projector_module_open,
	.release = projector_module_release,
};

static struct miscdevice projector_module_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "secProjector",
	.fops = &projector_module_fops,
};

void project_internal(int pattern)
{
	flush_workqueue(projector_work_queue);
	if (get_proj_status() == PRJ_OFF) {
		proj_testmode_pwron_seq();
	} else {
		if (get_proj_status() == PRJ_ON_RGB_LCD) {
			DPPDATAFLASH(Internal_pattern_direction);
			pwron_seq_source_res(INTERNAL_PATTERN);
			set_proj_status(PRJ_ON_INTERNAL);
		}

		if (get_proj_status() == RGB_LED_OFF) {
			DPPDATAFLASH(RGB_led_on);
			set_proj_status(PRJ_ON_INTERNAL);
		}

		switch (pattern) {
		case CHECKER:
			DPPDATAFLASH(I_4x4checker);
			verify_value = 20;
			break;
// BEGIN CSLBSuP PPNFAD a.jakubiak/m.wengierow
		case CHECKER_16x9:
			DPPDATAFLASH(I_16x9checker);
			verify_value = 40;
			break;
// END CSLBSuP PPNFAD a.jakubiak/m.wengierow
		case WHITE:
			DPPDATAFLASH(I_white);
			verify_value = 21;
			break;
		case BLACK:
			DPPDATAFLASH(I_black);
			verify_value = 22;
			break;
		case LEDOFF:
			DPPDATAFLASH(RGB_led_off);
			set_proj_status(RGB_LED_OFF);
			verify_value = 23;
			break;
		case RED:
			DPPDATAFLASH(I_red);
			verify_value = 24;
			break;
		case GREEN:
			DPPDATAFLASH(I_green);
			verify_value = 25;
			break;
		case BLUE:
			DPPDATAFLASH(I_blue);
			verify_value = 26;
			break;
		case BEAUTY:
			verify_value = 27;
			break;
		case STRIPE:
			DPPDATAFLASH(I_stripe);
			verify_value = 28;
			break;
		default:
			break;
		}
	}
	printk(KERN_ERR "[%s] : pattern = %d\n", __func__,pattern);
}


/* BEAM_HW : motor step */
void move_motor_step(int value)
{
	int i, difference;

	difference = value - motor_abs_step;

	if (!once_motor_verified) {
		for (i = 0; i < MAX_MOTOR_STEP; i++) {
			M08980CTRL(Motor_CW_out);
			printk(KERN_INFO "[%s] CW %d step\n", __func__, i);
		}

		motor_abs_step = 0;
		msleep(25);

		for (i = 0; i < value; i++) {
			M08980CTRL(Motor_CCW_out);
			printk(KERN_INFO "[%s] CCW %d step\n", __func__, i);
			motor_abs_step++;
		}
		once_motor_verified = 1;
	}
	else{
		if (difference < 0) {
			for (i = 0; i < -1 * difference; i++) {
				M08980CTRL(Motor_CW_out);
				printk(KERN_INFO "[%s] CW 1 step\n", __func__);
				motor_abs_step--;
			}
		} else if (difference > 0) {
			for (i = 0; i < difference; i++) {
				M08980CTRL(Motor_CCW_out);
				printk(KERN_INFO "[%s] CCW 1 step\n", __func__);
				motor_abs_step++;
			}
		}
	}

	printk(KERN_INFO "[%s] Projector Motor ABS Step : %d\n",
			__func__, motor_abs_step);
	verify_value = 300 + value;
}

static ssize_t store_motor_action(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int direction;

	sscanf(buf, "%d\n", &direction);
	flush_workqueue(stepmotor_work_queue);

	if (status != PRJ_OFF) {
		M08980CTRL(M08980_PI_LDO_Enable);
		
		switch (direction) {
		case MOTOR_CW:
			projector_motor_cw();
			break;
		case MOTOR_CCW:
			projector_motor_ccw();
			break;
		default:
			break;
		}
		
		M08980CTRL(M08980_PI_LDO_Disable);
	}

	return count;
}

static ssize_t store_brightness(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d\n", &value);

	if (get_proj_status() == PRJ_OFF) {
		switch (value) {
		case 1:
			brightness = BRIGHT_HIGH;
			break;
		case 2:
			brightness = BRIGHT_MID;
			break;
		case 3:
			brightness = BRIGHT_LOW;
			break;
		default:
			break;
		}
	} else {
		brightness = value;
		set_led_current(value);
		printk(KERN_INFO "[%s] Proj Brightness Changed : %d\n",
					__func__, value);
	}
	return count;
}

static ssize_t store_proj_key(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d\n", &value);

	flush_workqueue(projector_work_queue);
	switch (value) {
	case 0:
		ProjectorPowerOffSequence();
		verify_value = 0;
		break;
	case 1:
		if (get_proj_status() != PRJ_ON_RGB_LCD) {
			ProjectorPowerOnSequence();
			verify_value = 10;
		}
		break;
	default:
		break;
	};

	printk(KERN_INFO "[%s] -->  %d\n", __func__, value);
	return count;
}

static ssize_t store_keystone_control(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d\n", &value);

	switch (value) {
	case 0:
		DPPDATAFLASH(Keystone_Disable);
		break;
	case 1:
		DPPDATAFLASH(Keystone_Enable);
		break;
	default:
		break;
	}

	printk(KERN_INFO "[%s] -->  %d\n", __func__, value);
	return count;
}

static ssize_t store_keystone_set_angle(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	
	char value;

	sscanf(buf, "%d\n", &value);

	if(value >= -40 && value <= 40)
	{
		Keystone_SetAngle[4] = value;
		DPPDATAFLASH(Keystone_SetAngle);
	}
	
	printk(KERN_INFO "[%s] -->  %d\n", __func__, value);
	return count;
}

static ssize_t store_labb_control(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d\n", &value);

	switch (value) {
	case 0:
		LABB_Control[3] = LABB_OFF;
		DPPDATAFLASH(LABB_Control);
		break;
	case 1:
		LABB_Control[3] = LABB_ON;
		DPPDATAFLASH(LABB_Control);
		break;
	default:
		break;
	}

	printk(KERN_INFO "[%s] -->  %d\n", __func__, value);
	return count;
}

static ssize_t store_labb_set_strength(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d\n", &value);

	if(value >= 2 && value <= 150)
	{
		LABB_Control[4] = value;
		DPPDATAFLASH(LABB_Control);
	}

	printk(KERN_INFO "[%s] -->  %d\n", __func__, value);
	return count;
}
	
/* BEAM_HW : old_fw */
static ssize_t show_cal_history(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int size, result = 0;

	result = not_calibrated;

	size = sprintf(buf, "%d\n", result);

	return size;
}

static ssize_t show_screen_direction(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int size;

	size = sprintf(buf, "%d\n", screen_direction);

	return size;
}

static ssize_t show_retval(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int size;

	size = sprintf(buf, "%d\n", verify_value);

	return size;
}

static ssize_t store_screen_direction(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int  value;

	sscanf(buf, "%d\n", &value);

	if (value >= 0 && value <= 3) {
		screen_direction = value;
	}

	return count;
}

static ssize_t store_rotate_screen(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d\n", &value);

	if (value >= 0 && value <= 3) {
		flush_workqueue(projector_work_queue);
		screen_direction = value;
		queue_work(projector_work_queue, &projector_work_rotate_screen);

		printk(KERN_INFO "[%s] inputed rotate : %d\n", __func__, value);
	}

	return count;
}

static ssize_t store_projection_verify(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d\n", &value);
	printk(KERN_INFO "[%s] selected internal pattern : %d\n",
				__func__, value);


	if (value == CURTAIN_ON) {
		if (status != PRJ_ON_INTERNAL)
			return count;
		DPPDATAFLASH(Output_Curtain_Enable);
	} else if (value == CURTAIN_OFF) {
		if (status != PRJ_ON_INTERNAL)
			return count;
		DPPDATAFLASH(Output_Curtain_Disable);
	} else {
		saved_pattern = value;
		project_internal(value);
	}
	return count;
}


static ssize_t store_motor_verify(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d\n", &value);

	if (value >= 0 && value <= 60) {
		move_motor_step(value);
	}

	return count;
}

// BEGIN CSLBSuP PPNFAD a.jakubiak/m.wengierow
void int2bytes( unsigned int input, unsigned char *low, unsigned char *high ) {
	// this code is taken from a presentation "PAD100 Control"
	*high = input / 256;
	*low  = input - ( 256 * (*high) );
}

void set_custom_led_current(int red, int green, int blue) {
	unsigned char rh, rl, gh, gl, bh, bl;
	
	printk(KERN_INFO "CSLBSuP [%s] red=%d green=%d blue=%d\n", __func__, red, green, blue);
	
	int2bytes( (unsigned int)red, &rl, &rh );
	int2bytes( (unsigned int)green, &gl, &gh );
	int2bytes( (unsigned int)blue, &bl, &bh );	
	
	RED_MSB_DAC[3] = rh;
	RED_LSB_DAC[3] = rl;
	GREEN_MSB_DAC[3] = gh;
	GREEN_LSB_DAC[3] = gl;
	BLUE_MSB_DAC[3] = bh;
	BLUE_LSB_DAC[3] = bl;

	M08980CTRL(RED_MSB_DAC);
	M08980CTRL(RED_LSB_DAC);
	M08980CTRL(GREEN_MSB_DAC);
	M08980CTRL(GREEN_LSB_DAC);
	M08980CTRL(BLUE_MSB_DAC);
	M08980CTRL(BLUE_LSB_DAC);
	

	printk(KERN_INFO "CSLBSuP [%s] done red=%d green=%d blue=%d\n", __func__, red, green, blue);
}

static ssize_t store_led_current(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int red, green, blue;

	sscanf(buf, "%d %d %d\n", &red, &green, &blue);

	if (get_proj_status() == PRJ_OFF) {
		printk(KERN_INFO "CSLBSuP: [%s] Projector led current NOT changed : %d %d %d\n", __func__, red, green, blue);
	} else {
		set_custom_led_current(red, green, blue);
		printk(KERN_INFO "CSLBSuP: [%s] Projector led current changed : %d %d %d\n", __func__, red, green, blue);
	}
	return count;
}


void get_custom_led_current(int *red, int *green, int *blue) {
	
	unsigned char rh, rl, gh, gl, bh, bl;
	printk(KERN_INFO "CSLBSuP: [%s] ", __func__);
	
	
	rh = RED_MSB_DAC[3];
	rl = RED_LSB_DAC[3];
	gh = GREEN_MSB_DAC[3];
	gl = GREEN_LSB_DAC[3];
	bh = BLUE_MSB_DAC[3];
	bl = BLUE_LSB_DAC[3];
	

	*red   = rh * 256 + rl;
	*green = gh * 256 + gl;
	*blue  = bh * 256 + bl;

	printk(KERN_INFO "CSLBSuP [%s] done red=%d green=%d blue=%d\n", __func__, *red, *green, *blue);
}

static ssize_t show_led_current(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int size;
	int red, green, blue;

	get_custom_led_current(&red, &green, &blue);

	size = sprintf(buf, "%d %d %d\n", red, green, blue);

	return size;
}

void set_led_duty(int duty) {
	
	unsigned char d = (unsigned char)duty;
	printk(KERN_INFO "CSLBSuP [%s] duty=%d\n", __func__, duty);

	ChangeDuty(Dmd_seq, d);
	DPPDATAFLASH(Dmd_seq);
	

	printk(KERN_INFO "CSLBSuP [%s] done duty=%d\n", __func__, duty);
}

static ssize_t store_led_duty(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int duty;

	sscanf(buf, "%d\n", &duty);

	if (get_proj_status() == PRJ_OFF) {
		printk(KERN_INFO "CSLBSuP: [%s] Projector led duty NOT changed : %d\n", __func__, duty);
	} else {
		set_led_duty(duty);
		printk(KERN_INFO "CSLBSuP: [%s] Proj led duty changed : %d\n", __func__, duty);
	}
	return count;
}


void get_led_duty(int *duty) {

	unsigned char d;
	printk(KERN_INFO "CSLBSuP: [%s] ", __func__);

	d = RGB_BUF[18];

	*duty  = d;

	printk(KERN_INFO "CSLBSuP [%s] done duty=%c\n", __func__, (int) *duty);
}


static ssize_t show_led_duty(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int size;
	int duty;

	get_led_duty(&duty);

	size = sprintf(buf, "%d\n", duty);

	return size;
}
// END CSLBSuP PPNFAD a.jakubiak/m.wengierow

//static ssize_t store_read_test(struct device *dev,
//		struct device_attribute *attr, const char *buf, size_t count)
//{
//	//int value;
//
//	ProjectorReadTest();
//
//	return count;
//}

static DEVICE_ATTR(proj_motor, S_IRUGO | S_IWUSR, NULL, store_motor_action);
static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR, NULL, store_brightness);
static DEVICE_ATTR(proj_key, S_IRUGO | S_IWUSR, NULL, store_proj_key);
static DEVICE_ATTR(cal_history, S_IRUGO, show_cal_history, NULL);
static DEVICE_ATTR(rotate_screen, S_IRUGO | S_IWUSR,NULL, store_rotate_screen);
static DEVICE_ATTR(screen_direction, S_IRUGO | S_IWUSR,	show_screen_direction, store_screen_direction);
static DEVICE_ATTR(projection_verify, S_IRUGO | S_IWUSR,	NULL, store_projection_verify);
static DEVICE_ATTR(motor_verify, S_IRUGO | S_IWUSR,NULL, store_motor_verify);
//static DEVICE_ATTR(read_test, S_IRUGO | S_IWUGO,NULL, store_read_test);
static DEVICE_ATTR(retval, S_IRUGO, show_retval, NULL);
static DEVICE_ATTR(proj_keystone, S_IRUGO | S_IWUSR, NULL, store_keystone_control);
static DEVICE_ATTR(keystone_angle, S_IRUGO | S_IWUSR, NULL, store_keystone_set_angle);
static DEVICE_ATTR(proj_labb, S_IRUGO | S_IWUSR, NULL, store_labb_control);
static DEVICE_ATTR(labb_strength, S_IRUGO | S_IWUSR, NULL, store_labb_set_strength);
// BEGIN CSLBSuP PPNFAD a.jakubiak/m.wengierow
static DEVICE_ATTR(led_current, S_IRUGO | S_IWUSR, show_led_current, store_led_current);	
static DEVICE_ATTR(led_duty   , S_IRUGO | S_IWUSR, show_led_duty   , store_led_duty);
// END CSLBSuP PPNFAD a.jakubiak/m.wengierow
int __init projector_module_init(void)
{
	int ret;

printk(KERN_INFO "CSLBSuP: projector_module_init \n");

	sec_projector = device_create(sec_class, NULL, 0, NULL, "sec_projector");
	if (IS_ERR(sec_projector)) {
		printk(KERN_ERR "Failed to create device(sec_projector)!\n");
	}

	if (device_create_file(sec_projector, &dev_attr_proj_motor) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_proj_motor.attr.name);

	if (device_create_file(sec_projector, &dev_attr_brightness) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_brightness.attr.name);

	if (device_create_file(sec_projector, &dev_attr_proj_key) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_proj_key.attr.name);

	if (device_create_file(sec_projector, &dev_attr_proj_keystone) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_proj_keystone.attr.name);
				
	if (device_create_file(sec_projector, &dev_attr_proj_labb) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_proj_labb.attr.name);
				
	if (device_create_file(sec_projector, &dev_attr_keystone_angle) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_keystone_angle.attr.name);
				
	if (device_create_file(sec_projector, &dev_attr_labb_strength) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_labb_strength.attr.name);

	if (device_create_file(sec_projector, &dev_attr_cal_history) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_cal_history.attr.name);

	if (device_create_file(sec_projector, &dev_attr_rotate_screen) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_rotate_screen.attr.name);

	if (device_create_file(sec_projector, &dev_attr_screen_direction) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_screen_direction.attr.name);

	if (device_create_file(sec_projector, &dev_attr_projection_verify) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_projection_verify.attr.name);

	if (device_create_file(sec_projector, &dev_attr_motor_verify) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_motor_verify.attr.name);

	if (device_create_file(sec_projector, &dev_attr_retval) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_retval.attr.name);

// BEGIN CSLBSuP PPNFAD a.jakubiak/m.wengierow
	if (device_create_file(sec_projector, &dev_attr_led_current) < 0)
		printk(KERN_ERR "CSLBSuP: Failed to create device file(%s)!\n",
				dev_attr_led_current.attr.name);

	if (device_create_file(sec_projector, &dev_attr_led_duty) < 0)
		printk(KERN_ERR "CSLBSuP: Failed to create device file(%s)!\n",
				dev_attr_led_duty.attr.name);
// END CSLBSuP PPNFAD a.jakubiak/m.wengierow

	ret = i2c_add_driver(&dpp3430_i2c_driver);
	ret = i2c_add_driver(&M08980_i2c_driver);
	ret |= misc_register(&projector_module_device);
	if (ret) {
		printk(KERN_ERR "Projector driver registration failed!\n");
	}
	return ret;
}

void __exit projector_module_exit(void)
{
	i2c_del_driver(&dpp3430_i2c_driver);
	misc_deregister(&projector_module_device);
}

late_initcall(projector_module_init);
module_exit(projector_module_exit);

MODULE_DESCRIPTION("Samsung projector module driver");
MODULE_AUTHOR("Inbum Choi <inbum.choi@samsung.com>");
MODULE_LICENSE("GPL");
