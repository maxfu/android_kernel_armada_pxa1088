/*
* Copyright (C) 2006-2013, Samsung Electronics Co., Ltd. All Rights Reserved.
* Written by System S/W Group, Mobile Communication Division.
*/

#ifndef __VX6B3E_H
#define __VX6B3E_H


extern int vx6b3e_brige_init(struct pxa168fb_info *fbi);
extern int vx6b3e_reset(void);
extern int vx6b3e_power(struct pxa168fb_info *fbi, int on);

extern int vx6b3e_mipi_write(struct pxa168fb_info *fbi, u32 address, u32 value, u32 data_size);
extern int vx6b3e_i2c_read32(u16 reg, u32 *pval);
extern int vx6b3e_i2c_read(u32 addr, u32 *val, u32 data_size);

extern int vx6b3e_i2c_write32(u16 reg, u32 val);
extern int vx6b3e_i2c_release(void);
extern int vx6b3e_i2cTomipi_write(int dtype, int vit_com, u8 data_size, u8 *ptr );
extern int vx6b3e_i2cTomipi_read(int dtype, int vit_com, u8 data_size, u8 reg , u8 *pval);

extern int mpmu_vcxoen(void);

extern int get_panel_id(void);
extern int hx8369_backlight_updata(void);

#define VX6B3E_CONTROL_BY_I2C

#define VX6B3E_PWR_EN_VXE_1P8 64
#define VX6B3E_PWR_EN_VXE_1P2 63
#define VX6B3E_RESET 125

/*13M_EN FOR F/F*/
#define GPIO_13M_EN (30)


/*LCD/VX Bridge h/w switch*/
#define GPIO_LCD_VXMIPISWITCH_EN (33)
#define GPIO_LCD_SWITCH_OE (16)

#define GPIO_LCD_VXMIPISWITCH2_EN (34)
#define GPIO_LCD_SWITCH2_OE (17)



#define VX6B3E_MIPI_VENDOR_ID_1		0x5
#define VX6B3E_MIPI_VENDOR_ID_2		0x1
#define VX6B3E_MIPI_COMMAND_CSR_WRITE	0x40
#define VX6B3E_MIPI_COMMAND_CSR_OFFSET	0x41

#define VX6B3E_IIC_READ_CONTROL 0x0E
#define VX6B3E_IIC_WRITE_CONTROL 0x0A
#define CONTROL_BYTE_I2C_RELEASE (0x0u)
#define VX6B3E_IIC_RELEASE  {\
		CONTROL_BYTE_I2C_RELEASE, \
}


#define QL_MIPI_PANEL_CMD_SIZE 255
#define QL_VX_LCD_VC 0/* dcs read/write */
#define CONTROL_BYTE_DCS       (0x08u)
#define CONTROL_BYTE_GEN       (0x09u)

#define DTYPE_DCS_WRITE		0x05	/* short write, 0 parameter */
#define DTYPE_DCS_WRITE1	0x15	/* short write, 1 parameter */
#define DTYPE_DCS_READ		0x06	/* read */
#define DTYPE_DCS_LWRITE	0x39	/* long write *//* generic read/write */
#define DTYPE_GEN_WRITE		0x03	/* short write, 0 parameter */
#define DTYPE_GEN_WRITE1	0x13	/* short write, 1 parameter */
#define DTYPE_GEN_WRITE2	0x23	/* short write, 2 parameter */
#define DTYPE_GEN_LWRITE	0x29	/* long write */
#define DTYPE_GEN_READ		0x04	/* long read, 0 parameter */
#define DTYPE_GEN_READ1		0x14	/* long read, 1 parameter */
#define DTYPE_GEN_READ2		0x24	/* long read, 2 parameter */

/*Brightness level*/

#define MIN_BRIGHTNESS			0
#define MAX_BRIGHTNESS_LEVEL		255
#define MID_BRIGHTNESS_LEVEL		150
#define LOW_BRIGHTNESS_LEVEL		20
#define DIM_BRIGHTNESS_LEVEL		20
#define DEFAULT_BRIGHTNESS		MID_BRIGHTNESS_LEVEL


/* CABC PWM */
static char hx8369b_backlight_51h[] = {
	0x51,
	0x3F, 0x3C
};

/**************************************************
*		VX6B3E MACRO FOR MIPI			*
***************************************************/

static char vx6b3e_csr_wr_payload[9] = {
					VX6B3E_MIPI_VENDOR_ID_1,
					VX6B3E_MIPI_VENDOR_ID_2,
					VX6B3E_MIPI_COMMAND_CSR_WRITE,
					0x0, 0x0,	/* address 16bits */
					0x0, 0x0, 0x0, 0x0 /* data max 32bits */
};


/**************************************************
*		VX6B3E MACRO FOR I2C			*
***************************************************/

#define GEN_QL_CSR_OFFSET_LENGTH  {\
		CONTROL_BYTE_GEN, \
        0x29,  /* Data ID */\
        0x05,  /* Vendor Id 1 */\
        0x01,  /* Vendor Id 2 */\
        0x41,  /* Vendor Unique Command */\
        0x00,  /* Address LS */\
        0x00,  /* Address MS */\
        0x00,  /* Length LS */\
        0x00,  /* Length MS */\
    }

#define GEN_QL_CSR_WRITE  {\
		CONTROL_BYTE_GEN, \
        0x29,  /* Data ID */\
        0x05,  /* Vendor Id 1 */\
        0x01,  /* Vendor Id 2 */\
        0x40,  /* Vendor Unique Command */\
        0x00,  /* Address LS */\
        0x00,  /* Address MS */\
        0x00,  /* data LS */\
	0x00, \
	0x00, \
        0x00,  /* data MS */\
    }

/**************************************************
*		Structure for VX6B3E Driver control			*
***************************************************/

enum CABC {
	CABC_OFF,
	CABC_ON,
	CABC_MAX,
};

struct Vx6b3e_backlight_value {
	const unsigned int max;
	const unsigned int mid;
	const unsigned char low;
	const unsigned char dim;
};

typedef struct Vx6b3e_bridge_info {
	enum CABC			cabc;

	struct class			*Mdnie;
	struct backlight_device		*bd;
	struct lcd_device		*lcd;
	struct device			*dev_mdnie;
	struct device			*dev_bd;	
	struct Vx6b3e_backlight_value	*vee_lightValue;
	struct mutex			lock;

	int				negative;
	unsigned int			auto_brightness;
};

#endif
