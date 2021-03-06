#ifndef __HX8389B_PARAM_H__
#define __HX8389B_PARAM_H__

#include <mach/pxa168fb.h>

#define LP_MODE	(1)
#define HS_MODE (0)

#define HX8389B_SLEEP_OUT_DELAY (120)
#define HX8389B_DISP_ON_DELAY (10)
#define HX8389B_DISP_OFF_DELAY (120)
#define HX8389B_SLEEP_IN_DELAY (120)

#define HX8389B_REG_ID_1 0xDA
#define HX8389B_REG_ID_2 0xDB
#define HX8389B_REG_ID_3 0xDC

#define CONFIG_LDI_SUPPORT_MDNIE
#define ENABLE_MDNIE_TUNING

#ifdef CONFIG_LDI_SUPPORT_MDNIE
#define HX8389B_REG_MDNIE 0xCD
#endif

static u8 exit_sleep[] = {0x11, 0x00};
static u8 display_on[] = {0x29, 0x00};
static u8 display_off[] = {0x28, 0x00};
static u8 enter_sleep[] = {0x10, 0x00};

static u8 pkt_size_cmd[] = {0x1};
static u8 pkt_size_2_cmd[] = {0x2};
static u8 read_id1[] = {0xda};
static u8 read_id2[] = {0xdb};
static u8 read_id3[] = {0xdc};
static u8 read_esd[] = {0x0a};

static struct dsi_cmd_desc hx8389b_video_read_id1_cmds[] = {
	{DSI_DI_SET_MAX_PKT_SIZE, HS_MODE, 0, sizeof(pkt_size_cmd), pkt_size_cmd},
	{DSI_DI_DCS_READ, HS_MODE, 0, sizeof(read_id1), read_id1},
};

static struct dsi_cmd_desc hx8389b_video_read_id2_cmds[] = {
	{DSI_DI_SET_MAX_PKT_SIZE, HS_MODE, 0, sizeof(pkt_size_cmd), pkt_size_cmd},
	{DSI_DI_DCS_READ, HS_MODE, 0, sizeof(read_id2), read_id2},
};

static struct dsi_cmd_desc hx8389b_video_read_id3_cmds[] = {
	{DSI_DI_SET_MAX_PKT_SIZE, HS_MODE, 0, sizeof(pkt_size_cmd), pkt_size_cmd},
	{DSI_DI_DCS_READ, HS_MODE, 0, sizeof(read_id3), read_id3},
};

static struct dsi_cmd_desc hx8389b_video_read_esd_cmds[] = {
	{DSI_DI_SET_MAX_PKT_SIZE, HS_MODE, 0, sizeof(pkt_size_cmd), pkt_size_cmd},
	{DSI_DI_DCS_READ, HS_MODE, 0, sizeof(read_esd), read_esd},
};


static u8 hx8389b_reg_B9h[] = {
	0xB9,
	0xFF, 0x83, 0x89,
};

static u8 hx8389b_reg_DEh[] = {
	0xDE,
	0x05, 0x58,
};

static u8 hx8389b_reg_B1h[] = {
	0xB1,
	0x00, 0x00, 0x07, 0xEF, 0x50, 0x05, 0x11, 0x74, 
	0xF1, 0x2A, 0x34, 0x26, 0x26, 0x42, 0x01, 0x3A, 
	0xF5, 0x20, 0x80,
};

static u8 hx8389b_reg_B2h[] = {
	0xB2,
	0x00, 0x00, 0x78, 0x04, 0x07, 0x3F, 0x40,
};

static u8 hx8389b_reg_B4h[] = {
	0xB4,
	0x80, 0x08, 0x00, 0x32, 0x10, 0x00, 0x32, 0x13, 
	0xC7, 0x00, 0x00, 0x00, 0x35, 0x00, 0x40, 0x04, 
	0x37, 0x0A, 0x40, 0x1E, 0x52, 0x52, 0x0A, 0x0A, 
	0x40, 0x0A, 0x40, 0x14, 0x46, 0x50, 0x0A
};

static u8 hx8389b_reg_B6h[] = {
	0xB6,
	0x00, 0x91, 0x00, 0x91,
};

static u8 hx8389b_reg_CCh[] = {
	0xCC,
	0x0E
};

static u8 hx8389b_reg_D5h[] = {
	0xD5,
	0x80, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 
	0x00, 0x60, 0x88, 0x88, 0x99, 0x88, 0x01, 0x45, 
	0x88, 0x88, 0x01, 0x45, 0x23, 0x67, 0x88, 0x88, 
	0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 
	0x99, 0x54, 0x10, 0x88, 0x88, 0x76, 0x32, 0x54, 
	0x10, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88
};

static u8 hx8389b_reg_BAh[] = {
	0xBA,
	0x41, 0x93,
};

static u8 hx8389b_reg_35h[] = {
	0x35,
	0x00,
};

static u8 hx8389b_reg_C0h[] = {
	0xC0,
	0x43, 0x17,
};

static u8 hx8389b_reg_CBh[] = {
	0xCB,
	0x07, 0x07,
};

static u8 hx8389b_reg_E0h[] = {
	0xE0,
	0x00, 0x10, 0x18, 0x3A, 0x3D, 0x3F, 0x26, 0x46, 
	0x07, 0x0C, 0x0E, 0x12, 0x14, 0x12, 0x13, 0x11, 
	0x18, 0x00, 0x10, 0x18, 0x3A, 0x3D, 0x3F, 0x26, 
	0x46, 0x07, 0x0C, 0x0E, 0x12, 0x14, 0x12, 0x13, 
	0x11, 0x18
};

static u8 hx8389b_reg_C1h[] = {
	0xC1,
	0x01, 0x00, 0x08, 0x10, 0x1A, 0x21, 0x29, 0x31, 
	0x37, 0x3F, 0x47, 0x50, 0x58, 0x60, 0x68, 0x70, 
	0x78, 0x81, 0x88, 0x90, 0x99, 0xA0, 0xA7, 0xAF, 
	0xB7, 0xC0, 0xC9, 0xCE, 0xD6, 0xE0, 0xE7, 0xF1, 
	0xF8, 0xFF, 0xFB, 0x63, 0xA1, 0x2A, 0x7D, 0x69, 
	0x8E, 0x80, 0x00, 0x00, 0x08, 0x10, 0x1A, 0x21, 
	0x29, 0x31, 0x37, 0x3F, 0x47, 0x50, 0x58, 0x60, 
	0x68, 0x70, 0x78, 0x81, 0x88, 0x90, 0x99, 0xA0, 
	0xA7, 0xAF, 0xB7, 0xC0, 0xC9, 0xCE, 0xD6, 0xE0, 
	0xE7, 0xF1, 0xF8, 0xFF, 0xFB, 0x63, 0xA1, 0x2A, 
	0x7D, 0x69, 0x8E, 0x80, 0x00, 0x00, 0x08, 0x10, 
	0x1A, 0x21, 0x29, 0x31, 0x37, 0x3F, 0x47, 0x50, 
	0x58, 0x60, 0x68, 0x70, 0x78, 0x81, 0x88, 0x90, 
	0x99, 0xA0, 0xA7, 0xAF, 0xB7, 0xC0, 0xC9, 0xCE, 
	0xD6, 0xE0, 0xE7, 0xF1, 0xF8, 0xFF, 0xFB, 0x63, 
	0xA1, 0x2A, 0x7D, 0x69, 0x8E, 0x80, 0x00
};

static u8 hx8389b_reg_BAh_ULPS[] = {
	0xBA,
	0x41, 0x93, 0x00, 0x16, 0xA4, 0x10
};

static struct dsi_cmd_desc hx8389b_power_setting_table[] = {
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_B9h), hx8389b_reg_B9h},
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_DEh), hx8389b_reg_DEh},
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_B1h), hx8389b_reg_B1h},
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_B2h), hx8389b_reg_B2h},
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_B4h), hx8389b_reg_B4h},
};

static struct dsi_cmd_desc hx8389b_init_table[] = {
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_B9h), hx8389b_reg_B9h},
	{DSI_DI_DCS_SWRITE1, HS_MODE, 0, sizeof(hx8389b_reg_CCh), hx8389b_reg_CCh},
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_D5h), hx8389b_reg_D5h},
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_BAh), hx8389b_reg_BAh},
	{DSI_DI_DCS_SWRITE, HS_MODE, 0, sizeof(hx8389b_reg_35h), hx8389b_reg_35h},
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_C0h), hx8389b_reg_C0h},
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_CBh), hx8389b_reg_CBh},
};

static struct dsi_cmd_desc hx8389b_gamma_setting_table[] = {
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_E0h), hx8389b_reg_E0h},
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_C1h), hx8389b_reg_C1h},
};

static struct dsi_cmd_desc hx8389b_display_on_table[] = {
	{DSI_DI_DCS_SWRITE, HS_MODE, 0/*HX8389B_DISP_ON_DELAY*/, sizeof(display_on),display_on},
};

static struct dsi_cmd_desc hx8389b_sleep_out_table[] = {
	{DSI_DI_DCS_SWRITE, HS_MODE, 0/*HX8389B_SLEEP_OUT_DELAY*/, sizeof(exit_sleep),exit_sleep},
};

static struct dsi_cmd_desc hx8389b_display_off_table[] = {
	{DSI_DI_DCS_SWRITE, HS_MODE, 0/*HX8389B_DISP_OFF_DELAY*/, sizeof(display_off),display_off},
};

static struct dsi_cmd_desc hx8389b_sleep_in_table[] = {
	{DSI_DI_DCS_SWRITE, HS_MODE, 0, sizeof(enter_sleep),enter_sleep},
};

static struct dsi_cmd_desc hx8389b_ULPS_table[] = {
	{DSI_DI_DCS_LWRITE, HS_MODE, 0, sizeof(hx8389b_reg_BAh_ULPS), hx8389b_reg_BAh_ULPS},
};

#ifdef CONFIG_LDI_SUPPORT_MDNIE
enum SCENARIO {
	UI_MODE,
	VIDEO_MODE,
	VIDEO_WARM_MODE,
	VIDEO_COLD_MODE,
	CAMERA_MODE,
	NAVI_MODE,
	GALLERY_MODE,
	VT_MODE,
	SCENARIO_MAX,
};

enum OUTDOOR {
	OUTDOOR_OFF,
	OUTDOOR_ON,
	OUTDOOR_MAX,
};

typedef struct mdnie_config {
	int scenario;
	int negative;
	int outdoor;
};
 
static char mDNIe_UI_MODE[] = {
	0xCD,
    0x5A, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 
    0x02, 0xff, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0xa1, 0x05, 0xa2, 0x09, 0xa0, 0x04, 
    0xa0, 0x04, 0xac, 0x1c, 0xac, 0x1c, 0xa8, 0x15, 0xa8, 0x15, 0xa8, 0x15, 0xa0, 0x0a, 0xa0, 
    0x0a, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 
    0x98, 0x01, 0x08, 0x00, 0x04, 0x5b, 0x1f, 0xc4, 0x1f, 0xe1, 0x1f, 0xf4, 0x04, 0x2b, 0x1f, 
    0xe1, 0x1f, 0xf4, 0x1f, 0xc4, 0x04, 0x48
};
static char mDNIe_VIDEO_MODE[] = {
	0xCD,
    0x5A, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 
    0x06, 0xff, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0xa1, 0x05, 0xa2, 0x09, 0xa0, 0x04, 
    0xa0, 0x04, 0xac, 0x1c, 0xac, 0x1c, 0xa8, 0x15, 0xa8, 0x15, 0xa8, 0x15, 0xa0, 0x0a, 0xa0, 
    0x0a, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 
    0x98, 0x01, 0x08, 0x00, 0x04, 0x5b, 0x1f, 0xc4, 0x1f, 0xe1, 0x1f, 0xf4, 0x04, 0x2b, 0x1f, 
    0xe1, 0x1f, 0xf4, 0x1f, 0xc4, 0x04, 0x48
};
static char mDNIe_VIDEO_WARM_MODE[] = {
	0xCD,
	0x5A, 0x00, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE3, 0x00, 0xF3, 0x00, 0xFF, 0xFF, 0x00, 0x00,
	0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00,
	0x06, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00,
	0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20,
	0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00,
	0x20, 0x00, 0x20, 0x00, 0x04, 0xb5, 0x1f, 0x88, 0x1f, 0xc3, 0x1f, 0xe9, 0x04, 0x54, 0x1f,
	0xc3, 0x1f, 0xe9, 0x1f, 0x88, 0x04, 0x8f
};
static char mDNIe_VIDEO_COLD_MODE[] = {
	0xCD,
	0x5A, 0x00, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xed, 0x00, 0xe1, 0xFF, 0x00, 0x00,
	0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00,
	0x06, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00,
	0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20,
	0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00,
	0x20, 0x00, 0x20, 0x00, 0x04, 0xb5, 0x1f, 0x88, 0x1f, 0xc3, 0x1f, 0xe9, 0x04, 0x54, 0x1f,
	0xc3, 0x1f, 0xe9, 0x1f, 0x88, 0x04, 0x8f
};
static char mDNIe_CAMERA_MODE[] = {
	0xCD,
    0x5A, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 
    0x06, 0xff, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0xa1, 0x05, 0xa2, 0x09, 0xa0, 0x04, 
    0xa0, 0x04, 0xac, 0x1c, 0xac, 0x1c, 0xa8, 0x15, 0xa8, 0x15, 0xa8, 0x15, 0xa0, 0x0a, 0xa0, 
    0x0a, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 
    0x98, 0x01, 0x08, 0x00, 0x04, 0x5b, 0x1f, 0xc4, 0x1f, 0xe1, 0x1f, 0xf4, 0x04, 0x2b, 0x1f, 
    0xe1, 0x1f, 0xf4, 0x1f, 0xc4, 0x04, 0x48
};
static char mDNIe_NAVI_MODE[] = {
	0xCD,
	0x00
};
static char mDNIe_GALLERY_MODE[] = {
	0xCD,
    0x5A, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 
    0x02, 0xff, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0xa1, 0x05, 0xa2, 0x09, 0xa0, 0x04, 
    0xa0, 0x04, 0xac, 0x1c, 0xac, 0x1c, 0xa8, 0x15, 0xa8, 0x15, 0xa8, 0x15, 0xa0, 0x0a, 0xa0, 
    0x0a, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 0x98, 0x01, 
    0x98, 0x01, 0x08, 0x00, 0x04, 0x5b, 0x1f, 0xc4, 0x1f, 0xe1, 0x1f, 0xf4, 0x04, 0x2b, 0x1f, 
    0xe1, 0x1f, 0xf4, 0x1f, 0xc4, 0x04, 0x48
};
static char mDNIe_VT_MODE[] = {
	0xCD,
	0x00
};
static char mDNIe_NEGATIVE_MODE[] = {
	0xCD,
    0x5A, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 
    0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0xff, 
    0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 
    0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 
    0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 
    0x20, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00
 };
static char mDNIe_VIDEO_OUTDOOR_MODE[] = {
	0xCD,
	0x5A, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00,
	0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00,
	0x07, 0xFF, 0x00, 0x0A, 0xAF, 0x0D, 0x99, 0x14, 0x6D, 0x1B, 0x48, 0x2C, 0x05, 0xB4, 0x0F,
	0xBE, 0x26, 0xBE, 0x26, 0xBE, 0x26, 0xBE, 0x26, 0xAE, 0x0C, 0xAE, 0x0C, 0xAE, 0x0C, 0xAE,
	0x0C, 0xAE, 0x0C, 0xAE, 0x0C, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00,
	0x20, 0x00, 0x20, 0x00, 0x06, 0x7B, 0x1E, 0x5B, 0x1f, 0x2A, 0x1F, 0xAE, 0x05, 0x28, 0x1F,
	0x2A, 0x1f, 0xAE, 0x1E, 0x5B, 0x05, 0xF7
};
static char mDNIe_VIDEO_WARM_OUTDOOR_MODE[] = {
	0xCD,
	0x5A, 0x00, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE3, 0x00, 0xF3, 0x00, 0xFF, 0xFF, 0x00, 0x00,
	0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00,
	0x07, 0xFF, 0x00, 0x0A, 0xAF, 0x0D, 0x99, 0x14, 0x6D, 0x1B, 0x48, 0x2C, 0x05, 0xB4, 0x0F,
	0xBE, 0x26, 0xBE, 0x26, 0xBE, 0x26, 0xBE, 0x26, 0xAE, 0x0C, 0xAE, 0x0C, 0xAE, 0x0C, 0xAE,
	0x0C, 0xAE, 0x0C, 0xAE, 0x0C, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00,
	0x20, 0x00, 0x20, 0x00, 0x06, 0x7B, 0x1E, 0x5B, 0x1f, 0x2A, 0x1F, 0xAE, 0x05, 0x28, 0x1F,
	0x2A, 0x1f, 0xAE, 0x1E, 0x5B, 0x05, 0xF7
};
static char mDNIe_VIDEO_COLD_OUTDOOR_MODE[] = {
	0xCD,
	0x5A, 0x00, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xed, 0x00, 0xe1, 0xFF, 0x00, 0x00,
	0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00,
	0x07, 0xFF, 0x00, 0x0A, 0xAF, 0x0D, 0x99, 0x14, 0x6D, 0x1B, 0x48, 0x2C, 0x05, 0xB4, 0x0F,
	0xBE, 0x26, 0xBE, 0x26, 0xBE, 0x26, 0xBE, 0x26, 0xAE, 0x0C, 0xAE, 0x0C, 0xAE, 0x0C, 0xAE,
	0x0C, 0xAE, 0x0C, 0xAE, 0x0C, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00,
	0x20, 0x00, 0x20, 0x00, 0x06, 0x7B, 0x1E, 0x5B, 0x1f, 0x2A, 0x1F, 0xAE, 0x05, 0x28, 0x1F,
	0x2A, 0x1f, 0xAE, 0x1E, 0x5B, 0x05, 0xF7
};
static char mDNIe_CAMERA_OUTDOOR_MODE[] = {
	0xCD,
	0x5A, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00,
	0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x00,
	0x07, 0xff, 0x00, 0x0a, 0xaf, 0x0d, 0x99, 0x14, 0x6d, 0x1b, 0x48, 0x2c, 0x05, 0xb4, 0x0f,
	0xbe, 0x26, 0xbe, 0x26, 0xbe, 0x26, 0xbe, 0x26, 0xae, 0x0c, 0xae, 0x0c, 0xae, 0x0c, 0xae,
	0x0c, 0xae, 0x0c, 0xae, 0x0c, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00,
	0x20, 0x00, 0x20, 0x00, 0x06, 0x7b, 0x1e, 0x5b, 0x1f, 0x2a, 0x1f, 0xae, 0x05, 0x28, 0x1f,
	0x2a, 0x1f, 0xae, 0x1e, 0x5b, 0x05, 0xf7
};

static struct dsi_cmd_desc hx8389b_video_display_mDNIe_size[] = {
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_UI_MODE), mDNIe_UI_MODE}
};

static struct dsi_cmd_desc hx8389b_video_display_mDNIe_scenario_cmds[] = {
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_UI_MODE), mDNIe_UI_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_VIDEO_MODE), mDNIe_VIDEO_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_VIDEO_WARM_MODE), mDNIe_VIDEO_WARM_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_VIDEO_COLD_MODE), mDNIe_VIDEO_COLD_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_CAMERA_MODE), mDNIe_CAMERA_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_NAVI_MODE), mDNIe_NAVI_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_GALLERY_MODE), mDNIe_GALLERY_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_VT_MODE), mDNIe_VT_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_NEGATIVE_MODE), mDNIe_NEGATIVE_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_VIDEO_OUTDOOR_MODE), mDNIe_VIDEO_OUTDOOR_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_VIDEO_WARM_OUTDOOR_MODE), mDNIe_VIDEO_WARM_OUTDOOR_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_VIDEO_COLD_OUTDOOR_MODE), mDNIe_VIDEO_COLD_OUTDOOR_MODE},
	{DSI_DI_DCS_LWRITE, 0, 0,  sizeof(mDNIe_CAMERA_OUTDOOR_MODE), mDNIe_CAMERA_OUTDOOR_MODE},
};
#endif	/* CONFIG_LDI_SUPPORT_MDNIE */
#endif	/* __HX8389B_PARAM_H__ */
