#define MAX_BRIGHTNESS	255
#define MIN_BRIGHTNESS	20
#define DEFAULT_BRIGHTNESS 125
#define DEFAULT_PULSE 20
#define DIMMING_VALUE	31
#define BL_INIT_DELAY	3
#define GPIO_BL_CTRL	124
#define GPIO_BL_PWM_EN (75)
extern unsigned int system_rev;

struct brt_value brt_table_ktd[] = {
	{ 255,	11}, /* Max */
	{ 230,	12 },
	{ 210,	13 },
	{ 190,	14 },
	{ 170,	15 },
	{ 155,	16 },
	{ 145,	17 },
	{ 135,	18 },
	{ 130,	19 },
	{ 125,	20 }, /* default */
	{ 110,	21 },
	{ 95,	22 }, 
	{ 84,	23 },
	{ 78,	24 },
	{ 72,	25 },
	{ 59,	26 },     
	{ 52,	27 },
	{ 46,	28 },
	{ 39,	29 },
	{ 33,	30 },
	{ 26,	31 }, /* Min */
	{ 20,	31 }, /* Dimming */   
	{ 0,	32 }, /* Off */
};
