#define MAX_BRIGHTNESS	255
#define MIN_BRIGHTNESS	20
#define DEFAULT_BRIGHTNESS 125
#define DEFAULT_PULSE 20
#define DIMMING_VALUE	31
#define BL_INIT_DELAY	3
#define GPIO_BL_CTRL	9

struct brt_value brt_table_ktd[] = {
	{ 255,	3  }, /* Max */
	{ 245,	5 },
	{ 235,	7 },
	{ 225,	9 },
	{ 215,	11 },
	{ 200,	13 },
	{ 185,	15 },
	{ 170,	16 },
	{ 155,	17 },
	{ 140,	18 },
	{ 133,	19 },
	{ 125,	20 }, /* default */
	{ 110,	22 },
	{ 95,	24 },
	{ 80,	26 },
	{ 70,	27 },
	{ 60,	28 },
	{ 50,	29 },
	{ 40,	30 },
	{ 30,	31 }, /* Min */
	{ 20,	31 }, /* Dimming */
	{ 0,	32 }, /* Off */
};
