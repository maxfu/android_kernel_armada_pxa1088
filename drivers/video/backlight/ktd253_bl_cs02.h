#define MAX_BRIGHTNESS	255
#define MIN_BRIGHTNESS	20
#define DEFAULT_BRIGHTNESS 125
#define DEFAULT_PULSE 20
#define DIMMING_VALUE	31
#define BL_INIT_DELAY	55
#define GPIO_BL_CTRL	124

struct brt_value brt_table_ktd[] = {
	{ 255, 1 }, /* Max */
	{ 245, 3 },
	{ 235, 5 },
	{ 225, 7 },
	{ 215, 9 },
	{ 200, 11 },
	{ 185, 13 },
	{ 170, 15 },
	{ 155, 17 },
	{ 140, 18 },
	{ 133, 19 },
	{ 125, 20 },  /* default */
	{ 110, 21 },
	{ 95, 22 },
	{ 80, 23 },
	{ 70, 24 },
	{ 60, 25 },
	{ 50, 27 },
	{ 40, 29 },
	{ 30, 31 }, /* Min */
	{ 20, 31 }, /* Dimming */
	{ 0, 32 }, /* Off */
};
