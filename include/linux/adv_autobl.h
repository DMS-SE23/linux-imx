
#define AUTO_BL_PATH      "/etc/light_autobl.conf"
#define CONTROL_BL_PATH   "/etc/light_controlbl.conf"
#define THRES_RANGE_PATH  "/etc/light_range.conf"
#define THRES_LEVELS_PATH "/etc/light_levels.conf"
#define LUX200_CAL_PATH   "/etc/light_lux200.conf"
#define BACKLIGHT_PATH    "/sys/class/backlight/backlight/brightness"
#define LEVELS_ARRAY_SIZE   100
#define DEFAULT_LEVELS_SIZE  10


extern void adv_get_file_value(int *data,char *path);
extern void adv_set_brightness(int levels[LEVELS_ARRAY_SIZE][2],int lux,char *path,int *levels_size);
extern int adv_parser_levels(int levels[LEVELS_ARRAY_SIZE][2],char *str,int *levels_size);
extern int adv_get_levels(int levels[LEVELS_ARRAY_SIZE][2],char *path,int *levels_size);

