#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
//#include <mach/mt6575_pm_ldo.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 0,
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x72, 0x48, 0x78, 0x00},
    /*Lenovo-sw chenlj2 add 2011-06-03,modify parameter below two lines*/
    //.als_level  = { 4, 40,  80,   120,   160, 250,  400, 800, 1200,  1600, 2000, 3000, 5000, 10000, 65535},
    //.als_value  = {10, 20,20,  120, 120, 280,  280,  280, 1600,  1600,  1600,  6000,  6000, 9000,  10240, 10240},
    //modify for PR353124
    //.als_level  = { 8, 40,  80,   120,   160, 250,  400, 800, 1200,  1600, 2000, 3000, 5000, 10000, 65535},
//modify for PR352581, Val feedback backlight change is not obvious, and backlight is not bright enough under indoor ambient light
    //.als_value  = {30, 120,120,  120, 120, 120,  120,  280, 1600,  1600,  1600,  6000,  6000, 9000,  10240, 10240},
    //.als_value  = {10, 2100,2100,  2100, 2100, 2100,  2100,  2100, 2100,  2100,  2100,  6000,  6000, 9000,  10240, 10240},
    /*yi.zheng.hz modify for YarisM light sensor effect v1.0*/
    .als_level  = { 3, 40,  80,   120,   160, 250,  400, 800, 1200,  1600, 2000, 3000, 5000, 10000, 65535},
    .als_value  = {10, 250,250,  250, 250, 250,  2100,  2100, 2100,  3100,  4100,  8100,  10240, 10240,  10240, 10240},
    .ps_threshold_high = 850,//314,
    .ps_threshold_low = 750,//260,
    .ps_threshold = 900,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}


