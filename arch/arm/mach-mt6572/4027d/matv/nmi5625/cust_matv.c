//For mt6573_evb
///#include <mach/mt6575_pll.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/workqueue.h>
///#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_reg_base.h>
#include "cust_matv.h"
#include "cust_matv_comm.h"

/**************************************/
#define NMITV_NAME	"nmtatv"
#define NMITV_POWER_VGP3  MT6323_POWER_LDO_VGP3


/**************************************/


int cust_matv_power_on(void)
{  
	MATV_LOGE("[MATV][nmi5625] cust_matv_power_on Start at %s\n", NMITV_NAME);


	if(TRUE != hwPowerOn(MT6323_POWER_LDO_VCAM_IO, VOL_1800, NMITV_NAME))//I2C Power
		{
		printk("[MATV] Fail to enable TV I2C power\n");
		return 0;
		}
//[573206] [Soul4][TV] Add DTV NMI5625 driver by lizhao @ 2014.01.10 begin
	if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,NMITV_NAME))//for sub camera AVDD
		{
	      	printk("[MATV] Fail to enable sub camera AVDD\n");
	       return 0;
		}
//[573206] [Soul4][TV] Add DTV NMI5625 driver by lizhao @ 2014.01.10 end	
	if(TRUE != hwPowerOn(NMITV_POWER_VGP3, VOL_1800,NMITV_NAME)) //TV main power
		{
		printk("[MATV] Fail to enable TV main power\n");
		return 0;
		}
/***********************************************/	
//	TODO: add camera power here ,and set the sub camera to powerdown
//[573206] [Soul4][TV] Add DTV NMI5625 driver by lizhao @ 2014.01.10 begin
	if(mt_set_gpio_mode(GPIO_CAMERA_CMRST1_PIN,GPIO_MODE_00)){printk("[MATV] set GPIO_CAMERA_CMRST1_PIN mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_CMRST1_PIN,GPIO_DIR_OUT)){printk("[MATV] set GPIO_CAMERA_CMRST1_PIN dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN,GPIO_OUT_ZERO)){printk("[MATV] set GPIO_CAMERA_CMRST1_PIN failed!! \n");}
	
	if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_MODE_00)){printk("[MATV] set GPIO_CAMERA_CMPDN1_PIN mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){printk("[MATV] set GPIO_CAMERA_CMPDN1_PIN dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)){printk("[MATV] set GPIO_CAMERA_CMPDN1_PIN failed!! \n");}
//[573206] [Soul4][TV] Add DTV NMI5625 driver by lizhao @ 2014.01.10 end
/***********************************************/	
#if 0
	if(mt_set_gpio_mode(NMI_ATV_ANT_PIN,GPIO_MODE_00)){printk("[MATV] set NMI_ATV_ANT_PIN mode failed!! \n");}
	if(mt_set_gpio_dir(NMI_ATV_ANT_PIN,GPIO_DIR_OUT)){printk("[MATV] set NMI_ATV_ANT_PIN dir failed!! \n");}
	if(mt_set_gpio_out(NMI_ATV_ANT_PIN,GPIO_OUT_ONE)){printk("[MATV] set NMI_ATV_ANT_PIN failed!! \n");}
	
	if(mt_set_gpio_mode(NMI_FM_ANT_PIN,GPIO_MODE_00)){printk("[MATV] set NMI_FM_ANT_PIN mode failed!! \n");}
	if(mt_set_gpio_dir(NMI_FM_ANT_PIN,GPIO_DIR_OUT)){printk("[MATV] set NMI_FM_ANT_PIN dir failed!! \n");}
	if(mt_set_gpio_out(NMI_FM_ANT_PIN,GPIO_OUT_ZERO)){printk("[MATV] set NMI_FM_ANT_PIN failed!! \n");}
#endif	
	if(mt_set_gpio_mode(NMI_RESET_PIN,GPIO_MODE_00)){printk("[MATV] set NMI_RESET_PIN mode failed!! \n");}
	if(mt_set_gpio_dir(NMI_RESET_PIN,GPIO_DIR_OUT)){printk("[MATV] set NMI_RESET_PIN dir failed!! \n");}
	if(mt_set_gpio_out(NMI_RESET_PIN,GPIO_OUT_ONE)){printk("[MATV] set NMI_RESET_PIN failed!! \n");}

	if(mt_set_gpio_mode(NMI_POWER_VCORE_PIN,GPIO_MODE_00)){printk("[MATV] set NMI_POWER_VCORE_PIN mode failed!! \n");}
	if(mt_set_gpio_dir(NMI_POWER_VCORE_PIN,GPIO_DIR_OUT)){printk("[MATV] set NMI_POWER_VCORE_PIN dir failed!! \n");}
	if(mt_set_gpio_out(NMI_POWER_VCORE_PIN,GPIO_OUT_ONE)){printk("[MATV] set NMI_POWER_VCORE_PIN failed!! \n");}
//cust_matv_gpio_on();
    return 0;
}


int cust_matv_power_off(void)
{  
	MATV_LOGE("[MATV][nmi5625] cust_matv_power_off Start at %s\n", NMITV_NAME);
	
	
	if(mt_set_gpio_mode(NMI_RESET_PIN,GPIO_MODE_00)){printk("[MATV] set NMI_RESET_PIN mode failed!! \n");}
	if(mt_set_gpio_dir(NMI_RESET_PIN,GPIO_DIR_OUT)){printk("[MATV] set NMI_RESET_PIN dir failed!! \n");}
	if(mt_set_gpio_out(NMI_RESET_PIN,GPIO_OUT_ZERO)){printk("[MATV] set NMI_RESET_PIN failed!! \n");}	
	
	if(mt_set_gpio_mode(NMI_POWER_VCORE_PIN,GPIO_MODE_00)){printk("[MATV] set NMI_POWER_VCORE_PIN mode failed!! \n");}
	if(mt_set_gpio_dir(NMI_POWER_VCORE_PIN,GPIO_DIR_OUT)){printk("[MATV] set NMI_POWER_VCORE_PIN dir failed!! \n");}
	if(mt_set_gpio_out(NMI_POWER_VCORE_PIN,GPIO_OUT_ZERO)){printk("[MATV] set NMI_POWER_VCORE_PIN failed!! \n");}
#if 0
	if(mt_set_gpio_mode(NMI_ATV_ANT_PIN,GPIO_MODE_00)){printk("[MATV] set NMI_ATV_ANT_PIN mode failed!! \n");}
	if(mt_set_gpio_dir(NMI_ATV_ANT_PIN,GPIO_DIR_OUT)){printk("[MATV] set NMI_ATV_ANT_PIN dir failed!! \n");}
	if(mt_set_gpio_out(NMI_ATV_ANT_PIN,GPIO_OUT_ZERO)){printk("[MATV] set NMI_ATV_ANT_PIN failed!! \n");}
	
	if(mt_set_gpio_mode(NMI_FM_ANT_PIN,GPIO_MODE_00)){printk("[MATV] set NMI_FM_ANT_PIN mode failed!! \n");}
	if(mt_set_gpio_dir(NMI_FM_ANT_PIN,GPIO_DIR_OUT)){printk("[MATV] set NMI_FM_ANT_PIN dir failed!! \n");}
	if(mt_set_gpio_out(NMI_FM_ANT_PIN,GPIO_OUT_ONE)){printk("[MATV] set NMI_FM_ANT_PIN failed!! \n");}
#endif
#if 0	
	if(TRUE != hwPowerDown(NMITV_POWER_VGP3, NMITV_NAME)) 
		{
		printk("[MATV] Fail to disable TV main power\n");
		}
#endif
	if(TRUE != hwPowerDown(MT6323_POWER_LDO_VCAM_IO, NMITV_NAME)) 
		{
		printk("[MATV] Fail to disable TV I2C power\n");
		}
//[573206] [Soul4][TV] Add DTV NMI5625 driver by lizhao @ 2014.01.10 begin
	if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, NMITV_NAME)) 
		{
		printk("[MATV] Fail to disable Sub camera AVDD power\n");
		}	
//[573206] [Soul4][TV] Add DTV NMI5625 driver by lizhao @ 2014.01.10 end		
//cust_matv_gpio_off();	
    return 0;
}
/*modify by xiaopu.zhu I2S port fix for ATV there is unnecessary to use GPIO*/
#if 0
#define TV_I2S_LRCLK	GPIO142
#define TV_I2S_SCLK		GPIO143
#define TV_I2S_DATA		GPIO144


int cust_matv_gpio_on(void)
{
	MATV_LOGE("[MATV] [nmi5625]cust_matv_gpio_on Start at %s\n", NMITV_NAME);

    mt_set_gpio_mode(TV_I2S_DATA, GPIO_MODE_04);
    mt_set_gpio_mode(TV_I2S_SCLK, GPIO_MODE_04);
    mt_set_gpio_mode(TV_I2S_LRCLK, GPIO_MODE_04);

    return 1;
}

int cust_matv_gpio_off(void)
{
	MATV_LOGE("[MATV] [nmi5625]cust_matv_gpio_off Start at %s\n", NMITV_NAME);

    mt_set_gpio_mode(TV_I2S_DATA, GPIO_MODE_00);
    mt_set_gpio_dir(TV_I2S_DATA,GPIO_DIR_OUT);
    mt_set_gpio_out(TV_I2S_DATA,GPIO_OUT_ZERO);
	
    mt_set_gpio_mode(TV_I2S_SCLK, GPIO_MODE_00);
    mt_set_gpio_dir(TV_I2S_SCLK,GPIO_DIR_OUT);
    mt_set_gpio_out(TV_I2S_SCLK,GPIO_OUT_ZERO);
	
    mt_set_gpio_mode(TV_I2S_LRCLK, GPIO_MODE_00);
    mt_set_gpio_dir(TV_I2S_LRCLK,GPIO_DIR_OUT);
    mt_set_gpio_out(TV_I2S_LRCLK,GPIO_OUT_ZERO);

    return 1;
}
#else
int cust_matv_gpio_on(void)
{
	MATV_LOGE("[MATV] [nmi5625]cust_matv_gpio_on Start at %s\n", NMITV_NAME);
    return 1;
}

int cust_matv_gpio_off(void)
{
	MATV_LOGE("[MATV] [nmi5625]cust_matv_gpio_off Start at %s\n", NMITV_NAME);
    return 1;
}
#endif
