///#include <mach/mt6575_pll.h>
///#include <mach/mt6575_typedefs.h> 
///#include <mach/mt6575_gpio.h>
#include <kd_camera_hw.h>
#include "cust_gpio_usage.h"



#define CAMERA_IO_DRV_1800
//[573206][Soul4][TV] Add DTV NMI5625 driver by lizhao @ 2014.01.03 begin
//#define NMI_POWER_VDDIO_PIN            GPIO_NMI_POWER_VDDIO_PIN//GPIO23 //GPIO48//0xFF  //40        //gpio48  for VDDIO(2.8V)
#define NMI_POWER_VCORE_PIN            GPIO_NMI_POWER_VCORE_PIN//GPIO57//GPIO23 //GPIO70//0xFF //75  //   VCORE_1.2  
#define NMI_RESET_PIN                          GPIO_NMI_RESET_PIN//GPIO30 //GPIO78//0xFF //30  
#define NMI_ATV_ANT_PIN			GPIO_NMI_ATV_ANT_PIN//GPIO56//GPIO135
#define NMI_FM_ANT_PIN				GPIO_NMI_FM_ANT_PIN//GPIO58//GPIO136
//[573206][Soul4][TV] Add DTV NMI5625 driver by lizhao @ 2014.01.03 end

// NMI5625/NMI601B
#define MATV_I2C_CHANNEL     (0)        //I2C Channel 0
extern int cust_matv_power_on(void);
extern int cust_matv_power_off(void);
extern int cust_power_vgp3(void);

//customize matv i2s gpio and close fm i2s mode.
extern int cust_matv_gpio_on(void);
extern int cust_matv_gpio_off(void);


#if 1
#define MATV_LOGD printk
#else
#define MATV_LOGD(...)
#endif
#if 1
#define MATV_LOGE printk
#else
#define MATV_LOGE(...)
#endif


