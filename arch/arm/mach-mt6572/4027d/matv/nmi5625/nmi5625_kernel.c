#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/ctype.h>
#include <linux/proc_fs.h>  //ting.kang

#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/mm_types.h>
#include <linux/mm.h>
#include <linux/jiffies.h>
#include <linux/sched.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

//#include "cust_matv.h"
#include "nm5625_kernel.h"
#include "mach/mt_gpio.h"
#include "cust_matv.h"


MODULE_AUTHOR("nmi");
MODULE_DESCRIPTION("nmi TV 5625 driver");
MODULE_LICENSE("GPL");

//kangting 
static struct proc_dir_entry *nmi_status_proc = NULL;
#define NMI60X_STATUS "nmi60x_status"

//如果使用软件i2c，需要单独配置模拟sda/scl的gpio,参考nmi_gpio_i2c.c中的定义
#define NMI_HW_I2C
#ifndef NMI_HW_I2C
	#include "nmi_gpio_i2c.h"
#else
	//如果使用硬件i2c，NMI601可以兼容dma方式和非dma的方式，但nmi5625要必须使用MTK的DMA方式，进行i2c的读写。
	#define NMI_USE_MTK_I2C_DMA
#endif

//此处配置要将nmi设置挂载在哪一个i2c总线上
#define NMI_I2C_NUMBER MATV_I2C_CHANNEL

/**************************************************************
	
	Declareation: GPIO POWER/RST/EN_LDO configuration
	if one of them is not need , you use 0xFF instead.

**************************************************************/
#if 0
#define NMI_POWER_VDDIO_PIN            GPIO27 //GPIO48//0xFF  //40        //gpio48  for VDDIO(2.8V)
#define NMI_POWER_VCORE_PIN            GPIO92 //GPIO70//0xFF //75  //   VCORE_1.2  

#define NMI_RESET_PIN               GPIO63 //GPIO78//0xFF //30  

#define NMI_ATV_ANT_PIN				GPIO115
#define NMI_FM_ANT_PIN				GPIO114
#endif

extern int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName,bool On, char* mode_name);

/**************************************************************
	
	Global Defines & Variable

**************************************************************/

struct nmi_5625_dev {
	struct i2c_client *i2c_client_atv;

	struct mutex mu;
	struct class *tv_class;
	dev_t devn;
	struct	cdev cdv;
};

static int already_init = 0;
static struct nmi_5625_dev nd;

#ifdef NMI_USE_MTK_I2C_DMA
static u8 *gpDMABuf_va = NULL;
static u32 gpDMABuf_pa = NULL;
#endif


/**************************************************************
	
	V4L2: IOCTL functions from v4l2_int_ioctl_desc

**************************************************************/

/**************************************************************
	
	file operation:

**************************************************************/
static int s_ant_status = 0;
static ssize_t show_ant(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	   return sprintf(buf, "%u\n", s_ant_status);
}

static ssize_t store_ant(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
	ssize_t ret = -EINVAL;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;

		if (state != s_ant_status) {
			s_ant_status = state;
			if (s_ant_status > 0) {
				mt_set_gpio_out(NMI_ATV_ANT_PIN, GPIO_OUT_ONE);
			} else {
				mt_set_gpio_out(NMI_ATV_ANT_PIN, GPIO_OUT_ZERO);
			}
		}
	}
	return ret;
}

static DEVICE_ATTR (ant, 0644, show_ant, store_ant);


static int nmi5625_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	func_enter();
	if (!already_init) {
		ret = -ENODEV;
		goto _fail_;
	}

	/***************************************
	initialize 2.8V 1.2V RESET GPIO mode  for reference
 	 ****************************************/
	// kdCISModulePowerOn(DUAL_CAMERA_SUB_SENSOR,SENSOR_DRVNAME_GC0329_YUV,true,"nmiatv");
	/*if(mt_set_gpio_mode(GPIO_CAMERA_CMRST1_PIN,GPIO_CAMERA_CMRST1_PIN_M_GPIO)){printk("[CAMERA SENSOR] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_CMRST1_PIN,GPIO_DIR_OUT)){printk("[CAMERA SENSOR] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN,GPIO_OUT_ZERO)){printk("[CAMERA SENSOR] set gpio failed!! \n");}
	if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO)){printk("[CAMERA LENS] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){printk("[CAMERA LENS] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)){printk("[CAMERA LENS] set gpio failed!! \n");}
	*/
	//PWR Enable
	// mt_set_gpio_mode(NMI_POWER_VDDIO_PIN,GPIO_MODE_00);
	// mt_set_gpio_dir(NMI_POWER_VDDIO_PIN, GPIO_DIR_OUT);
	// mt_set_gpio_pull_enable(NMI_POWER_VDDIO_PIN,true);
	// mt_set_gpio_out(NMI_POWER_VDDIO_PIN, 0);
	
	// RF
	#if 0
	mt_set_gpio_mode(NMI_ATV_ANT_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(NMI_ATV_ANT_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(NMI_ATV_ANT_PIN,true);
	mt_set_gpio_out(NMI_ATV_ANT_PIN, GPIO_OUT_ONE);

	mt_set_gpio_mode(NMI_FM_ANT_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(NMI_FM_ANT_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(NMI_FM_ANT_PIN,true);
	mt_set_gpio_out(NMI_FM_ANT_PIN, GPIO_OUT_ZERO);
       #endif
	mt_set_gpio_mode(NMI_POWER_VCORE_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(NMI_POWER_VCORE_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(NMI_POWER_VCORE_PIN,true);
	mt_set_gpio_out(NMI_POWER_VCORE_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(NMI_RESET_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(NMI_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(NMI_RESET_PIN,true);
	mt_set_gpio_out(NMI_RESET_PIN, GPIO_OUT_ZERO);

	#ifndef NMI_HW_I2C
		nmi_i2c_init();
	#endif

	#ifdef NMI_USE_MTK_I2C_DMA    
		gpDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &gpDMABuf_pa, GFP_KERNEL);
		if(!gpDMABuf_va){
			printk("[nmi][Error] Allocate DMA I2C Buffer failed!\n");
		}
	#endif

	file->private_data = (void *)&nd;
	//
_fail_:

	func_exit();
	return ret;
}

static int nmi5625_release(struct inode * inode, struct file * file)
{
	int ret = 0;
	struct nmi_5625_dev *d = file->private_data;
	
	func_enter();
	dPrint(N_INFO, "LHJ nmi: nmi5625_release %d \n",already_init);
	 
	//kdCISModulePowerOn(DUAL_CAMERA_SUB_SENSOR,SENSOR_DRVNAME_DB8V63L_YUV,false,"atv");
/*
	if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,"mode_name")) {
		// PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		//return -EIO;
		return ret;
	}
*/	
	//Disable I2D Data pin
	#ifdef NMI_USE_MTK_I2C_DMA    
		if(gpDMABuf_va){
			dma_free_coherent(NULL, 4096, gpDMABuf_va, gpDMABuf_pa);
			gpDMABuf_va = NULL;
			gpDMABuf_pa = NULL;
		}
	#endif

	#ifndef NMI_HW_I2C
		nmi_i2c_deinit();
	#endif
	
	func_exit();	
	return ret;
}

#ifdef NMI_USE_MTK_I2C_DMA

ssize_t nmi5625_dma_read_m_byte( u8 *returnData_va, u32 returnData_pa,U16 len)
{   
    int     ret=0;

    if(len == 0) {
        printk("[Error]nmi Read Len should not be zero!! \n");
        return 0;
    }
    if (len > 0){
        ret = i2c_master_recv(nd.i2c_client_atv, returnData_pa, len);
        printk("[nmi]I2C Read Size = %d\n",ret);
        if (ret < 0) {
            printk("[Error]nmi reads data error!! \n");
            return 0;
        }
    }
       
    return ret;

}

ssize_t nmi5625_dma_write_m_byte( u8 *writeData_va, u32 writeData_pa,U16 len)
{

    int    ret=0;

    if(len == 0) {
        printk("[Error]nmi Write Len should not be zero!! \n");
        return 0;
    }
    
    if (len > 0){
        ret = i2c_master_send(nd.i2c_client_atv, writeData_pa, len);
        //printk("[nmi]I2C Send Size = %d\n",ret);
        if (ret < 0) {
            printk("[Error]nmi reads data error!! \n");
            return 0;
        }
    }

    return ret;
}

#else

static U8 i2cBuf[32];
//static u8 i2cBuf[256];

#endif

bool g_bIsAtvStart  =	false;

//kangting
static int nmi_status_read_proc (char *buf,char **start, off_t off, int count , int *eof, void *data)
{
	int len = 0;
	char *p = buf;
	p += sprintf(p,"NMI60X_STATUS = %d\n",g_bIsAtvStart);
	*start = buf + off;
	len = p - buf;
	if(len > off)
		len -= off;
	else
		len = 0;
	return len < count ? len : count;
}
static int nmi_status_write_proc (struct file *file, const char *buffer,unsigned long count, void *data)
{
	return count;
}
//ting.kang
static int nmi5625_ioctl(struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	struct nmi_5625_dev *d = file->private_data;
	long ret = 0;
	

	
	#ifdef NMI_USE_MTK_I2C_DMA
		U8 *pReadData = 0;
		U8 *pWriteData = 0;
		U8 *pa_addr = 0;
		int *user_data_addr;
	#else
		U8 *kbuf = &i2cBuf[0];
	#endif
	
    U16 len;
	
	switch ((cmd&0xffff0000)) {
        case NM5625_PWR_2P8_CTL:
		printk("NM5625_PWR_2P8_CTL, power11 %s\n",(arg==1)?"on":"off");

		if (arg == 1) {	/* on */
			// mt_set_gpio_out(u32 pin,u32 output)(NMI_POWER_VDDIO_PIN, 1);
			if (0 != cust_matv_power_on()) {
				goto _fail_;
			}
			mt_set_gpio_out(NMI_FM_ANT_PIN, GPIO_OUT_ONE);
			mt_set_gpio_out(NMI_ATV_ANT_PIN, GPIO_OUT_ZERO);
		} else{
			// mt_set_gpio_out(NMI_POWER_VDDIO_PIN, 0);
			if (0 != cust_matv_power_off()) {
				goto _fail_;
			}
			mt_set_gpio_out(NMI_FM_ANT_PIN, GPIO_OUT_ZERO);
			mt_set_gpio_out(NMI_ATV_ANT_PIN, GPIO_OUT_ONE);
		}
		break;
		
	case NM5625_PWR_1P2_CTL:
		printk("NM5625_PWR_1P2_CTL, power %s\n",(arg==1)?"on":"off");
		
		if (arg == 1) {	/* on */
#if 0
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, "nmitv")) {
				goto _fail_;
			}
			mt_set_gpio_mode(GPIO_CAMERA_CMRST1_PIN,GPIO_CAMERA_CMRST1_PIN_M_GPIO);
			mt_set_gpio_dir(GPIO_CAMERA_CMRST1_PIN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN,GPIO_OUT_ZERO);
			mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO);
			mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ONE);
			///
			if(TRUE != hwPowerOn(MT6323_POWER_LDO_VCAM_IO, VOL_1800, "nmitv")) {
				goto _fail_;
			}
			///
#endif
			mt_set_gpio_out(NMI_POWER_VCORE_PIN, GPIO_OUT_ONE);
			g_bIsAtvStart  = true;   			
		}else {
#if 0
			mt_set_gpio_mode(GPIO_CAMERA_CMRST1_PIN,GPIO_MODE_00);
			mt_set_gpio_dir(GPIO_CAMERA_CMRST1_PIN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN,GPIO_OUT_ZERO);
			mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_MODE_00);
			mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO);
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, "nmitv")) {
				goto _fail_;
			}
			if(TRUE != hwPowerDown(MT6323_POWER_LDO_VCAM_IO, "nmitv")) {
				goto _fail_;
			}
			///
#endif
			mt_set_gpio_out(NMI_POWER_VCORE_PIN, GPIO_OUT_ZERO);
			g_bIsAtvStart = false;
			///
			mt_set_gpio_out(NMI_FM_ANT_PIN, GPIO_OUT_ZERO);
			mt_set_gpio_out(NMI_ATV_ANT_PIN, GPIO_OUT_ONE);
		}	
		break;	
	case NM5625_ATV_RESET_CTL:
		printk("NM5625_ATV_RESET_CTL, power %s\n",(arg==1)?"on":"off");
       	if (arg == 1) {
			mt_set_gpio_out(NMI_RESET_PIN, GPIO_OUT_ONE);			
		} 
		else {
			mt_set_gpio_out(NMI_RESET_PIN, GPIO_OUT_ZERO);			
		}
		break;
				
	case NM5625_ATV_I2C_READ:			

			len = cmd&0xffff;	/* Note: I used the lower 16 bits for size */	
			
			dPrint(N_INFO,"NM5625_ATV_I2C_READ , len is (%d)\n" , len );
			mutex_lock(&d->mu);
			
			#ifdef NMI_HW_I2C
				#ifdef NMI_USE_MTK_I2C_DMA
				
					user_data_addr = (int *)arg;
					
					pReadData = gpDMABuf_va;
					pa_addr   = gpDMABuf_pa;
					
					if(!pReadData){
						printk("[Error] dma_alloc_coherent failed!\n");
						mutex_unlock(&d->mu);
						goto _fail_;
					}
					ret = nmi5625_dma_read_m_byte(pReadData, pa_addr, len);    
					if (ret < 0) {
						//dPrint(N_ERR, "nmi: failed i2c read...(%d)\n", ret);
						mutex_unlock(&d->mu);
						goto _fail_;
					}
										
					if (copy_to_user(user_data_addr, pReadData, len) ) {
						dPrint(N_ERR, "nmi: failed copy to user...\n");
						ret = -EFAULT;
						mutex_unlock(&d->mu);
						goto _fail_;
					}
				#else
					if ( len > 8 )
					{
						dPrint(N_ERR, "nmi: failed receive 8 more data from i2s bus , please use dma way to instead.\n");
						ret = -EFAULT;
						mutex_unlock(&d->mu);
						goto _fail_;
					}
					ret = i2c_master_recv(d->i2c_client_atv, kbuf, len);
					
					if (ret < 0) {
						dPrint(N_ERR, "nmi: failed i2c read...(%d)\n", ret);
						mutex_unlock(&d->mu);
						goto _fail_;
					}

					if (copy_to_user(arg, i2cBuf, len) ) {
						dPrint(N_ERR, "nmi: failed copy to user...\n");
						ret = -EFAULT;
						mutex_unlock(&d->mu);
						goto _fail_;
					}
				#endif //end for #ifdef NMI_USE_MTK_I2C_DMA
			#else
				ret = nmi_i2c_read(0x60,kbuf,len);
				
				//dPrint(N_TRACE,"kernel:nmi_i2c_read buf is (%x), length is (%d)\n",kbuf,len);

				if (copy_to_user(arg, i2cBuf, len) ) {
					dPrint(N_ERR, "nmi: failed copy to user...\n");
					ret = -EFAULT;
					mutex_unlock(&d->mu);
					goto _fail_;
				}
			#endif
			
			mutex_unlock(&d->mu);

            break;	
			
	case NM5625_ATV_I2C_WRITE:			

			len = cmd&0xffff;	/* Note: I used the lower 16 bits for size */	
			
			dPrint(N_INFO,"NM5625_ATV_I2C_WRITE , len is (%d)\n" , len );
			mutex_lock(&d->mu);
			
			#ifdef NMI_HW_I2C
				#ifdef NMI_USE_MTK_I2C_DMA
				
					user_data_addr = (int *)arg;
					
					pWriteData = gpDMABuf_va;
					pa_addr    = gpDMABuf_pa;
					if(!pWriteData){
						printk("[Error] dma_alloc_coherent failed!\n");
						mutex_unlock(&d->mu);
						goto _fail_;
					}
					ret = copy_from_user(pWriteData, user_data_addr, len);
					if ( ret < 0 ) {
						dPrint(N_ERR, "nmi: failed copy from user...\n");
						ret = -EFAULT;
						mutex_unlock(&d->mu);
						goto _fail_;
					}
					ret = nmi5625_dma_write_m_byte(pWriteData, pa_addr, len); 

					if (ret < 0) {
						//dPrint(N_ERR, "nmi: failed i2c read...(%d)\n", ret);
						mutex_unlock(&d->mu);
						goto _fail_;
					}
					
				#else
					if ( len > 8 )
					{
						dPrint(N_ERR, "nmi: failed to send 8 more data to i2s bus , please use dma way to instead.\n");
						ret = -EFAULT;
						mutex_unlock(&d->mu);
						goto _fail_;
					}
						
					if (copy_from_user(kbuf, arg, len)) {					
						dPrint(N_ERR, "nmi: failed copy from user...\n");
						ret = -EFAULT;
						goto _fail_;
					}
					
					ret = i2c_master_send(d->i2c_client_atv, kbuf, len);
						
					if (ret < 0) {
						dPrint(N_ERR, "nmi: failed i2c write...(%d)\n", ret);
						mutex_unlock(&d->mu);
						goto _fail_;
					}
				
				#endif //end for #ifdef NMI_USE_MTK_I2C_DMA
			#else
				if (copy_from_user(kbuf, arg, len)) {					
					dPrint(N_ERR, "nmi: failed copy from user...\n");
					ret = -EFAULT;
					goto _fail_;
				}
					
				ret = nmi_i2c_write(0x60,kbuf,len);
				dPrint(N_TRACE,"kernel:nmi_i2c_write buf is (%x), length is (%d)\n",kbuf,len);
			#endif
			
			mutex_unlock(&d->mu);
			
            break;
        
        default:
            break;
    }
_fail_:
	//func_exit();
	//dPrint(N_TRACE, "nmi_ioctl return value...(%d)\n", ret);
	return ret; 
}

static const struct file_operations nmi5625_fops = {
	.owner		= THIS_MODULE,
 	.unlocked_ioctl = nmi5625_ioctl,
	.open		= nmi5625_open,
	.release	= nmi5625_release,
};

/**************************************************************
	
	i2c:

**************************************************************/

static int nmi5625_remove(struct i2c_client *client)
{
	int ret = 0;

	func_enter();

	nd.i2c_client_atv = NULL;

	func_exit();
	
	return ret;
}

static int nmi5625_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, "nmiatv");                                                         
    return 0;                                                                                       
}        

static int nmi5625_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	
	struct device *dev;
	func_enter();

	if (!already_init) {
		memset(&nd, 0, sizeof(struct nmi_5625_dev));

		/**
			initialize mutex
		**/
		mutex_init(&nd.mu);

		/**
			register our driver
		**/
		if ((ret = alloc_chrdev_region (&nd.devn, 0, 1, "nmi")) < 0) {
			dPrint(N_ERR, "nmi: failed unable to get major...%d\n", ret);
			goto _fail_;
		}
		dPrint(N_INFO, "nmi:dynamic major(%d),minor(%d)\n", MAJOR(nd.devn), MINOR(nd.devn));

		cdev_init(&nd.cdv, &nmi5625_fops);
		nd.cdv.owner = THIS_MODULE;
		ret = cdev_add(&nd.cdv, nd.devn, 1);
		if (ret) {
			dPrint(N_ERR, "nmi: failed to add device...%d\n", ret);
			goto _fail_;
		}

		nd.tv_class = class_create(THIS_MODULE, "atv");
		if (IS_ERR(nd.tv_class)) {
			dPrint(N_ERR, "nmi: failed to create the atv class\n");
		}

     
		dev = device_create(nd.tv_class, NULL, nd.devn, NULL, "nmi");
		if (IS_ERR(dev)) {
			dPrint(N_ERR, "nmi: failed to create device\n");
		}
		device_create_file(dev, &dev_attr_ant);
		/*User interface end */
static const struct file_operations nmi_proc_fops = {
		.write = nmi_status_write_proc,
		.read = nmi_status_read_proc,
};
//kangting
	nmi_status_proc = proc_create(NMI60X_STATUS, 0666, NULL, &nmi_proc_fops);
    if (nmi_status_proc) {
        //nmi_status_proc->read_proc = nmi_status_read_proc;
        //nmi_status_proc->write_proc = nmi_status_write_proc;
    }
    else {
        dPrint(N_ERR,"add create_proc_entry %s fail \n",NMI60X_STATUS);
    }
//ting.kang
		already_init = 1;
	}

		nd.i2c_client_atv = client;
		nd.i2c_client_atv->timing = 400;
		
		#ifdef NMI_USE_MTK_I2C_DMA
		nd.i2c_client_atv->addr = nd.i2c_client_atv->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		#endif


_fail_:

	func_exit();
	return ret;
}

static const struct i2c_device_id nmi5625_id[] = {
	{"nmiatv", 0},
	{},
};

//static unsigned short force[] = {2, 0xc0, I2C_CLIENT_END, I2C_CLIENT_END};   
//static const unsigned short * const forces[] = { force, NULL };              
//static struct i2c_client_address_data addr_data = { .forces = forces,};

static struct i2c_driver nmi5625_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "nmiatv",
		  },
	.probe  = nmi5625_probe,
	.detect = nmi5625_detect,
	.remove = nmi5625_remove,
	.id_table = nmi5625_id,
//	.address_data =&addr_data,
};

static struct i2c_board_info __initdata nmi5625_i2c_dev={ I2C_BOARD_INFO("nmiatv", 0x60)};

/**************************************************************
	
	Module:

**************************************************************/
static __init int nmi5625_init(void)
{
	int ret = 0;
	
	func_enter();

	// 如果是硬件i2c，部分还需要修改此i2c的port number
	i2c_register_board_info(NMI_I2C_NUMBER, &nmi5625_i2c_dev, 1);

	ret = i2c_add_driver(&nmi5625_i2c_driver);
	if (ret < 0) {
		dPrint(N_ERR, "nmi: failed register i2c driver...(%d)\n", ret);
	}

	func_exit();

	return ret;
}

static __exit void nmi5625_clean(void)
{
	func_enter();

	i2c_del_driver(&nmi5625_i2c_driver);

	if (already_init) {
		device_destroy(nd.tv_class, nd.devn);
		cdev_del(&nd.cdv);
		unregister_chrdev_region(nd.devn, 1);
		already_init = 0;
	}
	
	func_exit();
}

module_init(nmi5625_init);
module_exit(nmi5625_clean);


