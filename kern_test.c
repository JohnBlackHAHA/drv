#include <linux/clk.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include "ioctl.h"
#define ERROR_LINE	printk("%s-%d\n",__FILE__,__LINE__)
#define AXIDEV_MAJOR			153	/* assigned */
#define N_AXI_MINORS			32	/* ... up to 256 */
#define RS422_DATA 0x4004//之下的宏都代表地址
#define ETH_DATA 0x4014
#define B_1553_DATA 0x4024 
#define RS422_LENGTH 0x4000
#define ETH_LENGTH 0x4010
#define B_1553_LENGTH 0x4020
static DECLARE_BITMAP(minors, N_AXI_MINORS);
dev_t	gDevt;
struct class *gDevClass = NULL;
void __iomem *gVirtMem = NULL;
static volatile int irq[3] = {0};//gpio中断使能返回中断向量号
static volatile int irq_request = 0;
static DECLARE_WAIT_QUEUE_HEAD(fpga_waitq);
static  int datapoint = 0;
static  int read_count=0 ;
static int count=0;
static struct  tasklet_struct my_tasklet;
static   TtmpData temdata[200]={0};//数据内存缓冲区
static TtmpData tmp = {0};
/*************************
*function		:
*description	:
*Input		:
*OutPut		:
**************************/
int axi_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    //if (temdata[read_count].length)
    //{
	//printk(" %x\n",temdata[read_count].length);
    if(copy_to_user((void *)buf,&temdata[read_count],sizeof(temdata[read_count])) != 0)
    {
	ERROR_LINE;
        return -EFAULT;
    }
    temdata[read_count]=tmp;
    temdata[read_count].length=0;
    temdata[read_count].type=0;
   // }
    if (read_count !=199 )
        read_count++;
    else
        read_count = 0;
    //}
    return 0;	
}


/*************************
*function		:
*description	:
*Input		:
*OutPut		:
**************************/
int axi_write(struct file *filp,const char __user *buf, size_t size, loff_t *ppos)
{
    return 0;
}



/*************************
*function		:
*description	:
*Input		:
*OutPut		:
**************************/
 int  axi_mmap(struct file *filp, struct vm_area_struct *vm_areap)
{
    return 0;
}




 /*************************
 *function		 :
 *description	 :
 *Input 	 :
 *OutPut	 :
 **************************/
 int axi_open(struct inode * inode,struct file *filp)
{
    return 0;
}

 void ShowOpInfo(void *pOp)
 {
    TAxiRwReg *pReg = (TAxiRwReg *)pOp;
    printk("pReg->nRead    : 0x%x\n",pReg->nRead);
    printk("pReg->nRegAddr : 0x%x\n",pReg->nRegAddr);
    printk("pReg->nRegVal  : 0x%x\n",pReg->nRegVal);
 }


 /*************************
 *function		 :
 *description	 :
 *Input 	 :
 *OutPut	 :
 **************************/

static long axi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    TAxiRwReg nRegOp = {0};
    int ret = 0;
    //printk("Get IOCTL Cmd %d with args %lu\n",cmd,arg);
    switch(cmd)
    {
        case AXI_CMD_RW_REG:
        if(copy_from_user(&nRegOp,(void *)arg,sizeof(nRegOp))!= 0)
        {
           ERROR_LINE;
           return -EFAULT;
        }
    //ShowOpInfo(&nRegOp);
        if(nRegOp.nRead == 1)
        {
            nRegOp.nRegVal  = *((const volatile u32 *) (gVirtMem + nRegOp.nRegAddr));
	}else{
                 *(volatile u32  *) (gVirtMem + nRegOp.nRegAddr) = nRegOp.nRegVal;
	     }
        if(copy_to_user((void *)arg,&nRegOp,sizeof(nRegOp)) != 0)
        {
            ERROR_LINE;
	    return -EFAULT;
	}
	break;
	default:
	ERROR_LINE;
	ret = -EFAULT;
        break;
    }
    return ret;
}



#ifdef CONFIG_COMPAT
static long axi_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return axi_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define axi_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

/*************************
*function		:
*description	:
*Input		:
*OutPut		:
**************************/
 int axi_release(struct inode * inode,struct file *filp)
{
    return 0;
}

const struct file_operations axi_drv_fops=
{
    .owner = THIS_MODULE,
    .read = axi_read,
    .write = axi_write,
    .unlocked_ioctl = axi_ioctl,
    .compat_ioctl = axi_compat_ioctl,
    .open = axi_open,
    .release = axi_release,
    .mmap = axi_mmap,
};
/*************************
*function		:
*description	:
*Input		:
*OutPut		:
**************************/
int fpga_data(long unsigned int arg)
{
    
    //printk("get");
    int tmp=0;
    int register_ads[3][3] = {RS422_LENGTH,ETH_LENGTH,B_1553_LENGTH,RS422_DATA,ETH_DATA,B_1553_DATA,1,2,3};//要读寄存器地址数组，另有类型地址
    int read_count = 0,count = 0;//读3组寄存器的循环
    for (read_count=0;read_count<3;read_count++)
    {
        int length= *((const volatile u32 *) (gVirtMem + register_ads[0][read_count]));
	//printk(KERN_ERR "\n%x",length);
        if(length !=0)
        {
            temdata[datapoint].length=length;
            temdata[datapoint].type=register_ads[2][read_count];
        }
        else
            continue;
        for(count=0;count<length;count++)
        {

            temdata[datapoint].data[count]  = *((const volatile u32 *) (gVirtMem + register_ads[1][read_count]));
	    //if(temdata[datapoint].data[count]==0x3)
            //printk(" %dA%x",count,temdata[datapoint].data[count]);

        }
        if(datapoint!=199)
            datapoint++;
        else
            datapoint=0;

    }
    /*int length= *((const volatile u32 *) (gVirtMem + 0x4010));
    int i= 0;
    for(i=0;i<3;i++)
    {
	int data =*((const volatile u32 *) (gVirtMem + 0x4014));
	printk("%d-%d\n",i,data);
    }*/
    //*(volatile u32  *) (gVirtMem + 0x5000) = 1;
    return 0;

}
irqreturn_t axi_irq_handle1(int nIrq,void *pArgs)
{
    if( *(volatile u32  *) (gVirtMem + 0x1040)==1)
    {
	//printk(KERN_INFO "get irq ");
    	*(volatile u32  *) (gVirtMem + 0x1030) = 1;
    	tasklet_schedule(&my_tasklet);
    } 
	//count++;
    return IRQ_HANDLED;
}


/*************************
*function		:
*description	:
*Input		:
*OutPut		:
**************************/
int axi_drv_init(void)
{
    int status = 0;
    int minor = -1;
    int ret[3] = {0};//中断请求返回值数组
    tasklet_init(&my_tasklet,fpga_data,0);
    printk("Driver Release %s %s\n",__DATE__,__TIME__);
    status = register_chrdev(AXIDEV_MAJOR, "axi", &axi_drv_fops);//主设备号，设备名，驱动的文件方法
    if (status < 0)
        return status;
    gVirtMem = ioremap(0x40000000,0x20000);
    if (IS_ERR(gVirtMem))
    {
        ERROR_LINE;
        return -1;
    }
    //*(volatile u32  *) (gVirtMem + 0x100c) = 1;
    gDevClass = class_create(THIS_MODULE, "axi_dev");//第一个参数指代当前模块
    if (IS_ERR(gDevClass))
    {
        unregister_chrdev(AXIDEV_MAJOR, "axi");
        ERROR_LINE;
        return PTR_ERR(gDevClass);
    }
  /*if(gpio_request(61,"AXI_IRQ") < 0)
    {
        printk(KERN_INFO "Request GPIO Failed.\n");
    }else{
             if(gpio_direction_input(61) < 0)
             {
                 printk(KERN_INFO "Set GPIO Dir Failed.\n");
	     }else{
		    irq[0]=gpio_to_irq(61);*/
		    if((ret[0] = request_irq(61, axi_irq_handle1, IRQF_TRIGGER_RISING ,"axi_dev",NULL)) < 0)
		    {
			printk(KERN_INFO "Request IRQ Failed with %d\n",ret[0]);
		    }
	//}
    //}
    minor = find_first_zero_bit(minors, N_AXI_MINORS);
    if (minor < N_AXI_MINORS) {
	   struct device *dev;
	   gDevt = MKDEV(AXIDEV_MAJOR, minor);
	   dev = device_create(gDevClass, NULL, gDevt,NULL, "axi_dev");
	   status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		   status = -ENODEV;
               }
    if (status == 0) {
        set_bit(minor, minors);
	}
	return 0;
}


/*************************
*function		:
*description	:
*Input		:
*OutPut		:
**************************/
void axi_drv_exit(void)
{
    *(volatile u32  *) (gVirtMem + 0x100c) = 0;
    iounmap(gVirtMem);
    free_irq(61,NULL);
    //gpio_free(61);
    //printk (KERN_INFO "%d",count);
    device_destroy(gDevClass, gDevt);
    class_destroy(gDevClass);
    unregister_chrdev(AXIDEV_MAJOR, "axi_driver");
}
MODULE_AUTHOR("John Black");
MODULE_LICENSE("Dual BSD/GPL");
module_init(axi_drv_init);
module_exit(axi_drv_exit);
