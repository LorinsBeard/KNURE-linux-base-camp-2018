#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h> 
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include "../External_GPIO_module/extGPIO.h"


MODULE_AUTHOR("MaksimHolikov, <golikov.mo@gmail.com>");
MODULE_DESCRIPTION("Driver to implement logic of smart lock");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");



#define DEVICE_NAME                "SmartLock"
#define PERIOD_UNLOCK_TIME         5000
#define NUMBER_OF_APPRUVED_DATA    5
#define MAX_LOG_MESSAGE_SIZE       100




typedef struct{
  //character device
  int    major;
  bool   cDevCreated;
  struct class*  devClass;
  struct device* DevDevice;

  //unlock button
	int butOldInterruptCount;

	u16 valueFromRFID;

 //timer for autolock
	u32 startUnlockTime;
	struct timer_list lockOperationTimer;

	struct task_struct *kthread;

	unsigned char appruvedNumbers[NUMBER_OF_APPRUVED_DATA];
}elementsOfLogic_t;

static elementsOfLogic_t logic = {0};


//===== Function for characte device ===
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

static const struct file_operations dev_fOps = {
    .owner   = THIS_MODULE,
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};
//===========


void  LockOperation(struct timer_list *lockOperationTimer);
void  UnlockOperation(void);
void  ReadApprovedNum(void);
void  WriteLog(bool isButton, bool isLocked);
void  OpenLock_btn(void);
bool  IsValueApproved(u16 readData);
int   ReadFromFile(void);





int MainLogic(void *data){

	while(!kthread_should_stop()){

    u32 interruptCount = GetInterruptCount();
    if(interruptCount > logic.butOldInterruptCount){
       printk(KERN_INFO "Button pressed detected ");
       logic.butOldInterruptCount = interruptCount;
       OpenLock_btn();
    }

    u8 cardkeyExist = 0;//isCardPresent(&logic.valueFromRFID)
    if(cardkeyExist){
      if(logic.valueFromRFID != 0){
        u8 readRFID_areCorrect = IsValueApproved(logic.valueFromRFID);
        if(readRFID_areCorrect){
          UnlockOperation();
          
          bool isButton = false;
          bool isLock   = false;
          WriteLog(isButton, isLock);
        }
        logic.valueFromRFID = 0;
        printk("In kthread");
      }
    }

    msleep(100);
		schedule();
	}
	return 0;
}



void  OpenLock_btn(void) {
  UnlockOperation();
  
  bool isButton = true;
  bool isLock   = false;
  WriteLog(isButton, isLock);

  return IRQ_NONE;
}

void  LockOperation(struct timer_list *lockOperationTimer){
  printk(KERN_INFO ">>>  Lock Operation");
	SetLockState(STATE_LOCK);
	SetLedMode(LOCK_OPEN_LED,  MODE_OFF);
	SetLedMode(LOCK_CLOSE_LED, MODE_ON);

  bool isButton = false;
  bool isLock   = true;
  WriteLog(isButton, isLock);

  printk(KERN_INFO "-----------------");       
 }

void UnlockOperation(void){
  printk(KERN_INFO "<<<  Unlock Operation");
	SetLockState(STATE_UNLOCK);
	SetLedMode(LOCK_OPEN_LED,  MODE_ON);
	SetLedMode(LOCK_CLOSE_LED, MODE_OFF);
 
  mod_timer( &logic.lockOperationTimer, jiffies + msecs_to_jiffies(PERIOD_UNLOCK_TIME) );   
}


bool IsValueApproved(u16 readData){
	bool shouldUnlock = false;

    u16 buferSize = sizeof(logic.appruvedNumbers);
    if( buferSize > 0){
      u16 i;
      for(i = 0; i < buferSize; i++){
      	if(logic.appruvedNumbers[i] == readData){
      		shouldUnlock = true;
      		break;
      	}
      }
    }else{
    	printk(KERN_INFO "Buffer of approved numbers is empty");
    }
	return shouldUnlock;
}



/*=====
        Character device functions
=====*/
static int InitChDev(void){
  int res = -1;
  
 logic.major = register_chrdev(0, DEVICE_NAME, &dev_fOps);
 if(logic.major < 0 ){
    logic.major = 0;
    printk(KERN_ERR "===regisrt char device error===");
    goto err;
 }
 printk(KERN_INFO "===register char device sucsessfully===  major = %d \n",
        logic.major);
 


   // Register the device class
   logic.devClass = class_create(THIS_MODULE, DEVICE_NAME);
   if (IS_ERR(logic.devClass)){               
      unregister_chrdev(logic.major, DEVICE_NAME);
      logic.major = 0;
      printk(KERN_ALERT "Failed to register device class\n");
      goto err;         
   }
   printk(KERN_INFO "device class registered correctly\n");
 
   // Register the device driver
   logic.DevDevice = device_create(logic.devClass, NULL, MKDEV(logic.major, 0), NULL, DEVICE_NAME);
   if (IS_ERR(logic.DevDevice)){           
      class_destroy(logic.devClass);
      unregister_chrdev(logic.major, DEVICE_NAME);
      logic.major = 0;
      printk(KERN_ALERT "Failed to create the device\n");
       goto err;
   }
   printk(KERN_INFO "device class created correctly\n");

 res = 0;

   err:
    return res;
}

static void DeinitCHhReg(void){ 
  if(logic.major != 0){
   device_destroy(logic.devClass, MKDEV(logic.major, 0));     
   class_unregister(logic.devClass);                         
   class_destroy(logic.devClass);          
   unregister_chrdev(logic.major, DEVICE_NAME);
   printk(KERN_INFO "====module %s removed ====\n", DEVICE_NAME);
  }
};


static int dev_open(struct inode *inode, struct file *file){
   if(logic.cDevCreated){
      return -EBUSY;
   }
   logic.cDevCreated = true;
   printk(KERN_INFO "file was open");
   return 0;
}

static int dev_release(struct inode *inode, struct file *file){
   logic.cDevCreated = false;
   printk(KERN_INFO "file was close");
   return 0;
}

static ssize_t dev_read(struct file *f, char *txt, size_t size, loff_t * off){
  printk(KERN_ERR "===read ===");
  return 0;
}

static ssize_t dev_write(struct file *f, const char *txt, size_t size, loff_t * off){
  printk(KERN_ERR "===write===");
  return 0;
}
//========================




void WriteLog(bool isButton, bool isLocked){


  if(isLocked){
    printk(KERN_INFO "Lock was lock at %d \n", jiffies);
  }else{
    if(isButton){
      printk(KERN_INFO "Lock was unlock by button at %d \n", jiffies);
    }else{
      printk(KERN_INFO "Lock was unlock by curd at %d \n", jiffies);
    }    
  }
}

void ReadApprovedNum(void){

  printk(KERN_INFO "Read approved keys");

  logic.appruvedNumbers[0] = 1;
  logic.appruvedNumbers[1] = 1578224569825478;
  logic.appruvedNumbers[2] = 7788994456611231;
  logic.appruvedNumbers[3] = 1789641200786054;
  logic.appruvedNumbers[4] = 1870000658422488;
}



/*=====
        INIT / DEINIT functions
=====*/

static int __init SmartLockControlInit(void) {
	int returnedStatus = 0;

	  printk(KERN_INFO "Smart lock start init\n");

    
    logic.butOldInterruptCount = GetInterruptCount();
    if(logic.butOldInterruptCount < 0) {
       printk(KERN_ERR "Cached IRQ count is %d it smaller then 0\n", logic.butOldInterruptCount);
    	 goto error;   
    }

    logic.kthread = kthread_run(MainLogic, NULL, "SmartLock");
    if(logic.kthread)
        printk(KERN_INFO "Thread created successfully\n");
    else{
        printk(KERN_ERR "Thread creation failed\n");
        goto error;
    }


   __init_timer(&logic.lockOperationTimer, LockOperation, 0);
   LockOperation(&logic.lockOperationTimer);   


  int res = InitChDev();
   if(res < 0){
     goto error;
   }
  
   ReadApprovedNum();

   SetLedMode(GREEN_LED, MODE_ON);
   printk(KERN_INFO "Smart lock control module has inited sucsessfully\n");

  return returnedStatus;


  error:
  	printk(KERN_ERR "Initting process has stopped by initialize error\n");
  	SetLedMode(RED_LED, MODE_ON);
  	return -1;
}


static void __exit SmartLockControlDeinit(void) {
   del_timer( &logic.lockOperationTimer );  
   kthread_stop(logic.kthread);

   DeinitCHhReg();

   printk("Smart lock control module has deinited sucsessfully\n");
}

module_init(SmartLockControlInit);
module_exit(SmartLockControlDeinit);