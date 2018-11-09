/*
 * GL BaseCamp smart lock control driver header
 * Copyright (C) 2018 Maksim Holikov <golikov.mo@gmail.com>
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */


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
#include "../mfrc522_driver/mfrc522_api.h"



MODULE_AUTHOR("MaksimHolikov, <golikov.mo@gmail.com>");
MODULE_DESCRIPTION("Driver to implement logic of smart lock");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");



#define DEVICE_NAME                "SmartLock"
#define PERIOD_UNLOCK_TIME         5000
#define NUMBER_OF_APPRUVED_DATA    2
#define MAX_LOG_MESSAGE_SIZE       100




typedef struct{
  //character device
  int    major;
  bool   cDevCreated;
  struct class*  devClass;
  struct device* DevDevice;

  //unlock button
	int butOldInterruptCount;

 //timer for autolock
	u32 startUnlockTime;
	struct timer_list lockOperationTimer;

	struct task_struct *kthread;

	u32 appruvedNumbers[NUMBER_OF_APPRUVED_DATA];
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

extern int isCardPresent(uint32_t *uid);
extern int  GetInterruptCount(void);
extern u8   SetLedMode(u8 led, u8 mode);
extern void SetLockState(u8 state);


void  LockOperation(struct timer_list *lockOperationTimer);
void  UnlockOperation(void);
void  ReadApprovedNum(void);
void  WriteLog(bool isButton, bool isLocked);
void  OpenLock_btn(void);
bool  IsValueApproved(u32 readData);
int   ReadFromFile(void);




int res;
int returnedStatus;
bool isButton;
bool isLock;
u32 interruptCount;


uint32_t valueFromRFID = 0;


int MainTheard(void *data){

	while(!kthread_should_stop()){
     
    interruptCount = GetInterruptCount();
    if(interruptCount > logic.butOldInterruptCount){
       printk(KERN_INFO "Button pressed detected ");
       logic.butOldInterruptCount = interruptCount;
       OpenLock_btn();
    }
     
    if( (isCardPresent(&valueFromRFID)) > 0){
    	printk(KERN_INFO "read val 0x%X", valueFromRFID);
      if(IsValueApproved(valueFromRFID)){
        UnlockOperation();
        
        isButton = false;
        isLock   = false;
        WriteLog(isButton, isLock);
      }
      valueFromRFID = 0;
      printk("In kthread");
    } 

    msleep(300);
		schedule();
	}
	return 0;
}



void  OpenLock_btn(void) {
  UnlockOperation();
  
  isButton = true;
  isLock   = false;
  WriteLog(isButton, isLock);
}


void  LockOperation(struct timer_list *lockOperationTimer){
  printk(KERN_INFO ">>>  Lock Operation");
	SetLockState(STATE_LOCK);
	SetLedMode(LOCK_OPEN_LED,  MODE_OFF);
	SetLedMode(LOCK_CLOSE_LED, MODE_ON);

  isButton = false;
  isLock   = true;
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


bool IsValueApproved(u32 readData){
    bool shouldUnlock;
	  shouldUnlock= false;

    if( sizeof(logic.appruvedNumbers) > 0){
      u16 i;
      for(i = 0; i < NUMBER_OF_APPRUVED_DATA; i++){
      	if(logic.appruvedNumbers[i] == readData){
          printk(KERN_INFO "approvedNumbers[%d]= 0x%X, readData = 0x%X", i, logic.appruvedNumbers[i], readData);
      		shouldUnlock = true;
      		break;
      	}
      }
      printk(KERN_INFO "Card is not approved!");
    }else{
    	printk(KERN_INFO "Buffer of approved numbers is empty");
    }
	return shouldUnlock;
}



/*=====
        Character device functions
=====*/
static int InitChDev(void){
 res = -1;
  
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
  res = 0;
  if(logic.cDevCreated){
      res = -EBUSY;
  }else{
    logic.cDevCreated = true;
    printk(KERN_INFO "file was open");
  }
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
    printk(KERN_INFO "Lock was lock at %lu \n", jiffies);
  }else{
    if(isButton){
      printk(KERN_INFO "Lock was unlock by button at %lu \n", jiffies);
    }else{
      printk(KERN_INFO "Lock was unlock by curd at %lu \n", jiffies);
    }    
  }
}

void ReadApprovedNum(void){

  printk(KERN_INFO "Read approved keys");

  logic.appruvedNumbers[0] = 0x904eef79;
  logic.appruvedNumbers[1] = 0x2ae953a3;
}



/*=====
        INIT / DEINIT functions
=====*/

static int __init SmartLockControlInit(void) {
  returnedStatus = 0;

	printk(KERN_INFO "Smart lock start init\n");

    
  logic.butOldInterruptCount = GetInterruptCount();
  if(logic.butOldInterruptCount < 0) {
     printk(KERN_ERR "Cached IRQ count is %d it smaller then 0\n", logic.butOldInterruptCount);
   	 goto error;   
  }

  logic.kthread = kthread_run(MainTheard, NULL, "SmartLock");
  if(logic.kthread)
      printk(KERN_INFO "Thread created successfully\n");
  else{
      printk(KERN_ERR "Thread creation failed\n");
      goto error;
  }

  __init_timer(&logic.lockOperationTimer, LockOperation, 0);
  LockOperation(&logic.lockOperationTimer);
  
  res = InitChDev();
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