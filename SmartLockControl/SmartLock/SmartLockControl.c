#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/sched.h> 
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include "../External_GPIO_module/extGPIO.h"

//@TODO implement main logic of work

//@TODO implement passible to write data to file (LOG)

#define DEVICE_NAME                "SmartLock"
#define PERIOD_UNLOCK_TIME         5000
#define NUMBER_OF_APPRUVED_DATA    5
#define MAX_LOG_MESSAGE_SIZE       100
#define LOG_FILE_PATH              "etc/value/SmartLock/log"
#define APPROVED_NUMB_FILE_PATH    "etc/value/SmartLock/approvedNumbers"




typedef struct{
  int  major;
  bool cDevCreated;

	int BUTTON_IRQ;
	int butInterrupt_Num;

	u16 valueFromRFID;

	u32 startUnlockTime;
	struct timer_list lockOperationTimer;

	struct task_struct *kthread;

	unsigned char appruvedNumbers[NUMBER_OF_APPRUVED_DATA];
}elementsOfLogic_t;


static elementsOfLogic_t logic = {0};


static struct class*  devClass  = NULL; 
static struct device* DevDevice = NULL;


static void LockOperation(struct timer_list *lockOperationTimer);
static void UnlockOperation(void);
bool IsValueApproved(u16 readData);



static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);


void ReadApprovedNum(void);
void WriteLog(unsigned char *text, u16 size);



static const struct file_operations dev_fOps = {
    .owner   = THIS_MODULE,
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};




int MainLogic(void *data){

	while(!kthread_should_stop()){
		if(logic.valueFromRFID != 0){
			u8 readRFID_areCorrect = IsValueApproved(logic.valueFromRFID);
			if(readRFID_areCorrect){
				UnlockOperation();
				
				char txt[MAX_LOG_MESSAGE_SIZE] = {0};
				sprintf(txt, "Lock was unlock by %d at %d \n", logic.valueFromRFID, jiffies);
				WriteLog(txt, sizeof(txt));
			}
			logic.valueFromRFID = 0;
			printk("In kthread");
		}
		schedule();
	}
	return 0;
}


static irqreturn_t RFID_DataReady( int irq, void *dev_id ) {
   printk(KERN_INFO"Read value from RFID");

   //logic.valueFromRFID = value from RFID module

   return IRQ_NONE;
}


static irqreturn_t OpenLock_btn( int irq, void *dev_id ) {
   printk(KERN_INFO "Button was pressed");
   UnlockOperation();
   

   char txt[MAX_LOG_MESSAGE_SIZE] = {0};
   sprintf(txt, "Lock was unlock by button at %d \n", jiffies);
   WriteLog(txt, sizeof(txt));

   return IRQ_NONE;
}

void  LockOperation(struct timer_list *lockOperationTimer){
	SetLockState(STATE_LOCK);
	SetLedMode(LOCK_OPEN_LED,  MODE_OFF);
	SetLedMode(LOCK_CLOSE_LED, MODE_ON);

    char txt[MAX_LOG_MESSAGE_SIZE] = {0};
    sprintf(txt, "Lock was lock at %d \n", jiffies);
    WriteLog(txt, sizeof(txt));


    printk(KERN_INFO "Lock Operation");     
 }

void UnlockOperation(void){
	 SetLockState(STATE_UNLOCK);
	 SetLedMode(LOCK_OPEN_LED,  MODE_ON);
	 SetLedMode(LOCK_CLOSE_LED, MODE_OFF);

    mod_timer( &logic.lockOperationTimer, jiffies + msecs_to_jiffies(PERIOD_UNLOCK_TIME) );
    printk(KERN_INFO "Unlock Operation");
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
        Functions for work with file
=====*/
static int dev_open(struct inode *inode, struct file *file)
{
   if(logic.cDevCreated){
      return -EBUSY;
   }
   logic.cDevCreated = true;
   printk(KERN_INFO "file was open");
   return 0;
}


static int dev_release(struct inode *inode, struct file *file)
{
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
   devClass = class_create(THIS_MODULE, DEVICE_NAME);
   if (IS_ERR(devClass)){               
      unregister_chrdev(logic.major, DEVICE_NAME);
      logic.major = 0;
      printk(KERN_ALERT "Failed to register device class\n");
      goto err;         
   }
   printk(KERN_INFO "device class registered correctly\n");
 
   // Register the device driver
   DevDevice = device_create(devClass, NULL, MKDEV(logic.major, 0), NULL, DEVICE_NAME);
   if (IS_ERR(DevDevice)){           
      class_destroy(devClass);
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

static void DeinitCHhReg(void)
{ 
  if(logic.major != 0){
   device_destroy(devClass, MKDEV(logic.major, 0));     
   class_unregister(devClass);                         
   class_destroy(devClass);          
   unregister_chrdev(logic.major, DEVICE_NAME);
   printk(KERN_INFO "====module %s removed ====\n", DEVICE_NAME);
  }
};






void WriteLog(unsigned char *text, u16 size){
  printk(KERN_INFO "write buff ");
	// struct file  *logFile = file_open(LOG_FILE_PATH, 0, 0);

	// file_write(logFile, 0, text, size);
	// file_close(logFile);
}

void ReadApprovedNum(void){
   printk(KERN_INFO "read buff ");
	// struct file  *aprNumFile = file_open(APPROVED_NUMB_FILE_PATH, 0, 0);

 //  printk("operation to open file has finished with res = %d", aprNumFile);

	// int res = file_read(aprNumFile, 0, logic.appruvedNumbers, 10);
 //  printk("operation to read file has finished with res = %d ", res);

	// file_close(aprNumFile);
   logic.appruvedNumbers[0] = 1;
}



/*=====
        INIT / DEINIT functions
=====*/

static int __init SmartLockControlInit(void) {
	int returnedStatus = 0;

	printk(KERN_INFO "Smart lock start init\n");

    
    logic.BUTTON_IRQ = GetInterruptNumber();
    if(logic.BUTTON_IRQ == 0) {
       printk(KERN_INFO "Cached IRQ number is %d \n", logic.BUTTON_IRQ);
    	 goto error;   
      } 

    if(request_irq( logic.BUTTON_IRQ, OpenLock_btn, IRQF_SHARED, "LockBtn_handler", &logic.butInterrupt_Num )){
      //printk(KERN_ERR "Error with create button interrupt handler by irq = %d \n", logic.BUTTON_IRQ);
    	//goto error;
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
   
   

  // ReadApprovedNum();

   SetLedMode(GREEN_LED, MODE_ON);
   printk(KERN_INFO "Smart lock control module has inited sucsessfully\n");

return returnedStatus;

error:
	printk(KERN_ERR "Initting process has stopped by initialize error\n");
	SetLedMode(RED_LED, MODE_ON);
	return -1;

}


static void __exit SmartLockControlDeinit(void) {
   free_irq( logic.BUTTON_IRQ, &logic.butInterrupt_Num );
   del_timer( &logic.lockOperationTimer );  
   kthread_stop(logic.kthread);

   DeinitCHhReg();

   printk("Smart lock control module has deinited sucsessfully\n");
}

module_init(SmartLockControlInit);
module_exit(SmartLockControlDeinit);




MODULE_AUTHOR("MaksimHolikov, <golikov.mo@gmail.com>");
MODULE_DESCRIPTION("Driver to implement logic of smart lock");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
