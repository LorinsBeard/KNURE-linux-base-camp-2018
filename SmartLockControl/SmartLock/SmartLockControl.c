#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/sched.h> 
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include "../External_GPIO_module/extGPIO.h"

//@TODO implement main logic of work
//@TODO implement read approved numbers from file at initialize function to mass
//@TODO implement passible to write data to file (LOG)

#define PERIOD_UNLOCK_TIME  5000
#define NUMBER_OF_APPRUVED_DATA 1


typedef struct{
	int BUTTON_IRQ;
	int interrupt_Num;

	u16 valueFromRFID;

	u32 startUnlockTime;
	struct timer_list lockOperationTimer;

	struct task_struct *kthread;

	u16 appruvedNumbers[NUMBER_OF_APPRUVED_DATA];
}elementsOfLogic_t;


static elementsOfLogic_t logic = {0};



static void LockOperation(struct timer_list *lockOperationTimer);
static void UnlockOperation(void);
bool IsValueApproved(u16 readData);


int MainLogic(void *data){

	while(!kthread_should_stop()){
		if(logic.valueFromRFID != 0){
			u16 readValue = 1;//@TODO read possible value from file
			u8 readRFID_areCorrect = IsValueApproved(readValue);
			if(readRFID_areCorrect){
				UnlockOperation();
				//Write to file who was open the lock
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

   u16 readValue     = 1; //@TODO read value from RFID
   u8 accessApproved = IsValueApproved(readValue); 
   if(accessApproved){
   	 printk(KERN_INFO "Access by %d approwed", readValue);
   }else{
   	 printk(KERN_INFO "Access by %d not approwed", readValue);
   }

   return IRQ_NONE;
}


static irqreturn_t OpenLock_btn( int irq, void *dev_id ) {
   printk(KERN_INFO "Button was pressed");
   UnlockOperation();
   return IRQ_NONE;
}

void  LockOperation(struct timer_list *lockOperationTimer){
	 SetLockState(STATE_LOCK);
	// SetLedMode(LOCK_OPEN_LED,  MODE_OFF);
	// SetLedMode(LOCK_CLOSE_LED, MODE_ON);
    //@TODO Should create possible to write operation to log file

    printk(KERN_INFO "Lock Operation");     
 }

void UnlockOperation(void){
	// SetLockState(STATE_UNLOCK);
	// SetLedMode(LOCK_OPEN_LED,  MODE_ON);
	// SetLedMode(LOCK_CLOSE_LED, MODE_OFF);
    //@TODO Should create possible to write operation to log file
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
        INIT / DEINIT functions
=====*/

static int __init SmartLockControlInit(void) {


    logic.BUTTON_IRQ = 117;
    if(request_irq( logic.BUTTON_IRQ, OpenLock_btn, IRQF_SHARED, "LockBtn_handler", &logic.interrupt_Num )){
    	//@TODO Turn on system state RED led
    	return -1;
    }

    logic.kthread = kthread_run(MainLogic, NULL, "SmartLock");
    if(logic.kthread)
        printk(KERN_INFO "Thread created successfully\n");
    else
        printk(KERN_INFO "Thread creation failed\n");


   __init_timer(&logic.lockOperationTimer, LockOperation, 0);
   LockOperation(&logic.lockOperationTimer);


   logic.appruvedNumbers[0]= 1;
   

    printk(KERN_INFO "Smart lock control module has inited sucsessfully\n");
return 0;
}


static void __exit SmartLockControlDeinit(void) {
   free_irq( logic.BUTTON_IRQ, &logic.interrupt_Num );
   del_timer( &logic.lockOperationTimer );  
   kthread_stop(logic.kthread);

   printk("Smart lock control module has deinited sucsessfully\n");
}

module_init(SmartLockControlInit);
module_exit(SmartLockControlDeinit);




MODULE_AUTHOR("MaksimHolikov, <golikov.mo@gmail.com>");
MODULE_DESCRIPTION("Driver to implement logic of smart lock");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");