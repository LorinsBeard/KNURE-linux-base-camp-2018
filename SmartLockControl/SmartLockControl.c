#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/sched.h> 
#include <linux/kthread.h>
#include <linux/timer.h>
#include "External_GPIO_module/extGPIO.h"

//@TODO Add stream for parsing value of read number
//@TODO implement main logic of work


#define PERIOD_UNLOCK_TIME  5000


typedef struct{
	int BUTTON_IRQ;
	int interrupt_Num;

	u16 valueFromRFID;

	u32 startUnlockTime;
	struct timer_list lockOperationTimer;

	struct task_struct *kthread;
}elementsOfLogic_t;

static elementsOfLogic_t logic = {0};



static void LockOperation(struct timer_list *lockOperationTimer);
static void UnlockOperation(void);



static int MainLogic(void *data){

	if(logic.valueFromRFID != 0){
		//@TODO read possible value from file
		// @TODO check value if they correct
		u8 readRFID_areCorrect = 1; // should be seted by previouce function
		if(readRFID_areCorrect){
			UnlockOperation();
			//Write to file who was open the lock
		}
		logic.valueFromRFID = 0;
		printk("In kthread");
	}

	return 0;
}






static irqreturn_t OpenLock_btn( int irq, void *dev_id ) {
   printk(__FUNCTION__);
   logic.valueFromRFID = 10;
   //UnlockOperation();
   return IRQ_NONE;
}

void  LockOperation(struct timer_list *lockOperationTimer){
	// SetLockState(STATE_LOCK);
	// SetLedMode(LOCK_OPEN_LED,  MODE_OFF);
	// SetLedMode(LOCK_CLOSE_LED, MODE_ON);
    //@TODO Should create possible to write operation to log file

    printk("Lock Operation");     
 }

void UnlockOperation(void){
	// SetLockState(STATE_UNLOCK);
	// SetLedMode(LOCK_OPEN_LED,  MODE_ON);
	// SetLedMode(LOCK_CLOSE_LED, MODE_OFF);
    //@TODO Should create possible to write operation to log file
    mod_timer( &logic.lockOperationTimer, jiffies + msecs_to_jiffies(PERIOD_UNLOCK_TIME) );
    printk("Unlock Operation");
}


/*=====
        INIT / DEINIT functions
=====*/

static int __init SmartLockControlInit(void) {


//INITIALIZE BUTTON PARSER
    logic.BUTTON_IRQ = 117;
    if(request_irq( logic.BUTTON_IRQ, OpenLock_btn, IRQF_SHARED, "LockBtn_handler", &logic.interrupt_Num )){
    	//@TODO Turn on system state RED led
    	return -1;
    }

//INITIALIZE THREAD FOR MAIN LOGIC
    kthread_run(MainLogic, NULL, "SmartLock");
    // kthread = kthread_create(MainLogic, NULL, "SmartLock");
    // wake_up_process(kthread);


   __init_timer(&logic.lockOperationTimer, LockOperation, 0);
   LockOperation(&logic.lockOperationTimer);


    printk("Smart lock control module has inited sucsessfully\n");
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