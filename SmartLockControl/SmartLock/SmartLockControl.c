#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/sched.h> 
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include "../External_GPIO_module/extGPIO.h"

//@TODO implement main logic of work
//@TODO create character driver

//@TODO implement passible to write data to file (LOG)

#define PERIOD_UNLOCK_TIME         5000
#define NUMBER_OF_APPRUVED_DATA    5
#define MAX_LOG_MESSAGE_SIZE       100
#define LOG_FILE_PATH              "etc/value/SmartLock/log"
#define APPROVED_NUMB_FILE_PATH    "etc/value/SmartLock/approvedNumbers"



typedef struct{
	int BUTTON_IRQ;
	int ButInterrupt_Num;

	u16 valueFromRFID;

	u32 startUnlockTime;
	struct timer_list lockOperationTimer;

	struct task_struct *kthread;

	unsigned char appruvedNumbers[NUMBER_OF_APPRUVED_DATA];
}elementsOfLogic_t;


static elementsOfLogic_t logic = {0};



static void LockOperation(struct timer_list *lockOperationTimer);
static void UnlockOperation(void);
bool IsValueApproved(u16 readData);


//Work with file
struct file *file_open(const char *path, int flags, int rights);
void file_close(struct file *file);
int file_read(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size);
int file_write(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size);


void ReadApprovedNum(void);
void WriteLog(unsigned char *text, u16 size);




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
struct file *file_open(const char *path, int flags, int rights) {
    struct file *filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if (IS_ERR(filp)) {
        err = PTR_ERR(filp);
        return NULL;
    }
    return filp;
}

void file_close(struct file *file) {
    filp_close(file, NULL);
}

int file_read(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size) {
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_read(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}   

int file_write(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size) {
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_write(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

void WriteLog(unsigned char *text, u16 size){
	struct file  *logFile = file_open(LOG_FILE_PATH, 0, 0);

	file_write(logFile, 0, text, size);
	file_close(logFile);
}

void ReadApprovedNum(void){
	struct file  *aprNumFile = file_open(APPROVED_NUMB_FILE_PATH, 0, 0);

	file_read(aprNumFile, 0, logic.appruvedNumbers, 10);
	file_close(aprNumFile);
}



/*=====
        INIT / DEINIT functions
=====*/

static int __init SmartLockControlInit(void) {
	int returnedStatus = 0;

	printk(KERN_INFO "Smart lock start init\n");

    
    logic.BUTTON_IRQ = GetInterruptNumber();
    if(logic.BUTTON_IRQ == 0) 
    	goto error;    

    if(request_irq( logic.BUTTON_IRQ, OpenLock_btn, IRQF_SHARED, "LockBtn_handler", &logic.ButInterrupt_Num ))
    	goto error;
    

    logic.kthread = kthread_run(MainLogic, NULL, "SmartLock");
    if(logic.kthread)
        printk(KERN_INFO "Thread created successfully\n");
    else{
        printk(KERN_ERR "Thread creation failed\n");
        goto error;
    }


   __init_timer(&logic.lockOperationTimer, LockOperation, 0);
   LockOperation(&logic.lockOperationTimer);

   ReadApprovedNum();

   SetLedMode(GREEN_LED, MODE_ON);
   printk(KERN_INFO "Smart lock control module has inited sucsessfully\n");


error:{
	printk(KERN_ERR "Initting process has stopped by initialize error\n");
	SetLedMode(RED_LED, MODE_ON);
	returnedStatus = -1;
}



return returnedStatus;
}


static void __exit SmartLockControlDeinit(void) {
   free_irq( logic.BUTTON_IRQ, &logic.ButInterrupt_Num );
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
