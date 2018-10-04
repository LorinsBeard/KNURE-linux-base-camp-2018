#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include "External_GPIO_module/extGPIO.h"


















/*=====
        INIT / DEINIT functions
=====*/

static int __init SmartLockControlInit(void) {
    printk("Smart lock control module has inited sucsessfully\n");
   

return 0;
}

static void __exit SmartLockControlDeinit(void) {
   printk("Smart lock control module has deinited sucsessfully\n");
}

module_init(SmartLockControlInit);
module_exit(SmartLockControlDeinit);




MODULE_AUTHOR("MaksimHolikov, <golikov.mo@gmail.com>");
MODULE_DESCRIPTION("Driver to implement logic of smart lock");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");