#include <linux/module.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mordyk Alexander kurtwalkit@gmail.com");
MODULE_DESCRIPTION("Memory Linux module");
MODULE_VERSION("0.1");

static uint inputBufferSize=1;//input Param;
static bool isKmalloc=true;//input Param (kmalloc/vmalloc);

module_param(inputBufferSize, uint, 0);
module_param(isKmalloc, invbool, 0);

static char *buffer;

static int create_buffer(void)
{

	if(isKmalloc==true){
	buffer = kmalloc(sizeof(inputBufferSize), GFP_KERNEL);
    printk("I got: %zu bytes of memory\n", ksize(buffer));
    kfree(buffer);
	}
	else if (isKmalloc==false){
	buffer = vmalloc(sizeof(inputBufferSize));
    printk("I got virtual : %zu bytes of memory\n", sizeof(buffer));
    vfree(buffer);
	}else printk("I dont have memory\n"); 

    return 0;
}

static int __init memory_init(void) {
	
  	printk("Memory module start\n");
  	create_buffer();
	return 0;
}

static void __exit memory_exit(void) {
	printk("Memory module stop\n");
}

module_init(memory_init);
module_exit(memory_exit);
