#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tymoshenko Vladyslav");

static bool isVmalloc=true;

module_param(isVmalloc, bool, 0);

static uint *buffer ;

static uint vmem_size = 1024;

static size_t kmem_size = 512;

static int memory_allocation(void)
{
 do
  {
   if(isVmalloc==true)
    {
		  buffer = vmalloc(vmem_size);
    	printk("%u bytes of virtual memory was allocated\n", buffer);
    	vfree(buffer);
    	vmem_size += 32;
	}
	else if (isVmalloc==false)
	{
		  buffer = kmalloc(kmem_size, GFP_KERNEL);
    	printk("%zu bytes of memory was allocated\n", buffer);
    	kfree(buffer);
	} 
 }while( buffer !=  0);
 printk("Can not allocate memory\n");
 return 0;
}

static int __init allocation_init(void) {
	
  	printk("Memory module start\n");
  	memory_allocation();
	return 0;
}

static void __exit allocation_exit(void) {
	printk("Memory manager have stoped\n");
}

module_init(allocation_init);
module_exit(allocation_exit);