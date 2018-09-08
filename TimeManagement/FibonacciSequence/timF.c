
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/init.h> /* Needed for the macros */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aleksandr Mordyk");
MODULE_DESCRIPTION("Home-work lesson 7 (fibonacci)");
MODULE_VERSION("0.1");

static long fibon = 0; //counter for fibonacci;
/*
@Description - function to calculating Fibonacci numbers;
@param - number;
@return - number of Fibonacci;
*/
int fibonacci(int N)  
{
if (N<=46){	
  if (N == 1 || N == 2)
    return 1; 
  return fibonacci(N - 1) + fibonacci(N - 2);
}else return 2147483641;
}

int g_time_interval = 3000;
struct timer_list g_timer;
 
static void _TimerHandler(struct timer_list * g_timer)
{
    /*Restarting the timer...*/
    mod_timer(g_timer, jiffies + msecs_to_jiffies(g_time_interval));
    fibon=fibon+1;
    printk(KERN_INFO "Fibonacci:\n");
	printk("%d ", fibonacci(fibon));	
}

static int __init example_init(void)
{
    printk(KERN_INFO "Fibonacci modele install!!!.\n");
    /*Starting the timer.*/	
    __init_timer(&g_timer,_TimerHandler, 0);
    mod_timer( &g_timer, jiffies + msecs_to_jiffies(g_time_interval));
    return 0;
}

static void __exit example_exit(void)
{
    del_timer(&g_timer);
    printk(KERN_INFO "Fibonacci module exited from kernel!!!\n");
}

module_init(example_init);
module_exit(example_exit);
