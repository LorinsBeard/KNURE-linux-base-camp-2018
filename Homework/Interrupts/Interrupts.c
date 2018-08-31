#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/fs.h>

#define DRIVER_NAME			"MyDriver"

//#define DRIVER_NAME   "My_interrupts_driver"
#define ROOTFS_NAME   "My_interrupts"


static struct class  *sys_FS;
static struct device *dev;


static const struct of_device_id my_drv_match[];









static ssize_t start_show(struct class *class,
	                      struct class_attribute *attr, char *buf)
{
	ssize_t i = 0;
	dev_info(dev, "in start");
	printk("start");	

	return i;
}

static ssize_t stop_show(struct class *class,
	                      struct class_attribute *attr, char *buf)
{
	ssize_t i = 0;
	dev_info(dev, "in stop");
	printk("stop");	

	return i;
}

static ssize_t clear_show(struct class *class,
	                      struct class_attribute *attr, char *buf)
{
	ssize_t i = 0;
	dev_info(dev, "in clear");
	printk("clear");	

	return i;
}


CLASS_ATTR_RO(start);
CLASS_ATTR_RO(stop);
CLASS_ATTR_RO(clear);

static void make_rootfs_entry(void)
{
	struct class *sysFS;
	
	sysFS = class_create(THIS_MODULE, ROOTFS_NAME);

	if (IS_ERR(sysFS)){
		dev_err(dev, "bad class create\n");
	}
	else{
		class_create_file(sysFS, &class_attr_start);
		class_create_file(sysFS, &class_attr_stop);
		class_create_file(sysFS, &class_attr_clear);

		sys_FS = sysFS;

		dev_info(dev, "sys class created = %s\n", ROOTFS_NAME);
	}
}








static int  Initialize(struct platform_device *pDev){
	

    const struct of_device_id *match;
	const char   *name;
    struct device_node *np;


	dev = &pDev->dev;
    
	match = of_match_device(of_match_ptr(my_drv_match), dev);
	if (!match) {
		dev_err(dev, "failed of_match_device()\n");
		return -EINVAL;
	}

    np = pDev->dev.of_node;
    if (np) {
		if (!of_property_read_string(np, "label", &name))
			dev_info(dev, "label = %s\n", name);

		if (np->name)
			dev_info(dev, "np->name = %s\n", np->name);
        
	}
	else{
		dev_err(dev, "failed to get device_node\n");
		return -EINVAL;
	}







  //make_rootfs_entry();
	

  dev_info(dev, "Inited");
  return 0;
}


static int Deinitialize(struct platform_device *pDev){
    class_remove_file(sys_FS, &class_attr_start);
    class_remove_file(sys_FS, &class_attr_stop);
    class_remove_file(sys_FS, &class_attr_clear);
    class_destroy(sys_FS);


    dev_info(dev, "deinetialezed");
    
    return 0;
}






static const struct of_device_id my_drv_match[] = {
	{ .compatible = "MaGol,test_drv", },
	{ },
};
MODULE_DEVICE_TABLE(of, my_drv_match);
 

static struct platform_driver my_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = my_drv_match,
	},
	.probe 		= Initialize,
	.remove 	= Deinitialize,
};
module_platform_driver(my_driver);










// static struct of_device_id interrupts_of_match[] = {
//   { .compatible = "MaGol,testDRV", },
//     {}
//   };
// MODULE_DEVICE_TABLE(of, interrupts_of_match);


// static struct platform_driver MyInterrupts_driver = {
//   .probe = Initialize,
//   .remove = Deinitialize,
//   .driver = {
//     .name = "DRIVER_NAME",
//     .owner = THIS_MODULE,
//     .of_match_table = interrupts_of_match,
//   },
// };

// module_init(Initialize);
// module_exit(Deinitialize);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maksim Holikiv  golikov.mo@gmail.com");
MODULE_DESCRIPTION("Modul for test interrupts");
MODULE_VERSION("1.0");