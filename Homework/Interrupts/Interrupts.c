#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/fs.h>
#include <linux/leds.h>
#include <linux/interrupt.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>


#define DRIVER_NAME	  "MyDriver"
#define ROOTFS_NAME   "My_interrupts"


#define BUTTON_SW4_IRQ 117


typedef struct {
	struct device       *dev;
    struct class        *sys_FS;
    
    struct gpio_desc    *LED_gpiod;

	struct led_trigger *led;

}myOBJs_t;


static u8                  ledState;
static u8                  countOfInterrupt;
static bool                isCalculateStatisticAppruved;
static int                 sw4_interruptID;
static myOBJs_t            obj_s;




static const struct of_device_id my_drv_match[];


static int startCount_show(struct class *class,
	                      struct class_attribute *attr, char *buf)
{
	int i = 0;
    isCalculateStatisticAppruved  = true;
    dev_info(obj_s.dev, "Interrupt of button has enable");	
	return i;
}

static int stopCount_show(struct class *class,
	                      struct class_attribute *attr, char *buf)
{
	int i = 0;
    isCalculateStatisticAppruved = false;
	dev_info(obj_s.dev, "Interrupt of button has disenable");
	return i;
}

static int clearStatistic_show(struct class *class,
	                      struct class_attribute *attr, char *buf)
{
	int i = 0;	
	countOfInterrupt = 0;
    dev_info(obj_s.dev,"Interrupts count equal %d \n", countOfInterrupt);

	return i;
}

static int ledSolid_show(struct class *class,
	                      struct class_attribute *attr, char *buf)
{
	int i = 0;
	dev_info(obj_s.dev, "in solid led");

    u8 state;
    if(ledState){
       state = 0; //LED_OFF;
    }else{
       state = 1; //LED_FULL;
    }
    ledState = !ledState;
	gpiod_set_value(obj_s.LED_gpiod, state);

	return i;
}
static int ledBlink_show(struct class *class,
	                      struct class_attribute *attr, char *buf)
{
	int i = 0;
	dev_info(obj_s.dev, "in ledBlink");

	return i;
}

static int ShowCount_show(struct class *class,
	                      struct class_attribute *attr, char *buf)
{	
	dev_info(obj_s.dev,"Interrupts count equal %d \n", countOfInterrupt);
	return 0;
}

CLASS_ATTR_RO(startCount);
CLASS_ATTR_RO(stopCount);
CLASS_ATTR_RO(clearStatistic);
CLASS_ATTR_RO(ledSolid);
CLASS_ATTR_RO(ledBlink);
CLASS_ATTR_RO(ShowCount);

static void make_sysfs_entry(void)
{
	struct class *sysFS;
	
	sysFS = class_create(THIS_MODULE, ROOTFS_NAME);

	if (IS_ERR(sysFS)){
		dev_err(obj_s.dev, "bad class create\n");
	}
	else{
		class_create_file(sysFS, &class_attr_startCount);
		class_create_file(sysFS, &class_attr_stopCount);
		class_create_file(sysFS, &class_attr_clearStatistic);
		class_create_file(sysFS, &class_attr_ledSolid);
		class_create_file(sysFS, &class_attr_ledBlink);
		class_create_file(sysFS, &class_attr_ShowCount);

		obj_s.sys_FS = sysFS;

		dev_info(obj_s.dev, "sys class created = %s \n", ROOTFS_NAME);
	}
}


static irqreturn_t Button_sw4_interrupt( int irq, void *dev_id ) {
   if(isCalculateStatisticAppruved){
   		countOfInterrupt += 1;

		u8 state = 0;
   		if((countOfInterrupt % 2) == 0){
   			state = 1;
   		}
   		gpiod_set_value(obj_s.LED_gpiod, state);
   }

   return IRQ_NONE;
}





static int  Initialize(struct platform_device *pDev){
	

    const struct of_device_id *match;
	const char                *name;
    struct device_node        *np;


	obj_s.dev = &pDev->dev;
    
	match = of_match_device(of_match_ptr(my_drv_match), obj_s.dev);
	if (!match) {
		dev_err(obj_s.dev, "failed of_match_device()\n");
		return -EINVAL;
	}

    np = pDev->dev.of_node;
    if (np) {
		if (!of_property_read_string(np, "label", &name))
			dev_info(obj_s.dev, "label = %s\n", name);

		if (np->name)
			dev_info(obj_s.dev, "np->name = %s\n", np->name);

		obj_s.LED_gpiod = devm_gpiod_get(obj_s.dev, "led", GPIOD_OUT_HIGH);
		 if (IS_ERR(obj_s.LED_gpiod)) {
		 	dev_err(obj_s.dev, "fail to get led-gpios()\n");
		 	return EINVAL;
		 }
		 if(!gpiod_direction_output(obj_s.LED_gpiod, 1))
		 	dev_info(obj_s.dev, "led-gpios set as OUT\n");

        
	}
	else{
		dev_err(obj_s.dev, "failed to get device_node\n");
		return -EINVAL;
	}
    
    sw4_interruptID = 0;
    if ( request_irq( BUTTON_SW4_IRQ, Button_sw4_interrupt, IRQF_SHARED, "Button_sw4_interrupt", &sw4_interruptID ) )
      return -1;

  

  	isCalculateStatisticAppruved  = true;
  	countOfInterrupt = 0;
    
    led_trigger_register_simple(dev_name(obj_s.dev), &obj_s.led );
    led_trigger_event(obj_s.led, LED_FULL);;


    

  make_sysfs_entry();
	

  dev_info(obj_s.dev, "Inited");
  return 0;
}


static int Deinitialize(struct platform_device *pDev){
    class_remove_file(obj_s.sys_FS, &class_attr_startCount);
    class_remove_file(obj_s.sys_FS, &class_attr_stopCount);
    class_remove_file(obj_s.sys_FS, &class_attr_clearStatistic);
    class_remove_file(obj_s.sys_FS,  &class_attr_ledSolid);
	class_remove_file(obj_s.sys_FS,  &class_attr_ledBlink);
	class_remove_file(obj_s.sys_FS,  &class_attr_ShowCount);
    class_destroy(obj_s.sys_FS);
    free_irq( BUTTON_SW4_IRQ, &sw4_interruptID );
    
    led_trigger_unregister_simple(obj_s.led);

    dev_info(obj_s.dev, "deinetialezed");
    
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







MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maksim Holikiv  golikov.mo@gmail.com");
MODULE_DESCRIPTION("Modul for test interrupts");
MODULE_VERSION("1.0");