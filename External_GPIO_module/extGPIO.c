
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>

#include "extGPIO.h"



#define DEVICE_NAME	"extGPIO"
#define SYSFS_NAME  "extGPIO"
#define INTERRUPT_GPIO    ((1 - 1) * 32 + 20)   // (position of letter in alphabet - 1) * 32 + pin number   PA20


typedef struct{
    struct gpio_desc *Lock_gpio;
    struct gpio_desc *Unlock_gpio;
    u8     state;
}Lock_t;

typedef struct{
    struct gpio_desc *Gled_gpio;
    struct gpio_desc *Rled_gpio;
}Indicate_t;

typedef struct{
    struct class  *sys_class;
    struct device *dev;
    Lock_t        lock;
    Indicate_t    leds;
    u32           countINT;
    u16           numberINT;
}extDevice_t;



static extDevice_t extDevices;
static const struct of_device_id extGPIO_Devices_match[];





void SetLockState(u8 state){

   switch(state){
    default:
    case STATE_LOCK:
        gpiod_set_value(extDevices.lock.Lock_gpio, 1);
        gpiod_set_value(extDevices.lock.Unlock_gpio, 0);
        extDevices.lock.state = STATE_LOCK;
    break;
    case STATE_UNLOCK:
        gpiod_set_value(extDevices.lock.Lock_gpio, 0);
        gpiod_set_value(extDevices.lock.Unlock_gpio, 1);
        extDevices.lock.state = STATE_UNLOCK;
    break;
   }
}

u8   GetLockState(void){
    return extDevices.lock.state;
}

u8   SetLedMode(u8 led, u8 mode){
    u8 status = 0;

    if(led < Leds_Amount){
       status = 1;

       switch(led){
        case RED_LED:
            gpiod_set_value(extDevices.leds.Gled_gpio, mode);
        break;
        case GREEN_LED:
            gpiod_set_value(extDevices.leds.Gled_gpio, mode);
        break;
        }
    }

    return status;
}

int  GetInterruptNumber(void){
    return 1000;
}

static  irq_handler_t extGPIO_button_interrupt( int irq, void *dev_id , struct ptr_regs *regs) {
   extDevices.countINT ++;

   return (irq_handler_t)IRQ_HANDLED;
}


//============DEBUG
static int ShowIntCnt_show(struct class *class,
                          struct class_attribute *attr, char *buf)
{   
    dev_info(extDevices.dev,"Interrupts count equal %d \n", extDevices.countINT);
    return 0;
}

CLASS_ATTR_RO(ShowIntCnt);

static void make_sysfs_entry(void)
{
    struct class *sysFS;
    
    sysFS = class_create(THIS_MODULE, SYSFS_NAME);

    if (IS_ERR(sysFS)){
        dev_err(extDevices.dev, "bad class create\n");
    }
    else{
        class_create_file(sysFS, &class_attr_ShowIntCnt);

        extDevices.sys_class = sysFS;

        dev_info(extDevices.dev, "sys class created = %s \n", SYSFS_NAME);
    }
}



//===========




/*=====
        INIT / DEINIT functions
=====*/
static int get_gpios_info(struct platform_device *pDev)
{
    struct device_node *np = pDev->dev.of_node;
    const char *name;
    u8     operationStatus = 0;
    
    dev_err(extDevices.dev, "read node\n");
    if (np) {
        if (!of_property_read_string(np, "label", &name))
            dev_info(extDevices.dev, "label = %s\n", name);

        //GREEN Led
        extDevices.leds.Gled_gpio = devm_gpiod_get(extDevices.dev, "green_Led", GPIOD_OUT_HIGH);
        if (IS_ERR(extDevices.leds.Gled_gpio)) {
            dev_err(extDevices.dev, "fail to get green_Led-gpios()\n");
            return EINVAL;
        }
        if(!gpiod_direction_output(extDevices.leds.Gled_gpio, 1))
           dev_info(extDevices.dev, "green_Led-gpios set as OUT\n");


    // //RED Led
    //     extDevices.leds.Rled_gpio = devm_gpiod_get(extDevices.dev, "red_Led", GPIOD_OUT_LOW);
    //     if (IS_ERR(extDevices.leds.Rled_gpio)) {
    //         dev_err(extDevices.dev, "fail to get red_Led-gpios()\n");
    //         return EINVAL;
    //     }
    //     if(!gpiod_direction_output(extDevices.leds.Rled_gpio, 1))
    //        dev_info(extDevices.dev, "red_Led-gpios set as OUT\n");
       
        
        operationStatus = 1;
    }
    else{
        dev_err(extDevices.dev, "failed to get device_node\n");
        return -EINVAL;
    }

    return operationStatus;
}

int extGPIO_button_irq_init(void){
    dev_err(extDevices.dev, "IN irq init");
    gpio_request(INTERRUPT_GPIO, "sysfs");
    gpio_direction_input(INTERRUPT_GPIO);
    gpio_export(INTERRUPT_GPIO, false);

    dev_info(extDevices.dev, "extGPIO_button: gpio %d connected to INT state is %d \n",
             INTERRUPT_GPIO, gpio_get_value(INTERRUPT_GPIO) );

    extDevices.numberINT = gpio_to_irq(INTERRUPT_GPIO);
    dev_info(extDevices.dev, "extGPIO_button: gpio %d irq is %d \n",   INTERRUPT_GPIO, extDevices.numberINT );

    int res = 0;
  
    res = request_irq( extDevices.numberINT, (irq_handler_t)extGPIO_button_interrupt,
                       IRQF_TRIGGER_RISING, "extGPIO_button_irq_handler", NULL );
    
    return res;
}


void extGPIO_button_irq_Free(void){
    free_irq(extDevices.numberINT, NULL);
    gpio_unexport(INTERRUPT_GPIO);
    gpio_free(INTERRUPT_GPIO);
}


static int ExtGPIO_Init(struct platform_device *pDev)
{
   const struct of_device_id *match;


    extDevices.dev = &pDev->dev;
    match          = of_match_device(of_match_ptr(extGPIO_Devices_match), extDevices.dev);
    if(!match){
        dev_err(extDevices.dev, "failed of_match_device \n");
        return -EINVAL;
    }
    int status = get_gpios_info(pDev);

    if(!status){
        dev_err(extDevices.dev, "failed to read get_gpios_info() \n");
        return -EINVAL;
    }
    // gpiod_set_value(extDevices.leds.Gled_gpio, 1);
    // gpiod_set_value(extDevices.leds.Rled_gpio, 1);


    extDevices.numberINT = 0;
    extGPIO_button_irq_init();

    make_sysfs_entry();

    dev_info(extDevices.dev, "extDriver was inited successfully!\n");

    return 0;
}



static int ExtGPIO_Deinit(struct platform_device *pDev)
{
    extGPIO_button_irq_Free();

    class_remove_file(extDevices.sys_class, &class_attr_ShowIntCnt);
    class_destroy(extDevices.sys_class);

    gpiod_set_value(extDevices.leds.Gled_gpio, 0);
    gpiod_set_value(extDevices.leds.Rled_gpio, 0);

	dev_info(extDevices.dev, "extDriver was removed successfully!\n");
	return 0;
}



/*=====
        CONFIGURATIONS TABLEs
=====*/

static const struct of_device_id extGPIO_Devices_match[] = {
	{ .compatible = "MaGol_AlKl,externalGPIO", },
	{ },
};
MODULE_DEVICE_TABLE(of, extGPIO_Devices_match);

static struct platform_driver extGPIO_driver = {
    .driver = {
        .name = "ExtGPIO",
        .of_match_table = extGPIO_Devices_match,
    },
    .probe      = ExtGPIO_Init,
    .remove     = ExtGPIO_Deinit,
};
module_platform_driver(extGPIO_driver);


MODULE_AUTHOR("MaksimHolikov, <golikov.mo@gmail.com>");
MODULE_DESCRIPTION("Driver to control external devices of smart lock");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");