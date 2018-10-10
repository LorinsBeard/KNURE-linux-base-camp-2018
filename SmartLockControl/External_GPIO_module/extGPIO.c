
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>

#include "extGPIO.h"



#define DEVICE_NAME	"extGPIO"
#define INTERRUPT_GPIO    ((1 - 1) * 32 + 14)   // (position of letter in alphabet - 1) * 32 + pin number   PA14


typedef struct{
    struct gpio_desc *Unlock_gpio;
    u8     state;
}Lock_t;

typedef struct{
    struct gpio_desc *Gled_gpio;
    struct gpio_desc *Rled_gpio;
    struct gpio_desc *lockOpenled_gpio;
    struct gpio_desc *lockCloseled_gpio;
}Indicate_t;

typedef struct{
    struct device *dev;
    Lock_t        lock;
    Indicate_t    leds;
    u32           countINT;
    u16           numberINT;
}extDevice_t;



static extDevice_t extDevices;
static const struct of_device_id extGPIO_Devices_match[];


void SetLockState(u8 state){       
    gpiod_set_value(extDevices.lock.Unlock_gpio, state);
    extDevices.lock.state = state;

    if(state = STATE_LOCK){
        printk("Lock state is LOCK");
    }else{
        printk("Lock state is UNLOCK");
    }
}
EXPORT_SYMBOL(SetLockState);


u8   GetLockState(void){
    return extDevices.lock.state;
}
EXPORT_SYMBOL(GetLockState);


u8   SetLedMode(u8 led, u8 mode){
    u8 status = 0;
   
    if(led < Leds_Amount){
       status = 1;
       switch(led){
        case RED_LED:
            gpiod_set_value(extDevices.leds.Rled_gpio, mode);
        break;
        case GREEN_LED:
            gpiod_set_value(extDevices.leds.Gled_gpio, mode);
        break;
        case LOCK_OPEN_LED:
            gpiod_set_value(extDevices.leds.lockOpenled_gpio, mode);
        break;
        case LOCK_CLOSE_LED:
            gpiod_set_value(extDevices.leds.lockCloseled_gpio, mode);
        break;
        }
    }

    return status;
}
EXPORT_SYMBOL(SetLedMode);


int  GetInterruptNumber(void){
    return extDevices.countINT;
}
EXPORT_SYMBOL(GetInterruptNumber);


static  irq_handler_t extGPIO_button_interrupt(unsigned int irq, void *dev_id , struct ptr_regs *regs) {
   extDevices.countINT ++;
   printk("count of button INT %d", extDevices.countINT);

   return (irq_handler_t)IRQ_HANDLED;
}




/*=====
        INIT / DEINIT functions
=====*/
static int get_gpios_info(struct platform_device *pDev){
    struct device_node *np = pDev->dev.of_node;
    const char *name;
    u8     operationStatus = 0;
    
    dev_err(extDevices.dev, "read node\n");

    if (np) {
        if (!of_property_read_string(np, "label", &name))
            dev_info(extDevices.dev, "label = %s\n", name);

        //GREEN Led
        extDevices.leds.Gled_gpio = devm_gpiod_get(extDevices.dev, "greenLed", GPIOD_OUT_HIGH);
        if (IS_ERR(extDevices.leds.Gled_gpio)) {
            dev_err(extDevices.dev, "fail to get greenLed-gpios()\n");
            return EINVAL;
        }
        if(!gpiod_direction_output(extDevices.leds.Gled_gpio, 1))
           dev_info(extDevices.dev, "greenLed-gpios set as OUT\n");


        //RED Led
        extDevices.leds.Rled_gpio = devm_gpiod_get(extDevices.dev, "redLed", GPIOD_OUT_HIGH);
        if (IS_ERR(extDevices.leds.Rled_gpio)) {
            dev_err(extDevices.dev, "fail to get redLed-gpios()\n");
            return EINVAL;
        }
        if(!gpiod_direction_output(extDevices.leds.Rled_gpio, 1))
           dev_info(extDevices.dev, "redLed-gpios set as OUT\n");


        //Lock open Led
        extDevices.leds.lockOpenled_gpio = devm_gpiod_get(extDevices.dev, "lockOpenLed", GPIOD_OUT_HIGH);
        if (IS_ERR(extDevices.leds.lockOpenled_gpio)) {
            dev_err(extDevices.dev, "fail to get lockOpenLed-gpios()\n");
            return EINVAL;
        }
        if(!gpiod_direction_output(extDevices.leds.lockOpenled_gpio, 1))
           dev_info(extDevices.dev, "lockOpenLed-gpios set as OUT\n");


       //Lock close Led
        extDevices.leds.lockCloseled_gpio = devm_gpiod_get(extDevices.dev, "lockCloseLed", GPIOD_OUT_HIGH);
        if (IS_ERR(extDevices.leds.lockCloseled_gpio)) {
            dev_err(extDevices.dev, "fail to get lockCloseLed-gpios()\n");
            return EINVAL;
        }
        if(!gpiod_direction_output(extDevices.leds.lockCloseled_gpio, 1))
           dev_info(extDevices.dev, "lockCloseLed-gpios set as OUT\n");

       //Unlock 
        extDevices.lock.Unlock_gpio = devm_gpiod_get(extDevices.dev, "unlock", GPIOD_OUT_HIGH);
        if (IS_ERR(extDevices.lock.Unlock_gpio)) {
            dev_err(extDevices.dev, "fail to get unlock-gpios()\n");
            return EINVAL;
        }
        if(!gpiod_direction_output(extDevices.lock.Unlock_gpio, 1))
           dev_info(extDevices.dev, "unlock-gpios set as OUT\n");
       
        
        operationStatus = 1;
    }
    else{
        dev_err(extDevices.dev, "failed to get device_node\n");
        return -EINVAL;
    }

    return operationStatus;
}

int extGPIO_button_irq_init(void){
    
    gpio_request(INTERRUPT_GPIO, "sysfs");
    gpio_direction_input(INTERRUPT_GPIO);
    gpio_export(INTERRUPT_GPIO, false);

    dev_info(extDevices.dev, "extGPIO_button: gpio %d connected to INT state is %d \n",
             INTERRUPT_GPIO, gpio_get_value(INTERRUPT_GPIO) );

    extDevices.numberINT = gpio_to_irq(INTERRUPT_GPIO);
    dev_info(extDevices.dev, "extGPIO_button: gpio %d irq is %d \n",   INTERRUPT_GPIO, extDevices.numberINT );

    int res = 0;
    res = request_irq( extDevices.numberINT,
                       (irq_handler_t)extGPIO_button_interrupt,
                       IRQF_TRIGGER_RISING,
                       "extGPIO_button_irq_handler",
                        NULL );
    
    return res;
}


void extGPIO_button_irq_Free(void){
    free_irq(extDevices.numberINT, NULL);
    gpio_unexport(INTERRUPT_GPIO);
    gpio_free(INTERRUPT_GPIO);
}



static int ExtGPIO_Init(struct platform_device *pDev){
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

    extGPIO_button_irq_init();


    dev_info(extDevices.dev, "extDriver was inited successfully!\n");

    return 0;
}



static int ExtGPIO_Deinit(struct platform_device *pDev){
    extGPIO_button_irq_Free();

   SetLedMode(RED_LED, MODE_OFF);
   SetLedMode(GREEN_LED, MODE_OFF);
   SetLedMode(LOCK_OPEN_LED, MODE_OFF);
   SetLedMode(LOCK_CLOSE_LED, MODE_OFF);
   SetLockState(STATE_LOCK);


	dev_info(extDevices.dev, "extDriver was removed successfully!\n");
	return 0;
}



/*=====
        CONFIGURATIONS TABLEs
=====*/

static const struct of_device_id extGPIO_Devices_match[] = {
	{ .compatible = "MaGol_OKL,externalGPIO", },
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