/*
 * Texas Instruments TMP103 SMBus temperature sensor driver
 * Copyright (C) 2014 Heiko Schocher <hs@denx.de>
 *
 * Based on:
 * Texas Instruments TMP102 SMBus temperature sensor driver
 *
 * Copyright (C) 2010 Steven King <sfking@fdwdc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include "mcp6050_Sensor.h"
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/delay.h>



#define DEVICE_NAME	"mcp6050_Sensor"
#define INTERRUPT_GPIO    ((1 - 1) * 32 + 6)   // (position of letter in alphabet - 1) * 32 + pin number   PA6


static struct device *dev;
int interrupt_cnt;
int interrupt_n;


typedef struct  {
    struct i2c_client *client;
	struct class      *sys_class;

    int accel_values[3];
    int gyro_values[3];
    int temperature;
}mcp6050_data_t;

static mcp6050_data_t sensor;




int ReadParameters(void);







static int Get_accel_show(struct class *class,
	                         struct class_attribute *attr, char *buf)
{
	dev_info(dev, "%s\n", __FUNCTION__);
    printk ("accelerometer`s [x,y,z] value is [%d,%d,%d]\n", sensor.accel_values[0], sensor.accel_values[1], sensor.accel_values[2]);
    return 0;
}

static int Get_giro_show(struct class *class,
                           struct class_attribute *attr, char *buf)
{
    dev_info(dev, "%s\n", __FUNCTION__);
    printk ("gyroscop`s [x,y,z] value is [%d,%d,%d]\n", sensor.gyro_values[0], sensor.gyro_values[1],sensor.gyro_values[2]);
    return 0;
}

static int Get_temperature_show(struct class *class,
                                struct class_attribute *attr, char *buf)
{
    dev_info(dev, "%s\n", __FUNCTION__);
    printk ("value of temperature is %d\n", sensor.temperature);
    return 0; 
}

static int Get_interrupts_count_show(struct class *class,
                                     struct class_attribute *attr, char *buf){
    printk ("counted interrupts is  %d\n", interrupt_cnt);
    return 0; 
}

CLASS_ATTR_RO(Get_accel);
CLASS_ATTR_RO(Get_giro);
CLASS_ATTR_RO(Get_temperature);
CLASS_ATTR_RO(Get_interrupts_count);

static void make_sysfs_entry(struct i2c_client *drv_client)
{
	struct device_node *np = drv_client->dev.of_node;

	if (np) {
        struct class *sys_class;

		sys_class = class_create(THIS_MODULE, DEVICE_NAME);

		if (IS_ERR(sys_class)){
			dev_err(dev, "bad class create\n");
		}
		else{
            int res;
			res = class_create_file(sys_class, &class_attr_Get_accel);
            res = class_create_file(sys_class, &class_attr_Get_giro);
            res = class_create_file(sys_class, &class_attr_Get_temperature);
            res = class_create_file(sys_class, &class_attr_Get_interrupts_count);

            sensor.sys_class = sys_class;			
		}
	}

}

static int ConfigSensor(void){
    u8 returnedStatus = 1;

    // i2c_smbus_write_byte_data(sensor.client, REG_CONFIG, 0);
    // i2c_smbus_write_byte_data(sensor.client, REG_GYRO_CONFIG, 0);
    // i2c_smbus_write_byte_data(sensor.client, REG_ACCEL_CONFIG, 0);
    // i2c_smbus_write_byte_data(sensor.client, REG_FIFO_EN, 0);
    // i2c_smbus_write_byte_data(sensor.client, REG_INT_PIN_CFG, 0);
    // i2c_smbus_write_byte_data(sensor.client, REG_INT_ENABLE, 0);
    // i2c_smbus_write_byte_data(sensor.client, REG_USER_CTRL, 0);
    // i2c_smbus_write_byte_data(sensor.client, REG_PWR_MGMT_1, 0);
    // i2c_smbus_write_byte_data(sensor.client, REG_PWR_MGMT_2, 0);

    i2c_smbus_write_byte_data(sensor.client, REG_PWR_MGMT_1, 0x0);
    //msleep_interruptible(20);

    i2c_smbus_write_byte_data(sensor.client, REG_CONFIG, 0);
    i2c_smbus_write_byte_data(sensor.client, REG_GYRO_CONFIG, 0);
    i2c_smbus_write_byte_data(sensor.client, REG_ACCEL_CONFIG, 0);
    i2c_smbus_write_byte_data(sensor.client, REG_FIFO_EN, 0);
    i2c_smbus_write_byte_data(sensor.client, REG_INT_PIN_CFG, 0xA0);
    i2c_smbus_write_byte_data(sensor.client, REG_INT_ENABLE, 1);
    i2c_smbus_write_byte_data(sensor.client, REG_USER_CTRL, 0);
    i2c_smbus_write_byte_data(sensor.client, REG_PWR_MGMT_1, 0);
    i2c_smbus_write_byte_data(sensor.client, REG_PWR_MGMT_2, 0);
    returnedStatus = 0;
    return returnedStatus;
}


int ReadParameters(void){
    int result = 1;

    /* read data accel */
    sensor.accel_values[0] = (s16)((u16)i2c_smbus_read_word_swapped(sensor.client, REG_ACCEL_XOUT_H));
    sensor.accel_values[1] = (s16)((u16)i2c_smbus_read_word_swapped(sensor.client, REG_ACCEL_YOUT_H));
    sensor.accel_values[2] = (s16)((u16)i2c_smbus_read_word_swapped(sensor.client, REG_ACCEL_ZOUT_H));


    /* read data gyro */
    sensor.gyro_values[0] = (s16)((u16)i2c_smbus_read_word_swapped(sensor.client, REG_GYRO_XOUT_H));
    sensor.gyro_values[1] = (s16)((u16)i2c_smbus_read_word_swapped(sensor.client, REG_GYRO_YOUT_H));
    sensor.gyro_values[2] = (s16)((u16)i2c_smbus_read_word_swapped(sensor.client, REG_GYRO_ZOUT_H));

    /* read data temp */
    sensor.temperature = (s16)((u16)i2c_smbus_read_word_swapped(sensor.client, REG_TEMP_OUT_H));
    sensor.temperature =  (sensor.temperature + 12420 + 170) / 340;


    result = 0;

    return result;
}


static irqreturn_t Button_sw4_interrupt( int irq, void *dev_id ) {
   dev_err(dev, "in interrupt");

   return IRQ_NONE;
}



static  irq_handler_t gyroExternal_interrupt( int irq, void *dev_id , struct ptr_regs *regs) {
   interrupt_cnt ++;
   ReadParameters();   
   return (irq_handler_t)IRQ_HANDLED;
}


int mpu6050_irq_init(void){
    dev_err(dev, "IN irq init");
    gpio_request(INTERRUPT_GPIO, "sysfs");
    gpio_direction_input(INTERRUPT_GPIO);
    gpio_export(INTERRUPT_GPIO, false);

    dev_info(dev, "mpu6050: gpio %d connected to INT state is %d \n",
             INTERRUPT_GPIO, gpio_get_value(INTERRUPT_GPIO) );

    interrupt_n = gpio_to_irq(INTERRUPT_GPIO);
    dev_info(dev, "mpu6050: gpio %d irq is %d \n",   INTERRUPT_GPIO, interrupt_n );

    int res = 0;
  
    res = request_irq( interrupt_n, (irq_handler_t)gyroExternal_interrupt, IRQF_TRIGGER_RISING, "mpu_6050_irq_handler", NULL );
    
    return res;
}


void mpu6050_irq_Free(void){
    free_irq(interrupt_n, NULL);
    gpio_unexport(INTERRUPT_GPIO);
    gpio_free(INTERRUPT_GPIO);
}



static int Sensor_mcp6050_probe(struct i2c_client *drv_client,
			                    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter;
	dev = &drv_client->dev;

	dev_info(dev, "init I2C driver\n");
    dev_info(dev, "i2c client address is 0x%X\n", drv_client->addr);


    sensor.client = drv_client;

    if(drv_client != NULL ){
        i2c_set_clientdata(sensor.client, &sensor);
    }else{
         dev_err(dev,"pointer to drv_client is null");
    }
    adapter = drv_client->adapter;

    if (!adapter)
    {
        dev_err(dev, "adapter indentification error\n");
        return -ENODEV;
    }

    dev_info(dev, "I2C client address %d \n", drv_client->addr);

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
            dev_err(dev, "operation not supported\n");
            return -ENODEV;
    }


    ConfigSensor();
    dev_info(dev, "senser was cofigurated \n");

	make_sysfs_entry(sensor.client);

    dev_info(dev, "number of pin is %d\n", INTERRUPT_GPIO);    
    int res = mpu6050_irq_init();
    dev_info(dev, "irq initialized with status %d \n", res);

    interrupt_cnt = 0;
    dev_info(dev, "mcp6050_sensor driver successfully loaded\n");

	return 0;
}



static int Sensor_mcp6050_ssd1306_remove(struct i2c_client *client)
{
	mpu6050_irq_Free();

    class_remove_file(sensor.sys_class, &class_attr_Get_accel);
    class_remove_file(sensor.sys_class, &class_attr_Get_giro);
    class_remove_file(sensor.sys_class, &class_attr_Get_temperature);
    class_remove_file(sensor.sys_class, &class_attr_Get_interrupts_count);
	class_destroy(sensor.sys_class);

	dev_info(dev, "Driver was remuved successfully!\n");
	return 0;
}



/*=====
        CONFIGURATIONS TABLEs
=====*/

static const struct of_device_id mcp6050_Sensor_match[] = {
	{ .compatible = "MaGol,sensor_mcp6050", },
	{ },
};
MODULE_DEVICE_TABLE(of, mcp6050_Sensor_match);

static const struct i2c_device_id mcp6050_Sensor_id[] = {
	{ "sensor_mcp6050", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mcp6050_Sensor_id);


static struct i2c_driver mcp6050_Sensor_driver = {
	.driver = {
		.name	= DEVICE_NAME,
		.of_match_table = mcp6050_Sensor_match,
	},
	.probe		= Sensor_mcp6050_probe,
	.remove 	= Sensor_mcp6050_ssd1306_remove,
	.id_table	= mcp6050_Sensor_id,
};
module_i2c_driver(mcp6050_Sensor_driver);

MODULE_AUTHOR("MaksimHolikov, <golikov.mo@gmail.com>");
MODULE_DESCRIPTION("Driver to control sensor based on mcp6050 chip");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
