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


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include "mcp6050_Sensor.h"

#define DEVICE_NAME	"mcp6050_Sensor"
 


static struct device *dev;




typedef struct  {
    struct i2c_client *client;
	struct class      *sys_class;

    int accel_values[3];
    int gyro_values[3];
    int temperature;
}mcp6050_data_t;

static mcp6050_data_t sensor;




int ReadParameters(void);







static int Get_accel_X_show(struct class *class,
	                         struct class_attribute *attr, char *buf)
{
	dev_info(dev, "%s\n", __FUNCTION__);
    ReadParameters();
    printk ("accelerometer`s X value is %d\n", sensor.accel_values[0]);
    return 0;
}

static int Get_accel_Y_show(struct class *class,
                            struct class_attribute *attr, char *buf)
{
    dev_info(dev, "%s\n", __FUNCTION__);
    ReadParameters();
    printk ("accelerometer`s Y value is %d\n", sensor.accel_values[1]);
    return 0;
}

static int Get_accel_Z_show(struct class *class,
                            struct class_attribute *attr, char *buf)
{
    dev_info(dev, "%s\n", __FUNCTION__);
    ReadParameters();
    printk ("accelerometer`s Z value is %d\n", sensor.accel_values[2]);
    return 0;  
}

static int Get_giro_X_show(struct class *class,
                           struct class_attribute *attr, char *buf)
{
    dev_info(dev, "%s\n", __FUNCTION__);
    ReadParameters();
    printk ("gyroscop`s X value is %d\n", sensor.gyro_values[0]);
    return 0;
}

static int Get_giro_Y_show(struct class *class,
                           struct class_attribute *attr, char *buf)
{
    dev_info(dev, "%s\n", __FUNCTION__);
    ReadParameters();
    printk ("gyroscop`s Y value is %d\n", sensor.gyro_values[1]);
    return 0;
}

static  Get_giro_Z_show(struct class *class,
                                 struct class_attribute *attr, char *buf)
{
    dev_info(dev, "%s\n", __FUNCTION__); 
    ReadParameters();
    printk ("gyroscop`s Z value is %d\n", sensor.gyro_values[2]);
    return 0;  
}

static int Get_temperature_show(struct class *class,
                                struct class_attribute *attr, char *buf)
{
    dev_info(dev, "%s\n", __FUNCTION__);
    ReadParameters();
    printk ("value of temperature is %d\n", sensor.temperature);
    return 0; 
}


CLASS_ATTR_RO(Get_accel_X);
CLASS_ATTR_RO(Get_accel_Y);
CLASS_ATTR_RO(Get_accel_Z);
CLASS_ATTR_RO(Get_giro_X);
CLASS_ATTR_RO(Get_giro_Y);
CLASS_ATTR_RO(Get_giro_Z);
CLASS_ATTR_RO(Get_temperature);


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
			res = class_create_file(sys_class, &class_attr_Get_accel_X);
			res = class_create_file(sys_class, &class_attr_Get_accel_Y);
            res = class_create_file(sys_class, &class_attr_Get_accel_Z);
            res = class_create_file(sys_class, &class_attr_Get_giro_X);
            res = class_create_file(sys_class, &class_attr_Get_giro_Y);
            res = class_create_file(sys_class, &class_attr_Get_giro_Z);
            res = class_create_file(sys_class, &class_attr_Get_temperature);

            sensor.sys_class = sys_class;			
		}
	}

}

static int ConfigSensor(void){
    u8 returnedStatus = 1;
    i2c_smbus_write_byte_data(sensor.client, REG_CONFIG, 0);
    i2c_smbus_write_byte_data(sensor.client, REG_GYRO_CONFIG, 0);
    i2c_smbus_write_byte_data(sensor.client, REG_ACCEL_CONFIG, 0);
    i2c_smbus_write_byte_data(sensor.client, REG_FIFO_EN, 0);
    i2c_smbus_write_byte_data(sensor.client, REG_INT_PIN_CFG, 0);
    i2c_smbus_write_byte_data(sensor.client, REG_INT_ENABLE, 0);
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

    ConfigSensor();
    dev_err(dev,"sensor has configured");

 //   adapter = drv_client->adapter;

 //    if (!adapter)
 //    {
 //        dev_err(dev, "adapter indentification error\n");
 //        return -ENODEV;
 //    }

 //    dev_info(dev, "I2C client address %d \n", drv_client->addr);

 //    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
 //            dev_err(dev, "operation not supported\n");
 //            return -ENODEV;
 //    }


	make_sysfs_entry(sensor.client);
    dev_info(dev, "mcp6050_sensor driver successfully loaded\n");

	return 0;
}



static int Sensor_mcp6050_ssd1306_remove(struct i2c_client *client)
{
	
    class_remove_file(sensor.sys_class, &class_attr_Get_accel_X);
    class_remove_file(sensor.sys_class, &class_attr_Get_accel_Y);
    class_remove_file(sensor.sys_class, &class_attr_Get_accel_Z);
    class_remove_file(sensor.sys_class, &class_attr_Get_giro_X);
    class_remove_file(sensor.sys_class, &class_attr_Get_giro_Y);
    class_remove_file(sensor.sys_class, &class_attr_Get_giro_Z);
    class_remove_file(sensor.sys_class, &class_attr_Get_temperature);
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
