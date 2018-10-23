/*
 * GL BaseCamp MFRC522 RFID driver
 * Copyright (C) 2018 Oleksii Klochko <lorins.dm@gmail.com>
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include "mfrc522.h"
#include "mfrc522_api.h"


#define DEVICE_NAME	"rfid_mfrc522"
#define CLASS_NAME	"rfid_reader" 
#define BUS_NAME	"spi0"


static struct device *dev;


struct mfrc522_data {
    struct spi_device 		*spi;
	struct class 			*sys_class;

	struct gpio_desc 		*irq; 		/* MFRC522 interrupt pin */
    struct gpio_desc 		*rst; 		/* MFRC522 reset pin */

};

static struct mfrc522_data *rfid;



/*-------------Functions definitions-------------*/

static int PCD_WriteRegister(PCD_Register reg, uint8_t *buff, uint8_t length)
{
	int returnedValue = -1;
	uint8_t tmpReg = reg;

	returnedValue = spi_write(rfid->spi, &tmpReg, 1);
	returnedValue = spi_write(rfid->spi, buff, length);

	return returnedValue;
}

static int PCD_ReadRegister(PCD_Register reg, uint8_t *buff, uint8_t length, uint8_t rxAlign)
{
	int returnedValue = -1;

	uint8_t address = 0x80 | reg;
	uint8_t index = 0;
	length = length - 1;
	returnedValue = spi_write(rfid->spi, &address, 1);
	if (rxAlign)
	{
		uint8_t mask = (0xFF << rxAlign) & 0xFF;
		int value = spi_w8r8(rfid->spi, address); //TODO: Error detection
		buff[0] = (buff[0] & ~mask) | ((uint8_t)value & mask);
		index = index + 1;
	}

	while (index < length)
	{
		//TODO: Error detection
		buff[index] = spi_w8r8(rfid->spi, address);
		index = index + 1;
	}

	buff[index] = spi_w8r8(rfid->spi, 0);

	return returnedValue;
}

static int PCD_SetRegisterBitMask(PCD_Register reg, uint8_t mask)
{
	int returnedValue = -1;
	uint8_t tmpValue;

	returnedValue = PCD_ReadRegister(reg, &tmpValue, 1, 0);
	tmpValue = tmpValue | mask;
	returnedValue = PCD_WriteRegister(reg, &tmpValue, 1);	

	return returnedValue;
}

static int PCD_ClearRegisterBitMask(PCD_Register reg, uint8_t mask)
{
	int returnedValue = -1;
	uint8_t tmpValue;

	returnedValue = PCD_ReadRegister(reg, &tmpValue, 1, 0);
	tmpValue = tmpValue & (~mask);
	returnedValue = PCD_WriteRegister(reg, &tmpValue, 1);	

	return returnedValue;
}

static int PCD_HardReset(void)
{
	int returnedValue = -1;


	return returnedValue;
}

static int PCD_AntennaOn(void)
{
	int returnedValue = -1;
	uint8_t value = 0;

	returnedValue = PCD_ReadRegister(TxControlReg, &value, 1, 0);
	if ((value & 0x03) != 0x03) {
		uint8_t tmpValue = value | 0x03;
		returnedValue = PCD_WriteRegister(TxControlReg, &tmpValue, 1);
	}

	return returnedValue;
}

static int PCD_AntennaOff(void)
{
	int returnedValue = -1;

	returnedValue = PCD_ClearRegisterBitMask(TxControlReg, 0x03);

	return returnedValue;
}



static int initMFRC522(struct spi_device *spidev)
{
	int returnedValue = -1;
	uint8_t value = 0;

	PCD_HardReset();
	
	value = 0x00;
	PCD_WriteRegister(TxModeReg, &value, 1);
	PCD_WriteRegister(RxModeReg, &value, 1);
	
	value = 0x26;
	PCD_WriteRegister(ModWidthReg, &value, 1);

	value = 0x80;
	PCD_WriteRegister(TModeReg, &value, 1);			
	value = 0xA9;
	PCD_WriteRegister(TPrescalerReg, &value, 1);		
	value = 0x03;
	PCD_WriteRegister(TReloadRegH, &value, 1);		
	value = 0xE8;
	PCD_WriteRegister(TReloadRegL, &value, 1);
	
	value = 0x40;
	PCD_WriteRegister(TxASKReg, &value, 1);		
	value = 0x3D;
	PCD_WriteRegister(ModeReg, &value, 1);		
	PCD_AntennaOn();

	return 0;
}


/*---------API Functions definitions-------------*/

int isCardPresent(void *uid)
{
	return 0;
}

/*---------Module Functions definitions----------*/

static ssize_t uid_show(struct class *class,
	struct class_attribute *attr, char *buf)
{	
	dev_info(dev, "sys_rfid_uid_show\n");

	return 0;
}


CLASS_ATTR_RO(uid);


static void make_sysfs_entry(struct spi_device *spidev)
{
	struct device_node *np = spidev->dev.of_node;
	const char *name;
	int res;

	struct class *sys_class;

	if (np) {

		sys_class = class_create(THIS_MODULE, DEVICE_NAME);

		if (IS_ERR(sys_class)){
			dev_err(dev, "bad class create\n");
		}
		else{
			res = class_create_file(sys_class, &class_attr_uid);


			rfid->sys_class = sys_class;
		}
	}

}



static int mfrc522_probe(struct spi_device *spidev)
{
	int initResult = 1;
	dev = &spidev->dev;

	dev_info(dev, "init SPI driver\n");


    rfid = devm_kzalloc(&spidev->dev, sizeof(struct mfrc522_data),
                        GFP_KERNEL);
    if (!rfid)
        return -ENOMEM;

    spidev->mode = SPI_MODE_0;
    spidev->bits_per_word = 8;
    spi_setup(spidev);

    rfid->spi = spidev;

    rfid->irq = devm_gpiod_get(&spidev->dev, "irq", GPIOD_IN);
    if (IS_ERR(rfid->irq)) {
        dev_err(dev, "GPIO init fault.\n");         
    }

    rfid->rst = devm_gpiod_get(&spidev->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(rfid->rst)) {
        dev_err(dev, "GPIO init fault.\n");         
    }

    initResult = gpiod_direction_input(rfid->irq);
    if (initResult)
        return initResult;

    initResult = gpiod_direction_output(rfid->rst, 1);
    if (initResult)
        return initResult;

	spi_set_drvdata(spidev, rfid);
	make_sysfs_entry(spidev);

	initResult = initMFRC522(spidev);
	if(initResult){
		dev_err(dev, "MFRC522 driver init sequence fault.\n");
	}

    dev_info(dev, "mfrc522 driver successfully loaded\n");

	return 0;
}



static int mfrc522_remove(struct spi_device *device)
{
	struct class *sys_class;

	sys_class = rfid->sys_class;

	class_remove_file(sys_class, &class_attr_uid);
	class_destroy(sys_class);

	dev_info(dev, "mfrc522 driver successfully unloaded\n");
	return 0;
}



static const struct of_device_id mfrc522_match[] = {
	{ .compatible = "China,rfid_mfrc522", },
	{ },
};
MODULE_DEVICE_TABLE(of, mfrc522_match);

static const struct spi_device_id mfrc522_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, mfrc522_id);


static struct spi_driver mfrc522_driver = {
	.driver = {
		.name	= DEVICE_NAME,
		.of_match_table = mfrc522_match,
	},
	.probe		= mfrc522_probe,
	.remove 	= mfrc522_remove,
	.id_table	= mfrc522_id,
};
module_spi_driver(mfrc522_driver);

MODULE_AUTHOR("Oleksii Klochko <lorins.dm@gmail.com>");
MODULE_DESCRIPTION("MFRC522 RFID reader driver");
MODULE_LICENSE("GPL");
