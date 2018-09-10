/*
 * GL BaseCamp ST7735 LCD driver
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
#include "st7735_def.h"


#define DEVICE_NAME	"lcd_st7735"
#define CLASS_NAME	"tft_display" 
#define BUS_NAME	"spi0"

#define ST7735_WIDTH	160
#define ST7735_HEIGHT	128


static struct device *dev;


struct st7735_data {
    struct spi_device 		*spi;
	struct class 			*sys_class;

	struct gpio_desc 		*dc; 		/* R/W operation */
    struct gpio_desc 		*rst; 		/* Display reset */

    int 					width;
    int 					height;
};

static struct st7735_data *lcd;



/*-------------Functions definitions-------------*/

static int writeCommand(struct spi_device *spidev, const int commandToWrite)
{
	int returnedValue = -1;
	gpiod_set_value(lcd->dc, DC_COMMAND);
	returnedValue = spi_write(spidev, &commandToWrite, 1);

	return returnedValue;	
}

static int writeData(struct spi_device *spidev, const int dataToWrite)
{
	int returnedValue = -1;
	gpiod_set_value(lcd->dc, DC_DATA);
	returnedValue = spi_write(spidev, &dataToWrite, 1);

	return returnedValue;
}

static int commandList(struct spi_device *spidev, const uint8_t *list) 
{
  uint8_t numCommands, numArgs;
  uint16_t ms;

  numCommands = *(list++);               // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writeCommand(spidev, *(list++));             //   Read, issue command
    numArgs  = *(list++);                //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writeData(spidev, *(list++));              //     Read, issue argument
    }

    if(ms) {
      ms = *(list++);             // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      msleep(ms);
    }
  }

  return 0;
}

static int initDisplay(struct spi_device *spidev)
{
	commandList(spidev, Rcmd1);
	commandList(spidev, Rcmd2red);
	commandList(spidev, Rcmd3);

  	//ST7735_FillScreen(0);                 // set screen to black

	return 0;
}






static ssize_t clear_show(struct class *class,
	struct class_attribute *attr, char *buf)
{	
	dev_info(dev, "sys_lcd_clear\n");

	return 0;
}


static ssize_t paint_show(struct class *class,
	struct class_attribute *attr, char *buf)
{
	dev_info(dev, "sys_lcd_paint\n");

	return 0;
}


CLASS_ATTR_RO(clear);
CLASS_ATTR_RO(paint);


static void make_sysfs_entry(struct spi_device *spidev)
{
	struct device_node *np = spidev->dev.of_node;
	const char *name;
	int res;

	struct class *sys_class;

	if (np) {

		/*
		if (!of_property_read_string(np, "label", &name))
			dev_info(dev, "label = %s\n", name);
		*/

		sys_class = class_create(THIS_MODULE, DEVICE_NAME);

		if (IS_ERR(sys_class)){
			dev_err(dev, "bad class create\n");
		}
		else{
			res = class_create_file(sys_class, &class_attr_clear);
			res = class_create_file(sys_class, &class_attr_paint);


			lcd->sys_class = sys_class;
		}
	}

}



static int st7735_probe(struct spi_device *spidev)
{
	int initResult = 1;
	dev = &spidev->dev;

	dev_info(dev, "init SPI driver\n");


    lcd = devm_kzalloc(&spidev->dev, sizeof(struct st7735_data),
                        GFP_KERNEL);
    if (!lcd)
        return -ENOMEM;

    spidev->mode = SPI_MODE_0;
    spidev->bits_per_word = 8;
    spi_setup(spidev);


    lcd->spi = spidev;

    lcd->width = ST7735_WIDTH;
    lcd->height = ST7735_HEIGHT;
    
    lcd->dc = devm_gpiod_get(&spidev->dev, "ds", GPIOD_OUT_LOW);
    if (IS_ERR(lcd->dc)) {
        dev_err(dev, "GPIO init fault.\n");         
    }

    lcd->rst = devm_gpiod_get(&spidev->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(lcd->rst)) {
        dev_err(dev, "GPIO init fault.\n");         
    }

    initResult = gpiod_direction_output(lcd->dc, 0);
    if (initResult)
        return initResult;

    initResult = gpiod_direction_output(lcd->rst, 1);
    if (initResult)
        return initResult;

	spi_set_drvdata(spidev, lcd);
	make_sysfs_entry(spidev);

	initResult = initDisplay(spidev);
	if(initResult){
		dev_err(dev, "Display init sequence fault.\n");
	}

    dev_info(dev, "st7735 driver successfully loaded\n");

	return 0;
}



static int st7735_remove(struct spi_device *device)
{
	struct class *sys_class;

	sys_class = lcd->sys_class;

	class_remove_file(sys_class, &class_attr_clear);
	class_remove_file(sys_class, &class_attr_paint);
	class_destroy(sys_class);

	dev_info(dev, "Goodbye, world!\n");
	return 0;
}



static const struct of_device_id st7735_match[] = {
	{ .compatible = "China,lcd_st7735", },
	{ },
};
MODULE_DEVICE_TABLE(of, st7735_match);

static const struct spi_device_id st7735_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, st7735_id);


static struct spi_driver st7735_driver = {
	.driver = {
		.name	= DEVICE_NAME,
		.of_match_table = st7735_match,
	},
	.probe		= st7735_probe,
	.remove 	= st7735_remove,
	.id_table	= st7735_id,
};
module_spi_driver(st7735_driver);

MODULE_AUTHOR("Oleksii Klochko <lorins.dm@gmail.com>");
MODULE_DESCRIPTION("ST7735 1.8\" LCD driver");
MODULE_LICENSE("GPL");
