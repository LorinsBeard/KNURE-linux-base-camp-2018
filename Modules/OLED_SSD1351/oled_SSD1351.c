#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include <linux/fb.h>
#include <linux/uaccess.h>


#include "oled_SSD1351.h"


#define DRIVER_NAME			"oled_ssd1351_drv"



#define PIN_SET        1
#define PIN_RESET      0
#define CS_SET         CS_Set(PIN_RESET)
#define CS_RESET       CS_Set(PIN_SET)


static struct device *dev;

static const struct of_device_id my_drvr_match[];



typedef struct{
   u8 x;
   u8 y;
}point_t;

typedef struct {
  u16        displayUpdateBufffer[SCREEN_WIDTH * SCREEN_HEIGHT];
  point_t    startPosition;
  point_t    finishPosition;
}updateScreen_t;


typedef struct {
	struct spi_device *spi_device;

	struct class *sys_class;
	struct gpio_desc *RST_gpiod;
	struct gpio_desc *DC_gpiod;	
    struct gpio_desc *CS_gpiod;

	u16 width;
	u16 height;

    updateScreen_t  updateScreen;
}oled_Display_t;


static oled_Display_t oledDisplay;
static uint32_t pseudo_palette[16];



static void SendCommand(u8 command){
    u8 operationStatus = 0;

    gpiod_set_value(oledDisplay.DC_gpiod, SEND_COMMAND);
    operationStatus = spi_write(oledDisplay.spi_device, &command, 1);

    if(operationStatus < 0){
        dev_info(dev, "Write data %02x failed with status %d\n", command, operationStatus);
    }
}

static void SendData(u8 *data){
    u8 operationStatus = 0;

    gpiod_set_value(oledDisplay.DC_gpiod, SEND_DATA);
    operationStatus = spi_write(oledDisplay.spi_device, data, 1);

    if(operationStatus < 0){
        dev_info(dev, "Write data %02x failed with status %d\n", data, operationStatus);
    }
}

static void ResetDisplay(u8 state){
   gpiod_set_value(oledDisplay.RST_gpiod, state);
}

static void CS_Set(u8 state){
    gpiod_set_value(oledDisplay.CS_gpiod, state);
}


static void OledInit(void) {
    
    dev_info(dev, "Oled display start to init \n");


   //Initialize display process
    CS_SET;

    ResetDisplay(PIN_RESET);
    mdelay(200);
    ResetDisplay(PIN_SET);
    mdelay(200);
    

    
    SendCommand(COMMAND_COMMAND_LOCK);
    SendData((u8*)0x12);
    SendCommand(COMMAND_COMMAND_LOCK);
    SendData((u8*)0xB1);

    SendCommand(COMMAND_DISPLAY_OFF);
    SendCommand(COMMAND_CLOCK_DIV);
    SendData((u8*)0x10);

    SendCommand(COMMAND_MUXRATIO);
    SendData((u8*)0x7F);

    SendCommand(COMMAND_DISPLAY_OFFSET);
    SendData((u8*)0x00);

    SendCommand(COMMAND_STARTLINE);
    SendData((u8*)0x00);

    SendCommand(COMMAND_SET_REMAP);
    SendData((u8*)0x74);

    SendCommand(COMMAND_SET_GPIO);
    SendData((u8*)0x00);

    SendCommand(COMMAND_FUNCTION_SELECT);
    SendData((u8*)0xC1);

    SendCommand(COMMAND_SET_VSL);
    SendData((u8*)0xA0);
    SendData((u8*)0xB5);
    SendData((u8*)0x55);

    SendCommand(COMMAND_CONTRAST_ABC);
    SendData((u8*)0xC8);  //Color A //8a
    SendData((u8*)0xA8);  //Color B //51
    SendData((u8*)0xC8);  //Color C //8a

    SendCommand(COMMAND_CONTRAST_MASTER);
    SendData((u8*)0x0F);

    SendCommand(COMMAND_SETGRAY);
    u8 i;
    for(i = 5; i < 23; ++i){
        SendData(i);
    }
    SendData((u8*)0x18);
    SendData((u8*)0x1a);
    SendData((u8*)0x1b);
    SendData((u8*)0x1C);
    SendData((u8*)0x1D);
    SendData((u8*)0x1F);
    SendData((u8*)0x21);
    SendData((u8*)0x23);
    SendData((u8*)0x25);
    SendData((u8*)0x27);
    for (i = 42; i <= 72; i += 3) {
        SendData(i);
    }
    for (i = 76; i <= 120; i += 4) {
        SendData(i);
    }
    for (i = 125; i <= 180; i += 5) {
        SendData(i);
    }

    SendCommand(COMMAND_PRECHARGE);
    SendData((u8*)0x32);

    SendCommand(COMMAND_PRECHARGE_LEVEL);
    SendData((u8*)0x16);

    SendCommand(COMMAND_PRECHARGE2);
    SendData((u8*)0x01);
    SendData((u8*)0x09);
    SendData((u8*)0x0F);

    SendCommand(COMMAND_VCOMH);
    SendData((u8*)0x05);

    SendCommand(COMMAND_NORMAL_DISPLAY);

    SendCommand(COMMAND_SET_COLUMN);
    SendData((u8*)0x00);
    SendData(SCREEN_WIDTH);

    SendCommand(COMMAND_SET_ROW);
    SendData((u8*)0x00);
    SendData(SCREEN_HEIGHT);

    SendCommand(COMMAND_DISPLAY_ON);
    CS_RESET;

    dev_info(dev, "Oled display was initted \n");       
}



static int UpdateScreen(void){
	int retunedValue = 0;

/* Setting location for update */
    CS_SET;
    SendCommand(COMMAND_SET_COLUMN);
    u8 data[2] = {oledDisplay.updateScreen.startPosition.x, oledDisplay.updateScreen.finishPosition.x};
	SendData(data);
	

    SendCommand(COMMAND_SET_ROW);
    data[0] = oledDisplay.updateScreen.startPosition.y;
    data[1] = oledDisplay.updateScreen.finishPosition.y;
	SendData(data);
    

/* Transmitting data */
    u8 x;
    u8 y;
    SendCommand(COMMAND_WRITE_RAM);
    
    u8 trsm [100] = {0};
    u8 cnt = 0;
    u16 dot = 0;
    for(x = oledDisplay.updateScreen.startPosition.x; x < oledDisplay.updateScreen.finishPosition.x; ++x){
    	for(y = oledDisplay.updateScreen.startPosition.y; y < oledDisplay.updateScreen.finishPosition.y; ++y){
    		dot = *(oledDisplay.updateScreen.displayUpdateBufffer + (x * oledDisplay.width) + y);
            trsm[cnt] = (dot >> 8);
            trsm[cnt + 1] = (dot);
           
            if(cnt > 100){
                SendData(trsm);
                cnt = 0;
            }    		
            retunedValue ++;
    	}
    }

    CS_RESET;
    dev_info(dev, "update returned = %d \n", retunedValue);
	return retunedValue;
}



static int clear_show(struct class *class,
	                      struct class_attribute *attr, char *buf)
{
	int result = 0;
	dev_info(dev, "clear");

    memset(oledDisplay.updateScreen.displayUpdateBufffer, COLOR_BLACK, sizeof(oledDisplay.updateScreen.displayUpdateBufffer));
    oledDisplay.updateScreen.startPosition.x = 0;
    oledDisplay.updateScreen.startPosition.y = 0;

	oledDisplay.updateScreen.finishPosition.x = SCREEN_WIDTH -1;
	oledDisplay.updateScreen.finishPosition.y = SCREEN_HEIGHT -1;

    int res = UpdateScreen();    
    if(!res){
       result = 1;
       dev_err(dev, "Clearing operation not successfully");
    }

	return result;
}


static int paint_show(struct class *class,
	                  struct class_attribute *attr, char *buf)
{
	int result = 0;
	dev_info(dev, "paint");
	/*Test*/
//=============
	memset(oledDisplay.updateScreen.displayUpdateBufffer, COLOR_DGREEN, sizeof(oledDisplay.updateScreen.displayUpdateBufffer));
    oledDisplay.updateScreen.startPosition.x = 20;
    oledDisplay.updateScreen.startPosition.y = 20;

	oledDisplay.updateScreen.finishPosition.x = 75;
	oledDisplay.updateScreen.finishPosition.y = 75;	

//==============

    int res = UpdateScreen();    
    if(!res){
       result = 1;
       dev_err(dev, "Painting operation not successfully");
    }

	return result;
}



CLASS_ATTR_RO(clear);
CLASS_ATTR_RO(paint);


static void make_sysfs_entry(struct spi_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const char *name;
	int res;

	struct class *sys_class;

	if (np) {
		sys_class = class_create(THIS_MODULE, DRIVER_NAME);

		if (IS_ERR(sys_class)){
			dev_err(dev, "bad class create\n");
		}
		else{
			res = class_create_file(sys_class, &class_attr_clear);
			res = class_create_file(sys_class, &class_attr_paint);

			oledDisplay.sys_class = sys_class;


			dev_info(dev, "sys class created = %s\n", DRIVER_NAME);
		}
	}

}




static int get_platform_info(struct spi_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const char *name;
    u8     operationStatus = 0;
	

	if (np) {
		if (!of_property_read_string(np, "label", &name))
			dev_info(dev, "label = %s\n", name);

		 oledDisplay.RST_gpiod = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
		 if (IS_ERR(oledDisplay.RST_gpiod)) {
		 	dev_err(dev, "fail to get reset-gpios()\n");
		 	return EINVAL;
		 }
		 if(!gpiod_direction_output(oledDisplay.RST_gpiod, 1))
		 	dev_info(dev, "reset-gpios set as OUT\n");


		oledDisplay.DC_gpiod = devm_gpiod_get(dev, "dc", GPIOD_OUT_HIGH);
		if (IS_ERR(oledDisplay.DC_gpiod)) {
			dev_err(dev, "fail to get dc-gpios()\n");
			return EINVAL;
		}
		if(!gpiod_direction_output(oledDisplay.DC_gpiod, 1))
			dev_info(dev, "dc-gpios set as OUT\n");



        oledDisplay.CS_gpiod = devm_gpiod_get(dev, "cs", GPIOD_OUT_HIGH);
        if (IS_ERR(oledDisplay.CS_gpiod)) {
            dev_err(dev, "fail to get cs-gpios()\n");
            return EINVAL;
        }
        if(!gpiod_direction_output(oledDisplay.CS_gpiod, 1))
            dev_info(dev, "cs-gpios set as OUT\n");

		
		operationStatus = 1;
	}
	else{
		dev_err(dev, "failed to get device_node\n");
		return -EINVAL;
	}

	return operationStatus;
}









/*
   INITIALIZE / DEINITIALIZE FUNCTIONS
*/

static int oled_ssd1351_probe(struct spi_device *spi)
{
	const struct of_device_id *match;
	
	dev = &spi->dev;

	match = of_match_device(of_match_ptr(my_drvr_match), dev);
	if (!match) {
		dev_err(dev, "failed of_match_device()\n");
		return -EINVAL;
	}

	 spi->mode = SPI_MODE_0;
	 spi->bits_per_word = 8;
	 spi_setup(spi);


	oledDisplay.spi_device 	= spi;
	oledDisplay.width  		= SCREEN_WIDTH;
	oledDisplay.height 		= SCREEN_HEIGHT;

	dev_info(dev, "spiclk %u KHz.\n",	(spi->max_speed_hz + 500) / 1000);


	if(!get_platform_info(spi)){
		dev_err(dev, "failed to get platform info\n");
		return -EINVAL;
	}

	OledInit();
	spi_set_drvdata(spi, &oledDisplay);

	make_sysfs_entry(spi);

	dev_info(dev, "module initialized\n");
	return 0;
}



static int oled_ssd1351_remove(struct spi_device *spi)
{
	
	 class_remove_file(oledDisplay.sys_class, &class_attr_clear);
	 class_remove_file(oledDisplay.sys_class, &class_attr_paint);
	 class_destroy(oledDisplay.sys_class);

	dev_info(dev, "Module deinstalled successfully!\n");
	return 0;
}





/*=====
        CONFIGURATIONS TABLEs
=====*/

static const struct of_device_id my_drvr_match[] = {
	{ .compatible = "MaGol,oled_ssd1351", },
	{ },
};
MODULE_DEVICE_TABLE(of, my_drvr_match);



static const struct spi_device_id oled_ssd1351_spi_ids[] = {
	{ "oled_ssd1351s", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, oled_ssd1351_spi_ids);
 

static struct spi_driver my_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = my_drvr_match,
	},
	.probe 		= oled_ssd1351_probe,
	.remove 	= oled_ssd1351_remove,
	.id_table 	= oled_ssd1351_spi_ids,
};
module_spi_driver(my_driver);



MODULE_AUTHOR("MaksimHolikov, golikov.mo@gmail.com");
MODULE_DESCRIPTION("Driver to control OLED display based on ssd1351 chip");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");