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
#include <linux/sched.h> 
#include <linux/kthread.h>
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


#define DEBUG_VAR(a) printk(KERN_INFO "DEBUG_VAR %d", a);
#define DEBUG_VAR_HEX(a) printk(KERN_INFO "DEBUG_VAR %X", a);
#define DEBUG_FN(a) printk(KERN_INFO "------------------ \nIn function %s", a);
#define DEBUG_TEXT(a) printk(KERN_INFO "Message is: %s", a);
#define DEBUG_ERROR(a) printk(KERN_ERR "%s", a);

static struct device *dev;
static struct task_struct *kthread;

struct mfrc522_data {
    struct spi_device 		*spi;
	struct class 			*sys_class;

	struct gpio_desc 		*irq; 		/* MFRC522 interrupt pin */
    struct gpio_desc 		*rst; 		/* MFRC522 reset pin */

	CardUid_t 				uid;

	uint8_t 				readCard[4];
	uint8_t					isThereCard;
};

static struct mfrc522_data *rfid;



/*-------------Functions definitions-------------*/

static int PCD_WriteOneRegister(uint8_t *reg, uint8_t *buff)
{
	int returnedValue = -1;
	//DEBUG_FN("PCD_WriteRegister");
	if (reg == NULL || buff == NULL)
	{
		printk(KERN_ERR "Pointer to NULL in function PCD_ReadOneRegister.\n");
		returnedValue = -1;
	}
	else
	{
		uint8_t tmpValue[2] = {*reg, *buff};
		returnedValue = spi_write(rfid->spi, tmpValue, 2);
		//returnedValue = spi_w8r8(rfid->spi, *buff);
	}
	return (returnedValue < 0) ? returnedValue : 0;
}


static int PCD_WriteRegister(uint8_t *reg, uint8_t *buff, uint8_t length)
{
	int returnedValue = -1;
	//DEBUG_FN("PCD_WriteRegister");
	if (reg == NULL || buff == NULL)
	{
		printk(KERN_ERR "Pointer to NULL in function PCD_ReadOneRegister.\n");
		returnedValue = -1;
	}
	else
	{
		uint8_t tmpValue[length + 1];
		tmpValue[0] = *reg;
		uint8_t i;
		for (i = 1; i< length + 1; i++)
		{
			tmpValue[i] = buff[i - 1];
		}

		//returnedValue = spi_write(rfid->spi, reg, 1);
		returnedValue = spi_write(rfid->spi, tmpValue, length + 1);
	}

	return returnedValue;
}


static int PCD_ReadOneRegister(uint8_t *reg, uint8_t *buff)
{
	int returnedValue = -1;
	//DEBUG_FN("PCD_ReadOneRegister");
	if (reg == NULL || buff == NULL)
	{
		printk(KERN_ERR "Pointer to NULL in function PCD_ReadOneRegister.\n");
		returnedValue = -1;
	}
	else
	{
		
		//DEBUG_VAR_HEX(*reg);

		uint8_t address = 0x80 | *reg;
		uint8_t tmpValue[2];

		tmpValue[0] = address;
		tmpValue[1] = 0x00; 

		//DEBUG_TEXT("Address is :")
		//DEBUG_VAR_HEX(address);


		returnedValue = spi_w8r16(rfid->spi, address);
		//*buff = spi_w8r8(rfid->spi, address);


		/*returnedValue = spi_write(rfid->spi, &address, 1);
		if (returnedValue < 0)
		{
			DEBUG_ERROR("Error when read");
			return returnedValue;
		}
		
		//  *buff = spi_w8r8(rfid->spi, 0);
		returnedValue = spi_read(rfid->spi, buff, 1);*/
		//DEBUG_TEXT("First read :");
		//DEBUG_VAR_HEX(*buff);

		if (returnedValue < 0)
		{
			DEBUG_ERROR("Error when read");
			return returnedValue;
		}

		*buff = (uint8_t)(returnedValue&0x00ff);

		//DEBUG_TEXT("Readed value :");
		//DEBUG_VAR_HEX(*buff);
		returnedValue = 0;
	}
	return returnedValue;
}

static int PCD_ReadRegister(uint8_t *reg, uint8_t *buff, uint8_t length, uint8_t rxAlign)
{
	int returnedValue = -1;
	//DEBUG_FN("PCD_ReadRegister");
	if (reg == NULL || buff == NULL)
	{
		printk(KERN_ERR "Pointer to NULL in function PCD_ReadRegister.\n");
		returnedValue = -1;
	}
	else
	{
		uint8_t address = 0x80 | *reg;

		uint8_t index = 0;
		
		returnedValue = spi_write(rfid->spi, &address, 1);
		if (length > 1)
		{
			/*length = length - 1;
			if (rxAlign)
			{
				uint8_t mask = (0xFF << rxAlign) & 0xFF;
				int value = spi_w8r8(rfid->spi, address); //TODO: Error detection
				buff[0] = (buff[0] & ~mask) | ((uint8_t)value & mask);
				index = index + 1;
			}

			//spi_read(rfid->spi, &buff[index], length);
			while (index < length)
			{
				//TODO: Error detection
				buff[index] = spi_w8r8(rfid->spi, address);
				index = index + 1;
			}

			buff[length] = spi_w8r8(rfid->spi, 0);*/
			//returnedValue = spi_read(rfid->spi, buff[index], 1);


			uint8_t tmpValue[length];
			int i = 0;
			for(i=0; i< length; i++)
			{
				tmpValue[i] = address;
			}

			tmpValue[length - 1] = 0x00;
			struct spi_transfer t = {
				.tx_buf = tmpValue,
				.rx_buf = buff,
				.len = length
			};

			spi_sync_transfer(rfid->spi, &t, 1);

			if(rxAlign)
			{
				uint8_t mask = (0xFF << rxAlign) & 0xFF;
				//int value = spi_w8r8(rfid->spi, address); //TODO: Error detection
				buff[0] = (0x00 & ~mask) | (buff[0] & mask);
				//index = index + 1;
			}


		}
		else
		{
			returnedValue = spi_read(rfid->spi, buff, 1);
		}
	}	
	
	return returnedValue;
}

static int PCD_SetRegisterBitMask(uint8_t *reg, uint8_t mask)
{
	//DEBUG_FN("PCD_SetRegisterBitMask:");
	int returnedValue = -1;
	uint8_t tmpValue;

	returnedValue = PCD_ReadOneRegister(reg, &tmpValue);
	//returnedValue = PCD_ReadRegister(reg, &tmpValue, 1,0);
	//printk(KERN_INFO "Readed tmpValue: %d", tmpValue);
	tmpValue = tmpValue | mask;
	//printk(KERN_INFO "tmpValue after mask: %d", tmpValue);
	returnedValue = PCD_WriteOneRegister(reg, &tmpValue);
	//returnedValue = PCD_WriteRegister(reg, &tmpValue, 1);

	//printk(KERN_INFO "Exiting PCD_SetRegisterBitMask with %d", returnedValue);
	return returnedValue;
}

static int PCD_ClearRegisterBitMask(uint8_t *reg, uint8_t mask)
{

	//DEBUG_FN("PCD_ClearRegisterBitMask:");
	int returnedValue = -1;
	uint8_t tmpValue;

	returnedValue = PCD_ReadOneRegister(reg, &tmpValue);
	//returnedValue = PCD_ReadRegister(reg, &tmpValue, 1, 0);
	//printk(KERN_INFO "Readed tmpValue: %d", tmpValue);
	tmpValue = tmpValue & (~mask);
	//printk(KERN_INFO "tmpValue after mask: %d", tmpValue);
	returnedValue = PCD_WriteOneRegister(reg, &tmpValue);
	//returnedValue = PCD_WriteRegister(reg, &tmpValue, 1);
	//printk(KERN_INFO "Exiting PCD_ClearRegisterBitMask with %d", returnedValue);	

	return returnedValue;
}

static int PCD_HardReset(void)
{
	int returnedValue = -1;

	gpiod_set_value(rfid->rst, 0);
	udelay(5);
	gpiod_set_value(rfid->rst, 1);

	msleep(50);

	returnedValue = 0;
	return returnedValue;
}

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
static int PCD_SReset(void) 
{
	int errorCode = -1;
	uint8_t tmpValue;
	
	errorCode = PCD_WriteOneRegister(&PCD_Registers[CommandReg], &PCD_Commands[PCD_SoftReset]);	// Issue the SoftReset command.
	
	uint8_t count = 0;
	do {
		msleep(50);
		errorCode = PCD_ReadOneRegister(&PCD_Registers[CommandReg], &tmpValue);
		if (errorCode)
		{
			DEBUG_ERROR("Exit PCD_SReset - SPI Register Read error");
			return errorCode;
		}
	} while ((tmpValue & (1 << 4)) && (++count) < 3);
	return errorCode;
} // End PCD_Reset()

static int PCD_AntennaOn(void)
{
	int returnedValue = -1;
	uint8_t value = 0;

	returnedValue = PCD_ReadRegister(&PCD_Registers[TxControlReg], &value, 1, 0);
	if ((value & 0x03) != 0x03) {
		uint8_t tmpValue = value | 0x03;
		returnedValue = PCD_WriteRegister(&PCD_Registers[TxControlReg], &tmpValue, 1);
	}

	return returnedValue;
}

static int PCD_AntennaOff(void)
{
	int returnedValue = -1;

	returnedValue = PCD_ClearRegisterBitMask(&PCD_Registers[TxControlReg], 0x03);

	return returnedValue;
}

/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CalculateCRC(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
								uint8_t length,		///< In: The number of bytes to transfer.
								uint8_t *result )	///< Out: Pointer to result buffer. Result is written to result[0..1], low uint8_t first.
{
	uint8_t tmpValue = 0x00;

	PCD_WriteRegister(&PCD_Registers[CommandReg], &PCD_Commands[PCD_Idle], 1);		// Stop any active command.
	tmpValue = 0x04;
	PCD_WriteRegister(&PCD_Registers[DivIrqReg], &tmpValue, 1);				// Clear the CRCIRq interrupt request bit
	tmpValue = 0x80;
	PCD_WriteRegister(&PCD_Registers[FIFOLevelReg], &tmpValue, 1);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(&PCD_Registers[FIFODataReg], data, length);	// Write data to the FIFO
	PCD_WriteRegister(&PCD_Registers[CommandReg], &PCD_Commands[PCD_CalcCRC], 1);		// Start the calculation
	
	uint16_t i = 5000;
	//msleep(90);
	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
	for (; i > 0; i--) {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		//tmpValue = 0;
		PCD_ReadRegister(&PCD_Registers[DivIrqReg], &tmpValue, 1, 0);
		if (tmpValue & 0x04) {									// CRCIRq bit set - calculation done
			PCD_WriteRegister(&PCD_Registers[CommandReg], &PCD_Commands[PCD_Idle], 1);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			//result[0] = 
			PCD_ReadRegister(&PCD_Registers[CRCResultRegL], result, 1, 0);
			//result[1] =
			PCD_ReadRegister(&PCD_Registers[CRCResultRegH], &result[1], 1, 0);
			return STATUS_OK;
		}

		udelay(5);
	}
	// 89ms passed and nothing happend. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()







/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_TransceiveData(	uint8_t *sendData,			///< Pointer to the data to transfer to the FIFO.
								uint8_t sendLen,			///< Number of bytes to transfer to the FIFO.
								uint8_t *backData,			///< NULL or pointer to buffer if data should be read back after executing the command.
								uint8_t *backLen,			///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
								uint8_t *validBits,		///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default NULL.
								uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
								uint8_t checkCRC )		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
{
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(&PCD_Commands[PCD_Transceive], waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()


///status = PCD_TransceiveData(command, 1, bufferATQA, bufferSize, &validBits, 0, 0);

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CommunicateWithPICC(	uint8_t *command,			///< The command to execute. One of the PCD_Command enums.
									uint8_t waitIRq,			///< The bits in the ComIrqReg register that signals successful completion of the command.
									uint8_t *sendData,			///< Pointer to the data to transfer to the FIFO.
									uint8_t sendLen,			///< Number of bytes to transfer to the FIFO.
									uint8_t *backData,	///< NULL or pointer to buffer if data should be read back after executing the command.
									uint8_t *backLen,	///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
									uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
									uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
									uint8_t checkCRC )		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
{

	//DEBUG_FN("PCD_CommunicateWithPICC");

/*	printk(KERN_INFO "INPUT Parameters:");
	printk(KERN_INFO "command: %X", *command);
	printk(KERN_INFO "waitIRq: %X", waitIRq);
	printk(KERN_INFO "sendData: %X", *sendData);
	printk(KERN_INFO "sendLen: %X", sendLen);
	printk(KERN_INFO "command: %X", *command);*/

	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	uint8_t tmpValue = 0x00;
	int errorCode = -1;

	

	PCD_WriteOneRegister(&PCD_Registers[CommandReg], &PCD_Commands[PCD_Idle]);			// Stop any active command.
	tmpValue = 0x7F;
	PCD_WriteOneRegister(&PCD_Registers[ComIrqReg], &tmpValue);					// Clear all seven interrupt request bits
	tmpValue = 0x80;
	PCD_WriteOneRegister(&PCD_Registers[FIFOLevelReg], &tmpValue);				// FlushBuffer = 1, FIFO initialization
	PCD_WriteOneRegister(&PCD_Registers[CommandReg], &PCD_Commands[PCD_Idle]);
	PCD_WriteRegister(&PCD_Registers[FIFODataReg], sendData, sendLen);	// Write sendData to the FIFO
	PCD_WriteOneRegister(&PCD_Registers[BitFramingReg], &bitFraming);		// Bit adjustments
	PCD_WriteOneRegister(&PCD_Registers[CommandReg], command);				// Execute the command
	if (*command == PCD_Commands[PCD_Transceive]) {
		tmpValue = 0x80;
		//DEBUG_TEXT("+++++++++++HERE IS JOHNNY++++++++++++");
		PCD_SetRegisterBitMask(&PCD_Registers[BitFramingReg], 0x80);	// StartSend=1, transmission of data starts
	}

	
	
	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit
	uint16_t i;
	uint8_t tmpBuff = 0;
	PCD_ReadOneRegister(&PCD_Registers[VersionReg], &tmpBuff);
	//printk(KERN_INFO "But before - readed value: %X", tmpBuff);
	for (i = 5000; i > 0; i--) {
		
		PCD_ReadOneRegister(&PCD_Registers[ComIrqReg], &tmpBuff);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		//printk(KERN_INFO "Readed value tmpBuff: %d", tmpBuff);
		if (tmpBuff & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (tmpBuff & 0x01) {						// Timer interrupt - nothing received in 25ms
			//DEBUG_ERROR("Exit from PCD_CommunicateWithPICC - timer interrupt.");
			return STATUS_TIMEOUT;
		}
		//msleep(10);
		udelay(10);
	}
	// 35.7ms and nothing happend. Communication with the MFRC522 might be down.
	if (i == 0) {
		DEBUG_ERROR("Exit from PCD_CommunicateWithPICC - timeout.");
		PCD_ReadOneRegister(&PCD_Registers[VersionReg], &tmpBuff);
		//printk(KERN_INFO "But before - readed value: %X", tmpBuff);
		//PCD_SReset();
		return STATUS_TIMEOUT;
	}
	
	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = 0;
	PCD_ReadRegister(&PCD_Registers[ErrorReg], &errorRegValue, 1, 0); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		DEBUG_ERROR("Exit from PCD_CommunicateWithPICC - BufferOvfl, ParityErr or ProtocolErr Error");
		return STATUS_ERROR;
	}
  
	uint8_t _validBits = 0;
	
	//printk(KERN_INFO "backlen before read: %d", *backLen);

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		uint8_t resLen = 0;
		errorCode = PCD_ReadOneRegister(&PCD_Registers[FIFOLevelReg], &resLen);	// Number of bytes in the FIFO
		if (errorCode)
		{
			DEBUG_ERROR("Exit from PCD_CommunicateWithPICC - error while read resLen value");
			return STATUS_ERROR;
		}
		if (resLen > *backLen) {
			DEBUG_ERROR("Exit from PCD_CommunicateWithPICC - length of data bigger than value backlen");
			return STATUS_NO_ROOM;
		}
		*backLen = resLen;											// Number of bytes returned
		PCD_ReadRegister(&PCD_Registers[FIFODataReg], backData, resLen, rxAlign);	// Get received data from FIFO
		_validBits = 0;
		PCD_ReadRegister(&PCD_Registers[ControlReg], &_validBits, 1, 0); 
		_validBits = _validBits & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}

	//printk(KERN_INFO "backlen after read: %d", *backLen);
	
	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		DEBUG_ERROR("Exit from PCD_CommunicateWithPICC - collision error");
		return STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			DEBUG_ERROR("Exit from PCD_CommunicateWithPICC - Mifare classic NAK is not ok");
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
		if (*backLen < 2 || _validBits != 0) {
			DEBUG_ERROR("Exit from PCD_CommunicateWithPICC - CRC WRONG");
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			DEBUG_ERROR("Exit from PCD_CommunicateWithPICC - status != STATUS_OK");
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			DEBUG_ERROR("Exit from PCD_CommunicateWithPICC - CRC WRONG");
			return STATUS_CRC_WRONG;
		}
	}
	
	//DEBUG_TEXT("Exit from PCD_CommunicateWithPICC - Statuc OK");
	return STATUS_OK;
} // End PCD_CommunicateWithPICC()




StatusCode PICC_RequestA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							uint8_t *bufferSize	)	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
{
	return PICC_REQA_or_WUPA(&PICC_Commands[PICC_CMD_REQA], bufferATQA, bufferSize);
} // End PICC_RequestA()


StatusCode PICC_WakeupA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							uint8_t *bufferSize	)	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
{
	return PICC_REQA_or_WUPA(&PICC_Commands[PICC_CMD_WUPA], bufferATQA, bufferSize);
} 


StatusCode PICC_REQA_or_WUPA(	uint8_t *command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
								uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
								uint8_t *bufferSize	)	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
{
	//DEBUG_FN("PICC_REQA_or_WUPA");

	uint8_t validBits;
	StatusCode status;
		
	if (bufferATQA == NULL || *bufferSize < 2) 
	{	// The ATQA response is 2 bytes long.
		DEBUG_ERROR("Exit from PICC_REQA_or_WUPA");
		return STATUS_NO_ROOM;
	}
	//printk(KERN_INFO "bufferSize: %d", *bufferSize);

	PCD_ClearRegisterBitMask(&PCD_Registers[CollReg], 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(command, 1, bufferATQA, bufferSize, &validBits, 0, 0);

	//printk(KERN_INFO "Value of status: %d", status);

	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		printk(KERN_ERR "Exit from PICC_REQA_or_WUPA with error - bufferSize = %d, validbits = %d", *bufferSize, validBits);
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()

StatusCode PICC_Select(	CardUid_t *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
						uint8_t validBits)		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
{
	uint8_t uidComplete;
	uint8_t selectDone;
	uint8_t useCascadeTag;
	uint8_t cascadeLevel = 1;
	StatusCode result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9] = {0};					// The SELECT/ANTICOLLISION commands uses a 7 uint8_t standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted uint8_t. 
	uint8_t *responseBuffer;
	uint8_t responseLength;
	uint8_t tmpValue;
	//DEBUG_FN("PICC_Select");

	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		DEBUG_ERROR("Exiting PICC_Select -  with STATUS_INVALID");
		return STATUS_INVALID;
	}
	
	// Prepare MFRC522
	PCD_ClearRegisterBitMask(&PCD_Registers[CollReg], 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_Commands[PICC_CMD_SEL_CL1];
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_Commands[PICC_CMD_SEL_CL2];
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_Commands[PICC_CMD_SEL_CL3];
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				DEBUG_ERROR("Exiting PICC_Select -  error with STATUS_INTERNAL_ERROR")
				return STATUS_INTERNAL_ERROR;
				break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_Commands[PICC_CMD_CT];
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					DEBUG_ERROR("Exiting PICC_Select -  error with CRC");
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			tmpValue = (rxAlign << 4) + txLastBits;
			PCD_WriteOneRegister(&PCD_Registers[BitFramingReg], &tmpValue);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, 0);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				uint8_t valueOfCollReg; 
				PCD_ReadRegister(&PCD_Registers[CollReg], &valueOfCollReg, 1, 0); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					DEBUG_ERROR("Exiting PICC_Select -  with STATUS_COLLISION");
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					DEBUG_ERROR("Exiting PICC_Select -  with STATUS_INTERNAL_ERROR");
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First uint8_t is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 uint8_t + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			DEBUG_ERROR("Exiting PICC_Select - STATUS NOT OK with CRC");
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			DEBUG_ERROR("Exiting PICC_Select -  with STATUS_CRC_WRONG");
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()


/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
StatusCode PICC_HaltA(void) 
{
	//DEBUG_FN("PICC_HaltA");
	StatusCode result;
	uint8_t buffer[4];
	
	// Build command buffer
	buffer[0] = PICC_Commands[PICC_CMD_HLTA];
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	
	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0, NULL, 0, 0);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()


/**
 * Performs a self-test of the MFRC522
 * See 16.1.1 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * 
 * @return Whether or not the test passed. Or false if no firmware reference is available.
 */
int PCD_PerformSelfTest(void) 
{
	uint8_t tmpValue;//[4] = {0x80, 0x09, 0x00, 0x00};
	int errorCode = -1;
	// This follows directly the steps outlined in 16.1.1
	// 1. Perform a soft reset.
	PCD_SReset();
	
	// 2. Clear the internal buffer by writing 25 bytes of 00h
	uint8_t ZEROES[25] = {0x00};
	tmpValue = 0x80;
	PCD_WriteRegister(&PCD_Registers[FIFOLevelReg], &tmpValue, 1);		// flush the FIFO buffer
	PCD_WriteRegister(&PCD_Registers[FIFODataReg], ZEROES, 25);	// write 25 bytes of 00h to FIFO
	PCD_WriteRegister(&PCD_Registers[CommandReg], &PCD_Commands[PCD_Mem], 1);		// transfer to internal buffer
	
	// 3. Enable self-test
	tmpValue = 0x09;
	PCD_WriteRegister(&PCD_Registers[AutoTestReg], &tmpValue, 1);
	
	// 4. Write 00h to FIFO buffer
	tmpValue = 0x00;
	PCD_WriteRegister(&PCD_Registers[FIFODataReg], &tmpValue, 1);
	
	// 5. Start self-test by issuing the CalcCRC command
	PCD_WriteRegister(&PCD_Registers[CommandReg], &PCD_Commands[PCD_CalcCRC], 1);
	
	// 6. Wait for self-test to complete
	uint8_t n;
	uint8_t i = 0;
	for (; i < 0xFF; i++) {
		// The datasheet does not specify exact completion condition except
		// that FIFO buffer should contain 64 bytes.
		// While selftest is initiated by CalcCRC command
		// it behaves differently from normal CRC computation,
		// so one can't reliably use DivIrqReg to check for completion.
		// It is reported that some devices does not trigger CRCIRq flag
		// during selftest.
		PCD_ReadOneRegister(&PCD_Registers[FIFOLevelReg], &n);
		if (n >= 64) {
			break;
		}
	}
	PCD_WriteRegister(&PCD_Registers[CommandReg], &PCD_Commands[PCD_Idle], 1);		// Stop calculating CRC for new content in the FIFO.
	
	// 7. Read out resulting 64 bytes from the FIFO buffer.
	uint8_t result[64];
	PCD_ReadRegister(&PCD_Registers[FIFODataReg], result, 64, 0);
	
	// Auto self-test done
	// Reset AutoTestReg register to be 0 again. Required for normal operation.
	tmpValue = 0x00;
	PCD_WriteRegister(&PCD_Registers[AutoTestReg], &tmpValue, 1);
	
	// Determine firmware version (see section 9.3.4.8 in spec)
/*	printk(KERN_INFO "First 8 readed bytes:\n %X %X %X %X %X %X %X %X ", \
			result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7]);
	printk(KERN_INFO "Second 8 readed bytes:\n %X %X %X %X %X %X %X %X ", \
			result[8], result[9], result[10], result[11], result[12], result[13], result[14], result[15]);*/

	for(i=0; i < 64	; i++)
	{
		if(result[i] != MFRC522_firmware_referenceV2_0[i])
		{
			DEBUG_ERROR("============SelfTEST not passed============");
			return -1;
		}
	}
	DEBUG_TEXT("============SelfTEST is passed============");
	
	// Test passed; all is good.
	return 0;
} // End PCD_PerformSelfTest()




/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return int
 */
int PICC_IsNewCardPresent() 
{
	uint8_t bufferATQA[2] = {0, 0};
	uint8_t bufferSize = sizeof(bufferATQA);
	uint8_t tmpValue = 0x00;
	int errorCode;
	StatusCode result;

	//DEBUG_FN("PICC_IsNewCardPresent");
	// Reset baud rates
	errorCode = PCD_WriteRegister(&PCD_Registers[TxModeReg], &tmpValue, 1);
	if(errorCode)
	{
		DEBUG_ERROR("Error while transfer");
		return errorCode;
	}

	errorCode = PCD_WriteRegister(&PCD_Registers[RxModeReg], &tmpValue, 1);
	if(errorCode)
	{
		DEBUG_ERROR("Error while transfer");
		return errorCode;
	}
	// Reset ModWidthReg
	tmpValue = 0x26;
	errorCode = PCD_WriteRegister(&PCD_Registers[ModWidthReg], &tmpValue, 1);
	if(errorCode)
	{
		DEBUG_ERROR("Error while transfer");
		return errorCode;
	}

	result = PICC_RequestA(bufferATQA, &bufferSize);
	//printk(KERN_INFO "Returned value by PICC_RequestA: %d", result);
	//PICC_HaltA();
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return uint8_t
 */
int PICC_ReadCardSerial(void) 
{
	StatusCode result = PICC_Select(&(rfid->uid), 0);
	return (result == STATUS_OK);
} 




	

int getID(void) 
{
	int returnedValue = -1;	
	//DEBUG_FN("getID");

	// Getting ready for Reading PICCs
	if ( ! PICC_IsNewCardPresent()) 
	{ //If a new PICC placed to RFID reader continue
		return -1;
	}
	//DEBUG_TEXT("!!!!!!!!!!!!!!!!PICC_isCardPresent passed!!!!!!!!!!!!!!!!!!!!!!!!");
	if ( ! PICC_ReadCardSerial()) 
	{   //Since a PICC placed get Serial and continue
		return -1;
	}
	//DEBUG_TEXT("!!!!!!!!!!!!!!!!PICC_ReadCardSerial passed!!!!!!!!!!!!!!!!!!!!!!!");
	// There are Mifare PICCs which have 4 uint8_t or 7 uint8_t UID care if you use 7 uint8_t PICC
	// I think we should assume every PICC as they have 4 uint8_t UID
	// Until we support 7 uint8_t PICCs
	uint8_t i = 0;
	for (; i < 4; i++) {  //
		rfid->readCard[i] = rfid->uid.uidByte[i];
	}
	printk(KERN_INFO "UID bytes is %X %X %X %X",rfid->readCard[0],rfid->readCard[1],rfid->readCard[2],rfid->readCard[3]);

	//rfid->readCard[0] = 1;
	PICC_HaltA(); // Stop reading

	returnedValue = 0;

	return returnedValue;
}


static int initMFRC522(struct spi_device *spidev)
{
	int errorCode = -1;
	uint8_t value = 0;
	uint8_t versionCode = 0;

	PCD_PerformSelfTest();

	errorCode = PCD_HardReset();
	//errorCode = PCD_SReset();
	if(errorCode)
	{
		DEBUG_TEXT("Error while reset");
		return errorCode;
	}
	printk(KERN_INFO "==============================================\
					\n\n\t\t INITIALIZATION CARD DRIVER \n\n =============================================");


	value = 0x00;
	errorCode = PCD_WriteRegister(&PCD_Registers[TxModeReg], &value, 1);
	if(errorCode)
	{
		DEBUG_TEXT("Error while transfer");
		return errorCode;
	}

	errorCode = PCD_WriteRegister(&PCD_Registers[RxModeReg], &value, 1);
	if(errorCode)
	{
		DEBUG_TEXT("Error while transfer");
		return errorCode;
	}

	value = 0x26;
	errorCode = PCD_WriteRegister(&PCD_Registers[ModWidthReg], &value, 1);
	if(errorCode)
	{
		DEBUG_TEXT("Error while transfer");
		return errorCode;
	}

	value = 0x80;
	errorCode = PCD_WriteRegister(&PCD_Registers[TModeReg], &value, 1);			
	if(errorCode)
	{
		DEBUG_TEXT("Error while transfer");
		return errorCode;
	}
	value = 0xA9;
	errorCode = PCD_WriteRegister(&PCD_Registers[TPrescalerReg], &value, 1);		
	if(errorCode)
	{
		DEBUG_TEXT("Error while transfer");
		return errorCode;
	}
	value = 0x03;
	errorCode = PCD_WriteRegister(&PCD_Registers[TReloadRegH], &value, 1);		
	if(errorCode)
	{
		DEBUG_TEXT("Error while transfer");
		return errorCode;
	}
	value = 0xE8;
	errorCode = PCD_WriteRegister(&PCD_Registers[TReloadRegL], &value, 1);
	if(errorCode)
	{
		DEBUG_TEXT("Error while transfer");
		return errorCode;
	}
	value = 0x40;
	errorCode = PCD_WriteRegister(&PCD_Registers[TxASKReg], &value, 1);		
	if(errorCode)
	{
		DEBUG_TEXT("Error while transfer");
		return errorCode;
	}
	value = 0x3D;
	errorCode = PCD_WriteRegister(&PCD_Registers[ModeReg], &value, 1);		
	if(errorCode)
	{
		DEBUG_TEXT("Error while transfer");
		return errorCode;
	}

	errorCode = PCD_AntennaOn();
	if(errorCode)
	{
		DEBUG_TEXT("Antenna isn't tuned on.");
		return errorCode;
	}

	errorCode = PCD_ReadOneRegister(&PCD_Registers[VersionReg], &versionCode);
	if (versionCode != 0x92)
	{
		printk(KERN_ERR "WRONG VERSION CODE!! Version = %X, but should be 0x91 or 0x92", versionCode);
		return -1;
	}

	rfid->isThereCard = 0;

/*	printk(KERN_INFO "Version of chip is: %X", versionCode);

	uint8_t tmpValue;

	PCD_ReadOneRegister(&PCD_Registers[Status1Reg], &tmpValue);

	printk(KERN_INFO "Status register 1: %d",tmpValue);

	PCD_ReadOneRegister(&PCD_Registers[Status2Reg], &tmpValue);

	printk(KERN_INFO "Status register 2: %d",tmpValue);*/	

	return 0;
}


int MainLogic(void *data){

	int isCard = 0;
	while(!kthread_should_stop())
	{
		isCard = getID();
		//DEBUG_FN("MainLogic");
		//DEBUG_VAR(isCard);
		if(!isCard && rfid->isThereCard == 0)
		{
			printk(KERN_INFO "\t\t!!!!!!CARD DETECTED!!!!!!\n");
			rfid->isThereCard = 1;
			//printk(KERN_INFO "UID bytes is %X %X %X %X",rfid->readCard[0],rfid->readCard[1],rfid->readCard[2],rfid->readCard[3]);
		}
		msleep(100);
		schedule();
	}
	return 0;
}



/*---------API Functions definitions-------------*/

int isCardPresent(uint32_t *uid)
{
	int returnedValue = -1;
	if(rfid->isThereCard)
	{
		/*uint8_t i = 0;
		for (; i < 4; i++) {  //
			uid[i] = rfid->readCard[i];
		}*/
		uint32_t tmpValue = 0;
/*		for(i = 3; i > 0; i--)
		{
			tmpValue|= rfid->readCard[i];
			tmpValue = tmpValue<<8;
		}*/
		tmpValue|= rfid->readCard[3];
		tmpValue = tmpValue<<8;
		tmpValue|= rfid->readCard[2];
		tmpValue = tmpValue<<8;
		tmpValue|= rfid->readCard[1];
		tmpValue = tmpValue<<8;
		tmpValue|= rfid->readCard[0];
		*uid = tmpValue;

		rfid->isThereCard = 0;

		returnedValue = 0;
	}

	return returnedValue;
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

	kthread = kthread_run(MainLogic, NULL, "MFRC522");
    if(kthread)
        printk(KERN_INFO "Thread created successfully\n");
    else{
        printk(KERN_ERR "Thread creation failed\n");
        return -1;
    }


    dev_info(dev, "mfrc522 driver successfully loaded\n");

	return 0;
}



static int mfrc522_remove(struct spi_device *device)
{
	struct class *sys_class;

	sys_class = rfid->sys_class;

	kthread_stop(kthread);

	class_remove_file(sys_class, &class_attr_uid);
	class_destroy(sys_class);


	dev_info(dev, "mfrc522 driver successfully unloaded\n");
	return 0;
}



static const struct of_device_id mfrc522_match[] = {
	{ .compatible = "NPX, mfrc522", },
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
