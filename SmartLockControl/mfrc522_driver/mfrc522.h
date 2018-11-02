/*
 * GL BaseCamp MFRC522 RFID driver header
 * Copyright (C) 2018 Oleksii Klochko <lorins.dm@gmail.com>
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */


// MFRC522 registers. Described in chapter 9 of the datasheet.
// When using SPI all addresses are shifted one bit left in the "SPI address uint8_t" (section 8.1.2.3)
typedef enum {
	CommandReg = 0,
	ComIEnReg,
	DivIEnReg,
	ComIrqReg,
	DivIrqReg,
	ErrorReg,
	Status1Reg,
	Status2Reg,
	FIFODataReg,
	FIFOLevelReg,
	WaterLevelReg,
	ControlReg,
	BitFramingReg,
	CollReg,
	ModeReg,
	TxModeReg,
	RxModeReg,
	TxControlReg,
	TxASKReg,
	TxSelReg,
	RxSelReg,
	RxThresholdReg,
	DemodReg,
	MfTxReg,
	MfRxReg,
	SerialSpeedReg,
	CRCResultRegH,
	CRCResultRegL,
	ModWidthReg,
	RFCfgReg,
	GsNReg,
	CWGsPReg,
	ModGsPReg,
	TModeReg,
	TPrescalerReg,
	TReloadRegH,
	TReloadRegL,
	TCounterValueRegH,
	TCounterValueRegL,
	TestSel1Reg,
	TestSel2Reg,
	TestPinEnReg,
	TestPinValueReg,
	TestBusReg,
	AutoTestReg,
	VersionReg,
	AnalogTestReg,
	TestDAC1Reg,
	TestDAC2Reg,
	TestADCReg
} PCD_Register_t;

static const uint8_t PCD_Registers[] = {
	0x01 << 1,	// starts and stops command execution
	0x02 << 1,	// enable and disable interrupt request control bits
	0x03 << 1,	// enable and disable interrupt request control bits
	0x04 << 1,	// interrupt request bits
	0x05 << 1,	// interrupt request bits
	0x06 << 1,	// error bits showing the error status of the last command executed 
	0x07 << 1,	// communication status bits
	0x08 << 1,	// receiver and transmitter status bits
	0x09 << 1,	// input and output of 64 uint8_t FIFO buffer
	0x0A << 1,	// number of uint8_ts stored in the FIFO buffer
	0x0B << 1,	// level for FIFO underflow and overflow warning
	0x0C << 1,	// miscellaneous control registers
	0x0D << 1,	// adjustments for bit-oriented frames
	0x0E << 1,	// bit position of the first bit-collision detected on the RF interface
	0x11 << 1,	// defines general modes for transmitting and receiving 
	0x12 << 1,	// defines transmission data rate and framing
	0x13 << 1,	// defines reception data rate and framing
	0x14 << 1,	// controls the logical behavior of the antenna driver pins TX1 and TX2
	0x15 << 1,	// controls the setting of the transmission modulation
	0x16 << 1,	// selects the internal sources for the antenna driver
	0x17 << 1,	// selects internal receiver settings
	0x18 << 1,	// selects thresholds for the bit decoder
	0x19 << 1,	// defines demodulator settings
	0x1C << 1,	// controls some MIFARE communication transmit parameters
	0x1D << 1,	// controls some MIFARE communication receive parameters
	0x1F << 1,	// selects the speed of the serial UART interface
	0x21 << 1,	// shows the MSB and LSB values of the CRC calculation
	0x22 << 1,
	0x24 << 1,	// controls the ModWidth setting?
	0x26 << 1,	// configures the receiver gain
	0x27 << 1,	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
	0x28 << 1,	// defines the conductance of the p-driver output during periods of no modulation
	0x29 << 1,	// defines the conductance of the p-driver output during periods of modulation
	0x2A << 1,	// defines settings for the internal timer
	0x2B << 1,	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
	0x2C << 1,	// defines the 16-bit timer reload value
	0x2D << 1,
	0x2E << 1,	// shows the 16-bit timer value
	0x2F << 1,
	0x31 << 1,	// general test signal configuration
	0x32 << 1,	// general test signal configuration
	0x33 << 1,	// enables pin output driver on pins D1 to D7
	0x34 << 1,	// defines the values for D1 to D7 when it is used as an I/O bus
	0x35 << 1,	// shows the status of the internal test bus
	0x36 << 1,	// controls the digital self-test
	0x37 << 1,	// shows the software version
	0x38 << 1,	// controls the pins AUX1 and AUX2
	0x39 << 1,	// defines the test value for TestDAC1
	0x3A << 1,	// defines the test value for TestDAC2
	0x3B << 1		// shows the value of ADC I and Q channels
};


// MFRC522 commands. Described in chapter 10 of the datasheet.
typedef enum {
	PCD_Idle = 0,
	PCD_Mem,
	PCD_GenerateRandomID,
	PCD_CalcCRC,
	PCD_Transmit,
	PCD_NoCmdChange,
	PCD_Receive,
	PCD_Transceive,
	PCD_MFAuthent,
	PCD_SoftReset
} PCD_Command_t;

static const uint8_t PCD_Commands[] = {
	0x00,		// no action, cancels current command execution
	0x01,		// stores 25 uint8_ts into the internal buffer
	0x02,		// generates a 10-uint8_t random ID number
	0x03,		// activates the CRC coprocessor or performs a self-test
	0x04,		// transmits data from the FIFO buffer
	0x07,		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
	0x08,		// activates the receiver circuits
	0x0C,		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
	0x0E,		// performs the MIFARE standard authentication as a reader
	0x0F		// resets the MFRC522
};

// Commands sent to the PICC.
typedef enum {
	PICC_CMD_REQA = 0,
	PICC_CMD_WUPA,
	PICC_CMD_CT,
	PICC_CMD_SEL_CL1,
	PICC_CMD_SEL_CL2,
	PICC_CMD_SEL_CL3,
	PICC_CMD_HLTA,
	PICC_CMD_RATS,
	PICC_CMD_MF_AUTH_KEY_A,
	PICC_CMD_MF_AUTH_KEY_B,
	PICC_CMD_MF_READ,
	PICC_CMD_MF_WRITE,
	PICC_CMD_MF_DECREMENT,
	PICC_CMD_MF_INCREMENT,
	PICC_CMD_MF_RESTORE,
	PICC_CMD_MF_TRANSFER,
	PICC_CMD_UL_WRITE
} PICC_Command;

static const uint8_t PICC_Commands[] = {
	0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
	0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
	0x88,		// Cascade Tag. Not really a command, but used during anti collision.
	0x93,		// Anti collision/Select, Cascade Level 1
	0x95,		// Anti collision/Select, Cascade Level 2
	0x97,		// Anti collision/Select, Cascade Level 3
	0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
	0xE0,     // Request command for Answer To Reset.
	0x60,		// Perform authentication with Key A
	0x61,		// Perform authentication with Key B
	0x30,		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
	0xA0,		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
	0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
	0xC1,		// Increments the contents of a block and stores the result in the internal data register.
	0xC2,		// Reads the contents of a block into the internal data register.
	0xB0,		// Writes the contents of the internal data register to a block.
	0xA2		// Writes one 4 byte page to the PICC.
};


// PICC types we can detect. Remember to update PICC_GetTypeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
typedef enum {
	PICC_TYPE_UNKNOWN		,
	PICC_TYPE_ISO_14443_4	,	// PICC compliant with ISO/IEC 14443-4 
	PICC_TYPE_ISO_18092		, 	// PICC compliant with ISO/IEC 18092 (NFC)
	PICC_TYPE_MIFARE_MINI	,	// MIFARE Classic protocol, 320 bytes
	PICC_TYPE_MIFARE_1K		,	// MIFARE Classic protocol, 1KB
	PICC_TYPE_MIFARE_4K		,	// MIFARE Classic protocol, 4KB
	PICC_TYPE_MIFARE_UL		,	// MIFARE Ultralight or Ultralight C
	PICC_TYPE_MIFARE_PLUS	,	// MIFARE Plus
	PICC_TYPE_MIFARE_DESFIRE,	// MIFARE DESFire
	PICC_TYPE_TNP3XXX		,	// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
	PICC_TYPE_NOT_COMPLETE	= 0xff	// SAK indicates UID is not complete.
} PICC_Type;


// Return codes from the functions in this class. Remember to update GetStatusCodeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
typedef enum {
	STATUS_OK				,	// Success
	STATUS_ERROR			,	// Error in communication
	STATUS_COLLISION		,	// Collission detected
	STATUS_TIMEOUT			,	// Timeout in communication.
	STATUS_NO_ROOM			,	// A buffer is not big enough.
	STATUS_INTERNAL_ERROR	,	// Internal error in the code. Should not happen ;-)
	STATUS_INVALID			,	// Invalid argument.
	STATUS_CRC_WRONG		,	// The CRC_A does not match
	STATUS_MIFARE_NACK		= 0xff	// A MIFARE PICC responded with NAK.
} StatusCode;


// A struct used for passing the UID of a PICC.
typedef struct {
	uint8_t		size;			// Number of bytes in the UID. 4, 7 or 10.
	uint8_t		uidByte[10];
	uint8_t		sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
} CardUid_t;


static int PCD_WriteRegister(uint8_t *reg, uint8_t *buff, uint8_t length);
static int PCD_ReadRegister(uint8_t *reg, uint8_t *buff, uint8_t length, uint8_t rxAlign);
static int PCD_SetRegisterBitMask(uint8_t *reg, uint8_t mask);
static int PCD_ClearRegisterBitMask(uint8_t *reg, uint8_t mask);
static int PCD_HardReset(void);
static int PCD_AntennaOn(void);
static int PCD_AntennaOff(void);
StatusCode PCD_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result);

static int PICC_IsNewCardPresent(void);
static int PICC_ReadCardSerial(void);



StatusCode PCD_TransceiveData(uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, uint8_t checkCRC);
StatusCode PCD_CommunicateWithPICC(uint8_t *command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, uint8_t checkCRC);
StatusCode PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize);
StatusCode PICC_WakeupA(uint8_t *bufferATQA, uint8_t *bufferSize);
StatusCode PICC_REQA_or_WUPA(uint8_t *command, uint8_t *bufferATQA, uint8_t *bufferSize);
StatusCode PICC_Select(CardUid_t *uid, uint8_t validBits /*= 0*/);
StatusCode PICC_HaltA(void);

uint8_t getID(void);

static int initMFRC522(struct spi_device *spidev);