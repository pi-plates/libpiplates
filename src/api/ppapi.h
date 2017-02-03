/*
 ============================================================================
 Name        : ppapi.h
 Author      : B. Eschrich
 Version     : 1.00
 Copyright   : Copyright (c) 2016-2017 by B. Eschrich (EoF)
 Description : PI-Plates RELAYplate API
 ============================================================================
 */

/*
Definitions:
    Address (addr): DAQCplates have jumpers on the board that allow
			their address to be set to a value between 0 and 7.
    ADC 	(analog to digital converter) channels can be 0 through 8 for a
			total of 9 channels. Reading channel 8 will return the power supply
			voltage.
    DIN 	(digital input) bit values can be 0 through 7 for a total of 8 bits
    DOUT 	(digital output) bit values can be 0 through 6 for a total of 7 bits
    PWM 	(pulse width modulator) channels can be 0 or 1 for a total of 2
			channels. The output values can be between 0 and 1023.
    DAC 	(digital to analog converter) channels can be 0 or 1 for a total of
			2 channels. The output value can be between 0 and 4.096 volts
    LED 	(led) values can be 0 for the red LED or 1 for the green.
 */

#ifndef _ppapi_h
#define _ppapi_h

// Maximum number of available boards per board type
#define PP_MAX_BOARD_COUNT 8

/* Using
 * 0 = /dev/spidev0.0
 * 1 = /dev/spidev0.1 */
#define PP_SPI_IO_CHANNEL 1

// generic operation delay
#define PP_DELAY 1000

// Board types
#define PP_BOARD_TYPE_RELAY 1
#define PP_BOARD_TYPE_DAQC 2
#define PP_BOARD_TYPE_MOTOR 3

// Maximum of available PI-Plates board types
#define PP_MAX_BOARD_TYPES 3

// Maximum list size of possible connected PI-Plates boards
#define PP_MAX_BOARD_LIST_SIZE (sizeof(board_t) * (PP_MAX_BOARD_COUNT * PP_MAX_BOARD_TYPES))

// PI-Plates - Maximum I/O
#define PP_MAX_RELAYS 			8
#define PP_MAX_DIGITAL_IN		8
#define PP_MAX_ANALOG_IN		8
#define PP_MAX_DIGITAL_OUT		7

// Relay or LED update modes
#define STATE_OFF	 			0x00
#define STATE_ON 				0x01
#define STATE_TOGGLE 			0x02
#define STATE_ALL				0x03

// Bit ON states
#define BIT1_STATE_ON			0x01
#define BIT2_STATE_ON			0x02
#define BIT3_STATE_ON			0x04
#define BIT4_STATE_ON			0x08
#define BIT5_STATE_ON			0x10
#define BIT6_STATE_ON			0x20
#define BIT7_STATE_ON			0x40
#define BIT8_STATE_ON			0x80

// Interrupt edge flags
#define PP_INT_EDGE_RAISE		'R'
#define PP_INT_EDGE_FALLING		'F'
#define PP_INT_EDGE_BOTH		'B'

/**
 * PI-Plates GPIO pin and board base address
 */
struct config
{
    /**
     * Digitial input pin - Signal to interrupt (BCM pin 22)
     */
    uint8_t pinInterrupt;
    /**
     * Digital output pin - Signal to control SPI bus transfer (BCM pin 25)
     */
    uint8_t pinFrameControl;
    /**
     * PI-Plates specific SPI board base address
     */
    uint8_t boardBaseAddr;
    /**
     * SPI BUS channel number (0=/dev/spidev0.0 or 1=/dev/spidev0.1)
     * Default is 1
     */
    uint8_t spiChannel;
};
typedef struct config config_t;

/**
 * PI-plates board handle structure
 */
struct board
{
    /**
     * GPIO/SPI configuration
     */
    config_t config;
    /**
     * The PI-Plates board address in range 0 to 7
     */
    uint8_t address;
    /**
     * The PI-Plates board type 1=RELAYplate 2=DAQCplate 3=MOTORplate
     */
    uint8_t type;
    /**
     * In case of the DAQCplate the Vcc value in volts
     */
    float vcc;
};
typedef struct board board_t;

/* API Version */
struct version
{
    const long major;
    const long minor;
    const long build;
    const long revision;
};
typedef struct version version_t;

/**
 * Retrieve available boards for specific board type
 * @param type One of the predefied board types (RELAY=1, DAQC=2 or MOTOR=3)
 * @param ppResult Pointer to the list. Caller must free allocated resources.
 * @return Number of boards in the list
 */
uint8_t getBoardList(const uint8_t type, board_t** ppResult);

/**
 * Retrieve count of available PI-Plates boards specified by board type
 * @param type One of the predefined board types
 * @return Number of available boards
 */
uint8_t getBoardCount(const uint8_t type);

/**
 * Retrieve a PI-Plates board handle by specified address. If the address
 * could not be found, return value is NULL
 * @param address The board address 0 to 7 (preselected by jumper on address header)
 * @return A pointer of the allocated board_t structure.
 */
board_t* getBoardByAddress(const uint8_t address);

/**
 * Print board informations for given PI-Plates board.
 * @param pBoard The board handle
 */
void printBoardInfo(const board_t* pBoard);

/**
 * Print C API version string
 */
void printAPIVersion();

/**
 * Retrieve C API version string
 */
void getAPIVersionString(char* pVersion, size_t size);

/**
 * Retrieve C API version information
 */
version_t* getAPIVersion();

/**
 * Initialize GPIO/SPI configuration structure to communicate
 * with the PI-Plates boards. Note that the pin numbers follows
 * wiringPi GPIO layout and will be translated to the Broadcom
 * pin layout. The C API library initializes the wiringPi with
 * BCM pin layout.
 * @param spiChannel Use constant PP_SPI_IO_CHANNEL for default
 * @param wpiPinINT Interrupt signal pin (wiringPi=3 BCM=22)
 * @param wpiPinFrame SPI frame signal pin (wiringPi=6 BCM=25)
 * @param boardBaseAddr The PI-Plates board address
 * @param pConfig Pointer to the configuration structure
 * @return 0 if succsess otherwise signal an error
 */
int initConfig(const uint8_t spiChannel, const uint8_t wpiPinINT, const uint8_t wpiPinFrame, const uint8_t boardBaseAddr, config_t* pConfig);

/**
 * Initialize the PI-Plate boards by specified board type. Each available board becomes
 * a board_t handle allocated in a global board list. You must call initConfig(...) first
 * to get the configuration for the board GPIO/SPI communication.
 * @param type One of the predefied board types (RELAY=1, DAQC=2 or MOTOR=3)
 * @return 0 if succsess otherwise signal an error
 */
int initBoards(uint8_t type, const config_t* pConfig);

/**
 * Enable frame signal to transmit commands to the
 * board through the SPI bus.
 * @param pBoard Handle of the PI-Plates board
 * @return 0 success otherwise signal an error
 */
int enableFrame(const board_t* pBoard);

/**
 * Disable frame signal to prevent transmission
 * through the SPI bus.
 * @param pBoard Handle of the PI-Plates board
 * @return 0 success otherwise signal an error
 */
int disableFrame(const board_t* pBoard);

/**
 * Retrieve the SPI board address. To test a valid board address
 * substract *pAddress - pBoard->config.boardBaseAddress. The result
 * must the same as pBoard->address. That indicate that the given board
 * address in the address field of the board_t structure is valid.
 * @param pBoard Handle of the PI-Plates board
 * @param pAddress Pointer to retrieve the SPI board address
 * @return 0 success otherwise signal an error
 */
int getAddress(const board_t* pBoard, uint8_t* pData);

/**
 * Reset the RELAYplate board and switch all relays off
 */
int reset(const board_t* pBoard);

/**
 * Return HW revision in byte format
 */
int getHWRevision(const board_t* pBoard, char* pData, const size_t size);

/**
 * Return FW revision in byte format
 */
int getFWRevision(const board_t* pBoard, char* pData, const size_t size);

/**
 * Return Pi-Plate descriptor string
 */
int getID(const board_t* pBoard, char* pData, const size_t size);

/**
 *
 */
int getProgMemory(const board_t* pBoard, const uint32_t address, char* pData, const size_t size);

/**
 * Update the state of the board LED
 */
int updateLED(const board_t* pBoard, const uint8_t led, const uint8_t state);

/**
 * Return the state of the board LED
 */
int getLEDState(const board_t* pBoard, const uint8_t led, uint8_t* pData);

/********************************************************************
 RELAYplate specific functions
 ********************************************************************/

/**
 * Update the relay state. Parameter state can be value of:
 * 	 STATE_OFF		Single relay off
 * 	 STATE_ON		Single relay on
 * 	 STATE_TOGGLE   Toggle single relay
 *	 STATE_ALL		Update all relays at once
 */
int updateRelay(const board_t* pBoard, const uint8_t relay, const uint8_t state);

/**
 * Switch a single relay on
 */
int relayON(const board_t* pBoard, const uint8_t relay);

/**
 * Switch a single relay off
 */
int relayOFF(const board_t* pBoard, const uint8_t relay);

/**
 * Toggle a single relay
 */
int toggleRelay(const board_t* pBoard, const uint8_t relay);

/**
 * Update the relay states at once.
 */
int updateRelays(const board_t* pBoard, const uint8_t mask);

/**
 * Get all relay states
 */
int getRelayState(const board_t* pBoard, uint8_t* pData);

/********************************************************************
 DAQCplate specific functions
 ********************************************************************/

/**
 *
 */
int getProgMemory(const board_t* pBoard, const uint32_t address, char* pData, const size_t size);

/**
 * Return voltage from single channel
 */
int getADC(const board_t* pBoard, const uint8_t channel, float* pData);

/**
 * Return voltages from all channels
 */
int getADCall(const board_t* pBoard, float data[], const size_t size);

/**
 * Return single bit value
 */
int getDINbit(const board_t* pBoard, const uint8_t bit, uint8_t* pData);

/**
 * Return all eight bits
 */
int getDINall(const board_t* pBoard, uint8_t* pData);

/**
 * Enable interrupts for an input change on the specified bit.
 * The "edge" value can be 'r' for rising, 'f' for falling,
 * or 'b' for both.
 */
int enableDINint(const board_t* pBoard, const uint8_t bit, const unsigned char edge);

/**
 * Disable interrupts on the specified bit
 */
int disableDINint(const board_t* pBoard, const uint8_t bit);

/**
 * Reads distance from an HC-SR04 ultrasonic sensor. The channel argument
 * calls out the value of the digital input / digital output pair required
 * to interface to the sensor. The units can 'i' for inches or 'c'
 * for centimetres.
 */
int getRange(const board_t* pBoard, const uint8_t channel, const unsigned char units, float* pData);

/**
 * Returns current state of on board switch. A value of 1 is
 * returned when the switch is up and a value of 0 is returned
 * when it's down.
 */
int getSWstate(const board_t* pBoard, uint8_t* pData);

/**
 * Allows the switch to generate an interrupts when pressed.
 * Global interrupts must be enabled before using this function.
 */
int enableSWint(const board_t* pBoard);

/**
 * Blocks push button on board from generating an interrupt.
 */
int disableSWint(const board_t* pBoard);

/**
 * Pushing button on board will short RPI GPIO23 to
 * GND and then remove 5VDC 45 seconds later.
 * Note that this setting is saved in nonvolatile memory
 * and only has to be performed once
 */
int enableSWpower(const board_t* pBoard);

/**
 * Disables the above. Note that this setting is stored
 * in nonvolatile memory and only has to be performed once.
 */
int disableSWpower(const board_t* pBoard);

/**
 * Update the digitial output. Parameter state can be value of:
 * 	 STATE_OFF		Single bit off
 * 	 STATE_ON		Single bit on
 * 	 STATE_TOGGLE   Toggle single bit
 *	 STATE_ALL		Update all digital outputs
 */
int updateDOUT(const board_t* pBoard, const uint8_t bit, const uint8_t state);

/**
 * Set single bit
 */
int digitalOutON(const board_t* pBoard, const uint8_t bit);

/**
 * Clear single bit
 */
int digitalOutOFF(const board_t* pBoard, const uint8_t bit);

/**
 * Toggle a single bit
 */
int digitalOutToggle(const board_t* pBoard, const uint8_t bit);

/**
 * Set all the bits a once. Bit mask must be 127 or less.
 */
int setDigitalOut(const board_t* pBoard, const uint8_t mask);

/**
 * Return the state byte of the digital output
 */
int getDOUTbyte(const board_t* pBoard, uint8_t* pData);

/**
 * Set PWM duty cycle from 0 to 1023 (0 to 100%)
 */
int setPWM(const board_t* pBoard, const uint8_t channel, uint32_t value);

/**
 * Return current PWM setting.
 */
int getPWM(const board_t* pBoard, const uint8_t channel, uint32_t* pData);

/**
 * Set DAC output voltage to 0 to 4.097 volts.
 */
int setDAC(const board_t* pBoard, const uint8_t channel, float value);

/**
 * Return current DAC output voltage.
 */
int getDAC(const board_t* pBoard, const uint8_t channel, float* pData);

/**
 * Calibrate the DAC outputs. Use this function if you are
 * unsure about the quality of your power supply.
 */
int calcDAC(const board_t* pBoard);

/**
 * Enable or disable interrupts from the DAQC.
 * Parameter state can be value of:
 * 	 STATE_OFF Interrupts off
 * 	 STATE_ON  Interrupts on
 */
int updateINT(const board_t* pBoard, const uint8_t state);

/**
 * Enable interrupts from the DAQC. GPIO22 will be pulled low
 * if an enabled event occurs.
 */
int enableINT(const board_t* pBoard);

/**
 * Disables and clears all interrupts on the DAQC.
 */
int disableINT(const board_t* pBoard);

/**
 * Returns 16 bit flag value then clears all INT flags
 */
int getINTflags(const board_t* pBoard, uint16_t* pFlags);


#endif // _ppapi_h
