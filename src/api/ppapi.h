/*
 ============================================================================
 Name        : ppapi.h
 Author      : B. Eschrich
 Version     : 1.00
 Copyright   : Copyright (c) 2016-2017 by B. Eschrich (EoF)
 Description : PI-Plates RELAYplate API
 ============================================================================
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

#define PP_BOARD_TYPE_RELAY 1
#define PP_BOARD_TYPE_DAQC 2
#define PP_BOARD_TYPE_MOTOR 3

// Maximum of available PI-Plates board types
#define PP_MAX_BOARD_TYPES 3

// Maximum of possible PI-Plates boards connected
#define PP_MAX_BOARD_LIST_SIZE (sizeof(board_t) * (PP_MAX_BOARD_COUNT * PP_MAX_BOARD_TYPES))

/* PI-Plates - RELAYPlate constants */
#define MAX_RELAYS 				8

/* Relay and LED States */
#define STATE_OFF	 			0x00
#define STATE_ON 				0x01
#define STATE_TOGGLE 			0x02
#define STATE_ALL				0x03

/* Bit ON states */
#define BIT1_STATE_ON			0x01
#define BIT2_STATE_ON			0x02
#define BIT3_STATE_ON			0x04
#define BIT4_STATE_ON			0x08
#define BIT5_STATE_ON			0x10
#define BIT6_STATE_ON			0x20
#define BIT7_STATE_ON			0x40
#define BIT8_STATE_ON			0x80

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
 * must be zero. That indicate that the given board address in the
 * address field of the board_t structure is valid.
 * @param pBoard Handle of the PI-Plates board
 * @param pAddress Pointer to retrieve the SPI board address
 * @return 0 success otherwise signal an error
 */
int getAddress(const board_t* pBoard, uint8_t* pData);

/**
 *
 */
int reset(const board_t* pBoard);

/**
 *
 */
int getHWRevision(const board_t* pBoard, char* pData, const size_t size);

/**
 *
 */
int getFWRevision(const board_t* pBoard, char* pData, const size_t size);

/**
 *
 */
int getID(const board_t* pBoard, char* pData, const size_t size);

/**
 *
 */
int getProgMemory(const board_t* pBoard, const uint32_t address, char* pData, const size_t size);

/**
 *
 */
int updateLED(const board_t* pBoard, const uint8_t led, const uint8_t state);

/**
 *
 */
int getLEDState(const board_t* pBoard, const uint8_t led, uint8_t* pData);

/********************************************************************
 RELAYplate specific functions
 ********************************************************************/

/**
 *
 */
int updateRelay(const board_t* pBoard, const uint8_t relay, const uint8_t state);

/**
 *
 */
int relayON(const board_t* pBoard, const uint8_t relay);

/**
 *
 */
int relayOFF(const board_t* pBoard, const uint8_t relay);

/**
 *
 */
int toggleRelay(const board_t* pBoard, const uint8_t relay);

/**
 *
 */
int updateRelays(const board_t* pBoard, const uint8_t mask);

/**
 *
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
 *
 */
int getADC(const board_t* pBoard, const uint8_t channel, float* pData);

/**
 *
 */
int getADCall(const board_t* pBoard, const uint8_t channel, float* pData, const size_t size);

/**
 *
 */
int getDINbit(const board_t* pBoard, const uint8_t bit, uint8_t* pData);

/**
 *
 */
int getDINall(const board_t* pBoard, uint8_t* pData);

/**
 *
 */
int enableDINint(const board_t* pBoard, const uint8_t bit, const unsigned char edge);

/**
 *
 */
int disableDINint(const board_t* pBoard, const uint8_t bit);

/**
 *
 */
int getRange(const board_t* pBoard, const uint8_t channel, const unsigned char units, float* pData);

/**
 *
 */
int getSWstate(const board_t* pBoard, uint8_t* pData);

/**
 *
 */
int enableSWint(const board_t* pBoard);

/**
 *
 */
int disableSWint(const board_t* pBoard);

/**
 *
 */
int enableSWpower(const board_t* pBoard);

/**
 *
 */
int disableSWpower(const board_t* pBoard);

/**
 *
 */
int updateDOUT(const board_t* pBoard, const uint8_t bit, const uint8_t state);

/**
 *
 */
int digitalOutON(const board_t* pBoard, const uint8_t bit);

/**
 *
 */
int digitalOutOFF(const board_t* pBoard, const uint8_t bit);

/**
 *
 */
int digitalOutToggle(const board_t* pBoard, const uint8_t bit);

/**
 *
 */
int setDigitalOut(const board_t* pBoard, const uint8_t bitMask);

/**
 *
 */
int getDOUTbyte(const board_t* pBoard, uint8_t* pData);

/**
 *
 */
int setPWM(const board_t* pBoard, const uint8_t channel, uint32_t value);

/**
 *
 */
int getPWM(const board_t* pBoard, const uint8_t channel, uint32_t* pData);

/**
 *
 */
int setDAC(const board_t* pBoard, const uint8_t channel, float value);

/**
 *
 */
int getDAC(const board_t* pBoard, const uint8_t channel, float* pData);

/**
 *
 */
int calcDAC(const board_t* pBoard);

/**
 *
 */
int updateINT(const board_t* pBoard, const uint8_t state);

/**
 *
 */
int enableINT(const board_t* pBoard);

/**
 *
 */
int disableINT(const board_t* pBoard);

/**
 *
 */
int getINTflags(const board_t* pBoard, uint16_t* pFlags);


#endif // _ppapi_h
