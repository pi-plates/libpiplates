# C library for the PI-Plates I/O boards
## made for Raspberry PI (libpiplates)

## API overview

Help would be nice - ThX.

## API Defines and Functions

Help would be nice - ThX.

#### struct config####
PI-Plates GPIO pin and board base address structure
**uint8_t pinInterrupt** Digitial input pin - Signal to interrupt (BCM pin 22)
**uint8_t pinFrameControl** Digital output pin - Signal to control SPI bus transfer (BCM pin 25)
**uint8_t boardBaseAddr** PI-Plates specific SPI board base address
**uint8_t spiChannel** SPI BUS channel number (0=/dev/spidev0.0 or 1=/dev/spidev0.1) *Default is 1*

**PI-plates board handle structure**
struct board
{
	**GPIO/SPI configuration**
    config_t config;

    **The PI-Plates board address in range 0 to 7**
    uint8_t address;

    **The PI-Plates board type 1=RELAYplate 2=DAQCplate 3=MOTORplate**
    uint8_t type;

    **In case of the DAQCplate the Vcc value in volts**
    float vcc;
};
typedef struct board board_t;

**API Version structure**
struct version
{
	const long major;
	const long minor;
	const long build;
	const long revision;
};
typedef struct version version_t;

#### uint8_t getBoardList(const uint8_t type, board_t\*\* ppResult);
Retrieve available boards for specific board type
**param type** One of the predefied board types (RELAY=1, DAQC=2 or MOTOR=3)
**param ppResult** Pointer to the list. Caller must free allocated resources.
**return** Number of boards in the list

#### uint8_t getBoardCount(const uint8_t type);
Retrieve count of available PI-Plates boards specified by board type
**param type** One of the predefined board types
**return** Number of available boards

#### board_t\* getBoardByAddress(const uint8_t address);
Retrieve a PI-Plates board handle by specified address. If the address could not be found, return value is NULL
**param address** The board address 0 to 7 (preselected by jumper on address header)
**return** A pointer of the allocated board_t structure.

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
uint8_t getAddress(const board_t* pBoard, uint8_t* pData);


## Requirements and Dependencies

Help would be nice - ThX.

### Thrid party dependencies

	- wiringPi library for Raspberry PI

### Copyright (c) 2016-2017 by B. Eschrich, (E)mpire (O)f (F)un Guild
### All rights reserved.


# Changes -
