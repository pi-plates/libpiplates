/*
 ============================================================================
 Name        : pprelay.h
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

/* Relay state bits */
#define RELAY1_STATE_ON			0x01
#define RELAY2_STATE_ON			0x02
#define RELAY3_STATE_ON			0x04
#define RELAY4_STATE_ON			0x08
#define RELAY5_STATE_ON			0x10
#define RELAY6_STATE_ON			0x20
#define RELAY7_STATE_ON			0x40

/* PI-Plates GPIO pin and board base address */
struct config
{
    uint8_t pinInterrupt;
    uint8_t pinFrameControl;
    uint8_t boardBaseAddr;
    uint8_t spiChannel;
};

typedef struct config config_t;

/* PI-plates board handle structure */
struct board
{
    config_t config;
    uint8_t address;
    uint8_t type;
    float vcc;
};

typedef struct board board_t;

/**
 *
 */
uint8_t getBoardList(const uint8_t type, board_t** result);

/**
 *
 */
uint8_t getBoardCount(const uint8_t type);

/**
 *
 */
board_t* getBoardByAddress(const uint8_t address);

/**
 *
 */
int initConfig(const uint8_t spiChannel, const uint8_t wpiPinINT, const uint8_t wpiPinFrame, const uint8_t boardBaseAddr, config_t* pConfig);

/**
 *
 */
int initBoards(uint8_t type, const config_t* pConfig);

/**
 *
 */
int enableFrame(const board_t* board);

/**
 *
 */
int disableFrame(const board_t* board);

/**
 *
 */
uint8_t getAddress(const board_t* board, uint8_t* address);

/**
 *
 */
int reset(const board_t* board);

/**
 *
 */
int getHWRevision(const board_t* board, char* revision, const size_t size);

/**
 *
 */
int getFWRevision(const board_t* board, char* revision, const size_t size);

/**
 *
 */
int getID(const board_t* board, char* id, const size_t size);

/**
 *
 */
int getProgMemory(const board_t* board, const uint32_t address, char* data, const size_t size);

/**
 *
 */
int updateLED(const board_t* board, const uint8_t led, const uint8_t state);

/**
 *
 */
int getLEDState(const board_t* board, const uint8_t led, uint8_t* state);

/********************************************************************
 RELAYplate specific functions
 ********************************************************************/

/**
 *
 */
int updateRelay(const board_t* board, const uint8_t relay, const uint8_t state);

/**
 *
 */
int relayON(const board_t* board, const uint8_t relay);

/**
 *
 */
int relayOFF(const board_t* board, const uint8_t relay);

/**
 *
 */
int toggleRelay(const board_t* board, const uint8_t relay);

/**
 *
 */
int updateRelays(const board_t* board, const uint8_t mask);

/**
 *
 */
int getRelayState(const board_t* board, uint8_t* state);

/********************************************************************
 DAQCplate specific functions
 ********************************************************************/

/**
 *
 */
int getProgMemory(const board_t* board, const uint32_t address, char* data, const size_t size);

/**
 *
 */
int getADC(const board_t* board, const uint8_t channel, float* data);

/**
 *
 */
int getADCall(const board_t* board, const uint8_t channel, float* data, const size_t size);

/**
 *
 */
int getDINbit(const board_t* board, const uint8_t bit, uint8_t* data);

/**
 *
 */
int getDINall(const board_t* board, uint8_t* data);

/**
 *
 */
int enableDINint(const board_t* board, const uint8_t bit, const unsigned char edge);

/**
 *
 */
int disableDINint(const board_t* board, const uint8_t bit);

/**
 *
 */
int getRange(const board_t* board, const uint8_t channel, const unsigned char units, float* data);

/**
 *
 */
int getSWstate(const board_t* board, uint8_t* state);

/**
 *
 */
int enableSWint(const board_t* board);

/**
 *
 */
int disableSWint(const board_t* board);

/**
 *
 */
int enableSWpower(const board_t* board);

/**
 *
 */
int disableSWpower(const board_t* board);

/**
 *
 */
int updateDOUT(const board_t* board, const uint8_t bit, const uint8_t state);

/**
 *
 */
int digitalOutON(const board_t* board, const uint8_t bit);

/**
 *
 */
int digitalOutOFF(const board_t* board, const uint8_t bit);

/**
 *
 */
int digitalOutToggle(const board_t* board, const uint8_t bit);

/**
 *
 */
int setDigitalOut(const board_t* board, const uint8_t bitMask);

/**
 *
 */
int getDOUTbyte(const board_t* board, uint8_t* value);

/**
 *
 */
int setPWM(const board_t* board, const uint8_t channel, uint32_t value);

/**
 *
 */
int getPWM(const board_t* board, const uint8_t channel, uint32_t* data);

/**
 *
 */
int setDAC(const board_t* board, const uint8_t channel, float value);

/**
 *
 */
int getDAC(const board_t* board, const uint8_t channel, float* data);

/**
 *
 */
int calcDAC(const board_t* board);

/**
 *
 */
int updateINT(const board_t* board, const uint8_t state);

/**
 *
 */
int enableINT(const board_t* board);

/**
 *
 */
int disableINT(const board_t* board);

/**
 *
 */
int getINTflags(const board_t* board, uint16_t* flags);


#endif // _ppapi_h
