/*
 ============================================================================
 Name        : ppapi.c
 Author      : B. Eschrich
 Version     : 1.00
 Copyright   : Copyright (c) 2016-2017 by B. Eschrich (EoF)
 Description : PI-Plates RELAYplate, DAQCplate and MOTORplate API
 ============================================================================
 */

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>

// GPIO/SPI library
#include <wiringPi.h>

// Project version informations
#include "../version.h"

// Out SPI bus operations
#include "spiops.h"

// PI Plates specific
#include "ppapi.h"

// API Version
static version_t g_version =
{
    .major		= PP_MAJOR,
    .minor 		= PP_MINOR,
    .build		= PP_BUILD,
    .revision	= PP_REVISION,
};

// Available PI-Plates boards
static board_t g_board_list[PP_MAX_BOARD_LIST_SIZE];
static uint8_t g_board_count = 0;

// Initialize wiringPi only once
static uint8_t g_first_call = 0;

/* ========================================================================
 * Internal functions
 * ======================================================================== */

/**
 *
 */
static int verifyBoardType(const uint8_t type)
{
    if((type >= PP_BOARD_TYPE_RELAY) && (type <= PP_BOARD_TYPE_MOTOR))
    {
        return 1;
    }
    fprintf(stderr, "Board type %d is out of range. Must be between 1 and 3.\n", type);
    return 0;
}

/**
 *
 */
static int verifyRelay(const uint8_t relay)
{
    if((relay >= 1) && (relay < MAX_RELAYS))
    {
        return 1;
    }
    fprintf(stderr, "Relay number #%d is out of range. Must be between 1 and 7.\n", relay);
    return 0;
}

/**
 *
 */
static int verifyAddr(const board_t* pBoard)
{
    const uint8_t type = pBoard->type;
    const uint8_t address = pBoard->address;

    int index;
    for(index = 0; index < g_board_count; index++)
    {
        if(g_board_list[index].address == address)
        {
            return 1;
        }
    }
    fprintf(stderr, "Board address #%d of board type #%d not found.\n", address, type);
    return 0;
}

/**
 *
 */
static int verifyState(const uint8_t state)
{
    if((state >= STATE_OFF) && (state <= STATE_ALL))
    {
        return 1;
    }
    fprintf(stderr, "Object state value #%d is out of range.\n", state);
    return 0;
}

/**
 *
 */
static void verifyPointer(const void* p)
{
    if(p == NULL)
    {
        errno = EFAULT;
        perror("NULL pointer access! Terminate process.");
        abort();
    }
}

/**
 *
 */
static int boardAllowed(const board_t* pBoard, const uint8_t type)
{
    verifyPointer(pBoard);

    if(pBoard->type == type)
    {
        return 1;
    }
    fprintf(stderr, "Board type %d is not allowed for called function.\n", pBoard->type);
    return 0;
}

/**
 *
 */
static int verifyDINchannel(const uint8_t channel)
{
    if((channel >= 0) && (channel <= 7))
    {
        return 1;
    }

    fprintf(stderr, "Digital input channel value out of range. Must be in the range of 0 to 7");
    return 0;
}

/**
 *
 */
static int verifyAINchannel(const uint8_t channel)
{
    if((channel >= 0) && (channel <= 8))
    {
        return 1;
    }

    fprintf(stderr, "Analog input channel value out of range. Must be in the range of 0 to 8");
    return 0;
}

/**
 *
 */
static int verifyDOUTchannel(const uint8_t channel)
{
    if((channel >= 0) && (channel <= 6))
    {
        return 1;
    }

    fprintf(stderr, "Digital output channel value out of range. Must be in the range of 0 to 6");
    return 0;
}

/**
 *
 */
static int verifyLED(const uint8_t led)
{
    if((led >= 0) && (led <= 1))
    {
        return 1;
    }

    fprintf(stderr, "Invalid LED value. Must be 0 or 1");
    return 0;
}

/**
 *
 */
static int sendCommand(const board_t* pBoard, const board_command_t* pCommand)
{
    if(enableFrame(pBoard) != 0)
    {
        return -1;
    }

    int rc = spiCommand(pCommand);

    if(disableFrame(pBoard) != 0)
    {
        return -1;
    }

    return rc;
}

/* ========================================================================
 * Generic PI-Plates functions
 * ======================================================================== */


/**
 * Print board informations for given PI-Plates board.
 * @param pBoard The board handle
 */
void printBoardInfo(const board_t* pBoard)
{
    verifyPointer(pBoard);

    char id[64] = "";
    char revision[10];

    printf("GPIO config ------------------\n");
    printf("Interrupt pin.....: %d\n", pBoard->config.pinInterrupt);
    printf("Frame control pin.: %d\n", pBoard->config.pinFrameControl);
    printf("Board base address: %d\n", pBoard->config.boardBaseAddr);
    printf("SPI channel.......: %d\n", pBoard->config.spiChannel);
    printf("Board specific ---------------\n");
    printf("Type..............: %d\n", pBoard->type);
    printf("Address...........: %d\n", pBoard->address);
    printf("Vcc...............: %2.2f Volt\n", pBoard->vcc);
    if(getID(pBoard, id, sizeof(id)) == 0)
    {
        printf("Board ID..........: %s\n", id);
    }
    if(getHWRevision(pBoard, revision, sizeof(revision)) == 0)
    {
        printf("Hardware Revision.: %s\n", revision);
    }
    if(getFWRevision(pBoard, revision, sizeof(revision)) == 0)
    {
        printf("Firmware Revision.: %s\n", revision);
    }
}

/**
 * Print C API version string
 */
void printAPIVersion()
{
    printf(
        ":: PI-Plates C API %s-%s\n"
        ":: Copyright (c) 2016-2017 by B. Eschrich (EoF)\n\n",
        PP_FULLVERSION_STRING, PP_STATUS_SHORT);
}

/**
 * Retrieve C API version string
 */
void getAPIVersionString(char* pVersion, size_t size)
{
    if(size > sizeof(PP_FULLVERSION_STRING) + 4)
    {
        snprintf(pVersion, size, "%s-%s", PP_FULLVERSION_STRING, PP_STATUS_SHORT);
    }
}

/**
 * Retrieve C API version information
 */
version_t* getAPIVersion()
{
    return &g_version;
}

/**
 * Retrive available boards for specific board type
 * @param type One of the predefied board types (RELAY=1, DAQC=2 or MOTOR=3)
 * @param ppResult Pointer to the list. Caller must free allocate resources.
 */
uint8_t getBoardList(const uint8_t type, board_t** ppResult)
{
    verifyPointer(ppResult);

    // invalidate result pointer
    *ppResult = NULL;

    if(verifyBoardType(type) == 0)
    {
        return 0;
    }

    // return of n boards found
    int boardsFound = 0;

    // make board type specific board list
    uint8_t index;
    board_t* pBoard;
    board_t* rb;

    for(index = 0; index < g_board_count; index++)
    {
        pBoard = &g_board_list[index];

        // is requested board type?
        if(pBoard->type == type)
        {
            // firtype One of the predefied board types (RELAY=1, DAQC=2 or MOTOR=3)st time in loop
            if(*ppResult == NULL)
            {
                *ppResult = (board_t*) malloc(sizeof(board_t) * PP_MAX_BOARD_COUNT);
            }
            rb = &(*ppResult[boardsFound]);
            memcpy(rb, pBoard, sizeof(board_t));

            // next board
            boardsFound++;

            // finish if max reached
            if(boardsFound >= PP_MAX_BOARD_COUNT)
            {
                break;
            }
        }
    }

    return boardsFound;
}

/**
 * Retrieve a PI-Plates board handle by specified address. If the address
 * could not be found, return value is NULL
 * @param address The board address 0 to 7 (preselected by jumper on address header)
 * @return A pointer of the allocated board_t structure.
 */
board_t* getBoardByAddress(const uint8_t address)
{
    uint8_t index;
    board_t* result;
    for(index = 0; index < g_board_count; index++)
    {
        result = &g_board_list[index];
        if(result->address == address)
        {
            return result;
        }
    }
    return NULL;
}

/**
 * Retrieve count of available PI-Plates boards specified by board type
 * @param type One of the predefied board types (RELAY=1, DAQC=2 or MOTOR=3)
 * @return Number of available boards
 */
uint8_t getBoardCount(const uint8_t type)
{
    if(verifyBoardType(type) == 0)
    {
        return 0;
    }

    uint8_t index;
    uint8_t boardsFound = 0;
    for(index = 0; index < g_board_count; index++)
    {
        board_t* pBoard = &g_board_list[index];
        if(pBoard->type == type)
        {
            boardsFound++;
        }
    }
    return boardsFound;
}

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
int initConfig(const uint8_t spiChannel, const uint8_t wpiPinINT, const uint8_t wpiPinFrame, const uint8_t boardBaseAddr, config_t* pConfig)
{
    verifyPointer(pConfig);

    if(g_first_call == 0)
    {
        g_first_call = 1;
        wiringPiSetupGpio(); // BCM pin layout root mode
    }

    pConfig->spiChannel = spiChannel;
    pConfig->pinInterrupt = wpiPinToGpio(wpiPinINT);
    pConfig->pinFrameControl = wpiPinToGpio(wpiPinFrame);
    pConfig->boardBaseAddr = boardBaseAddr;

    return 0;
}

/**
 * Initialize the PI-Plate boards by specified board type. Each available board becomes
 * a board_t handle allocated in a global board list. You must call initConfig(...) first
 * to get the configuration for the board GPIO/SPI communication.
 * @param type One of the predefied board types (RELAY=1, DAQC=2 or MOTOR=3)
 * @return 0 if succsess otherwise signal an error
 */
int initBoards(uint8_t type, const config_t* pConfig)
{
    if(g_first_call == 0)
    {
        fprintf(stderr, "FATAL: You must call 'initConfig()' first.\n");
        return -1;
    }

    board_t* pBoard;

    verifyPointer(pConfig);

    // clear available board list first time
    if(g_board_count == 0)
    {
        memset(g_board_list, 0, PP_MAX_BOARD_LIST_SIZE);
    }

    // set a pointer to the start of the list
    pBoard = &g_board_list[g_board_count];

    // copy basic operation values into configuration structure
    memcpy(&pBoard->config, pConfig, sizeof(config_t));

    // set requested board type
    pBoard->type = type;

    // Initialize frame signal
    pinMode(pConfig->pinFrameControl, OUTPUT);

    // lock SPI frame transfer
    if(disableFrame(pBoard) < 0)
    {
        return -1;
    }

    // Initialize interrupt control
    pinMode(pConfig->pinInterrupt, INPUT);
    pullUpDnControl(pConfig->pinInterrupt, PUD_UP);

    // time to system
    usleep(PP_DELAY);

    uint8_t i;
    uint8_t index = 0;

    // find PI-Plates addresses for requested board type
    for(i = 0; i < PP_MAX_BOARD_COUNT; i++)
    {
        board_t tstbrd;

        // load board configuration
        memcpy(&tstbrd.config, pConfig, sizeof(config_t));
        tstbrd.type = type;
        tstbrd.address = i;

        // check board address
        uint8_t addr;
        if(getAddress(&tstbrd, &addr) < 0)
        {
            fprintf(stderr, "Unable to get board address.\n");
            return -1;
        }

        int rc = (addr - pConfig->boardBaseAddr);
        if(rc == i)
        {
            // save board configuration at end of list
            memcpy(&g_board_list[g_board_count], &tstbrd, sizeof(board_t));
            // set new allocated board
            g_board_count++;
            // increment found counter
            index++;
        }
    }

    // any boards found?
    if(index == 0)
    {
        fprintf(stderr, "No boards of type %d found!\n", type);
        return -1;
    }

    // force board reset to available boards
    for(i = 0; i < g_board_count; i++)
    {
        if(g_board_list[i].type == type)
        {
            if(reset(&g_board_list[i]) < 0)
            {
                return -1;
            }

            // DAQC specific initialization
            if(g_board_list[i].type == PP_BOARD_TYPE_DAQC)
            {
                // set default VCC value
                g_board_list[i].vcc = (float) 10000;

                // measure real VCC value
                while(1)
                {
                    if(calcDAC(&g_board_list[i]) < 0)
                    {
                        return -1;
                    }
                    if(g_board_list[i].vcc > 3.0f)
                    {
                        break;
                    }
                }

                // reset all digital outputs
                updateDOUT(&g_board_list[i], 0, STATE_ALL);

                // initialize PWM
                setPWM(&g_board_list[i], 0, 0);
                setPWM(&g_board_list[i], 1, 0);
            }

            usleep(45000);
        }
    }

    // wait reset process finished
    sleep(1);

    // success
    return 0;
}

/**
 * Enable frame signal to transmit commands to the
 * board through the SPI bus.
 * @param pBoard Handle of the PI-Plates board
 * @return 0 success otherwise signal an error
 */
int enableFrame(const board_t* pBoard)
{
    verifyPointer(pBoard);

    // enable SPI frame transfer
    digitalWrite(pBoard->config.pinFrameControl, HIGH);

    // time to system
    usleep(PP_DELAY);

    // check bit has raised
    if(!digitalRead(pBoard->config.pinFrameControl))
    {
        return -1;
    }

    return 0;
}

/**
 * Disable frame signal to prevent transmission
 * through the SPI bus.
 * @param pBoard Handle of the PI-Plates board
 * @return 0 success otherwise signal an error
 */
int disableFrame(const board_t* pBoard)
{
    verifyPointer(pBoard);

    // enable SPI frame transfer
    digitalWrite(pBoard->config.pinFrameControl, LOW);

    // time to system
    usleep(PP_DELAY);

    // check bit has released
    if(digitalRead(pBoard->config.pinFrameControl))
    {
        return -1;
    }

    return 0;
}

/**
 * Retrieve the SPI board address. To test a valid board address
 * substract *pAddress - pBoard->config.boardBaseAddress. The result
 * must the same as pBoard->address. That indicate that the given board
 * address in the address field of the board_t structure is valid.
 * @param pBoard Handle of the PI-Plates board
 * @param pAddress Pointer to retrieve the SPI board address
 * @return 0 success otherwise signal an error
 */
int getAddress(const board_t* pBoard, uint8_t* pAddress)
{
    verifyPointer(pBoard);
    verifyPointer(pAddress);

    // reset return
    *pAddress = 0;

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x00,
        .param1 = 0,
        .param2 = 0,
        .result = pAddress,
        .size = sizeof(uint8_t),
        .stopAt0x0 = 0,
    };

    return sendCommand(pBoard, &bc);
}

/**
 *
 */
int reset(const board_t* pBoard)
{
    verifyPointer(pBoard);

    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x0F,
        .param1 = 0,
        .param2 = 0,
        .size = 0,
    };

    return sendCommand(pBoard, &bc);
}

/**
 *
 */
int getHWRevision(const board_t* pBoard, char* revision, const size_t size)
{
    verifyPointer(pBoard);
    verifyPointer(revision);

    // reset return
    memset(revision, 0, size);

    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(size < 10)
    {
        fprintf(stderr, "Buffer size %d to small to return revision.\n", size);
        return -1;
    }

    uint8_t value = 0;
    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x02,
        .param1 = 0,
        .param2 = 0,
        .result = &value,
        .size = sizeof(uint8_t),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    float whole = (float)(value >> 4);
    float point = (float)(value & 0x0F);
    float rev = whole + point / 10;
    snprintf(revision, size, "%2.2f", rev);

    return 0;
}

/**
 *
 */
int getFWRevision(const board_t* pBoard, char* revision, const size_t size)
{
    verifyPointer(pBoard);
    verifyPointer(revision);

    // reset return
    memset(revision, 0, size);

    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(size < 10)
    {
        fprintf(stderr, "Buffer size %d to small to return revision.\n", size);
        return -1;
    }

    uint8_t value = 0;
    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x03,
        .param1 = 0,
        .param2 = 0,
        .result = &value,
        .size = sizeof(uint8_t),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    float whole = (float)(value >> 4);
    float point = (float)(value & 0x0F);
    float rev = whole + point / 10;
    snprintf(revision, size, "%2.2f", rev);

    return 0;
}

/**
 *
 */
int getID(const board_t* pBoard, char* id, const size_t size)
{
    verifyPointer(pBoard);
    verifyPointer(id);

    // reset return
    memset(id, 0, size);

    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(size < 21)
    {
        fprintf(stderr, "Buffer size %d to small to return the ID.\n", size);
        return -1;
    }

    uint8_t result[20];
    memset(result, 0, sizeof(result));

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x01,
        .param1 = 0,
        .param2 = 0,
        .result = result,
        .size = sizeof(result),
        .stopAt0x0 = 1,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    snprintf(id, size, "%s", result);
    return strlen(id);
}

/**
 *
 */
int updateLED(const board_t* pBoard, const uint8_t led, const uint8_t state)
{
    verifyPointer(pBoard);

    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(pBoard->type == PP_BOARD_TYPE_DAQC)
    {
        if(verifyLED(led) == 0)
        {
            return -1;
        }
    }
    if(verifyState(state) == 0)
    {
        return -1;
    }

    int command = 0x00;
    switch(state)
    {
        // set
        case STATE_ON:
        {
            command = 0x60;
            break;
        }
        // clear
        case STATE_OFF:
        {
            command = 0x61;
            break;
        }
        // toggle
        case STATE_TOGGLE:
        {
            command = 0x62;
            break;
        }
        default:
        {
            fprintf(stderr, "Invalid LED state value %d.", state);
            return -1;
        }
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = command,
        .param1 = (pBoard->type != PP_BOARD_TYPE_DAQC ? 0 : led),
        .param2 = 0,
        .size = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int getLEDState(const board_t* pBoard, const uint8_t led, uint8_t* state)
{
    verifyPointer(pBoard);
    verifyPointer(state);

    // reset return
    *state = 0;

    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(verifyLED(led) == 0)
    {
        return -1;
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x63,
        .param1 = led,
        .param2 = 0,
        .result = state,
        .size = sizeof(uint8_t),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/* ========================================================================
 * PI-Plates - RELAYplate functions
 * ======================================================================== */

/**
 *
 */
int updateRelay(const board_t* pBoard, const uint8_t relay, const uint8_t state)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_RELAY) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(verifyState(state) == 0)
    {
        return -1;
    }
    if(state != STATE_ALL)
    {
        if(verifyRelay(relay) == 0)
        {
            return -1;
        }
    }
    else if(relay > 127)
    {
        fprintf(stderr, "Relay argument #%d out of range. Must be between 0 and 127", relay);
        return 0;
    }

    int command = 0x00;
    switch(state)
    {
        // clear
        case STATE_OFF:
        {
            command = 0x11;
            break;
        }
        // set
        case STATE_ON:
        {
            command = 0x10;
            break;
        }
        // toggle
        case STATE_TOGGLE:
        {
            command = 0x12;
            break;
        }
        // set/clr all relays by bit mask
        case STATE_ALL:
        {
            command = 0x13;
            break;
        }
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = command,
        .param1 = relay,
        .param2 = 0,
        .size = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int getRelayState(const board_t* pBoard, uint8_t* state)
{
    verifyPointer(pBoard);

    // reset return
    *state = 0;

    if(boardAllowed(pBoard, PP_BOARD_TYPE_RELAY) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }

    uint8_t result = 0;
    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x14,
        .param1 = 0,
        .param2 = 0,
        .result = &result,
        .size = sizeof(result),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    // return relay states
    *state = result;

    return 0;
}

/**
 *
 */
int relayON(const board_t* pBoard, const uint8_t relay)
{
    return updateRelay(pBoard, relay, STATE_ON);
}

/**
 *
 */
int relayOFF(const board_t* pBoard, const uint8_t relay)
{
    return updateRelay(pBoard, relay, STATE_OFF);
}

/**
 *
 */
int toggleRelay(const board_t* pBoard, const uint8_t relay)
{
    return updateRelay(pBoard, relay, STATE_TOGGLE);
}

/**
 *
 */
int updateRelays(const board_t* pBoard, const uint8_t mask)
{
    return updateRelay(pBoard, mask, STATE_ALL);
}

/* ========================================================================
 * PI-Plates - DAQCplate functions
 * ======================================================================== */

/**
 *
 */
int getProgMemory(const board_t* pBoard, const uint32_t address, char* data, const size_t size)
{
    verifyPointer(pBoard);
    verifyPointer(data);

    // reset return
    memset(data, 0, size);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(size < 3)
    {
        fprintf(stderr, "Buffer size %d to small to return the data value.\n", size);
        return -1;
    }

    uint8_t result[2];
    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x0F,
        .param1 = address >> 8,
        .param2 = address & 0xff,
        .result = result,
        .size = sizeof(result),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    int value = (256 * result[0] + result[1]);
    snprintf(data, size, "%02X", value);

    return strlen(data);
}

//=======================================
// ADC Functions
//=======================================

/**
 *
 */
int getADC(const board_t* pBoard, const uint8_t channel, float* data)
{
    verifyPointer(pBoard);
    verifyPointer(data);

    // reset return
    *data = 0;

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(verifyAINchannel(channel) == 0)
    {
        return -1;
    }

    uint8_t result[2] = {0,0};
    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x30,
        .param1 = channel,
        .param2 = 0,
        .result = result,
        .size = sizeof(result),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    float value;

    value = (256 * result[0] + result[1]);
    value = round(value * 4.096f / 1024);
    if(channel == 8)
    {
        value = value * 2.0f;
    }

    // return ADC value
    *data = value;

    return 0;
}

/**
 *
 */
int getADCall(const board_t* pBoard, float data[], const size_t size)
{
    verifyPointer(pBoard);
    verifyPointer(data);

    // reset return
    memset(data, 0, size);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(size < (sizeof(float) * 8))
    {
        fprintf(stderr, "ERROR: Parameter data must be an array of float with 8 elements.");
        return -1;
    }

    uint8_t result[16];
    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x31,
        .param1 = 0,
        .param2 = 0,
        .result = result,
        .size = sizeof(result),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    uint8_t index;

    // fill ADC value table
    for(index = 0; index < 8; index++)
    {
        data[index] = (256 * result[2 * index] + result[2 * index + 1]);
        data[index] = round(data[index] * 4.096f / 1024);
    }

    return 0;
}

//=======================================
// Digital Input Functions
//=======================================

/**
 *
 */
int getDINbit(const board_t* pBoard, const uint8_t bit, uint8_t* state)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(verifyDINchannel(bit) == 0)
    {
        return -1;
    }

    uint8_t result = 0;
    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x20,
        .param1 = bit,
        .param2 = 0,
        .result = &result,
        .size = sizeof(result),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    // return state value
    *state = result;

    return 0;
}

/**
 *
 */
int getDINall(const board_t* pBoard, uint8_t* states)
{
    verifyPointer(pBoard);

    // reset return
    *states = 0;

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }

    uint8_t result[] = {0,0};
    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x25,
        .param1 = 0,
        .param2 = 0,
        .result = result,
        .size = sizeof(result),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    // return states value
    *states = result[0];

    return 0;
}

/**
 *
 */
int enableDINint(const board_t* pBoard, const uint8_t bit, const unsigned char edge)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(verifyDINchannel(bit) == 0)
    {
        return -1;
    }

    uint8_t cmd = 0;

    if((edge == 'f') || (edge == 'F'))
    {
        cmd = 0x21;
    }
    if((edge == 'r') || (edge == 'R'))
    {
        cmd = 0x22;
    }
    if((edge == 'b') || (edge == 'B'))
    {
        cmd = 0x23;
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = cmd,
        .param1 = bit,
        .param2 = 0,
        .result = 0,
        .size = 0,
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int disableDINint(const board_t* pBoard, const uint8_t bit)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(verifyDINchannel(bit) == 0)
    {
        return -1;
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x24,
        .param1 = bit,
        .param2 = 0,
        .result = 0,
        .size = 0,
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

//=======================================
// Hybrid Input Functions
//=======================================

/**
 *
 */
int getRange(const board_t* pBoard, const uint8_t channel, const unsigned char units, float* data)
{
    verifyPointer(pBoard);
    verifyPointer(data);

    // reset return
    *data = 0.0f;

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    const unsigned char c = tolower(units);
    if(c != 'i' && c != 'c')
    {
        fprintf(stderr, "ERROR: incorrect units parameter. Must be 'c' or 'i'.\n");
        return -1;
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x80,
        .param1 = channel,
        .param2 = 0,
        .result = 0,
        .size = 0,
        .stopAt0x0 = 0,
    };

    if(enableFrame(pBoard) != 0)
    {
        return -1;
    }

    int rc = spiCommand(&bc);
    if(rc < 0)
    {
        disableFrame(pBoard);
        return rc;
    }

    usleep(7000);

    uint8_t result[2] = {0, 0};

    bc.command = 0x81;
    bc.result = result;
    bc.size = sizeof(result);

    rc = spiCommand(&bc);
    if(rc < 0)
    {
        disableFrame(pBoard);
        return rc;
    }

    if(disableFrame(pBoard) != 0)
    {
        return -1;
    }

    float range = result[0] * 256 + result[1];
    if(range == 0.0f)
    {
        fprintf(stderr, "ERROR: Sensor #%d failue.\n", channel);
        return -1;
    }

    if(c == 'c')
    {
        range = range / 58.326;
    }
    if(c == 'i')
    {
        range = range / 148.148;
    }

    *data = round(range);

    usleep(PP_DELAY*10);

    return 0;
}

//=======================================
// Switch Functions
//=======================================

/**
 *
 */
int getSWstate(const board_t* pBoard, uint8_t* state)
{
    verifyPointer(pBoard);
    verifyPointer(state);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }

    uint8_t result = 0;
    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x50,
        .param1 = 0,
        .param2 = 0,
        .result = &result,
        .size = sizeof(result),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    // return state value
    *state = result;

    return 0;
}

/**
 *
 */
int enableSWint(const board_t* pBoard)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x51,
        .param1 = 0,
        .param2 = 0,
        .result = 0,
        .size = 0,
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int disableSWint(const board_t* pBoard)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x52,
        .param1 = 0,
        .param2 = 0,
        .result = 0,
        .size = 0,
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int enableSWpower(const board_t* pBoard)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x53,
        .param1 = 0,
        .param2 = 0,
        .result = 0,
        .size = 0,
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int disableSWpower(const board_t* pBoard)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x54,
        .param1 = 0,
        .param2 = 0,
        .result = 0,
        .size = 0,
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

// ============================================
// Digital Output Functions
// ============================================

/**
 *
 */
int updateDOUT(const board_t* pBoard, const uint8_t bit, const uint8_t state)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(verifyState(state) == 0)
    {
        return -1;
    }
    if(state != STATE_ALL)
    {
        if(verifyDOUTchannel(bit) == 0)
        {
            return -1;
        }
    }
    else if(bit > 127)
    {
        fprintf(stderr, "Digital output value #%d out of range. Must be between 0 and 127", bit);
        return 0;
    }

    int command = 0x00;
    switch(state)
    {
        // clear
        case STATE_OFF:
        {
            command = 0x11;
            break;
        }
        // set
        case STATE_ON:
        {
            command = 0x10;
            break;
        }
        // toggle
        case STATE_TOGGLE:
        {
            command = 0x12;
            break;
        }
        // set/clr all relays by bit mask
        case STATE_ALL:
        {
            command = 0x13;
            break;
        }
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = command,
        .param1 = bit,
        .param2 = 0,
        .size = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int digitalOutON(const board_t* pBoard, const uint8_t bit)
{
    return updateDOUT(pBoard, bit, STATE_ON);
}

/**
 *
 */
int digitalOutOFF(const board_t* pBoard, const uint8_t bit)
{
    return updateDOUT(pBoard, bit, STATE_OFF);
}

/**
 *
 */
int digitalOutToggle(const board_t* pBoard, const uint8_t bit)
{
    return updateDOUT(pBoard, bit, STATE_TOGGLE);
}

/**
 *
 */
int setDigitalOut(const board_t* pBoard, const uint8_t bitMask)
{
    return updateDOUT(pBoard, bitMask, STATE_ALL);
}

/**
 *
 */
int getDOUTbyte(const board_t* pBoard, uint8_t* value)
{
    verifyPointer(pBoard);
    verifyPointer(value);

    // reset return
    *value = 0;

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }

    uint8_t result = 0;
    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x14,
        .param1 = 0,
        .param2 = 0,
        .result = &result,
        .size = sizeof(result),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    // return DOUT bit states
    *value = result;

    return 0;
}

// ============================================
// PWM and DAC Output Functions
// ============================================

int setPWM(const board_t* pBoard, const uint8_t channel, uint32_t value)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(channel < 0 || channel > 1)
    {
        fprintf(stderr, "ERROR: PWM channel %d must be 0 or 1\n", channel);
        return -1;
    }
    if(value < 0 || value > 1023)
    {
        fprintf(stderr, "ERROR: PWM value %d out of range - must be between 0 and 1023\n", value);
        return -1;
    }

    const uint8_t hibyte = (uint8_t) value >> 8;
    const uint8_t lobyte = (uint8_t) value - (hibyte << 8);

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x40 + channel,
        .param1 = hibyte,
        .param2 = lobyte,
        .result = 0,
        .size = 0,
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int getPWM(const board_t* pBoard, const uint8_t channel, uint32_t* data)
{
    verifyPointer(pBoard);
    verifyPointer(data);

    // reset return
    *data = 0;

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(channel < 0 || channel > 1)
    {
        fprintf(stderr, "ERROR: PWM channel %d must be 0 or 1\n", channel);
        return -1;
    }

    uint8_t result[2] = {0,0};

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x40 + channel + 2,
        .param1 = 0,
        .param2 = 0,
        .result = result,
        .size = sizeof(result),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    // return PWM value
    *data = (256 * result[0] + result[1]);

    return 0;
}

/**
 *
 */
int setDAC(const board_t* pBoard, const uint8_t channel, float volts)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(channel < 0 || channel > 1)
    {
        fprintf(stderr, "ERROR: DAC channel %d must be 0 or 1\n", channel);
        return -1;
    }
    if(volts < 0.0f || volts > 4.095f)
    {
        fprintf(stderr, "ERROR: PWM value %f out of range - must be between 0 and 4.095 volts\n", volts);
        return -1;
    }

    const int16_t value = (int)(volts / pBoard->vcc * 1024);
    const uint8_t hibyte = (uint8_t) value >> 8;
    const uint8_t lobyte = (uint8_t) value- (hibyte << 8);

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x40 + channel,
        .param1 = hibyte,
        .param2 = lobyte,
        .result = 0,
        .size = 0,
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int getDAC(const board_t* pBoard, const uint8_t channel, float* data)
{
    verifyPointer(pBoard);
    verifyPointer(data);

    // reset return
    *data = 0.0f;

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(channel < 0 || channel > 1)
    {
        fprintf(stderr, "ERROR: DAC channel %d must be 0 or 1\n", channel);
        return -1;
    }

    uint8_t result[2] = {0,0};

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x40 + channel + 2,
        .param1 = 0,
        .param2 = 0,
        .result = result,
        .size = sizeof(result),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    // return DAC value
    *data = (256 * result[0] + result[1]) * (pBoard->vcc / 1023);

    return 0;
}

/**
 *
 */
int calcDAC(const board_t* pBoard)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }

    float* vcc = &((board_t*) pBoard)->vcc;
    if(getADC(pBoard, 8, vcc) != 0)
    {
        return -1;
    }

    return 0;
}

// ============================================
// Interrupt Functions
// ============================================

/**
 *
 */
int updateINT(const board_t* pBoard, const uint8_t state)
{
    verifyPointer(pBoard);

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }
    if(verifyState(state) == 0)
    {
        return -1;
    }

    int command = 0x00;
    switch(state)
    {
        // disable INT
        case STATE_OFF:
        {
            command = 0x05;
            break;
        }
        // enable INT
        case STATE_ON:
        {
            command = 0x04;
            break;
        }
        default:
        {
            fprintf(stderr, "INT state #%d not valid. Must be 0 or 1", state);
            return -1;
        }
    }

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = command,
        .param1 = 0,
        .param2 = 0,
        .size = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int enableINT(const board_t* pBoard)
{
    return updateINT(pBoard, STATE_ON);
}

/**
 *
 */
int disableINT(const board_t* pBoard)
{
    return updateINT(pBoard, STATE_OFF);
}

/**
 *
 */
int getINTflags(const board_t* pBoard, uint16_t* flags)
{
    verifyPointer(pBoard);
    verifyPointer(flags);

    // reset return
    *flags = 0;

    if(boardAllowed(pBoard, PP_BOARD_TYPE_DAQC) == 0)
    {
        return -1;
    }
    if(verifyAddr(pBoard) == 0)
    {
        return -1;
    }

    uint8_t result[2] = {0,0};

    board_command_t bc =
    {
        .channel = pBoard->config.spiChannel,
        .address = pBoard->config.boardBaseAddr + pBoard->address,
        .command = 0x06,
        .param1 = 0,
        .param2 = 0,
        .result = result,
        .size = sizeof(result),
        .stopAt0x0 = 0,
    };

    if(sendCommand(pBoard, &bc) != 0)
    {
        return -1;
    }

    // return value of interrupt flags
    *flags = (256 * result[0] + result[1]);

    return 0;
}
