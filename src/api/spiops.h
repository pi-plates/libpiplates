/*
 ============================================================================
 Name        : spiops.c
 Author      : B. Eschrich
 Version     :
 Copyright   : Copyright (c) 2016-2017 by B. Eschrich (EoF)
 Description : PI-Plates SPI bus operations
 ============================================================================
 */
#ifndef _spiops_h
#define _spiops_h

/* SPI command structure
 * channel		- SPI bus channel to use
 * address		- PI Plates board address
 * command		- PI Plates board command
 * param1		- Command parameter 1
 * param2		- Command parameter 2
 * size			- Return buffer size
 * result		- Pointer to the return buffer
 * stopAt0x0	- Stop string read operation if 0x0 found
 */
struct board_command
{
    uint8_t channel;
    uint8_t address;
    uint8_t command;
    uint8_t param1;
    uint8_t param2;
    uint8_t size;
    uint8_t* result;
    uint8_t stopAt0x0;
};

/* The board command structure type */
typedef struct board_command board_command_t;

/**
 * Execute a PI Plates command through SPI bus
 * @param pCommand Is a pointer of the board_command structure
 */
int spiCommand(const board_command_t* pCommand);

#endif // _spiops_h
