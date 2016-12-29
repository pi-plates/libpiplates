/*
 ============================================================================
 Name        : rpi-control.c
 Author      : B. Eschrich
 Version     : 1.00
 Copyright   : Copyright (c) 2016-2017 by B. Eschrich (EoF)
 Description : PI-Plates RELAYplate and DAQCplate control
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

// GPIO/SPI library
#include <wiringPi.h>

// Out SPI bus operations
#include "../api/spiops.h"

// PI Plates RELAYplate API
#include "../api/ppapi.h"

#define DAQC_BOARD_ADDR 0
#define RELAY_BOARD_ADDR 1

/**
 *
 */
static void pabort(const char* s)
{
    perror(s);
    abort();
}

static int testRelayBoard()
{
    config_t config;

    puts("=================================================");
    puts("RELAYplate test");
    puts("=================================================");

    // Initialize GPIO / SPI addresses for the RELAYplate
    if(initConfig(PP_SPI_IO_CHANNEL, 3, 6, 24, &config) < 0)
    {
        pabort("initConfig() failed.");
        return EXIT_FAILURE;
    }

    // Initialize all available RELAYplate boards
    if(initBoards(PP_BOARD_TYPE_RELAY, &config) < 0)
    {
        pabort("initBoards() failed.");
        return EXIT_FAILURE;
    }

    // get our configured RELAYplate board (see jumper @ address header)
    board_t* board = getBoardByAddress(RELAY_BOARD_ADDR);
    if(board == NULL)
    {
        pabort("initBoards() failed. No board for given address available!");
        return EXIT_FAILURE;
    }

    char id[64] = "";
    if(getID(board, id, sizeof(id)) < 0)
    {
        pabort("getID() failed.");
        return EXIT_FAILURE;
    }

    printf("Board ID: %s\n", id);

    char revision[10];

    if(getHWRevision(board, revision, sizeof(revision)) < 0)
    {
        pabort("getHWRevision() failed.");
        return EXIT_FAILURE;
    }
    printf("Hardware Revision: %s\n", revision);

    if(getFWRevision(board, revision, sizeof(revision)) < 0)
    {
        pabort("getFWRevision() failed.");
        return EXIT_FAILURE;
    }
    printf("Firmware Revision: %s\n", revision);

    // switch off green board LED
    if(updateLED(board, 0, STATE_OFF) < 0)
    {
        pabort("LED switch off failed.");
        return EXIT_FAILURE;
    }

    uint8_t state;
    int relay = 4;

    for(relay = 1; relay < MAX_RELAYS; relay++)
    {
        printf("Switch ON  relay #%d ", relay);
        if(relayON(board, relay) < 0)
        {
            pabort("\nRelay ON failed.");
            return EXIT_FAILURE;
        }

        usleep(220000);

        if(getRelayState(board, &state) < 0)
        {
            pabort("\nGet relay state failed.");
            return EXIT_FAILURE;
        }
        printf("states: 0x%x\n", state);

        printf("Switch OFF relay #%d ", relay);
        if(relayOFF(board, relay) < 0)
        {
            pabort("\nRelay OFF failed.");
            return EXIT_FAILURE;
        }

        usleep(220000);

        if(getRelayState(board, &state) < 0)
        {
            pabort("\nGet relay state failed.");
            return EXIT_FAILURE;
        }
        printf("states: 0x%x\n", state);
    }

    uint8_t relays = RELAY2_STATE_ON | RELAY4_STATE_ON | RELAY6_STATE_ON;
    if(updateRelays(board, relays) < 0)
    {
        pabort("Switch relays by bit mask failed.");
        return EXIT_FAILURE;
    }

    usleep(589000);

    // switch off two relays
    relays = relays & ~RELAY4_STATE_ON;
    relays = relays & ~RELAY6_STATE_ON;

    // switch on another two relays
    relays |= RELAY5_STATE_ON;
    relays |= RELAY7_STATE_ON;

    if(updateRelays(board, relays) < 0)
    {
        pabort("Switch relays by bit mask failed.");
        return EXIT_FAILURE;
    }

    usleep(589000);

    if(updateRelays(board, 0) < 0)
    {
        pabort("Switch relays by bit mask failed.");
        return EXIT_FAILURE;
    }

    // switch on green board LED
    if(updateLED(board, 0, STATE_ON) < 0)
    {
        pabort("LED switch on failed.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int testDAQCBoard()
{
    config_t config;

    puts("=================================================");
    puts("DAQCplate test");
    puts("=================================================");

    // Initialize GPIO / SPI addresses for the DAQCplate
    if(initConfig(PP_SPI_IO_CHANNEL, 3, 6, 8, &config) < 0)
    {
        pabort("initConfig() failed.");
        return EXIT_FAILURE;
    }

    // Initialize all available RELAYplate boards
    if(initBoards(PP_BOARD_TYPE_DAQC, &config) < 0)
    {
        pabort("initBoards() failed.");
        return EXIT_FAILURE;
    }

    // get our configured RELAYplate board (see jumper @ address header)
    board_t* board = getBoardByAddress(DAQC_BOARD_ADDR);
    if(board == NULL)
    {
        pabort("initBoards() failed. No board for given address available!");
        return EXIT_FAILURE;
    }

    char id[64] = "";
    if(getID(board, id, sizeof(id)) < 0)
    {
        pabort("getID() failed.");
        return EXIT_FAILURE;
    }

    printf("Board ID: %s\n", id);

    char revision[10];

    if(getHWRevision(board, revision, sizeof(revision)) < 0)
    {
        pabort("getHWRevision() failed.");
        return EXIT_FAILURE;
    }
    printf("Hardware Revision: %s\n", revision);

    if(getFWRevision(board, revision, sizeof(revision)) < 0)
    {
        pabort("getFWRevision() failed.");
        return EXIT_FAILURE;
    }
    printf("Firmware Revision: %s\n", revision);

    if(calcDAC(board) < 0)
    {
        pabort("calDAC() failed.");
        return EXIT_FAILURE;
    }
    printf("Board VCC........: %2.3f\n", board->vcc);

    updateLED(board, 0, STATE_OFF);
    updateLED(board, 1, STATE_OFF);

    sleep(1);

    updateLED(board, 0, STATE_TOGGLE);

    sleep(1);

    updateLED(board, 0, STATE_OFF);
    updateLED(board, 1, STATE_TOGGLE);

    uint16_t flags = 0;
    if(getINTflags(board, &flags) < 0)
    {
        pabort("getINTflags() failed");
        return EXIT_FAILURE;
    }
    printf("INT flags........: 0x%04x\n", flags);

    float range = 0.0f;

    if(getRange(board, 0, 'i', &range) < 0)
    {
        pabort("getRange(0, i) failed");
        return EXIT_FAILURE;
    }
    printf("Range(0,i).......: %4.2f\n", range);

    if(getRange(board, 0, 'c', &range) < 0)
    {
        pabort("getRange(0, c) failed");
        return EXIT_FAILURE;
    }
    printf("Range(0,c).......: %4.2f\n", range);

    /*	--> Both calls failing ?!
    	if (getRange(board, 1, 'i', &range) < 0)
    	{
    		pabort("getRange(1, i) failed");
            return EXIT_FAILURE;
    	}
        printf("Range(1,i).......: %4.2f\n", range);

    	if (getRange(board, 1, 'c', &range) < 0)
    	{
    		pabort("getRange(1, c) failed");
            return EXIT_FAILURE;
    	}
        printf("Range(1,c).......: %4.2f\n", range);
    */

    float data = 0.0f;

    if(getADC(board, 0, &data) < 0)
    {
        pabort("getADC(0) failed");
        return EXIT_FAILURE;
    }
    printf("ADC(0)...........: %4.2f volt\n", data);

    if(getADC(board, 1, &data) < 0)
    {
        pabort("getADC(1) failed");
        return EXIT_FAILURE;
    }
    printf("ADC(1)...........: %4.2f volt\n", data);

    uint32_t pwm = 0;
    if(getPWM(board, 0, &pwm) < 0)
    {
        pabort("getPWM(0) failed");
        return EXIT_FAILURE;
    }
    printf("PWM(0)...........: %d\n", pwm);

    if(getPWM(board, 1, &pwm) < 0)
    {
        pabort("getPWM(1) failed");
        return EXIT_FAILURE;
    }
    printf("PWM(1)...........: %d\n", pwm);

    float dac = 0;
    if(getDAC(board, 0, &dac) < 0)
    {
        pabort("getDAC(0) failed");
        return EXIT_FAILURE;
    }
    printf("DAC(0)...........: %2.2f\n", dac);

    if(getDAC(board, 1, &dac) < 0)
    {
        pabort("getDAC(1) failed");
        return EXIT_FAILURE;
    }
    printf("DAC(1)...........: %2.2f\n", dac);

    uint8_t dout = RELAY2_STATE_ON | RELAY5_STATE_ON;
    if(setDigitalOut(board, dout) < 0)
    {
        pabort("setDigitalOut() failed");
        return EXIT_FAILURE;
    }

    uint8_t doutstates = 0;
    if(getDOUTbyte(board, &doutstates) < 0)
    {
        pabort("getDOUTbyte() failed");
        return EXIT_FAILURE;
    }
    printf("DOUT states......: 0x%x\n", doutstates);

    if(enableDINint(board, 2, 'r') < 0)
    {
        pabort("enableDINint() failed");
        return EXIT_FAILURE;
    }

    if(enableDINint(board, 4, 'r') < 0)
    {
        pabort("enableDINint() failed");
        return EXIT_FAILURE;
    }

    uint8_t dinstates = 0;
    if(getDINall(board, &dinstates) < 0)
    {
        pabort("getDINall() failed");
        return EXIT_FAILURE;
    }
    printf("DIN states.......: 0x%x\n", dinstates);

    uint8_t swstates = 0;
    if(getSWstate(board, &swstates) < 0)
    {
        pabort("getSWstate() failed");
        return EXIT_FAILURE;
    }
    printf("SW states........: 0x%x\n", swstates);

    int i;
    float adcAll[8];

    if(getADCall(board, 0, adcAll, sizeof(adcAll)) < 0)
    {
        pabort("getADCall(0) failed");
        return EXIT_FAILURE;
    }
    for(i = 0; i < 8; i++)
    {
        printf("ADC(0)[%d]........: %2.2f\n", i, adcAll[i]);
    }

    if(getADCall(board, 1, adcAll, sizeof(adcAll)) < 0)
    {
        pabort("getADCall(1) failed");
        return EXIT_FAILURE;
    }
    for(i = 0; i < 8; i++)
    {
        printf("ADC(1)[%d]........: %2.2f\n", i, adcAll[i]);
    }

    //--
    return EXIT_SUCCESS;
}

int main(void)
{
    int ret;

    ret = testRelayBoard();
    if(ret != EXIT_SUCCESS)
    {
        return ret;
    }

    ret = testDAQCBoard();
    if(ret != EXIT_SUCCESS)
    {
        return ret;
    }

    puts("Finish!");

    return EXIT_SUCCESS;
}
