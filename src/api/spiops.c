/*
 ============================================================================
 Name        : spiops.c
 Author      : B. Eschrich
 Version     :
 Copyright   : Copyright (c) 2016-2017 by B. Eschrich (EoF)
 Description : PI-Plates SPI bus operations
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <wiringPiSPI.h>

#include "spiops.h"

/*GCC specific: not all static's used here */
#pragma GCC diagnostic ignored "-Wunused-function"

/* Enable/Disable I/O trace */
#ifdef TRACE_SPI_OPS
#define __TRACE_SPI_OPS			1
#else
#define __TRACE_SPI_OPS			0
#endif

/* SPI device initialization parameters */
#define PP_SPI_BUS_SPEED		500000

/* Using
 * 0 = /dev/spidev0.0
 * 1 = /dev/spidev0.1 */
static const char* devices[2] =
{
    "/dev/spidev0.0",
    "/dev/spidev0.1"
};

/**
 *
 */
static void hex_dump(const void* src, size_t length, size_t line_size, char* prefix)
{
    int i = 0;
    const unsigned char* address = src;
    const unsigned char* line = address;
    unsigned char c;
    printf("%s | ", prefix);
    while(length-- > 0)
    {
        printf("%02X ", *address++);
        if(!(++i % line_size) || (length == 0 && i % line_size))
        {
            if(length == 0)
            {
                while(i++ % line_size)
                {
                    printf("__ ");
                }
            }
            printf(" | ");  /* right close */
            while(line < address)
            {
                c = *line++;
                printf("%c", (c < 33 || c == 255) ? 0x2E : c);
            }
            printf("\n");
            if(length > 0)
            {
                printf("%s | ", prefix);
            }
        }
    }
}

static int spiError(int code, const char* message, ...)
{
    va_list argp ;
    char buffer [1024] ;


    va_start(argp, message) ;
    vsnprintf(buffer, 1023, message, argp) ;
    va_end(argp) ;

    fprintf(stderr, "%s", buffer) ;

    return (code * -1);
}

/**
 *
 */
static int spiSetup(int fd, uint32_t* xferspeed, uint32_t* xfermode)
{
    int ret;
    uint8_t bits = 8;
    uint32_t mode = *xfermode;
    uint32_t mode32 = 0;
    uint32_t speed = (*xferspeed <= 0 ? PP_SPI_BUS_SPEED : *xferspeed);
    uint32_t maxspeed = 0;

    /*
     * SPI transfer mode
     */
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if(ret == -1)
    {
        return spiError(1020, "Can't set spi transfer mode.\n");
    }
    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
    if(ret == -1)
    {
        return spiError(1021, "Can't get spi transfer mode.\n");
    }

    /*
     * SPI full transfer mode
     */
#ifdef SPI_IOC_WR_MODE32
    //ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode32);
    //if (ret == -1) {
    //	puts("Can't set spi full mode");
    //	return 0;
    //}
#endif
#ifdef SPI_IOC_RD_MODE32
    ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode32);
    if(ret == -1)
    {
        puts("Can't get spi full mode");
        return 0;
    }
#endif

    /*
     * Bits per word
     */
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if(ret == -1)
    {
        return spiError(1000, "Can't set bits per word.\n");
    }
    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if(ret == -1)
    {
        return spiError(1001, "Can't get bits per word.\n");
    }

    /*
     * Max speed hz
     */
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &maxspeed);
    if(ret == -1)
    {
        return spiError(1010, "Can't get max speed hz");
    }
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if(ret == -1)
    {
        return spiError(1011, "Can't set max speed hz");
    }
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if(ret == -1)
    {
        return spiError(1012, "Can't get speed hz");
    }

    if(__TRACE_SPI_OPS)
    {
        printf("spiSetup():\n");
        printf("- SPI mode......: 0x%x M32=0x%x\n", mode, mode32);
        printf("- Bits per word.: %d\n", bits);
        printf("- Xfer speed....: %d Hz (%d KHz) [Max %d Hz (%d KHz)]\n",
               speed, speed/1000, maxspeed, maxspeed/1000);
    }

    // return bus speed and mode
    *xferspeed = speed;
    *xfermode = mode;

    // wait here?
    usleep(1000);

    // success
    return 0;
}

/**
 *
 */
static int spiRead(int fd, uint8_t const* buff, size_t len, uint32_t speed, uint32_t mode, uint32_t delay)
{
    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));

    spi.tx_buf = (unsigned long) NULL;
    spi.rx_buf = (unsigned long) buff;
    spi.len = len;
    spi.delay_usecs = delay;
    spi.speed_hz = speed;
    spi.bits_per_word = 8;
    spi.cs_change = 0;

    // adapted from py_spidev/spidev_module
#ifdef SPI_IOC_WR_MODE32
    spi.tx_nbits = 0;
#endif
#ifdef SPI_IOC_RD_MODE32
    spi.rx_nbits = 0;
#endif

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
    if(ret < 1)
    {
        return spiError(1100, "spiRead(): Can't send spi message");
    }

    if(mode & SPI_CS_HIGH)
    {
        ret = read(fd, (void*) &buff[0], 0);
    }

    if(__TRACE_SPI_OPS)
    {
        hex_dump(buff, len, 32, "RX");
    }

    // success
    return 0;
}

/**
 *
 */
static int spiWrite(int fd, uint8_t const* buff, size_t len, uint32_t speed, uint32_t delay)
{
    if(__TRACE_SPI_OPS)
    {
        hex_dump(buff, len, 32, "TX");
    }

    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));

    spi.tx_buf = (unsigned long) buff;
    spi.rx_buf = (unsigned long) NULL;
    spi.len = len;
    spi.delay_usecs = delay;
    spi.speed_hz = speed;
    spi.bits_per_word = 8;
    spi.cs_change = 0;

    // adapted from py_spidev/spidev_module
#ifdef SPI_IOC_WR_MODE32
    spi.tx_nbits = 0;
#endif
#ifdef SPI_IOC_RD_MODE32
    spi.rx_nbits = 0;
#endif

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
    if(ret < 1)
    {
        return spiError(1200, "spiWrite(): Can't send spi message");
    }

    usleep(1000);

    // success
    return 0;
}

/*
 *
 */
static int spiXfer(int fd, uint8_t const* tx, uint8_t const* rx, size_t len, uint32_t speed, uint32_t mode, uint32_t delay)
{
    if(__TRACE_SPI_OPS)
    {
        hex_dump(tx, len, 32, "TX");
    }

    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));

    spi.tx_buf = (unsigned long) tx;
    spi.rx_buf = (unsigned long) rx;
    spi.len = len;
    spi.delay_usecs = delay;
    spi.speed_hz = speed;
    spi.bits_per_word = 8;
    spi.cs_change = 0;

    // adapted from py_spidev/spidev_module
#ifdef SPI_IOC_WR_MODE32
    spi.tx_nbits = 0;
#endif
#ifdef SPI_IOC_RD_MODE32
    spi.rx_nbits = 0;
#endif

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
    if(ret < 1)
    {
        return spiError(1300, "spiXfer(): Can't send spi message");
    }

    if((mode & SPI_CS_HIGH) && rx != NULL)
    {
        char temp[1] = "";
        ret = read(fd, &temp[0], 0);
    }

    if(__TRACE_SPI_OPS)
    {
        hex_dump(rx, len, 32, "RX");
    }

    // success
    return 0;
}

/**
 *
 */
int spiCommand(const board_command_t* pCommand)
{
    if(__TRACE_SPI_OPS)
    {
        printf("spiCommand(): channel=0x%02x address=0x%02x command=0x%02x p1=0x%02x p2=0x%02x size=%d device=%s\n",
               pCommand->channel,
               pCommand->address,
               pCommand->command,
               pCommand->param1,
               pCommand->param2,
               pCommand->size,
               devices[pCommand->channel]);
    }

    // default return fail
    int ret = -1;

    // 4 parameters
    uint8_t txbuff[4] =
    {
        pCommand->address,
        pCommand->command,
        pCommand->param1,
        pCommand->param2,
    };
    size_t size = sizeof(txbuff);

    int fd = open(devices[pCommand->channel], O_RDWR);
    if(fd < 0)
    {
        ret = spiError(1400, "Unable to open SPI bus device. Make sure SPI is enabled by raspi-config tool.");
        goto cleanup;
    }

    //SPI_CS_HIGH | SPI_CPHA | SPI_CPOL | SPI_RX_QUAD | SPI_TX_QUAD / SPI_NO_CS;
    uint32_t mode = SPI_CPHA | SPI_RX_DUAL | SPI_TX_DUAL | SPI_NO_CS;
    uint32_t speed = PP_SPI_BUS_SPEED;
    ret = spiSetup(fd, &speed, &mode);
    if(ret < 0)
    {
        goto cleanup;
    }
    uint32_t wr_speed = (speed == PP_SPI_BUS_SPEED ? speed - 200000 : speed);

    if(__TRACE_SPI_OPS)
    {
        printf("spiCommand(): fd=%d size=%d wr_speed=%d mode=0x%02x addr=%d\n", fd, size, wr_speed, mode, txbuff[0]);
    }

    // write RELAYplate command
    ret = spiWrite(fd, txbuff, size, wr_speed, 60);
    if(ret < 0)
    {
        goto cleanup;
    }

    // Read command response if necessary
    if(pCommand->result != NULL && pCommand->size > 0)
    {
        int i = 0;
        uint8_t byte[1] = {0x00};
        while(i < pCommand->size)
        {
            ret = spiRead(fd, &byte[0], 1, speed, mode, 20);
            if(ret < 0)
            {
                goto cleanup;
            }
            // stop at zero terminator
            if(((byte[0] == 0x0) && pCommand->stopAt0x0))
            {
                break;
            }
            pCommand->result[i] = byte[0];
            usleep(700);
            i++;
        }
    }

    usleep(1000);

    // success
    ret = 0;

cleanup:
    //cleanup stuff
    if(fd >= 0)
    {
        close(fd);
    }

    // finish
    return ret;
}
