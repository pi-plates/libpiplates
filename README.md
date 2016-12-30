# C library for the PI-Plates I/O boards
## made for Raspberry PI (libpiplates)

## API overview

Help would be nice - ThX.

## API Defines and Functions

Help would be nice - ThX.

####PI-Plates GPIO pin and board base address structure####
```
struct config {
	// Digitial input pin - Signal to interrupt (BCM pin 22)
	uint8_t pinInterrupt

	// Digital output pin - Signal to control SPI bus transfer (BCM pin 25)
	uint8_t pinFrameControl

	// PI-Plates specific SPI board base address
	uint8_t boardBaseAddr

	// SPI BUS channel number (0=/dev/spidev0.0 or 1=/dev/spidev0.1) Default is 1
	uint8_t spiChannel
};
```

####PI-plates board handle structure####
```
struct board
{
	// GPIO/SPI configuration
    config_t config;

    // The PI-Plates board address in range 0 to 7
    uint8_t address;

    // The PI-Plates board type 1=RELAYplate 2=DAQCplate 3=MOTORplate
    uint8_t type;

    // In case of the DAQCplate the Vcc value in volts
    float vcc;
};
```

####API Version structure####
```
struct version
{
	const long major;
	const long minor;
	const long build;
	const long revision;
};
```

## Requirements and Dependencies

Help would be nice - ThX.

### Thrid party dependencies

	- wiringPi library for Raspberry PI

### Copyright (c) 2016-2017 by B. Eschrich, (E)mpire (O)f (F)un Guild
### All rights reserved.


# Changes -
