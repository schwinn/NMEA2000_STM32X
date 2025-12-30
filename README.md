# NMEA2000_STM32

This library provides a STM32 Arduino CAN driver for the NMEA2000 library.

Library is under test. I have tested is with a DevEBox STM32F407VGT6 board. Fast packet frame order is preserved and it takes care of the frame priority when moving frames into the mailboxes. The library uses interrupts for sending and receiving.

This library is mainly a merge of these repositories: <br>
https://github.com/pazi88/STM32_CAN <br>
https://github.com/ttlappalainen/NMEA2000_Teensyx

## How to use.
To use this library, CAN module needs to be enabled in HAL drivers. If PIO is used, it's enough
to add `-DHAL_CAN_MODULE_ENABLED` as build flag. With Arduino IDE it's easiest to create `hal_conf_extra.h` file
to same folder with sketch and haven `#define HAL_CAN_MODULE_ENABLED` there. 

## Usage


    #include <NMEA2000.h>
    #include <N2kMessages.h>
    #include <NMEA2000_STM32.h>

    tNMEA2000 &NMEA2000 = *(new tNMEA2000_STM32(PB_8, PB_9));

    void setup() {
      NMEA2000.Open();
    }

    void loop() {
	    NMEA2000.ParseMessages();
    }


## Thanks

Thanks to Timo Lappalainen for the great NMEA2000 library and to Pasi Kemppainen for the CAN driver.

