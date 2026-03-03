# NMEA2000_STM32X

This library provides a STM32 Arduino CAN driver for the NMEA2000 library. It works for bxCAN and FDCAN devices.

## Details
- Library is well tested on STM32F4 and STM32U5
- Uses interrupts for sending and receiving
- Fast packet frame order is preserved 100%
- Uses own ringbuffer queue per priority. Puts packets based on priority into the FIFO queue (FDCAN) or TX mailboxes (bcXAN)

## How to use.
To use this library, CAN module needs to be enabled in HAL drivers. If PIO is used, it's enough
to add `-DHAL_CAN_MODULE_ENABLED` for bxCAN or `-DHAL_FDCAN_MODULE_ENABLED` for FDCAN as build flag. With Arduino IDE it's easiest to create `hal_conf_extra.h` file to same folder with sketch and have `#define HAL_CAN_MODULE_ENABLED` or `#define HAL_FDCAN_MODULE_ENABLED` there. 

For FDCAN, the FDCAN clock should be initialized in `SystemClock_Config()`, like `PeriphClkInit.Fdcan1ClockSelection = RCC_FDCAN1CLKSOURCE_PLL1;`.

## Usage

    #include <NMEA2000.h>
    #include <N2kMessages.h>
    #include <NMEA2000_STM32X.h>

    tNMEA2000 &NMEA2000 = *(new tNMEA2000_STM32X(PB_8, PB_9));

    void setup() {
      NMEA2000.Open();
    }

    void loop() {
	    NMEA2000.ParseMessages();
    }

## Contributing & Hardware Testing

Contributions are welcome! If you are testing this driver on a **new STM32 MCU or board**, I'd especially love to hear from you.

## Credits
This library is mainly a merge of these repositories: <br>
https://github.com/pazi88/STM32_CAN <br>
https://github.com/ttlappalainen/NMEA2000_Teensyx <br>
https://github.com/simplefoc/SimpleCANio <br>
https://github.com/duckylotl/STM32_CAN

