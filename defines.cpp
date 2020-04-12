#include "defines.h"

byte PIN_BUTTON_1 = 0;
byte PIN_BUTTON_2 = 9;
byte PIN_BUTTON_3 = 10;
byte PIN_BUTTON_4 = 14;
byte PIN_RELAY_1 = 12; //D6
byte PIN_RELAY_2 = 5; //D1
byte PIN_RELAY_3 = 4; //D2
byte PIN_RELAY_4 = 15; //D?
byte PIN_RELAY_5 = 255; //D?
byte PIN_LED = 13; //D?
byte PIN_RFRX = 255;
byte PIN_RFTX = 255;
byte PIN_BOOST = 255;
byte PIN_BOOST_EN = 255;
byte PIN_LATCH_COM = 255;
byte PIN_SENSOR1 = 255;
byte PIN_SENSOR2 = 255;
byte PIN_RAINSENSOR = 255;
byte PIN_FLOWSENSOR = 16; //D0

/*
 GPIO_KEY1,        // GPIO00 Button 1
     GPIO_USER,        // GPIO01 Serial RXD and Optional sensor
     GPIO_USER,        // GPIO02 Optional sensor
     GPIO_USER,        // GPIO03 Serial TXD and Optional sensor
     GPIO_REL3,        // GPIO04 Sonoff 4CH Red Led and Relay 3 (0 = Off, 1 = On)
     GPIO_REL2,        // GPIO05 Sonoff 4CH Red Led and Relay 2 (0 = Off, 1 = On)
                       // GPIO06 (SD_CLK   Flash)
                       // GPIO07 (SD_DATA0 Flash QIO/DIO/DOUT)
                       // GPIO08 (SD_DATA1 Flash QIO/DIO/DOUT)
     GPIO_KEY2,        // GPIO09 Button 2
     GPIO_KEY3,        // GPIO10 Button 3
                       // GPIO11 (SD_CMD   Flash)
     GPIO_REL1,        // GPIO12 Red Led and Relay 1 (0 = Off, 1 = On)
     GPIO_LED1_INV,    // GPIO13 Blue Led (0 = On, 1 = Off)
     GPIO_KEY4,        // GPIO14 Button 4
     GPIO_REL4,        // GPIO15 Red Led and Relay 4 (0 = Off, 1 = On)
*/
