#ifndef DEFINES_H
#define DEFINES_H

#define DEBUG 1

#if DEBUG
#define debug(x)   Serial.print(x);
#define debugln(x)  Serial.println(x);
#define debugf(x, y) Serial.printf(x, y)

#else 
#define debug(x) 
#define debugln(x)  
#endif // END DEBUG


/* LCD defines  */
#define LCD_ADDR    0x27
#define LCD_ROWS    4
#define LCD_COLS    20

#define BAUD        115200
#define GPS_BAUD    9600


/* LORA pins */
#define MOSI    23
#define MISO    19
#define SCK     18
#define NSS     4
#define RESET   2
#define DIO0    32
#define LORA_FREQUENCY 868E6


#endif