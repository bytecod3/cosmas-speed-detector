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

#define BAUD            115200
#define GPS_BAUD        9600
#define GSM_BAUD        9600
#define GPS_RX          26
#define GPS_TX          25
#define BUZZER          2
#define BUZZ_DELAY      200
#define STARTUP_DELAY   3000

/* LORA pins */
#define MOSI    23
#define MISO    19
#define SCK     18
#define NSS     15
#define RESET   27
#define DIO0    32
#define LORA_FREQUENCY 868E6

#define DECEL_DELAY 20  // milliseconds to delay when decelerating

#define EN1_A 14
#define EN1_B 13
#define IN1_1 12
#define IN1_2 5
#define IN1_3 4
#define IN1_4 33
#define M1_PWM
#define MATH_PI 3.142
#define BLACKSPOT_RADIUS 3
#define BLACKSPOT_SPEED_LIMIT 120 // PWM values
#define NORMAL_SPEED 240

#define NUM_BLACKSPOTS 4

// for different blackspot areas, different speed 
#define BLACKSPOT1_SPEED    90 // A BLACKSPOT
#define BLACKSPOT2_SPEED    100
#define BLACKSPOT3_SPEED    200
#define BLACKSPOT4_SPEED    120 // A BLACKSPOT

/*============================GSM VARIABLES=============================*/
#define GSM_RX 16
#define GSM_TX 17

#define TAMPER_SWITCH 36

#endif