#include <Arduino.h>
#include "defines.h"
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <LoRa.h>

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
float latitude, longitude;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);  // set the LCD address to 0x3F for a 16 chars and 2 line display

void init_serial();
void init_lcd();
void init_lora();
void get_gps();
void update_lcd();

/**
 * @brief initalize the serial monitor
 */
void init_serial() {
    Serial.begin(BAUD);
    gpsSerial.begin(GPS_BAUD);
}

/**
 * @brief initialize the LCD screen
 * 
 */
void init_lcd() {
    lcd.init();
    lcd.clear();
    lcd.backlight(); // turn on backlight

    // Welcome message
    lcd.setCursor(4,0); // character 2 on line 0
    lcd.print("Hello there!");

    lcd.setCursor(6,1);
    lcd.print("Welcome.");
}

/**
 * @brief init LORA 
 */
void init_lora() {
    LoRa.setPins(NSS, RESET, DIO0);
    if(!LoRa.begin(LORA_FREQUENCY)) {
        debugln("Lora failed to start");
    } else {
        debugln("Lora started OK!");
    }
}

void get_gps() {
    // Get location
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());

        if (gps.location.isUpdated()) {
            latitude = gps.location.lat();
            longitude = gps.location.lng();

            Serial.print("Latitude= "); 
            Serial.print(latitude, 2);
            Serial.print(" Longitude= "); 
            Serial.println(longitude, 2);
             

            // time - in 24 hr format
            // Hour (0-23) (u8) 
            Serial.print(gps.time.hour());Serial.print(":");
            // Minute (0-59) (u8)
            Serial.print(gps.time.minute());Serial.print(":");
            // Second (0-59) (u8)
            Serial.print(gps.time.second());
            Serial.println();
            // 100ths of a second (0-99) (u8)
        } 
    }
}

void update_lcd() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Lat:");
    lcd.print(latitude);
    lcd.setCursor(0,1);
    lcd.print("Long:");
    lcd.print(longitude);
    delay(2000);
    lcd.clear();
}

void setup() {
    init_serial();
    init_lcd();
    init_lora();
}

void loop() {

    get_gps();
    update_lcd();

}