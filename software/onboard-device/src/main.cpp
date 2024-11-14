#include <Arduino.h>
#include "defines.h"
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);  // set the LCD address to 0x3F for a 16 chars and 2 line display

/**
 * @brief initalize the serial monitor
 */
void initSerial() {
    Serial.begin(BAUD);
}

/**
 * @brief initialize the LCD screen
 * 
 */
void initLCD() {
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
void initLORA() {

}

void setup() {
    initSerial();
    initLCD();
    initLORA();
}

void loop() {

}