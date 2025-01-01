#include <Arduino.h>
#include "defines.h"
#include "motor-driver.h"
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <LoRa.h>
#include <SoftwareSerial.h>

HardwareSerial gsm(1);
HardwareSerial gpsSerial(2);
//SoftwareSerial gpsSerial(GPS_TX, GPS_RX);

TinyGPSPlus gps;

double blackspot_list[3][2] = {{}, {}, {}};

double latitude;
double longitude;
uint8_t day=0, month=0, hr=0, mint=0, sec=0;
uint16_t year=0;

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);  // set the LCD address to 0x3F for a 16 chars and 2 line display
Motor mtr_1(EN1_A, EN1_B, IN1_1, IN1_2, IN1_3, IN1_4);

void init_serial();
void init_lcd();
void init_lora();
void init_motors();
void read_gps();
void update_lcd();
void buzz();

void init_motors() {
    mtr_1.init_motor_pins();
}

/**
 * @brief initialize the serial monitor
 */
void init_serial() {
    Serial.begin(BAUD);
}

void initGPS(){
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX,GPS_TX);
    debugln("Initialized GPS");
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
    lcd.print("Hello Cosmas!");

    lcd.setCursor(6,1);
    lcd.print("Welcome.");

    delay(STARTUP_DELAY);

}

void init_buzzer() {
    pinMode(BUZZER, OUTPUT);
}

void buzz() {
    digitalWrite(BUZZER, HIGH);
    delay(BUZZ_DELAY);

    digitalWrite(BUZZER, LOW);
    delay(BUZZ_DELAY);

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

void read_gps() {
//    debugln(gpsSerial.read());
    gps.encode(gpsSerial.read());

    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());

        if (gps.location.isUpdated()){
            // Get location
            debug("Location: ");
            if(gps.location.isValid()) {
                latitude = gps.location.lat();
                Serial.print(gps.location.lat(), 2);
                debug(F(","));

                longitude = gps.location.lng();
                Serial.print(gps.location.lng(), 2);

            } else {
                debug(F("INVALID"));
            }

            // Get time and date
            debug(F("Date/time: "));
            if(gps.date.isValid()) {
                month = gps.date.month();
                debug(month);
                debug(F("/"));

                day = gps.date.day();
                debug(day);
                debug(F("/"));

                year = gps.date.year();
                debug(year);
            } else {
                debug(F("INVALID"));
            }

            // time
            debug(F(" "));
            if (gps.time.isValid()) {

                hr = gps.time.hour();
                if (hr < 10) debug(F("0"));
                debug(hr);

                mint = gps.time.minute();
                if (mint < 10) debug(F("0"));
                debug(mint);

                sec = gps.time.second();
                if (sec < 10) debug(F("0"));
                debug(sec);

            } else {
                debug(F("INVALID"));
            }

            debugln();
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
    init_buzzer();
    init_motors();
    initGPS();
    buzz();
    buzz();
}

void loop() {
    read_gps();
    update_lcd();
}