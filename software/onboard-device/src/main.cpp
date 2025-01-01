#include <Arduino.h>
#include "defines.h"
#include "motor-driver.h"
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <math.h>

HardwareSerial gsm(1);
HardwareSerial gpsSerial(2);
//SoftwareSerial gpsSerial(GPS_TX, GPS_RX);

TinyGPSPlus gps;
double latitude;
double longitude;
uint8_t day=0, month=0, hr=0, mint=0, sec=0;
uint16_t year=0;

float blackspot_list[3][2] = {{-1.0, 30.01}, {-1.0, 40.02}, {-1, 50.02}};
float current_coords[2] = {-1.0, 30.00};
float earth_radius = 6378.0;

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);  // set the LCD address to 0x3F for a 16 chars and 2 line display
Motor mtr_1(EN1_A, EN1_B, IN1_1, IN1_2, IN1_3, IN1_4);

void init_serial();
void init_lcd();
void init_lora();
void init_motors();
void read_gps();
void update_lcd();
void updateLCDCustom(char*, char*, char*, char*);
void buzz();
float getHaversineDistance(float, float, float, float);
float deg2rad(float);

float getHaversineDistance(float x1, float y1, float x2, float y2) {
    /**
     * Haversine formula for distance between 2 coordinates
     * x1, y1 - black-spot coordinates
     * x2, y2 - current vehicle coordinates
     */

    float del_lat_rad = deg2rad(x2-x1);
    float del_long_rad = deg2rad(y2-y1);
    float hav_lat = sin((del_lat_rad/2))*sin((del_lat_rad/2));
    float hav_long = sin((del_long_rad/2))*sin((del_long_rad/2));
    float hav_d = hav_lat + (cos(deg2rad(x1))*cos(deg2rad(x2))) * hav_long;
    float d_km = 2 * earth_radius * asin(sqrt(hav_d));
    return d_km;
}

float deg2rad(float a) {
    return a * (MATH_PI/180);
}

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

void updateLCDCustom(char* msg1, char* msg2, char* msg3, char* msg4) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(msg1);
    lcd.setCursor(0, 1);
    lcd.print(msg2);
    lcd.setCursor(0, 2);
    lcd.print(msg3);
    lcd.setCursor(0, 3);
    lcd.print(msg4);

    delay(2000);
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

    mtr_1.start(mtr_1.get_speed());

}

void loop() {
    read_gps();
    update_lcd();

    float d = getHaversineDistance(current_coords[0], current_coords[1],
                                   blackspot_list[0][0], blackspot_list[0][1]);
    if(d < BLACKSPOT_RADIUS) {
        // beep
        // show on screen
        updateLCDCustom("Blackspot Area!",
                        "Reduce speed",
                        "..." ,
                        "Auto reducing..");
        // decelerate
        mtr_1.set_speed(BLACKSPOT_SPEED);
        mtr_1.move_forward();
    } else {
        // maintain
        // display road clear
        updateLCDCustom("Road clear!",
                        "",
                        "",
                        "");

        mtr_1.set_speed(NORMAL_SPEED);
    }
}