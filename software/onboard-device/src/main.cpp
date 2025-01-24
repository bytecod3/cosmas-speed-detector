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

TinyGPSPlus gps;
double latitude;
double longitude;
uint8_t day=0, month=0, hr=0, mint=0, sec=0;
uint16_t year=0;

float current_coords[2] = {-1.0, 30.00}; // our current location
float blackspot_list[NUM_BLACKSPOTS][2] = {
    {-1.0, 30.01},  // black spot area 1 - within blackspot radius BY 1 km
    {-1.0, 30.4},   // black spot area 2 - outside by 44 km
    {-1, 30.8},    // black spot are 3 - outside by 88 km
    {-1, 30.02}     // black spot area 4 - within the radius by 2 km, but different speed
};

float earth_radius = 6378.0;

char loraMSG[256]; // to store lora message
char number_plate[10] = "KDA-005D";
char blackspotID[] = "BLK001";
float vehicle_speed = 50;
char violation_time[50];
char vehicle_location[50];
char blackspot_location[50];
char nearest_station[50] = "SALGAA STATION"; 

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);  // set the LCD address to 0x3F for a 16 chars and 2 line display
Motor mtr_1(EN1_A, EN1_B, IN1_1, IN1_2, IN1_3, IN1_4);

char new_speed_buffer[10];
char speed_limit_message_buffer[20];
char curr_speed_buffer[20];

void init_serial();
void init_lcd();
void init_lora();
void init_motors();
void initGSM();
void sendSMS(String msg);
void updateSerial();
void read_gps();
void update_lcd();
void updateLCDCustom(char*, char*, char*, char*);
void buzz();
float getHaversineDistance(float, float, float, float);
float deg2rad(float);
void sendLORAMsg(char*);
uint8_t checkTamper();

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

void initGSM() {
    Serial.println("[+]Initializing GSM module...");
    gsm.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);

    gsm.println("AT"); // handshake
    updateSerial();
    gsm.println("AT+CSQ"); // check signal quality
    updateSerial();
    gsm.println("AT+CCID"); // confirm sim is plugged
    updateSerial();
    gsm.println("AT+CREG?"); // is it registered
    updateSerial();
    gsm.println("AT+COPS?"); // check which network
    updateSerial();

    // Serial2.print("AT+CMGF=1\r"); //SMS text mode

    // // set up the GSM to send SMS to ESP32, not the notification only
    gsm.println("AT+CNMI=2,2,0,0,0");
    delay(1000);

}

/**
 * Send SMS 
 */
void sendSMS(String msg){
    gsm.println("AT+CMGF=1");
    delay(1000);
    // updateSerial();
    gsm.println("AT+CMGS=\"+254700750148\"");
    delay(1000);
    gsm.println(msg); // send SMS
    delay(100);
    gsm.write(0x1A); //ascii code for ctrl+z, DEC->26, HEX->0x1A
    delay(1000);
    debugln("SMS Sent Successfully.");
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

void updateSerial() {
  delay(200);
  while (Serial.available()) 
  {
    gsm.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(gsm.available()) 
  {
    Serial.write(gsm.read());//Forward what Software Serial received to Serial Port
  }
}

void sendLORAMsg(char* msg) {
    debugln("[...]Sending LORA message");

    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
    delay(200);
}

// check if the speed governor has been tampered with
// simulated by using a rocker switch
uint8_t checkTamper() {
    if(!digitalRead(TAMPER_SWITCH)) {
        return 1; // tampered with
    } else {
        return 0;
    }
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
    initGSM();
    pinMode(TAMPER_SWITCH, INPUT);
    mtr_1.start(mtr_1.get_speed());
}

void loop() {
    read_gps();
    update_lcd();

    // this is purely for demo purposes
    // loop through each coordinate to check if it is within or outside the radius of the blackspot
    // each time, we check the tamper and dynamic speed reduction
    for (int i =0 ; i < NUM_BLACKSPOTS; i++) {

        if(i == 0) {
            updateLCDCustom("======================", "BLACKSPOT AREA 1", "Inside radius", "======================");
        } else if(i == 1) {
            updateLCDCustom("======================", "BLACKSPOT AREA 2", "Outside radius", "======================");
        } else if(i == 2) {
            updateLCDCustom("======================", "BLACKSPOT AREA 3", "Outside radius", "======================");
        }

        float d = getHaversineDistance(current_coords[0], current_coords[1],
                                   blackspot_list[i][0], blackspot_list[i][1]);

    
        // VIOLATION REPORTING USING GSM AND LORA TO THE AUTHORITIES 
        // HERE THE DEVICE HAS BEEN TAMPERED WITH 
        // IT IS A VIOLATION IF WE OVERSPEED INSIDE THE BLACKSPOT AREA
        if(checkTamper()) { 
            if(d < BLACKSPOT_RADIUS) {
                // get vehicle speed 
                vehicle_speed = mtr_1.get_speed();
                sprintf(curr_speed_buffer, "Vehicle speed: %d",vehicle_speed);

                if(vehicle_speed > BLACKSPOT_SPEED_LIMIT) {
                    // check which black spot we are operating from 
                    if(i == 0) {
                        sprintf(speed_limit_message_buffer,  "Speed limit: %d", BLACKSPOT1_SPEED);
                    } else if(i == 2) {
                        sprintf(speed_limit_message_buffer,  "Speed limit: %d", BLACKSPOT2_SPEED);
                    } else {
                        sprintf(speed_limit_message_buffer,  "Speed limit: %d", BLACKSPOT3_SPEED);
                    }
                    
                    updateLCDCustom("Device tampered",speed_limit_message_buffer, curr_speed_buffer,"");
                    delay(2000);
                    updateLCDCustom("Blackspot Area!", "Speed violation", "Reporting...", "");
                    buzz();

                    // this is a violation 
                    sprintf(violation_time, "[%d:%d:%d]-[%d/%d/%d]", hr, mint, sec,day,month,year);
                    sprintf(vehicle_location, "[%.2f,%.2f]", latitude, longitude);
                    sprintf(blackspot_location, "[%.2f,%.2f]", blackspot_list[0][0], blackspot_list[0][0]); // first blackspot in the list

                    // compose lora packet 
                    sprintf(loraMSG, "plate:%s, speed:%f, time:%s, vehicle_location:%s, blackspot_id:%s, blackspot-location:%s, station:%s",
                            number_plate,// vehicle number plate
                            vehicle_speed,// speed
                            violation_time,// time violated
                            vehicle_location, // location of the vehicle 
                            blackspotID,// blackspot ID
                            blackspot_location,// blackspot coordinates 
                            nearest_station // nearest police station
                    );

                    // send lora message on violation 
                    sendLORAMsg(loraMSG);

                    // send GSM messages 
                    
                }
            }

        // DEVICE HAS NOT BEEN TAMPERED WITH HERE
        // DYNAMIC REDUCTION IN VEHICLE SPEED AT BLACKSPOT USING THE HAVERSINE FORMULA
        } else { 

            if(d < BLACKSPOT_RADIUS) { // if we are inside the blackspot
                // beep
                // show on screen
                if(i == 0) {
                    sprintf(speed_limit_message_buffer,  "Speed limit: %d", BLACKSPOT1_SPEED);
                } else if(i == 1) {
                    sprintf(speed_limit_message_buffer,  "Speed limit: %d", BLACKSPOT2_SPEED);
                } else if(i == 2) {
                    sprintf(speed_limit_message_buffer,  "Speed limit: %d", BLACKSPOT3_SPEED);
                } else if(i == 3) {
                    sprintf(speed_limit_message_buffer,  "Speed limit: %d", BLACKSPOT4_SPEED);
                } 

                updateLCDCustom("Blackspot Area!",
                                speed_limit_message_buffer,
                                "Reduce speed" ,
                                "Auto reducing..");

                // decelerate
                if(i == 0) {
                    mtr_1.set_speed(BLACKSPOT1_SPEED);
                } else if (i == 1) {
                    mtr_1.set_speed(BLACKSPOT2_SPEED);
                } else if(i == 2) {
                    mtr_1.set_speed(BLACKSPOT3_SPEED);
                } else if(i == 3) {
                    mtr_1.set_speed(BLACKSPOT4_SPEED);
                }
                
                mtr_1.move_forward();

                sprintf(new_speed_buffer, "%d", mtr_1.get_speed());

                // display new sped on the LCD screen 
                updateLCDCustom("Regulated speed:", new_speed_buffer, "", "");

            } else {
                // maintain
                // display road clear
                updateLCDCustom("Road clear.",
                                "Safe journey!",
                                "",
                                "");

                mtr_1.set_speed(NORMAL_SPEED);
                mtr_1.move_forward();
            }   

        }

        updateLCDCustom("===================", "DONE CHECKING", "==================", "");
        buzz();
    } // end of for loop
}