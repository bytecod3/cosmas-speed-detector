#include <SPI.h>
#include <LoRa.h>

const int cs_pin = 2;
const int reset_pin = 5;
const int dio = 15; // DIO0 on HOPE RF LORA MODULE - pin must have hardware interrupt

void initLORA() {
  Serial.begin(115200);
  // Initialize LoRa
  LoRa.setPins(cs_pin, reset_pin, dio);
  if (!LoRa.begin(868E6)) {
      Serial.println("Starting LoRa failed!");
      while (1);
  }
  Serial.println("LoRa Initialized");
}

void setup() {
  initLORA();
  
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
      String packetData = "";

      while (LoRa.available()) {
          packetData += (char)LoRa.read();
      }

      Serial.println("Received LoRa packet: " + packetData);
  }

}
