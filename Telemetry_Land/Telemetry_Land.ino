/*  
 *  Authors: Group 5 3rd Semester BEng Mechatronics 2024
 *
 *  
 *  "Telemetry Test 2.1: Land Module"
 *  
 *  
 *  
*/ 
 
/* For Telemetry */
#include <SPI.h>
#include <LoRa.h>
// Define the pins used by the transceiver module
#define ss 17
#define rst 20
#define dio0 21
 
/* Pinout */
 
/*
LoRa        - ESP32 
 
 RX-SWITCH (4) - RX   (20)
 TX-SWITCH (5) - SCK  (10)
 DIO0      (6) - MISO (8)
 DIO1      (7) - MOSI (7)
 
LoRa        - Raspberry Pi Pico
 
 RX-SWITCH (4) - GP20/Reset (26)
 TX-SWITCH (5) - GP18/SCK   (24)
 DIO0      (6) - RX         (21)
 DIO1      (7) - GP19/TX    (7)
*/
 
/* Programme function prototypes */
void processStatus(void);
void handleEmergency(void);
// Setups
void setup_Telemetry(void);
 
void setup(){
 
  Serial.begin(9600); // Check if works with 115200
  setup_Telemetry();
}
 
void loop(){
 
  int packetSize = LoRa.parsePacket();
 
  if(packetSize){
 
    String messageType = LoRa.readStringUntil(',');
 
    if(messageType == "S"){
      processStatus();
    }
    else if(messageType == "E"){
 
      handleEmergency();
    }
  }
}
 
void processStatus(void){
 
  String latitude = LoRa.readStringUntil(',');
  String longitude = LoRa.readStringUntil(',');
  String currentProcess = LoRa.readStringUntil(',');
  String state = LoRa.readStringUntil('\n');
 
  Serial.println("Status Update:");
  Serial.print("Latitude: ");
  Serial.println(latitude);
  Serial.print("Longitude: ");
  Serial.println(longitude);
  Serial.print("Process: ");
  Serial.println(currentProcess);
  Serial.print("State: ");
  Serial.println(state);
}
void handleEmergency(void){
 
  String cause = LoRa.readStringUntil('\n');
 
  Serial.println("Emergency Signal Received!");
  Serial.print("Cause: ");
  Serial.println(cause);
  Serial.println("Do you want to send an emergency stop? (Y/N)");
 
  while (!Serial.available());
  char response = Serial.read();
  if (response == 'Y' || response == 'y'){
 
    LoRa.beginPacket();
    LoRa.print("STOP");
    LoRa.endPacket();
    Serial.println("Emergency Stop Sent");
  }
}
 
/* ELectronic components setup functions */
void setup_Telemetry(void){
 
  while(!Serial);
  LoRa.setPins(ss, rst, dio0);
  // LoRa.setPins(rst, dio0);
 
  if(!LoRa.begin(868E6)){
 
    Serial.println("Starting LoRa failed!");
    while(1);
  }
  Serial.println("Land Module Initialized");
}
 