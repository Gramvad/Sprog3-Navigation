/*  
 *  Authors: Group 5 3rd Semester BEng Mechatronics 2024
 *
 *  
 *  "Telemetry Test 2.2: Boat Module"
 *  
 *  
 *  
*/ 
 
#include <SoftwareSerial.h> // Importing SoftwareSerial library
 
static const int RXPin = 5, TXPin = 4; // Defining software serial rx and tx pin
SoftwareSerial ss(RXPin, TXPin);
 
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
 
/* Hardcoded variables */
String currentProcess = "following waypoint";
String currentState = "Forward";
float latitude = 12.3456;
float longitude = 65.4321;
bool emergencyStop = false;
 
/* Programme function prototypes */
void setup_Telemetry(void);
 
void setup(){
 
  Serial.begin(9600); // Check if works with 115200
  setup_Telemetry();
}
 
void loop(){
 
  if(emergencyStop){
 
    Serial.println("Emergency Stop Triggered!");
    while (1);
  }
 
  sendStatusUpdate();
  receiveCommand();
  delay(1000);
}
 
void sendStatusUpdate() {
    LoRa.beginPacket();
    LoRa.print("S,");
    LoRa.print(latitude, 6);
    LoRa.print(",");
    LoRa.print(longitude, 6);
    LoRa.print(",");
    LoRa.print(currentProcess);
    LoRa.print(",");
    LoRa.print(currentState);
    LoRa.endPacket();
    Serial.println("Status Update Sent");
}
 
void receiveCommand() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        String command = LoRa.readStringUntil('\n');
        if (command.startsWith("STOP")) {
            emergencyStop = true;
        } else if (command.startsWith("WAYPOINT")) {
            handleWaypoint(command);
        } else if (command.startsWith("MANUAL")) {
            handleManualCommand(command);
        }
    }
}
 
void handleWaypoint(String command) {
    // Parse waypoint command (e.g., "WAYPOINT,12.3456,65.4321")
    int commaIndex = command.indexOf(',');
    String newLatitude = command.substring(commaIndex + 1, command.indexOf(',', commaIndex + 1));
    String newLongitude = command.substring(command.indexOf(',', commaIndex + 1) + 1);
 
    latitude = newLatitude.toFloat();
    longitude = newLongitude.toFloat();
    currentProcess = "holding waypoint";
 
    Serial.println("Waypoint Updated:");
    Serial.print("Latitude: ");
    Serial.println(latitude);
    Serial.print("Longitude: ");
    Serial.println(longitude);
}
 
void handleManualCommand(String command) {
 
  // Parse manual command (e.g., "MANUAL,Heading,Distance")
  int commaIndex = command.indexOf(',');
  String heading = command.substring(commaIndex + 1, command.indexOf(',', commaIndex + 1));
  String distance = command.substring(command.indexOf(',', commaIndex + 1) + 1);
 
  Serial.println("Manual Command Received:");
  Serial.print("Heading: ");
  Serial.println(heading);
  Serial.print("Distance: ");
  Serial.println(distance);
 
  currentProcess = "manual control";
}
 
/* ELectronic components setup functions */
void setup_Telemetry(void){
 
  while(!Serial);
  LoRa.setPins(ss, rst, dio0);
 
  if(!LoRa.begin(868E6)){
 
    Serial.println("Starting LoRa failed!");
    while(1);
  }
  Serial.println("Land Module Initialized");
}
 