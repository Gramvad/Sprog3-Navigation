/*  
 *  Authors: Group 5 3rd Semester BEng Mechatronics 2024
 *
 *  
 *  "Land Module"
 *  
 *  V1 - Initial Prototype
 *  
 */
 
/* For Telemetry */
#include <SPI.h>
#include <LoRa.h>
// Define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2
 
int counter = 0;
 
/* Programme function prototypes */
 
// Send
void send_waypoint(void);
void send_command(void);
void send_EmergencyStop(void);
 
// Receive
void receive_status(void);
void receive_EmergencySignal(void);
 
/* Electronic components setup function prototypes */
void setup_Telemetry(void);
 
void setup(){
 
  Serial.begin(115200);
  setup_Telemetry();
}
 
void loop(){
 
  // Try to parse packet
  int packetSize = LoRa.parsePacket();
  if(packetSize){
 
    // Read packet
    String message_type = LoRa.readStringUntil(',');
 
    if(message_type == "S"){
 
      receive_Status();
    }
    else if(message_type == "E"){
 
      receive_EmergencySignal();
    }
  }
 
  // Sending data
  Serial.println("What do you want to send?");
  Serial.println("\n1. Waypoint (Latitude & longitude");
  Serial.println("2. Manual Command (State & distance");
  Serial.println("3. Emergency stop");
  Serial.println("4. Send nothing");
 
  // Wait for input from the user
  while(!Serial.available());
  int send = Serial.parseInt();
 
  switch(send){
 
    case 1:
      send_waypoint();
      break;
    case 2:
      send_ManualCommand();
      break;
    case 3:
      send_EmergencyStop();
      break;
    case 4:
      break;
    default: 
      Serial.println("Not a valid input.");
      break;
  }
}
 
/* Programme functions */
 
// Send
void send_Waypoint(void){
 
  // Ask latitude
  Serial.println("\nType latitude:");
  Serial.println("\tHigher (+): North");
  Serial.println("\tLower  (-): South");
 
  while(!Serial.available()); // Wait for input from the user
  double latitude = Serial.parseInt();
 
  // Ask longitude
  Serial.println("\nType latitude:");
  Serial.println("\tHigher (+): East");
  Serial.println("\tLower  (-): West");
 
  while(!Serial.available()); // Wait for input from the user
  double longitude = Serial.parseInt();
 
  // Send latitude
  LoRa.beginPacket();
  LoRa.print(latitude);
  LoRa.endPacket();
 
  // Send longitude
  LoRa.beginPacket();
  LoRa.print(longitude);
  LoRa.endPacket();
}
void send_ManualCommand(void){
 
  // Ask heading (state)
  Serial.println("\nType a heading (state):");
  Serial.println("\n1. Forward");
  Serial.println("2. Backward");
  Serial.println("3. Left");
  Serial.println("4. Right");
  Serial.println("5. Rotate left");
  Serial.println("6. Rotate right");
  Serial.print(">> ");
 
  while(!Serial.available()); // Wait for input from the user
  int heading = Serial.parseInt();
 
  // Ask distance to be travelled (in m)
  Serial.print("\nType a distance to be travelled (in meters): ");
 
  while(!Serial.available()); // Wait for input from the user
  int distance = Serial.parseInt();
 
  // Send heading
  LoRa.beginPacket();
  LoRa.print(heading);
  LoRa.endPacket();
 
  // Send distance
  LoRa.beginPacket();
  LoRa.print(distance);
  LoRa.endPacket();
}
void send_EmergencyStop(void){
 
  String Kill_signal = "Kill_signal";
 
  LoRa.beginPacket();
  LoRa.print(Kill_signal);
  LoRa.endPacket();
 
  // Send Kill_signal
}
 
// Receive
void receive_Status(void){
 
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
void receive_EmergencySignal(void){
 
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
 
/* Electronic components setup functions  */
void setup_Telemetry(void){
 
  while (!Serial);
  Serial.println("LoRa Sender");
 
  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
 
  // 868E6 for Europe
  while (!LoRa.begin(868E6)) {
    Serial.println(".");
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3); // Sync word to match the receiver
  Serial.println("LoRa Initializing OK!");
}