//PERF

#include <ArduinoBLE.h>

float driveBias = 0;
float turnBias = 0;

BLEService driveService("19B10000-E8F2-537E-4F6C-D104768A1214");
//BLEStringCharacteristic pidValuesBLE("2A20", BLERead | BLENotify, 20);
BLEFloatCharacteristic driveCharac("19B10000-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify);
BLEFloatCharacteristic turnCharac("19B10000-E8F2-537E-4F6C-D104768A1216", BLERead | BLENotify);



void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  // configure the button pin as input
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1)
      ;
  }
  // initialize the Bluetooth® Low Energy hardware
  BLE.setLocalName("Robot_controller");
  BLE.setAdvertisedService(driveService);       // add the service UUID
  driveService.addCharacteristic(driveCharac);  // add the battery level characteristic
  driveService.addCharacteristic(turnCharac);   // add the battery level characteristic

  BLE.addService(driveService);  // Add the battery service
  driveCharac.writeValue(driveBias);
  turnCharac.writeValue(turnBias);

  BLE.advertise();
  //Keyboard.begin();
  Serial.println("Setup complete");
}

void loop() {
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected()) {
      if (Serial.available()) {
        char inChar = Serial.read();
        if (inChar == '8') {  //forward
          driveBias = 1;
        } 

        if (inChar == '2') {  //backward
          driveBias = -1;
        }
        
        if (inChar == '5') {  //backward
          driveBias = 0;
        }


        if (inChar == '6') {  //right
          turnBias = 2;
        } 

        if (inChar == '4') {  //left
          turnBias = 2;
        } 

        Serial.println(driveBias);
        Serial.println(inChar);

        if (central) {
          driveCharac.writeValue(driveBias);
          turnCharac.writeValue(turnBias);

        } else {
          Serial.println("Couldnt send to robot");
        }
      }
    }
    // when the central disconnects, turn off the LED:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
