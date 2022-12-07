//PERF

#include <ArduinoBLE.h>

String pidValues ="24.5;0.0095;655"; //format Kp;Ki;Kd

BLEService customService("19B10000-E8F2-537E-4F6C-D104768A1214"); 
BLEStringCharacteristic pidValuesBLE("2A19", BLERead | BLENotify,20);


void setup() {
  Serial.begin(9600);
  while (!Serial);

  // configure the button pin as input

  // initialize the Bluetooth® Low Energy hardware
  BLE.setLocalName("PID_Monitor");
  BLE.setAdvertisedService(customService); // add the service UUID
  customService.addCharacteristic(pidValuesBLE); // add the battery level characteristic
  BLE.addService(customService); // Add the battery service
  pidValuesBLE.writeValue(pidValues); 
  BLE.advertise();


}

void loop() {
  BLEDevice central = BLE.central();
  while(Serial.available()){
    pidValues = Serial.readString();  

    if(central){
      pidValuesBLE.writeValue(pidValues);
    }else{
      Serial.println("Couldnt send to robot");
    }
  }

}


