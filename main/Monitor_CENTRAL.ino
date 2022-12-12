//CENTRAL


#include <ArduinoBLE.h>

double PIDout_value;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  Serial.println("Bluetooth® Low Energy Central - PIDmonitor");

  // start scanning for peripherals
  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "PID_Monitor") {
      return;
    }

    BLE.stopScan();

    readValues(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
  }
}

void readValues(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }


  BLECharacteristic PIDoutput = peripheral.characteristic("2A19");

  if (!PIDoutput) {
    Serial.println("Peripheral does not a characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    PIDoutput.read();
      float floatValue;
      PIDoutput.readValue( &floatValue, 4 ); //read the value with bluetooth
      Serial.println( floatValue );

  }

  Serial.println("Peripheral disconnected");
}
