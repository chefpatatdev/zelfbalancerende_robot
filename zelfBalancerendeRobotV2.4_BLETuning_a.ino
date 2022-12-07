#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

float aX, aY, aZ;  //acceleratie rond de assen
float gX, gY, gZ;  //hoeksnelheid rond de assen
float aAngle = 0;  //hoek berekend door de accelerometer
float gAngle = 0;  //hoek berekend door de gyroscoop

float alpha = 0.993;  //waarde die bepaald hoe zwaar de gyroscoop hoek doorweegt in verhouding tot accelerometer
float minPWM = 38;  //42,  De PWM waarde waarbij de motoren ongeveer beginnen te werken
float maxPWM = 255;

unsigned long currentTime = 0, deltaTime = 0, previousTime = 0;  //voor tijd tussen 2 loops bij te houden

double currentAngle = 0;
double previousAngle = 0;

int enA = 7;
int in1 = 9;
int in2 = 8;

int enB = 6;
int in3 = 4;
int in4 = 5;

// PID

const float PIDMaster = 1;
const float Kp = 31 * PIDMaster; //25.5
const float Ki = 0.225 * PIDMaster; //0.097 , 0.12
const float Kd = 1000 * PIDMaster; //520 , 540


double errorSize=0, lastError=0;
double cumError, rateError;
double setPoint = -2.75;//-3.19
double outPID;
float motorOutput;
double speed = 0;

float driveBias =0;



/* SETUP */

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.begin(9600);
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");  //IMU binnenhalen
    while (1)
      ;
  }

  IMU.readAcceleration(aX, aY, aZ);
  IMU.readGyroscope(gX, gY, gZ);  //Dit wordt uitgevoerd omdat de eerste waarden onbruikbaar zijn (waarden die niet kloppen)
  delay(1500);
  BLE.begin();
  Serial.println("Bluetooth® Low Energy Central - LED control");

  // start scanning for peripherals
  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
}

/* LOOP */

void calculateAngle() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    currentTime = millis();
    deltaTime = currentTime - previousTime; //tijd nodig voor loop uit te voeren
    previousTime = currentTime;

    IMU.readAcceleration(aX, aY, aZ);  //accelerometer uitlezen
    IMU.readGyroscope(gX, gY, gZ);     //gyroscoop uitlezen

    aAngle = atan2(aY, aZ) * RAD_TO_DEG;                                               // bgtan tussen aX en aZ en dan omgezet naar graden
    gAngle = (float)gX * deltaTime / 1000;                                             //hoeksnelheid(dgrs/s) * tijd(s) = hoek (dgrs)
    currentAngle = (double)(alpha * (previousAngle - gAngle) + (1 - alpha) * aAngle);  //complementary filter met alpha als gewicht
    previousAngle = currentAngle;
    calculatePID(currentAngle);
  }
}

void calculatePID(double currentAngle){
  errorSize = tan((setPoint+driveBias - currentAngle)*DEG_TO_RAD)*RAD_TO_DEG;  //proportionele term omgezet met tangens 
  cumError += errorSize* deltaTime;
  cumError = constrain(cumError,-4000,4000);
  rateError = (errorSize - lastError)/(deltaTime);  //afgeleide
  outPID = Kp*errorSize + Ki*cumError + Kd*rateError; //totale PID berekeneing
  lastError = errorSize;
  motorSturing(outPID); //motorsturing aanspreken met nodige input
}

void motorSturing(int motorSpeed) {  // Programma om de richting en de snelheid van de motoren te sturen
  motorSpeed = constrain(motorSpeed , -255,255);
  if (motorSpeed < 0) {
    digitalWrite(in1, LOW);  // Omkeren van de polariteit van de linker motor om de robot naar achter te sturen
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);  // Omkeren van de polariteit van de rechter motor om de robot naar achter te sturen
    digitalWrite(in4, HIGH);

    motorOutput = ((maxPWM-minPWM)/maxPWM)*motorSpeed - minPWM;
    analogWrite(enA, abs(motorOutput));
    analogWrite(enB, abs(motorOutput));

  } else {

    digitalWrite(in1, HIGH);  // Omkeren van de polariteit van de linker motor om de robot naar voor te sturen
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);  // Omkeren van de polariteit van de rechter motor om de robot naar voor te sturen
    digitalWrite(in4, LOW);

    motorOutput = ((maxPWM-minPWM)/maxPWM)*motorSpeed + minPWM;
    analogWrite(enA, abs(motorOutput));
    analogWrite(enB, abs(motorOutput)); 
  }
}



void loop() {
  BLEDevice peripheral = BLE.available();

    if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "Robot_controller") {
      return;
    }

    BLE.stopScan();

    BLEcontrol(peripheral);
    BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
  }
  
  
}

void BLEcontrol(BLEDevice peripheral) {
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

  // retrieve the LED characteristic
  BLECharacteristic driveCharac = peripheral.characteristic("19B10000-E8F2-537E-4F6C-D104768A1215");

  if (!driveCharac) {
    Serial.println("Peripheral does not have LED characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    calculateAngle();
    driveCharac.readValue( &driveBias, 4 );
  }

  Serial.println("Peripheral disconnected");
}

