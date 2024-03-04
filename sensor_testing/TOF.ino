#include <Wire.h>
#include <VL53L0X.h>

#define XSHUT_PIN_1 PA6
#define XSHUT_PIN_2 PA7
#define XSHUT_PIN_3 PB5
#define XSHUT_PIN_4 PB11
#define XSHUT_PIN_5 PB10

VL53L0X sensorLeft;
VL53L0X sensorMidLeft;
VL53L0X sensorCenter;
VL53L0X sensorMidRight;
VL53L0X sensorRight;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);


  // Set the XSHUT pins as outputs
  pinMode(XSHUT_PIN_1, OUTPUT);
  pinMode(XSHUT_PIN_2, OUTPUT);
  pinMode(XSHUT_PIN_3, OUTPUT);
  pinMode(XSHUT_PIN_4, OUTPUT);
  pinMode(XSHUT_PIN_5, OUTPUT);


  // Keep all sensors in reset mode (XSHUT LOW)
  digitalWrite(XSHUT_PIN_1, LOW);
  digitalWrite(XSHUT_PIN_2, LOW);
  digitalWrite(XSHUT_PIN_3, LOW);
  digitalWrite(XSHUT_PIN_4, LOW);
  digitalWrite(XSHUT_PIN_5, LOW);
 // digitalWrite(8, LOW);


    // Power up sensor 1
  digitalWrite(XSHUT_PIN_1, HIGH); // Set XSHUT pin for sensor 1 to HIGH (VCC)
  delay(100); // Wait for sensor 1 to start
  sensorLeft.init();
  sensorLeft.setAddress(0x30); // Set new I2C address (0x30)

  // // Power up sensor 2
  digitalWrite(XSHUT_PIN_2, HIGH); // Set XSHUT pin for sensor 1 to HIGH (VCC)
  delay(100); // Wait for sensor 1 to start
  sensorCenter.init();
  sensorCenter.setAddress(0x31); // Set new I2C address (0x30)

  // // Power up sensor 3
  digitalWrite(XSHUT_PIN_3, HIGH); // Set XSHUT pin for sensor 1 to HIGH (VCC)
  delay(100); // Wait for sensor 1 to start
  sensorRight.init();
  sensorRight.setAddress(0x32); // Set new I2C address (0x30)

    // // Power up sensor 3
  digitalWrite(XSHUT_PIN_4, HIGH); // Set XSHUT pin for sensor 1 to HIGH (VCC)
  delay(100); // Wait for sensor 1 to start
  sensorMidLeft.init();
  sensorMidLeft.setAddress(0x33); // Set new I2C address (0x30)

    // // Power up sensor 3
  digitalWrite(XSHUT_PIN_5, HIGH); // Set XSHUT pin for sensor 1 to HIGH (VCC)
  delay(100); // Wait for sensor 1 to start
  sensorMidRight.init();
  sensorMidRight.setAddress(0x34); // Set new I2C address (0x30)

  sensorLeft.startContinuous();
  sensorCenter.startContinuous();
  sensorRight.startContinuous();
  sensorMidRight.startContinuous();
  sensorMidLeft.startContinuous();

}
int distL;
int distC;
int distR;
int distMR;
int distML;
void loop() {
  //motor(1,1,200);
  // for sensor 1
  distL = sensorLeft.readRangeContinuousMillimeters();
  Serial.print("DistL : ");
  Serial.print(distL);
  Serial.print('\t');

 // //for sensor 3
  distML = sensorMidLeft.readRangeContinuousMillimeters();
  Serial.print("DistML: ");
  Serial.print(distML);
  Serial.print('\t');


  // //for sensor 2
  distC = sensorCenter.readRangeContinuousMillimeters();
  Serial.print("DistC : ");
  Serial.print(distC);
  Serial.print('\t');

   // //for sensor 3
  distMR = sensorMidRight.readRangeContinuousMillimeters();
  Serial.print("DistMR: ");
  Serial.print(distMR);
  Serial.print('\t');

  // //for sensor 3
  distR = sensorRight.readRangeContinuousMillimeters();
  Serial.print("DistR: ");
  Serial.print(distR);
  Serial.println('\t');

}
