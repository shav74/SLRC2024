// --------------------------- libaries -------------------

#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_TCS34725.h>
#include <Adafruit_GFX.h>
#include <LiquidCrystal_I2C.h>
#include <SharpIR.h>

// ------------------- State variables -------------------

#define model 1080

#define MAGNET 53

#define DELAY 1000

#define IR1 30
#define IR2 32
#define IR3 34
#define IR4 36
#define IR5 38
#define IR6 40
#define IR7 42
#define IR8 44

#define SHARP_R A1
#define SHARP_LB A3
#define SHARP_LM A2
#define SHARP_LT A0

#define TOF_FR 0
#define TOF_FL 1
#define TOF_FM 2

#define COLOR_F 5
#define COLOR_R 3
#define COLOR_L 4

#define XSHUT_PIN_FR 47
#define XSHUT_PIN_FL 45
#define XSHUT_PIN_FM 41
// #define XSHUT_PIN_4 14
// #define XSHUT_PIN_5 22

#define LMotorA 24
#define LMotorB 22
#define LMotorPWM 7

#define RMotorA 26
#define RMotorB 28
#define RMotorPWM 6

#define CUBOID 15
#define CYLINDER 14
#define SH_B 4
#define SH_G 5

#define MAX_SPEED 230  //200

#define WHITE 0
#define BLACK 1

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

// ------------------- Objects -----------------

Adafruit_VL53L0X tofFR = Adafruit_VL53L0X();
Adafruit_VL53L0X tofFL = Adafruit_VL53L0X();
Adafruit_VL53L0X tofFM = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

SharpIR SharpIRR(SHARP_R, model);
SharpIR SharpIRLT(SHARP_LT, model);
SharpIR SharpIRLM(SHARP_LM, model);
SharpIR SharpIRLB(SHARP_LB, model);

Adafruit_TCS34725 colorFront = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 colorLeft = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 colorRight = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

LiquidCrystal_I2C lcd(0x3F, 16, 2);


// -------------------- state variables -----------------

int MotorBasespeed = 210;  //160

int IR_val[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int IR_weights[8] = { -40, -20, -10, -5, 5, 10, 20, 40 };

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAjust = 0;

bool box1 = true;
bool box2 = true;
bool box3 = true;

bool box1Check = true;
bool box2Check = true;
bool box3Check = true;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 6.5;  //6.5
float Kd = 1.2;    //1
float Ki = 0;

int min_in_circle = 0;
int max_in_circle = 0;

int distTOFOne;
int distTOFTwo;
int distTOFThree;
int distTOFFour;
int distTOFFive;
int distTOFSix;

void PID_control();
void read_IR();
void set_speed();
void set_forward();
void TCA9548A();
void turnright();
void turnleft();
bool isRight();
bool isLeft();
bool isCross();
char getColor();
int getColorJunction();
int getSharpIRDistance();
bool isBoxAttached();
void go_millis();
void putBox();
int setMaxMin();
int getTOFDist();
void slow_stop();
void go();

void go_millis(int interval) {
  unsigned long start_millis = millis();
  while (millis() - start_millis < interval) {
    read_IR();
    go();
  }
}

void putBox() {
  while (!isColorJunction()) {
    go();
  }

  set_forward();
  delay(800);

  while (!isCross()) {
    go();
  }

  turnleft();
  delay(1700);

  go_millis(200);

  while (!isCross()) {
    set_speed_reverse(150);
  }

  set_speed_reverse(150);
  delay(1000);

  slow_stop();

  while (!isCross()) {
    go();
  }


  while ((getTOFDist(TOF_FL, tofFL) > 45 && getTOFDist(TOF_FL, tofFL) > 0) && (getTOFDist(TOF_FR, tofFR) > 45 && getTOFDist(TOF_FR, tofFR) > 0)) {
    read_IR();
    go();
  }
  slow_stop();

  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print(getTOFDist(TOF_FL, tofFL));
  lcd.setCursor(2, 1);
  lcd.print(getTOFDist(TOF_FR, tofFR));

  digitalWrite(MAGNET, LOW);

  set_speed_reverse(150);
  delay(500);

  stop();
}

bool isBoxAttached() {
  if (getTOFDist(TOF_FM, tofFM) <= 55 && getTOFDist(TOF_FM, tofFM) > 0) {
    return true;
  } else {
    return false;
  }
}

bool isBoxPresent() {
  int distFM = getTOFDist(TOF_FM, tofFM);
  if (distFM < 60 && distFM > 0) {
    lcd.clear();
    lcd.print(distFM);
    return true;
  } else {
    return false;
  }
}

int getSharpIRDistance(SharpIR sensor) {
  return sensor.distance();
}

int setMaxMin(int num) {
  if (num > max_in_circle && num < 30) {
    max_in_circle = num;
  }
  if (num < min_in_circle && num < 30) {
    min_in_circle = num;
  }
}

int getColorJunction() {
  char colorLeft = getColor(COLOR_L, colorLeft);
  char colorRight = getColor(COLOR_R, colorRight);

  if (colorRight == 'b' && colorLeft == 'g') {
    return 1;
  } else if (colorRight == 'g' && colorLeft == 'b') {
    return 1;
  } else if (colorLeft == 'r' || colorRight == 'r') {
    return -1;
  } else {
    return 0;
  }
}

void go() {
  PID_control();
  set_speed();
}

char getColor(int mux_port, Adafruit_TCS34725 color) {
  TCA9548A(mux_port);
  int r, g, b, c;
  color.getRawData(&r, &g, &b, &c);
  color.calculateColorTemperature_dn40(r, g, b, c);

  if (r > g && r > b) {
    return 'r';
  } else if (b > g && b > r) {
    return 'b';
  } else if (g > b && g > r) {
    return 'g';
  } else {
    return 'e';
  }
}

int getTOFDist(int mux_port, Adafruit_VL53L0X tof) {

  TCA9548A(mux_port);
  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  } else {
    return -1;
  }
}

void PID_control() {
  error = 0;
  for (int i = 0; i < 8; i++) {
    error += (IR_weights[i]) * IR_val[i];
  }
  P = error;
  I = I + error;
  D = error - previousError;

  previousError = error;

  float speedAdjust = (Kp * P + Ki * I + Kd * D);
  LMotorSpeed = MotorBasespeed + speedAdjust;
  RMotorSpeed = MotorBasespeed - speedAdjust;

  if (LMotorSpeed < 0) {
    LMotorSpeed = 0;
  }
  if (RMotorSpeed < 0) {
    RMotorSpeed = 0;
  }
  if (LMotorSpeed > MAX_SPEED) {
    LMotorSpeed = MAX_SPEED;
  }
  if (RMotorSpeed > MAX_SPEED) {
    RMotorSpeed = MAX_SPEED;
  }
}

void read_IR() {
  IR_val[0] = digitalRead(IR1);
  IR_val[1] = digitalRead(IR2);
  IR_val[2] = digitalRead(IR3);
  IR_val[3] = digitalRead(IR4);
  IR_val[4] = digitalRead(IR5);
  IR_val[5] = digitalRead(IR6);
  IR_val[6] = digitalRead(IR7);
  IR_val[7] = digitalRead(IR8);
}
void turnright() {
  digitalWrite(RMotorA, LOW);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorA, HIGH);
  digitalWrite(LMotorB, LOW);
  analogWrite(LMotorPWM, MAX_SPEED);
  // delay(160);
}

void turnleft() {
  digitalWrite(RMotorA, HIGH);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorA, LOW);
  digitalWrite(LMotorB, LOW);
  analogWrite(RMotorPWM, MAX_SPEED);
  // delay(160);
}

void set_speed() {
  digitalWrite(RMotorA, HIGH);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorA, HIGH);
  digitalWrite(LMotorB, LOW);
  analogWrite(LMotorPWM, LMotorSpeed);
  analogWrite(RMotorPWM, RMotorSpeed);
}

void set_speed_reverse(int speed) {
  digitalWrite(LMotorA, LOW);
  digitalWrite(LMotorB, HIGH);
  digitalWrite(RMotorA, LOW);
  digitalWrite(RMotorB, HIGH);
  analogWrite(RMotorPWM, speed);
  analogWrite(LMotorPWM, speed + 30);
}

void set_foward_custom(int speed) {
  digitalWrite(LMotorA, HIGH);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, HIGH);
  digitalWrite(RMotorB, LOW);
  analogWrite(RMotorPWM, speed + 20);
  analogWrite(LMotorPWM, speed);
}

void set_forward() {
  digitalWrite(LMotorA, HIGH);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, HIGH);
  digitalWrite(RMotorB, LOW);
  analogWrite(RMotorPWM, MAX_SPEED);
  analogWrite(LMotorPWM, MAX_SPEED);
}

void left_circle() {
  set_forward();
  delay(100);
  while (!isRight()) {
    go();
  }
  turnright();
  delay(1300);
  lcd.clear();
  lcd.print("got it");

  while (!isCross()) {
    go();
  }
  turnleft();
  delay(1000);
  lcd.clear();
  lcd.print("in circle");
  stop();
  delay(0);

  int sample_count = 0;

  unsigned long currentMillis = 0;
  unsigned long startMillis = millis();

  lcd.clear();
  lcd.print("started millis");

  while ((millis() - startMillis) <= 2000) {
    read_IR();
    go();
  }
  stop();
  lcd.clear();

  min_in_circle = getSharpIRDistance(SharpIRR);

  while (!isLeftCircle()) {
    currentMillis = millis();
    while (millis() - currentMillis <= 250) {
      if (isLeftCircle()) {
        break;
      }
      go();
    }
    sample_count++;
    int distance_to_object = getSharpIRDistance(SharpIRR);

    lcd.clear();
    lcd.setCursor(2, 0);
    //lcd.print(sample_count);

    lcd.setCursor(2, 1);
    lcd.print(distance_to_object);

    setMaxMin(distance_to_object);
  }

  if (max_in_circle - min_in_circle > 4) {
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print(max_in_circle);
    lcd.setCursor(7, 0);
    lcd.print(min_in_circle);
    lcd.setCursor(2, 1);
    lcd.print("square");
    digitalWrite(CUBOID, HIGH);
  } else {
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print(max_in_circle);
    lcd.setCursor(7, 0);
    lcd.print(min_in_circle);
    lcd.setCursor(2, 1);
    lcd.print("circle");
    digitalWrite(CYLINDER, HIGH);
  }
}

void right_circle() {
  set_forward();
  delay(100);

  while (!isLeft()) {
    go();
  }
  turnleft();
  delay(1000);
  lcd.clear();
  lcd.print("got it");

  while (!isCross()) {
    go();
  }
  turnleft();
  delay(1000);
  lcd.clear();
  lcd.print("in circle");
  stop();
  delay(0);

  int sample_count = 0;

  unsigned long currentMillis = 0;
  unsigned long startMillis = millis();

  lcd.clear();
  lcd.print("started millis");

  while ((millis() - startMillis) <= 2000) {
    read_IR();
    go();
  }
  stop();
  lcd.clear();

  min_in_circle = getSharpIRDistance(SharpIRR);

  while (!isLeftCircle()) {
    currentMillis = millis();
    while (millis() - currentMillis <= 250) {
      if (isLeftCircle()) {
        break;
      }
      go();
    }
    sample_count++;
    int distance_to_object = getSharpIRDistance(SharpIRR);

    lcd.clear();
    lcd.setCursor(2, 0);
    //lcd.print(sample_count);

    lcd.setCursor(2, 1);
    lcd.print(distance_to_object);

    setMaxMin(distance_to_object);
  }

  if (max_in_circle - min_in_circle > 4) {
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print(max_in_circle);
    lcd.setCursor(7, 0);
    lcd.print(min_in_circle);
    lcd.setCursor(2, 1);
    lcd.print("square");
    digitalWrite(CUBOID, HIGH);
  } else {
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print(max_in_circle);
    lcd.setCursor(7, 0);
    lcd.print(min_in_circle);
    lcd.setCursor(2, 1);
    lcd.print("circle");
    digitalWrite(CYLINDER, HIGH);
  }
}

void rotate() {
  digitalWrite(LMotorA, HIGH);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, LOW);
  digitalWrite(RMotorB, HIGH);
  analogWrite(RMotorPWM, MotorBasespeed - 30);
  analogWrite(LMotorPWM, MotorBasespeed - 30);
}

void stop() {
  digitalWrite(LMotorA, LOW);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, LOW);
  digitalWrite(RMotorB, LOW);
}
void slow_stop() {
  digitalWrite(LMotorA, HIGH);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, HIGH);
  digitalWrite(RMotorB, LOW);
  analogWrite(LMotorPWM, 0);
  analogWrite(RMotorPWM, 0);
}

bool isRight() {
  read_IR();
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == BLACK && IR_val[4] == BLACK && IR_val[5] == BLACK && IR_val[6] == BLACK && IR_val[7] == BLACK) {
    return true;
  }
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == BLACK && IR_val[5] == BLACK && IR_val[6] == BLACK && IR_val[7] == BLACK) {
    return true;
  }
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == BLACK && IR_val[6] == BLACK && IR_val[7] == BLACK) {
    return true;
  }
  return false;
}

bool isCross() {
  read_IR();
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == BLACK) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == BLACK) {
    return true;
  }
  return false;
}

bool isLeft() {
  read_IR();
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == BLACK && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == BLACK && IR_val[3] == BLACK && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == BLACK && IR_val[3] == BLACK && IR_val[4] == BLACK && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  return false;
}

bool isLeftCircle() {
  read_IR();
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == BLACK && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == BLACK && IR_val[3] == BLACK && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == BLACK && IR_val[3] == BLACK && IR_val[4] == BLACK && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == BLACK) {
    return true;
  }
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == BLACK) {
    return true;
  }

  return false;
}

bool isColorJunction() {
  read_IR();
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == BLACK && IR_val[7] == BLACK) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == BLACK) {
    return true;
  }
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == BLACK) {
    return true;
  }
  return false;
}

bool isInMiddle() {
  read_IR();
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == BLACK && IR_val[7] == BLACK) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == BLACK && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == BLACK && IR_val[6] == BLACK && IR_val[7] == BLACK) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == BLACK && IR_val[6] == BLACK && IR_val[7] == BLACK) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == BLACK && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == BLACK && IR_val[7] == BLACK) {
    return true;
  }
  return false;
}

void uTurn() {
  rotate();
  delay(1000);
  while (!isInMiddle()) {
    rotate();
  }
  set_speed_reverse(130);
  delay(500);
}

void grabBox() {
  set_speed_reverse(150);
  delay(500);
  while (!isCross()) {
    set_speed_reverse(150);
  }
  set_speed_reverse(150);
  delay(500);
  slow_stop();

  // digitalWrite(MAGNET, HIGH);

  while (!isCross()) {
    go();
  }
  set_forward();
  delay(200);

  while (!isColorJunction()) {
    go();
  }

  set_speed_reverse(150);
  delay(500);

  while (!isCross()) {
    set_speed_reverse(150);
  }
  set_speed_reverse(150);
  delay(600);
  slow_stop();
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  while (!Serial) {
    delay(1);
  }

  lcd.init();
  lcd.clear();
  lcd.backlight();

  lcd.setCursor(2, 0);
  lcd.print("Panthera");

  lcd.setCursor(2, 1);
  lcd.print("Team #1");

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);

  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);
  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  pinMode(SH_G, OUTPUT);
  pinMode(SH_B, OUTPUT);
  pinMode(CUBOID, OUTPUT);
  pinMode(CYLINDER, OUTPUT);

  // Set the XSHUT pins as outputs
  pinMode(XSHUT_PIN_FR, OUTPUT);
  pinMode(XSHUT_PIN_FL, OUTPUT);
  pinMode(XSHUT_PIN_FM, OUTPUT);

  pinMode(MAGNET, OUTPUT);

  // Keep all sensors in reset mode (XSHUT LOW)
  digitalWrite(XSHUT_PIN_FR, LOW);
  digitalWrite(XSHUT_PIN_FL, LOW);
  digitalWrite(XSHUT_PIN_FM, LOW);

  digitalWrite(SH_G, LOW);
  digitalWrite(SH_B, LOW);
  digitalWrite(CUBOID, LOW);
  digitalWrite(CYLINDER, LOW);


  delay(10);
  digitalWrite(XSHUT_PIN_FR, HIGH);
  digitalWrite(XSHUT_PIN_FL, HIGH);
  digitalWrite(XSHUT_PIN_FM, HIGH);

  //color sensors

  TCA9548A(5);
  if (colorFront.begin()) {
  } else {
    Serial.print("color F");
    while (1)
      ;
  }

  TCA9548A(3);
  if (colorRight.begin()) {
  } else {
    Serial.print("color R");
    while (1)
      ;
  }

  TCA9548A(4);
  if (colorLeft.begin()) {
  } else {
    Serial.print("color L");
    while (1)
      ;
  }

  // initialize TOF sensors
  TCA9548A(0);
  delay(100);
  if (!tofFR.begin(0x29)) {
    Serial.println("Failed to detect and initialize TOF sensor FR!");
    while (1) {}
  }

  TCA9548A(1);
  delay(100);
  if (!tofFL.begin(0x29)) {
    Serial.println("Failed to detect and initialize TOF sensor FL!");
    while (1) {}
  }

  TCA9548A(2);
  delay(100);
  if (!tofFM.begin(0x29)) {
    Serial.println("Failed to detect and initialize TOF sensor FM!");
    while (1) {}
  }

  set_forward();
  delay(20);


  // ------------------------------------------- main code --------------------------------------------------

  // first right junction
  while (!isLeft()) {
    go();
  }
  lcd.print("left");

  // skip the junction
  set_forward();
  delay(200);
  lcd.clear();
  lcd.print("skipped");

  while (getTOFDist(TOF_FM, tofFM) >= 70 && getTOFDist(TOF_FM, tofFM) > 0) {
    read_IR();
    go();
  }
  stop();
  while (getTOFDist(TOF_FM, tofFM) >= 70 && getTOFDist(TOF_FM, tofFM) > 0) {
    read_IR();
    go();
  }
  stop();

  lcd.clear();
  lcd.print(getTOFDist(TOF_FM, tofFM));
  delay(1000);

  //get the color of the stone holder
  char colorStone = getColor(COLOR_F, colorFront);
  lcd.clear();
  lcd.print(colorStone);
  if (colorStone == 'g') {
    digitalWrite(SH_G, HIGH);
  } else if (colorStone == 'b') {
    digitalWrite(SH_B, HIGH);
  }

  while (!isLeftCircle()) {
    set_speed_reverse(120);
  }

  stop();
  delay(100);

  turnleft();
  delay(1000);

  while (!isRight()) {
    go();
  }
  lcd.clear();
  lcd.print("right");

  turnright();
  delay(1300);

  while (!isCross()) {
    go();
  }
  lcd.clear();
  lcd.print("junction");

  turnleft();
  delay(1000);

  while (!isColorJunction()) {
    go();
  }
  slow_stop();

  lcd.clear();
  lcd.print("color junction");
  delay(1000);

  while (getColorJunction() == 0) {
    set_foward_custom(130);
    delay(230);
    stop();
  }
  stop();
  if (getColorJunction() == -1) {
    while (getColorJunction() == 0 || getColorJunction() == -1) {
      set_speed_reverse(130);
      delay(280);
      stop();
    }
  }
  stop();
  lcd.clear();
  char color_left_circle = getColor(COLOR_L, colorLeft);
  lcd.print(color_left_circle);
  delay(1000);

  lcd.clear();
  char color_right_circle = getColor(COLOR_R, colorRight);
  lcd.print(color_right_circle);
  delay(1000);

  // turn according to the color
  char after_colour_j;
  if (colorStone == color_right_circle) {
    turnright();
    delay(DELAY + 300);
    after_colour_j = 'r';
  } else {
    turnleft();
    delay(DELAY);
    after_colour_j = 'l';
  }

  if (after_colour_j == 'r') {
    right_circle();
  } else if (after_colour_j = 'l') {
    left_circle();
  }

  // after the circle
  turnleft();
  delay(1000);

  // ----------------------- boxes -------------------------------

  if (after_colour_j == 'l') {

    MotorBasespeed = 180;

    while (!isLeft()) {
      go();
    }
    turnleft();
    delay(DELAY + 400);

    while (!isColorJunction()) {
      go();
    }
    set_forward();
    delay(200);

    turnleft();
    delay(DELAY);

    MotorBasespeed = 200;

  } else if (after_colour_j == 'r') {

    MotorBasespeed = 180;

    while (!isRight()) {
      go();
    }
    turnright();
    delay(DELAY + 400);

    while (!isColorJunction()) {
      go();
    }
    set_forward();
    delay(200);

    turnright();
    delay(DELAY + 300);

    MotorBasespeed = 200;

  }

  bool box1 = true;
  bool box2 = true;
  bool box3 = true;

  digitalWrite(MAGNET, HIGH);

  while (!isCross()) {
    go();
  }

  set_forward();
  delay(100);

  turnright();
  delay(DELAY + 300);

  lcd.clear();
  lcd.print("box 1");

  while (!isBoxPresent()) {
    if (isColorJunction()) {
      stop();
      box1 = false;
      break;
    }
    read_IR();
    go();
  }
  stop();


  if (box1 == true) {
    lcd.clear();
    lcd.print("box present");

    grabBox();

    //check if box 1 is attached
    if (isBoxAttached()) {
      lcd.clear();
      lcd.print("grabbed 1");
      while (!isCross()) {
        go();
      }

      set_forward();
      delay(100);

      turnright();
      delay(DELAY + 300);

      putBox();

    } else {
      //digitalWrite(MAGNET, LOW);
      lcd.clear();
      lcd.print("cardboard 1");

      //check box 2
      while (!isCross()) {
        go();
      }

      set_forward();
      delay(100);

      turnleft();
      delay(DELAY);

      while (!isBoxPresent()) {
        if (isColorJunction()) {
          box2 = false;
          break;
        }
        read_IR();
        go();
      }
      stop();

      if (box2 == true) {

        grabBox();

        //check if box 2 is attached
        if (isBoxAttached()) {
          lcd.clear();
          lcd.print("grabbed 2");
          // delay(2000);

          while (!isCross()) {
            go();
          }

          set_forward();
          delay(100);

          turnleft();
          delay(DELAY + 600);

          go_millis(300);

          while (!isCross()) {
            set_speed_reverse(150);
          }
          set_speed_reverse(150);
          delay(500);

          while (!isCross()) {
            go();
          }

          set_forward();
          delay(100);

          turnleft();
          delay(DELAY + 500);

          putBox();
        } else {
          //digitalWrite(MAGNET, LOW);
          lcd.clear();
          lcd.print("cardboard 2");
          // delay(1000);

          while (!isCross()) {
            go();
          }

          // digitalWrite(MAGNET, HIGH);

          set_forward();
          delay(100);

          turnleft();
          delay(DELAY);

          go_millis(300);

          while (!isCross()) {
            set_speed_reverse(150);
          }

          set_speed_reverse(150);
          delay(400);

          while (!isCross()) {
            go();
          }

          set_forward();
          delay(300);

          while (!isColorJunction()) {
            go();
          }

          set_speed_reverse(150);
          delay(500);
          while (!isCross()) {
            set_speed_reverse(150);
          }
          set_speed_reverse(150);
          delay(500);

          while (!isCross()) {
            go();
          }

          set_forward();
          delay(100);

          turnleft();
          delay(DELAY + 300);

          putBox();
        }
      } else {

        set_speed_reverse(150);
        delay(400);

        while (!isCross()) {
          set_speed_reverse(150);
        }

        set_speed_reverse(150);
        delay(400);

        while (!isCross()) {
          go();
        }

        set_forward();
        delay(100);

        turnleft();
        delay(DELAY);

        go_millis(300);

        while (!isCross()) {
          set_speed_reverse(150);
        }

        set_speed_reverse(150);
        delay(300);

        while (!isCross()) {
          go();
        }

        set_forward();
        delay(400);

        while (!isColorJunction()) {
          go();
        }

        set_speed_reverse(150);
        delay(500);
        while (!isCross()) {
          set_speed_reverse(150);
        }
        set_speed_reverse(150);
        delay(500);

        while (!isCross()) {
          go();
        }

        set_forward();
        delay(100);

        turnleft();
        delay(DELAY);

        putBox();
      }
    }
  } else {
    set_speed_reverse(150);
    delay(400);

    while (!isCross()) {
      set_speed_reverse(150);
    }

    set_speed_reverse(150);
    delay(400);

    while (!isCross()) {
      go();
    }

    set_forward();
    delay(100);

    turnleft();
    delay(DELAY + 300);

    while (!isBoxPresent()) {
      if (isColorJunction()) {
        box2 = false;
        break;
      }
      read_IR();
      go();
    }
    stop();

    if (box2 == true) {

      grabBox();

      //check if box 2 is attached
      if (isBoxAttached()) {
        lcd.clear();
        lcd.print("grabbed 2");
        // delay(2000);

        while (!isCross()) {
          go();
        }
        set_forward();
        delay(100);

        turnleft();
        delay(DELAY + 600);

        go_millis(300);

        while (!isCross()) {
          set_speed_reverse(150);
        }
        set_speed_reverse(150);
        delay(500);

        while (!isCross()) {
          go();
        }

        set_forward();
        delay(100);

        turnleft();
        delay(DELAY + 500);

        putBox();
      } else {
        //digitalWrite(MAGNET, LOW);
        lcd.clear();
        lcd.print("cardboard 2");
        // delay(1000);

        while (!isCross()) {
          go();
        }

        // digitalWrite(MAGNET, HIGH);

        set_forward();
        delay(100);

        turnleft();
        delay(DELAY);

        go_millis(300);

        while (!isCross()) {
          set_speed_reverse(150);
        }

        set_speed_reverse(150);
        delay(400);

        while (!isCross()) {
          go();
        }

        set_forward();
        delay(300);

        while (!isColorJunction()) {
          go();
        }

        set_speed_reverse(150);
        delay(500);
        while (!isCross()) {
          set_speed_reverse(150);
        }
        set_speed_reverse(150);
        delay(500);

        while (!isCross()) {
          go();
        }

        set_forward();
        delay(100);

        turnleft();
        delay(DELAY);

        putBox();
      }
    } else {

      set_speed_reverse(150);
      delay(400);

      while (!isCross()) {
        set_speed_reverse(150);
      }

      set_speed_reverse(150);
      delay(400);

      while (!isCross()) {
        go();
      }

      set_forward();
      delay(100);

      turnleft();
      delay(DELAY);

      go_millis(300);

      while (!isCross()) {
        set_speed_reverse(150);
      }

      set_speed_reverse(150);
      delay(400);

      while (!isCross()) {
        go();
      }

      set_forward();
      delay(300);

      while (!isColorJunction()) {
        go();
      }

      set_speed_reverse(150);
      delay(500);
      while (!isCross()) {
        set_speed_reverse(150);
      }
      set_speed_reverse(150);
      delay(500);

      while (!isCross()) {
        go();
      }

      set_forward();
      delay(100);

      turnleft();
      delay(DELAY);

      putBox();
    }
  }
  while (!isCross()) {
    set_speed_reverse(150);
  }

  set_speed_reverse(150);
  delay(400);

  while (!isCross()) {
    go();
  }

  turnright();
  delay(DELAY);

  while (!isColorJunction()) {
    go();
  }

  set_forward();
  delay(100);

  slow_stop();
}

void loop() {
}
