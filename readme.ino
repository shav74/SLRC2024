#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4
#define IR6 A5
#define IR7 5
#define IR8 6

#define RMotorA 12
#define RMotorB 13
#define RMotorPWM 11

#define LMotorA 9
#define LMotorB 8
#define LMotorPWM 10

#define MAX_SPEED 80

#define WHITE 0
#define BLACK 1

int MotorBasespeed = 60;  //56

int IR_val[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int IR_weights[8] = { -40, -20, -10, -5, 5, 10, 20, 40 };

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAjust = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 5;    //4.5
float Kd = 4.8;  //1.8
float Ki = -0;

void PID_control();
void read_IR();
void set_speed();
void set_forward();
void turnright();
void turnleft();
void isRight();
void isLeft();
void isCross();


void setup() {
  Serial.begin(9600);
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

  set_forward();
  delay(2000);

  // ------------------------------------------- main code --------------------------------------------------
  
}

void loop() {}

void go() {
  read_IR();
  PID_control();
  set_speed();
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
  LMotorSpeed = MotorBasespeed - speedAdjust;
  RMotorSpeed = MotorBasespeed + speedAdjust;

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
  delay(160);
}
void turnleft() {
  digitalWrite(RMotorA, HIGH);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorA, LOW);
  digitalWrite(LMotorB, LOW);
  analogWrite(RMotorPWM, MAX_SPEED);
  delay(160);
}
void set_speed() {
  digitalWrite(RMotorA, HIGH);
  digitalWrite(RMotorB, LOW);
  digitalWrite(LMotorA, HIGH);
  digitalWrite(LMotorB, LOW);
  analogWrite(LMotorPWM, LMotorSpeed);
  analogWrite(RMotorPWM, RMotorSpeed);
}
void set_forward() {
  digitalWrite(LMotorA, HIGH);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, HIGH);
  digitalWrite(RMotorB, LOW);
  analogWrite(RMotorPWM, MAX_SPEED);
  analogWrite(LMotorPWM, MAX_SPEED);
}
void stop() {
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorB, LOW);
}

void isRight() {
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == BLACK && IR_val[4] == BLACK && IR_val[5] == BLACK && IR_val[6] == BLACK && IR_val[7] == BLACK) {
    return true;
  }
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == BLACK && IR_val[5] == BLACK && IR_val[6] == BLACK && IR_val[7] == BLACK) {
    return true;
  }
  if (IR_val[0] == WHITE && IR_val[1] == WHITE && IR_val[2] == WHITE && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == BLACK && IR_val[6] == BLACK && IR_val[7] == BLACK) {
    return true;
  }
}


void isLeft() {
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == BLACK && IR_val[3] == WHITE && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == BLACK && IR_val[3] == BLACK && IR_val[4] == WHITE && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
  if (IR_val[0] == BLACK && IR_val[1] == BLACK && IR_val[2] == BLACK && IR_val[3] == BLACK && IR_val[4] == BLACK && IR_val[5] == WHITE && IR_val[6] == WHITE && IR_val[7] == WHITE) {
    return true;
  }
}

void isCross() {
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
}
