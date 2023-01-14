#include <QTRSensors.h>
#include <EEPROM.h>

// pins for the motors.
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

// pins for the LEDs (used as headlights).
const int ledPin1 = 9;
const int ledPin2 = 8;
const int ledPin3 = 12;
const int ledPin4 = 1;
const int ledPin5 = 2;
const int ledPin6 = 3;

// speed of the motors.
int m1Speed = 0;
int m2Speed = 0;
int motorSpeed;

// parameters used for the PID controller.
float kp = 8;
float ki = 0.0001;
float kd = 2;

// values passed to the PID controller.
int p = 1;
int i = 0;
int d = 0;

// values used for computing the derivative.
int error = 0;
int lastError = 0;

// constants for the motors speed.
const int maxSpeed = 255;
const int minSpeed = -200;
const int baseSpeed = 255;

// constants used for calibration.
const int maxNumberOfMoves = 10;
const int sensorCalibrationBound = 700;

QTRSensors qtr;
const int sensorCount = 6;
int sensorValues[sensorCount];

// function that performs the auto calibration of
// the line follower by moving left and right
// along the line according to the values read
// by the sensor.
void calibrate() {
  digitalWrite(LED_BUILTIN, HIGH);

  int speed = 200, state = 1, numberOfMoves = 0;
  while (numberOfMoves < maxNumberOfMoves) {
    qtr.calibrate();
    qtr.read(sensorValues);

    if (state == 1) {
      if (sensorValues[0] < sensorCalibrationBound) {
        setMotorSpeed(speed, -speed);
      } else {
        state = 2;
      }
    }
    if (state == 2) {
      if (sensorValues[5] < sensorCalibrationBound) {
        setMotorSpeed(-speed, speed);
      } else {
        state = 1;
        numberOfMoves++;
      }
    }
  }
  // EEPROM.put(0, qtr.calibrationOn.minimum);
  // EEPROM.put(30, qtr.calibrationOn.maximum);
  // EEPROM.put(60, qtr.calibrationOn.initialized);
  // Serial.println("end calibration");
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);

  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(ledPin5, OUTPUT);
  pinMode(ledPin6, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, HIGH);
  digitalWrite(ledPin3, HIGH);
  digitalWrite(ledPin4, HIGH);
  digitalWrite(ledPin5, HIGH);
  digitalWrite(ledPin6, HIGH);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);

  Serial.begin(9600);
  calibrate();

  // EEPROM.get(60, qtr.calibrationOn.initialized);
  // EEPROM.get(30, qtr.calibrationOn.maximum);
  // EEPROM.get(0, qtr.calibrationOn.minimum); 
}

void loop() {

  pidControl(kp, ki, kd);

  computeMotorsSpeed();

  setMotorSpeed(m1Speed, m2Speed);
}


// calculates the PID value based on error, kp, kd, ki, p, i and d.
void pidControl(float kp, float ki, float kd) {
  error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);

  p = error;
  i = i + error;
  d = error - lastError;
  lastError = error;

  motorSpeed = kp * p + ki * i + kd * d;
}

// calculates the motor speeds based on the speed provided
// by the PID controller.
void computeMotorsSpeed() {
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  if (error < 0) {
    m1Speed += motorSpeed;
  } else if (error > 0) {
    m2Speed -= motorSpeed;
  }

  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);
}

// each arguments takes values between -255 and 255. The negative
// values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  motor1Speed = -motor1Speed;
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}