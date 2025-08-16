/*
  Project: Obstacle Avoiding Car
  Author: Wired Wanderer (https://youtube.com/shorts/5Ac8ldPXbF8)
  Date: August 2025
  License: MIT License (see LICENSE file)

  Description:
  Arduino-based obstacle avoiding car using:
  - Ultrasonic sensor (HC-SR04) for distance measurement
  - Servo for scanning left/center/right
  - L298N Motor Driver for controlling two DC motors
  - LEDs & Buzzer for alerts
  - Optional: Web dashboard (ESP8266/NodeMCU) for remote alerts

  Features:
  - Detects obstacles ahead and steers around them
  - Servo scanning for smarter avoidance
  - Stuck detection & recovery
  - LEDs and buzzer for feedback
*/

#include <AFMotor.h>
#include <Servo.h>

AF_DCMotor rightBack(1);
AF_DCMotor rightFront(2);
AF_DCMotor leftFront(3);
AF_DCMotor leftBack(4);

Servo servoLook;

byte trig = A0;
byte echo = A5;

byte maxDist = 150;
byte criticalDist = 20;
byte stopDist = 40;

float timeOut = 2.0 * (maxDist + 10.0) / 0.034 * 1000.0;

byte baseMotorSpeed = 80;
int motorOffset = 5;      // Tune this if turns are uneven
int turnSpeed = 60;
int reverseDuration = 300;
int turnDuration = 400;
int turnAroundDuration = 800;

int servoPos = 90;
int servoDir = 1;
unsigned long lastScanTime = 0;
const int scanInterval = 75;

int consecutiveLeftTurns = 0;
int consecutiveRightTurns = 0;
const int MAX_CONSECUTIVE_TURNS = 3;
unsigned long lastTurnAttemptTime = 0;
const unsigned long TURN_COOLDOWN_MS = 2000;

void setup() {
  Serial.begin(9600);

  rightBack.setSpeed(baseMotorSpeed);
  rightFront.setSpeed(baseMotorSpeed);
  leftFront.setSpeed(baseMotorSpeed + motorOffset);
  leftBack.setSpeed(baseMotorSpeed + motorOffset);

  stopMove();

  servoLook.attach(10);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  servoLook.write(servoPos);
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastScanTime > scanInterval) {
    lastScanTime = currentTime;
    servoPos += servoDir * 10;
    if (servoPos >= 180 || servoPos <= 0) {
      servoDir *= -1;
      servoPos += servoDir * 10;
    }
    servoLook.write(servoPos);
  }

  int distance = getDistance();
  Serial.print("Distance: "); Serial.println(distance); // Debug

  if (distance <= criticalDist) {
    stopMove();
    Serial.println("CRITICAL: Reversing!");
    moveBackward();
    delay(reverseDuration);
    stopMove();
    delay(200);
    consecutiveLeftTurns = 0;
    consecutiveRightTurns = 0;
    lastTurnAttemptTime = 0;
    int turnDir = checkDirectionFullScan();
    executeTurn(turnDir);
  }
  else if (distance < stopDist) {
    stopMove();
    Serial.println("Obstacle detected: Scanning...");

    bool stuckDetected = false;
    int recoveryTurnDir = -1;
    if (consecutiveLeftTurns >= MAX_CONSECUTIVE_TURNS) {
      Serial.println("STUCK: Consecutive Left Turns. Recovering...");
      stuckDetected = true;
      recoveryTurnDir = 2; // Force right
    }
    else if (consecutiveRightTurns >= MAX_CONSECUTIVE_TURNS) {
      Serial.println("STUCK: Consecutive Right Turns. Recovering...");
      stuckDetected = true;
      recoveryTurnDir = 0; // Force left
    }

    if (stuckDetected) {
      moveBackward();
      delay(reverseDuration * 2);
      stopMove();
      delay(200);
      executeTurn(recoveryTurnDir);
      consecutiveLeftTurns = 0;
      consecutiveRightTurns = 0;
      lastTurnAttemptTime = 0;
    }
    else {
      int turnDir = checkDirectionFullScan();
      executeTurn(turnDir);

      if (currentTime - lastTurnAttemptTime < TURN_COOLDOWN_MS && lastTurnAttemptTime != 0) {
        if (turnDir == 0) { consecutiveLeftTurns++; consecutiveRightTurns = 0; }
        else if (turnDir == 2) { consecutiveRightTurns++; consecutiveLeftTurns = 0; }
        else { consecutiveLeftTurns = 0; consecutiveRightTurns = 0; }
      }
      else {
        if (turnDir == 0) { consecutiveLeftTurns = 1; consecutiveRightTurns = 0; }
        else if (turnDir == 2) { consecutiveRightTurns = 1; consecutiveLeftTurns = 0; }
        else { consecutiveLeftTurns = 0; consecutiveRightTurns = 0; }
      }
      lastTurnAttemptTime = currentTime;
    }
  }
  else {
    moveForward();
    consecutiveLeftTurns = 0;
    consecutiveRightTurns = 0;
    lastTurnAttemptTime = 0;
  }
}

void moveForward() {
  rightBack.setSpeed(baseMotorSpeed);
  rightFront.setSpeed(baseMotorSpeed);
  leftFront.setSpeed(baseMotorSpeed + motorOffset);
  leftBack.setSpeed(baseMotorSpeed + motorOffset);
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
}

void moveBackward() {
  rightBack.setSpeed(baseMotorSpeed);
  rightFront.setSpeed(baseMotorSpeed);
  leftFront.setSpeed(baseMotorSpeed + motorOffset);
  leftBack.setSpeed(baseMotorSpeed + motorOffset);
  rightBack.run(BACKWARD);
  rightFront.run(BACKWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
}

void stopMove() {
  rightBack.run(RELEASE);
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
  rightBack.setSpeed(0);
  rightFront.setSpeed(0);
  leftFront.setSpeed(0);
  leftBack.setSpeed(0);
}

void executeTurn(int turnDirection) {
  switch (turnDirection) {
    case 0: turnLeft(turnDuration); break;
    case 1: turnLeft(turnAroundDuration); break;
    case 2: turnRight(turnDuration); break;
    default: stopMove(); break;
  }
}

void turnLeft(int duration) {
  rightBack.setSpeed(baseMotorSpeed + turnSpeed);
  rightFront.setSpeed(baseMotorSpeed + turnSpeed);
  leftFront.setSpeed(baseMotorSpeed + motorOffset + turnSpeed);
  leftBack.setSpeed(baseMotorSpeed + motorOffset + turnSpeed);
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  delay(duration);
  stopMove();
  delay(100);
}

void turnRight(int duration) {
  rightBack.setSpeed(baseMotorSpeed + turnSpeed);
  rightFront.setSpeed(baseMotorSpeed + turnSpeed);
  leftFront.setSpeed(baseMotorSpeed + motorOffset + turnSpeed);
  leftBack.setSpeed(baseMotorSpeed + motorOffset + turnSpeed);
  rightBack.run(BACKWARD);
  rightFront.run(BACKWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
  delay(duration);
  stopMove();
  delay(100);
}

int getDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long pulseTime = pulseIn(echo, HIGH, (unsigned long)timeOut);
  float speedOfSound = 0.0343;
  int distance = (pulseTime / 2) * speedOfSound;
  if (pulseTime == 0) return maxDist + 1;
  return distance;
}

int checkDirectionFullScan() {
  int distances[3];
  servoLook.write(180); delay(300);
  distances[0] = getDistance();
  servoLook.write(0); delay(400);
  distances[2] = getDistance();
  servoLook.write(90); delay(300);
  distances[1] = getDistance();

  Serial.print("Distances - Left: "); Serial.print(distances[0]);
  Serial.print(", Front: "); Serial.print(distances[1]);
  Serial.print(", Right: "); Serial.println(distances[2]);

  if (distances[0] > stopDist && distances[2] > stopDist) {
    return (distances[0] >= distances[2]) ? 0 : 2;
  }
  else if (distances[0] > stopDist) return 0;
  else if (distances[2] > stopDist) return 2;
  else return 1;
}