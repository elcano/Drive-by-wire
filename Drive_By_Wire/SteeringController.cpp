#include <Arduino.h>
#include "PID_v1.h"
#include "DBW_Pins.h"
#include "Settings.h"
#include "SteeringController.h"


SteeringController::SteeringController()
  : steerPID(&steerAngleUS, &PIDSteeringOutput_us, &desiredTurn_us,
             proportional_steering, integral_steering, derivative_steering, DIRECT) {
  
  pinMode(STEER_ON_PIN, OUTPUT);
  pinMode(LEFT_TURN_PIN, OUTPUT);
  pinMode(RIGHT_TURN_PIN, OUTPUT);
  
  digitalWrite(STEER_ON_PIN, HIGH);  // Assumes DBWversion == 4
  digitalWrite(LEFT_TURN_PIN, LOW);
  digitalWrite(RIGHT_TURN_PIN, LOW);

  steerPID.SetOutputLimits(MIN_TURN_MS, MAX_TURN_MS);
  steerPID.SetSampleTime(PID_CALCULATE_TIME);
  steerPID.SetMode(AUTOMATIC);

  Steer_Servo.attach(STEER_PULSE_PIN);
  delay(1);

  if (DEBUG) {
    Serial.println("Steering Setup Complete");
  }
}

SteeringController::~SteeringController() {}

int32_t SteeringController::update(int32_t desiredAngle) {
  int32_t mappedAngle = desiredAngle;

  engageSteering(mappedAngle); 

  delay(1);
  steerAngleUS = computeAngleLeft();  // Use left sensor
  return steerAngleUS;
}

void SteeringController::SteeringPID(int32_t input) {
  desiredTurn_us = input;
  if (PIDSteeringOutput_us != currentSteeringUS) {
    Steer_Servo.writeMicroseconds(PIDSteeringOutput_us);
    currentSteeringUS = PIDSteeringOutput_us;
  }
}

#define NEUTRAL_INPUT_ANGLE 127
#define INPUT_DEADZONE 10

void SteeringController::engageSteering(int32_t input) {

  Serial.println("INPUT: ");
  Serial.println(input);
  if (input > 120 && input < 130 ) {
    digitalWrite(LEFT_TURN_PIN, LOW);
    digitalWrite(RIGHT_TURN_PIN, LOW);
    steeringMode = 0;
    Serial.println("Neutral");
  } else if (input > 130 && input < 280) {
    digitalWrite(LEFT_TURN_PIN, HIGH);  // Turn left
    digitalWrite(RIGHT_TURN_PIN, LOW);
    steeringMode = 1;
    Serial.println("left turn");
  } else {
    digitalWrite(LEFT_TURN_PIN, LOW);
    digitalWrite(RIGHT_TURN_PIN, HIGH);  // Turn right
    steeringMode = -1;
    Serial.println("right turn");
  }
  if (DEBUG) {
    Serial.print("Input Angle: ");
    Serial.println(input);
    Serial.print("Current Sensor Angle: ");
    Serial.println(currentAngle);
  }
}


int32_t SteeringController::computeAngleLeft() {
  int32_t val = analogRead(L_SENSE_PIN);
  if (DEBUG) {
    Serial.print("Left sensor: ");
    Serial.println(val);
  }
  return val;
}

int32_t SteeringController::computeAngleRight() {
  int32_t val = analogRead(R_SENSE_PIN);
  currentAngle = val;
  if (DEBUG) {
    Serial.print("Right sensor: ");
    Serial.println(val);
  }
  return val;
}

int16_t SteeringController::getSteeringMode() {
  return steeringMode;
}
