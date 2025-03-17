#include "DBW_Pins.h"
#include "Settings.h"
#include "SteeringController.h"

#ifndef TESTING
#include <Arduino.h>
#include "PID_v1.h"
#endif

SteeringController::SteeringController()
    : steerPID(&steerAngleUS, &PIDSteeringOutput_us, &desiredTurn_us,
               proportional_steering, integral_steering, derivative_steering, DIRECT)
{
    pinMode(STEER_ON_PIN, OUTPUT);
    pinMode(LM393_SIGNAL_PIN, INPUT); // lm393 comparator for MOSFET
    pinMode(LEFT_TURN_MOSFET, OUTPUT);  
    pinMode(RIGHT_TURN_MOSFET, OUTPUT);

    // Initialize all MOSFETs as OFF
    digitalWrite(LEFT_TURN_MOSFET, LOW);
    digitalWrite(RIGHT_TURN_MOSFET, LOW);


    steerPID.SetOutputLimits(MIN_TURN_MS, MAX_TURN_MS);
    steerPID.SetSampleTime(PID_CALCULATE_TIME);
    steerPID.SetMode(AUTOMATIC);

    Steer_Servo.attach(STEER_PULSE_PIN);
    delay(1);
    //Steer_Servo.write(90);
    if(DEBUG){
        Serial.println("Steering Setup Complete");
    }
}

SteeringController::~SteeringController()
{
}

int32_t SteeringController::update(int32_t desiredAngle) {
    desiredAngle = map(desiredAngle, MIN_TURN_Mdegrees, MAX_TURN_Mdegrees, MIN_TURN_MS, MAX_TURN_MS);

    if (USE_PIDS) {
        SteeringPID(desiredAngle);
    }
    else {
        engageSteering(desiredAngle);
    }
    //delay(1);
    int32_t steerAngleUS = computeAngleRight();

    return map(currentSteeringUS, MIN_TURN_MS,MAX_TURN_MS,MIN_TURN_Mdegrees,MAX_TURN_Mdegrees);
    //return desiredAngle;
}

//Private
void SteeringController::SteeringPID(int32_t input) {
    desiredTurn_us = input;
    steerPID.Compute();
    if (PIDSteeringOutput_us != currentSteeringUS) {
        controlHBridge(PIDSteeringOutput_us);
        Steer_Servo.writeMicroseconds(PIDSteeringOutput_us);
        currentSteeringUS = PIDSteeringOutput_us;
    }
}

void SteeringController::engageSteering(int32_t input) {
    if (input > MAX_TURN_MS)
        input = MAX_TURN_MS;
    else if (input < MIN_TURN_MS)
        input = MIN_TURN_MS;
    if (currentSteeringUS != input) {
        if (DEBUG) {
            Serial.print("MAP Steering: ");
            Serial.println(input);
        }
        currentSteeringUS = input;
    }
    Steer_Servo.writeMicroseconds(input);
    delay(1);
}

int32_t SteeringController::computeAngleLeft() {
    int32_t val = analogRead(L_SENSE_PIN);
    val = map(val, Left_Read_at_MIN_TURN, Left_Read_at_MAX_TURN, MIN_TURN_MS, MAX_TURN_MS);
    /*if(DEBUG){
        Serial.print("Left sensor: ");
        Serial.println(val);
    } */
    return val;
}

int32_t SteeringController::computeAngleRight() {
    int32_t val = analogRead(R_SENSE_PIN);
    val = map(val, Right_Read_at_MIN_TURN, Right_Read_at_MAX_TURN, MIN_TURN_MS, MAX_TURN_MS);
    /* if(DEBUG){
        Serial.print("Right sensor: ");
        Serial.println(val);
    } */
    return val;
}

// Controls the H-Bridge with the LM393 comparator
void SteeringController::controlHBridge(int32_t target_us) {
    bool lm393_output = digitalRead(LM393_SIGNAL_PIN); 

    if (target_us > currentSteeringUS) {
        // Turning RIGHT
        if (lm393_output == LOW) { 
            digitalWrite(LEFT_TURN_MOSFET, LOW);
            digitalWrite(RIGHT_TURN_MOSFET, HIGH);
        }
    } 
    else if (target_us < currentSteeringUS) {
        // Turning LEFT
        if (lm393_output == LOW) { 
            digitalWrite(RIGHT_TURN_MOSFET, LOW);
            digitalWrite(LEFT_TURN_MOSFET, HIGH);
        }
    } 
    else {
        // No movement or steady state
        digitalWrite(LEFT_TURN_MOSFET, LOW);
        digitalWrite(RIGHT_TURN_MOSFET, LOW);
    }
}
