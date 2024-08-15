#pragma once
#include <Servo.h>
#include "PID_v1.h"

class SteeringController {
  Servo Steer_Servo;
  PID steerPID; // currently not used summer 2024
  double steerAngleUS; 
  double PIDSteeringOutput_us;
  double desiredTurn_us; // input 
  int32_t currentSteeringUS = 0;
  int32_t currentAngle;
  int32_t threshold = 20; // angle threshold based on noise and determines when the steering is good enougth to hold the steering position
  int16_t steeringMode = 0; // used to indicate current steering board operation in the datalogger
  int16_t waitCycles; // adds delay between left and right turns to protect the steering board
  void SteeringPID(int32_t input);
  int32_t computeAngleLeft();
  void engageSteering(int32_t input);
public:
  SteeringController();
  ~SteeringController();
  int32_t update(int32_t desiredAngle);
  int16_t getSteeringMode();
  int32_t computeAngleRight(); // right angle sensor is less accurate than left sensor and is currently unused and it is public for datalogging purposes 
};
