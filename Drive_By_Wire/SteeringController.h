// #pragma once
// #include <Servo.h>
// #include "PID_v1.h"

// class SteeringController {
//   Servo Steer_Servo;
//   PID steerPID;
//   double steerAngleUS;
//   double PIDSteeringOutput_us;
//   double desiredTurn_us;
//   int32_t currentSteeringUS = 0;
//   int32_t currentAngle;
//   int32_t threshold = 20;
//   int16_t steeringMode = 0;
//   void SteeringPID(int32_t input);
//   int32_t computeAngleLeft();
//   //int32_t computeAngleRight();
//   void engageSteering(int32_t input);
// public:
//   SteeringController();
//   ~SteeringController();
//   int32_t update(int32_t desiredAngle);
//   int16_t getSteeringMode();
//    int32_t computeAngleRight();
// };

#pragma once

#include <Servo.h>
#include "PID_v1.h"

class SteeringController {
private:
  Servo Steer_Servo;
  PID steerPID;

  // PID control variables
  double steerAngleUS;
  double PIDSteeringOutput_us;
  double desiredTurn_us;

  // State tracking
  int32_t currentSteeringUS = 0;
  int32_t currentAngle = 0;
  int32_t threshold = 65;
  int16_t steeringMode = 0;

  // Private methods
  void SteeringPID(int32_t input);
  void engageSteering(int32_t input);
  int32_t computeAngleLeft();

public:
  SteeringController();
  ~SteeringController();

  // Main update method
  int32_t update(int32_t desiredAngle);

  // Optional getter for debugging
  int16_t getSteeringMode();

  // Optional use of right sensor (if supported)
  int32_t computeAngleRight();
};

