#pragma once
#include <Servo.h>
#include "PID_v1.h"

class SteeringController {
  Servo Steer_Servo;
  PID steerPID;
  double steerAngleUS;
  double PIDSteeringOutput_us;
  double desiredTurn_us;
  int32_t currentSteeringUS = 0;
  int32_t currentAngle;
  int32_t threshold = 20;
  int16_t steeringMode = 0;
  void SteeringPID(int32_t input);
  int32_t computeAngleLeft();
  //int32_t computeAngleRight();
  void engageSteering(int32_t input);
public:
  SteeringController();
  ~SteeringController();
  int32_t update(int32_t desiredAngle);
  int16_t getSteeringMode();
   int32_t computeAngleRight();
};
