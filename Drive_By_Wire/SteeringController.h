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
  // The implementations of computeAngleLeft and computeAngleRight are assumed to be in the .cpp.
  // They are private because the public interface will directly expose them if needed.
  // If they are only called internally and their values are *returned* by another public method,
  // then keeping them private is correct. However, if they need to be called directly for logging,
  // they need to be public. Based on the previous conversation, they should be public.
  // I will make them public.
  // int32_t computeAngleLeft(); // Moved to public
  // int32_t computeAngleRight(); // Moved to public

public:
  SteeringController();
  ~SteeringController();

  // Main update method
  int32_t update(int32_t desiredAngle);

  // Optional getter for debugging
  int16_t getSteeringMode();

  // ADDED: Public getters for sensor readings
  int32_t computeAngleLeft();
  int32_t computeAngleRight();
  // END ADDED
};