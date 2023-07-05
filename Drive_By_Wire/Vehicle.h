#pragma once

#include "Brakes.h"
#include "ThrottleController.h"
#include "SteeringController.h"
#include "RC_Controller.h"

#if DBWversion >= 4
// Only for Arduino Due
#include <due_can.h>    // from due-can library
#endif

class Vehicle {
private:
  static Brakes brake;
  static ThrottleController throttle;
  SteeringController steer;
  static RC_Controller RC;

#if DBWversion >= 4
  // Only for Due CAN features -- CAN_FRAME is not compatible with Arduino Mega
  // CAN_FRAME is defined in can_common.h
  CAN_FRAME incoming;
  CAN_FRAME outgoing;
#endif
  static volatile int16_t desired_speed_mmPs;
  static volatile int16_t desired_brake;
  static volatile int16_t desired_angle;
  int16_t currentSpeed;
  int16_t currentBrake;
  int16_t currentAngle;
  int brakeHold; // Hold brakes with 12V 
  void recieveCan();


public:
  Vehicle();
  ~Vehicle();
  void update();
  void updateRC();
  void eStop();
  void hard_Coded_Test(int16_t, int16_t, int16_t);
  void real_System(int16_t, int16_t, int16_t);
};
