#pragma once

#include "Brakes.h"
#include "ThrottleController.h"
#include "SteeringController.h"
#include "RC_Controller.h"
#include <TimeLib.h> 
#include <SD.h>
#include <stdio.h>
#include "DriveMode.h" // ADDED: Required for DriveMode type

#if DBWversion >= 4
// Only for Arduino Due
#include <due_can.h>     // from due-can library
#endif

extern const char *monthName[12];
extern tmElements_t tm;

class Vehicle {
private:

static RC_Controller* RC;
static ThrottleController* throttle;
static SteeringController* steer;
static Brakes* brake;


File logfile;

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
  int16_t currentRightAngle;

  int brakeHold; // Hold brakes with 12V 
  long throttlePulse_ms;
  long steerPulse_ms;
  int16_t steeringVal; // new val

  // ADDED: New member variables for logging (just declarations)
  DriveMode currentDriveMode;
  bool eStopActive;
  // END ADDED

  //void recieveCan(); // This was commented out in your original code
  void initalize(); 
  void error(char *str);
  void print2digits(int number);
  bool getTime(const char *str);
  bool getDate(const char *str);


public:

  Vehicle();
  ~Vehicle();
  void update();
  void updateRC();
  void eStop();
  void hard_Coded_Test(int16_t, int16_t, int16_t);
  void real_System(int16_t, int16_t, int16_t);
  void LogSD();
  void LogMonitor(); 
  
};