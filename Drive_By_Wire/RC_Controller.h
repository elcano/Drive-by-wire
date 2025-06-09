#pragma once
#include "DriveMode.h"

// Number of RC channels
#define RC_NUM_SIGNALS 4

// RC Channels
#define RC_CH1_STEERING 0  
#define RC_CH2_THROTTLE_BR 1 
#define RC_CH3_ESTOP 2 
#define RC_CH4_DRIVEMODE 3

// Channel Pins on RC
#if DBWversion < 4
#define STEERING_CH1_PIN 18  
#define THROTTLE_BR_CH2_PIN 19  

#else  // Due
#define STEERING_CH1_PIN 8  
#define THROTTLE_BR_CH2_PIN 9  
#define RC_CH3_ESTOP 10
#define DRIVE_MODE_CH4_PIN 11
#endif

class RC_Controller {

private:
  static unsigned volatile long riseTime[RC_NUM_SIGNALS];
  static unsigned volatile long elapsedTime[RC_NUM_SIGNALS];
  static unsigned long RC_RISE[RC_NUM_SIGNALS];    
  static unsigned long RC_ELAPSED[RC_NUM_SIGNALS]; 

  unsigned long previousTime[RC_NUM_SIGNALS];
  long RC_VALUES_MAPPED[RC_NUM_SIGNALS];  
  static int RC_vals[RC_NUM_SIGNALS];

  long prevSteering;
  long prevThrottleBrake;
  int throttleBrakeFlag; // check valid throttle/brake data
  int steeringFlag; // check valid steering data

  int ch4PulseWidth = 1500;  // Store CH4 pulse width
  DriveMode driveMode = NEUTRAL_MODE;



public:
  RC_Controller();
  ~RC_Controller();
  void clearFlag();
  bool checkValidData();
  void mapValues();
  void mapSteering();
  void mapThrottleBrake();
  long getRawPulse(int channel);
  static void ISR_STEERING_RISE();
  static void ISR_STEERING_FALL();
  static void ISR_THROTTLE_RISE();
  static void ISR_THROTTLE_FALL();
  static void ISR_DRIVEMODE_RISE();
  static void ISR_DRIVEMODE_FALL();
   
  static void ISR_ESTOP_CHANGE();

  static volatile bool estopFlagChanged;
  static volatile unsigned long estopPulseWidth;

    void init();
    void update();  // Add this to update signals
    DriveMode getDriveMode() const;


  long getMappedValue(int channel);
};
