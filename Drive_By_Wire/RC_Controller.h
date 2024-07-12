#pragma once

// Number of RC channels
#define RC_NUM_SIGNALS 3

// RC Channels
#define RC_CH1_STEERING 0  
#define RC_CH2_THROTTLE_BR 1  

// Channel Pins on RC
#if DBWversion == 3
#define STEERING_CH1_PIN 18  
#define THROTTLE_BR_CH2_PIN 19  
#else  // Due
#define STEERING_CH1_PIN 48
#define THROTTLE_BR_CH2_PIN 46  
#endif

class RC_Controller {

private:
  static unsigned volatile long riseTime[RC_NUM_SIGNALS];
  static unsigned volatile long elapsedTime[RC_NUM_SIGNALS];
  static unsigned long RC_RISE[RC_NUM_SIGNALS];    
  static unsigned long RC_ELAPSED[RC_NUM_SIGNALS]; 

  unsigned long previousTime[RC_NUM_SIGNALS];
  long RC_VALUES_MAPPED[RC_NUM_SIGNALS];  

  long prevSteering;
  long prevThrottleBrake;
  int throttleBrakeFlag; // check valid throttle/brake data
  int steeringFlag; // check valid steering data

public:
  RC_Controller();
  ~RC_Controller();
  void clearFlag();
  bool checkValidData();
  void mapValues();
  void mapSteering();
  void mapThrottleBrake();
  static void ISR_STEERING_RISE();
  static void ISR_STEERING_FALL();
  static void ISR_THROTTLE_RISE();
  static void ISR_THROTTLE_FALL();
  long getMappedValue(int channel);
};
