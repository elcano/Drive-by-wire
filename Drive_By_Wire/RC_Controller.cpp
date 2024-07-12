#include <Arduino.h>
#include "RC_Controller.h"
#include "Settings.h"

unsigned volatile long RC_Controller::riseTime[RC_NUM_SIGNALS] = { 0 };     // rising edge, beginning of pulse
unsigned volatile long RC_Controller::elapsedTime[RC_NUM_SIGNALS] = { 0 };  // falling edge, end of pulse

unsigned long RC_Controller::RC_RISE[RC_NUM_SIGNALS] = { 0 };     // stores the beginning of pulse
unsigned long RC_Controller::RC_ELAPSED[RC_NUM_SIGNALS] = { 0 };  // stores the total pulse width


RC_Controller::RC_Controller() {

  // Setup input for RC reciever
  pinMode(STEERING_CH1_PIN, INPUT);
  pinMode(THROTTLE_BR_CH2_PIN, INPUT);

  // Interrupts to pins and ISR functions
  attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_RISE, RISING);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_RISE, RISING);

  // Initialize default values
  for (int i = 0; i < RC_NUM_SIGNALS; i++) {
    RC_VALUES_MAPPED[i] = 0;  // keeps track of mapped values
    previousTime[i] = 0;      // keep track of the time for each signal
  }
  prevSteering = 0;       // keep track of steering data
  prevThrottleBrake = 0;  // keep track of throttle data
}

RC_Controller::~RC_Controller() {}

// Obtain the mapped values for each channel
long RC_Controller::getMappedValue(int channel) {
  return RC_VALUES_MAPPED[channel];
}

// Calls other function to map pulse widths accordingly
void RC_Controller::mapValues() {
  mapThrottleBrake();
  mapSteering();
}

// Maps value into steering
// 779 (Left - 1020 us), 722 (Straight - 1440us), 639 (Right - 1860us)
// new Values for Left angle sensor 630 (Left - 1020 us), 315 (Straight - 1440us), 91 (Right - 1860us)
void RC_Controller::mapSteering() {
  unsigned long currentTime = micros();

  // smooth steering, can change value 2000 is arbitrary value
  if (currentTime - previousTime[RC_CH1_STEERING] >= 2000) {

    int steeringValue = 0;
    long pulseWidth = RC_ELAPSED[RC_CH1_STEERING];
    if (pulseWidth < 1000 || pulseWidth > 2000) {  // invalid data, out of range
      return;
    }

    // filtering pulse widths
    if (prevSteering == pulseWidth) {
      if (pulseWidth > 1500) {  // calibrated steering values
        steeringValue = map(pulseWidth, 1012, 1440, 630, 315);
      } else {
        steeringValue = map(pulseWidth, 1440, 1860, 315, 91);
      }
      RC_VALUES_MAPPED[RC_CH1_STEERING] = steeringValue;
    }

    Serial.println("Steering:" + String(prevSteering));
    previousTime[RC_CH1_STEERING] = currentTime;  // update time
    prevSteering = pulseWidth;                    // steering data
    steeringFlag = 1;                             // valid data
  }
}

// Maps values for throttle (0 - 255); >= 1500us pulse width
// Also calls brake (<1100 us pulse width)
void RC_Controller::mapThrottleBrake() {
  unsigned long currentTime = micros();

  // filtering pulse width at next period cycle (~16.6ms)
  if (currentTime - previousTime[RC_CH2_THROTTLE_BR] >= 16600) {

    long pulseWidth = RC_ELAPSED[RC_CH2_THROTTLE_BR];
    if (pulseWidth < 1000 || pulseWidth > 2000) {  // invalid data, out of range
      return;
    }

    // filter pulse widths
    if (prevThrottleBrake == pulseWidth) {
      if (pulseWidth < 1100) {  // brakes
        RC_VALUES_MAPPED[RC_CH2_THROTTLE_BR] = -1;
        Serial.println("Throttle:" + String(pulseWidth));
      } else if(pulseWidth >= 1500) {  
        int throttleValue;    
            
                                                  // throttle
        throttleValue = map(pulseWidth, 1500, 2000, 0, 120);  // maximum to 120 counts, increase if needed 
        
        RC_VALUES_MAPPED[RC_CH2_THROTTLE_BR] = throttleValue;
      } else {  // set throttle to 0
        RC_VALUES_MAPPED[RC_CH2_THROTTLE_BR] = 0;
      }
    }

    Serial.println("Throttle:" + String(pulseWidth));
    //Serial.println(RC_VALUES_MAPPED[1]);
    delay(10);
    previousTime[RC_CH2_THROTTLE_BR] = currentTime;  // update time
    prevThrottleBrake = pulseWidth;                  // throttle or brake
    throttleBrakeFlag = 1;                           // valid data
  }
}

// Checks for valid data for steering, throttle, and brakes
bool RC_Controller::checkValidData() {
  return (throttleBrakeFlag && steeringFlag);
}

// Clears flag after data processing
void RC_Controller::clearFlag() {
  throttleBrakeFlag = 0;
  steeringFlag = 0;
}


/* Interrupt Service Routine */
void RC_Controller::ISR_STEERING_RISE() {
  if (digitalRead(STEERING_CH1_PIN) == HIGH) {  // filter pulse widths
    noInterrupts();
    riseTime[RC_CH1_STEERING] = micros();
    RC_RISE[RC_CH1_STEERING] = riseTime[RC_CH1_STEERING];
    attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_FALL, FALLING);
    interrupts();
  }
}

void RC_Controller::ISR_STEERING_FALL() {
  if (digitalRead(STEERING_CH1_PIN) == LOW) {  // filter pulse widths
    noInterrupts();
    elapsedTime[RC_CH1_STEERING] = micros() - RC_RISE[RC_CH1_STEERING];
    RC_ELAPSED[RC_CH1_STEERING] = elapsedTime[RC_CH1_STEERING];
    attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_RISE, RISING);
    interrupts();
  }
}

void RC_Controller::ISR_THROTTLE_RISE() {
  if (digitalRead(THROTTLE_BR_CH2_PIN) == HIGH) {  // filter pulse widths
    noInterrupts();
    riseTime[RC_CH2_THROTTLE_BR] = micros();
    RC_RISE[RC_CH2_THROTTLE_BR] = riseTime[RC_CH2_THROTTLE_BR];
    attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_FALL, FALLING);
    interrupts();
  }
}

void RC_Controller::ISR_THROTTLE_FALL() {
  if (digitalRead(THROTTLE_BR_CH2_PIN) == LOW) {  // filter pulse widths
    noInterrupts();
    elapsedTime[RC_CH2_THROTTLE_BR] = micros() - RC_RISE[RC_CH2_THROTTLE_BR];
    RC_ELAPSED[RC_CH2_THROTTLE_BR] = elapsedTime[RC_CH2_THROTTLE_BR];
    attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_RISE, RISING);
    interrupts();
  }
}