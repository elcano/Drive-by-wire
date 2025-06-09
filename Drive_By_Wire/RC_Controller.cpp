#include <Arduino.h>
#include "RC_Controller.h"
#include "DBW_Pins.h"
#include "Settings.h"
#include "DriveMode.h"

unsigned volatile long RC_Controller::riseTime[RC_NUM_SIGNALS] = { 0 };
unsigned volatile long RC_Controller::elapsedTime[RC_NUM_SIGNALS] = { 0 };

unsigned long RC_Controller::RC_RISE[RC_NUM_SIGNALS] = { 0 };
unsigned long RC_Controller::RC_ELAPSED[RC_NUM_SIGNALS] = { 0 };

volatile bool RC_Controller::estopFlagChanged = false;
volatile unsigned long RC_Controller::estopPulseWidth = 1500;
int currentDriveMode = 0; 

RC_Controller::RC_Controller() {
  pinMode(STEERING_CH1_PIN, INPUT);
  pinMode(THROTTLE_BR_CH2_PIN, INPUT);
  pinMode(RC_CH3_ESTOP, INPUT);
  pinMode(DRIVE_MODE_CH4_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_RISE, RISING);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_RISE, RISING); 
  attachInterrupt(digitalPinToInterrupt(RC_CH3_ESTOP), RC_Controller::ISR_ESTOP_CHANGE, CHANGE);

  for (int i = 0; i < RC_NUM_SIGNALS; i++) {
    RC_VALUES_MAPPED[i] = 0;
    previousTime[i] = 0;
  }

  prevSteering = 0;
  prevThrottleBrake = 0;
}

RC_Controller::~RC_Controller() {}

long RC_Controller::getMappedValue(int channel) {
  return RC_VALUES_MAPPED[channel];
}

void RC_Controller::mapValues() {
  mapThrottleBrake();
  mapSteering();

  //maps Estop
  unsigned long estopPulse = RC_ELAPSED[RC_CH3_ESTOP];
  RC_VALUES_MAPPED[RC_CH3_ESTOP] = estopPulse;
  
}

void RC_Controller::mapSteering() {
  unsigned long currentTime = micros();
  if (currentTime - previousTime[RC_CH1_STEERING] >= 2000) {
    int steeringValue = 0;
    long pulseWidth = RC_ELAPSED[RC_CH1_STEERING];
    if (pulseWidth < 1000 || pulseWidth > 2000) return;

    if (prevSteering != pulseWidth) {
      if (pulseWidth < 1440)
        steeringValue = map(pulseWidth, 1000, 1440, 630, 315);
      else
        steeringValue = map(pulseWidth, 1440, 1996, 91, 315);

      RC_VALUES_MAPPED[RC_CH1_STEERING] = steeringValue;
    }

    Serial.println("Steering:" + String(prevSteering));
    previousTime[RC_CH1_STEERING] = currentTime;
    prevSteering = pulseWidth;
    steeringFlag = 1;
  }
}

void RC_Controller::mapThrottleBrake() {
  unsigned long currentTime = micros();
  if (currentTime - previousTime[RC_CH2_THROTTLE_BR] >= 16600) {
    long pulseWidth = RC_ELAPSED[RC_CH2_THROTTLE_BR];
    if (pulseWidth < 1000 || pulseWidth > 2000) return;

    if (pulseWidth < 1100) {
      RC_VALUES_MAPPED[RC_CH2_THROTTLE_BR] = -1;
      Serial.println("Throttle:" + String(pulseWidth));
    } else if (pulseWidth >= 1500) {
      int throttleValue = map(pulseWidth, 1500, 2000, 0, 150);
      RC_VALUES_MAPPED[RC_CH2_THROTTLE_BR] = throttleValue;
    } else {
        // Neutral state — neither throttle nor brake
        RC_VALUES_MAPPED[RC_CH2_THROTTLE_BR] = 0;
    }

    Serial.println(RC_VALUES_MAPPED[RC_CH2_THROTTLE_BR]);

    previousTime[RC_CH2_THROTTLE_BR] = currentTime;
    prevThrottleBrake = pulseWidth;
    throttleBrakeFlag = 1;
  }
}

bool RC_Controller::checkValidData() {
  return (throttleBrakeFlag && steeringFlag);
}

void RC_Controller::clearFlag() {
  throttleBrakeFlag = 0;
  steeringFlag = 0;
}

void RC_Controller::updateDriveSelection() {
  ch4PulseWidth = pulseIn(DRIVE_MODE_CH4_PIN, HIGH, 100000);  // 100ms timeout

  if (ch4PulseWidth < 500 || ch4PulseWidth > 3000) {
    // Ignore invalid pulse
    return;
  }

  //removed reverse mode since it wasnt working with neutral.
  if (ch4PulseWidth < 1200) {
    driveMode = NEUTRAL_MODE;
  } else if (ch4PulseWidth < 2300) {
    driveMode = NEUTRAL_MODE;
  } else {
    driveMode = DRIVE_MODE;
  }
}

DriveMode RC_Controller::getDriveMode() const {
    return driveMode;
}

bool RC_Controller::processEStop() {
  if (!estopFlagChanged) return estopState;

  estopFlagChanged = false;

  Serial.print("E-STOP Pulse: ");
  Serial.println(estopPulseWidth);

  if (estopPulseWidth > 1800) {
    if (!estopState) {
      estopState = true;
      Serial.println(">>> E-STOP ON <<<");
    }
  } else if (estopPulseWidth < 1200) {
    if (estopState) {
      estopState = false;
      Serial.println(">>> E-STOP OFF <<<");
    }
  }

  return estopState;
}


bool RC_Controller::isEStopActive() const {
  return estopState;
}

void RC_Controller::ISR_STEERING_RISE() {
  if (digitalRead(STEERING_CH1_PIN) == HIGH) {
    noInterrupts();
    riseTime[RC_CH1_STEERING] = micros();
    RC_RISE[RC_CH1_STEERING] = riseTime[RC_CH1_STEERING];
    attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_FALL, FALLING);
    interrupts();
  }
}

void RC_Controller::ISR_STEERING_FALL() {
  if (digitalRead(STEERING_CH1_PIN) == LOW) {
    noInterrupts();
    elapsedTime[RC_CH1_STEERING] = micros() - RC_RISE[RC_CH1_STEERING];
    RC_ELAPSED[RC_CH1_STEERING] = elapsedTime[RC_CH1_STEERING];
    attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_RISE, RISING);
    interrupts();
  }
}

void RC_Controller::ISR_THROTTLE_RISE() {
  if (digitalRead(THROTTLE_BR_CH2_PIN) == HIGH) {
    noInterrupts();
    riseTime[RC_CH2_THROTTLE_BR] = micros();
    RC_RISE[RC_CH2_THROTTLE_BR] = riseTime[RC_CH2_THROTTLE_BR];
    attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_FALL, FALLING);
    interrupts();
  }
}

void RC_Controller::ISR_THROTTLE_FALL() {
  if (digitalRead(THROTTLE_BR_CH2_PIN) == LOW) {
    noInterrupts();
    elapsedTime[RC_CH2_THROTTLE_BR] = micros() - RC_RISE[RC_CH2_THROTTLE_BR];
    RC_ELAPSED[RC_CH2_THROTTLE_BR] = elapsedTime[RC_CH2_THROTTLE_BR];
    attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_RISE, RISING);
    interrupts();
  }
}


void RC_Controller::ISR_ESTOP_CHANGE() {
  static unsigned long estopRise = 0;
  static unsigned long prevPulse = 1500;
  static unsigned long lastValidPulseTime = 0;  // <-- debounce time marker

  bool pinState = digitalRead(RC_CH3_ESTOP);
  unsigned long now = micros();

  if (pinState) {
    estopRise = now;
  } else {
    unsigned long pulse = now - estopRise;

    // Debounce: Require at least 50 ms between accepted changes
    if ((now - lastValidPulseTime) > 50000) { // 50 ms
      if (pulse >= 1000 && pulse <= 2100) {
        if (abs((int32_t)pulse - (int32_t)prevPulse) > 50) {
          estopPulseWidth = pulse;
          estopFlagChanged = true;
          prevPulse = pulse;
          lastValidPulseTime = now;
        }
      }
    }
  }
}
















