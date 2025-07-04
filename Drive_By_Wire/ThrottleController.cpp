#include <Arduino.h>
#include "DBW_Pins.h"
#if DBWversion < 4
// Only for Arduino Mega
#include <PinChangeInterrupt.h>
#endif  // Mega
#include "ThrottleController.h"
#include "DriveMode.h"


volatile uint32_t ThrottleController::tickTime_ms[2];

ThrottleController::ThrottleController()
  : speedPID(&speedCyclometerInput_mmPs, &PIDThrottleOutput_pwm, &desiredSpeed_mmPs, proportional_throttle, integral_throttle, derivative_throttle, DIRECT) {

  currentThrottlePWM = 0;

  speedPID.SetOutputLimits(MIN_ACC_OUT, MAX_ACC_OUT);
  speedPID.SetSampleTime(PID_CALCULATE_TIME);
  speedPID.SetMode(AUTOMATIC);
  calcTime_ms[0] = 0;
  calcTime_ms[1] = 0;
  prevSpeed_mmPs = 0;

#if DBWversion < 4
  // Only for Arduino Mega
  pinMode(DAC_SS_PIN, OUTPUT);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();

  if (IRPT_WHEEL != 3)
    attachPCINT(digitalPinToPCINT(IRPT_WHEEL), tick, RISING);
  else
    attachInterrupt(digitalPinToInterrupt(IRPT_WHEEL), tick, RISING);  //pin 3 on Mega
#else    // Due
  attachInterrupt(IRPT_WHEEL, tick, RISING);
#endif  // Mega
  if (DEBUG)
    Serial.println("Throttle Setup Complete");
}

ThrottleController::~ThrottleController() {
}


/*
sets the throttle signal to zero
*/
void ThrottleController::stop() {
  write(DAC_CHANNEL, 0);
  currentThrottlePWM = 0;
}

// not used?
void ThrottleController::tick() {
  uint32_t tick = millis();
  noInterrupts();
  if ((tick - tickTime_ms[0]) > MIN_TICK_TIME_ms) {
    tickTime_ms[1] = tickTime_ms[0];
    tickTime_ms[0] = tick;
  }
  interrupts();
}

/**
 * receives requested speed from Vehicle requested from High Level board
 * attempts to adjust speed either through PIDS or standard engageThrottle() 
 * based on the PIDs being on or off in Settings
 * param dSpeed desired speed in mm/s
 */
int32_t ThrottleController::update(int32_t dSpeed, DriveMode mode) {
  if (mode == NEUTRAL_MODE) {
    stop();  // Disable DAC output
    return 0;
  }

  //Reverse doesnt seem to work
  // if (mode == REVERSE_MODE) {
  //   dSpeed = constrain(dSpeed, 0, 150);  // prevent negatives
  //   dSpeed = map(dSpeed, 0, 150, 150, 0);  // invert DAC for reverse, or just limit
  // }

  if (USE_PIDS)
    ThrottlePID(dSpeed);
  else
    engageThrottle(dSpeed);

  computeSpeed();

  if (DEBUG) {
    Serial.println("mm Speed: " + String(speedCyclometerInput_mmPs));
    Serial.print("PWM speed: ");
    Serial.println(currentThrottlePWM);
  }

  return dSpeed;
}



//Private functions
void ThrottleController::write(int32_t address, int32_t value) {

#if DBWversion < 4
  // Only for Arduino Mega
  int byte1 = ((value & 0xF0) >> 4) | 0x10;  // active mode, bits D7-D4
  int byte2 = (value & 0x0F) << 4;
  if (address < 2) {
    // take the SS pin low to select the chip:
    digitalWrite(DAC_SS_PIN, LOW);
    if (address >= 0) {
      if (address == 1)
        byte1 |= 0x80;  // second channnel
      SPI.transfer(byte1);
      SPI.transfer(byte2);
    }
    // take the SS pin high to de-select the chip:
    digitalWrite(DAC_SS_PIN, HIGH);
  } 
  else {
    // take the SS pin low to select the chip:
    digitalWrite(DAC_SS_PIN, LOW);
    if (address <= 3) {
      if (address == 3)
        byte1 |= 0x80;  // second channnel
      SPI.transfer(byte1);
      SPI.transfer(byte2);
    }
    // take the SS pin high to de-select the chip:
    digitalWrite(DAC_SS_PIN, HIGH);
  }
#else  // Arduino Due

  // Addresss should be 0
  // value is 0 to 4095.
  if (address != 1)
    analogWrite(DAC0, value);
  else
    analogWrite(DAC1, value);

//   if (address != 1)
//   analogWrite(DAC0, map(value, 0, 120, 0, 4095));
// else
//   analogWrite(DAC1, map(value, 0, 120, 0, 4095));
#endif
}



void ThrottleController::ThrottlePID(int32_t desiredValue) {
  if (desiredValue >= (speedCyclometerInput_mmPs + 10)) {
    noInterrupts();
    speedPID.Compute();
    //currentThrottlePWM = (int32_t)PIDThrottleOutput_pwm;
    engageThrottle(PIDThrottleOutput_pwm);
    interrupts();
  }
}

/** 
 *  
 *  @param input desired speed in mm/s
 *  !UPDATE THIS OBSERVED INFO! (LAST UPDATE: May 10, 2013, TCF)
0.831 V at rest 52 counts
1.20 V: nothing 75
1.27 V: just starting 79
1.40 V: slow, steady 87
1.50 V: brisker 94
3.63 V: max 227 counts
255 counts = 4.08 V


Current Calibration: (Last Update: May 14, 2023)
1.187 V: nothing 75
1.20 V: Just starting 82
1.345 V: Slow, steady 85
1.50 V: Briskter, 100
1.85 V: 120 - Maximum Count (for current testing purposes, can extend maximum count)
*/

/* need to fix this function */
void ThrottleController::engageThrottle(int32_t input) {

  // Reset the DAC once if power is off (DAC does not write the same value when PWR is off)
  if (throttleSet == 0) {
    if (DEBUG)
      Serial.println("Reset DAC");
    write(DAC_CHANNEL, 0);
    throttleSet = 1;
  }

  /*  // speedometer, currently not using. 
  int32_t temp = input;
  if (input != 0){
    //needs to be updated, currently this will change the voltage to a desired
    //speed based on starting at 0, it doesn't take into account the current speed of trike
    input = map(input, 0, MAX_SPEED_mmPs, MIN_ACC_OUT, MAX_ACC_OUT);
  }
  */

  if (DEBUG)
    Serial.println("MAPPED speed: " + String(input));

  if (input != currentThrottlePWM) {
    if (input > 255) {
      input = 255;
    } else if (input < 0) {
      input = 0;
    }

    noInterrupts();
    write(DAC_CHANNEL, input);
    currentThrottlePWM = input;  // Remember most recent throttle PWM value.
    interrupts();
  }
}


int32_t ThrottleController::extrapolateSpeed() {
  int32_t y;
  int32_t t = millis();
  //slope calculation
  y = (speedCyclometerInput_mmPs - prevSpeed_mmPs) / (calcTime_ms[0] - calcTime_ms[1]);
  // * change in time
  y *= (t - calcTime_ms[0]);
  // + current speed
  y += speedCyclometerInput_mmPs;

  if (y < 0)
    y = 0;
  return y;
}

/*
Uses previous two speeds to extrapolate the current speed
Used to determine when we have stopped
*/
void ThrottleController::computeSpeed() {
  uint32_t tempTick[2];
  noInterrupts();
  tempTick[0] = tickTime_ms[0];
  tempTick[1] = tickTime_ms[1];
  interrupts();
  if (tempTick[1] == 0)
    speedCyclometerInput_mmPs = 0;
  else if (calcTime_ms[0] == 0) {
    speedCyclometerInput_mmPs = WHEEL_CIRCUM_MM * (1000.0 / (tempTick[0] - tempTick[1]));
    prevSpeed_mmPs = speedCyclometerInput_mmPs;
    calcTime_ms[1] = tempTick[1];
    calcTime_ms[0] = tempTick[0];
  } else {
    if (calcTime_ms[1] == tempTick[1]) {
      uint32_t timeDiff = millis() - calcTime_ms[0];
      if (timeDiff > MAX_TICK_TIME_ms) {
        speedCyclometerInput_mmPs = 0;
        if (timeDiff > (2 * MAX_TICK_TIME_ms)) {
          prevSpeed_mmPs = 0;
          noInterrupts();
          tickTime_ms[1] = 0;
          interrupts();
        }
      } else if (prevSpeed_mmPs > speedCyclometerInput_mmPs) {
        speedCyclometerInput_mmPs = extrapolateSpeed();
      }
    } else {
      calcTime_ms[1] = calcTime_ms[0];
      calcTime_ms[0] = tempTick[0];
      prevSpeed_mmPs = speedCyclometerInput_mmPs;
      speedCyclometerInput_mmPs = WHEEL_CIRCUM_MM * (1000.0 / (tempTick[0] - tempTick[1]));
    }
  }
}

int ThrottleController::getCurrentThrottlePWM() const {
    return currentThrottlePWM;
}
