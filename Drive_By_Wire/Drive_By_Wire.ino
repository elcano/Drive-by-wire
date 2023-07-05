/* Drive_By_Wire
 *  Accept commands over CAN bus
 *  Implement vehicle throttle, brakes and steering as commanded
 *  Report status over CAN bus
 */
#include "DBW_Pins.h"
#include <SPI.h>
#include "Vehicle.h"

#define baud 115200  // baudrate for debugging with a host PC over USB serial

Vehicle *myTrike;

//Timing stuff
#define LOOP_TIME_MS 100
uint32_t nextTime;
uint32_t endTime;
uint32_t delayTime;
#define ULONG_MAX 0x7FFFFFFF

void setup() {
  Serial.begin(baud);
  if (DEBUG) {
    Serial.println("main Setup complete");
  }
  myTrike = new Vehicle();
}

void loop() {
  //Timing code
  nextTime = nextTime + LOOP_TIME_MS;
  uint32_t timeStart_ms = millis();

  //myTrike->update();
  myTrike->updateRC();

  //Timing code
  endTime = millis();
  delayTime = 0UL;
  if ((nextTime >= endTime) && (((endTime < LOOP_TIME_MS) && (nextTime < LOOP_TIME_MS)) || ((endTime >= LOOP_TIME_MS) && (nextTime >= LOOP_TIME_MS)))) {
    delayTime = nextTime - endTime;
    if (nextTime < LOOP_TIME_MS)
      delayTime = ULONG_MAX - endTime + nextTime;
    else {
      nextTime = endTime;
      delayTime = 0UL;
    }
  }
  if (delayTime > 0UL)
    delay(delayTime);
}
