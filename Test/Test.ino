/* Drive_By_Wire
 *  Accept commands over CAN bus
 *  Implement vehicle throttle, brakes and steering as commanded
 *  Report status over CAN bus
 */


#include "DBW_Pins.h"
#include <SPI.h>
#include "Test.h"





#ifdef __AVR_ATmega2560__
  #include "mcp_can.h"
#endif

#define baud 115200   // baudrate for debugging with a host PC over USB serial

Test *myTest;

void setup() {
  Serial.begin(baud);
  if (DEBUG) {
    Serial.println("main Setup complete");
  }
  myTest = new Test();
}

// Note: Each function from test.cpp is supposed to be called one at a time. 
void loop()
{
  myTest->testBrake();
  //myTest->testSteering();
  //myTest->testThrottle();
}
