#include "DBW_Pins.h"
#include "Test.h"
#include "Can_Protocol.h"

// NOTE: All of these functions depend on the .ino loop()
// - Functions are meant to test one by one. 

Test::Test() {
  currentBrakeState = BR_OFF;
  currentSteeringState = LEFT;
  printOnce = 0;
  prevTime_ms = millis();
  debugBrake = new Brakes();
  debugSteer = new SteeringController();
  debugThrottle = new ThrottleController();
}

/*
* testBrake(): Controls the braking by activating relays and solenoid (DBW v3, check DBW_Pins.h)
* BrakeVoltPin (D37) - 24V/12V (relay)
* BrakeOnPin (D39) - Solenoid (controls the braking)

* Assumption: No relay inversion, 36V PWR Supply
* Expected Behavior: 24V (to activate brakes); 12V (to hold brakes)
* - Stop: Activate 24V (for 800ms) and Solenoid (VoltPin/BrakeOnPin - HIGH)
* - Update: Switch to 12V (VoltPin - LOW); Solenoid (BrakeOnPin - HIGH)
* - Release(): Release brakes (VoltPin/BrakeOnPin - LOW)
*
* Timing Behavior: 
* - Update() waits about 2 seconds (MaxHi_ms) before switching to 12V
* - Release() timing behavior: 
*     First Loop: Update() waits 800ms -> calls release after 5.2 seconds 
*     Nth Loop (due to 2000 ms delay): Calls release after 3.2 seconds 
*/
void Test::testBrake() {
  unsigned long currentTime = millis();

  // Activate the brakes
  if (currentBrakeState == BR_OFF) {
    Serial.println("Activating the Brakes with 24V");
    debugBrake->Stop();
    currentBrakeState = BR_HI_VOLTS;
  }

  // Hold the brakes, calls this function most of the time.
  // Note: Switches to 12V (either to 800ms or 2 seconds)
  debugBrake->Update();
  currentBrakeState = BR_LO_VOLTS;

  // Release the brakes
  // Calls release after "6" seconds (see timing behavior)
  if (currentTime - prevTime_ms >= 6000) {
    prevTime_ms = currentTime;
    Serial.println("Releasing the Brakes");
    debugBrake->Release();
    currentBrakeState = BR_OFF;
    delay(2000);
  }
}

/*
* testThrottle(): Send a 1V - 4V signal to throttle pin (DBW v3, check DBW_Pins.h)
* Throttle Pin: DACA - not a pin on Arduino
* Assumption: 36V PWR, DACA 
* 
* Update the following parameters (LAST UPDATE: May 10, 2013, TCF)
* 0.831 V at rest 52 counts
* 1.20 V: nothing 75
* 1.27 V: just starting 79
* 1.40 V: slow, steady 87
* 1.50 V: brisker 94
* 3.63 V: max 227 counts
* 255 counts = 4.08 V
*
* Expected Behavior: 
* - Gradually increase speed (sweeping to 1V - 4V)
* - Gradually decrease speed (sweeping to 4V - 1V)
* - Stop for 3 seconds
*/
void Test::testThrottle() {

  // Accelerate
  Serial.println("Accelerate:");
  for (int i = 75; i <= 227; i++) {
    debugThrottle->update(i);
    delay(150);
  }
  delay(2000);

  // Decelerate
  Serial.println("Decelerate:");
  for (int j = 227; j >= 75; j--) {
    debugThrottle->update(j); 
    delay(150);
  }
  delay(2000);

  // Stop
  Serial.println("Stop:");
  debugThrottle->stop();
  delay(3000);
}

// Calls left, middle, right every three seconds
void Test::testSteering() {
  Serial.println("Current Position: Left");
  debugSteer->update(-24000);
  delay(3000);

  Serial.println("Current Position: Straight");
  debugSteer->update(0);
  delay(3000);

  Serial.println("Current Position: Right");
  debugSteer->update(25000);
  delay(3000);
}
