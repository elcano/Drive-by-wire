#include <Arduino.h>
#include "DBW_Pins.h"
#include "Settings.h"
#include "Brakes.h"
#define DEBUG 1

// Ensure RELAYInversion is set to false in Brakes.h for this fix to work as intended.
// #define RELAYInversion false (in Brakes.h)

Brakes::Brakes() {
  // Initialize Pins
  pinMode(BrakeOnPin, OUTPUT);
  pinMode(BrakeVoltPin, OUTPUT);

  // Brakes are released as a default setting
  clock_hi_ms = millis();
  state = BR_OFF;
  Release(); // This will now correctly attempt to RELEASE the brakes on startup

  if (DEBUG)
    Serial.println("Brake Setup Complete");
}


/* Expected behavior:
    * This function should physically RELEASE the brakes.
    * Based on observation, sending HIGH to the pins physically DISENGAGES.
    */
void Brakes::Release() {
  if (DEBUG)
    Serial.println("RELEASE BRAKE");

  // Release the brakes, state is BR_OFF
  // Send HIGH to pins to physically DISENGAGE the brakes
  digitalWrite(BrakeOnPin, HIGH);   // Explicitly HIGH
  digitalWrite(BrakeVoltPin, HIGH); // Explicitly HIGH
  noInterrupts();
  state = BR_OFF;
  interrupts();
}


/* Expected behavior:
    * This function should physically ENGAGE the brakes (apply 24V initially).
    * Based on observation, sending LOW to the pins physically ENGAGES.
    */
void Brakes::Stop() {
  if (DEBUG)
    Serial.println("ACTIVATE BRAKE, applying 24V");

  noInterrupts();
  // Apply 24V to activate the solenoid and physically ENGAGE the brakes.
  // Send LOW to pins to physically ENGAGE the brakes
  digitalWrite(BrakeVoltPin, LOW);   // Explicitly LOW for 24V activation
  if (state != BR_HI_VOLTS) {
    clock_hi_ms = millis();   // keep track of when the higher voltage was applied.
  }
  digitalWrite(BrakeOnPin, LOW);   // Explicitly LOW to activate solenoid and hold brakes
  state = BR_HI_VOLTS;
  interrupts();
}


/* Expected behavior
    * If 24V has been on too long, switch to 12V for holding, while keeping brakes engaged.
    */
void Brakes::Update() {

  // Keep track of state and when higher voltage was applied
  noInterrupts();
  brake_state tempState = state;
  uint32_t tempClock = clock_hi_ms; // <-- CORRECTED: Use clock_hi_ms here
  interrupts();

  unsigned long tick = millis();
  if (tempState == BR_HI_VOLTS && tick - tempClock > MaxHi_ms) {
    // Have reached maximum time to keep voltage high

    if (DEBUG)
      Serial.println("BRAKE SWITCH, switching to 12V");

    // To hold brakes at 12V, we need the state that physically ENGAGES (LOW).
    // Assuming BrakeVoltPin LOW selects 12V (and HIGH selects 24V).
    digitalWrite(BrakeVoltPin, HIGH); // Explicitly LOW for 12V holding state
    noInterrupts();
    state = BR_LO_VOLTS;
    interrupts();
  }
}

