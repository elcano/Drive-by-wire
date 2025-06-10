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


  Release(); // This will now apply the "release" logic based on the new RELAYInversion
  // If the initial power-on state still engages brakes, this line might need to be removed
  // or adjusted. Let's try this first.


  if (DEBUG)
    Serial.println("Brake Setup Complete");
}


/* Expected behavior:

    * This function should physically RELEASE the brakes.
    * Based on observation, sending HIGH to the pins physically DISENGAGES.

    * LEDs go off for relays 2 and 3;
    * Relay 2 has NO (connected to solenoids) open, and there is no power to solenoids.
    * Relay 3 connects COM (other end of solenoid) to NO (12V) 

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

    * Both LEDs come on for Relays 2 and 3
    * Relay 2 connects NO (solenoids) to COM (ground)
    * Relay 3 connects COM (other end of solenoids) to NC (36V)

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

    * If 36V has been on too long, relay 3 changes LED on to off, switching from 24 to 12V
    * If the switch is high, brakes will be released, with both LEDs off.

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


