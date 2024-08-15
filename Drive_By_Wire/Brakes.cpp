#include <Arduino.h>
#include "DBW_Pins.h"
#include "Settings.h"
#include "Brakes.h"

Brakes::Brakes() {
  // Initialize Pins
  pinMode(BrakeOnPin, OUTPUT);
  pinMode(BrakeVoltPin, OUTPUT);

  // Brakes are released as a default setting
  clock_hi_ms = millis();
  state = BR_OFF;
  Release();

  if (DEBUG)
    Serial.println("Brake Setup Complete");
}


/*  Expected behavior:
   * LEDs go off for relays 2 and 3;
   * Relay 2 has NO (connected to solenoids) open, and there is no power to solenoids.
   * Relay 3 connects COM (other end of solenoid) to NO (12V) 
   * For DBW v4 the relays are wired backwards where the NO and NC are swapped
   */
void Brakes::Release() {
  if (DEBUG)
    Serial.println("RELEASE BRAKE");

  // Release the brakes, state is BR_OFF
  digitalWrite(BrakeOnPin, RELAYInversion ? HIGH : LOW);
  digitalWrite(BrakeVoltPin, RELAYInversion ? HIGH : LOW);
  noInterrupts();
  state = BR_OFF;
  interrupts();
}


/*  Expected behavior:
   *  Both LEDs come on for Relays 2 and 3
   *  Relay 2 connects NO (solenoids) to COM (ground)
   *  Relay 3 connects COM (other end of solenoids) to NC (36V)
   */
void Brakes::Stop() {
  if (DEBUG)
    Serial.println("ACTIVATE BRAKE, applying 24V");

  noInterrupts();
  digitalWrite(BrakeVoltPin, RELAYInversion ? LOW : HIGH);  // Need the higher voltage to activate the solenoid.
  if (state != BR_HI_VOLTS) {
    clock_hi_ms = millis();  // keep track of when the higher voltage was applied.
  }
  digitalWrite(BrakeOnPin, RELAYInversion ? LOW : HIGH);  // Activate solenoid to apply brakes.
  state = BR_HI_VOLTS;
  interrupts();
}


/* Expected behavior
   *  If 36V has been on too long, relay 3 changes LED on to off, switching from 24 to 12V
   *  If the switch is high, brakes will be released, with both LEDs off.
   */
void Brakes::Update() {

  // Keep track of state and when higher voltage was applied
  noInterrupts();
  brake_state tempState = state;
  uint32_t tempClock = clock_hi_ms;
  interrupts();

  unsigned long tick = millis();
  if (tempState == BR_HI_VOLTS && tick - tempClock > MaxHi_ms) {
    // Have reached maximum time to keep voltage high

    if (DEBUG)
      Serial.println("BRAKE SWITCH, switching to 12V");

    digitalWrite(BrakeVoltPin, RELAYInversion ? HIGH : LOW);  // Set to lower voltage, which will keep brakes applied
    noInterrupts();
    state = BR_LO_VOLTS;
    interrupts();
  }
}
