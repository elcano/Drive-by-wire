#ifndef _DBW_PINS_
#define _DBW_PINS_

#define DEBUG true
#define USE_PIDS false // throttle currently uses a PID controller, Steering does not current use a PID controller

#include <Arduino.h>

//Version can be 1,3 or 4; LLB = Low Level Board = Drive By Wire
#define DBWversion 4

/*---------- Version 1 - No Longer supported------------------------
#if DBWversion == 1
//Wheel click interrupt
#define IRPT_WHEEL         3
#define STEER_OUT_PIN      5
#define STEER_ON           8
#define BrakeVoltPin       9
#define BrakeOnPin        10
#define SelectAB          53
#define SelectCD          49
#define DAC_CHANNEL        0
#define AngleSensorLeft   A2
#define AngleSensorRight  A3
*/
/*---------- Version 3: Arduino Mega -------------------------------------*/
#if DBWversion < 4

/* Pin declarations */
// Serial Monitor uses pins 0,1 for RX0,TX0
// Servo to control the brakes; not used
#define BRAKE_PULSE_PIN     2
 // keep-alive watchdog is not yet implemented
#define WATCHDOG_PIN        5  
// Pin used to steer the vehicle with a pulse
#define STEER_PULSE_PIN     6
// Relay that turns on power to the steering system
#define STEER_ON_PIN        8
// Wheel angle sensors mounted on left and right steering columns
#define L_SENSE_PIN    A2
#define R_SENSE_PIN    A3

// Command to e-bike controller to use regenerative braking; not used 
#define REGEN_PIN          22
// Command to e-bike controller to drive in reverse; not used
#define REVERSE_PIN        24
// Switch to drive in forward or reverse; not used
#define FWDSW_PIN          34
#define D_36_PIN           36  // Not used
// Brakes, have relays for both on/off as well as selecting 12/24v power.
#define BrakeVoltPin       37
#define BUZZER_PIN         38  // Not used
#define BrakeOnPin         39

// CANbus slave select
#define CAN_SS_PIN         49  
// SPI chip select for DAC slave select. Channel A is for throttle
#define DAC_SS_PIN         48 
// we only use channel A for the throttle. B is routed but unused.
#define DAC_CHANNEL         0   
// Pins 51,52,53 are MISO, MOSI, SCK.
// originally intended as a return ground, these analog pins are free and on the steering header.
// Intent was to use L_SENSE - L_RTN as a differential signal.
//#define L_RTN_PIN        A6
//#define R_RTN_PIN        A7

/* Signals A8 through A15 are digital interrupts */
/* The Arduino Mega only has 6 interrupts, but an additional 8 can be added through the port pin change interrupt. */
//Wheel click interrupt (digitally high or low, referred to as an "odometer"). Once per wheel revolution.
// IRPT_WHEEL = WHEELROTATION
#define IRPT_WHEEL         A8
// physical E-Stop port; not used
#define IRPT_ESTOP_PIN     A9 
// Interrupt for can msg; not used
#define IRPT_CAN_PIN       A10 

// motor hall phases occur multiple times per wheel revolution
// A future design may make use of these for more precise speed feedback.
// The speedometer_pin would be jumpered to the signal in use
//Not implemented
#define SPEEDOMETER_PIN    A11
#define IRPT_PHASE_C       A12
#define EBIKE_POWER_IN_PIN A13
#define IRPT_PHASE_B       A14
#define IRPT_PHASE_A       A15


/*---------- Version 4: Arduino Due -------------------------------------*/
#else // DBWversion >=4

// Serial Monitor uses pins 0,1 for RX0,TX0
// Pins D0-D13 and A0-A5 are reserved for the Motor Shield
// The shield may block access to A6 and A7.
/*
 *  --------Motor shield pins --------------
 *  Channel A Current sensing   A0
 *  Channel B Current sensing   A1
 *  In2 Connector               A2
 *  In3 Connector               A3
 *  Channel A Speed (PWM)       3
 *  Out5 connector              5
 *  Out6 connector              6
 *  Channel B Brake             8
 *  Channel A Brake             9
 *  Channel B Speed (PWM)       11
 *  Channel A Direection        12
 *  Channel B Direection        13
 *  
 *  Version 4 connectors:
 *  RJ45 Steering: 
 *        L_RTN, L_SENSE, GND, 5V, STEER_PULSE, GND, R_RTN, R_SENSE, GND, 5V
 *  Two 4 wire screw terminals for CAN bus
 *        12V in, GND, CAN LO, CAN HI
 *  One 4 wire screw terminal for brake solenoid
 *        12V in, GND, 24V in, Brake solenoid out (12 or 24V)
 *        The 12V in on this connector is tied to the 12V in on the CAN bus connector,
 *        There should be only one connector to 12V power in, 
 *        A spare 12V connector could be wired to the motor shield, which
 *        already has a common ground.`
 *  One 4 wire screw terminal:
 *        12V out (controlled by STEER_ON), GND, IRPT_WHEEL, GND
 *  RJ45 to e-bike Motor Controller:
 *        FWDSW, EBIKE_POWER_IN, GND, DAC0, DAC1, SPEEDOMETER, WATCHDOG, 
 *        BRAKE_PULSE, REVERSE, REGEN, 

 */

// Wheel angles sensors mounted on left and right steering columns
#define L_SENSE_PIN        A10
#define R_SENSE_PIN        A11
// originally intended as a return ground, these analog pins are free and on the steering header.
// Intent was to use L_SENSE - L_RTN as a differential signal.
#define L_RTN_PIN          26
#define R_RTN_PIN          27

// Command to e-bike controller to use regenerative braking; not used 
#define REGEN_PIN           22
// The odd numbered pins from 23 to 39 are unassigned.

// Command to e-bike controller to drive in reverse; not used
#define REVERSE_PIN         24
// Pin used to steer the vehicle with a pulse
#define STEER_PULSE_PIN     26
// Relay that turns on power to the steering system
#define STEER_ON_PIN        28
// Debug or status for DBW
#define DBW_LED             30
 // keep-alive watchdog is not yet implemented
#define WATCHDOG_PIN        32  
// Pins 34 and 35 could conflict with the Router board on the Bridge.
// Switch to drive in forward or reverse; not used
#define FWDSW_PIN          36
#define BUZZER_PIN         38  // Not used
#define BrakeOnPin         44
// Brakes, have relays for both on/off as well as selecting 12/24v power.
#define BrakeVoltPin       40

// Servo to control the brakes; not used
#define BRAKE_PULSE_PIN    41
/* Signals D42-49 are digital interrupts */
/* If implemented on aa Arduino Mega, these are on Port L and can use pin change interrupt. */
/* Any Arduino Due pin can be an interrupt. */
// Interrupt for can msg; not used
#define IRPT_CAN_PIN       42
// Multiple clicks per wheel revolution (not implemented)
#define SPEEDOMETER_PIN    43
// When the E-bike controller has power, this line is 5V.
// If it falls to zero, ebike has lost power.
#define EBIKE_POWER_IN_PIN 45

// Wheel click interrupt (digitally high or low, referred to as an "odometer"). Once per wheel revolution.
// Odometer reed switch is pulled up to high voltage.
// IRPT_WHEEL = WHEELROTATION
#define IRPT_WHEEL         47
// DAC channel is he DAC address for throttle; not an Arduino pin.
#define DAC_CHANNEL         0   
/*------------ Dropped signals ------------------
 * The following signals have been dropped
// Motor hall phases occur multiple times per wheel revolution
// A future design may make use of these for more precise speed feedback.
// The speedometer_pin would be jumpered to the signal in use
   The three Hall Phases were never used: IRPT_PHASE_A, IRPT_PHASE_B, IRPT_PHASE_C
   If we use a signal from the hub motor, it will be on the SPEEDOMETER_PIN 

// physical E-Stop port; not used
  IRPT_ESTOP_PIN is not used. It was intended for a remote e-stop, but the
  e-stop now comes from the CAN bus.
  D_36_PIN    // Never used; intended as a spare signal
 -------------------------------------------------------------*/

// On the Due, SPI, CAN_SS and DAC_SS are not used. Use DAC0, DAC1, CANRX and CANTX  
// On Due MISO, MOSI, SCK are only on the ICSP, which is blocked by the motor shield.

#endif  // DBWversion 

#endif   //_DBW_PINS_
