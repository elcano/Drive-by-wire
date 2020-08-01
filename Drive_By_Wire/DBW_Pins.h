#ifndef _DBW_PINS_
#define _DBW_PINS_

#define CARLA true
#define DEBUG true
#define USE_PIDS false

#ifndef TESTING
#include <Arduino.h>
#endif

//Version can be 1,3 or 4; LLB = Low Level Board = Drive By Wire
#define LLBversion 3

/*---------- Version 1 - No Longer supported------------------------
#if LLBversion == 1
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
/*---------- Version 3 -------------------------------------*/
#if LLBversion == 3

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

// Wheel angles sensors munted on left and right steering columns
#define L_SENSE_PIN        A2
#define R_SENSE_PIN        A3
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
#define EBIKE_CONTROL_PIN  A13
#define IRPT_PHASE_B       A14
#define IRPT_PHASE_A       A15


/*---------- Version 4 -------------------------------------*/
#else 
#endif

#endif   //_DBW_PINS_
