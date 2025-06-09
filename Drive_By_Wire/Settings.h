/*
 * Settings.h is meant to capture characteristics of a particular vehicle.
 * There shoould be no Settings.h file in the GitHub repository.
 * The file in the repository should be SettingsTemplate.h
 * Modify this example file to match your individual vehicle.
 * Do not check your local Settings.h into the repository.
 */


#pragma once


/*=======================================================================
Minimum and maximum values to send to the motor
Minimum and maximum speed allowed
*/

// New variables for version 3 or earlier
#define RC_STRAIGHT_MIN_MS 1000
#define RC_STRAIGHT_MAX_MS 1850

// New variables for version 4
#define RC_STRAIGHT_MIN 511
#define RC_STRAIGHT_MAX 511

//min/max acceleration
#define MIN_ACC_OUT 50
#define MAX_ACC_OUT 227
#define MAX_SPEED_KmPh 20

//NEWLY ADDED FOR ESTOP
#define RC_CH3_ESTOP 10


// max speed
const int32_t KmPh_mmPs = 1000000 / 3600;
const int32_t MAX_SPEED_mmPs = MAX_SPEED_KmPh * KmPh_mmPs;  //at 20KMPH this is roughly 5,555


// min speed -> slower is interpreted as stopped
//set as 1% of the maximum speed -> 0.2 KmPh
const int32_t MIN_SPEED_mmPs = 0.01 * MAX_SPEED_mmPs;


/* Time that the brakes can be high */
const uint32_t MaxHi_ms = 2000;


/*=========================================================================


Settings for the Steering
 
Minimum/Maximum and center turning signals
*/


// Minimum turn in Degrees*1000 = Left 24 degrees
#define MIN_TURN_Kdegrees -24000
// Maximum turn in Degrees*1000 = Right 25 degrees
#define MAX_TURN_Kdegrees 25000
// if using pulse width control for steering, the minimum pulse in ms
#define MIN_TURN_MS 1000
// maximum steering servo pulse width in ms
#define MAX_TURN_MS 1850


/* There are wheel angle sensors on the left and right steering columns
   Each returns an analog voltage between 0 and 5 volts.
   After the signal passes through ADC it is a value from 0 to 1023.
   Idealy we would line up the sensors so that 511 is straight ahead.
   It is not practical to get ideal alignment.
   Move the wheels to extreme left to get the read value at MIN_TURN.
   Move the wheels to extreme right to get the read value at MAX_TURN.
   These settings will vary significantly from vehicle to vehicle.
*/
// These values have worked for the Carla simulator
/*
#define Left_Read_at_MIN_TURN 532
#define Left_Read_at_MAX_TURN 206
#define Right_Read_at_MIN_TURN 106
#define Right_Read_at_MAX_TURN 532
*/


// Orange Trike wheel angle sensor
// wheel angle sensor had issues only using right sensor values
#define Left_Read_at_MIN_TURN 532
#define Left_Read_at_MAX_TURN 206
#define Right_Read_at_MIN_TURN 673 // Min_turn =Hard right
#define Right_Read_at_MAX_TURN 786 // Max_turn= Hard left
#define Left_Straight_Read 731 // left side of drive
#define Right_Straight_Read 731 // right side of drive
/*======================================================================
Vehicle Data
Wheel Diameter, Turn Radius
*/


#define WHEEL_DIAMETER_MM 495.3
//derived settings
const int32_t WHEEL_CIRCUM_MM = (WHEEL_DIAMETER_MM * PI);


/*=======================================================================
PID tuning for steering and throttle
*/


#define PID_CALCULATE_TIME 50


// Motor PID
const double proportional_throttle = .0175;
const double integral_throttle = .2;
const double derivative_throttle = .00001;


// Steering PID
const double proportional_steering = .0175;
const double integral_steering = .5;
const double derivative_steering = .00001;
