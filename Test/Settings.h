/*
 * Settings.h is meant to capture characteristics of a particular vehicle.
 * There shoould be no Settings.h file in the GitHub repository.
 * The file in the repository should be SettingsTemplate.h
 * Modify this example file to match your individual vehicle.
 * Do not check your local Settings.h into the repository.
 */

#pragma once

/*
Minimum and maximum values to send to the motor
Minimum and maximum speed allowed
(currently minimum is set to 1% of the maximum for no real reason)
*/

//min/max acceleration
#define MIN_ACC_OUT 50
#define MAX_ACC_OUT 227
#define MAX_SPEED_KmPh 20

// max speed 
const int32_t KmPh_mmPs = 1000000/3600;
const int32_t MAX_SPEED_mmPs = MAX_SPEED_KmPh*KmPh_mmPs;  //at 20KMPH this is roughly 5,555

// min speed -> slower is interpreted as stopped
//set as 1% of the maximum speed -> 0.2 KmPh
const int32_t MIN_SPEED_mmPs = 0.01*MAX_SPEED_mmPs;

/* Time that the brakes can be high */
const uint32_t MaxHi_ms = 800;

/*
Settings for the Steering 
Minimum/Maximum and center turning signals
*/

#if CARLA
#define MIN_TURN_Mdegrees -24000
#define MAX_TURN_Mdegrees 25000
#define MIN_Right_Sensor 725 //not used
#define MAX_Right_Sensor 785 //not used
#define MIN_Left_Sensor 485 //not used
#define MAX_Left_Sensor 313 //not used
#define Left_Read_at_MIN_TURN  532
#define Left_Read_at_MAX_TURN 206
#define Right_Read_at_MIN_TURN 106
#define Right_Read_at_MAX_TURN 532
#define MIN_TURN_MS 1000
#define MAX_TURN_MS 1850

#else
#define MIN_TURN_Mdegrees -24000
#define MAX_TURN_Mdegrees 25000
#define MIN_Right_Sensor 725
#define MAX_Right_Sensor 785
#define MIN_Left_Sensor 485
#define MAX_Left_Sensor 313
#define Left_Read_at_MIN_TURN  485
#define Right_Read_at_MIN_TURN 725
#define Left_Read_at_MAX_TURN 313
#define Right_Read_at_MAX_TURN 785
#define MIN_TURN_MS 1000
#define MAX_TURN_MS 1850
#endif

/*
Vehicle Data
Wheel Diameter, Turn Radius
*/

#define WHEEL_DIAMETER_MM 495.3
//derived settings
const int32_t WHEEL_CIRCUM_MM = (WHEEL_DIAMETER_MM * PI);

/*
PID tunning for steering and throttle
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
