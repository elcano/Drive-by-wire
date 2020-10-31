#pragma once

#include "Brakes.h"
#include "ThrottleController.h"
#include "SteeringController.h"
#include <due_can.h> // added for due conversion ******************

class Vehicle{
private:
	static Brakes brake;
	static ThrottleController throttle;
	SteeringController steer;
 	CAN_FRAME incoming; // added for due conversion ******************
	static volatile int32_t desired_speed_mmPs;
	static volatile int32_t desired_angle;
	int32_t currentSpeed;
	int32_t currentAngle;
  	//static void recieveCan();
  	void recieveCan(); // removed static to verify code compiles -- error using CAN_FRAME incoming in static function -- TODO: how to resolve while using static??
	

public:
	Vehicle();
	~Vehicle();
	void update();
	static void eStop();
  void hard_Coded_Test(int32_t,int32_t);
  void real_System(int32_t,int32_t);
	
};
