#pragma once

#include "Brakes.h"
#include "ThrottleController.h"
#include "SteeringController.h"
#ifdef __SAM3X8E__
	// Only for Arduino Due
  	#include <due_can.h> 
#endif

class Vehicle{
private:
	static Brakes brake;
	static ThrottleController throttle;
	SteeringController steer;
  	#ifdef __SAM3X8E__
	  	// Only for Due CAN features -- CAN_FRAME is not compatible with Arduino Mega
		// CAN_FRAME is defined in can_common.h
		CAN_FRAME incoming; 
		CAN_FRAME outgoing;
	#endif
	static volatile int32_t desired_speed_mmPs;
	static volatile int32_t desired_angle;
	int32_t currentSpeed;
	int32_t currentAngle;
  	//static void recieveCan();
  	void recieveCan(); 
	

public:
	Vehicle();
	~Vehicle();
	void update();
	static void eStop();
  void hard_Coded_Test(int32_t,int32_t);
  void real_System(int32_t,int32_t);
	
};
