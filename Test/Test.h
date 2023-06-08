
#include "Brakes.h"
#include "ThrottleController.h"
#include "SteeringController.h"

#define steeringPeriod 20000

class Test {
private:
  Brakes *debugBrake;
  ThrottleController *debugThrottle;
  SteeringController *debugSteer;
  unsigned long prevTime_ms;
  int printOnce;

  enum brake_state { BR_OFF,
                     BR_HI_VOLTS,
                     BR_LO_VOLTS };
  brake_state currentBrakeState;

  enum steeringState { LEFT,
                       STRAIGHT,
                       RIGHT
  };
  steeringState currentSteeringState;

public:
  Test();
  void testBrake();
  void testSteering();
  void testThrottle();
};
