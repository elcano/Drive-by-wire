#include "Can_Protocol.h"

#ifdef __AVR_ATmega2560__ // MEGA
#include <SPI.h>
#include "mcp2515_can.h"
const int CAN_SS_PIN = 49;
mcp2515_can CAN(CAN_SS_PIN);

#else // DUE
#include <due_can.h>
#endif


void setup() {
#ifdef __AVR_ATmega2560__ // MEGA: Setup for CAN

  Serial.begin(115200);
  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_16MHz)) {
    Serial.println("CAN BUS Shield init fail");
    delay(100);
  }
  Serial.println("CAN BUS init ok!");

#else // DUE: Setup for CAN
  Serial.begin(115200);
  Can0.begin(CAN_BPS_500K);
#endif
}


/*
Arguments for Function:  
currentSpeed: 0 - 255
currentBrake: 0 (release brake), brake > 0 (apply brakes)
currentAngle: Calibration will need to be done/tested again
  - Left: 779 (1ms)
  - Straight: 722 (1.5ms)
  - Right: 639 (1.85ms)
*/
void sendData(int16_t currentSpeed, int16_t currentBrake, int16_t currentAngle) {
  if (currentSpeed < 0 || currentBrake < 0 || currentAngle < 0) {
    return;
  }
  if (currentSpeed > 0 && currentBrake > 0) { // only apply brake if currentSpeed = 0
    currentBrake = 0;  // brakes should not be on, default release brakes
  }

  typedef union {
    struct {
      uint16_t sspeed;
      uint16_t brake;
      uint16_t angle;
      uint16_t reserved;  // not used
    };
  } speedAngleMessage;

  speedAngleMessage MSG;
  MSG.sspeed = currentSpeed;
  MSG.brake = currentBrake;
  MSG.angle = currentAngle;
  MSG.reserved = 0;

#ifdef __AVR_ATmega2560__ // MEGA: Sending CAN Message
  if (CAN_OK == CAN.MCP_CAN::sendMsgBuf(HiDrive_CANID, 0, 8, (uint8_t*)&MSG)) {
    Serial.println("MEGA: Sending Message");
  } else {
    Serial.println("Message failed");
  }

#else // DUE: Sending CAN Message
  CAN_FRAME outgoing;
  outgoing.id = HiDrive_CANID;
  outgoing.extended = false;
  outgoing.length = 8;
  outgoing.data.s0 = MSG.sspeed;
  outgoing.data.s1 = MSG.brake;
  outgoing.data.s2 = MSG.angle;
  if (Can0.sendFrame(outgoing)) {
    Serial.println("DUE: Sending Message");
  } else {
    Serial.println("Message failed");
  }
#endif
}

void loop() {
  //sendData(0,0,0);
  delay(1000);
}