#include "DBW_Pins.h"
#include <SPI.h>
#include "Vehicle.h"
#include <SD.h>
#include <RTClib.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include "Can_Protocol.h"
#include <stdio.h>
#include "Settings.h"

#include <Arduino.h>
#include "DBW_Pins.h"
#if DBWversion < 4
// Only for Arduino Mega
#include <mcp2515_can.h>     // CAN_BUS_shield library
//#include <PinChangeInterrupt.h>
#endif  // Mega

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
tmElements_t tm;

const int chipSelect  = 53; //chipSelect pin for the SD card Reader

RC_Controller* Vehicle::RC;
Brakes* Vehicle::brake;
ThrottleController* Vehicle::throttle;
SteeringController* Vehicle::steer;

#if DBWversion < 4
mcp2515_can CAN(CAN_SS_PIN);  // pin for CS on Mega
#endif


volatile int16_t Vehicle::desired_speed_mmPs;
volatile int16_t Vehicle::desired_brake;
volatile int16_t Vehicle::desired_angle;
bool eStopActive = false;
static bool estopState = false;
static unsigned long lastPulseTime = 0;

/****************************************************************************
   Constructor
 ****************************************************************************/
Vehicle::Vehicle() {
  
  Serial.println("Vehicle constructor starting...");
  
  RC = new RC_Controller();
  brake = new Brakes();
  throttle = new ThrottleController();
  steer = new SteeringController();
  // Intialize default values
  currentSpeed = 0;
  currentAngle = 0;
  currentBrake = 0;
  brakeHold = 0;
  desired_speed_mmPs = 0;
  desired_brake = 0;
  desired_angle = 0;
  currentDriveMode = 0;
// #if DBWversion < 4
//   // Keep trying to initialize CAN
//   while (0) { // changed to false For testing purposes CAN.begin(CAN_500KBPS) ReEnable for DBWV4
//     if (DEBUG) {
//       Serial.println("CAN BUS Shield init fail");
//     }
//     delay(1000);
//   }
//   if (DEBUG)
//     Serial.println("CAN BUS init ok!");

// #else  // Due
//   if (!Can0.begin(CAN_BPS_500K))  // initalize CAN with 500kbps baud rate
//   {
//     Serial.println("Can0 init success");
//   } else {
//     Serial.println("Can0 init failed");
//   }

// #endif  // DBWversion
  //attachPCINT(digitalPinToPCINT(IRPT_ESTOP_PIN), eStop, RISING);
  //attachPCINT(digitalPinToPCINT(IRPT_CAN_PIN), recieveCan, RISING);

  //initalize();
  Serial.println("Vehicle constructor finished.");
}

/*****************************************************************************
   Destructor
 ****************************************************************************/
Vehicle::~Vehicle() {
}
/*******************************
Initilze SD Card
******************/ 

void Vehicle::initalize() {
  Serial.println("initialize() starting");

  // ---------- Configure RTC with __DATE__ and __TIME__ ----------
  bool parse = false, config = false;

  if (getDate(__DATE__) && getTime(__TIME__)) {
    parse = true;
    if (RTC.write(tm)) {
      config = true;
    }
  }

  if (parse && config) {
    Serial.print("DS1307 configured Time=");
    Serial.print(__TIME__);
    Serial.print(", Date=");
    Serial.println(__DATE__);
  } else if (parse) {
    Serial.println("DS1307 Communication Error :-{");
    Serial.println("Please check your circuitry");
  } else {
    Serial.print("Could not parse compiler time, Time=\"");
    Serial.print(__TIME__);
    Serial.print("\", Date=\"");
    Serial.print(__DATE__);
    Serial.println("\"");
  }

  if (RTC.read(tm)) {
    Serial.print("Ok, Time = ");
    print2digits(tm.Hour); Serial.print(':');
    print2digits(tm.Minute); Serial.print(':');
    print2digits(tm.Second);
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(tm.Day); Serial.print('/');
    Serial.print(tm.Month); Serial.print('/');
    Serial.println(tmYearToCalendar(tm.Year));
  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped. Please run SetTime.");
    } else {
      Serial.println("DS1307 read error!");
    }
  }

  // Initialize SD Card
  Serial.print("Initializing SD card...");
  pinMode(chipSelect, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    logfile = File();  // mark logfile as invalid
    return;
  }
  Serial.println("Card initialized.");

  //Generate Unique Filename
  char filename[13];
  for (int i = 0; i < 100; i++) {
    sprintf(filename, "LOG%02d.CSV", i);
    if (!SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if (!logfile) {
    Serial.print("FAILED to open file: ");
    Serial.println(filename);
    error("file unable to open!");
    return;
  }

  Serial.print("Logging to: ");
  Serial.println(filename);

  //Write CSV Header 
  logfile.println("time_ms,desired_speed_ms,desired_brake,desired_angle,current_speed,current_brake,current_angle,throttle_pulse,steerpulse,brakeHold,steeringVal,steeringAngleRight,driveMode");
  Serial.println("initialize() finished");
}



/*****************************************************************************
   Struct for sending current speed and angle to high-level board through CAN
 ****************************************************************************/
typedef union {
  struct {
    uint16_t sspeed;
    uint16_t brake;
    uint16_t angle;
    uint16_t reserved;
  };
} speedAngleMessage;

/*******************************************************************************************************
   Checks for receipt of new CAN message and updates current
   Vehicle speed and steering angle by passing code received from CAN
   from RC or high-level board to the throttle and steering controllers
 *******************************************************************************************************/
void Vehicle::update() {
  //recieveCan();  //check for new message
  int16_t tempDspeed;
  int16_t tempDbrake;
  int16_t tempDangle;

  noInterrupts();
  tempDspeed = desired_speed_mmPs;
  tempDbrake = desired_brake;
  tempDangle = desired_angle;
  interrupts();

  //*******************************************************************************************
  //CHOOSE between the 2 below based on test or actually running the system

  //hard_Coded_Test(0, 24000, 0); //test only, no vehicle results

  Serial.print("Input Speed: "); Serial.println(tempDspeed);
  Serial.print("Input Brake: "); Serial.println(tempDbrake);
  Serial.print("Input Angle: "); Serial.println(tempDangle);
  real_System(tempDspeed, tempDbrake, tempDangle);  //real system when using bike, not test

  //*******************************************************************************************

  //If speed not less than zero, send current speed and angle to high-level for processing
  if (currentSpeed < 0)  //stopped
    return;

  //build struct to send data to Highlevel through CAN
  speedAngleMessage MSG;
  MSG.sspeed = currentSpeed;
  MSG.brake = currentBrake;
  MSG.angle = currentAngle;
  MSG.reserved = 0;

  // unknown status (120 - 145)
 #if DBWversion < 4
  CAN.MCP_CAN::sendMsgBuf(Actual_CANID, 0, 0, 8, (uint8_t*)&MSG);

  if (DEBUG) {
    if (CAN.MCP_CAN::sendMsgBuf(Actual_CANID, 0, 0,  8, (uint8_t*)&MSG)) {
      Serial.println("Sending Message to MEGA");
    } else {
      Serial.println("Message Failed");
    }
  }
// #else  // Due
//   outgoing.data.int16[0] = MSG.sspeed;
//   outgoing.data.int16[1] = MSG.brake;
//   outgoing.data.int16[2] = MSG.angle;
//   outgoing.data.int16[3] = MSG.reserved;
//   Can0.sendFrame(outgoing);

  if (DEBUG) {
    if (Can0.sendFrame(outgoing)) {
      Serial.println("Sending Message to DUE");
    } else {
      Serial.println("Message Failed");
    }
  }
#endif DBWversion
  // Update every second
  delay(1000);
}

/********************************************************************************************************
   Actual system for executing results with bike
 *******************************************************************************************************/
void Vehicle::real_System(int16_t tempDspeed, int16_t tempDbrake, int16_t tempDangle) {
  DriveMode mode = RC->getDriveMode();  // Get current mode from RC controller
  currentDriveMode = static_cast<int>(mode);

  currentSpeed = throttle->update(tempDspeed, mode);   // <- use '->'
  currentBrake = tempDbrake;
  currentAngle = steer->update(tempDangle);      // <- use '->'

  Serial.println("-----------------------------------------");
  Serial.println("Drive Mode: " + String(
    mode == REVERSE_MODE ? "REVERSE" :
    mode == NEUTRAL_MODE ? "NEUTRAL" :
                      "DRIVE"
  ));
  Serial.println("Actual Speed: " + String(currentSpeed));
  Serial.println("Actual Brake: " + String(currentBrake));
  Serial.println("Actual Angle: " + String(currentAngle));
}

/********************************************************************************************************
   Hard coded to give back feedback to high level as if
   the lowlevel had responded to the speed and angle requested by Highlevel board
 *******************************************************************************************************/
// void Vehicle::hard_Coded_Test(int16_t tempDspeed, int16_t tempDbrake, int16_t tempDangle) {
//   currentSpeed = tempDspeed;
//   currentBrake = tempDbrake;
//   currentAngle = tempDangle;
// }

/*************************************************************************************
   Checks for receipt of a message from CAN bus for new
   speed/angle/brake instructions from RC or high-level board
 ************************************************************************************/
// void Vehicle::recieveCan() {  //need to ADD ALL the other CAN IDs possible (RC instructions etc. 4-23-19)
//   noInterrupts();
//   unsigned char len = 0;
//   unsigned char buf[8];
//   unsigned int canId = 0;

// #if DBWversion < 4                           // CAN message receipt, if system is using Arduino Mega
//   if (CAN_MSGAVAIL == CAN.checkReceive()) {  //found new instructions
//     CAN.readMsgBuf(&len, buf);               // read data,  len: data length, buf: data buf
//     canId = CAN.getCanId();
//     interrupts();

//     if (canId == HiDrive_CANID) {  // the drive ID receive from high level
//       if (DEBUG) {
//         Serial.println("RECEIVED CAN MESSAGE FROM HIGH LEVEL WITH ID: " + String(canId, HEX));
//       }

//       // SPEED IN mm/s
//       uint16_t low_result = (buf[1] << 8) | buf[0];
//       desired_speed_mmPs = (int16_t)low_result;

//       // BRAKE ON/OFF
//       uint16_t mid_result = (buf[3] << 8) | buf[2];
//       desired_brake = (int16_t)mid_result;

//       if (desired_brake > 0 && brakeHold == 0) {  // Activate Brakes
//         eStop();
//       } else if (desired_brake > 0 && brakeHold == 1) {  // Hold Brakes
//         brake.Update();
//       } else {
//         brake.Release();  // Release Brakes
//         brakeHold = 0;
//       }

//       // WHEEL ANGLE
//       uint16_t high_result = (buf[5] << 8) | buf[4];
//       desired_angle = (int16_t)high_result;

//       if (DEBUG) {
//         Serial.print("CAN Speed: " + String(desired_speed_mmPs, DEC));
//         Serial.print(", CAN Brake: " + String(mid_result, DEC));
//         Serial.print(",  CAN Angle: ");
//         Serial.println(desired_angle, DEC);
//       }

//     } else if (canId == HiStatus_CANID) {  //High-level Status change (just e-stop for now 4/23/19)
//       desired_speed_mmPs = 0;
//       eStop();
//     }
//   }

// #else  // else CAN message receipt for system using Arduino Due
//   Can0.watchForRange(Actual_CANID, HiStatus_CANID);  //filter for high level communication
//   while (Can0.available() > 0) {                     // check if CAN message available
//     Can0.read(incoming);                             // reading data from CAN message
//     canId = incoming.id;
//   }
//   interrupts();
//   if (canId == HiDrive_CANID) {  // the drive ID receive from high level
//     if (DEBUG) {
//       Serial.println("RECEIVED CAN MESSAGE FROM HIGH LEVEL WITH ID: " + String(canId, HEX));
//     }

//     // SPEED IN mm/s
//     int16_t low_result = incoming.data.int16[0];
//     desired_speed_mmPs = low_result;


//     // BRAKE ON/OFF
//     int16_t mid_result = incoming.data.int16[1];
//     desired_brake = mid_result;
//     if (desired_brake > 0 && brakeHold == 0) {  // Activate Brakes
//       eStop();
//     } else if (desired_brake > 0 && brakeHold == 1) {  // Hold Brakes
//       brake.Update();
//     } else {  // Release Brakes
//       brake.Release();
//       brakeHold = 0;
//     }

//     // WHEEL ANGLE
//     int16_t high_result = incoming.data.int16[2];
//     desired_angle = high_result;

//     if (DEBUG) {
//       Serial.print("CAN Speed: " + String(low_result, DEC));
//       Serial.print(", CAN Brake: " + String(mid_result, DEC));
//       Serial.print(",  CAN Angle: ");
//       Serial.println(high_result, DEC);
//       Serial.println("mapped angle: " + String(desired_angle));
//     }
//   } else if (canId == HiStatus_CANID) {  //High-level Status change (just e-stop for now 4/23/19)
//     desired_speed_mmPs = 0;
//     eStop();
//   }

// #endif
// }

//Estop method for high or RC calls
void Vehicle::eStop() {
  if (DEBUG)
    Serial.println("E-Stop!");
  brake->Stop();      // <- use '->'
  throttle->stop();   // <- use '->'
  brakeHold = 1;

}
void Vehicle::applyEStop() {
  eStop(); // engages brakes + stops throttle
  brake->Update();
  throttle->stop(); // ensure DAC is zero
  desired_speed_mmPs = 0;
  desired_brake = 100;
}

void Vehicle::handleThrottleAndBrake() {
  throttlePulse_ms = RC->getMappedValue(RC_CH2_THROTTLE_BR);
  DriveMode mode = RC->getDriveMode();

  if (throttlePulse_ms == -1) {
    applyEStop();
  } else {
    brakeHold = 0;
    brake->Release();

    throttle->update(throttlePulse_ms, mode);
    desired_speed_mmPs = throttlePulse_ms;
    desired_brake = 0;
  }
}

void Vehicle::handleSteering() {
  steerPulse_ms = RC->getMappedValue(RC_CH1_STEERING);
  desired_angle = steerPulse_ms;
}

void Vehicle::updateRC() {
  RC->mapValues();
  RC->updateDriveSelection(); //updates drive mode

  if (RC->processEStop()) {
    applyEStop();
    RC->clearFlag();
    return;
  }

  handleThrottleAndBrake();
  handleSteering();
  RC->clearFlag();
}

void Vehicle::LogMonitor() {
  Serial.print(desired_speed_mmPs);  Serial.print(", ");
  Serial.print(desired_brake);  Serial.print(", ");
  Serial.print(desired_angle);  Serial.print(", ");
  Serial.print(currentSpeed);  Serial.print(", ");
  Serial.print(currentBrake);  Serial.print(", ");
  Serial.print(currentAngle);  Serial.print(", ");
  Serial.println(brakeHold); // Serial.print(", ");
 Serial.print(throttlePulse_ms);  Serial.print(", ");
  Serial.print(steerPulse_ms);  
}

void Vehicle::LogSD(){
 // Log data to the file
 if (!logfile) {
  Serial.println("ERROR: Log file not open!");
  return;
}
  logfile.print(millis());
  logfile.print(",");
  logfile.print(desired_speed_mmPs);
  logfile.print(",");
  logfile.print(desired_brake);
  logfile.print(",");
  logfile.print(desired_angle);
  logfile.print(",");
  logfile.print(currentSpeed);
  logfile.print(",");
  logfile.print(currentBrake);
  logfile.print(",");
  logfile.print(currentAngle);
  logfile.print(",");
  logfile.print(throttlePulse_ms);
  logfile.print(",");
  logfile.print(steerPulse_ms);
  logfile.print(",");
  logfile.print(brakeHold);
  logfile.print(",");
  logfile.print(steeringVal);
  logfile.print(",");
  logfile.print(currentRightAngle);
  logfile.print(",");
  logfile.print(currentDriveMode);
  logfile.println(); 
  logfile.flush();   // Flush the file to make sure data is written immediately
}
void Vehicle::error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  while(1);
}

void Vehicle::print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}


bool Vehicle::getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}


bool Vehicle::getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}
