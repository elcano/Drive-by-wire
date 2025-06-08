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
#include "DriveMode.h" // ADDED: Required for DriveMode type

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
// Changed: eStopActive is now a member of Vehicle, so remove global declaration.
// bool eStopActive = false; // REMOVED: This is now a member of Vehicle.

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
  // ADDED: Initialize new member variables
  currentDriveMode = NEUTRAL_MODE; // Default initialization
  eStopActive = false;             // Default initialization
  // END ADDED

// #if DBWversion < 4
//    // Keep trying to initialize CAN
//    while (0) { // changed to false For testing purposes CAN.begin(CAN_500KBPS) ReEnable for DBWV4
//      if (DEBUG) {
//        Serial.println("CAN BUS Shield init fail");
//      }
//      delay(1000);
//    }
//    if (DEBUG)
//      Serial.println("CAN BUS init ok!");

// #else   // Due
//    if (!Can0.begin(CAN_BPS_500K))   // initalize CAN with 500kbps baud rate
//    {
//      Serial.println("Can0 init success");
//    } else {
//      Serial.println("Can0 init failed");
//    }

// #endif   // DBWversion
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

void Vehicle::initalize(){
  Serial.println("initialize() starting");
  bool parse=false;
  bool config=false;

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
    Serial.print("Could not parse info from the compiler, Time=\"");
    Serial.print(__TIME__);
    Serial.print("\", Date=\"");
    Serial.print(__DATE__);
    Serial.println("\"");
  }

  if (RTC.read(tm)) {
    Serial.print("Ok, Time = ");
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.println();
  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped. Please run SetTime.");
    } else {
      Serial.println("DS1307 read error!");
    }
  }


  Serial.print("Initializing SD card...");

  pinMode(53, OUTPUT);

  if (!SD.begin(53)) {
    Serial.println("Card failed, or not present");
    logfile = File(); // make logfile invalid
    return;
  }
  Serial.println("Card initialized.");

  // Setup log file
  char filename[] = "MM_DD_00.CSV";

  filename[0] = '0';
  filename[1] = '1';
  filename[3] = '0';
  filename[4] = '1';

  int i = 1;
  do {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    i++;
  } while (SD.exists(filename) && (i < 100));

  logfile = SD.open(filename, FILE_WRITE);

  if (!logfile) {
    error("file unable to open!");
  }

  Serial.print("Logging to: ");
  Serial.println(filename);

  // --- MODIFIED: Added new column headers ---
  logfile.println("time_ms,desired_speed_ms,desired_brake,desired_angle,current_speed,current_brake,current_angle,throttle_pulse,steerpulse,brakeHold,steeringVal,steeringAngleRight,actual_throttle_pwm,drive_mode,steering_mode,estop_active");
  // --- END MODIFIED ---
  logfile.flush();

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
  //recieveCan();   //check for new message
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
  real_System(tempDspeed, tempDbrake, tempDangle);   //real system when using bike, not test

  // --- MODIFIED: Call LogSD() here ---
  LogSD();
  // --- END MODIFIED ---

  //*******************************************************************************************

  //If speed not less than zero, send current speed and angle to high-level for processing
  if (currentSpeed < 0)   //stopped
    return;

  //build struct to send data to Highlevel through CAN
  speedAngleMessage MSG;
  MSG.sspeed = currentSpeed;
  MSG.brake = currentBrake;
  MSG.angle = currentAngle;
  MSG.reserved = 0;

 #if DBWversion < 4
  CAN.MCP_CAN::sendMsgBuf(Actual_CANID, 0, 0, 8, (uint8_t*)&MSG);

  if (DEBUG) {
    if (CAN.MCP_CAN::sendMsgBuf(Actual_CANID, 0, 0,  8, (uint8_t*)&MSG)) {
      Serial.println("Sending Message to MEGA");
    } else {
      Serial.println("Message Failed");
    }
  }
// #else   // Due
//   outgoing.data.int16[0] = MSG.sspeed;
//   outgoing.data.int16[1] = MSG.brake;
//   outgoing.data.int16[2] = MSG.angle;
//   outgoing.data.int16[3] = MSG.reserved;
//   Can0.sendFrame(outgoing);

  if (DEBUG) {
    // Original code had a Can0.sendFrame(outgoing) call here within an #if DBWversion < 4 block.
    // This is syntactically incorrect for DBWversion < 4 (Mega).
    // I'm commenting it out as it's not part of the requested logging changes and
    // would likely cause a compilation error.
    // if (Can0.sendFrame(outgoing)) {
    //   Serial.println("Sending Message to DUE");
    // } else {
    //   Serial.println("Message Failed");
    // }
  }
#endif // DBWversion
  // Update every second
  delay(1000);
}

/********************************************************************************************************
    Actual system for executing results with bike
*******************************************************************************************************/
void Vehicle::real_System(int16_t tempDspeed, int16_t tempDbrake, int16_t tempDangle) {
  DriveMode mode = RC->getDriveMode();  // Get current mode from RC controller

  currentSpeed = throttle->update(tempDspeed, mode);   // <- use '->'
  currentBrake = tempDbrake;
  currentAngle = steer->update(tempDangle);      // <- use '->'

  // ADDED: Store drive mode for logging
  currentDriveMode = mode;
  // END ADDED

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

// --- Existing hard_Coded_Test and recieveCan functions ... (no changes here) ---

//Estop method for high or RC calls
void Vehicle::eStop() {
  if (DEBUG)
    Serial.println("E-Stop!");
  //noInterrupts();
  brake->Stop();       // <- use '->'
  throttle->stop();   // <- use '->'
  brakeHold = 1;
  //interrupts();
}

void Vehicle::updateRC() {
  RC->mapValues();
  RC->update(); //updates drive mode
  DriveMode mode = RC->getDriveMode();
  throttlePulse_ms = RC->getMappedValue(RC_CH2_THROTTLE_BR);
  steerPulse_ms = RC->getMappedValue(RC_CH1_STEERING);

  static int32_t prevPWM = 0;

  static bool estopState = false;
  static unsigned long lastPulseTime = 0;

  if (RC_Controller::estopFlagChanged) {
    RC_Controller::estopFlagChanged = false;

    Serial.print("E-STOP Pulse: ");
    Serial.println(RC_Controller::estopPulseWidth);

    unsigned long now = millis();
    if (now - lastPulseTime > 100) {   // simple debounce
      lastPulseTime = now;

      // Directly set state based on pulse width
      if (RC_Controller::estopPulseWidth > 1800) {
        estopState = true;
        Serial.println(">>> E-STOP ON <<<");
      } else if (RC_Controller::estopPulseWidth < 1200) {
        estopState = false;
        Serial.println(">>> E-STOP OFF <<<");
      }
    }
  }

  // ADDED: Update eStopActive member variable for logging
  eStopActive = estopState;
  // END ADDED

  if (estopState) {
    eStop(); // engages brakes + stops throttle
    throttle->stop(); // <-- make sure throttle is set to 0 volts
    desired_speed_mmPs = 0;
    desired_brake = 100;
    RC->clearFlag();
    return;
  }

    if (throttlePulse_ms == -1) {
      eStop();

      desired_speed_mmPs = 0;
      desired_brake = 100;
    } else {
      brakeHold = 0;
      brake->Release();


      throttle->update(throttlePulse_ms, mode);
      desired_speed_mmPs = throttlePulse_ms;
      desired_brake = 0;
    }

    desired_angle = steerPulse_ms;
    RC->clearFlag();
}

void Vehicle::LogMonitor() {
  Serial.print(desired_speed_mmPs);   Serial.print(", ");
  Serial.print(desired_brake);   Serial.print(", ");
  Serial.print(desired_angle);   Serial.print(", ");
  Serial.print(currentSpeed);   Serial.print(", ");
  Serial.print(currentBrake);   Serial.print(", ");
  Serial.print(currentAngle);   Serial.print(", ");
  Serial.println(brakeHold); // Serial.print(", ");
  Serial.print(throttlePulse_ms);   Serial.print(", ");
  Serial.print(steerPulse_ms);   
}

void Vehicle::LogSD(){
  // Log data to the file
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
  logfile.println(currentRightAngle);
  // --- ADDED: New data points ---
  logfile.print(",");
  // NOTE: You'll need to ensure ThrottleController has a public getCurrentThrottlePWM() method
  // and SteeringController has a public getSteeringMode() method.
  // I'm assuming those methods exist for compilation based on previous discussions.
  logfile.print(throttle->getCurrentThrottlePWM());
  logfile.print(",");
  logfile.print(currentDriveMode); // Log the integer value of the DriveMode enum
  logfile.print(",");
  logfile.print(steer->getSteeringMode());
  logfile.print(",");
  logfile.println(eStopActive ? 1 : 0); // Log eStopActive as 1 (true) or 0 (false)
  // --- END ADDED ---
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