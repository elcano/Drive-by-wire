# DriveByWire

This repository is for use with the **Drive by Wire** board created to be used for the ELCANO self-driving tricycle. In order for this module to run external libraries are required.

Repositories were updated 6/8/2023 but clobbered previous history. Chief addition is use of an RC controller that allows manual control of throttle, brakes and steering.
The Sender files are intended for use on the Hi-Level board and do not really belong in the Drive-by-wire repository.

Libraries needed are listed below:

* CAN_BUS_SHIELD by Seeed-Studio

    https://github.com/Seeed-Studio/CAN_BUS_Shield

    Contains CAN_BUS communication protocols and required files such as **mcp_can** and **can-serial**

* PID by Brett Beauregard

    https://github.com/br3ttb/Arduino-PID-Library

    A simple controller calculator (proportional integral derivative) to calculate the error between a desired output and an actual output. Has files **PID_v1**

* Pin Change Interrupt by Nico Hood

    https://github.com/NicoHood/PinChangeInterrupt

    For custom configuration of Arduino pin properties. Has files **PinChangeInterupt**

* MCP48x2 DAC by Jonas Forsberg

    https://github.com/SweBarre/MCP48x2.

    Library for Digital to anolog converter. Has files **MCP48x2**

* Arduino Due Timer Interrupts by Ivan Seidel

    https://github.com/ivanseidel/DueTimer

    Library for setting the frequency of Arduino updates. Has files **DueTimer**
    
* Due CAN and CAN Common by Collin Kidder

    https://github.com/collin80/due_can
    
    https://github.com/collin80/can_common


## To Install

* Download the **Drive_By_Wire** module into the project parent directory
```
git clone https://github.com/elcano/Drive-by-wire.git
```
* Move into the **Drive_By_Wire** folder
```
cd Drive_By_Wire
```
* Download the required libraries
```
git clone https://github.com/Seeed-Studio/Seeed_Arduino_CAN.git
git clone https://github.com/br3ttb/Arduino-PID-Library.git
git clone https://github.com/NicoHood/PinChangeInterrupt.git
git clone https://github.com/SweBarre/MCP48x2.git
git clone https://github.com/ivanseidel/DueTimer.git
git clone https://github.com/collin80/can_common.git
git clone https://github.com/collin80/due_can.git
```
* Add the folders above to the IDE directory. In Windows, add the required libraries to: C:\Users\<username>\Documents\Arduino\libraries
* Arduino Due requires additional installation. On the IDE, go to Tools -> Boards -> Boards Manager -> Arduino SAM Boards (32-bits ARM Cortex-M3) 

## Navigating Arduino IDE: 
* **Selecting Board:** Tools -> Board -> Arduino AVR/SAM Boards -> Arduino Mega/Due
* **Selecting Serial Port:** Tools -> Port -> COM (Arduino Mega/Due)
* **Accessing Serial Monitor:** Right-hand Corner -> Select Baud Rate (115200)

## Modification:
* Vehicle.cpp/Vehicle.h: 
  - Constructor of CAN Object is type ```mcp2515_can``` not ```MCP_CAN```
  - Include ```mcp2515_can.h```
  - sendMsgBuf needs to reference MCP_CAN ```CAN.MCP_CAN::sendMsgBuf```
