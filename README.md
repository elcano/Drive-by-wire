# DriveByWire

This repository is for use with the **Drive by Wire** board created to be used for the ELCANO self-driving tricycle. In order for this module to run external libraries are required.

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


## Drive-by-Wire: 
* **Low-level board code with RC control support with existing code for brakes, steering, and throttle.** 

## Test: 
* **Low-level board code with three functions to test each subsystems individually (brakes, steering, throttle).** 
* testBrake(): Activates brakes with 24V via Stop(), Holds brakes with 12V after 800ms via Update(), Releases brakes every ~2 seconds.
* testThrottle(): Gradually increases speed and decreases speed on DACA.
* testSteering(): Changes directions every 3 seconds, moving the linear actuator. 

## Sender: 
* **High-level board code to communicate with low-level board using CAN communication.** 
* Arguments for Function: Each data is reserved for 2 bytes. 
    *   currentSpeed: Controls the speed via the DACA, input: (0 -255).
    *   currentBrake: Apply and updates brakes if necessary, input: (0 - releases brakes, anything above 0 - applies brakes).
    *   currentAngle: Changes direction based on input (may need to calibrate based on sensors), left: 779 (1ms), middle: 722 (1.5ms), right: 639 (1.85ms).
