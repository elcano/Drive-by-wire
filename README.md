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

## To Install

* Download the **Drive_By_Wire** module into the project parent directory
```
$ git clone https://github.com/elcano/Drive-by-wire.git
```
* Move into the **Drive_By_Wire** folder
```
$ cd Drive_By_Wire
```
* Download the required libraries
```
$ git clone https://github.com/Seeed-Studio/Seeed_Arduino_CAN.git
$ git clone https://github.com/br3ttb/Arduino-PID-Library.git
$ git clone https://github.com/NicoHood/PinChangeInterrupt.git
$ git clone https://github.com/SweBarre/MCP48x2.git
$ git clone https://github.com/ivanseidel/DueTimer.git
```
* Add the folders to your IDE for the project and thats it.
