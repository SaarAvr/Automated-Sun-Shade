Automated Sun Shade (Name to be improved upon)

A smart solution for creating a static shade at a desired place, using GPS location and time of day to calculate sun location, and a set of motors to move shade to correct position. Currently this project is for a small scale demonstration, with printed 3d parts. 

Project structure:

# Mobile App
An Android app created using .NET MAUI platform, its functions:
- Acquire GPS coordinates and current time from the phone utilities
- Connects to an ESP32 device over BLE
- Send the data packet

# ESP32 board
A simple communication unit between the App and the STM32 unit.
-Broadcast and connect by BLE to app
-Transfer data packet by UART to the STM32

# STM32 board 
Main controller unit, using an STM32 NUCLEO_F411RET to calculate and move motors
-Initialize motors position
-wait until recieving packet through UART Interrupt routine
-Calculates sun position using SolTrack libraries (https://sourceforge.net/projects/soltrack/)
-Calculates desired motor position
-Utilize 3 servos using generated PWM signals and a stepper motor to move structure to correct position


Future updates to include video demonstration