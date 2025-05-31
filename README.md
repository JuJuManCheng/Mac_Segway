# Mac Segway Project

A self-balancing segway project using Teensy 4.0 microcontroller.

## Hardware Requirements

- Teensy 4.0
- MPU9250 IMU sensor
- 4 DC motors with encoders
- Motor driver board
- Power supply

## Pin Configuration

### Motor Control Pins
- Direction pins: 15, 14, 6, 28
- PWM pins: 25, 24, 9, 29
- Encoder pins: 30, 31, 33, 32

### IMU
- I2C interface (Wire)

## Features

- Motor control with PWM
- IMU data reading
- Encoder position tracking
- SBUS receiver support

## Building and Uploading

This project uses PlatformIO. To build and upload:

1. Install PlatformIO
2. Clone this repository
3. Open the project in PlatformIO
4. Build and upload to your Teensy 4.0

## Project Structure

- `src/main.cpp`: Main program
- `include/Motor.hpp`: Motor control class
- `include/MotorController.hpp`: Multi-motor controller class
- `include/imu_mpu9255_utility.hpp`: IMU utility functions 