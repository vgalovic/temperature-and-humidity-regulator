# Environmental Regulator Controller

## Overview

This project implements an embedded environmental regulator system using an AVR microcontroller. It monitors temperature and humidity using a DHT22 sensor and controls a DC fan (via PWM) and a buzzer based on user-defined thresholds.

The system features a keypad interface for user interaction, an I2c LCD for status display, and serial debugging capabilities.

---

## Features

- **Temperature and humidity monitoring** with DHT22 sensor
- **Fan speed control** proportional to temperature using PWM
- **Buzzer alert** when humidity exceeds configured thresholds
- **Keypad interface** for adjusting system parameters (humidity and temperature limits)
- **16x2 I2C LCD display** showing sensor data and system status
- **Debug mode** with serial output for sensor diagnostics
- **Test modes** for fan and buzzer functionality

---

## Hardware Requirements

- AVR microcontroller (e.g., ATmega328P)
- DHT22 temperature and humidity sensor
- DC fan connected to PWM-capable pin
- Buzzer connected to digital output pin
- 16x2 LCD with I2C interface
- 4x2 matrix keypad (2 rows, 4 columns)
- Appropriate pull-up resistors and wiring as per schematic

---

## Pin Configuration

| Peripheral   | Pin Number         | Description                     |
| ------------ | ------------------ | ------------------------------- |
| DHT22        | 12                 | Sensor data pin (digital)       |
| Fan PWM      | 10                 | PWM output to control fan speed |
| Buzzer       | 11                 | Digital output for buzzer       |
| Keypad Row 1 | 9                  | Keypad row 1                    |
| Keypad Row 2 | 8                  | Keypad row 2                    |
| Keypad Col 1 | 5                  | Keypad column 1                 |
| Keypad Col 2 | 4                  | Keypad column 2                 |
| Keypad Col 3 | 3                  | Keypad column 3                 |
| Keypad Col 4 | 2                  | Keypad column 4                 |
| LCD I2C      | 0x27 (I2C address) | I2C interface for LCD           |

---

## Software Components

- `main.c`: Main application controlling flow, display, keypad, and hardware
- `main.h`: Definitions of pins, constants, enums, and function prototypes
- `dht22.c/h`: Driver for DHT22 sensor readings
- `keypad.c/h`: Keypad scanning and debounce handling
- `lcd.c/h`: LCD driver for I2C 16x2 display
- `usart.c/h`: USART serial communication for debugging
- `tasks.c/h`: Task scheduler for periodic actions
- `timebase.c/h`: Timing utilities for delays and intervals
- `pin.c/h`: Pin abstraction utilities
- `strfmt.c/h`: String formatting helpers

---

## Usage

1. **Initialization**  
   Call `init()` to initialize hardware peripherals and sensors.

2. **Setup**  
   Call `setup()` to configure pins and display startup messages.

3. **Main Loop**  
   Continuously call `tasksRun()` within the `loop()` function to handle scheduled actions like sensor updates, keypad scanning, and display refresh.

4. **User Interaction**

   - Use the keypad to switch between display modes, test modes, and debug mode.
   - Press keys '6' to enter parameter adjustment mode and use keypad to increase/decrease thresholds for humidity and temperature.
   - Parameters will affect when the buzzer sounds or the fan speed changes.

5. **Debugging**  
   Enable debug mode to send detailed sensor diagnostics via USART at 9600 baud.

---

## Keypad Controls

| Key | Action                                      |
| --- | ------------------------------------------- |
| 1   | Switch to debug mode, display temp only     |
| 2   | Switch to debug mode, display humidity only |
| 3   | Toggle fan test mode                        |
| 4   | Toggle buzzer test mode                     |
| 5   | Toggle debug output enable                  |
| 6   | Enter threshold parameter adjustment mode   |
| 8   | Clear LCD display or exit parameter mode    |

---

## Threshold Adjustment Keys (in parameter mode)

| Key | Action                         |
| --- | ------------------------------ |
| 1   | Increase minimum humidity      |
| 5   | Decrease minimum humidity      |
| 2   | Increase maximum humidity      |
| 6   | Decrease maximum humidity      |
| 3   | Increase minimum temperature   |
| 7   | Decrease minimum temperature   |
| 4   | Reset parameters to default    |
| 8   | Exit parameter adjustment mode |

---

## Build and Flash Instructions with CMake

This project uses **CMake** to generate build files for AVR microcontrollers.

### Prerequisites

- [CMake](https://cmake.org/download/)
- AVR GCC toolchain (`avr-gcc`, `avr-objcopy`, `avrdude`, etc.)
- AVR programmer (e.g., USBasp, AVRISP mkII)

### Build steps

```bash
mkdir build
cd build
cmake ..
make
make program
```
