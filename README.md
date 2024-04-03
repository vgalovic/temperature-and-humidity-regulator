# Temperature Regulator System

## Description:

This Arduino code implements a Temperature Regulator System using various components including a keypad, LCD display (controlled via I2C), DHT22 sensor for temperature and humidity, a DC fan, and a buzzer. The system allows users to monitor temperature and humidity levels, control the fan speed based on temperature, and activate the buzzer based on humidity thresholds. Users can also set parameter values through the keypad interface and monitor the system's status on the LCD.

---

## Libraries Used
- **Keypad**: Used for interfacing with the keypad (https://github.com/Chris--A/Keypad.git).
- **LiquidCrystal_I2C**: Controls the LCD via I2C communication (https://downloads.arduino.cc/libraries/github.com/marcoschwartz/LiquidCrystal_I2C-1.1.2.zip).
- **DHT22**: Enables reading data from the DHT22 sensor (https://github.com/dvarrel/DHT22.git).

---

## Pin Definitions and Component Initialization

### Keypad Pins
- Row Pins: R1 (9), R2 (8)
- Column Pins: C1 (5), C2 (4), C3 (3), C4 (2)

### DHT22 Sensor Pin
- DHT Pin: DHT (12)

### LCD Initialization
- **I2C Address of LCD**: The LCD is initialized with the I2C address `0x27`.

---

## Constants and Parameters
- **TIME**: Delay time for retrieving information from DHT22 sensor.
- **D_HUMI_MIN**: Default value used for minimum humidity threshold.
- **D_HUMI_MAX**: Default value used for maximum humidity threshold.
- **D_TEMP_MIN**: Default value used for minimum temperature threshold.
- **HUMI_MIN_LOWER_LIMIT**, **HUMI_MIN_UPPER_LIMIT**: Lower and upper limits for minimum humidity threshold.
- **HUMI_MAX_LOWER_LIMIT**, **HUMI_MAX_UPPER_LIMIT**: Lower and upper limits for maximum humidity threshold.
- **TEMP_MIN_LOWER_LIMIT**, **TEMP_MIN_UPPER_LIMIT**: Lower and upper limits for minimum temperature threshold.
- **HUMI_STEP**: Steps for adjusting humidity thresholds.
- **TEMP_STEP**: Step for adjusting temperature threshold.

---

## Global Variables

- **en_mil**: Variable to check whether the time has passed for retrieving information from DHT22.
- **display_state**: Chooses the LCD's display mode between the fan speed and buzzer operation and the temperature and humidity displays.
- **debug_enable**: Choose whether or not to display the output of dht.debug() on the serial monitor.
- **action_state**: Ascertains the operational status of action().
- **humi_min**: Set lower limit for humidity triggering buzzer.
- **humi_max**: Set upper limit for humidity triggering buzzer.
- **temp_min**: Set lower limit for temperature for fan speed control.

---

## Functions
- **buzzControle(uint8_t humi)**: Controls the buzzer based on humidity level.
- **dcFanControle(uint8_t temp)**: Controls the DC fan based on temperature.
- **display()**: Displays temperature, humidity, fan speed, and buzzer status on the LCD.
- **debugMode()**: Enters a debugging mode, printing DHT22 debug information to the serial monitor.
- **action()**: Controls different actions based on `action_state`.
- **keypadEvent(char key)**: Responds to keypad inputs, triggering different actions.
- **set_parameters()**: Sets parameter values using keypad input and displays them on the LCD.

---

## Setup Function
- Initializes components, displays initial messages on the LCD.

## Loop Function
- Checks for keypad input and performs actions accordingly.
- Triggers actions based on time intervals.

---

*Note: This Markdown file provides a detailed overview of the Arduino code and its functionalities.*

