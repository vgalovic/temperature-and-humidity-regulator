# Code Bases Overview

This project includes **two implementations** of the same environmental regulator controller functionality:

1. **Arduino IDE Version**

   - Developed using the Arduino IDE environment
   - Leverages readily available Arduino libraries installed via the IDE’s library manager
   - Simplifies hardware abstraction and coding using Arduino API calls

2. **Bare-Metal AVR Version**

   - Developed from scratch using direct AVR register programming
   - Does **not** rely on Arduino libraries or APIs
   - Provides low-level control over microcontroller peripherals

Both code bases perform the same tasks:

- Reading temperature and humidity from a DHT22 sensor
- Controlling a PWM fan speed based on temperature
- Activating a buzzer when humidity exceeds thresholds
- Interacting with a keypad for user input
- Displaying status on an I2C LCD
- Supporting debug mode via serial communication
