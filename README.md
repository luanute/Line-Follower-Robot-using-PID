# ESP32 Bluetooth Line Follower Robot Controlled via Dabble App

This project implements a line follower robot using an ESP32 microcontroller, controlled wirelessly via the Dabble app over Bluetooth. The robot uses QTR-8RC sensors to detect white lines on a dark surface, PID control for precise navigation, and supports servo and relay control through the gamepad interface in the app.

---

## Key Features

- Line following using QTR-8RC array (8 sensors).  
- Control of two DC motors with adjustable speed and direction.  
- Control of three servos for additional mechanical functions (grip, release, shoot).  
- Relay control for switching external devices on/off.  
- Bluetooth communication with Dabble app for wireless control.  
- Two driving modes: manual control via gamepad and autonomous line following using PID.  
- Responsive controls for speed, servos, relay, and driving mode via app buttons.

---

## Hardware Components

- ESP32 microcontroller  
- QTR-8RC reflectance sensor array for line detection  
- Two DC motors with BTS7960 drivers  
- Three servo motors for mechanical actuation  
- Relay module for external switching  
- Status LED indicator  
- Bluetooth connection to smartphone via Dabble app

---

## Installation and Usage Instructions

1. Connect hardware components according to the pin assignments defined in the source code (ESP32 pins, sensors, motors, servos, relay, LED).  
2. Install the required Arduino libraries: DabbleESP32, ESP32Servo, QTRSensors.  
3. Upload the provided source code to the ESP32 board using Arduino IDE.  
4. Open the Dabble app on your smartphone and connect to the ESP32 Bluetooth device (SSID: "wavextensa").  
5. Use the on-screen gamepad controls to operate the robot:  
   - Select button: cycle through speed settings  
   - Start button: toggle driving modes (manual/autonomous)  
   - Square, Triangle, Circle buttons: toggle servo motors 1, 2, and 3 respectively  
   - Cross button: toggle the relay on/off  
   - Directional pad: manual movement control (forward, backward, turn left/right)

---

## Code Overview

- `setup()`: Initializes GPIO pins, attaches servo motors, calibrates QTR sensors, and establishes Bluetooth connection via Dabble.  
- `loop()`: Continuously reads input from the Dabble gamepad, manages servo and relay states, handles driving mode switching, and calls line following PID control if autonomous mode is active.  
- `speed_run(leftSpeed, rightSpeed)`: Controls the speed and direction of the two DC motors through PWM signals.  
- `pid()`: Implements the PID algorithm to adjust motor speeds based on the line position error detected by the sensors, keeping the robot following the line accurately.  
- `robot_control()`: Reads sensor values, computes position error, and invokes PID controller.  
- Global variables store servo states, speed levels, PID parameters, and sensor readings for continuous feedback and control.

---

## Contact

For questions or feedback, feel free to reach out via email: **nguyenkluan957@gmail.com**

