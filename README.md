Gesture-Controlled Robot

Project Summary:

Gesture-Controlled Robot is a project where hand movement is used to drive the way a robot will move. The project is constructed using an Arduino Uno, an MPU-6050 motion sensor, an L298N motor driver, and a DC motor. The robot uses tilt movements to cause it to move forward, backward, left, right or in a circle. The project demonstrates my C programming skills, sensor use, and hardware connection skills, which I learned in preparation for hardware internships.

Features:

Gesture Control: MPU-6050 sensor tilts (worn on hand) determines the direction of the robot.
Circular Tilt Control: Smooth circular movements achieved by mapping MPU-6050 tilt angles to differential motor speeds.
Real-Time Motion: Quick response to movements with accurate motor control using L298N.
Modular Code: C code with easy-to-use functions for sensor reading and motor control.
I2C Communication: It communicates with the MPU-6050 using the I2C protocol.

Parts:

Arduino Uno: A miniature computer that interprets sensor information and manages motors.
MPU-6050: 6-axis gyroscope/accelerometer to recognize hand tilt movements.
L298N Motor Driver: Drives two DC motors for robot mobility.
DC Motors (2): Power the robot wheels.
Battery (9V): Powers the Arduino and motors.
Jumper Wires: Male-female and normal single stranded for connections.
Mount board: An unassuming robot body with wheels.

Pre-requisites:

Software:
Arduino IDE (version 2.0 or later)
Libraries: Wire.h (I2C), Adafruit_MPU6050 (or a similar MPU-6050 library)

Hardware: Found in Components
Materials required:
Tools: Screwdriver and calm mind(optional)

Installation and Configuration:

I installed the latest version of the Arduino IDE, libraries necessary for functioning of MPU6050 accelerometer/gyrometer 
and sensor.

Hardware Assembly
Connect MPU-6050 to Arduino
VCC to 5V
GND → GND
SCL is A5.
SDA → A4

Connect L298N to Arduino:
IN1 → Pin 9
IN2 → Pin 8
IN3 → Pin 7
IN4 → Pin 6
ENA, ENB → 5V (or speed control PWM pins)
L298N VCC → 9V battery, GND → common GND

Recognize L298N OUT1/OUT2 and OUT3/OUT4 DC motors.
Install the components on the mount board and lock connections.

Software Installation:

Download the Arduino IDE from arduino.cc.
Install Adafruit_MPU6050 library using Arduino Library Manager.
Clone or download this repository.


Upload Code:

Open gesture_robot.ino in Arduino IDE.
Connect Arduino Uno through USB.
Choose port and board (Arduino Uno) in IDE.
Upload code to Arduino.

Usage:

Power the robot with the 9V battery.
Place the MPU-6050 on a flat surface.
Tilt the flat surface on which the MPU6050 is placed to control movement:
Forward tilt: Robot moves forward.
Backward tilt: Robot moves backward.
Left tilt: Robot turns left.
Right tilt: Robot turns right.
Circular tilt : Robot moves in smooth arcs or circles by adjusting motor speeds differentially.
Monitor Serial output (9600 baud) for debugging sensor data.
Utilize Serial output (9600 baud) for debugging sensor information.

Code Structure

gesture_robot.ino:
Setup: Initializes MPU-6050, I2C, and motor connections.
Loop: Reads MPU-6050 tilt angles, scales to motor commands, and drives L298N. Functions: Modular sensor reading code, angle calculation code, and motor control code.


Utilizes pointers and arrays to manage data effectively.

Problems and Solutions

MPU-6050 Calibration: I resolved the "Failed to find MPU6050 chip" issue by verifying the I2C wiring and employing an I2C scanner.

Motor Synchronization: PWM signals were tuned for smooth turns.

Power Management: Employed a 9V battery with shared GND to stabilize power.

Future Improvements

Apply ESP32 to realize remote gesture wireless control.

Employ AI to identify gestures using AIML techniques. Add sensors, like ultrasonic ones, to help avoid obstacles.
You can contribute forking this repository, raising bugs or propose more efficient concepts/codes by sending pull requests. 


Acknowledgments: I learned to build this with the help of Grok and Gemini. Thanks to Adafruit for the MPU-6050 library and documentation. This project is open-source and available for educational use.

Contact Shreyas Mutha at:
Linkedin: https://www.linkedin.com/in/shreyas-mutha-5b092b2a7/
Email: shreyasmutha82006@gmail.com

Also please suggest some newer and cooler projects!
