# Multi-Sensor Controlled Intelligent Robot (Arduino)

This project implements an autonomous robotic system using an Arduino Uno. It integrates multiple sensors and actuators for environment-aware movement and obstacle avoidance.

## 🧠 Features

- **IR Sensors (Left, Mid, Right)** for line-following or surface detection.
- **Ultrasonic Sensor** for front obstacle distance measurement.
- **Servo Motor** to rotate the ultrasonic sensor or perform scanning.
- **Dual DC Motors** with directional and speed control.
- **Command-based serial control** for different modes or behaviors.

## 🔧 Hardware Components

- Arduino Uno
- 3x IR Sensors
- 1x Ultrasonic Sensor (HC-SR04)
- 1x Servo Motor (SG90 or compatible)
- 2x DC Motors with L298N or equivalent motor driver
- Breadboard, jumper wires, power supply

## 📂 Files

- `MCI_project.ino` — Main Arduino sketch with logic for movement, scanning, and control.

## 🚀 Getting Started

1. Wire all components according to the pin definitions in the code.
2. Upload `MCI_project.ino` to the Arduino Uno using the Arduino IDE.
3. Open Serial Monitor (9600 baud) to send commands and observe behavior.

## ⚠️ Notes

- Ensure sufficient power supply for motors.
- Some constants may need tuning based on your sensor or motor models.
- Servo initialization starts at 90°, can be adjusted per application.

## 🏷️ Tags

`Arduino` `Robotics` `IR Sensors` `Ultrasonic` `Servo Motor` `Autonomous Robot` `Obstacle Avoidance`
