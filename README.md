# 🚗 Obstacle Avoiding Car using Arduino Uno + L293D Motor Shield

This project builds an **autonomous obstacle avoiding car** using:
- Arduino Uno
- L293D Motor Shield
- Ultrasonic Sensor (HC-SR04)
- Servo Motor (SG90)
- LEDs for indicators
- DC Motors

The car detects obstacles, scans left/right using a servo, and avoids collisions by turning in the better direction. LEDs show movement direction.

---

## ✨ Features
- Automatic obstacle detection
- Servo-based scanning
- LED indicators for turns
- Smooth motor control with L293D Shield
- Compact & beginner-friendly

---

## 🔧 Components Required
- Arduino Uno
- L293D Motor Shield
- Ultrasonic Sensor (HC-SR04)
- Servo Motor (SG90)
- 2x DC Motors + Wheels
- LEDs + Resistors
- Battery pack

---

## ⚡ Wiring (text only)
- Ultrasonic → VCC=5V, GND=GND, TRIG=Pin 9, ECHO=Pin 8
- Servo → VCC=5V, GND=GND, Signal=Pin 10
- LED Left → Pin 4
- LED Right → Pin 5
- Motors → Motor A & Motor B terminals on shield
- Power → External battery pack to motor shield

---

## ▶️ How to Run
1. Upload `ObstacleAvoidingCar.ino` to Arduino Uno.
2. Place the car on a flat surface.
3. Power it with an external battery pack.
4. Watch it avoid obstacles automatically!

---

## 🎥 Demo Video
▶️ [Watch on YouTube](https://youtube.com/shorts/5Ac8ldPXbF8)

---

## 📜 License
This project is licensed under the MIT License.  
© 2025 Wired Wanderer
