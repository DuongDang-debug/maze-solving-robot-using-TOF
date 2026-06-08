# 🧩 Maze-Solving Robot using TOF Sensors

Source code and algorithms for an autonomous maze-solving robot (Micromouse style) utilizing Time-of-Flight (TOF) distance sensors for precise navigation and obstacle avoidance.

## 🛠️ Hardware Components
* **Microcontroller:** ESP32 (or Arduino depending on core architecture).
* **Sensors:** VL53L0X TOF (Time-of-Flight) laser distance sensors (Front, Left, Right) for precise wall distance measurement.
* **Motors:** N20 micro DC gear motors with encoders for precise speed control and odometry.
* **Motor Driver:** TB6612 motor driver IC.

## 🧠 Navigation & Algorithms
* **Flood-Fill Algorithm:** Logic for maze mapping, pathfinding, and solving.
* **PID Control:** Closed-loop feedback system using TOF sensor data and motor encoders to keep the robot centered in corridors and perform accurate 90/180-degree turns.

## 📸 Media & Schematics
<img width="1020" height="1020" alt="image" src="https://github.com/user-attachments/assets/ca550274-ad6b-4a1f-bcb9-a050de9837c1" />

## ⚙️ Setup & Configuration
1. Open the project in your preferred IDE (Arduino IDE / PlatformIO).
2. Install necessary libraries for the **VL53L0X** sensors.
3. Fine-tune the core parameters in the configuration file:
   * PID constants (`Kp`, `Ki`, `Kd`).
   * Target wall distance thresholds.
   * Turn angles and base motor speeds.
4. Upload the firmware to the robot and calibrate the sensors.
