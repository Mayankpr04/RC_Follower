# BLE Beacon-Following Robot with Obstacle Avoidance

This project implements a robot that follows a BLE beacon using RSSI (Received Signal Strength Indicator) values while avoiding obstacles using an ultrasonic sensor. The system uses an ESP32 for signal processing and an Arduino UNO for control logic and actuation.

## Project Structure
```
├── esp32_ble_tracker.py # MicroPython script for BLE scanning and RSSI processing
├── outlier_mean_filtering.py # Python script for outlier removal and mean RSSI filtering
├── kalman_filtering.py # Python script for smoothing RSSI using a Kalman filter
├── main_control_loop.ino # Arduino sketch for motion control and obstacle avoidance
├── README.md # Project documentation
```
## Overview

- The BLE beacon broadcasts advertisement packets.
- The ESP32 scans for the beacon and records RSSI values.
- RSSI is filtered using either:
  - Outlier removal + mean averaging
  - Kalman filtering
- Filtered RSSI is sent to the Arduino via UART.
- The Arduino:
  - Reads RSSI and ultrasonic distance
  - Decides whether to move forward, rotate, or avoid obstacles
  - Sends commands to the L298N motor driver

## Hardware Used

- ESP32 feather V2
- Arduino UNO R3
- HC-SR04 Ultrasonic Sensor
- L298N Motor Driver
- 4 DC Motors and a chassis
- BLE Beacon or BLE-enabled smartphone

## Software Setup

- MicroPython running on ESP32
- Arduino IDE for control logic
- SoftwareSerial used for UART communication
- Python used for offline RSSI data analysis and filtering

## Filtering Approaches

- **Outlier + Mean Filtering**: Discards values too far from the average, then averages the remaining RSSI samples.
- **Kalman Filtering**: Applies probabilistic smoothing to reduce signal noise and predict the true RSSI trend.

## Results

- RSSI filtering improves signal stability and robot responsiveness.
- Obstacle detection prevents collisions and enables smoother path following.

## License

This project is licensed under the MIT License.
