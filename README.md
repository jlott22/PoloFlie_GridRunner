# crazyflie_pololu_integration
Develop a central hub and Pololu robot code using MQTT and Crazyflie PA to coordinate drones and ground robots in navigating to grid waypoints. 

# Multi-Agent Grid Navigation with MQTT and Crazyflie PA

## Overview
This project implements a coordinated control system for **Crazyflie drones** and **Pololu 3pi+ 2040 OLED robots** to navigate to waypoints on a defined grid. A central **hub script** communicates with all agents using **MQTT** and leverages **Crazyflie Positioning Anchor (PA)** for precise drone localization.  
Pololu robots communicate via **ESP32 bridges** that forward MQTT messages to the robot’s onboard controller, enabling synchronized movement with the drones.

---

## Features
- **Centralized Control**: Single hub script manages both drones and ground robots.
- **MQTT Communication**: Lightweight, real-time messaging between hub, drones, and robots.
- **Waypoint Navigation**: Send drones and robots to grid-based coordinates.
- **Crazyflie PA Integration**: Enables accurate indoor positioning for drones.
- **Two-Way Communication**: Robots and drones report back status and coordinates.

---

## Repository Structure
├── esp27july25.ino # ESP32 firmware for Pololu communication over MQTT
├── pololu_integrated27JULY.py # Pololu 3pi+ robot code for executing commands from ESP32
├── pololucrazyHUB.py # Central hub script managing waypoints and device coordination

---

## Requirements

### Hardware
- Crazyflie 2.1 Drone + Crazyflie PA (Positioning Anchor) system  
- Pololu 3pi+ 2040 OLED robot  
- ESP32 module for each Pololu robot  
- Wi-Fi router for LAN communication  

### Software & Libraries
- **Python 3.x**  
  - `paho-mqtt`  
  - `cflib` (Crazyflie Python library)  
- **Arduino IDE** with:
  - `ESP32MQTTClient` library  
  - `WiFi` library  

---

## Setup

### 1. ESP32 Firmware
1. Open `esp27july25.ino` in Arduino IDE.
2. Set your Wi-Fi SSID and password.
3. Set your MQTT broker IP (`192.168.1.10` by default).
4. Upload to the ESP32 connected to each Pololu robot.

### 2. Pololu Robot Code
1. Load `pololu_integrated27JULY.py` onto the Pololu 3pi+ 2040 OLED.
2. Ensure the UART pins match the ESP32 connection (TX=GP28, RX=GP29).
3. The robot will execute commands received over MQTT.

### 3. Hub Script
1. Run `pololucrazyHUB.py` on your central control computer.
2. The script will:
   - Connect to the MQTT broker.
   - Assign waypoints to drones and robots.
   - Process status updates from all agents.

---

## Usage
1. Power on the Crazyflie and ensure the PA system is active.
2. Power on the Pololu robots and their ESP32 bridges.
3. Start the hub script to send navigation commands.
4. Monitor terminal output for position updates and status logs.

---

## Example Command Flow
1. Hub sends `goto` command to a Crazyflie at `(x, y, z)`.
2. Crazyflie moves using PA-based positioning.
3. Hub sends movement commands to Pololu robots.
4. Robots report back coordinates upon reaching their waypoint.
