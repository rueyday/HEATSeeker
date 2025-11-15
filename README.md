# HEATSeeker: Smart Glasses with Remote-Operated Rescue Robot

**Contributors:**  
Jungeun Seo • Sophie Yang • Ruey Day • Yi Keen Lim  
**Course:** EECS 373 (Embedded Systems) at the University of Michigan  

---

## 🧠 Project Overview

**Heat Seeker** is an embedded systems project that integrates smart glasses and a remote-operated rescue robot to track and visualize heat signatures. The system enables a user to monitor a robot’s thermal camera feed through a head-up display (HUD) embedded in the glasses and control the robot remotely using a wireless controller.

The goal is to provide a lightweight, responsive, and intuitive human–robot interface for search-and-rescue applications.

---

## 🚀 System Description

### Smart Glasses
- **HUD Display:** Shows live robot camera feed, battery level, system heat, and movement direction.
- **Control Panel:** Two physical buttons on the glasses’ temple:
  - **Mode Button:** Switches between HUD display modes.
  - **Scale Button:** Adjusts the display scale (accounts for focal length).
- **Wireless Communication:** Receives robot telemetry and video feed via XBee module.

### Rescue Robot
- **Thermal Tracking:** Equipped with an **AMG8833 IR thermal camera** to detect and follow heat signatures.
- **Wireless Control:** Receives navigation commands from the controller over XBee.
- **Feedback:** Sends camera images and system status (e.g., temperature, battery) to the glasses.

### Remote Controller
- **User Input:** Sends directional and motion commands to the robot.
- **Wireless Link:** Uses XBee for communication with the robot.

---

## ⚙️ System Architecture

### Functional Overview
- The **robot** detects and transmits thermal data.
- The **glasses** visualize that data in real-time.
- The **controller** directs the robot’s movement.

### Communication Flow
1. Controller → Robot: Navigation and movement commands  
2. Robot → Glasses: Camera data and system telemetry  
3. User → Glasses: Display mode and scale selection

---

## 🧩 Component List

### In-Lab Components
- STM32 microcontroller board  
- Push buttons & simple switches  
- LCD / OLED display (for HUD)  
- DC motors (for mobility)  
- Resistors (high-resistance for pull-downs)  
- Capacitors (signal stability)  
- XBee modules (for wireless communication)

### Possible Additions
- LiPo battery (for portable power)  
- Infrared thermal camera (**AMG8833 8x8 IR array**)  
- Range sensor (**VL53L1X**)  

---

## 🌟 Reach Goals

- Design a **custom PCB** for the glasses module  
- Enable **video recording** and onboard SD storage  
- Add **head-movement control** using accelerometers  
- Implement **self-orientation sensors** on the robot  
- Upgrade to **capacitive touch buttons** on the glasses  

---

## 🔧 I/O Summary

**Inputs:**
- Power switches (glasses and robot)  
- Display mode button  
- Scale adjustment button  
- Controller directional input  

**Outputs:**
- Visual data on HUD (thermal view, status info)  
- Robot motor responses to control inputs  

---

## 📊 Diagrams

### System Overview
- Robot ↔ Glasses ↔ Controller communication network  
- Embedded systems for each subsystem (see component diagrams)  

*(Include your Figure 3–6 diagrams here as images when uploading to GitHub)*

---

## 📚 References

- **Range Sensor:** [VL53L1X Distance Sensor – STMicroelectronics](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html)  
- **Thermal Camera:** [AMG8833 8x8 IR Array – Amazon](https://www.amazon.com/dp/B07V5Y8J91)  
- **Smart Glasses Base Design:** [Brilliant Labs Monocle Documentation](https://docs.brilliant.xyz/monocle/monocle/)  

---

## 🏁 Summary

**Heat Seeker** demonstrates the integration of embedded communication, sensing, and human–machine interfaces in a cohesive system. By merging wearable HUD technology with a thermal-tracking robot, this project explores how embedded systems can assist in real-world rescue and navigation tasks.

---

> **Note:** This repository includes code, schematics, and documentation for the EECS 373 final project. All materials are intended for educational use.
