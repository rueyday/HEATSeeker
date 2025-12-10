# HEATSeeker: Smart Glasses with Remote-Operated Rescue Robot


> **[Watch the HEATSeeker Video Demo!](https://www.youtube.com/watch?v=Ano5yxAiWmE)**
> **[Visit the Project Website Here]** (Insert actual project webpage link)

**Contributors:** Jungeun Seo • Sophie Yang • Ruey Day • Yi Keen Lim  
**Course:** EECS 373 (Embedded Systems) at the University of Michigan  

---

## Project Overview

**Heat Seeker** is an embedded systems project that integrates smart glasses and a remote-operated rescue robot to track and visualize heat signatures. The system enables a user to monitor a robot’s thermal camera feed through a head-up display (HUD) embedded in the glasses and control the robot remotely using a wireless controller. The goal is to provide a lightweight, responsive, and intuitive human–robot interface for search-and-rescue applications.

---

## System Description

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