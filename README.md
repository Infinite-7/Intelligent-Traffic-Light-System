# Intelligent Traffic Light System Using 4 Arduino Boards

## Overview
This project addresses urban traffic congestion and pedestrian safety using **4 Arduino boards**. The system dynamically manages traffic light sequences, integrates pedestrian requests, and adjusts green light durations in real time based on traffic density.

## Features
- **Adaptive Traffic Control**:
  - Adjusts green light durations dynamically based on traffic density using an LDR and potentiometer.
- **Pedestrian Safety**:
  - Handles pedestrian crossing requests via a debounced button, ensuring safety without disrupting the traffic sequence.
- **Board Synchronization**:
  - Coordinates 4 Arduino boards using UART communication for smooth traffic management.

## Hardware Components
- **Master Board**: Manages the main traffic intersection and pedestrian requests.
- **Slave Boards (1, 2, 3)**: Control secondary intersections independently after receiving signals from the master board.
- **LDR (Light Dependent Resistor)**: Measures ambient light to estimate traffic density.
- **Potentiometer**: Sets a threshold for adaptive control of green light durations.
- **Button**: Debounced button for pedestrian crossing requests.
- **LEDs**: Represent traffic light states for each board (Red, Yellow, Green).

## How to Run
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Infinite-7/Intelligent-Traffic-Light-System.git
   cd Intelligent-Traffic-Light-System
   
2. **Set Up Hardware**:
    - Follow the circuit diagrams to assemble the components for the master and slave boards.
  
3. **Upload Code**:
    - Upload master_board.ino to the master Arduino board.
    - Upload slave_board.ino to the three slave Arduino boards.
  
## Future Enhancements
- **IoT Integration**:
  - Enable remote traffic monitoring and control via cloud-based platforms.
- **Additional Sensors**:
  - Integrate ultrasonic sensors for better vehicle detection at intersections.
- **Mobile App Development**:
  - Provide real-time traffic updates and system controls through a mobile interface.
