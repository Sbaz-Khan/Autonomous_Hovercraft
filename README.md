# Autonomous Hovercraft

**COEN/ELEC 290 Engineering Design Project — Concordia University**

## Project Overview

This project involved the design, construction, and testing of an Arduino-based autonomous hovercraft capable of navigating through a maze while detecting surrounding walls and obstacles.

The hovercraft combined mechanical fabrication, sensor-based navigation, embedded control, propulsion, and steering into a single autonomous platform. Infrared and ultrasonic sensors provided information about the surrounding environment, while an Arduino microcontroller processed the sensor measurements and controlled the hovercraft's movement.

The vehicle used a dedicated lift fan to create an air cushion beneath the chassis, a thrust fan for forward propulsion, and a servo-controlled steering mechanism for directional control.

---

## Objectives

The main objectives of the project were to:

- Design and construct a functional hovercraft
- Generate sufficient lift for stable hovering
- Reduce friction between the hovercraft and the ground
- Detect nearby walls and obstacles using distance sensors
- Navigate autonomously through a maze
- Integrate sensing, control, propulsion, and mechanical systems
- Test and improve the system through iterative experimentation

---

## System Architecture

    Infrared / Ultrasonic Sensors
                 │
                 ▼
              Arduino
                 │
                 ▼
        Navigation / Control
           ┌─────┼─────┐
           ▼     ▼     ▼
         Lift  Thrust  Steering
          Fan    Fan    Servo

The infrared and ultrasonic sensors provide information about nearby walls and obstacles to the Arduino. Based on these measurements and the programmed navigation logic, the Arduino generates commands for the propulsion and steering systems.

---

## Hardware

The prototype included:

- Arduino microcontroller
- Ultrasonic distance sensor
- Lift fan
- Thrust / propulsion fan
- Servo motor for directional control
- Battery and power system
- Custom hovercraft chassis
- Flexible plastic hovercraft skirt

---

## Software

The control system was developed using:

- Arduino IDE
- C/C++
- Sensor processing
- Autonomous navigation logic
- Motor and actuator control

---

## How It Works

### Lift System

A lift fan forces air into a flexible plastic skirt underneath the hovercraft. As the skirt inflates, air escapes beneath the vehicle and creates a thin cushion of air between the hovercraft and the ground.

This air cushion significantly reduces friction, allowing the hovercraft to move more easily across the surface.

### Propulsion and Steering

A separate thrust fan provides forward propulsion.

Directional control is achieved using a servo-controlled steering mechanism that redirects the thrust, allowing the hovercraft to turn and change direction while moving through the maze.

### Obstacle Detection and Navigation

Ultrasonic sensors continuously measure the distance between the hovercraft and nearby walls or obstacles.

These measurements are processed by the Arduino, which uses the navigation logic to determine the appropriate propulsion and steering commands.

By continuously sensing its surroundings and adjusting its movement, the hovercraft is able to navigate through the test environment autonomously.

---

## My Contributions

My primary contribution to the project focused on the **physical construction, mechanical integration, calibration, and testing of the hovercraft**.

My work included:

- Fabricating and assembling the hovercraft chassis and body
- Cutting, constructing, and installing the flexible plastic hovercraft skirt
- Assisting with the physical integration of the lift, propulsion, sensing, and steering components
- Adjusting component placement and weight distribution to improve balance and stability
- Assisting with infrared and ultrasonic sensor calibration
- Performing practical testing and evaluating the hovercraft's behavior under different conditions
- Troubleshooting mechanical and stability issues during development
- Supporting final system integration and testing

The navigation software was developed as part of the overall team project. My contribution was primarily focused on the physical platform, mechanical assembly, system integration, sensor calibration, stability, and experimental testing.

---

## Development and Testing

The hovercraft was developed using an iterative design and testing process.

Individual subsystems were evaluated before being integrated into the final platform. Testing included the lift system, propulsion system, steering response, sensor measurements, weight distribution, and overall hovercraft stability.

Mechanical adjustments and sensor calibration were performed throughout development to improve the hovercraft's ability to hover consistently, maintain stability, detect obstacles, and navigate through the maze.

### Construction

![Hovercraft Construction](media/construction.jpg)

### Sensor and Navigation Testing

![Sensor and Navigation Testing](media/testing.jpg)

### Final Hovercraft

![Final Hovercraft](media/final_hovercraft.jpg)

---

## Final Demonstration

A video of the completed hovercraft navigating the test environment can be added below.

**Demo:** [Add video link]

---

## Results

The final prototype successfully demonstrated:

- Stable hovering using the air-cushion lift system
- Forward propulsion using the thrust fan
- Directional control using the servo steering mechanism
- Detection of nearby walls and obstacles
- Integration of mechanical, electrical, sensing, and control systems
- Autonomous navigation within the designated test environment

The completed hovercraft demonstrated the successful integration of multiple engineering subsystems into a functional autonomous vehicle.

---

## What I Learned

Through this project, I gained practical experience with:

- Engineering prototyping and fabrication
- Mechanical assembly and system integration
- Hovercraft lift and propulsion principles
- Weight distribution and vehicle stability
- Infrared and ultrasonic sensor calibration
- Embedded sensing and control systems
- Hardware/software interaction
- Experimental testing and troubleshooting
- Iterative engineering design
- Engineering teamwork and system integration

---

## Course

**COEN/ELEC 290 — Engineering Team Design Project**  
**Concordia University**
