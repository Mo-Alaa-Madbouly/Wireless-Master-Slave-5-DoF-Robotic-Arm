# Wireless Master–Slave 5-DOF Robotic Arm

A wireless **5-DOF Master–Slave robotic arm** that captures the operator's movements using a master arm and reproduces them on a remote slave arm.

The project combines **mechanical design, embedded systems, wireless communication, servo control, and ROS 2 / MoveIt 2** into a single robotic platform.

> **MET 467 – Mechatronics Project**  
> Mechatronics & Robotics Engineering Program  
> Faculty of Engineering – Assiut University  
> 2025–2026

---

## 📌 Overview

The system consists of two robotic arms:

- **Master Arm** → operated by the user and used to measure joint positions.
- **Slave Arm** → remotely reproduces the master's movements.

The master arm uses **five potentiometers** to measure the position of its five joints. An ESP32 processes these measurements and sends the joint angles wirelessly using **ESP-NOW**.

A second ESP32 receives the data and controls the five servo motors of the slave arm through a **PCA9685 PWM driver**.

The system supports:

1. **Real-Time Mode** – the slave follows the master continuously.
2. **Record & Playback Mode** – up to four poses can be recorded and replayed.
3. **ROS 2 / MoveIt Mode** – planned trajectories can be generated using ROS 2 and MoveIt 2 and sent toward the physical robot.

The overall concept is:

```text
        MASTER ARM
            │
            │ 5 Potentiometers
            ▼
          ESP32
            │
            │ ESP-NOW
            ▼
          ESP32
            │
            ▼
         PCA9685
            │
            ▼
      5 Servo Motors
            │
            ▼
        SLAVE ARM
```

---

## ✨ Main Features

- 🦾 5-degree-of-freedom robotic arm
- 🎮 Wireless master–slave teleoperation
- 📡 ESP-NOW low-latency communication
- 🎛️ Potentiometer-based joint position sensing
- ⚙️ PCA9685 servo control
- 🔄 Real-time motion replication
- 💾 Record and playback of up to **4 poses**
- 🔧 Interchangeable end effectors
- 🦾 Gripper end effector
- 🥄 Ladle end effector
- 🤖 ROS 2 integration
- 🧠 MoveIt 2 motion planning
- 🖥️ RViz visualization
- 🌐 Gazebo simulation
- 🔌 Physical hardware control through microcontroller interfaces

The project was designed to provide a relatively low-cost robotic platform suitable for **education, research, teleoperation, automation, and remote manipulation**.

---

# 🏗️ System Architecture

The project can be divided into four main layers:

```text
┌─────────────────────────────────────────────┐
│                 MASTER ARM                  │
│                                             │
│  Mechanical joints + 5 Potentiometers      │
│                     │                       │
│                     ▼                       │
│                   ESP32                     │
└─────────────────────┬───────────────────────┘
                      │
                      │ ESP-NOW
                      │
                      ▼
┌─────────────────────────────────────────────┐
│                  SLAVE ARM                  │
│                                             │
│                   ESP32                     │
│                     │                       │
│                     ▼                       │
│                  PCA9685                    │
│                     │                       │
│                     ▼                       │
│              5 Servo Motors                 │
│                     │                       │
│                     ▼                       │
│                Robotic Arm                  │
└─────────────────────────────────────────────┘
```

For advanced control, ROS 2 and MoveIt 2 form an additional software layer:

```text
SolidWorks
    │
    │ URDF / Mesh Export
    ▼
ROS 2
    │
    ├── Robot Description
    ├── ros2_control
    ├── Gazebo
    ├── RViz
    │
    ▼
MoveIt 2
    │
    ├── Kinematics
    ├── Motion Planning
    └── Trajectory Generation
            │
            ▼
      Hardware Interface
            │
            ▼
        Physical Robot
```

---

# 🦾 Mechanical Design

## Slave Arm

The slave robot uses a **serial robotic arm configuration** with five revolute joints.

The five degrees of freedom are:

| Joint | Function |
|---|---|
| J1 | Base rotation |
| J2 | Shoulder |
| J3 | Elbow |
| J4 | Wrist pitch |
| J5 | Wrist orientation |

This configuration provides a balance between mechanical complexity, workspace, and controllability.

The structural components were designed for 3D printing using **PLA+**.

### Torque Design

A torque analysis was performed to select suitable actuators for the main joints.

The calculated required torque at the critical joint was approximately:

```text
T = 29.66 kg·cm
```

Therefore, **35 kg·cm FT7135M servo motors** were selected for the main joints.

---

# 🔧 Interchangeable End Effectors

One of the main mechanical features is the **twist-lock / bayonet-style end-effector mechanism**.

The tool can be replaced using a short rotational locking motion.

```text
Robot Wrist
     │
     ▼
 ┌───────────┐
 │ Twist Lock│
 └─────┬─────┘
       │
   ┌───┴────┐
   │        │
   ▼        ▼
Gripper    Ladle
```

### Gripper

The gripper is designed for:

- Picking objects
- Holding objects
- Pick-and-place operations

It uses a small **SG90 servo** for opening and closing.

### Ladle

The ladle is intended for:

- Scooping
- Carrying small particles
- Transferring liquids or similar materials

Both tools use the same twist-lock interface, allowing the robot to change tools depending on the task.

---

# 🎛️ Master Arm

The master arm is used as a human-operated input device.

Each of its five joints is connected to a rotary potentiometer.

When the operator moves a joint:

```text
Mechanical Joint
       │
       ▼
Potentiometer
       │
       ▼
Analog Voltage
       │
       ▼
ESP32 ADC
       │
       ▼
Joint Angle
```

The ESP32 reads the potentiometers through its ADC and converts the readings into joint angles.

The system uses joint-specific calibration because each potentiometer may have a different physical range.

---

# 📡 Wireless Communication

## ESP-NOW

Communication between the two ESP32 boards is handled using **ESP-NOW**.

Unlike normal Wi-Fi communication, ESP-NOW allows the ESP32 devices to communicate directly without requiring a router.

```text
Master ESP32
     │
     │ ESP-NOW
     │
     ▼
Slave ESP32
```

The master acts as the transmitter and the slave acts as the receiver.

### Why ESP-NOW?

ESP-NOW was selected because the application requires:

- Low communication latency
- Direct device-to-device communication
- Small data packets
- Fast transmission
- No Wi-Fi router
- Suitable response time for teleoperation

---

# 📦 Communication Data Format

The five joint angles are packed into an `ArmPosition` structure.

Conceptually:

```text
ArmPosition
┌────────┬────────┬────────┬────────┬────────┐
│ Joint0 │ Joint1 │ Joint2 │ Joint3 │ Joint4 │
├────────┼────────┼────────┼────────┼────────┤
│   1B   │   1B   │   1B   │   1B   │   1B   │
└────────┴────────┴────────┴────────┴────────┘

Total packet size = 5 bytes
```

Each joint value represents an angle between:

```text
0° ───────────────► 180°
```

This compact packet allows the system to transmit the complete arm position efficiently.

---

# 🎮 Operating Modes

## Mode 1 – Real-Time Control

In real-time mode, the master continuously controls the slave.

```text
Move Master Joint
       │
       ▼
Read Potentiometer
       │
       ▼
Convert ADC → Angle
       │
       ▼
ESP-NOW
       │
       ▼
Slave ESP32
       │
       ▼
PCA9685
       │
       ▼
Servo Movement
```

The system does not need to transmit a new packet for every ADC reading.

A **3° deadband** is used so that small changes do not unnecessarily flood the wireless channel.

---

# 💾 Mode 2 – Record & Playback

The second mode allows the operator to record specific arm positions.

Up to **four poses** can be stored.

Example:

```text
Pose 1 → Pick position
Pose 2 → Lift object
Pose 3 → Move to destination
Pose 4 → Release position
```

The recording process stores the five joint angles at each selected moment.

```text
Master Position
      │
      ▼
Press RECORD
      │
      ▼
Capture J1..J5
      │
      ▼
Store Pose
      │
      ├── Pose 1
      ├── Pose 2
      ├── Pose 3
      └── Pose 4
```

During playback, the stored positions are sent to the slave sequentially.

---

# 🤖 Mode 3 – ROS 2 / MoveIt 2

The third mode extends the system beyond direct manual control.

The robot was modeled in **SolidWorks**, exported as a URDF/mesh model, and integrated into ROS 2.

The ROS 2 workspace is named:

```text
Jarvis
```

and contains the robot description package:

```text
jarvis_description
```

The robot model was converted into Xacro to make the configuration more modular.

---

## ROS 2 Software Stack

```text
                    MoveIt 2
                       │
                       ▼
              Motion Planning
                       │
                 OMPL / RRTConnect
                       │
                       ▼
              Trajectory Generation
                       │
                       ▼
             Controller Manager
                       │
                       ▼
         FollowJointTrajectory
                       │
                       ▼
             Hardware Interface
                       │
                       ▼
                 Arduino
                       │
                       ▼
                  PCA9685
                       │
                       ▼
                  Servos
```

The MoveIt configuration contains robot kinematics, joint limits, planning parameters, controller configuration, and other motion-planning settings.

---

# 🖥️ RViz & Gazebo

The ROS 2 package includes launch infrastructure for visualization and simulation.

### RViz

Used to visualize the robot model and interact with the motion-planning system.

```text
jarvis.display.py
```

### Gazebo

Used to simulate the robot before interacting with the physical hardware.

```text
jarvis.gazebo.py
```

This creates a development workflow where robot behavior can be tested virtually before being transferred to the real robot.

---

# 🧠 Motion Planning

MoveIt 2 generates collision-free trajectories using the configured planning system.

The documented pipeline uses:

```text
Motion Goal
     │
     ▼
MoveIt 2
     │
     ▼
OMPL / RRTConnect
     │
     ▼
Collision-Free Trajectory
     │
     ▼
Trajectory Optimization
     │
     ▼
FollowJointTrajectory
     │
     ▼
Hardware Interface
```

The project documentation describes the ROS 2 side as a bridge between high-level motion planning and low-level servo actuation.

---

# ⚙️ Hardware

## Main Components

| Component | Purpose |
|---|---|
| ESP32 ×2 | Master and slave controllers |
| Potentiometers ×5 | Master joint position sensing |
| PCA9685 | Multi-channel PWM generation |
| FT7135M ×4 | Main robot joint actuation |
| SG90 ×1 | Gripper actuation |
| 8.4 V battery | Power source |
| LCD | Mode/status display |
| Push buttons | Mode and operation control |
| 3D-printed PLA+ parts | Mechanical structure |

The electronics architecture is centered around the ESP32, with potentiometers providing master feedback and the PCA9685 generating PWM signals for the slave servos.

---

# 🔋 Power System

The documented battery is rated at:

```text
8.4 V
1000 mAh
```

The project estimated an effective usable capacity of approximately:

```text
800 mAh
```

Based on the documented current-consumption calculation, the estimated operating time was approximately:

```text
14–15 minutes
```

Actual runtime can vary depending on payload, servo loading, movement profile, and battery condition.

---

# 💻 Software

The project combines embedded firmware with robotics software.

### Embedded

- C/C++
- ESP32
- ESP-NOW
- Arduino framework
- I²C
- PCA9685 servo control

### Robotics

- ROS 2
- MoveIt 2
- RViz
- Gazebo
- URDF
- Xacro
- ros2_control

### Mechanical Design

- SolidWorks
- 3D printing
- URDF/mesh export

---

# 📂 Suggested Repository Structure

A clean repository can be organized like this:

```text
wireless-master-slave-robot-arm/
│
├── README.md
│
├── hardware/
│   ├── master/
│   ├── slave/
│   └── schematics/
│
├── firmware/
│   ├── master/
│   └── slave/
│
├── ros2_ws/
│   └── src/
│       ├── jarvis_description/
│       └── jarvis_moveit_config/
│
├── cad/
│   ├── master/
│   ├── slave/
│   └── end_effectors/
│
├── documentation/
│   └── project-report.pdf
│
└── media/
    ├── images/
    └── videos/
```

> The exact directory structure should be adjusted to match the files actually included in the repository.

---

# 🚀 How the System Works

## Real-Time Mode

1. The operator moves the master arm.
2. The five potentiometers measure the joint positions.
3. The master ESP32 reads the analog signals.
4. The readings are converted into angles.
5. The angles are packed into an `ArmPosition` packet.
6. The packet is transmitted through ESP-NOW.
7. The slave ESP32 receives the packet.
8. The slave converts the angles into PWM commands.
9. The PCA9685 generates the servo signals.
10. The slave arm reproduces the master's movement.

---

## Record & Playback Mode

1. Move the master arm to a desired position.
2. Press the record button.
3. The five joint angles are stored.
4. Repeat for additional positions.
5. Press perform/playback.
6. The stored poses are transmitted sequentially.
7. The slave reproduces the recorded sequence.

Maximum documented number of stored poses:

```text
4
```

---

## ROS 2 / MoveIt Mode

1. Define a target pose in MoveIt.
2. MoveIt calculates a valid trajectory.
3. The planner generates a collision-free joint path.
4. The trajectory is passed to the configured controller interface.
5. The hardware bridge converts joint positions into servo commands.
6. The commands are sent to the microcontroller.
7. The PCA9685 controls the physical servos.
8. The real robot follows the planned trajectory.

---

# 🔌 Servo Control

The slave ESP32 communicates with the PCA9685 through I²C.

The documented servo configuration uses:

```text
PWM Frequency : 50 Hz
Angle Range   : 0–180°
```

The firmware maps the angle to the PCA9685 PWM range:

```text
0°   → 102 ticks
180° → 492 ticks
```

This produces the PWM signal required by the servo motors.

---

# 🧪 Development Workflow

The project follows a complete mechatronics development workflow:

```text
Mechanical Design
       │
       ▼
SolidWorks CAD
       │
       ▼
3D Printing
       │
       ▼
Electronics Assembly
       │
       ▼
ESP32 Firmware
       │
       ▼
ESP-NOW Communication
       │
       ▼
Servo Control
       │
       ▼
ROS 2 Integration
       │
       ▼
MoveIt 2
       │
       ▼
Gazebo / RViz
       │
       ▼
Physical Robot
```

This makes the project useful not only as a robotic arm, but also as an example of integrating **mechanical engineering, electronics, embedded programming, and robotics software**.

---

# 📊 Project Specifications

| Specification | Value |
|---|---|
| Robot type | Master–Slave |
| Slave DOF | 5 |
| Master sensing | 5 potentiometers |
| Main controller | ESP32 |
| Wireless protocol | ESP-NOW |
| Servo driver | PCA9685 |
| Main actuators | FT7135M |
| Gripper actuator | SG90 |
| Main servo torque | 35 kg·cm |
| Slave structure | Serial manipulator |
| Material | PLA+ |
| Battery | 8.4 V, 1000 mAh |
| Recordable poses | Up to 4 |
| ROS framework | ROS 2 |
| Motion planning | MoveIt 2 |
| Visualization | RViz |
| Simulation | Gazebo |

---

# 🎯 Applications

The system can be adapted for applications such as:

- Remote robotic manipulation
- Pick-and-place tasks
- Educational robotics
- Mechatronics demonstrations
- Hazardous-environment manipulation
- Research platforms
- Basic automation
- Human-to-robot teleoperation

The project documentation specifically identifies remote handling, automation, education, and research as potential application areas.

---

# 🔮 Future Improvements

Possible extensions include:

- Computer vision
- Closed-loop joint feedback
- Encoder-based position sensing
- Trajectory interpolation
- More advanced autonomous control
- AI-based manipulation
- Improved collision handling
- More sophisticated end effectors
- Higher-capacity power system
- Better mechanical rigidity
- Additional ROS 2 hardware interfaces

The project report also identifies computer vision, closed-loop feedback, trajectory interpolation, and AI-based autonomous control as potential future developments.

---

# 👥 Team

**Mechatronics & Robotics Engineering Program**  
**Faculty of Engineering – Assiut University**

### Team Members
- Mohamed Alaa-eldeen Madbouly              Leader - Embedded System
- Ahmed Mohamed Ahmed Abdel-Aziz        ROS2 Workspace
- Rami Awad Issa                                            Master Arm Design
- Abdul-Rahman Mohamed Abdo Fawzy     Slave Arm Design
- Beshoy Medhat Monir                                 End-Effector Design

### Supervisor

**Assoc. Prof. Dr. Shuaiby Mohamed**

---

# 📄 Documentation

The complete project documentation contains the mechanical design, electrical system, control strategy, communication method, ROS 2 / MoveIt integration, calculations, and implementation details.

**Project:** Wireless Master–Slave 5-DOF Robotic Arm  
**Course:** MET 467 – Mechatronics Project  
**Academic Year:** 2025–2026

---

# ⭐ Acknowledgment

This project was developed as a practical mechatronics project combining:

**Mechanical Design + Electronics + Embedded Systems + Wireless Communication + Robotics Software**

The system demonstrates how a physical robotic platform can evolve from direct human teleoperation into a ROS 2 and MoveIt-based motion-planning system.

---
