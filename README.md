# Smart Mobility 2025--Autonomous Driving in QCar1 and AMR1 implementing ROS2

This repository contains the complete development workspace used to implement autonomous mobility solutions on two platforms (Quanser QCar1 and AMR1) as part of the Smart Mobility Concentration at Tecnológico de Monterrey, Campus Puebla (Aug–Dec 2025).

Quanser QCar 1 (LiDAR, IMU, Encoders, Pure Pursuit controller)
AMR1 Custom Vehicle (ESP32-based sensors, IMU + encoders, CAN-based, Pure Pursuit controller)

The system includes:

Sensor acquisition (IMU, LiDAR, Encoders)

Real-time filtering (Kalman)

Autonomous navigation (Pure Pursuit, Ackermann model)

Hardware/firmware 


## Workspace Structure
```
smart_mobility_2025/
│
├── Data/                                   # Datasets: trajectories, CSV logs, experiment metrics
│   ├── amr_pure_pursuit/                   # AMR Pure Pursuit logs (plots + CSV)
│   ├── qcar_expo_pure_pursuit/             # QCar results for the Expo demonstration
│   └── qcar_pure_pursuit/                  # QCar Pure Pursuit experiments
│
├── hardware_instrumentation/               # ESP32 / Arduino firmware
│   ├── encoders/
│   │   ├── contador_encoder_amh19.ino
│   │   └── encoder_amr_amh19_pyserial.ino
│   │
│   ├── imu/
│   │   ├── imu_bo055_esp32.ino
│   │   └── imu_bo055_esp32_ros2.ino
│   │
│   └── imu_encoder_amr_pyserial_amh19.ino  # Combined IMU + Encoder firmware
│
├── src/                                    # ROS2 workspace packages
│   ├── sensores_kalman/                    # Main ROS2 package (Kalman filters, pose, Pure Pursuit)
│   │   ├── resource/
│   │   ├── sensores_kalman/
│   │   │   ├── amr_encoder_amh19.py
│   │   │   ├── amr_imu_encoder.py
│   │   │   ├── amr_pure_pursuit.py
│   │   │   ├── lidar_kalman_node_amh19.py
│   │   │   ├── pose_ekf_amr.py
│   │   │   ├── pose_ekf_qcar_2.py
│   │   │   ├── pose_final_qcar.py
│   │   │   ├── qcar_lidar_alert_2.py
│   │   │   ├── qcar_pure_pursuit.py
│   │   │   ├── qcar_watchdog_node.py
│   │   │   └── trayectoria_grabar_csv_node.py
│   │   │
│   │   ├── test/
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   └── setup.py
│   │
│   └── sm_interfaces/                       # Custom ROS2 interfaces (.msg)
│
├── team_amr/                                # Submodule: AMR team workspace firmware 
│
├── LICENSE
├── README.md
└── .gitignore
```



## System Launch

This workspace includes the full autonomous driving stack for the Quanser QCar1 and the AMR1 platform.  
Below is the recommended launch sequence for running the QCar on real hardware.

---

### **Launch QCar sensor drivers**
Starts all hardware interfaces (IMU, LiDAR, encoder, motor interface, camera if available).

```bash
ros2 launch qcar qcar_launch_sm_aim.py
```

---

## Autonomous Stack Nodes for QCar1

### **Run QCar Pose Estimator**
Integrates IMU + encoder commands to estimate pose \[x, y, θ\].

```bash
ros2 run sensores_kalman pose_ekf_qcar_2
```

### **Run Obstacle Detector (LiDAR-based)**
Publishes `/qcar/obstacle_alert` when obstacles appear in the frontal cone.

```bash
ros2 run sensores_kalman qcar_lidar_alert_2
```

### **Run Watchdog Node**
Ensures a STOP command is sent if `/qcar/user_command` stops publishing.

```bash
ros2 run sensores_kalman qcar_watchdog_node
```

---

## **Pure Pursuit Controller (Autonomous Driving)**

### **Run Pure Pursuit controller**
This node computes the steering command based on the loaded trajectory and publishes automatic driving commands,
creates a circular trajectory if none waypoints.csv are loaded.

```bash
ros2 run sensores_kalman pure_pursuit_node
```
ROS2 Command to load a recorded trajectory.
```bash
ros2 run sensores_kalman pure_pursuit_node --ros-args -p path_csv:=/home/user/route/name_trajectory.csv
```

---

## Optional Individual Sensor Nodes

Run only if you want to debug/visualize each sensor independently:


### LiDAR visualization 
```bash
ros2 run sensores_kalman lidar_kalman_node_amh19
```

### Record Trajectories to CSV
```bash
ros2 run sensores_kalman trayectoria_grabar_csv_node
```
and command node has to run for Qcar trayectories
```bash
ros2 run qcar command
```

## Autonomous Stack Nodes for AMR1

### **Run AMR1 Odometry**
Integrates IMU + encoder data and publishes in a custom interface \[pulses, velocity, distance, acceleration in X, Y, Z and roll, pitch, yaw\].

```bash
ros2 run sensores_kalman amr_imu_encoder
```
### **Run AMR1 Pose Estimator**
Integrates IMU + encoder commands to estimate pose \[x, y, θ\].

```bash
ros2 run sensores_kalman amr_pose_ekf_amr
```

## **Pure Pursuit Controller (Autonomous Driving)**

### **Run Pure Pursuit controller**
This node computes the steering command based on the loaded trajectory and publishes automatic driving commands,
creates a sinewave trajectory if none waypoints.csv are loaded.

```bash
ros2 run sensores_kalman amr_pure_pursuit
```
ROS2 Command to load a recorded trajectory.
```bash
ros2 run sensores_kalman pure_pursuit_node --ros-args -p path_csv:=/home/user/route/name_trajectory.csv
```

---

---

# Node Description Summary

| Node | Description | Subscribes | Publishes |
|------|-------------|--------|--------|
| **pose_ekf_qcar_2** | Pose estimation from IMU + Encoder | `/qcar/velocity`, `/qcar/imu` | `/qcar/pose`|
| **qcar_lidar_alert_2** | Frontal LiDAR obstacle detection | `/qcar/scan`| `/qcar/obstacle_alert` |
| **qcar_pure_pursuit** | Pure Pursuit controller for autonomous driving | `/qcar/pose`, `/qcar/user_command`, `/qcar/obstacle_alert`| None |
| **qcar_watchdog_node** | Safety node: forces STOP if command frequency drops | `/qcar/user_command` | None |
| **lidar_kalman_node_amh19** | LiDAR QoS + filtered visualization | `/qcar/scan` | None |

---
## Development & Execution Environment

This project was developed and tested using a combination of high-performance laptops and embedded hardware to ensure reliable real-time autonomy for both QCar1 and AMR1 platforms.

### **Main Development Machine**

All ROS2 nodes, Pure Pursuit controllers, EKF estimators, and integration pipelines were developed and executed on:

ASUS ROG Strix G16

CPU: Intel Core i9

GPU: NVIDIA RTX 4060

RAM: 32 GB

Storage: 1 TB NVMe SSD

OS: Ubuntu 22.04 LTS

Middleware: ROS2 Humble 

This machine was used to run:

All ROS 2 launch files

Real-time QCar teleoperation

Pure Pursuit & control nodes

Sensor fusion (IMU, LiDAR, encoder)


### **Supporting Development Laptop**

A secondary system was also used for testing, debugging and remote execution:

ASUS TUF Gaming (Ryzen)

CPU: AMD Ryzen Series

GPU: NVIDIA RTX series

RAM: 16 GB

OS (host): Windows 11

WSL2 Distribution: Ubuntu 22.04 LTS

Used for Pure Pursuit and control nodes
Dataset generation, visualization, and analysis



### **Embedded Computing for AMR1**

To support autonomous navigation and sensor acquisition for the AMR1 platform, a dedicated embedded system was used:

NVIDIA Jetson Orin Developer Kit

ARM64 architecture

Hosted ROS 2 nodes for IMU, encoders and Pure Pursuit

Executed firmware communication with ESP32 microcontrollers

Used as the on-board brain for:

Low-latency sensor acquisition

Hardware-level filtering

Real-time control loops

CAN / serial integration


### **Microcontroller Layer (ESP32)**

A set of ESP32 microcontrollers was used for:

Encoder acquisition

IMU acquisition

Serial and CAN communication

Integrated hardware instrumentation

Firmware was placed under:
hardware_instrumentation/

## A message for those who innovates:

## Authors

Abraham Moro Hernández
Tecnológico de Monterrey – Campus Puebla
Concentración en Movilidad Inteligente 
LinkedIn www.linkedin.com/in/abraham-moro-hernandez-amh19

Licencia

Distribuido bajo la licencia Apache 2.0, compatible con el ecosistema ROS 2.


Basado en el entorno Quanser QCar ROS 2 y las librerías pal.products.qcar.
Desarrollado como parte del curso Movilidad Inteligente (MR3004C) — Tecnológico de Monterrey.


