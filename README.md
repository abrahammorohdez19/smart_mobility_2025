# Smart Mobility 2025--Autonomous Driving in QCar1 and AMR1 implementing ROS2

This repository contains the complete development workspace used to implement autonomous mobility solutions on two platforms (Quanser QCar1 and AMR1) as part of the Smart Mobility Concentration at Tecnológico de Monterrey, Campus Puebla (Aug–Dec 2025).

Quanser QCar 1 (LiDAR, IMU, Encoders, Pure Pursuit controller)
AMR1 Custom Vehicle (ESP32-based sensors, IMU + encoders, CAN-based, Pure Pursuit controller)

The system includes:

Sensor acquisition (IMU, LiDAR, Encoders)

Real-time filtering (Kalman)

Autonomous navigation (Pure Pursuit, Ackermann model)

Hardware firmware (ESP32, Jetson Nano Orin) for integrated sensing
ROS2 (Humble) nodes for state estimation, mapping, and control


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
ros2 launch qcar qcar_launch_modified_aim_sm.py
```

---

## Autonomous Stack Nodes

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
This node computes the steering command based on the loaded trajectory and publishes automatic driving commands.

For QCar:
```bash
ros2 run sensores_kalman qcar_pure_pursuit
```

For AMR:
```bash
ros2 run sensores_kalman amr_pure_pursuit
```

---

## 🔍 Optional Individual Sensor Nodes

Run only if you want to debug/visualize each sensor independently:

### IMU Kalman Filter
```bash
ros2 run sensores_kalman imu_kalman_node
```

### LiDAR visualization + QoS handling
```bash
ros2 run sensores_kalman lidar_kalman_node_amh19
```

### Record Trajectories to CSV
```bash
ros2 run sensores_kalman trayectoria_grabar_csv_node
```

---

# Node Description Summary

| Node | Description | Topics |
|------|-------------|--------|
| **qcar_pose** | Pose estimation from IMU + velocity | `/qcar/velocity`, `/qcar/user_command` |
| **qcar_pure_pursuit** | Pure Pursuit controller for autonomous driving | `/qcar/pose`, `/qcar/user_command` |
| **qcar_lidar_alert_2** | Frontal LiDAR obstacle detection | `/qcar/scan`, `/qcar/obstacle_alert` |
| **qcar_watchdog_node** | Safety node: forces STOP if command frequency drops | `/qcar/user_command` |
| **imu_kalman_node** | IMU noise reduction using a Kalman filter | `/qcar/imu` |
| **lidar_kalman_node_amh19** | LiDAR QoS + filtered visualization | `/qcar/scan` |
| **ekf_fusion_node** | Extended Kalman Filter combining IMU + LiDAR + encoder | `/qcar/imu`, `/qcar/scan`, `/qcar/velocity` |

---




 Autor

Abraham Moro Hernández
Tecnológico de Monterrey – Campus Puebla
Concentración en Movilidad Inteligente 
LinkedIn www.linkedin.com/in/abraham-moro-hernandez-amh19

Licencia

Distribuido bajo la licencia Apache 2.0, compatible con el ecosistema ROS 2.


Basado en el entorno Quanser QCar ROS 2 y las librerías pal.products.qcar.
Desarrollado como parte del curso Movilidad Inteligente (MR3004C) — Tecnológico de Monterrey.


