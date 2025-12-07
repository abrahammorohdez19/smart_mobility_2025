# Smart Mobility 2025--Autonomous Driving in QCar1 and AMR1 implementing ROS2

This repository contains the complete development workspace used to implement autonomous mobility solutions on two platforms (Quanser QCar1 and AMR1) as part of the Smart Mobility Concentration at Tecnológico de Monterrey, Campus Puebla (Aug–Dec 2025).

Quanser QCar 1 (LiDAR, IMU, Encoders, Pure Pursuit controller)
AMR1 Custom Vehicle (ESP32-based sensors, IMU + encoders, CAN-based, Pure Pursuit controller)

The system includes:

Sensor acquisition (IMU, LiDAR, Encoders)

Real-time filtering (Kalman)

Autonomous navigation (Pure Pursuit, Ackermann model)

Hardware firmware (ESP32, Jetson Nano Orin) for integrated sensing
ROS 2 nodes for state estimation, mapping, and control

---
smart_mobility_2025/
│
├── Data/                              # Dataset de trayectorias, análisis y CSVs reales del AMR/QCar
│   ├── amr_pure_pursuit/              # Resultados de Pure Pursuit en AMR
│   ├── qcar_expo_pure_pursuit/        # Resultados de pruebas del QCar
│   └── qcar_pure_pursuit/             # Métricas y trayectorias del QCar
│
├── hardware_instrumentation/          # Firmware para microcontroladores (ESP32/Arduino)
│   ├── encoders/                      # Lectura de encoders del AMR/QCar
│   │   ├── contador_encoder_amh19.ino
│   │   └── encoder_amr_amh19_pyserial.ino
│   │
│   ├── imu/                           # Lectura raw de IMU y variantes ROS2
│   │   ├── imu_bo055_esp32.ino
│   │   └── imu_bo055_esp32_ros2.ino
│   │
│   └── imu_encoder_amr_pyserial_amh19.ino   # Firmware combinado IMU + encoder (AMR)
│
├── src/                               # Paquetes ROS2 del workspace
│   ├── sensores_kalman/               # Principal paquete ROS2 del proyecto
│   │   ├── resource/
│   │   ├── sensores_kalman/           # Nodos ROS2 reales
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
│   │   ├── test/                      # Utils, pruebas unitarias
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   └── setup.py
│   │
│   └── sm_interfaces/                 # Interfaces ROS2 personalizadas (.msg)
│
├── team_amr/ (submodule)              # Repositorio del resto del equipo (AMR—Movilidad Inteligente)
│
├── LICENSE
├── README.md
└── .gitignore


---

## Ejecución del sistema

**Lanzar sensores del QCar físico:**
```bash
ros2 launch qcar qcar_launch_modified.py

2️Activar control mediante el gamepad Logitech F710:

ros2 run qcar command

3️Ejecutar el nodo de fusión EKF:

ros2 run sensores_kalman ekf_fusion_node

4️(Opcional) Ejecutar nodos individuales:

ros2 run sensores_kalman imu_kalman_node
ros2 run sensores_kalman lidar_qos_node

Descripción de los nodos
Nodo	Descripción	Tópicos utilizados
imu_kalman_node	Aplica un filtro de Kalman a los datos de la IMU para reducir ruido en aceleraciones y giros.	/qcar/imu
lidar_qos_node	Lee el LiDAR a 10 Hz (QoS ajustado), aplica Kalman punto a punto y muestra los datos polares filtrados.	/qcar/scan
ekf_fusion_node	Fusión de sensores (IMU + LiDAR + Encoder) mediante un Filtro de Kalman Extendido. Estima pose [x, y, θ, v].	/qcar/imu, /qcar/scan, /qcar/velocity
velocity_listener	Nodo auxiliar para validar la lectura del encoder/velocidad.	/qcar/velocity
Formulación del EKF

Modelo de movimiento del QCar (cinemático):
x˙=vcos⁡(θ),y˙=vsin⁡(θ),θ˙=ω,v˙=0
x˙=vcos(θ),y˙​=vsin(θ),θ˙=ω,v˙=0

Vector de estado:
X=[x, y, θ, v]T
X=[x,y,θ,v]T

Jacobiano del modelo (matriz F):
F=[10−vsin⁡(θ) dtcos⁡(θ) dt01vcos⁡(θ) dtsin⁡(θ) dt00100001]
F=
​1000​0100​−vsin(θ)dtvcos(θ)dt10​cos(θ)dtsin(θ)dt01​
​

Covarianzas:
Q=diag(0.01, 0.01, 0.02, 0.05)
Q=diag(0.01,0.01,0.02,0.05)
Rimu=[0.02],Rvel=[0.05],Rlidar=diag(0.1, 0.1)
Rimu​=[0.02],Rvel​=[0.05],Rlidar​=diag(0.1,0.1)
Visualización en tiempo real

    IMU: gráficas en tiempo real de aceleraciones crudas vs. filtradas.

    LiDAR: radar polar con datos en 360° y reducción de ruido.

    EKF: impresión en consola de la estimación de pose y velocidad.

 Autor

Abraham Moro Hernández
Tecnológico de Monterrey – Campus Puebla
Concentración en Movilidad Inteligente 
LinkedIn www.linkedin.com/in/abraham-moro-hernandez-amh19

Licencia

Distribuido bajo la licencia Apache 2.0, compatible con el ecosistema ROS 2.


Basado en el entorno Quanser QCar ROS 2 y las librerías pal.products.qcar.
Desarrollado como parte del curso Movilidad Inteligente (MR3004C) — Tecnológico de Monterrey.


