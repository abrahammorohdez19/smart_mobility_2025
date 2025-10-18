# Smart Mobility QCar ROS2

Proyecto académico para el desarrollo e integración de sensores del **Quanser QCar** en **ROS 2**.  
Incluye filtros de **Kalman** (IMU, LiDAR y Encoder), así como un **Filtro de Kalman** para la fusión de sensores y estimación de pose del vehículo.

---

## Estructura del workspace

smart_mobility_qcar_ros2/
├── src/
│ ├── sensores_kalman/ # Paquete principal
│ │ ├── imu_kalman_node.py # Filtro de Kalman individual para IMU
│ │ ├── lidar_qos_node.py # Filtro de Kalman + QoS para LiDAR
│ │ ├── ekf_fusion_node.py # Filtro de Kalman Extendido (fusión de sensores)
│ │ └── velocity_listener.py # Nodo auxiliar para leer velocidad del QCar
│ ├── qcar_description/ # URDF y descripción del vehículo
│ ├── qcar_gazebo/ # Simulación del QCar en Gazebo
│ └── otros_nodos/ # Scripts o utilidades adicionales
├── build/
├── install/
└── log/


> 🔹 Solo la carpeta `src/` se versiona.  
> Las carpetas `build/`, `install/` y `log/` se excluyen mediante `.gitignore`.

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


