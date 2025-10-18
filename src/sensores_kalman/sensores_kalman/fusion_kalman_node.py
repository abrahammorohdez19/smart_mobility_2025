import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, atan2

class FusionKalmanNode(Node):
    def __init__(self):
        super().__init__('fusion_kalman_node')

        # --- Estado: [x, y, θ, v]
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 0.1

        # Matrices de covarianza del proceso y medición
        self.Q = np.diag([0.05, 0.05, 0.01, 0.05])   # Ruido del modelo
        self.R = np.diag([0.3, 0.3])                 # Ruido de medición (LiDAR)

        # --- Último tiempo
        self.last_time = None

        # --- Suscripciones
        self.create_subscription(Imu, '/qcar/imu', self.imu_cb, 10)
        self.create_subscription(Vector3Stamped, '/qcar/velocity', self.vel_cb, 10)
        self.create_subscription(LaserScan, '/qcar/scan', self.lidar_cb, 10)

        # --- Publicador de estado
        self.pub_state = self.create_publisher(Float32MultiArray, '/qcar/state_estimation', 10)

        # --- Variables de entrada
        self.wz = 0.0    # velocidad angular (rad/s)
        self.v = 0.0     # velocidad lineal (m/s)
        self.z_lidar = None

        # --- Plot setup
        plt.ion()
        self.fig, self.ax = plt.subplots(1, 2, figsize=(10, 5))
        self.traj = []
        self.v_log = []

        self.get_logger().info("Fusion Kalman Node inicializado — IMU + Encoder + LiDAR activo.")

    # =============================
    # Callbacks de sensores
    # =============================
    def imu_cb(self, msg):
        self.wz = msg.angular_velocity.z

    def vel_cb(self, msg):
        self.v = msg.vector.x

    def lidar_cb(self, msg):
        # Convertir LiDAR a coordenadas cartesianas relativas
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Toma el punto más cercano como "medición de posición relativa"
        if np.any(valid):
            i_min = np.argmin(ranges[valid])
            r = ranges[valid][i_min]
            theta = angles[valid][i_min]
            zx = r * cos(theta)
            zy = r * sin(theta)
            self.z_lidar = np.array([[zx], [zy]])

    # =============================
    # Filtro EKF
    # =============================
    def ekf_update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        self.last_time = now

        # --- 1️⃣ Predicción ---
        θ = self.x[2, 0]
        v = self.x[3, 0]

        # Modelo cinemático diferencial (discreto)
        F = np.array([
            [1, 0, -v * sin(θ) * dt, cos(θ) * dt],
            [0, 1,  v * cos(θ) * dt, sin(θ) * dt],
            [0, 0,  1,               0],
            [0, 0,  0,               1]
        ])

        B = np.array([
            [0, 0],
            [0, 0],
            [dt, 0],
            [0, dt]
        ])

        u = np.array([[self.wz], [self.v]])
        self.x = self.x + np.array([
            [v * cos(θ) * dt],
            [v * sin(θ) * dt],
            [self.wz * dt],
            [0.0]
        ])

        self.P = F @ self.P @ F.T + self.Q

        # --- 2️⃣ Corrección (si hay medición LiDAR) ---
        if self.z_lidar is not None:
            H = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0]
            ])

            z_pred = H @ self.x
            y = self.z_lidar - z_pred                    # Innovación
            S = H @ self.P @ H.T + self.R                # Covarianza de innovación
            K = self.P @ H.T @ np.linalg.inv(S)          # Ganancia de Kalman
            self.x = self.x + K @ y                      # Actualización de estado
            self.P = (np.eye(4) - K @ H) @ self.P        # Actualización de covarianza

        # --- 3️⃣ Publicar estado ---
        msg = Float32MultiArray()
        msg.data = self.x.flatten().tolist()
        self.pub_state.publish(msg)

        # --- 4️⃣ Actualizar gráfica ---
        self.update_plot()

    def update_plot(self):
        self.traj.append(self.x[:2, 0].tolist())
        self.v_log.append(self.x[3, 0])

        if len(self.traj) > 2:
            traj_arr = np.array(self.traj)
            self.ax[0].cla()
            self.ax[0].plot(traj_arr[:, 0], traj_arr[:, 1], color='orange')
            self.ax[0].set_title("Trayectoria estimada (x,y)")
            self.ax[0].set_xlabel("X [m]")
            self.ax[0].set_ylabel("Y [m]")
            self.ax[0].axis("equal")

        self.ax[1].cla()
        self.ax[1].plot(self.v_log, color='steelblue')
        self.ax[1].set_title("Velocidad (m/s)")
        self.ax[1].set_xlabel("Muestras")
        self.ax[1].set_ylabel("v")

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    # =============================
    # Loop principal
    # =============================
    def spin_loop(self):
        rate = self.create_rate(30)
        while rclpy.ok():
            self.ekf_update()
            rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    node = FusionKalmanNode()
    try:
        node.spin_loop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
