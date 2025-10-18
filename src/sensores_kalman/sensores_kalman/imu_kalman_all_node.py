import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
#from mpl_toolkits.mplot3d import Axes3D


class KalmanFilter:
    def __init__(self, Q=1e-4, R=0.1):
        self.x = 0.0   # estimación
        self.P = 1.0   # incertidumbre
        self.Q = Q
        self.R = R

    def update(self, z: float) -> float:
        # Predicción
        self.P += self.Q
        # Ganancia de Kalman
        K = self.P / (self.P + self.R)
        # Corrección
        self.x = self.x + K * (z - self.x)
        # Actualizar incertidumbre
        self.P = (1 - K) * self.P
        return self.x


class ImuKalmanNode(Node):
    def __init__(self):
        super().__init__('imu_kalman_node')

        # ======= Variables para integración =======
        self.last_time = None
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.angles = np.zeros(3)  # roll, pitch, yaw
        self.traj = deque(maxlen=1000)
        self.pose_log = []  # ← aquí se inicializa una sola vez

        # ======= Parámetros =======
        self.declare_parameter('Q', 1e-4)
        self.declare_parameter('R', 0.1)
        Q = float(self.get_parameter('Q').value)
        R = float(self.get_parameter('R').value)

        # Un filtro independiente por variable IMU
        self.kf = {
            'accel_x': KalmanFilter(Q=Q, R=R),
            'accel_y': KalmanFilter(Q=Q, R=R),
            'accel_z': KalmanFilter(Q=Q, R=R),
            'roll': KalmanFilter(Q=Q, R=R),
            'pitch': KalmanFilter(Q=Q, R=R),
            'yaw': KalmanFilter(Q=Q, R=R)
        }

        # Buffers de datos por variable
        self.buffers_raw = {k: deque(maxlen=200) for k in self.kf}
        self.buffers_kf = {k: deque(maxlen=200) for k in self.kf}

        # ======= Suscripción =======
        self.create_subscription(Imu, '/qcar/imu', self.imu_cb, qos_profile_sensor_data)

        # ======= Gráficas 2D =======
        plt.ion()
        self.fig, self.ax = plt.subplots(2, 3, figsize=(10, 6))
        self.fig.suptitle("Filtro de Kalman - IMU del QCar")

        self.labels = ['accel_x', 'accel_y', 'accel_z', 'roll', 'pitch', 'yaw']
        self.lines_raw = {}
        self.lines_kf = {}

        for i, key in enumerate(self.labels):
            r = i // 3
            c = i % 3
            self.ax[r][c].set_title(key)
            self.ax[r][c].set_xlabel("Muestras")
            self.ax[r][c].set_ylabel("Valor")
            (line_raw,) = self.ax[r][c].plot([], [], label="Raw")
            (line_kf,) = self.ax[r][c].plot([], [], label="Kalman")
            self.lines_raw[key] = line_raw
            self.lines_kf[key] = line_kf
            self.ax[r][c].legend()

        # ======= Gráfica 3D =======
        #self.fig3d = plt.figure()
        #self.ax3d = self.fig3d.add_subplot(111, projection='3d')
        #self.ax3d.set_xlabel("X [m]")
        #self.ax3d.set_ylabel("Y [m]")
        #self.ax3d.set_zlabel("Z [m]")
        #self.ax3d.set_title("Trayectoria estimada (Kalman)")

        self.get_logger().info(f"IMU+Kalman listo. Q={Q}, R={R}, escuchando /qcar/imu")

    def imu_cb(self, msg: Imu):
        # ======= Filtrar lecturas =======
        data = {
            'accel_x': float(msg.linear_acceleration.x),
            'accel_y': float(msg.linear_acceleration.y),
            'accel_z': float(msg.linear_acceleration.z),
            'roll': float(msg.angular_velocity.x),
            'pitch': float(msg.angular_velocity.y),
            'yaw': float(msg.angular_velocity.z)
        }

        for key in data:
            val = data[key]
            val_kf = self.kf[key].update(val)
            self.buffers_raw[key].append(val)
            self.buffers_kf[key].append(val_kf)

        self.update_plot()

        # ======= Integrar en el tiempo =======
        current_time = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.last_time = current_time

        # Orientación
        self.angles[0] += self.kf['roll'].x * dt
        self.angles[1] += self.kf['pitch'].x * dt
        self.angles[2] += self.kf['yaw'].x * dt

        # Aceleraciones → posición
        accel = np.array([
            self.kf['accel_x'].x,
            self.kf['accel_y'].x,
            self.kf['accel_z'].x - 9.81
        ])
        self.vel += accel * dt
        self.pos += self.vel * dt

        # Registrar trayectoria
        self.traj.append(self.pos.copy())

        # ======= Guardar pose actual =======
        pose_vector = np.concatenate((self.pos, np.degrees(self.angles)))
        self.pose_log.append(pose_vector)

        # ======= Actualizar gráfica 3D =======
       # xyz = np.array(self.traj)
       # self.ax3d.cla()
       # self.ax3d.set_xlabel("X [m]")
       # self.ax3d.set_ylabel("Y [m]")
       # self.ax3d.set_zlabel("Z [m]")
       # self.ax3d.set_title("Trayectoria estimada (Kalman)")
       # if len(xyz) > 1:
       #     self.ax3d.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color='orange')
       # plt.pause(0.01)

        # ======= Log =======
        self.get_logger().info(f"POSE = {pose_vector.round(3)}")

    def update_plot(self):
        for i, key in enumerate(self.labels):
            r = i // 3
            c = i % 3
            x = np.arange(len(self.buffers_raw[key]))
            self.lines_raw[key].set_data(x, list(self.buffers_raw[key]))
            self.lines_kf[key].set_data(x, list(self.buffers_kf[key]))
            self.ax[r][c].relim()
            self.ax[r][c].autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def destroy_node(self):
        # ======= Guardar CSV al final =======
        if len(self.pose_log) > 0:
            np.savetxt('imu_pose_log.csv', np.array(self.pose_log),
                       header='x,y,z,roll,pitch,yaw(deg)', delimiter=',')
            self.get_logger().info("Archivo imu_pose_log.csv guardado correctamente ")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuKalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
