import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class KalmanFilter:
    def __init__(self, Q=1e-4, R=0.1):
        self.x = 0.0   # estimación
        self.P = 1.0   # incertidumbre
        self.Q = Q
        self.R = R

    def update(self, z: float) -> float:
        self.P = self.P + self.Q
        K = self.P / (self.P + self.R)
        self.x = self.x + K * (z - self.x)
        self.P = (1.0 - K) * self.P
        return self.x

class ImuKalmanNode(Node):
    def __init__(self):
        super().__init__('imu_kalman_node')

        # Parámetros
        self.declare_parameter('Q', 1e-4)
        self.declare_parameter('R', 0.1)
        Q = float(self.get_parameter('Q').value)
        R = float(self.get_parameter('R').value)
        self.kf = KalmanFilter(Q=Q, R=R)

        # Suscripción al IMU
        self.create_subscription(Imu, '/qcar/imu', self.imu_cb, qos_profile_sensor_data)

        # Buffers para graficar
        self.buffer_raw = deque(maxlen=200)
        self.buffer_kf = deque(maxlen=200)

        # Configurar gráfica
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line_raw, = self.ax.plot([], [], label="IMU raw accel_x")
        self.line_kf, = self.ax.plot([], [], label="IMU Kalman accel_x")
        self.ax.legend()
        self.ax.set_xlabel("Muestras")
        self.ax.set_ylabel("Aceleración X (m/s^2)")
        self.ax.set_title("Filtro de Kalman sobre IMU")

        self.get_logger().info(f"IMU+Kalman listo. Q={Q}, R={R}, escuchando /qcar/imu")

    def imu_cb(self, msg: Imu):
        # Tomamos la aceleración en X
        accel_x = float(msg.linear_acceleration.x)
        accel_x_kf = self.kf.update(accel_x)

        # Guardamos datos
        self.buffer_raw.append(accel_x)
        self.buffer_kf.append(accel_x_kf)

        # Actualizamos gráfica
        self.update_plot()

    def update_plot(self):
        x = np.arange(len(self.buffer_raw))
        self.line_raw.set_data(x, list(self.buffer_raw))
        self.line_kf.set_data(x, list(self.buffer_kf))
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = ImuKalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
