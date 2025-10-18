import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3Stamped
import matplotlib.pyplot as plt
from collections import deque
import numpy as np


class KalmanFilter:
    def __init__(self, Q=1e-4, R=0.05):
        self.x = 0.0
        self.P = 1.0
        self.Q = Q
        self.R = R

    def update(self, z):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.x += K * (z - self.x)
        self.P *= (1 - K)
        return self.x


class VelocityKalmanNode(Node):
    def __init__(self):
        super().__init__('velocity_kalman_node')

        # ====== Filtro Kalman ======
        self.kf = KalmanFilter()

        # ====== Subscripción ======
        self.sub = self.create_subscription(Vector3Stamped, '/qcar/velocity', self.cb, qos_profile_sensor_data)
        self.get_logger().info("Nodo KalmanVelocity activo — escuchando /qcar/velocity")

        # ====== Buffers para graficar ======
        self.buffer_raw = deque(maxlen=200)
        self.buffer_kf = deque(maxlen=200)

        # ====== Configuración de gráfica ======
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 5))
        self.line_raw, = self.ax.plot([], [], label='Velocidad Raw', color='lightblue', alpha=0.8)
        self.line_kf, = self.ax.plot([], [], label='Velocidad Kalman', color='orange', linewidth=2)
        self.ax.set_xlabel("Muestras")
        self.ax.set_ylabel("Velocidad lineal (m/s)")
        self.ax.set_title("Filtro de Kalman - Velocidad QCar (Encoder magnético)")
        self.ax.legend()
        self.ax.grid(True)

    def cb(self, msg):
        vel_raw = msg.vector.x
        vel_kf = self.kf.update(vel_raw)

        # Guardar en buffers
        self.buffer_raw.append(vel_raw)
        self.buffer_kf.append(vel_kf)

        # Actualizar gráfica
        self.update_plot()

        # Log de consola
        self.get_logger().info(f"vel_raw={vel_raw:.3f} | vel_kf={vel_kf:.3f}")

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
    node = VelocityKalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
