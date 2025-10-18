import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt


class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener_node')

        # ==========================
        #  CONFIGURACIÓN DE QoS
        # ==========================
        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # ==========================
        #  SUSCRIPCIÓN AL LIDAR
        # ==========================
        self.create_subscription(
            LaserScan,
            '/qcar/scan',
            self.lidar_callback,
            qos_profile=qos_lidar
        )

        # ==========================
        #  TIMER DE 10 Hz (SYNC)
        # ==========================
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.latest_scan = None  # Se actualizará con cada mensaje nuevo

        # ==========================
        #  CONFIGURACIÓN DE GRÁFICA
        # ==========================
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_title("Vista LiDAR QCar (10 Hz)", va='bottom')
        self.ax.set_rmax(12.0)
        self.scatter = None

        self.get_logger().info("📡 Nodo LidarListener activo: escuchando /qcar/scan a 10 Hz")

    # -----------------------------------------------------------
    # CALLBACK del tópico: guarda el último escaneo recibido
    # -----------------------------------------------------------
    def lidar_callback(self, msg: LaserScan):
        self.latest_scan = msg

    # -----------------------------------------------------------
    # TIMER: dibuja a 10 Hz el último mensaje recibido
    # -----------------------------------------------------------
    def timer_callback(self):
        if self.latest_scan is not None:
            self.plot_scan(self.latest_scan)

    # -----------------------------------------------------------
    # DIBUJO POLAR (Radar)
    # -----------------------------------------------------------
    def plot_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=float)
        valid = np.isfinite(ranges)

        if not np.any(valid):
            self.get_logger().warn("❌ LiDAR sin datos válidos")
            return

        # Ángulos y distancias
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        ranges = np.clip(ranges, msg.range_min, msg.range_max)

        # Redibujo
        self.ax.clear()
        self.ax.scatter(angles[valid], ranges[valid], s=5, c='orange')
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_rmax(msg.range_max)
        self.ax.set_title("Vista LiDAR QCar (10 Hz)", va='bottom')
        plt.pause(0.001)

        # Métricas básicas
        d_min = np.min(ranges[valid])
        d_mean = np.mean(ranges[valid])
        self.get_logger().info(f"📏 Distancia mínima = {d_min:.2f} m | Promedio = {d_mean:.2f} m")


# -----------------------------------------------------------
# MAIN
# -----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LidarListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
