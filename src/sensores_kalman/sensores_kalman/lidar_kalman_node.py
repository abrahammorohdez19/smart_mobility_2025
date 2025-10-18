import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
import time

class EKFFusion(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')

        # --- Estado inicial ---
        self.x = np.zeros((4, 1))  # [x, y, theta, v]
        self.P = np.eye(4) * 0.1
        self.last_time = time.time()

        # --- Ruido ---
        self.Q = np.diag([0.01, 0.01, 0.02, 0.05])
        self.R_imu = np.array([[0.02]])
        self.R_vel = np.array([[0.05]])
        self.R_lidar = np.diag([0.1, 0.1])

        # --- QoS (compatible con Dashing) ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Suscriptores ---
        self.create_subscription(Imu, '/qcar/imu', self.imu_cb, qos)
        self.create_subscription(LaserScan, '/qcar/scan', self.lidar_cb, qos)
        self.create_subscription(Vector3Stamped, '/qcar/velocity', self.vel_cb, qos)

        self.get_logger().info("EKF Fusion Node activo — IMU + LiDAR + Encoder")

    # ==============================================================
    def predict(self, omega, dt):
        x, y, theta, v = self.x.flatten()

        # Modelo de movimiento (cinemática diferencial)
        x_pred = x + v * math.cos(theta) * dt
        y_pred = y + v * math.sin(theta) * dt
        theta_pred = theta + omega * dt
        v_pred = v

        self.x = np.array([[x_pred], [y_pred], [theta_pred], [v_pred]])

        # Jacobiano F
        F = np.eye(4)
        F[0, 2] = -v * math.sin(theta) * dt
        F[0, 3] = math.cos(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt
        F[1, 3] = math.sin(theta) * dt

        self.P = F @ self.P @ F.T + self.Q

    # ==============================================================
    def update(self, z, H, R):
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P

    # ==============================================================
    def imu_cb(self, msg):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        omega = msg.angular_velocity.z
        self.predict(omega, dt)

    # ==============================================================
    def vel_cb(self, msg):
        v = msg.vector.x   # ✅ Así viene en el QCar físico
        H = np.array([[0, 0, 0, 1]])
        z = np.array([[v]])
        self.update(z, H, self.R_vel)

    # ==============================================================
    def lidar_cb(self, msg):
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges)
        if not np.any(valid):
            return

        # Ángulos
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # 🔧 Frente real del QCar ≈ -90° (ajustable)
        front_angle = math.radians(-90)
        front_idx = np.argmin(np.abs(angles - front_angle))
        r = ranges[front_idx]

        if not np.isfinite(r):
            return

        # Coordenadas relativas
        x_m = r * math.cos(angles[front_idx])
        y_m = r * math.sin(angles[front_idx])

        z = np.array([[x_m], [y_m]])
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])

        self.update(z, H, self.R_lidar)

        self.get_logger().info(
            f"x={self.x[0,0]:.2f}, y={self.x[1,0]:.2f}, θ={math.degrees(self.x[2,0]):.1f}°, v={self.x[3,0]:.2f}"
        )


# ==============================================================
def main(args=None):
    rclpy.init(args=args)
    node = EKFFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
