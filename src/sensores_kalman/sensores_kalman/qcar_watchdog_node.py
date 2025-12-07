# Nodo watchdog para asegurar STOP en el QCar si se dejan de publicar comandos

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import math


class QCarWatchdog(Node):
    def __init__(self):
        super().__init__('qcar_watchdog')

        # Timeout configurable
        self.declare_parameter('timeout', 0.2)  # segundos sin comandos "reales" => STOP
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Subscripción y publicación al mismo tópico
        self.sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/user_command',
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            Vector3Stamped,
            '/qcar/user_command',
            10
        )

        # Tiempo del último comando "real"
        self.last_real_cmd_time = self.get_clock().now()

        # Timer del watchdog (20 Hz)
        self.timer = self.create_timer(0.05, self.watchdog_cb)

        self.get_logger().info(
            f"QCarWatchdog iniciado. Timeout = {self.timeout:.3f} s"
        )

    def cmd_callback(self, msg: Vector3Stamped):
        """
        Recibe TODOS los /qcar/user_command.
        - Si vienen del controlador (frame_id != 'watchdog'): actualiza last_real_cmd_time.
        - Si vienen del propio watchdog (frame_id == 'watchdog'): los ignora para el timeout.
        """
        if msg.header.frame_id != 'watchdog':
            # Comando real del controlador
            self.last_real_cmd_time = self.get_clock().now()

    def watchdog_cb(self):
        """
        Si pasa más tiempo del timeout sin comandos reales, manda STOP.
        """
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_real_cmd_time.nanoseconds) * 1e-9

        if dt > self.timeout:
            # Publicar STOP solo si no lo hemos hecho recientemente
            stop = Vector3Stamped()
            stop.header.stamp = now.to_msg()
            stop.header.frame_id = 'watchdog' 

            stop.vector.x = 0.0
            stop.vector.y = 0.0
            stop.vector.z = 0.0

            try:
                self.pub.publish(stop)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = QCarWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
