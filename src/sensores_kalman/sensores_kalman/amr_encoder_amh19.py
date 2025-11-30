import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import serial
import json

class EncoderNode(Node):
    def __init__(self):
        super().__init__('amr_encoder_amh19')

        # === ABRIR SERIAL CON PYSERIAL ===
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # === PUBLICADOR VECTOR3STAMPED ===
        self.pub = self.create_publisher(Vector3Stamped, '/amr/encoder', 10)

        # Timer a 50 ms
        self.timer = self.create_timer(0.05, self.leer_serial)

        self.get_logger().info("Nodo /amr/encoder (Vector3Stamped) iniciado leyendo AMH19 ESP32 Encoder")

    def leer_serial(self):
        try:
            line = self.ser.readline().decode().strip()

            if not line.startswith("{"):
                return

            data = json.loads(line)

            pulsos = float(data["pulsos"])
            dist   = float(data["dist"])
            vel    = float(data["vel"])

            msg = Vector3Stamped()

            # HEADER
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "encoder_frame_amh19"

            # VECTOR3
            msg.vector.x = dist      # metros recorridos
            msg.vector.y = vel       # m/s
            msg.vector.z = pulsos    # pulsos totales

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Error al leer serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
