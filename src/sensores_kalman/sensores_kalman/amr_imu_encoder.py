import rclpy
from rclpy.node import Node
from sm_interfaces.msg import Amr
import serial
import json

class EncoderNode(Node):
    def __init__(self):
        super().__init__('amr_odom')

        # === ABRIR SERIAL CON PYSERIAL ===
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # === PUBLICADOR FLOAT64MULTIARRAY ===
        self.pub = self.create_publisher(Amr, '/amr/odom', 10)

        # Timer a 50 ms
        self.timer = self.create_timer(0.05, self.leer_serial)

        self.get_logger().info("Nodo /amr/odom iniciado leyendo ESP32 Encoder")

    def leer_serial(self):
        try:
            line = self.ser.readline().decode().strip()

            if not line.startswith("{"):
                return

            data = json.loads(line)

            pulsos = int(data["pulsos"])
            dist   = float(data["dist"])
            vel    = float(data["vel"])
            ax     = float(data["ax"])
            ay     = float(data["ay"])
            az     = float(data["az"])
            roll   = float(data["roll"])
            pitch  = float(data["pitch"])
            yaw    = float(data["yaw"])

            msg = Amr()  
            msg.distance = dist
            msg.velocity = vel
            msg.pulses = pulsos
            msg.acceleration_x = ax
            msg.acceleration_y = ay
            msg.acceleration_z = az
            msg.roll = roll
            msg.pitch = pitch
            msg.yaw = yaw

            # HEADER
            # msg.header.stamp = self.get_clock().now().to_msg()
            # msg.header.frame_id = "odom_frame_amh19"

            # Float64MultiArray data
            # msg.data[0] = dist      # metros recorridos
            # msg.data[1] = vel       # m/s
            # msg.data[2] = pulsos    # pulsos totales
            # msg.data[3] = ax        # aceleración en x
            # msg.data[4] = ay        # aceleración en y
            # msg.data[5] = az        # aceleración en z
            # msg.data[6] = roll      # roll en grados
            # msg.data[7] = pitch     # pitch en grados
            # msg.data[8] = yaw       # yaw en grados

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
