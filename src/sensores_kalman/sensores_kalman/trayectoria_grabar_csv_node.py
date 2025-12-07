# Nodo ROS2 para grabar la trayectoria del Qcar o AMR en un CSV
# Suscribe a un tópico de tipo Vector3Stamped (x,y,theta) y guarda los puntos en un archivo CSV al finalizar la ejecución.
# Qcar -> /qcar/pose
# AMR  -> /amr/pose

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

import csv, math, os
from datetime import datetime
import atexit  

def dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        # Parámetros
        self.declare_parameter('topic', '/amr/pose') # tópico  para AMR
        #self.declare_parameter('topic', '/qcar/pose') # tópico para Qcar
        #self.declare_parameter('min_spacing', 0.2)  # m
        self.declare_parameter('outfile', '')       # por defecto: cwd/waypoints_YYYYMMDD_HHMMSS.csv

        self.points = []

        topic = self.get_parameter('topic').value
        #self.min_spacing = float(self.get_parameter('min_spacing').value)
        self.outfile = self.get_parameter('outfile').value

        if not self.outfile:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.outfile = os.path.join(os.getcwd(), f'waypoints{stamp}.csv')

        # Suscripción a (Vector3Stamped)
        self.create_subscription(Vector3Stamped, topic, self.pose_cb, 10)

        # Guardar al salir
        atexit.register(self.save_points)

        self.get_logger().info(f'Grabando {topic} -> {self.outfile}')

    def pose_cb(self, msg: Vector3Stamped):
        x = msg.vector.x
        y = msg.vector.y
        theta = msg.vector.z  # en radianes normalmente

        #if (not self.points) or dist(self.points[-1][:2], (x, y)) >= self.min_spacing:
        self.points.append((x, y, theta))

    def save_points(self):
        try:
            with open(self.outfile, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['x', 'y', 'theta'])
                w.writerows(self.points)
            self.get_logger().info(
                f'Guardado {len(self.points)} puntos en: {self.outfile}'
            )
        except Exception as e:
            self.get_logger().error(f'Error guardando CSV: {e}')

def main():
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_points()  # extra seguridad
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
