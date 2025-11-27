#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Parámetros
        self.distance_threshold = 0.35  # 35cm
        self.angle_range = 22.5  # grados al frente (+-30°)
        
        # OFFSET del LiDAR respecto al frente real del QCar
        # Ajusta este valor según tu configuración
        # -1.5708 = -90° (si el frente real está a la derecha del LiDAR)
        # +1.5708 = +90° (si el frente real está a la izquierda del LiDAR)
        self.front_angle_offset = 4.71  # -90 grados en radianes
        
        # Debug mode: True para encontrar el offset correcto
        self.debug_mode = True
        
        # Publisher para la alerta
        self.alert_pub = self.create_publisher(Bool, '/qcar/obstacle_alert', 10)
        
        # Subscriber al LiDAR del QCar
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/qcar/scan',
            self.lidar_callback,
            10
        )
        
        self.get_logger().info(f'Obstacle detector iniciado')
        self.get_logger().info(f'  - Threshold: {self.distance_threshold}m')
        self.get_logger().info(f'  - Angle range: ±{self.angle_range}°')
        self.get_logger().info(f'  - Front offset: {np.degrees(self.front_angle_offset):.1f}°')
        if self.debug_mode:
            self.get_logger().info(f'  - DEBUG MODE: ON (pon obstáculo al frente para calibrar)')
    
    def lidar_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = np.array(msg.ranges)
        
        # DEBUG: Encontrar dónde está el obstáculo más cercano
        if self.debug_mode:
            min_dist = float('inf')
            min_angle = 0
            for i, d in enumerate(ranges):
                if msg.range_min < d < min_dist:
                    min_dist = d
                    min_angle = angle_min + i * angle_increment
            self.get_logger().info(
                f'Obstáculo más cercano: {min_dist:.2f}m a {min_angle:.3f} rad ({np.degrees(min_angle):.1f}°)'
            )
        
        # Índices para el rango frontal CORREGIDO con offset
        center_index = int((self.front_angle_offset - angle_min) / angle_increment)
        range_indices = int((self.angle_range * 3.14159 / 180) / angle_increment)
        
        start_idx = max(0, center_index - range_indices)
        end_idx = min(len(ranges) - 1, center_index + range_indices)
        
        # Buscar obstáculo en el rango frontal
        obstacle_detected = False
        min_distance = float('inf')
        
        for i in range(start_idx, end_idx + 1):
            distance = ranges[i]
            if msg.range_min < distance < self.distance_threshold:
                obstacle_detected = True
                min_distance = min(min_distance, distance)
        
        # Publicar alerta
        alert_msg = Bool()
        alert_msg.data = obstacle_detected
        self.alert_pub.publish(alert_msg)
        
        if obstacle_detected:
            self.get_logger().warn(f' ¡OBSTÁCULO DETECTADO a {min_distance:.2f}m!')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()