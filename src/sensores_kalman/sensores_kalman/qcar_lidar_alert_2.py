#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Parámetros
        self.distance_threshold = 0.35  # 35cm
        
        # Conos de visión según velocidad
        self.angle_range_low = 22.5   # grados para velocidad 0-1 m/s
        self.angle_range_high = 30.0  # grados para velocidad > 1 m/s
        self.velocity_threshold = 1.0  # m/s
        
        # Ángulo actual (se actualiza según velocidad)
        self.angle_range = self.angle_range_low
        
        # Velocidad actual (componente X)
        self.current_velocity_x = 0.0
        
        # OFFSET del LiDAR respecto al frente real del QCar
        self.front_angle_offset = 4.71  # radianes
        
        # Debug mode
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
        
        # Subscriber a la velocidad del QCar
        self.velocity_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/velocity',
            self.velocity_callback,
            10
        )
        
        self.get_logger().info(f'Obstacle detector iniciado')
        self.get_logger().info(f'  - Threshold: {self.distance_threshold}m')
        self.get_logger().info(f'  - Angle range (vel <= {self.velocity_threshold}): ±{self.angle_range_low}°')
        self.get_logger().info(f'  - Angle range (vel > {self.velocity_threshold}): ±{self.angle_range_high}°')
        self.get_logger().info(f'  - Front offset: {np.degrees(self.front_angle_offset):.1f}°')
        
        if self.debug_mode:
            self.get_logger().info(f'  - DEBUG MODE: ON')

    def velocity_callback(self, msg):
        """Callback para actualizar la velocidad y ajustar el cono de visión"""
        self.current_velocity_x = abs(msg.vector.x)  # Valor absoluto por si va en reversa
        
        # Ajustar el cono de visión según la velocidad
        if self.current_velocity_x > self.velocity_threshold:
            self.angle_range = self.angle_range_high
        else:
            self.angle_range = self.angle_range_low
        
        if self.debug_mode:
            self.get_logger().info(
                f'Velocidad X: {self.current_velocity_x:.2f} m/s -> Cono: ±{self.angle_range}°'
            )

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
        range_indices = int((self.angle_range * np.pi / 180) / angle_increment)
        
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
            self.get_logger().warn(
                f'¡OBSTÁCULO DETECTADO a {min_distance:.2f}m! (vel: {self.current_velocity_x:.2f} m/s, cono: ±{self.angle_range}°)'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()