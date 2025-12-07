#!/usr/bin/env python3
"""
Estimador de Pose para AMR
Fusiona información del encoder (velocidad) e IMU (yaw)
Publica pose como Vector3Stamped y muestra en terminal

Subscripciones:
    /amr/odom  (sm_interfaces/msg/Amr) - Datos unificados encoder + IMU

Publicaciones:
    /amr/pose  (Vector3Stamped) - Pose estimada [x, y, theta]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sm_interfaces.msg import Amr
import numpy as np
import math


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        
        # ═══════════════════════════════════════════════════════════
        # Estado inicial [x, y, theta]
        # ═══════════════════════════════════════════════════════════
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientación en radianes
        
        # Variables para integración
        self.last_time = None
        self.velocity = 0.0      # Velocidad lineal del encoder
        self.yaw_deg = 0.0       # Yaw en grados del IMU
        self.theta_imu = 0.0     # Theta en radianes
        
        # Calibración de theta
        self.theta_offset = None  # Se calibra con el primer valor recibido
        self.calibrated = False
        
        # ═══════════════════════════════════════════════════════════
        # Subscriber
        # ═══════════════════════════════════════════════════════════
        self.amr_sub = self.create_subscription(
            Amr,
            '/amr/odom',
            self.amr_callback,
            10
        )
        
        # ═══════════════════════════════════════════════════════════
        # Publisher
        # ═══════════════════════════════════════════════════════════
        self.pose_pub = self.create_publisher(
            Vector3Stamped,
            '/amr/pose',
            10
        )
        
        # Timer para actualizar e imprimir (20 Hz)
        self.timer = self.create_timer(0.05, self.update_pose)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('  POSE ESTIMATOR AMR INICIADO')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Subscrito a:')
        self.get_logger().info('  - /amr/odom (sm_interfaces/msg/Amr)')
        self.get_logger().info('Publicando en:')
        self.get_logger().info('  - /amr/pose (Vector3Stamped)')
        self.get_logger().info('=' * 50)

    def normalize_angle(self, angle):
        """Normaliza ángulo al rango [0, 2*pi]"""
        while angle >= 2 * math.pi:
            angle -= 2 * math.pi
        while angle < 0:
            angle += 2 * math.pi
        return angle

    def amr_callback(self, msg: Amr):
        """Callback del AMR - obtiene velocidad y yaw"""
        # Obtener velocidad del encoder
        self.velocity = msg.velocity
        
        # Obtener yaw en grados y convertir a radianes
        self.yaw_deg = msg.yaw
        raw_theta = math.radians(self.yaw_deg)
        
        # Calibrar con el primer valor recibido
        if not self.calibrated:
            self.theta_offset = raw_theta
            self.calibrated = True
            self.get_logger().info(f'✓ Theta calibrado! Offset: {self.yaw_deg:.2f}°')
        
        # Aplicar offset y normalizar al rango [0, 2π]
        self.theta_imu = self.normalize_angle(raw_theta - self.theta_offset)
    
    def update_pose(self):
        """Actualiza la pose integrando velocidad y yaw"""
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calcular dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Evitar dt muy grandes o negativos
        if dt <= 0 or dt > 1.0:
            return
        
        # ═══════════════════════════════════════════════════════════
        # Usar theta directo del IMU
        # ═══════════════════════════════════════════════════════════
        self.theta = self.theta_imu
        
        # Actualizar posición integrando velocidad
        self.x += self.velocity * math.cos(self.theta) * dt
        self.y += self.velocity * math.sin(self.theta) * dt
        
        # ═══════════════════════════════════════════════════════════
        # Publicar pose como Vector3Stamped
        # ═══════════════════════════════════════════════════════════
        pose_msg = Vector3Stamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = 'amr_odom'
        pose_msg.vector.x = self.x
        pose_msg.vector.y = self.y
        pose_msg.vector.z = self.theta  # Theta en radianes
        
        self.pose_pub.publish(pose_msg)
        
        # ═══════════════════════════════════════════════════════════
        # Mostrar en terminal
        # ═══════════════════════════════════════════════════════════
        theta_deg = math.degrees(self.theta)

        print(f'\rPosición: X={self.x:.3f} Y={self.y:.3f} | Orientación: {theta_deg:.1f}° | Vel: {self.velocity:.3f}m/s   ', end='')

def main(args=None):
    rclpy.init(args=args)
    
    node = PoseEstimator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\n' + '=' * 50)
        print('  POSE ESTIMATOR DETENIDO')
        print('=' * 50)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
