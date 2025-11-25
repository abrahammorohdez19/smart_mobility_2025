#!/usr/bin/env python3
"""
Estimador de Pose para QCar
Fusiona información del encoder (velocidad) e IMU (yaw rate)
Publica pose como Vector3Stamped y muestra en terminal

Subscripciones:
    /qcar/velocity  (Vector3Stamped) - Velocidad del encoder
    /imu/accel_raw  (TwistStamped)   - Datos IMU con yaw rate en angular.z

Publicaciones:
    /qcar/pose      (Vector3Stamped) - Pose estimada [x, y, theta]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, TwistStamped
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
        self.theta_imu = 0.0     # Theta directo del IMU (radianes)
        
        # Calibración de theta
        self.theta_offset = None  # Se calibra con el primer valor recibido
        self.calibrated = False
        
        # ═══════════════════════════════════════════════════════════
        # Subscribers
        # ═══════════════════════════════════════════════════════════
        self.vel_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/velocity',
            self.velocity_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            TwistStamped,
            '/imu/accel_raw',
            self.imu_callback,
            10
        )
        
        # ═══════════════════════════════════════════════════════════
        # Publisher
        # ═══════════════════════════════════════════════════════════
        self.pose_pub = self.create_publisher(
            Vector3Stamped,
            '/qcar/pose',
            10
        )
        
        # Timer para actualizar e imprimir (20 Hz)
        self.timer = self.create_timer(0.05, self.update_pose)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('  POSE ESTIMATOR INICIADO')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Subscrito a:')
        self.get_logger().info('  - /qcar/velocity (Vector3Stamped)')
        self.get_logger().info('  - /imu/accel_raw (TwistStamped)')
        self.get_logger().info('Publicando en:')
        self.get_logger().info('  - /qcar/pose (Vector3Stamped)')
        self.get_logger().info('=' * 50)

    def velocity_callback(self, msg: Vector3Stamped):
        """Callback del encoder - obtiene velocidad lineal"""
        # Asumiendo que vector.x es la velocidad lineal del vehículo
        self.velocity = msg.vector.x
    
    def imu_callback(self, msg: TwistStamped):
        """Callback del IMU - obtiene theta directamente (en radianes)"""
        raw_theta = msg.twist.angular.z
        
        # Calibrar con el primer valor recibido
        if not self.calibrated:
            self.theta_offset = raw_theta
            self.calibrated = True
            self.get_logger().info(f'✓ Theta calibrado! Offset: {math.degrees(self.theta_offset):.2f}°')
        
        # Aplicar offset para que inicie en 0
        self.theta_imu = raw_theta - self.theta_offset
    
    def update_pose(self):
        """Actualiza la pose integrando velocidad y yaw rate"""
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
        pose_msg.header.frame_id = 'odom'
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
