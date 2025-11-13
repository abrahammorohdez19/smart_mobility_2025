#!/usr/bin/env python3
"""
Nodo unificado que calcula orientación, trayectoria y publica la pose completa del QCar
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import math

class QCarPose(Node):
    def __init__(self):
        super().__init__('qcar_pose')
        
        # Parámetros
        self.declare_parameter('wheelbase', 0.256)  # m
        self.declare_parameter('max_steering_angle', 0.3)  # rad
        self.declare_parameter('velocity_threshold', 0.000000000001)  # m/s
        
        self.L = self.get_parameter('wheelbase').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        
        # Estado completo
        self.x = 0.0  # Posición X (m)
        self.y = 0.0  # Posición Y (m)
        self.theta = 0.0  # Orientación (rad)
        
        self.vx = 0.0  # Velocidad en X (m/s)
        self.vy = 0.0  # Velocidad en Y (m/s)
        self.velocity = 0.0  # Magnitud de velocidad (m/s)
        self.steering = 0.0  # Ángulo de dirección (rad)
        
        # Tiempo
        self.last_time = self.get_clock().now()
        
        # Subscribers
        self.velocity_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/velocity',
            self.velocity_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/user_command',
            self.command_callback,
            10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(
            Vector3Stamped,
            '/qcar/pose',
            10
        )
        
        self.orientation_pub = self.create_publisher(
            Vector3Stamped,
            '/qcar/orientation',
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            Vector3Stamped,
            '/qcar/trajectory',
            10
        )
        
        # Timer para integrar
        self.timer = self.create_timer(0.02, self.integrate)  # 50 Hz
        
        self.get_logger().info('Nodo unificado QCAR Pose iniciado')
        self.get_logger().info(f'Wheelbase: {self.L} m')
        self.get_logger().info(f'Max steering: {math.degrees(self.max_steering):.1f}°')
        self.get_logger().info(f'Velocity threshold: {self.velocity_threshold} m/s')
        self.get_logger().info('Posición inicial: (0, 0, 0°)')
        
    def velocity_callback(self, msg):
        # Guardar componentes de velocidad
        self.vx = msg.vector.x
        self.vy = msg.vector.y
        
        # Calcular magnitud
        raw_velocity = math.sqrt(self.vx**2 + self.vy**2)
        
        # Aplicar threshold (ignorar ruido)
        if abs(raw_velocity) < self.velocity_threshold:
            self.velocity = 0.0
        else:
            self.velocity = raw_velocity
        
    def command_callback(self, msg):
        # Obtener steering del comando
        self.steering = msg.vector.y
        
    def integrate(self):
        """Integrar modelo de bicicleta para orientación y posición"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Limitar dt para evitar saltos grandes
        dt = max(0.001, min(dt, 0.1))
        
        # 1. Integrar orientación (modelo de bicicleta)
        omega = (self.velocity / self.L) * math.sin(self.steering)
        self.theta += omega * dt
        
        # 2. Integrar posición (componentes directas de velocidad)
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # 3. Publicar orientación
        orientation_msg = Vector3Stamped()
        orientation_msg.header.stamp = current_time.to_msg()
        orientation_msg.header.frame_id = 'base_link'
        orientation_msg.vector.x = 0.0
        orientation_msg.vector.y = 0.0
        orientation_msg.vector.z = self.theta
        self.orientation_pub.publish(orientation_msg)
        
        # 4. Publicar trayectoria
        trajectory_msg = Vector3Stamped()
        trajectory_msg.header.stamp = current_time.to_msg()
        trajectory_msg.header.frame_id = 'odom'
        trajectory_msg.vector.x = self.x
        trajectory_msg.vector.y = self.y
        trajectory_msg.vector.z = self.theta
        self.trajectory_pub.publish(trajectory_msg)
        
        # 5. Publicar pose completa (x, y, theta)
        pose_msg = Vector3Stamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.vector.x = self.x
        pose_msg.vector.y = self.y
        pose_msg.vector.z = self.theta
        self.pose_pub.publish(pose_msg)
        
        # 6. Log completo en terminal
        if int(current_time.nanoseconds / 1e9) % 1 == 0 and dt < 0.1:
            theta_deg = math.degrees(self.theta)
            steering_deg = math.degrees(self.steering)
            self.get_logger().info(
                f'Pos: X={self.x:+.3f}m Y={self.y:+.3f}m | '
                f'θ = {self.theta:.3f}rad | Degree = {theta_deg:+.2f}° | '
                f'v = {self.velocity:.10f} m/s | δ = {steering_deg:+.2f}°'
            )
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = QCarPose()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n{"="*60}')
        print(f'Pose final:')
        print(f'  X = {node.x:+.3f} m')
        print(f'  Y = {node.y:+.3f} m')
        print(f'  θ = {node.theta:.3f} rad = {math.degrees(node.theta):+.2f}°')
        print(f'  Distancia recorrida: {math.sqrt(node.x**2 + node.y**2):.3f} m')
        print(f'{"="*60}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()