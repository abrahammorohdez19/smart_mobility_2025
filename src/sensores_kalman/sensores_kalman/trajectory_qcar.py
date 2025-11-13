#!/usr/bin/env python3
"""
Nodo que calcula la trayectoria del QCar (posición X, Y)
integrando directamente las componentes de velocidad
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
import math

class QCarTrajectory(Node):
    def __init__(self):
        super().__init__('qcar_trajectory')
        
        # Estado de posición
        self.x = 0.0  # Posición en X (m)
        self.y = 0.0  # Posición en Y (m)
        
        # Variables para integración
        self.theta = 0.0  # Orientación actual (rad)
        self.vx = 0.0  # Velocidad en X (m/s)
        self.vy = 0.0  # Velocidad en Y (m/s)
        self.velocity = 0.0  # Magnitud de velocidad (m/s)
        
        # Tiempo
        self.last_time = self.get_clock().now()
        
        # Subscribers
        self.orientation_sub = self.create_subscription(
            Vector3Stamped,  # ← CAMBIADO de Float64 a Vector3Stamped
            '/qcar/orientation',
            self.orientation_callback,
            10
        )
        
        self.velocity_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/velocity',
            self.velocity_callback,
            10
        )
        
        # Publisher
        self.trajectory_pub = self.create_publisher(
            Vector3Stamped,
            '/qcar/trajectory',
            10
        )
        
        # Timer para integrar posición
        self.timer = self.create_timer(0.02, self.integrate_position)  # 50 Hz
        
        self.get_logger().info('Nodo de trayectoria QCAR iniciado')
        self.get_logger().info('Posición inicial: (0, 0)')
        self.get_logger().info('Método: Integración directa de componentes de velocidad')
    
    def orientation_callback(self, msg):
        """Actualizar orientación desde /qcar/orientation"""
        self.theta = msg.vector.z  # ← CAMBIADO de msg.data a msg.vector.z
    
    def velocity_callback(self, msg):
        """Actualizar velocidad desde /qcar/velocity"""
        # Guardar componentes de velocidad
        self.vx = msg.vector.x
        self.vy = msg.vector.y
        
        # Calcular magnitud para el log
        self.velocity = math.sqrt(self.vx**2 + self.vy**2)
    
    def integrate_position(self):
        """Integrar posición directamente con las componentes de velocidad"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Limitar dt para evitar saltos grandes
        dt = max(0.001, min(dt, 0.1))
        
        # Integración directa - los signos ya vienen en vx y vy
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # Crear y publicar mensaje
        msg = Vector3Stamped()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'odom'
        msg.vector.x = self.x
        msg.vector.y = self.y
        msg.vector.z = self.theta  # Guardamos theta por si lo necesitas
        self.trajectory_pub.publish(msg)
        
        # Log cada 1 segundo
        if int(current_time.nanoseconds / 1e9) % 1 == 0 and dt < 0.1:
            self.get_logger().info(
                f'Pos: X={self.x:+7.3f}m Y={self.y:+7.3f}m | '
                #f'Vel: vx={self.vx:+6.3f} vy={self.vy:+6.3f} |v|={self.velocity:.3f} | '
                f'θ={math.degrees(self.theta):+7.2f}°'
            )
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = QCarTrajectory()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n{"="*60}')
        print(f'Posición final: X = {node.x:+.3f} m, Y = {node.y:+.3f} m')
        print(f'Distancia recorrida: {math.sqrt(node.x**2 + node.y**2):.3f} m')
        print(f'{"="*60}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()