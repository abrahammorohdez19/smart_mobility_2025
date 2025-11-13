import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

class QCarPosePublisher(Node):
    def __init__(self):
        super().__init__('qcar_pose_publisher')
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Suscriptores
        self.trajectory_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/trajectory',
            self.trajectory_callback,
            10
        )
        
        self.orientation_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/orientation',
            self.orientation_callback,
            10
        )
        
        # Publicador de la pose completa
        self.pose_pub = self.create_publisher(
            Vector3Stamped,
            '/qcar/pose',
            10
        )
        
    def trajectory_callback(self, msg):
        self.x = msg.vector.x
        self.y = msg.vector.y
        self.publish_pose(msg.header)
        
    def orientation_callback(self, msg):
        self.theta = msg.vector.z
        self.publish_pose(msg.header)
        
    def publish_pose(self, header):
        pose_msg = Vector3Stamped()
        pose_msg.header = header
        pose_msg.vector.x = self.x
        pose_msg.vector.y = self.y
        pose_msg.vector.z = self.theta
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QCarPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()