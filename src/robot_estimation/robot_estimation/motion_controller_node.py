import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller_node') 
        
        # Robot parameters
        self.declare_parameter('wheel_radius', 0.1)       
        self.declare_parameter('wheel_separation', 0.7)   
        
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.left_rpm_pub = self.create_publisher(
            Float64,
            'left_wheel_rpm', 
            10
        )
        self.right_rpm_pub = self.create_publisher(
            Float64,
            'right_wheel_rpm', 
            10
        )
        
        self.get_logger().info('Motion Controller Node has started and is listening to /cmd_vel.')
    
    def cmd_vel_callback(self, msg):
        
        linear_vel = msg.linear.x   # v (m/s)
        angular_vel = msg.angular.z # omega (rad/s)
        
        
        omega_left_wheel = (2.0 * linear_vel - angular_vel * self.L) / (2.0 * self.R)
        omega_right_wheel = (2.0 * linear_vel + angular_vel * self.L) / (2.0 * self.R)
        
        left_rpm = (omega_left_wheel * 60.0) / (2.0 * math.pi)
        right_rpm = (omega_right_wheel * 60.0) / (2.0 * math.pi)
        
        left_msg = Float64(data=left_rpm)
        right_msg = Float64(data=right_rpm)
        
        self.left_rpm_pub.publish(left_msg)
        self.right_rpm_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()