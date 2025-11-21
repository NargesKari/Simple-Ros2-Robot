import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped 
import math


class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__('complementary_filter_node') 
        
        # Parameters
        self.declare_parameter('complementary_alpha', 0.98) 
        self.alpha = self.get_parameter('complementary_alpha').value
        
        # Subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/filtered', 
            self.imu_callback,
            10
        )
        
        # Publisher
        self.orientation_pub = self.create_publisher(
            QuaternionStamped,
            'estimation/orientation', 
            10
        )
        
        # State variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = None
        
        self.get_logger().info('Complementary Filter Node started.')
    
    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calculate dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0 or dt > 1.0: 
            return
        
        # Get filtered angular velocity (rad/s)
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z
        
        # Get filtered linear acceleration
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z
        
        # ۱. Gyroscope integration (Predict)
        gyro_roll = self.roll + gyro_x * dt
        gyro_pitch = self.pitch + gyro_y * dt
        
        # ۲. Accelerometer angles (Measure)
        # Note: Roll and Pitch derived from gravity vector
        accel_roll = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
        accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
        
        # ۳. Complementary filter (Fusion)
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        
        # Yaw: فقط از ژیروسکوپ استفاده می شود (Yaw correction نیاز به سنسور دیگری مانند مگنتومتر دارد)
        self.yaw += gyro_z * dt
        
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        quat = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        
        orientation_msg = QuaternionStamped()
        orientation_msg.header.stamp = self.get_clock().now().to_msg()
        orientation_msg.header.frame_id = 'base_link' 
        
        orientation_msg.quaternion.x = quat[0]
        orientation_msg.quaternion.y = quat[1]
        orientation_msg.quaternion.z = quat[2]
        orientation_msg.quaternion.w = quat[3]
        
        self.orientation_pub.publish(orientation_msg)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion (x, y, z, w)"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    node = ComplementaryFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()