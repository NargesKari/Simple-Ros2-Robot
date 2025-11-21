import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')

        self.declare_parameter('complementary_alpha', 0.98) 
        self.alpha = self.get_parameter('complementary_alpha').value
        
        self.roll = 0.0
        self.pitch = 0.0
        self.last_time = self.get_clock().now()
        
        self.imu_sub = self.create_subscription(Imu, 'imu/filtered', self.imu_callback, 10)
        
        self.orientation_pub = self.create_publisher(Quaternion, 'estimation/orientation', 10)
        
        self.get_logger().info('Complementary Filter Node started.')

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0.0:
            return

        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z
        
        if accel_z == 0:
            accel_z = 1e-6 
            
        roll_acc = math.atan2(accel_y, accel_z)
        pitch_acc = math.atan2(-accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z))
        
        roll_dot = msg.angular_velocity.x
        pitch_dot = msg.angular_velocity.y
        
        roll_gyro = self.roll + roll_dot * dt
        pitch_gyro = self.pitch + pitch_dot * dt

        self.roll = self.alpha * roll_gyro + (1 - self.alpha) * roll_acc
        self.pitch = self.alpha * pitch_gyro + (1 - self.alpha) * pitch_acc
        
        self.last_time = current_time

        q = self.euler_to_quaternion(self.roll, self.pitch, 0.0)
        
        quat_msg = Quaternion()
        quat_msg.x = q[0]
        quat_msg.y = q[1]
        quat_msg.z = q[2]
        quat_msg.w = q[3]

        self.orientation_pub.publish(quat_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = ComplementaryFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()