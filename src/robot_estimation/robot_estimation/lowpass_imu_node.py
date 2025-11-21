import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class LowpassImuFilter(Node):
    def __init__(self):
        super().__init__('lowpass_imu_filter_node')
        
        # ۱. پارامتر فیلتر Lowpass (آلفا)
        # آلفای کوچکتر = فیلترینگ قوی‌تر (پاسخ کندتر)
        self.declare_parameter('lowpass_alpha', 0.8) 
        self.alpha = self.get_parameter('lowpass_alpha').value
        
        # ۲. متغیرهای وضعیت (مقادیر فیلتر شده قبلی)
        self.filtered_accel = [0.0, 0.0, 0.0]  # [x, y, z]
        self.filtered_gyro = [0.0, 0.0, 0.0]   # [x, y, z]
        self.gyro_bias = [0.0, 0.0, 0.0]       # بایاس ژیروسکوپ
        
        # ۳. سابسکرایبر (ورودی IMU خام)
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        
        self.imu_pub = self.create_publisher(Imu, 'imu/filtered', 10)
        
        self.get_logger().info('Lowpass IMU Filter Node started.')

    def imu_callback(self, msg: Imu):
        self.filtered_accel[0] = self.alpha * msg.linear_acceleration.x + (1 - self.alpha) * self.filtered_accel[0]
        self.filtered_accel[1] = self.alpha * msg.linear_acceleration.y + (1 - self.alpha) * self.filtered_accel[1]
        self.filtered_accel[2] = self.alpha * msg.linear_acceleration.z + (1 - self.alpha) * self.filtered_accel[2]

        self.filtered_gyro[0] = self.alpha * msg.angular_velocity.x + (1 - self.alpha) * self.filtered_gyro[0]
        self.filtered_gyro[1] = self.alpha * msg.angular_velocity.y + (1 - self.alpha) * self.filtered_gyro[1]
        self.filtered_gyro[2] = self.alpha * msg.angular_velocity.z + (1 - self.alpha) * self.filtered_gyro[2]

        filtered_msg = msg
        filtered_msg.linear_acceleration.x = self.filtered_accel[0]
        filtered_msg.linear_acceleration.y = self.filtered_accel[1]
        filtered_msg.linear_acceleration.z = self.filtered_accel[2]
        
        filtered_msg.angular_velocity.x = self.filtered_gyro[0]
        filtered_msg.angular_velocity.y = self.filtered_gyro[1]
        filtered_msg.angular_velocity.z = self.filtered_gyro[2]

        self.imu_pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LowpassImuFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()