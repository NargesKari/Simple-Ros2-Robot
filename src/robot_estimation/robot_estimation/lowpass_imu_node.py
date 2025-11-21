import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np 


class LowpassImuFilter(Node):
    def __init__(self):
        super().__init__('lowpass_imu_filter_node')
        
        self.declare_parameter('alpha_accel', 0.1)
        self.declare_parameter('alpha_gyro', 0.1)
        self.declare_parameter('bias_samples', 100) 
        
        self.alpha_accel = self.get_parameter('alpha_accel').value
        self.alpha_gyro = self.get_parameter('alpha_gyro').value
        self.bias_samples = self.get_parameter('bias_samples').value
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu_data_raw', 
            self.imu_callback,
            10
        )
        
        self.imu_pub = self.create_publisher(
            Imu,
            'imu/filtered', 
            10
        )
        
        self.accel_filtered = np.array([0.0, 0.0, 0.0])
        self.gyro_filtered = np.array([0.0, 0.0, 0.0])
        
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.accel_bias = np.array([0.0, 0.0, 0.0]) 
        self.bias_samples_collected = 0
        self.bias_sum_gyro = np.array([0.0, 0.0, 0.0])
        self.bias_sum_accel = np.array([0.0, 0.0, 0.0])
        self.bias_calibrated = False
        
        self.get_logger().info('IMU Filter node started. Calibrating bias...')

    def imu_callback(self, msg: Imu):
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        
        if not self.bias_calibrated:
            if self.bias_samples_collected < self.bias_samples:
                self.bias_sum_gyro += gyro
                self.bias_sum_accel += accel
                self.bias_samples_collected += 1
                return
            else:
                self.gyro_bias = self.bias_sum_gyro / self.bias_samples
                self.accel_bias = self.bias_sum_accel / self.bias_samples
                
                self.accel_bias[2] -= 9.81 
                
                self.bias_calibrated = True
                self.get_logger().info(f'Bias calibration complete! Gyro bias: {self.gyro_bias}')
                self.get_logger().info(f'Accel bias (Excluding Gravity): {self.accel_bias}')
        
        accel_corrected = accel - self.accel_bias
        gyro_corrected = gyro - self.gyro_bias
        
        self.accel_filtered = (self.alpha_accel * accel_corrected + 
                              (1 - self.alpha_accel) * self.accel_filtered)
        
        self.gyro_filtered = (self.alpha_gyro * gyro_corrected + 
                             (1 - self.alpha_gyro) * self.gyro_filtered)
        
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        
        filtered_msg.linear_acceleration.x = self.accel_filtered[0]
        filtered_msg.linear_acceleration.y = self.accel_filtered[1]
        filtered_msg.linear_acceleration.z = self.accel_filtered[2]
        
        filtered_msg.angular_velocity.x = self.gyro_filtered[0]
        filtered_msg.angular_velocity.y = self.gyro_filtered[1]
        filtered_msg.angular_velocity.z = self.gyro_filtered[2]
        
        
        filtered_msg.orientation = msg.orientation
        filtered_msg.orientation_covariance = msg.orientation_covariance
        filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        self.imu_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LowpassImuFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()