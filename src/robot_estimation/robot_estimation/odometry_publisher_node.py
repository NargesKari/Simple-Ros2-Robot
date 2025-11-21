import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher_node')

        # ۱. پارامترهای ربات
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_separation', 0.7)

        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        
        # ۲. متغیرهای وضعیت (Position, Orientation)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # ۳. متغیرهای ورودی
        self.last_time = self.get_clock().now()
        self.v_l = 0.0 # سرعت خطی چرخ چپ (متر بر ثانیه)
        self.v_r = 0.0 # سرعت خطی چرخ راست (متر بر ثانیه)
        
        # ۴. پابلیشرها و برودکستر
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ۵. سابسکرایبرهای RPM چرخ
        # توجه: ما فرض می‌کنیم که این تاپیک‌ها سرعت خطی (متر بر ثانیه) را منتشر می‌کنند،
        # یا باید آن‌ها را از RPM دریافتی از کنترلر تبدیل کنیم.
        self.left_sub = self.create_subscription(Float64, 'left_wheel_rpm', self.left_rpm_callback, 10)
        self.right_sub = self.create_subscription(Float64, 'right_wheel_rpm', self.right_rpm_callback, 10)
        
        # ۶. تایمر برای به‌روزرسانی (مثلاً ۲۰ هرتز)
        self.timer = self.create_timer(0.05, self.update_odometry) # ۲۰Hz = 0.05s

    def rpm_to_vel(self, rpm):
        # تبدیل RPM به سرعت خطی (متر بر ثانیه)
        # V = RPM * (2 * pi * R) / 60
        return rpm * (2.0 * math.pi * self.R) / 60.0

    def left_rpm_callback(self, msg):
        self.v_l = self.rpm_to_vel(msg.data)

    def right_rpm_callback(self, msg):
        self.v_r = self.rpm_to_vel(msg.data)

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0.0:
            return

        # سرعت خطی و زاویه‌ای ربات (سینماتیک مستقیم)
        v = (self.v_r + self.v_l) / 2.0
        omega = (self.v_r - self.v_l) / self.L
        
        # محاسبه تغییرات موقعیت
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt
        
        # به‌روزرسانی موقعیت (انتگرال گیری)
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta)) # نرمالایز به [-pi, pi]
        
        # --- انتشار TF (broadcast odom -> base_link) ---
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = self.euler_to_quaternion(0, 0, self.theta) # تبدیل Yaw به کواترنیون
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # --- انتشار پیام Odometry ---
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation # استفاده از کواترنیون TF
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom)

        self.last_time = current_time

    def euler_to_quaternion(self, roll, pitch, yaw):
        # تبدیل اویلر به کواترنیون
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    odom_pub = OdometryPublisher()
    rclpy.spin(odom_pub)
    odom_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()