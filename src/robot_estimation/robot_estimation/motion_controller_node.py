import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller_node')
        
        # ۱. پارامترهای ربات (شعاع چرخ و فاصله بین چرخ‌ها)
        self.declare_parameter('wheel_radius', 0.1) 
        self.declare_parameter('wheel_separation', 0.7) 
        self.declare_parameter('max_rpm', 9.55) # RPM 

        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.MAX_RPM = self.get_parameter('max_rpm').value
        
        # ۲. پابلیشرها برای RPM
        self.left_rpm_pub = self.create_publisher(Float64, 'left_wheel_rpm', 10)
        self.right_rpm_pub = self.create_publisher(Float64, 'right_wheel_rpm', 10)
        
        # ۳. سابسکرایبر برای فرمان سرعت (cmd_vel)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Motion Controller Node has started and is listening to /cmd_vel.')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x    # سرعت خطی (متر بر ثانیه)
        omega = msg.angular.z # سرعت زاویه‌ای (رادیان بر ثانیه)
        
        # سینماتیک معکوس: تبدیل سرعت خطی/زاویه‌ای به سرعت چرخ‌ها (V_L, V_R)
        # V_R = v + omega * L/2
        # V_L = v - omega * L/2
        
        v_r = v + (omega * self.L) / 2.0
        v_l = v - (omega * self.L) / 2.0
        
        # تبدیل سرعت خطی (متر بر ثانیه) به RPM
        # RPM = V / (2 * pi * R) * 60
        
        right_rpm = (v_r / (2.0 * 3.14159 * self.R)) * 60.0
        left_rpm = (v_l / (2.0 * 3.14159 * self.R)) * 60.0
        
        # محدود کردن RPM به یک مقدار حداکثر (مثلاً برای شبیه‌سازی محدودیت موتور)
        right_rpm = max(min(right_rpm, self.MAX_RPM), -self.MAX_RPM)
        left_rpm = max(min(left_rpm, self.MAX_RPM), -self.MAX_RPM)
        
        # انتشار فرمان RPM
        self.right_rpm_pub.publish(Float64(data=right_rpm))
        self.left_rpm_pub.publish(Float64(data=left_rpm))

def main(args=None):
    rclpy.init(args=args)
    motion_controller = MotionController()
    rclpy.spin(motion_controller)
    motion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()