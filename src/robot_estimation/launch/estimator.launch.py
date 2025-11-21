from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.1'
    )
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.7'
    )
    
    lowpass_alpha_accel_arg = DeclareLaunchArgument(
        'alpha_accel', default_value='0.1'
    )
    lowpass_alpha_gyro_arg = DeclareLaunchArgument(
        'alpha_gyro', default_value='0.1'
    )
    complementary_alpha_arg = DeclareLaunchArgument(
        'complementary_alpha', default_value='0.98'
    )
    
    
    # IMU Filtering Nodes
    lowpass_filter_node = Node(
        package='robot_estimation',
        executable='lowpass_imu_node',
        name='lowpass_filter',
        output='screen',
        parameters=[
            {'alpha_accel': LaunchConfiguration('alpha_accel')},
            {'alpha_gyro': LaunchConfiguration('alpha_gyro')},
        ],
        remappings=[
            ('/imu_data_raw', '/zed/zed_node/imu/data_raw'), 
            ('imu/filtered', '/imu/filtered'), 
        ]
    )
    
    complementary_filter_node = Node(
        package='robot_estimation',
        executable='complementary_filter_node', 
        name='orientation_estimator',
        output='screen',
        parameters=[
            {'complementary_alpha': LaunchConfiguration('complementary_alpha')},
        ],
        remappings=[
            ('imu/filtered', '/imu/filtered'), 
            ('estimation/orientation', '/estimation/orientation'), 
        ]
    )

    # Motion Control Node
    motion_controller_node = Node(
        package='robot_estimation',
        executable='motion_controller_node',
        name='motion_controller',
        output='screen',
        parameters=[
            {'wheel_radius': LaunchConfiguration('wheel_radius')},
            {'wheel_separation': LaunchConfiguration('wheel_separation')},
        ],
        remappings=[
            ('left_wheel_rpm', '/left_motor_rpm'),
            ('right_wheel_rpm', '/right_motor_rpm'),
        ]
    )

    # Odometry Nodes (using actual RPM from JointState)
    wheel_rpm_publisher_node = Node(
        package='robot_estimation',
        executable='wheel_rpm_publisher', 
        name='wheel_rpm_publisher',
        output='screen',
        parameters=[
            {'wheel_radius': LaunchConfiguration('wheel_radius')},
        ],
        remappings=[
            ('left_rpm', '/actual_left_rpm'),
            ('right_rpm', '/actual_right_rpm'),
        ]
    )
    
    odometry_publisher_node = Node(
        package='robot_estimation',
        executable='odometry_publisher_node',
        name='odometry_publisher',
        output='screen',
        parameters=[
            {'wheel_radius': LaunchConfiguration('wheel_radius')},
            {'wheel_separation': LaunchConfiguration('wheel_separation')},
        ],
        remappings=[
            ('left_wheel_rpm', '/actual_left_rpm'),
            ('right_wheel_rpm', '/actual_right_rpm'),
            ('odom', '/odom'),
        ]
    )
    
    
    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        lowpass_alpha_accel_arg,
        lowpass_alpha_gyro_arg,
        complementary_alpha_arg,
        
        lowpass_filter_node,
        complementary_filter_node,
        
        motion_controller_node,
        
        wheel_rpm_publisher_node,
        odometry_publisher_node,
    ])