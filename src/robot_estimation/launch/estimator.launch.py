from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.1', description='Radius of the wheels in meters.'
    )
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.7', description='Distance between wheels in meters.'
    )
    
    lowpass_alpha_arg = DeclareLaunchArgument(
        'lowpass_alpha', default_value='0.8', description='Alpha value for the Lowpass Filter.'
    )
    complementary_alpha_arg = DeclareLaunchArgument(
        'complementary_alpha', default_value='0.98', description='Alpha value for the Complementary Filter.'
    )
    
    
    lowpass_filter_node = Node(
        package='robot_estimation',
        executable='lowpass_imu_node', 
        name='lowpass_filter',
        output='screen',
        parameters=[
            {'lowpass_alpha': LaunchConfiguration('lowpass_alpha')},
        ],
        remappings=[
            ('/imu_data', '/robot/imu_raw'), 
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

    motion_controller_node = Node(
        package='robot_estimation',
        executable='motion_controller_node',
        name='motion_controller',
        output='screen',
        parameters=[
            {'wheel_radius': LaunchConfiguration('wheel_radius')},
            {'wheel_separation': LaunchConfiguration('wheel_separation')},
        ],
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
    )
    
    
    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        lowpass_alpha_arg,
        complementary_alpha_arg,
        
        lowpass_filter_node,
        complementary_filter_node,
        motion_controller_node,
        odometry_publisher_node,
    ])