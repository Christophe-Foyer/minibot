from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Command velocity topic'
        ),
        DeclareLaunchArgument(
            'wheel_base',
            default_value='0.160',
            description='Distance between wheels in meters'
        ),
        DeclareLaunchArgument(
            'wheel_diameter', 
            default_value='0.210',
            description='Wheel diameter in meters'
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='1.0',
            description='Maximum linear speed in m/s'
        ),

        Node(
            package='waveshare_stepper',
            executable='waveshare_stepper_node',
            name='waveshare_stepper_node',
            parameters=[{
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'wheel_base': LaunchConfiguration('wheel_base'),
                'wheel_diameter': LaunchConfiguration('wheel_diameter'),
                'max_speed': LaunchConfiguration('max_speed'),
                'steps_per_revolution': 200,
                'acceleration': 0.5,
                'step_delay': 0.001,
            }],
            output='screen',
        )
    ])
